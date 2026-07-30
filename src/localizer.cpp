/**
 * @file localizer.cpp
 * @brief 定位管理器的实现
 *
 * 逐方法对应原 Python localization.py 中的 Localizer 类
 */
#include "prism_topomap/localizer.h"
#include <ros/ros.h>
#include <cmath>
#include <algorithm>
#include <sys/stat.h>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <limits>

namespace prism_topomap {

static void mkdirIfNotExists(const std::string& path) {
    struct stat st;
    if (stat(path.c_str(), &st) != 0) {
#ifdef _WIN32
        mkdir(path.c_str());
#else
        mkdir(path.c_str(), 0755);
#endif
    }
}

// ============================================================================
// 构造函数
// ============================================================================
Localizer::Localizer(TopologicalGraph& graph,
                     std::shared_ptr<InferenceClient> inference_client,
                     double registration_score_threshold,
                     int top_k,
                     const std::string& save_dir,
                     const FlowTraceConfig& trace_config)
    : graph_(graph),
      inference_client_(inference_client),
      reg_score_threshold_(registration_score_threshold),
      top_k_(top_k),
      save_dir_(save_dir),
      trace_config_(trace_config) {
    if (!save_dir_.empty()) {
        mkdirIfNotExists(save_dir_);
    }
}

// ============================================================================
// updateCurrentState: 线程安全地更新当前观测
// 对应 Python:
//   def update_current_state(self, global_pose, cur_desc, cur_grid, timestamp):
//       self.mutex.acquire()
//       self.global_pose = global_pose
//       self.descriptor = cur_desc
//       self.grid = cur_grid
//       self.stamp = timestamp
//       self.mutex.release()
// ============================================================================
void Localizer::updateCurrentState(const Pose2D& global_pose,
                                   const std::vector<float>& descriptor,
                                   const LocalGrid& grid,
                                   double timestamp,
                                   int frame_id,
                                   bool trace_detailed) {
    std::lock_guard<std::mutex> lock(mutex_);
    global_pose_ = global_pose;
    descriptor_ = descriptor;
    grid_ = grid.copy();
    stamp_ = timestamp;
    frame_id_ = frame_id;
    trace_detailed_ = trace_detailed;
    initialized_ = true;

    if (trace_config_.enabled && trace_detailed_) {
        const cv::Mat& occ = grid_.getLayer("occupancy");
        ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=LOCALIZER_SNAPSHOT] "
                 "action=WRITE descriptor_dim=%lu occupancy=%dx%d occupancy_nonzero=%d "
                 "graph_vertices=%d",
                 frame_id_, stamp_, descriptor_.size(), occ.rows, occ.cols,
                 cv::countNonZero(occ), graph_.numVertices());
    }
}

// ============================================================================
// getCurrentState: 获取当前快照 (内部使用)
// 对应 Python: Localizer.get_current_state()
// ============================================================================
Localizer::Snapshot Localizer::getCurrentState() {
    std::lock_guard<std::mutex> lock(mutex_);
    Snapshot snap;
    snap.global_pose = global_pose_;
    snap.descriptor = descriptor_;
    snap.grid = grid_.copy();
    snap.timestamp = stamp_;
    snap.frame_id = frame_id_;
    snap.trace_detailed = trace_detailed_;
    return snap;
}

// ============================================================================
// getLocalizedState: 获取定位结果 (线程安全)
// 对应 Python: Localizer.get_localized_state()
// ============================================================================
LocalizedState Localizer::getLocalizedState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return localized_state_;
}

// ============================================================================
// writeLocalizedState: 写入定位结果 (内部使用)
// 对应 Python: Localizer.write_localized_state()
// ============================================================================
void Localizer::writeLocalizedState(const std::vector<int>& matched_ids,
                                    const std::vector<Pose2D>& rel_poses,
                                    const std::vector<int>& unmatched_ids,
                                    const Pose2D& global_pose,
                                    double stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    localized_state_.vertex_ids_matched = matched_ids;
    localized_state_.rel_poses = rel_poses;
    localized_state_.vertex_ids_unmatched = unmatched_ids;
    localized_state_.global_pose = global_pose;
    localized_state_.timestamp = stamp;
    localized_stamp = stamp;
}

// ============================================================================
// localize: 执行一次定位
// 对应 Python: Localizer.localize()
//
// 流程:
// 1. 获取当前快照
// 2. FAISS 检索 top-k 候选
// 3. 对每个候选做栅格配准 (通过 Python Service)
// 4. 过滤低分数匹配
// 5. 提取位姿变换
// 6. 写入定位结果
// ============================================================================
void Localizer::localize() {
    if (!initialized_) {
        ROS_INFO("Waiting for messages to initialize localizer...");
        if (trace_config_.enabled) {
            ROS_INFO("[FLOW][STAGE=LOCALIZER_SNAPSHOT] action=TIMER_SKIP reason=NOT_INITIALIZED");
        }
        return;
    }

    ROS_INFO("Starting localization from stamp %.3f", stamp_);
    const auto localize_start = std::chrono::steady_clock::now();

    // 1. 获取当前快照
    Snapshot snap = getCurrentState();
    Pose2D start_global_pose = snap.global_pose;
    double start_stamp = snap.timestamp;
    LocalGrid start_grid = snap.grid;
    std::vector<float> start_desc = snap.descriptor;

    if (trace_config_.enabled && snap.trace_detailed) {
        ROS_INFO("[FLOW][FRAME=%d][LOC_STAMP=%.6f][STAGE=LOCALIZER_SNAPSHOT] "
                 "action=TIMER_READ wall_time=%.6f descriptor_dim=%lu occupancy=%dx%d "
                 "occupancy_nonzero=%d graph_vertices=%d",
                 snap.frame_id, start_stamp, ros::WallTime::now().toSec(),
                 start_desc.size(), start_grid.gridSize(), start_grid.gridSize(),
                 cv::countNonZero(start_grid.getLayer("occupancy")), graph_.numVertices());
    }

    if (start_desc.empty()) {
        ROS_WARN("Localizer: descriptor is empty, skipping");
        if (trace_config_.enabled) {
            ROS_WARN("[FLOW][FRAME=%d][LOC_STAMP=%.6f][STAGE=FAISS] "
                     "result=SKIPPED_EMPTY_DESCRIPTOR",
                     snap.frame_id, start_stamp);
        }
        return;
    }

    // 2. FAISS 检索 top-k
    const auto faiss_start = std::chrono::steady_clock::now();
    auto [dists, pred_i] = graph_.searchIndex(start_desc, top_k_);
    const double faiss_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - faiss_start).count();

    if (pred_i.empty()) {
        ROS_INFO("FAISS index is empty, cannot localize");
        n_loc_fails++;
        if (trace_config_.enabled) {
            ROS_INFO("[FLOW][FRAME=%d][LOC_STAMP=%.6f][STAGE=FAISS] "
                     "query_dim=%lu index_size=%d top_k=%d elapsed_ms=%.3f "
                     "result=EMPTY_INDEX",
                     snap.frame_id, start_stamp, start_desc.size(),
                     graph_.indexSize(), top_k_, faiss_ms);
        }
        return;
    }

    if (trace_config_.enabled && snap.trace_detailed) {
        std::ostringstream candidates;
        candidates << std::fixed << std::setprecision(6) << "[";
        for (size_t i = 0; i < pred_i.size(); ++i) {
            if (i > 0) candidates << ",";
            candidates << pred_i[i] << ":" << dists[i];
        }
        candidates << "]";
        ROS_INFO("[FLOW][FRAME=%d][LOC_STAMP=%.6f][STAGE=FAISS] "
                 "query_dim=%lu index_size=%d top_k=%d elapsed_ms=%.3f candidates=%s",
                 snap.frame_id, start_stamp, start_desc.size(),
                 graph_.indexSize(), top_k_, faiss_ms, candidates.str().c_str());
    }

    // 3. 对每个候选做配准
    std::vector<int> pred_i_filtered;
    // 存储配准结果: [rot_x, rot_y, rot_z, trans_x, trans_y, trans_z]
    std::vector<std::vector<double>> pred_tf;
    std::vector<double> reg_scores;

    for (int i = 0; i < static_cast<int>(pred_i.size()); ++i) {
        int idx = pred_i[i];
        if (idx < 0) continue;

        const Vertex& cand_vertex = graph_.getVertex(idx);
        LocalGrid cand_grid = cand_vertex.grid.copy();//当前观测栅格
        LocalGrid grid_copy = start_grid.copy();//候选节点历史栅格

        // 调用 Python 配准服务 (localization 类型)
        auto reg_result = inference_client_->gridRegistration(
            grid_copy.getLayer("occupancy"),
            cand_grid.getLayer("occupancy"),
            "localization"
        );

        reg_scores.push_back(reg_result.score);
        ROS_INFO("Vertex %d registration score: %.3f", idx, reg_result.score);

        if (!reg_result.success || reg_result.score < reg_score_threshold_) {
            //认为候选节点虽然描述符相似，但局部几何结构不匹配。
            pred_i_filtered.push_back(-1);
            pred_tf.push_back({0, 0, 0, 0, 0, 0});
        } else {
            // 计算位姿变换矩阵
            Eigen::Matrix4d tf_matrix = cand_grid.getTfMatrixXY(
                reg_result.trans_i, reg_result.trans_j, reg_result.rot_angle);
                //栅格像素平移，二维旋转
            pred_i_filtered.push_back(idx);

            // 提取旋转向量和平移向量
            Eigen::Matrix3d rot = tf_matrix.block<3, 3>(0, 0);
            // 简化: 只取绕 z 轴的角度
            double theta = std::atan2(rot(1, 0), rot(0, 0));
            double tx = tf_matrix(0, 3);
            double ty = tf_matrix(1, 3);
            double tz = tf_matrix(2, 3);
            // 存储为 [rot_x=0, rot_y=0, rot_z=theta, trans_x, trans_y, trans_z=0]
            pred_tf.push_back({0.0, 0.0, theta, tx, ty, tz});
        }

        if (trace_config_.enabled &&
            ((snap.trace_detailed && trace_config_.registration_candidates) ||
             !reg_result.success)) {
            const bool matched = reg_result.success &&
                                 reg_result.score >= reg_score_threshold_;
            double metric_x = std::numeric_limits<double>::quiet_NaN();
            double metric_y = std::numeric_limits<double>::quiet_NaN();
            double metric_theta = std::numeric_limits<double>::quiet_NaN();
            if (matched) {
                const auto& tf = pred_tf.back();
                metric_x = tf[3];
                metric_y = tf[4];
                metric_theta = tf[2];
            }
            ROS_INFO("[FLOW][FRAME=%d][LOC_STAMP=%.6f][STAGE=REGISTRATION] "
                     "type=localization candidate=%d ref=%dx%d ref_nonzero=%d "
                     "cand=%dx%d cand_nonzero=%d service_success=%s score=%.6f "
                     "threshold=%.6f pixel_tf=(%.3f,%.3f,%.6f) "
                     "metric_pose=(%.4f,%.4f,%.6f) result=%s elapsed_ms=%.3f",
                     snap.frame_id, start_stamp, idx,
                     grid_copy.gridSize(), grid_copy.gridSize(),
                     cv::countNonZero(grid_copy.getLayer("occupancy")),
                     cand_grid.gridSize(), cand_grid.gridSize(),
                     cv::countNonZero(cand_grid.getLayer("occupancy")),
                     reg_result.success ? "true" : "false", reg_result.score,
                     reg_score_threshold_, reg_result.trans_i, reg_result.trans_j,
                     reg_result.rot_angle, metric_x, metric_y, metric_theta,
                     matched ? "MATCHED" : "UNMATCHED", reg_result.elapsed_ms);
        }
    }

    // 4. 过滤结果
    std::vector<int> vertex_ids_matched;//配准后候选结果id
    std::vector<Pose2D> rel_poses;//配准成功节点相对位姿
    std::vector<int> vertex_ids_unmatched;//被检索到但配准失败的节点ID

    // 未匹配的顶点
    for (int i = 0; i < static_cast<int>(pred_i.size()); ++i) {
        int idx = pred_i[i];
        if (idx < 0) continue;
        bool found_in_filtered = false;
        for (int fi : pred_i_filtered) {
            if (fi == idx) { found_in_filtered = true; break; }
        }
        if (!found_in_filtered) {
            vertex_ids_unmatched.push_back(idx);
        }
    }

    // 已匹配的顶点
    if (pred_i_filtered.empty()) {
        n_loc_fails++;
    }

    for (int i = 0; i < static_cast<int>(pred_i_filtered.size()); ++i) {
        int idx = pred_i_filtered[i];
        if (idx < 0) continue;

        vertex_ids_matched.push_back(idx);
        // rel_pose = [trans_x, trans_y, rot_z]
        // 对应 Python: rel_poses.append(transforms[i, [3, 4, 2]])
        rel_poses.push_back(Pose2D(pred_tf[i][3], pred_tf[i][4], pred_tf[i][2]));
    }

    // 5. 写入定位结果
    writeLocalizedState(vertex_ids_matched, rel_poses,
                        vertex_ids_unmatched, start_global_pose, start_stamp);

    cnt++;

    if (trace_config_.enabled && snap.trace_detailed) {
        const double total_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - localize_start).count();
        ROS_INFO("[FLOW][FRAME=%d][LOC_STAMP=%.6f][STAGE=LOCALIZATION_RESULT] "
                 "matched=%lu unmatched=%lu result_stamp=%.6f elapsed_ms=%.3f",
                 snap.frame_id, start_stamp, vertex_ids_matched.size(),
                 vertex_ids_unmatched.size(), start_stamp, total_ms);
    }
}

} // namespace prism_topomap
