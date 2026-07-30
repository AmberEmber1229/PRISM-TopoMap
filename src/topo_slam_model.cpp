/**
 * @file topo_slam_model.cpp
 * @brief 核心拓扑 SLAM 算法实现
 *
 * 逐方法对应原 Python prism_topomap.py 中的 TopoSLAMModel 类
 * 这是整个系统中最核心的文件, 包含主更新循环的全部逻辑
 */
#include "prism_topomap/topo_slam_model.h"
#include <ros/ros.h>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <limits>
#include <chrono>
#include <iterator>
#include <unordered_set>

namespace prism_topomap {

// ============================================================================
// initParamsFromConfig: 从 YAML 配置加载参数
// 对应 Python: TopoSLAMModel.__init__() 中的 self.xxx = config['xxx']
// ============================================================================
void TopoSLAMModel::initParamsFromConfig(const YAML::Node& config) {
    const YAML::Node topomap_config = config["topomap"] ? config["topomap"] : config;
    const YAML::Node input_config = config["input"];
    const YAML::Node pointcloud_config = input_config["pointcloud"];
    const YAML::Node pr_config = config["place_recognition"] ? config["place_recognition"] : config;
    const YAML::Node grid_config = config["local_occupancy_grid"] ? config["local_occupancy_grid"] : config;
    const YAML::Node reg_config = config["scan_matching"] ? config["scan_matching"] : config;
    const YAML::Node inline_reg_config = config["scan_matching_along_edge"] ? config["scan_matching_along_edge"] : config;
    const YAML::Node viz_config = config["visualization"] ? config["visualization"] : config;

    mode_ = topomap_config["mode"].as<std::string>("mapping");
    iou_threshold_ = topomap_config["iou_threshold"].as<double>(0.8);
    localization_frequency_ = topomap_config["localization_frequency"].as<double>(0.5);
    rel_pose_correction_frequency_ = topomap_config["rel_pose_correction_frequency"].as<double>(0.0);
    max_edge_length_ = topomap_config["max_edge_length"].as<double>(10.0);
    loop_edge_max_abs_distance_error_ =
        topomap_config["loop_edge_max_abs_distance_error"].as<double>(1e9);
    loop_edge_max_distance_ratio_ =
        topomap_config["loop_edge_max_distance_ratio"].as<double>(1e9);
    loop_edge_ratio_min_distance_ =
        topomap_config["loop_edge_ratio_min_distance"].as<double>(0.5);
    loop_edge_max_yaw_error_ =
        topomap_config["loop_edge_max_yaw_error"].as<double>(M_PI);
    drift_coef_ = topomap_config["drift_coef"].as<double>(0.02);
    localization_timeout_ = topomap_config["localization_timeout"].as<double>(10.0);

    floor_height_ = pointcloud_config["floor_height"].as<double>(0.0);
    ceil_height_ = pointcloud_config["ceiling_height"].as<double>(1.0);
    pointcloud_quantization_size_ = pr_config["pointcloud_quantization_size"].as<float>(0.5);

    grid_resolution_ = grid_config["resolution"].as<double>(0.1);
    grid_radius_ = grid_config["radius"].as<double>(18.0);
    max_grid_range_ = grid_config["max_range"].as<double>(8.0);

    reg_score_threshold_ = reg_config["score_threshold"].as<double>(0.6);
    inline_reg_score_threshold_ = inline_reg_config["score_threshold"].as<double>(0.5);

    local_jump_threshold_ = inline_reg_config["jump_threshold"].as<double>(3.0);
    map_frame_ = viz_config["map_frame"].as<std::string>("map");
    top_k_ = pr_config["top_k"].as<int>(5);

    if (topomap_config["start_location"]) {
        start_location_ = topomap_config["start_location"].as<int>(-1);
    }
    if (topomap_config["start_local_pose"]) {
        auto slp = topomap_config["start_local_pose"];
        start_local_pose_ = Pose2D(slp[0].as<double>(), slp[1].as<double>(), slp[2].as<double>());
        has_start_local_pose_ = true;
    }
}

// ============================================================================
// 构造函数
// ============================================================================
TopoSLAMModel::TopoSLAMModel(const YAML::Node& config,
                               std::shared_ptr<InferenceClient> inference_client,
                               const std::string& path_to_load_graph,
                               const std::string& path_to_save_graph,
                               const std::string& path_to_save_logs,
                               const FlowTraceConfig& trace_config)
    : inference_client_(inference_client),
      graph_(inference_client,
             config["scan_matching_along_edge"]["score_threshold"].as<double>(
                 config["inline_registration_score_threshold"].as<double>(0.5)),
             config["local_occupancy_grid"]["resolution"].as<double>(
                 config["grid_resolution"].as<double>(0.1)),
             config["local_occupancy_grid"]["radius"].as<double>(
                 config["grid_radius"].as<double>(18.0)),
             config["local_occupancy_grid"]["max_range"].as<double>(
                 config["max_grid_range"].as<double>(8.0)),
             config["place_recognition"]["descriptor_length"].as<int>(
                 config["descriptor_length"].as<int>(256))),
      localizer_(graph_, inference_client,
                 config["scan_matching"]["score_threshold"].as<double>(
                     config["registration_score_threshold"].as<double>(0.6)),
                 config["place_recognition"]["top_k"].as<int>(
                     config["top_k"].as<int>(5)),
                 path_to_save_logs,
                 trace_config),
      cur_grid_(config["local_occupancy_grid"]["resolution"].as<double>(
                    config["grid_resolution"].as<double>(0.1)),
                config["local_occupancy_grid"]["radius"].as<double>(
                    config["grid_radius"].as<double>(18.0)),
                config["local_occupancy_grid"]["max_range"].as<double>(
                    config["max_grid_range"].as<double>(8.0)),
                config["input"]["pointcloud"]["floor_height"].as<double>(
                    config["floor_height"].as<double>(0.0)),
                config["input"]["pointcloud"]["ceiling_height"].as<double>(
                    config["ceil_height"].as<double>(1.0))),
      path_to_load_graph_(path_to_load_graph),
      path_to_save_graph_(path_to_save_graph),
      path_to_save_logs_(path_to_save_logs),
      trace_config_(trace_config) {

    initParamsFromConfig(config);
    inference_client_->setTraceConfig(trace_config_);
    ROS_INFO("Loop edge validation: max_abs_distance_error=%.3f m, "
             "max_distance_ratio=%.3f (active above %.3f m), "
             "max_yaw_error=%.3f rad",
             loop_edge_max_abs_distance_error_,
             loop_edge_max_distance_ratio_,
             loop_edge_ratio_min_distance_,
             loop_edge_max_yaw_error_);

    // 如果有预加载图
    if (!path_to_load_graph_.empty()) {
        graph_.loadFromJson(path_to_load_graph_);
        ROS_INFO("Loaded graph from %s (vertices: %d)", path_to_load_graph_.c_str(), graph_.numVertices());
    }
}

// ============================================================================
// updateRelPoseByOdom: 里程计积分
// 对应 Python: TopoSLAMModel.update_rel_pose_of_vcur_by_odom()
// ============================================================================
void TopoSLAMModel::updateRelPoseByOdom(const Pose2D& cur_odom_pose) {
    if (!odom_initialized_) {
        odom_pose_ = cur_odom_pose;
        odom_initialized_ = true;
        // ROS_DEBUG("[ODOM] First odom received: (%.4f, %.4f, %.4f) — initializing odom_pose_",
        //          cur_odom_pose[0], cur_odom_pose[1], cur_odom_pose[2]);
        return;
    }
    Pose2D rel_odom = getRelPose(odom_pose_, cur_odom_pose);
    // Pose2D old_rel_pose = rel_pose_of_vcur_;
    rel_pose_of_vcur_ = applyPoseShift(rel_pose_of_vcur_, rel_odom);
    // ROS_DEBUG("[ODOM] delta=(%.4f,%.4f,%.4f) odom_old=(%.4f,%.4f,%.4f) odom_new=(%.4f,%.4f,%.4f) "
    //          "rel_pose: (%.4f,%.4f,%.4f) -> (%.4f,%.4f,%.4f)",
    //          rel_odom[0], rel_odom[1], rel_odom[2],
    //          odom_pose_[0], odom_pose_[1], odom_pose_[2],
    //          cur_odom_pose[0], cur_odom_pose[1], cur_odom_pose[2],
    //          old_rel_pose[0], old_rel_pose[1], old_rel_pose[2],
    //          rel_pose_of_vcur_[0], rel_pose_of_vcur_[1], rel_pose_of_vcur_[2]);
    odom_pose_ = cur_odom_pose;
}

// ============================================================================
// processObservations: 观测处理 (描述符提取 + 栅格更新)
// 对应 Python: TopoSLAMModel.process_observations()
// ============================================================================
void TopoSLAMModel::processObservations(
    const sensor_msgs::PointCloud2& cloud_msg,
    const PointCloudPtr& cur_cloud,
    bool has_image_front, bool has_image_back,
    const sensor_msgs::Image& img_front,
    const sensor_msgs::Image& img_back,
    const PointCloudPtr& cur_curbs,
    double x, double y, double theta) {

    // 1. 调用 Python 推理服务提取描述符，相似地点的描述符距离较小
    //后续定位器会用这个描述符从所有拓扑节点中找出最相似的 top_k 个节点
    auto desc_result = inference_client_->getDescriptor(
        cloud_msg,
        has_image_front, has_image_back,
        img_front, img_back,
        pointcloud_quantization_size_);

    if (desc_result.success) {
        cur_desc_ = desc_result.descriptor;
        // 确保描述符是正确维度
        // ROS_DEBUG("描述符提取成功, 维度=%lu", cur_desc_.size());
    } else {
        cur_desc_.clear();
        ROS_WARN("Descriptor extraction failed!");
    }

    // 2. 点云投影到栅格 (更新当前局部栅格)
    // 注意: Python 中传入的 theta 取反 (-theta)
    const auto grid_flow_start = std::chrono::steady_clock::now();
    LocalGridUpdateStats grid_stats;
    cur_grid_.updateFromCloudAndTransform(cur_cloud, x, y, -theta,
                                          trace_config_.enabled ? &grid_stats : nullptr);
    //此处的 cur_grid_ 不是某个拓扑节点已经存储的栅格，而是当前机器人正在维护的观测栅格。
    // 3. Curb update
    if (cur_curbs && !cur_curbs->empty()) {
        cur_grid_.updateCurbsFromCloud(cur_curbs);
        grid_stats.curbs_updated = grid_stats.curbs_layer_exists;
    }

    if (trace_config_.enabled && trace_detailed_) {
        const double grid_total_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - grid_flow_start).count();
        ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=GRID] "
                 "input_points=%lu finite_points=%lu in_range=%lu out_of_range=%lu "
                 "obstacle_points=%lu resolution=%.3f radius=%.3f max_range=%.3f "
                 "grid=%dx%d grid_shift=(%.4f,%.4f,%.6f) "
                 "occupancy_unknown=%d occupancy_free=%d occupancy_obstacle=%d "
                 "density_nonzero=%d density_max=%.3f height_nonzero=%d height_max=%.3f "
                 "curbs_input=%s curbs_layer=%s curbs_updated=%s "
                 "occupancy_update_ms=%.3f elapsed_ms=%.3f",
                 trace_frame_id_, current_stamp_,
                 grid_stats.input_points, grid_stats.finite_points,
                 grid_stats.in_range_points, grid_stats.out_of_range_points,
                 grid_stats.obstacle_points, cur_grid_.resolution(),
                 cur_grid_.radius(), cur_grid_.maxRange(),
                 cur_grid_.gridSize(), cur_grid_.gridSize(),
                 x, y, -theta, grid_stats.unknown_cells,
                 grid_stats.free_cells, grid_stats.occupied_cells,
                 grid_stats.density_nonzero_cells, grid_stats.density_max,
                 grid_stats.height_nonzero_cells, grid_stats.height_max,
                 (cur_curbs && !cur_curbs->empty()) ? "true" : "false",
                 grid_stats.curbs_layer_exists ? "true" : "false",
                 grid_stats.curbs_updated ? "true" : "false",
                 grid_stats.elapsed_ms, grid_total_ms);
    }
}

// ============================================================================
// getRelPoseFromStamp: 获取某时间戳的相对位姿
// 对应 Python: TopoSLAMModel.get_rel_pose_from_stamp()
// ============================================================================
Pose2D TopoSLAMModel::getRelPoseFromStamp(double timestamp) const {
    if (rel_poses_stamped_.empty()) {
        return rel_pose_of_vcur_;
    }

    // 对齐 Python：取第一个 timestamp >= query 的位姿；若不存在则取最后一个
    int idx = 0;
    while (idx < static_cast<int>(rel_poses_stamped_.size()) &&
           rel_poses_stamped_[idx].timestamp < timestamp) {
        ++idx;
    }
    if (idx == static_cast<int>(rel_poses_stamped_.size())) idx -= 1;

    return rel_poses_stamped_[idx].pose;
}

// ============================================================================
// getRelPoseSinceLocalization: 计算自定位以来的相对位移
// 对应 Python: TopoSLAMModel.get_rel_pose_since_localization()
// ============================================================================
Pose2D TopoSLAMModel::getRelPoseSinceLocalization() const {
    if (rel_poses_stamped_.empty() || localization_results_.timestamp <= 0.0) {
        return rel_pose_of_vcur_;
    }

    Pose2D loc_rel_pose = getRelPoseFromStamp(localization_results_.timestamp);
    return getRelPose(
        Pose2D(loc_rel_pose[0], loc_rel_pose[1], loc_rel_pose[2]),
        Pose2D(rel_pose_of_vcur_[0], rel_pose_of_vcur_[1], rel_pose_of_vcur_[2])
    );
}

// ============================================================================
// checkPathCondition: 检查路径条件 (用于回环检测)
// 对应 Python: TopoSLAMModel.check_path_condition()
// ============================================================================
bool TopoSLAMModel::checkPathCondition(int u, int v) {
    auto path_result = graph_.getPathWithLength(u, v);
    if (!path_result.found) return true;  // 不可达, 允许回环

    Pose2D rel_pose_along_path = Pose2D::Zero();//初始化
    const auto& path = path_result.path;
    for (int i = 1; i < static_cast<int>(path.size()); ++i) {//遍历路径中的每一条边
        Pose2D edge = graph_.getEdge(path[i - 1], path[i]);
        rel_pose_along_path = applyPoseShift(rel_pose_along_path, edge);
    }
    //rel_pose_along_path表示沿旧路径累积相对位姿
    //straight_length近似表示从节点u直接到节点v的直线距离
    double straight_length = std::sqrt(rel_pose_along_path[0] * rel_pose_along_path[0] +
                                       rel_pose_along_path[1] * rel_pose_along_path[1]);
    //path_result.length相当于机器人沿着旧路线实际走过的距离
    return (path_result.length > 3.0 * straight_length || straight_length < 10.0);
    //条件A：沿图走的距离超过直接位移的三倍（旧路径很绕，存在捷径）
    //条件B：即使路径没有超过直线距离三倍，只要两个节点的几何距离小于10米，也允许判断为回环(两个节点本身就比较近)
}

// ============================================================================
// findLoopClosure: 回环检测
// 对应 Python: TopoSLAMModel.find_loop_closure()
// ============================================================================
TopoSLAMModel::LoopClosureCandidate
TopoSLAMModel::findLoopClosure(const std::vector<int>& vertex_ids,
                               const std::vector<double>& dists) {
    LoopClosureCandidate candidate;
                                       
    found_loop_closure_ = false;//每一帧进入回环检测时，先清除上一帧的结果。
    path_.clear();//保存发现回环前，节点u和v在旧拓扑图中的路径用于可视化

    if (trace_config_.enabled && trace_detailed_) {
        std::ostringstream candidates;
        candidates << std::fixed << std::setprecision(4) << "[";
        const size_t n_log = std::min(vertex_ids.size(), dists.size());
        for (size_t i = 0; i < n_log; ++i) {
            if (i > 0) candidates << ",";
            candidates << vertex_ids[i] << ":" << dists[i];
        }
        candidates << "]";
        ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                 "step=LOOP_CHECK candidates=%s",
                 trace_frame_id_, current_stamp_, candidates.str().c_str());
    }

    const size_t n = std::min(vertex_ids.size(), dists.size());
    for (size_t i = 0; i < n; ++i) {//遍历所有节点对
        for (size_t j = 0; j < n; ++j) {
            int u = vertex_ids[i];
            int v = vertex_ids[j];
            if (u < 0 || v < 0) continue;//过滤无效节点
            //Dijkstra 获取旧图路径
            auto path_result = graph_.getPathWithLength(u, v);
            if (!path_result.found) {
                if (trace_config_.enabled && trace_detailed_) {
                    ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                             "step=LOOP_PAIR u=%d v=%d path_found=false result=REJECT",
                             trace_frame_id_, current_stamp_, u, v);
                }
                continue;
            }
            //返回：是否存在路径.found、具体节点序列.path、路径总长度.length
            double dst_through_cur = dists[i] + dists[j];//计算当前位置到u、v两个点的距离之和作为新路径
            const bool path_long_enough = path_result.length > 5.0;
            const bool shortcut_better = path_result.length > 2.0 * dst_through_cur;
            bool geometry_condition = false;
            if (path_long_enough && shortcut_better) {
                geometry_condition = checkPathCondition(u, v);
            }

            if (trace_config_.enabled && trace_detailed_) {
                ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                         "step=LOOP_PAIR u=%d v=%d path_length=%.4f "
                         "dst_through_cur=%.4f path_gt_5=%s shortcut_gt_2x=%s "
                         "geometry_condition=%s result=%s",
                         trace_frame_id_, current_stamp_, u, v,
                         path_result.length, dst_through_cur,
                         path_long_enough ? "true" : "false",
                         shortcut_better ? "true" : "false",
                         geometry_condition ? "true" : "false",
                         (path_long_enough && shortcut_better && geometry_condition)
                             ? "ACCEPT" : "REJECT");
            }

            if (path_long_enough &&//旧路径必须超过5m，排除距离本来就很近的邻居节点防止反复触发回环
                shortcut_better &&//旧路径至少要比新路径长两倍
                geometry_condition) {//额外检查几何结构
                path_ = path_result.path;//存储路径，但不直接修改图结构，主循环里修改
                ROS_INFO("\n\n\n=== LOOP CLOSURE CANDIDATE: connect %d and %d through current ===\n\n\n",
                         u, v);
                if (trace_config_.enabled) {
                    ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                             "event=LOOP_CANDIDATE_DETECTED u=%d v=%d path_length=%.4f "
                             "dst_through_cur=%.4f",
                             trace_frame_id_, current_stamp_, u, v,
                             path_result.length, dst_through_cur);
                }
                candidate.found = true;
                candidate.u = u;
                candidate.v = v;
                candidate.path_length = path_result.length;
                candidate.distance_through_current = dst_through_cur;
                candidate.path = path_result.path;
                return candidate;
            }
        }
    }

    return candidate;
}

// ============================================================================
// isInsideVcur: 当前位置是否在当前节点栅格可见区域内
// 对应 Python: TopoSLAMModel.is_inside_vcur()
// ============================================================================
bool TopoSLAMModel::isInsideVcur() const {
    if (last_vertex_id_ < 0) return false;

    const Vertex& v = graph_.getVertex(last_vertex_id_);
    return v.grid.isInside(rel_pose_of_vcur_[0], rel_pose_of_vcur_[1], rel_pose_of_vcur_[2]);
    //将机器人相对于当前节点的位姿传给节点栅格，本质上是在检查：机器人当前位置是否位于该节点栅格的已知或有效区域中？
}

// ============================================================================
// reattachByEdge: 沿边匹配切换节点
// 对应 Python: TopoSLAMModel.reattach_by_edge()
// 流程:
// 1. 遍历当前节点的邻居
// 2. 对每个邻居计算预测位置与实际位置偏移
// 3. 如果偏移足够小, 调用 getTransformToVertex 做配准
// 4. 配准成功则切换到该邻居节点
// ============================================================================
bool TopoSLAMModel::reattachByEdge(bool require_match) {
    if (last_vertex_id_ < 0) {
        if (trace_config_.enabled && trace_detailed_) {
            ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                     "step=EDGE_REATTACH result=REJECT reason=NO_CURRENT_VERTEX",
                     trace_frame_id_, current_stamp_);
        }
        return false;
    }//当前节点还没初始化无法沿边切换
//last_vertex_id_表示机器人当前被归属到哪个拓扑节点
    const auto& edges = graph_.getEdgesFrom(last_vertex_id_);
    if (edges.empty()) {
        if (trace_config_.enabled && trace_detailed_) {
            ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                     "step=EDGE_REATTACH vertex=%d result=REJECT reason=NO_NEIGHBORS",
                     trace_frame_id_, current_stamp_, last_vertex_id_);
        }
        return false;
    }//当前节点还没任何邻居边无法沿边切换

    double min_dist = std::numeric_limits<double>::infinity();
    int nearest_vertex_id = -1;
    Pose2D pose_on_edge;
//rel_pose_of_vcur_表示机器人相对于当前节点中心坐标系的二维位姿
    // 计算当前位置距离当前节点中心的长度
    double dist_to_vcur = std::sqrt(rel_pose_of_vcur_[0] * rel_pose_of_vcur_[0] +
                                    rel_pose_of_vcur_[1] * rel_pose_of_vcur_[1]);
    std::ostringstream neighbor_trace;
    neighbor_trace << std::fixed << std::setprecision(4) << "[";

    for (const auto& entry : edges) {
        int neighbor_id = entry.vertex_id;
        const Pose2D& edge_rel_pose = entry.rel_pose;
//每条 edge_rel_pose 表示从当前节点中心到邻居节点中心的相对位姿
        double dx = rel_pose_of_vcur_[0] - edge_rel_pose[0];
        double dy = rel_pose_of_vcur_[1] - edge_rel_pose[1];
        double dist = std::sqrt(dx * dx + dy * dy);

        if (neighbor_trace.tellp() > 1) neighbor_trace << ";";
        neighbor_trace << entry.vertex_id << ":edge=("
                       << edge_rel_pose[0] << "," << edge_rel_pose[1] << ","
                       << edge_rel_pose[2] << "),robot_to_prediction=" << dist;

        if (dist < min_dist) {
            min_dist = dist;
            nearest_vertex_id = neighbor_id;
            pose_on_edge = edge_rel_pose;
        }
    }//遍历所有邻居边，找最接近机器人当前位置的邻居节点中心（保存最小值）
    neighbor_trace << "]";

    if (trace_config_.enabled && trace_detailed_) {
        ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                 "step=EDGE_REATTACH vertex=%d require_match=%s "
                 "robot_pose=(%.4f,%.4f,%.6f) dist_to_current=%.4f "
                 "neighbors=%s selected=%d selected_dist=%.4f",
                 trace_frame_id_, current_stamp_, last_vertex_id_,
                 require_match ? "true" : "false",
                 rel_pose_of_vcur_[0], rel_pose_of_vcur_[1],
                 rel_pose_of_vcur_[2], dist_to_vcur,
                 neighbor_trace.str().c_str(), nearest_vertex_id, min_dist);
    }

    bool changed = false;

    // 如果所有的边都太远，或者离目标节点的距离还不如离当前节点的距离小，则退出
    if (min_dist >= dist_to_vcur || min_dist >= 5.0 || nearest_vertex_id < 0) {
        if (trace_config_.enabled && trace_detailed_) {
            const char* reason = nearest_vertex_id < 0
                ? "NO_CANDIDATE"
                : (min_dist >= dist_to_vcur
                    ? "NOT_CLOSER_THAN_CURRENT"
                    : "CANDIDATE_DISTANCE_GE_5");
            ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                     "step=EDGE_REATTACH result=REJECT reason=%s",
                     trace_frame_id_, current_stamp_, reason);
        }
        return false;
    }

    Pose2D rel_pose_to_vertex = getRelPose(rel_pose_of_vcur_, pose_on_edge);
//rel_pose_of_vcur_：当前节点 → 机器人；pose_on_edge：当前节点 → 邻居节点
//rel_pose_to_vertex：机器人 → 邻居节点的位姿，但是这是基于拓扑边和里程计得到的初始预测
    if (require_match) {//默认为true表示必须通过栅格配准再次验证，不能仅凭图中的边位姿和里程计直接切换，因为：
        //边位姿可能包含建图误差；里程计会累积漂移；甚至可能存在之前错误建立的边
        // 先对齐栅格，然后再做配准
        LocalGrid cur_grid_transformed = cur_grid_.copy();//复制当前栅格，不能修改因为他仍是主循环当前观测
        Pose2D rel_pose_to_vertex_inv = graph_.inverseTransform(rel_pose_to_vertex[0], rel_pose_to_vertex[1], rel_pose_to_vertex[2]);
        cur_grid_transformed.transform(rel_pose_to_vertex_inv[0], rel_pose_to_vertex_inv[1], -rel_pose_to_vertex_inv[2]);
//将当前栅格粗略变换到邻居节点坐标系，对rel_pose_to_vertex取逆得到邻居 → 机器人：根据已有拓扑边和里程计，把当前观测粗略放到邻居节点坐标系中，提供配准初值
        auto tf_result = graph_.getTransformToVertex(nearest_vertex_id, cur_grid_transformed);
//比较：粗对齐后的当前栅格与邻居节点保存的历史栅格，配准失败则不切换
        if (trace_config_.enabled &&
            (trace_detailed_ || !tf_result.success)) {
            const auto& candidate_grid = graph_.getVertex(nearest_vertex_id).grid;
            ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=REGISTRATION] "
                     "type=inline candidate=%d ref=%dx%d ref_nonzero=%d "
                     "cand=%dx%d cand_nonzero=%d service_success=%s score=%.6f "
                     "threshold=%.6f pixel_tf=(%.3f,%.3f,%.6f) "
                     "metric_correction=(%.4f,%.4f,%.6f) elapsed_ms=%.3f",
                     trace_frame_id_, current_stamp_, nearest_vertex_id,
                     cur_grid_transformed.gridSize(), cur_grid_transformed.gridSize(),
                     cv::countNonZero(cur_grid_transformed.getLayer("occupancy")),
                     candidate_grid.gridSize(), candidate_grid.gridSize(),
                     cv::countNonZero(candidate_grid.getLayer("occupancy")),
                     tf_result.service_success ? "true" : "false", tf_result.score,
                     graph_.inlineRegistrationThreshold(), tf_result.trans_i,
                     tf_result.trans_j, tf_result.rot_angle,
                     tf_result.x, tf_result.y, tf_result.theta,
                     tf_result.elapsed_ms);
        }
        if (tf_result.success) {
            Pose2D corr_inv = graph_.inverseTransform(tf_result.x, tf_result.y, tf_result.theta);
            Pose2D final_pose = applyPoseShift(rel_pose_to_vertex, corr_inv);
//将配准修正融合到原预测位姿：机器人与邻居节点位姿=里程计给出的粗估计+栅格配准给出的精修正
            double diff = std::sqrt((final_pose[0] - rel_pose_to_vertex[0]) * (final_pose[0] - rel_pose_to_vertex[0]) +
                                    (final_pose[1] - rel_pose_to_vertex[1]) * (final_pose[1] - rel_pose_to_vertex[1]));
//计算修正量：配准结果与拓扑先验结果之间的差值
            if (diff < local_jump_threshold_) {//修正量是否小于阈值，检查局部一致性
                const int old_vertex_id = last_vertex_id_;
                ROS_INFO("Edge reattach: from vertex %d to vertex %d (match_dist=%.2f)",
                         last_vertex_id_, nearest_vertex_id, diff);
                //沿边切换成功后需要更新的状态
                rel_pose_of_vcur_ = graph_.inverseTransform(final_pose[0], final_pose[1], final_pose[2]);//取逆表示新当前节点 → 机器人
                last_vertex_id_ = nearest_vertex_id;//当前节点切换
                edge_reattach_cnt_++;
                last_successful_match_time_ = current_stamp_;
                rel_poses_stamped_.clear();//清空位姿时间历史，现在都换成当前节点→ 机器人
                rel_poses_stamped_.push_back({current_stamp_, rel_pose_of_vcur_});
                changed = true;
                trace_decision_ = "EDGE_SWITCH";
                if (trace_config_.enabled) {
                    ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                             "event=EDGE_SWITCH from=%d to=%d correction_jump=%.4f "
                             "jump_threshold=%.4f new_rel_pose=(%.4f,%.4f,%.6f)",
                             trace_frame_id_, current_stamp_, old_vertex_id,
                             nearest_vertex_id, diff, local_jump_threshold_,
                             rel_pose_of_vcur_[0], rel_pose_of_vcur_[1],
                             rel_pose_of_vcur_[2]);
                }
            } else if (trace_config_.enabled && trace_detailed_) {
                ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                         "step=EDGE_REATTACH candidate=%d result=REJECT "
                         "reason=CORRECTION_JUMP jump=%.4f threshold=%.4f",
                         trace_frame_id_, current_stamp_, nearest_vertex_id,
                         diff, local_jump_threshold_);
            }
        } else if (trace_config_.enabled && trace_detailed_) {
            ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                     "step=EDGE_REATTACH candidate=%d result=REJECT "
                     "reason=INLINE_REGISTRATION_FAILED",
                     trace_frame_id_, current_stamp_, nearest_vertex_id);
        }
    }

    if (!changed && !require_match) {//纯定位模式localization
        const int old_vertex_id = last_vertex_id_;
        ROS_INFO("Edge reattach (no match): from vertex %d to vertex %d", last_vertex_id_, nearest_vertex_id);
        rel_pose_of_vcur_ = graph_.inverseTransform(rel_pose_to_vertex[0], rel_pose_to_vertex[1], rel_pose_to_vertex[2]);
        last_vertex_id_ = nearest_vertex_id;
        edge_reattach_cnt_++;
        rel_poses_stamped_.clear();
        rel_poses_stamped_.push_back({current_stamp_, rel_pose_of_vcur_});
        changed = true;
        trace_decision_ = "LOCALIZATION_FALLBACK";
        if (trace_config_.enabled) {
            ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                     "event=LOCALIZATION_FALLBACK from=%d to=%d "
                     "new_rel_pose=(%.4f,%.4f,%.6f)",
                     trace_frame_id_, current_stamp_, old_vertex_id,
                     nearest_vertex_id, rel_pose_of_vcur_[0],
                     rel_pose_of_vcur_[1], rel_pose_of_vcur_[2]);
        }
    }

    if (changed) {
        need_to_change_vcur_ = false;//重置 need_to_change_vcur_表示切换已经解决了“当前节点不合适”的问题
        if (has_rel_pose_vcur_to_loc_) {
            Pose2D inv_pose_on_edge = graph_.inverseTransform(pose_on_edge[0], pose_on_edge[1], pose_on_edge[2]);
            rel_pose_vcur_to_loc_ = applyPoseShift(inv_pose_on_edge, rel_pose_vcur_to_loc_);//新节点坐标系下的定位参考位姿
        }
    }

    return changed;
}

// ============================================================================
// reattachByLocalization: 根据定位结果切换节点
// 对应 Python: TopoSLAMModel.reattach_by_localization()
//它可以实现：
//1.回到很早以前经过的区域；
//2.跨越非邻接节点重新归属；
//3.从局部里程计错误中恢复。
// ============================================================================
bool TopoSLAMModel::reattachByLocalization(double iou_threshold_val,
                                           double localized_stamp) {
    const auto& vertex_ids = localization_results_.vertex_ids_matched;
    const auto& rel_poses = localization_results_.rel_poses;
    //rel_poses[i] 是定位时刻当前观测相对于候选节点的配准位姿
    if (vertex_ids.empty() || rel_poses.empty()) {
        if (trace_config_.enabled && trace_detailed_) {
            ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                     "step=LOCALIZATION_REATTACH result=REJECT reason=NO_CANDIDATES",
                     trace_frame_id_, current_stamp_);
        }
        return false;
    }//没有定位结果就失败
    if (last_vertex_id_ < 0) return false;

    if (!rel_poses_stamped_.empty() && localized_stamp < rel_poses_stamped_.front().timestamp) {
        ROS_WARN("Old localization! Ignore it");
        if (trace_config_.enabled) {
            ROS_WARN("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                     "step=LOCALIZATION_REATTACH result=REJECT reason=OLD_COORDINATE_FRAME "
                     "loc_stamp=%.6f history_start=%.6f",
                     trace_frame_id_, current_stamp_, localized_stamp,
                     rel_poses_stamped_.front().timestamp);
        }
        return false;//拒绝跨节点坐标系的旧结果
    }
//定位延迟补偿的关键变量
    rel_pose_vcur_to_loc_ = getRelPoseFromStamp(localized_stamp);//表示在定位快照被取走的那一刻，机器人相对于当前节点的位姿
    has_rel_pose_vcur_to_loc_ = true;
    const Pose2D rel_pose_after_localization = getRelPose(rel_pose_vcur_to_loc_, rel_pose_of_vcur_);//表示从定位时刻到当前时刻，机器人又运动了多少
    const Pose2D rel_since_loc = getRelPoseSinceLocalization();//本质上也在计算定位发生以后累计的运动，后续用于将当前栅格对齐到候选节点
    const size_t n = std::min(vertex_ids.size(), rel_poses.size());

    for (size_t i = 0; i < n; ++i) {//逐个处理定位切换候选
        const int vid = vertex_ids[i];
        if (vid < 0 || vid >= graph_.numVertices()) continue;
        const Pose2D& loc_rel = rel_poses[i];

        Pose2D inv_loc_rel = graph_.inverseTransform(loc_rel[0], loc_rel[1], loc_rel[2]);
        Pose2D pred_rel_pose_vcur_to_v = applyPoseShift(rel_pose_vcur_to_loc_, inv_loc_rel);
        //旧当前节点 → 定位时刻机器人+定位时刻机器人 → 候选节点=旧当前节点 → 候选节点，用于于在 mapping 模式中添加：旧当前节点 ↔ 候选节点的拓扑边
        Pose2D rel_pose_robot_to_loc = getRelPose(rel_since_loc, loc_rel);
        double iou = cur_grid_.getIoU(graph_.getVertex(vid).grid,
                                      rel_pose_robot_to_loc[0],
                                      rel_pose_robot_to_loc[1],
                                      rel_pose_robot_to_loc[2]);
        //计算当前观测与候选节点的 IoU：将异步定位时刻的配准结果，补偿到当前时刻，再检查当前栅格与候选节点栅格是否仍然重叠
        Pose2D vcur_to_v = getRelPose(graph_.getVertex(last_vertex_id_).pose_for_visualization,
                                      graph_.getVertex(vid).pose_for_visualization);
        Pose2D cur_to_v = getRelPose(vcur_to_v, rel_pose_of_vcur_);
        double dst = std::sqrt(cur_to_v[0] * cur_to_v[0] + cur_to_v[1] * cur_to_v[1]);//使用两个节点的全局可视化位姿估算候选节点相对于当前机器人位置有多远
        const double drift_limit =
            drift_coef_ * (current_stamp_ - last_successful_match_time_) + 10.0;

        if (trace_config_.enabled && trace_detailed_) {
            ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                     "step=LOCALIZATION_CANDIDATE candidate=%d loc_stamp=%.6f "
                     "localized_pose=(%.4f,%.4f,%.6f) "
                     "motion_compensation=(%.4f,%.4f,%.6f) "
                     "compensated_pose=(%.4f,%.4f,%.6f) iou=%.6f "
                     "iou_gate=%.6f need_change=%s drift_distance=%.4f "
                     "drift_limit=%.4f",
                     trace_frame_id_, current_stamp_, vid, localized_stamp,
                     loc_rel[0], loc_rel[1], loc_rel[2],
                     rel_pose_after_localization[0],
                     rel_pose_after_localization[1],
                     rel_pose_after_localization[2],
                     rel_pose_robot_to_loc[0], rel_pose_robot_to_loc[1],
                     rel_pose_robot_to_loc[2], iou, iou_threshold_val,
                     need_to_change_vcur_ ? "true" : "false", dst, drift_limit);
        }

        if (dst > drift_limit) {
            ROS_INFO("Vertex %d is too far to match", vid);//距离上次可靠匹配越久，里程计可能漂移得越大，因此允许的候选距离也逐渐增大。
            //如果候选节点远得超出合理运动范围，则认为定位结果不可信。
            if (trace_config_.enabled && trace_detailed_) {
                ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                         "step=LOCALIZATION_CANDIDATE candidate=%d result=REJECT "
                         "reason=DRIFT_GATE",
                         trace_frame_id_, current_stamp_, vid);
            }
            continue;
        }

        if (iou > iou_threshold_val || need_to_change_vcur_) {//IoU足够高或者系统已经明确要求离开当前节点，后面恒为true
            ROS_INFO("Localization reattach: to vertex %d (IoU=%.3f, need_change=%s)",
                     vid, iou, need_to_change_vcur_ ? "true" : "false");
            last_successful_match_time_ = localized_stamp;
            const int old_vertex_id = last_vertex_id_;

            if (mode_ == "mapping") {
                if (trace_config_.enabled) {
                    const double predicted_length = std::hypot(
                        pred_rel_pose_vcur_to_v[0], pred_rel_pose_vcur_to_v[1]);
                    const Pose2D& old_global =
                        graph_.getVertex(old_vertex_id).pose_for_visualization;
                    const Pose2D& candidate_global =
                        graph_.getVertex(vid).pose_for_visualization;
                    const double direct_length = std::hypot(
                        candidate_global[0] - old_global[0],
                        candidate_global[1] - old_global[1]);
                    const double abs_diff =
                        std::abs(predicted_length - direct_length);
                    const double ratio =
                        std::max(predicted_length, direct_length) /
                        std::max(1e-6, std::min(predicted_length, direct_length));
                    const bool already_exists =
                        graph_.hasEdge(old_vertex_id, vid);
                    ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=EDGE] "
                             "EDGE_TYPE=LOCALIZATION source=%d target=%d "
                             "rel_pose=(%.4f,%.4f,%.6f) predicted_length=%.4f "
                             "global_direct_length=%.4f abs_diff=%.4f ratio=%.4f "
                             "already_exists=%s "
                             "validation=DRIFT_AND_IOU_ONLY action=%s",
                             trace_frame_id_, current_stamp_, old_vertex_id, vid,
                             pred_rel_pose_vcur_to_v[0],
                             pred_rel_pose_vcur_to_v[1],
                             pred_rel_pose_vcur_to_v[2], predicted_length,
                             direct_length, abs_diff, ratio,
                             already_exists ? "true" : "false",
                             already_exists ? "KEEP_EXISTING" : "ADD");
                }
                graph_.addEdge(last_vertex_id_, vid,
                               pred_rel_pose_vcur_to_v[0],
                               pred_rel_pose_vcur_to_v[1],
                               pred_rel_pose_vcur_to_v[2]);
            }//添加：旧当前节点 → 定位候选节点的边

            last_vertex_id_ = vid;
            need_to_change_vcur_ = false;//候选节点正式成为新的当前节点
            Pose2D pred_rel_pose = applyPoseShift(loc_rel, rel_pose_after_localization);
            rel_pose_of_vcur_ = pred_rel_pose;//当前时刻机器人相对于候选节点的位姿，防止机器人因使用旧定位结果而瞬间跳回历史位置

            Pose2D inv_pred_rel_pose_vcur_to_v =
                graph_.inverseTransform(pred_rel_pose_vcur_to_v[0], pred_rel_pose_vcur_to_v[1], pred_rel_pose_vcur_to_v[2]);
            rel_pose_vcur_to_loc_ = applyPoseShift(inv_pred_rel_pose_vcur_to_v, rel_pose_vcur_to_loc_);
            has_rel_pose_vcur_to_loc_ = true;
//切换前：rel_pose_vcur_to_loc_是基于旧当前节点的；切换后需要转换成基于候选节点的表达，确保后续异步定位时间补偿仍然使用同一个坐标系
            rel_poses_stamped_.clear();
            rel_poses_stamped_.push_back({current_stamp_, rel_pose_of_vcur_});
            trace_decision_ = "LOCALIZATION_SWITCH";
            if (trace_config_.enabled) {
                ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                         "event=LOCALIZATION_SWITCH from=%d to=%d loc_stamp=%.6f "
                         "iou=%.6f drift_distance=%.4f drift_limit=%.4f "
                         "new_rel_pose=(%.4f,%.4f,%.6f)",
                         trace_frame_id_, current_stamp_, old_vertex_id, vid,
                         localized_stamp, iou, dst, drift_limit,
                         rel_pose_of_vcur_[0], rel_pose_of_vcur_[1],
                         rel_pose_of_vcur_[2]);
            }
            return true;//清空历史因为当前节点坐标系已经改变
        }

        if (trace_config_.enabled && trace_detailed_) {
            ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                     "step=LOCALIZATION_CANDIDATE candidate=%d result=REJECT "
                     "reason=IOU_AND_CHANGE_GATE",
                     trace_frame_id_, current_stamp_, vid);
        }
    }

    return false;
}

// ============================================================================
// validateLoopEdges: 在创建节点前验证真正能够新增的回环边。
// 当前节点会通过顺序边连接，不计作新的回环约束。
// ============================================================================
std::vector<TopoSLAMModel::ValidatedLoopEdge>
TopoSLAMModel::validateLoopEdges(
    const std::vector<int>& vertex_ids,
    const std::vector<Pose2D>& rel_poses,
    const Pose2D& pose_stamped) const {
    std::vector<ValidatedLoopEdge> valid_edges;
    if (!has_rel_pose_vcur_to_loc_) {
        if (trace_config_.enabled && !vertex_ids.empty()) {
            ROS_WARN("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=EDGE] "
                     "EDGE_TYPE=LOOP_PRECHECK action=REJECT_ALL "
                     "reason=NO_TIME_REFERENCE candidates=%lu",
                     trace_frame_id_, current_stamp_, vertex_ids.size());
        }
        return valid_edges;
    }

    const int proposed_vertex_id = graph_.numVertices();
    const Pose2D proposed_vcur_to_loc =
        getRelPose(pose_stamped, rel_pose_vcur_to_loc_);
    std::unordered_set<int> seen_vertex_ids;
    const size_t n = std::min(vertex_ids.size(), rel_poses.size());

    for (size_t i = 0; i < n; ++i) {
        const int vid = vertex_ids[i];
        if (vid < 0 || vid >= graph_.numVertices()) {
            if (trace_config_.enabled) {
                ROS_WARN("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=EDGE] "
                         "EDGE_TYPE=LOOP_PRECHECK source=%d target=%d "
                         "action=REJECT reason=INVALID_VERTEX_ID",
                         trace_frame_id_, current_stamp_, proposed_vertex_id, vid);
            }
            continue;
        }
        if (!seen_vertex_ids.insert(vid).second) {
            continue;
        }
        if (vid == last_vertex_id_) {
            if (trace_config_.enabled && trace_detailed_) {
                ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=EDGE] "
                         "EDGE_TYPE=LOOP_PRECHECK source=%d target=%d "
                         "action=REJECT reason=SEQUENTIAL_VERTEX",
                         trace_frame_id_, current_stamp_, proposed_vertex_id, vid);
            }
            continue;
        }

        const Pose2D inv_rel = graph_.inverseTransform(
            rel_poses[i][0], rel_poses[i][1], rel_poses[i][2]);
        const Pose2D pred_rel_pose =
            applyPoseShift(proposed_vcur_to_loc, inv_rel);
        const double pred_dist = std::sqrt(
            pred_rel_pose[0] * pred_rel_pose[0] +
            pred_rel_pose[1] * pred_rel_pose[1]);
        const double direct_dx =
            global_pose_for_visualization_[0] -
            graph_.getVertex(vid).pose_for_visualization[0];
        const double direct_dy =
            global_pose_for_visualization_[1] -
            graph_.getVertex(vid).pose_for_visualization[1];
        const double direct_dist =
            std::sqrt(direct_dx * direct_dx + direct_dy * direct_dy);
        const double abs_diff = std::abs(pred_dist - direct_dist);
        const double ratio =
            std::max(pred_dist, direct_dist) /
            std::max(1e-6, std::min(pred_dist, direct_dist));
        const double direct_yaw = normalize(
            graph_.getVertex(vid).pose_for_visualization[2] -
            global_pose_for_visualization_[2]);
        const double yaw_error =
            std::abs(normalize(pred_rel_pose[2] - direct_yaw));

        if (pred_dist > max_edge_length_) {
            ROS_WARN("[NEWVTX] Rejecting proposed loop edge %d->%d: "
                     "pred_dist=%.1f > max_edge=%.1f",
                     proposed_vertex_id, vid, pred_dist, max_edge_length_);
            if (trace_config_.enabled) {
                ROS_WARN("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=EDGE] "
                         "EDGE_TYPE=LOOP_PRECHECK source=%d target=%d "
                         "pred_pose=(%.4f,%.4f,%.6f) predicted_length=%.4f "
                         "global_direct_length=%.4f abs_diff=%.4f ratio=%.4f "
                         "direct_yaw=%.6f yaw_error=%.6f "
                         "max_edge_length=%.4f action=REJECT "
                         "reason=PREDICTED_LENGTH",
                         trace_frame_id_, current_stamp_,
                         proposed_vertex_id, vid,
                         pred_rel_pose[0], pred_rel_pose[1], pred_rel_pose[2],
                         pred_dist, direct_dist, abs_diff, ratio,
                         direct_yaw, yaw_error,
                         max_edge_length_);
            }
            continue;
        }

        const double min_dist = std::min(pred_dist, direct_dist);
        const bool abs_distance_inconsistent =
            abs_diff > loop_edge_max_abs_distance_error_;
        const bool ratio_inconsistent =
            min_dist > loop_edge_ratio_min_distance_ &&
            ratio > loop_edge_max_distance_ratio_;
        const bool yaw_inconsistent =
            yaw_error > loop_edge_max_yaw_error_;
        if (abs_distance_inconsistent ||
            ratio_inconsistent ||
            yaw_inconsistent) {
            ROS_WARN("[NEWVTX] Rejecting inconsistent proposed loop edge "
                     "%d->%d: pred_dist=%.1f direct_dist=%.1f "
                     "(ratio=%.1f, diff=%.1f, yaw_error=%.2f)",
                     proposed_vertex_id, vid, pred_dist, direct_dist,
                     ratio, abs_diff, yaw_error);
            if (trace_config_.enabled) {
                ROS_WARN("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=EDGE] "
                         "EDGE_TYPE=LOOP_PRECHECK source=%d target=%d "
                         "pred_pose=(%.4f,%.4f,%.6f) predicted_length=%.4f "
                         "global_direct_length=%.4f abs_diff=%.4f ratio=%.4f "
                         "direct_yaw=%.6f yaw_error=%.6f "
                         "abs_distance_inconsistent=%s "
                         "ratio_inconsistent=%s yaw_inconsistent=%s "
                         "action=REJECT reason=GEOMETRY_INCONSISTENT",
                         trace_frame_id_, current_stamp_,
                         proposed_vertex_id, vid,
                         pred_rel_pose[0], pred_rel_pose[1], pred_rel_pose[2],
                         pred_dist, direct_dist, abs_diff, ratio,
                         direct_yaw, yaw_error,
                         abs_distance_inconsistent ? "true" : "false",
                         ratio_inconsistent ? "true" : "false",
                         yaw_inconsistent ? "true" : "false");
            }
            continue;
        }

        valid_edges.push_back(
            {vid, pred_rel_pose, pred_dist, direct_dist});
        if (trace_config_.enabled) {
            ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=EDGE] "
                     "EDGE_TYPE=LOOP_PRECHECK source=%d target=%d "
                     "pred_pose=(%.4f,%.4f,%.6f) predicted_length=%.4f "
                     "global_direct_length=%.4f abs_diff=%.4f ratio=%.4f "
                     "direct_yaw=%.6f yaw_error=%.6f "
                     "action=ACCEPT",
                     trace_frame_id_, current_stamp_,
                     proposed_vertex_id, vid,
                     pred_rel_pose[0], pred_rel_pose[1], pred_rel_pose[2],
                     pred_dist, direct_dist, abs_diff, ratio,
                     direct_yaw, yaw_error);
        }
    }
    return valid_edges;
}

// ============================================================================
// addNewVertex: 创建新的拓扑节点
// 对应 Python: TopoSLAMModel.add_new_vertex()
// ============================================================================
bool TopoSLAMModel::addNewVertex(const std::vector<int>& vertex_ids,
                                 const std::vector<Pose2D>& rel_poses,
                                 const LoopClosureCandidate* required_loop) {
    if (!graph_.isDescriptorValid(cur_desc_)) {
        ROS_ERROR("[NEWVTX] Refusing to create vertex with invalid descriptor "
                  "(dim=%lu)", cur_desc_.size());
        if (trace_config_.enabled) {
            ROS_ERROR("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=VERTEX] "
                      "event=SKIP_CREATE reason=INVALID_DESCRIPTOR "
                      "descriptor_dim=%lu graph_vertices=%d faiss_size=%d "
                      "faiss_identity=%d",
                      trace_frame_id_, current_stamp_, cur_desc_.size(),
                      graph_.numVertices(), graph_.indexSize(),
                      graph_.indexIdentitySize());
        }
        return false;
    }

    const Pose2D pose_stamped = getRelPoseFromStamp(current_stamp_);
    std::vector<int> vertices_to_validate = vertex_ids;
    std::vector<Pose2D> rel_poses_to_validate = rel_poses;
    if (required_loop != nullptr) {
        vertices_to_validate.clear();
        rel_poses_to_validate.clear();
        std::unordered_set<int> selected_endpoints;
        const int endpoints[2] = {required_loop->u, required_loop->v};
        for (int endpoint : endpoints) {
            if (!selected_endpoints.insert(endpoint).second) {
                continue;
            }
            const auto it = std::find(
                vertex_ids.begin(), vertex_ids.end(), endpoint);
            if (it == vertex_ids.end()) {
                ROS_WARN("[LOOP] Trigger endpoint %d is missing from "
                         "localization candidates", endpoint);
                if (trace_config_.enabled) {
                    ROS_WARN("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                             "event=LOOP_REJECTED "
                             "reason=TRIGGER_ENDPOINT_MISSING "
                             "trigger_u=%d trigger_v=%d missing_endpoint=%d "
                             "graph_vertices=%d graph_edges=%d",
                             trace_frame_id_, current_stamp_,
                             required_loop->u, required_loop->v, endpoint,
                             graph_.numVertices(),
                             graph_.undirectedEdgeCount());
                }
                return false;
            }
            const size_t index =
                static_cast<size_t>(std::distance(vertex_ids.begin(), it));
            if (index >= rel_poses.size()) {
                ROS_WARN("[LOOP] Trigger endpoint %d has no matching "
                         "relative pose", endpoint);
                if (trace_config_.enabled) {
                    ROS_WARN("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                             "event=LOOP_REJECTED "
                             "reason=TRIGGER_REL_POSE_MISSING "
                             "trigger_u=%d trigger_v=%d missing_endpoint=%d "
                             "graph_vertices=%d graph_edges=%d",
                             trace_frame_id_, current_stamp_,
                             required_loop->u, required_loop->v, endpoint,
                             graph_.numVertices(),
                             graph_.undirectedEdgeCount());
                }
                return false;
            }
            vertices_to_validate.push_back(endpoint);
            rel_poses_to_validate.push_back(rel_poses[index]);
        }
    }
    const std::vector<ValidatedLoopEdge> valid_loop_edges =
        validateLoopEdges(
            vertices_to_validate, rel_poses_to_validate, pose_stamped);
    if (required_loop != nullptr) {
        const auto endpoint_is_covered =
            [this, &valid_loop_edges](int endpoint) {
                if (endpoint == last_vertex_id_) {
                    return true;
                }
                return std::any_of(
                    valid_loop_edges.begin(), valid_loop_edges.end(),
                    [endpoint](const ValidatedLoopEdge& edge) {
                        return edge.vertex_id == endpoint;
                    });
            };
        const bool u_covered = endpoint_is_covered(required_loop->u);
        const bool v_covered = endpoint_is_covered(required_loop->v);
        if (trace_config_.enabled) {
            ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=EDGE] "
                     "EDGE_TYPE=LOOP_TRIGGER_PRECHECK "
                     "trigger_u=%d trigger_v=%d last_vertex=%d "
                     "u_covered=%s v_covered=%s valid_loop_edges=%lu "
                     "action=%s",
                     trace_frame_id_, current_stamp_,
                     required_loop->u, required_loop->v, last_vertex_id_,
                     u_covered ? "true" : "false",
                     v_covered ? "true" : "false",
                     valid_loop_edges.size(),
                     (u_covered && v_covered) ? "ACCEPT" : "REJECT");
        }
        if (!u_covered || !v_covered) {
            ROS_WARN("[LOOP] Candidate rejected before vertex creation: "
                     "trigger endpoints are not both covered");
            if (trace_config_.enabled) {
                ROS_WARN("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                         "event=LOOP_REJECTED "
                         "reason=TRIGGER_ENDPOINTS_NOT_COVERED "
                         "trigger_u=%d trigger_v=%d last_vertex=%d "
                         "u_covered=%s v_covered=%s "
                         "valid_loop_edges=%lu graph_vertices=%d "
                         "graph_edges=%d",
                         trace_frame_id_, current_stamp_,
                         required_loop->u, required_loop->v, last_vertex_id_,
                         u_covered ? "true" : "false",
                         v_covered ? "true" : "false",
                         valid_loop_edges.size(), graph_.numVertices(),
                         graph_.undirectedEdgeCount());
            }
            return false;
        }
    }

    const int vertices_before = graph_.numVertices();
    const int index_before = graph_.indexSize();
    const cv::Mat& current_occ = cur_grid_.getLayer("occupancy");
    cv::Mat occ_mask;
    cv::compare(current_occ, 0, occ_mask, cv::CMP_EQ);
    const int occupancy_unknown = cv::countNonZero(occ_mask);
    cv::compare(current_occ, 1, occ_mask, cv::CMP_EQ);
    const int occupancy_free = cv::countNonZero(occ_mask);
    cv::compare(current_occ, 2, occ_mask, cv::CMP_EQ);
    const int occupancy_obstacle = cv::countNonZero(occ_mask);

    int new_id = graph_.addVertex(//创建新节点保存当前帧的
        global_pose_for_visualization_,//全局可视化位姿
        cur_desc_,//当前描述符
        cur_grid_//当前局部栅格
    );
    if (new_id < 0) {
        if (trace_config_.enabled) {
            ROS_ERROR("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=VERTEX] "
                      "event=SKIP_CREATE reason=GRAPH_OR_FAISS_REJECTED "
                      "descriptor_dim=%lu graph_vertices=%d faiss_size=%d "
                      "faiss_identity=%d",
                      trace_frame_id_, current_stamp_, cur_desc_.size(),
                      graph_.numVertices(), graph_.indexSize(),
                      graph_.indexIdentitySize());
        }
        return false;
    }

    if (trace_config_.enabled) {
        ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=VERTEX] event=CREATE "
                 "vertices_before=%d vertices_after=%d new_id=%d "
                 "global_pose=(%.4f,%.4f,%.6f) descriptor_dim=%lu "
                 "occupancy_unknown=%d occupancy_free=%d occupancy_obstacle=%d "
                 "faiss_before=%d faiss_after=%d faiss_identity=%d",
                 trace_frame_id_, current_stamp_, vertices_before,
                 graph_.numVertices(), new_id,
                 global_pose_for_visualization_[0],
                 global_pose_for_visualization_[1],
                 global_pose_for_visualization_[2], cur_desc_.size(),
                 occupancy_unknown, occupancy_free, occupancy_obstacle,
                 index_before, graph_.indexSize(),
                 graph_.indexIdentitySize());
    }

    Pose2D new_rel_pose_of_vcur = getRelPose(pose_stamped, rel_pose_of_vcur_);
    //重置重置机器人在新节点中的位姿，理论上为[0，0，0]
    // Safety check: warn if edge is abnormally long
    double edge_dist = std::sqrt(pose_stamped[0] * pose_stamped[0] +
                                 pose_stamped[1] * pose_stamped[1]);
    if (edge_dist > max_edge_length_ * 3.0) {//添加安全性校验，剔除不符合检验的长边
        ROS_WARN("[DIAG] Abnormally long edge: %.2f m (pose_stamped=(%.2f,%.2f,%.2f))",
                 edge_dist, pose_stamped[0], pose_stamped[1], pose_stamped[2]);
    }

    // 诊断: 打印 addNewVertex 时的完整状态
    {
        double gx = global_pose_for_visualization_[0];
        double gy = global_pose_for_visualization_[1];
        double gth = global_pose_for_visualization_[2];
        double last_gx = 0.0, last_gy = 0.0, last_gth = 0.0;
        if (last_vertex_id_ >= 0) {
            const auto& lv = graph_.getVertex(last_vertex_id_);
            last_gx = lv.pose_for_visualization[0];
            last_gy = lv.pose_for_visualization[1];
            last_gth = lv.pose_for_visualization[2];
        }
        ROS_INFO("[NEWVTX] new_id=%d global_pose=(%.4f,%.4f,%.4f) "
                 "last_vtx=%d last_gpose=(%.4f,%.4f,%.4f) "
                 "pose_stamped=(%.4f,%.4f,%.4f) rel_pose_vcur=(%.4f,%.4f,%.4f) "
                 "n_localized=%lu n_rel_poses=%lu has_vcur_to_loc=%d",
                 new_id, gx, gy, gth,
                 last_vertex_id_, last_gx, last_gy, last_gth,
                 pose_stamped[0], pose_stamped[1], pose_stamped[2],
                 rel_pose_of_vcur_[0], rel_pose_of_vcur_[1], rel_pose_of_vcur_[2],
                 vertex_ids.size(), rel_poses.size(), has_rel_pose_vcur_to_loc_ ? 1 : 0);
    }

    // 建立新旧节点之间的拓扑边
    if (last_vertex_id_ >= 0) {
        const bool already_exists = graph_.hasEdge(last_vertex_id_, new_id);
        if (trace_config_.enabled) {
            ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=EDGE] "
                     "EDGE_TYPE=SEQUENTIAL source=%d target=%d "
                     "rel_pose=(%.4f,%.4f,%.6f) length=%.4f "
                     "warning_long=%s still_added=true already_exists=%s",
                     trace_frame_id_, current_stamp_, last_vertex_id_, new_id,
                     pose_stamped[0], pose_stamped[1], pose_stamped[2],
                     edge_dist,
                     edge_dist > max_edge_length_ * 3.0 ? "true" : "false",
                     already_exists ? "true" : "false");
        }
        graph_.addEdge(last_vertex_id_, new_id,
                       pose_stamped[0], pose_stamped[1], pose_stamped[2]);
        ROS_INFO("Add edge (%d)->(%d) rel_pose=(%.2f,%.2f,%.2f) dist=%.2f",
                 last_vertex_id_, new_id,
                 pose_stamped[0], pose_stamped[1], pose_stamped[2], edge_dist);
    }

    rel_pose_of_vcur_ = new_rel_pose_of_vcur;
    if (has_rel_pose_vcur_to_loc_) {
        rel_pose_vcur_to_loc_ = getRelPose(pose_stamped, rel_pose_vcur_to_loc_);
    }

    // 候选边已经在创建节点前完成校验。这里仅提交通过预检的边。
    for (const auto& edge : valid_loop_edges) {
        const bool already_exists = graph_.hasEdge(new_id, edge.vertex_id);
        if (trace_config_.enabled) {
            ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=EDGE] "
                     "EDGE_TYPE=LOOP_CANDIDATE source=%d target=%d "
                     "pred_pose=(%.4f,%.4f,%.6f) predicted_length=%.4f "
                     "global_direct_length=%.4f already_exists=%s "
                     "validation=PRECHECKED action=%s",
                     trace_frame_id_, current_stamp_, new_id, edge.vertex_id,
                     edge.rel_pose[0], edge.rel_pose[1], edge.rel_pose[2],
                     edge.predicted_length, edge.direct_length,
                     already_exists ? "true" : "false",
                     already_exists ? "KEEP_EXISTING" : "ADD");
        }
        graph_.addEdge(new_id, edge.vertex_id,
                       edge.rel_pose[0], edge.rel_pose[1], edge.rel_pose[2]);
        ROS_INFO("Add loop edge (%d)->(%d) rel_pose=(%.2f,%.2f,%.2f) pred_dist=%.1f direct_dist=%.1f",
                 new_id, edge.vertex_id,
                 edge.rel_pose[0], edge.rel_pose[1], edge.rel_pose[2],
                 edge.predicted_length, edge.direct_length);
    }
//新节点成为当前节点
    last_vertex_id_ = new_id;
    need_to_change_vcur_ = false;
    rel_poses_stamped_.clear();
    rel_poses_stamped_.push_back({current_stamp_, rel_pose_of_vcur_});
    if (trace_config_.enabled) {
        ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=VERTEX] "
                 "event=SET_CURRENT vertex=%d rel_pose=(%.4f,%.4f,%.6f) "
                 "graph_vertices=%d graph_edges=%d faiss_size=%d "
                 "faiss_identity=%d valid_loop_edges=%lu",
                 trace_frame_id_, current_stamp_, last_vertex_id_,
                 rel_pose_of_vcur_[0], rel_pose_of_vcur_[1],
                 rel_pose_of_vcur_[2], graph_.numVertices(),
                 graph_.undirectedEdgeCount(), graph_.indexSize(),
                 graph_.indexIdentitySize(), valid_loop_edges.size());
    }
    return true;
}

// ============================================================================
// correctRelPose: 相对位姿校正
// 对应 Python: TopoSLAMModel.correct_rel_pose()
// ============================================================================
void TopoSLAMModel::correctRelPose() {
    if (last_vertex_id_ < 0) return;

    if (cur_grid_.getLayer("occupancy").empty()) return;

    Pose2D rel_old = getRelPoseFromStamp(current_stamp_);
    LocalGrid cur_grid_transformed = cur_grid_.copy();
    cur_grid_transformed.transform(rel_old[0], rel_old[1], -rel_old[2]);

    auto tf_result = graph_.getTransformToVertex(last_vertex_id_, cur_grid_transformed);

    if (tf_result.success) {
        double match_dist = std::sqrt(tf_result.x * tf_result.x + tf_result.y * tf_result.y);
        if (match_dist < 0.5) { // Python 中为 0.5
            Pose2D a_inv_v_inv_n = applyPoseShift(Pose2D(tf_result.x, tf_result.y, tf_result.theta), rel_pose_of_vcur_);
            rel_pose_of_vcur_ = a_inv_v_inv_n;
            rel_pose_cnt_++;
        }
    }
}

// ============================================================================
// initLocalization: 初始定位 (仅首帧, localization 模式)
// 对应 Python: TopoSLAMModel.init_localization()
// ============================================================================
void TopoSLAMModel::initLocalization() {
    if (mode_ == "localization" && graph_.numVertices() > 0) {
        if (start_location_ >= 0) {
            // 使用预设的起始位置
            last_vertex_id_ = start_location_;
            if (has_start_local_pose_) {
                rel_pose_of_vcur_ = start_local_pose_;
            }
            need_to_change_vcur_ = false;
            rel_pose_vcur_to_loc_ = rel_pose_of_vcur_;
            has_rel_pose_vcur_to_loc_ = true;
            rel_poses_stamped_.clear();
            rel_poses_stamped_.push_back({current_stamp_, rel_pose_of_vcur_});
            ROS_INFO("Init localization: using preset location %d", start_location_);
        } else {
            // 等待定位器结果
            localizer_.localize();
            auto state = localizer_.getLocalizedState();
            if (!state.vertex_ids_matched.empty()) {
                last_vertex_id_ = state.vertex_ids_matched[0];
                Pose2D inv_rel = graph_.inverseTransform(state.rel_poses[0][0], state.rel_poses[0][1], state.rel_poses[0][2]);
                rel_pose_of_vcur_ = inv_rel;
                need_to_change_vcur_ = false;
                rel_pose_vcur_to_loc_ = rel_pose_of_vcur_;
                has_rel_pose_vcur_to_loc_ = true;
                rel_poses_stamped_.clear();
                rel_poses_stamped_.push_back({current_stamp_, rel_pose_of_vcur_});
                ROS_INFO("Init localization success: vertex %d", last_vertex_id_);
            } else {
                ROS_WARN("Init localization failed, waiting for next frame...");
            }
        }
    }
}

// ============================================================================
// saveGraph
// ============================================================================
void TopoSLAMModel::saveGraph() {
    if (!path_to_save_graph_.empty()) {
        graph_.saveToJson(path_to_save_graph_);
    }
}

// ============================================================================
// getPathToMetricGoal: 获取到目标点的拓扑路径
// 对应 Python: prism_topomap_node.py get_navigation_subgoal 中的路径查找
// ============================================================================
std::vector<int> TopoSLAMModel::getPathToMetricGoal(double x, double y) {
    // 找到离目标最近的顶点
    double best_dist = std::numeric_limits<double>::infinity();
    int best_id = -1;

    for (int i = 0; i < graph_.numVertices(); ++i) {
        double dx = graph_.getVertex(i).pose_for_visualization[0] - x;
        double dy = graph_.getVertex(i).pose_for_visualization[1] - y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < best_dist) {
            best_dist = dist;
            best_id = i;
        }
    }

    if (best_id < 0 || last_vertex_id_ < 0) return {};

    auto path_result = graph_.getPathWithLength(last_vertex_id_, best_id);
    if (path_result.found) {
        return path_result.path;
    }
    return {};
}

void TopoSLAMModel::logFlowSummary(double update_start_wall_sec) {
    if (!trace_config_.enabled) return;

    const double elapsed_ms =
        (ros::WallTime::now().toSec() - update_start_wall_sec) * 1000.0;
    const double localization_stamp = localization_results_.timestamp;
    const double localization_age =
        localization_stamp > 0.0 ? current_stamp_ - localization_stamp : -1.0;
    const char* inside_text = trace_inside_valid_
        ? (trace_inside_ ? "true" : "false")
        : "NA";

    ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=SUMMARY] "
             "vertex_before=%d vertex_after=%d decision=%s inside=%s "
             "iou=%.6f rel_dist=%.4f localization_stamp=%.6f "
             "localization_age=%.6f matched=%lu unmatched=%lu "
             "graph_vertices=%d graph_edges=%d faiss_size=%d "
             "faiss_identity=%d update_elapsed_ms=%.3f",
             trace_frame_id_, current_stamp_, trace_vertex_before_,
             last_vertex_id_, trace_decision_.c_str(), inside_text,
             trace_inside_valid_ ? cur_iou_ : -1.0, trace_rel_dist_,
             localization_stamp, localization_age,
             localization_results_.vertex_ids_matched.size(),
             localization_results_.vertex_ids_unmatched.size(),
             graph_.numVertices(), graph_.undirectedEdgeCount(),
             graph_.indexSize(), graph_.indexIdentitySize(), elapsed_ms);
}

// ============================================================================
// update: 主更新循环
// 对应 Python: TopoSLAMModel.update()
//
// 这是整个系统的核心函数, 每帧调用一次。
// 严格按照原 Python 逻辑的顺序执行。
// ============================================================================
void TopoSLAMModel::update(
    const Pose2D& global_pose,
    const Pose2D& cur_odom_pose,
    const sensor_msgs::PointCloud2& cloud_msg,
    const PointCloudPtr& cur_cloud,
    bool has_image_front, bool has_image_back,
    const sensor_msgs::Image& image_front,
    const sensor_msgs::Image& image_back,
    const PointCloudPtr& cur_curbs) {

    const double flow_update_start = ros::WallTime::now().toSec();
    trace_vertex_before_ = last_vertex_id_;
    trace_decision_ = "UNSET";
    trace_inside_valid_ = false;
    trace_inside_ = false;
    trace_rel_dist_ = 0.0;
    found_loop_closure_ = false;
    path_.clear();
    global_pose_for_visualization_ = global_pose;

    // =============================================
    // TRACE: 打印 update() 入口参数 (调试时取消注释)
    // =============================================
    // ROS_DEBUG("[TRACE] update() entry: stamp=%.3f mode=%s "
    //          "global_pose=(%.4f,%.4f,%.4f) cur_odom_pose=(%.4f,%.4f,%.4f) "
    //          "odom_pose_=(%.4f,%.4f,%.4f) odom_init=%d last_vid=%d",
    //          current_stamp_, mode_.c_str(),
    //          global_pose[0], global_pose[1], global_pose[2],
    //          cur_odom_pose[0], cur_odom_pose[1], cur_odom_pose[2],
    //          odom_pose_[0], odom_pose_[1], odom_pose_[2],
    //          odom_initialized_ ? 1 : 0, last_vertex_id_);

    // 步骤A：里程计积分运算 —— grid_shift 用于栅格坐标变换
    // Python原版代码：x,y,theta = get_rel_pose(*cur_odom_pose, *self.odom_pose)
    // 含义：（源坐标系=新位姿，目标坐标系=旧位姿）——逆变换；
    // 该逆变换会在观测处理函数 process_observations 中与负偏角(-theta)结合，
    // 最终算出正确的栅格仿射偏移量。
    // =============================================
    Pose2D grid_shift = Pose2D::Zero();
    const bool odom_was_initialized = odom_initialized_;
    const Pose2D prev_odom_pose = odom_pose_;
    const Pose2D rel_pose_before = rel_pose_of_vcur_;
    Pose2D forward_delta = Pose2D::Zero();
    if (odom_initialized_) {
        grid_shift = getRelPose(cur_odom_pose, odom_pose_);
        forward_delta = getRelPose(odom_pose_, cur_odom_pose);
    }
    updateRelPoseByOdom(cur_odom_pose);

    if (trace_config_.enabled && trace_detailed_) {
        ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=ODOM] initialized_before=%s "
                 "prev_odom=(%.4f,%.4f,%.6f) cur_odom=(%.4f,%.4f,%.6f) "
                 "delta=(%.4f,%.4f,%.6f) grid_shift=(%.4f,%.4f,%.6f) "
                 "rel_pose_before=(%.4f,%.4f,%.6f) rel_pose_after=(%.4f,%.4f,%.6f)",
                 trace_frame_id_, current_stamp_,
                 odom_was_initialized ? "true" : "false",
                 prev_odom_pose[0], prev_odom_pose[1], prev_odom_pose[2],
                 cur_odom_pose[0], cur_odom_pose[1], cur_odom_pose[2],
                 forward_delta[0], forward_delta[1], forward_delta[2],
                 grid_shift[0], grid_shift[1], grid_shift[2],
                 rel_pose_before[0], rel_pose_before[1], rel_pose_before[2],
                 rel_pose_of_vcur_[0], rel_pose_of_vcur_[1], rel_pose_of_vcur_[2]);
    }

    // ROS_DEBUG("[DIAG] grid_shift=(%.4f, %.4f, %.4f) rel_pose_vcur=(%.2f, %.2f, %.2f)",
    //           grid_shift[0], grid_shift[1], grid_shift[2],
    //           rel_pose_of_vcur_[0], rel_pose_of_vcur_[1], rel_pose_of_vcur_[2]);

    // =============================================
    // 步骤 B: 观测处理 (描述符提取 + 栅格更新)
    // =============================================
    processObservations(cloud_msg, cur_cloud,
                        has_image_front, has_image_back,
                        image_front, image_back,
                        cur_curbs,
                        grid_shift[0], grid_shift[1], grid_shift[2]);

    // =============================================
    // 步骤 C: 更新定位器状态
    // =============================================
    localizer_.updateCurrentState(global_pose, cur_desc_, cur_grid_, current_stamp_,
                                  trace_frame_id_, trace_detailed_);
    //global_pose	当前全局可视化位姿 cur_desc_	当前地点描述符
    //cur_grid_	当前局部占据栅格  current_stamp_	当前观测时间戳
    // 记录带时间戳的相对位姿
    rel_poses_stamped_.push_back({current_stamp_, rel_pose_of_vcur_});

    // =============================================
    // 步骤 D: 初始定位 (仅首帧)
    // =============================================
    if (last_vertex_id_ < 0) {
        if (mode_ == "localization") {
            initLocalization();
            trace_decision_ = last_vertex_id_ >= 0
                ? "LOCALIZATION_INIT"
                : "WAIT_LOCALIZATION";
            logFlowSummary(flow_update_start);
            return;
        } else {
            // mapping 模式: 仅在 descriptor 有效且已成功进入 FAISS 后
            // 才提交第一个顶点。
            if (addNewVertex({}, {})) {
                trace_decision_ = "FIRST_VERTEX";
            } else {
                trace_decision_ = "WAIT_DESCRIPTOR";
            }
            logFlowSummary(flow_update_start);
            return;
        }
    }

    // =============================================
    // 步骤 E: 获取定位结果
    // =============================================
    localization_results_ = localizer_.getLocalizedState();
    if (localization_results_.timestamp > 0.0) {
        localization_time_ = localization_results_.timestamp;
    }
    const double localized_stamp = localization_results_.timestamp;
    const bool localization_is_fresh =
        (rel_poses_stamped_.empty() ||
         localized_stamp <= 0.0 ||
         localized_stamp >= rel_poses_stamped_.front().timestamp - 1e-3);
    if (localization_is_fresh && localized_stamp > 0.0) {
        rel_pose_vcur_to_loc_ = getRelPoseFromStamp(localized_stamp);
        has_rel_pose_vcur_to_loc_ = true;
    }

    if (trace_config_.enabled && trace_detailed_) {
        const double age = localized_stamp > 0.0
            ? current_stamp_ - localized_stamp
            : -1.0;
        ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=LOCALIZATION_RESULT] "
                 "action=CONSUME result_stamp=%.6f age=%.6f fresh=%s "
                 "matched=%lu unmatched=%lu rel_pose_vcur_to_loc=(%.4f,%.4f,%.6f) "
                 "has_time_reference=%s",
                 trace_frame_id_, current_stamp_, localized_stamp, age,
                 localization_is_fresh ? "true" : "false",
                 localization_results_.vertex_ids_matched.size(),
                 localization_results_.vertex_ids_unmatched.size(),
                 rel_pose_vcur_to_loc_[0], rel_pose_vcur_to_loc_[1],
                 rel_pose_vcur_to_loc_[2],
                 has_rel_pose_vcur_to_loc_ ? "true" : "false");

        std::ostringstream matched_candidates;
        matched_candidates << "[";
        for (size_t i = 0;
             i < localization_results_.vertex_ids_matched.size(); ++i) {
            if (i > 0) matched_candidates << ",";
            matched_candidates << localization_results_.vertex_ids_matched[i];
        }
        matched_candidates << "]";
        ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] action=BEGIN "
                 "last_vertex=%d rel_pose=(%.4f,%.4f,%.6f) "
                 "localization_candidates=%s need_change_initial=%s",
                 trace_frame_id_, current_stamp_, last_vertex_id_,
                 rel_pose_of_vcur_[0], rel_pose_of_vcur_[1],
                 rel_pose_of_vcur_[2], matched_candidates.str().c_str(),
                 need_to_change_vcur_ ? "true" : "false");
    }

    // =============================================
    // 步骤 F: 回环检测 (mapping 模式)
    // =============================================
    if (mode_ == "mapping" && localization_is_fresh) {
        std::vector<int> vertex_ids = localization_results_.vertex_ids_matched;//当前观测可能匹配到的拓扑节点 ID
        std::vector<Pose2D> rel_poses = localization_results_.rel_poses;//当前观测到这些候选节点的估计距离
        if (vertex_ids.size() == rel_poses.size()) {
            bool has_last_vertex = false;//主动将当前节点也加入候选
            for (int id : vertex_ids) {
                if (id == last_vertex_id_) {
                    has_last_vertex = true;
                    break;
                }
            }
            //如果当前节点不在定位结果中：
            if (!has_last_vertex && last_vertex_id_ >= 0 && has_rel_pose_vcur_to_loc_) {
                vertex_ids.push_back(last_vertex_id_);
                Pose2D inv_rel_pose = graph_.inverseTransform(rel_pose_vcur_to_loc_[0],
                                                              rel_pose_vcur_to_loc_[1],
                                                              rel_pose_vcur_to_loc_[2]);
                rel_poses.push_back(inv_rel_pose);
            }

            std::vector<double> dists;
            dists.reserve(rel_poses.size());
            for (const auto& rp : rel_poses) {
                dists.push_back(std::sqrt(rp[0] * rp[0] + rp[1] * rp[1]));//根据相对位姿只使用平移部分计算距离
            }
            const LoopClosureCandidate loop_candidate =
                findLoopClosure(vertex_ids, dists);
            if (loop_candidate.found) {
                ROS_INFO("Loop candidate found. Validate loop edges before "
                         "creating a vertex");
                if (addNewVertex(
                        vertex_ids, rel_poses, &loop_candidate)) {
                    found_loop_closure_ = true;
                    trace_decision_ = "LOOP_NEW_VERTEX";
                    if (trace_config_.enabled) {
                        const int path_start = path_.empty() ? -1 : path_.front();
                        const int path_end = path_.empty() ? -1 : path_.back();
                        ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f]"
                                 "[STAGE=DECISION] event=LOOP_DETECTED "
                                 "result=CONFIRMED trigger_u=%d trigger_v=%d "
                                 "path_start=%d path_end=%d "
                                 "graph_vertices=%d graph_edges=%d",
                                 trace_frame_id_, current_stamp_,
                                 loop_candidate.u, loop_candidate.v,
                                 path_start, path_end, graph_.numVertices(),
                                 graph_.undirectedEdgeCount());
                    }
                    logFlowSummary(flow_update_start);
                    return;
                }
                found_loop_closure_ = false;
                path_.clear();
            }
        }
    }

    // =============================================
    // 步骤 G: 当前节点切换判定
    // =============================================

    // G.1: 沿边匹配切换
    bool changed = reattachByEdge(true);
//成功的话last_vertex_id_已经变成新节点；rel_pose_of_vcur_也已经变成机器人相对于新节点的位姿，后面的IoU就会自动针对新节点重新计算
    // G.2: IoU 判定
    Pose2D inv_rel_pose = graph_.inverseTransform(rel_pose_of_vcur_[0],
                                                  rel_pose_of_vcur_[1],
                                                  rel_pose_of_vcur_[2]);
    cur_iou_ = cur_grid_.getIoU(graph_.getVertex(last_vertex_id_).grid,
                                inv_rel_pose[0],
                                inv_rel_pose[1],
                                inv_rel_pose[2],
                                false, iou_cnt_++);

    // G.3: 核心切换判定 (严格对齐 Python)
    bool inside_vcur = isInsideVcur();
    double rel_dist = std::sqrt(rel_pose_of_vcur_[0] * rel_pose_of_vcur_[0] +
                                rel_pose_of_vcur_[1] * rel_pose_of_vcur_[1]);
    trace_inside_valid_ = true;
    trace_inside_ = inside_vcur;
    trace_rel_dist_ = rel_dist;

    if (trace_config_.enabled && trace_detailed_) {
        ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                 "step=KEEP_CHECK vertex=%d inside=%s iou=%.6f "
                 "iou_threshold=%.6f rel_dist=%.4f max_edge_length=%.4f "
                 "edge_changed=%s",
                 trace_frame_id_, current_stamp_, last_vertex_id_,
                 inside_vcur ? "true" : "false", cur_iou_, iou_threshold_,
                 rel_dist, max_edge_length_, changed ? "true" : "false");
    }

    if (!inside_vcur || cur_iou_ < iou_threshold_ || rel_dist > max_edge_length_) {
        need_to_change_vcur_ = true;
        if (trace_config_.enabled && trace_detailed_) {
            ROS_INFO("[FLOW][FRAME=%d][STAMP=%.6f][STAGE=DECISION] "
                     "decision=NEED_CHANGE reason_inside=%s reason_iou=%s "
                     "reason_distance=%s",
                     trace_frame_id_, current_stamp_,
                     inside_vcur ? "false" : "true",
                     cur_iou_ < iou_threshold_ ? "true" : "false",
                     rel_dist > max_edge_length_ ? "true" : "false");
        }
        // 打印因为什么原因想切换/创建顶点
        if (!inside_vcur) {
            ROS_INFO("Moved outside vcur %d", last_vertex_id_);
        } else if (cur_iou_ < iou_threshold_) {
            ROS_INFO("Low IoU %.3f < %.3f", cur_iou_, iou_threshold_);
        } else {
            ROS_INFO("Too far from location center (dist=%.2f > %.2f)", rel_dist, max_edge_length_);
        }

        if (!changed) {
            // 判断异步定位结果是否过旧
            if (current_stamp_ - localization_results_.timestamp < 5.0) {
                // 尝试根据定位结果直接跳过去
                changed = reattachByLocalization(cur_iou_, localization_results_.timestamp);
                //当前节点的直接邻居中没有合适节点，但全局位置识别认为机器人可能位于图中的其他节点
                // 如果定位也没能跳成功
                if (!changed && mode_ == "mapping") {
                    ROS_INFO("No proper vertex to change. Add new vertex");
                    trace_decision_ = "NEW_VERTEX";
                    bool vertex_created = false;
                    if (localization_is_fresh) {
                        vertex_created = addNewVertex(
                            localization_results_.vertex_ids_matched,
                            localization_results_.rel_poses);
                    } else {
                        vertex_created = addNewVertex({}, {});
                    }
                    if (!vertex_created) {
                        trace_decision_ = "WAIT_DESCRIPTOR";
                    }
                }
            } else {
                // 定位数据太旧了
                if (mode_ == "mapping") {
                    ROS_INFO("No recent localization. Add new vertex");
                    trace_decision_ =
                        addNewVertex({}, {}) ? "NEW_VERTEX" : "WAIT_DESCRIPTOR";
                } else {
                    ROS_WARN("No recent localization");
                    trace_decision_ = "WAIT_LOCALIZATION";
                }
            }

            // 在 localization 模式的最终兜底
            if (!changed && mode_ == "localization") {
                const bool fallback_changed = reattachByEdge(false);
                trace_decision_ = fallback_changed
                    ? "LOCALIZATION_FALLBACK"
                    : "WAIT_LOCALIZATION";
            }
        }
    } else if (trace_decision_ == "UNSET") {
        trace_decision_ = "KEEP";
    }

    // 重新计算并输出状态（因为位姿可能在切换节点时被重置）
    rel_dist = std::sqrt(rel_pose_of_vcur_[0] * rel_pose_of_vcur_[0] +
                         rel_pose_of_vcur_[1] * rel_pose_of_vcur_[1]);
    trace_rel_dist_ = rel_dist;
    if (trace_decision_ == "UNSET") {
        trace_decision_ = changed ? "EDGE_SWITCH" : "KEEP";
    }
    ROS_INFO("vtx=%d, rel_pose=(%.1f,%.1f,%.2f), dist=%.1f, IoU=%.3f, total_vtx=%d",
             last_vertex_id_,
             rel_pose_of_vcur_[0], rel_pose_of_vcur_[1], rel_pose_of_vcur_[2],
             rel_dist, cur_iou_, graph_.numVertices());
    logFlowSummary(flow_update_start);
}

} // namespace prism_topomap
