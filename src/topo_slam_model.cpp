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
                               const std::string& path_to_save_logs)
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
                 path_to_save_logs),
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
      path_to_save_logs_(path_to_save_logs) {

    initParamsFromConfig(config);

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
        return;
    }
    Pose2D rel_odom = getRelPose(odom_pose_, cur_odom_pose);
    rel_pose_of_vcur_ = applyPoseShift(rel_pose_of_vcur_, rel_odom);
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

    // 1. 调用 Python 推理服务提取描述符
    auto desc_result = inference_client_->getDescriptor(
        cloud_msg,
        has_image_front, has_image_back,
        img_front, img_back,
        pointcloud_quantization_size_);

    if (desc_result.success) {
        cur_desc_ = desc_result.descriptor;
        // 确保描述符是正确维度
        ROS_DEBUG("描述符提取成功, 维度=%lu", cur_desc_.size());
    } else {
        cur_desc_.clear();
        ROS_WARN("Descriptor extraction failed!");
    }

    // 2. 点云投影到栅格 (C++ 本地计算)
    // 注意: Python 中传入的 theta 取反 (-theta)
    cur_grid_.updateFromCloudAndTransform(cur_cloud, x, y, -theta);

    // 3. Curb update
    if (cur_curbs && !cur_curbs->empty()) {
        cur_grid_.updateCurbsFromCloud(cur_curbs);
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

    Pose2D rel_pose_along_path = Pose2D::Zero();
    const auto& path = path_result.path;
    for (int i = 1; i < static_cast<int>(path.size()); ++i) {
        Pose2D edge = graph_.getEdge(path[i - 1], path[i]);
        rel_pose_along_path = applyPoseShift(rel_pose_along_path, edge);
    }

    double straight_length = std::sqrt(rel_pose_along_path[0] * rel_pose_along_path[0] +
                                       rel_pose_along_path[1] * rel_pose_along_path[1]);
    return (path_result.length > 3.0 * straight_length || straight_length < 10.0);
}

// ============================================================================
// findLoopClosure: 回环检测
// 对应 Python: TopoSLAMModel.find_loop_closure()
// ============================================================================
bool TopoSLAMModel::findLoopClosure(const std::vector<int>& vertex_ids,
                                    const std::vector<double>& dists) {
    found_loop_closure_ = false;
    path_.clear();

    const size_t n = std::min(vertex_ids.size(), dists.size());
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            int u = vertex_ids[i];
            int v = vertex_ids[j];
            if (u < 0 || v < 0) continue;

            auto path_result = graph_.getPathWithLength(u, v);
            if (!path_result.found) continue;

            double dst_through_cur = dists[i] + dists[j];
            if (path_result.length > 5.0 &&
                path_result.length > 2.0 * dst_through_cur &&
                checkPathCondition(u, v)) {
                found_loop_closure_ = true;
                path_ = path_result.path;
                ROS_INFO("\n\n\n=== LOOP CLOSURE FOUND! connect %d and %d through current ===\n\n\n",
                         u, v);
                return true;
            }
        }
    }

    return false;
}

// ============================================================================
// isInsideVcur: 当前位置是否在当前节点栅格可见区域内
// 对应 Python: TopoSLAMModel.is_inside_vcur()
// ============================================================================
bool TopoSLAMModel::isInsideVcur() const {
    if (last_vertex_id_ < 0) return false;

    const Vertex& v = graph_.getVertex(last_vertex_id_);
    return v.grid.isInside(rel_pose_of_vcur_[0], rel_pose_of_vcur_[1], rel_pose_of_vcur_[2]);
}

// ============================================================================
// reattachByEdge: 沿边匹配切换节点
// 对应 Python: TopoSLAMModel.reattach_by_edge()
//
// 流程:
// 1. 遍历当前节点的邻居
// 2. 对每个邻居计算预测位置与实际位置偏移
// 3. 如果偏移足够小, 调用 getTransformToVertex 做配准
// 4. 配准成功则切换到该邻居节点
// ============================================================================
bool TopoSLAMModel::reattachByEdge(bool require_match) {
    if (last_vertex_id_ < 0) return false;

    const auto& edges = graph_.getEdgesFrom(last_vertex_id_);
    if (edges.empty()) return false;

    double min_dist = std::numeric_limits<double>::infinity();
    int nearest_vertex_id = -1;
    Pose2D pose_on_edge;

    // 计算当前位置距离自身的长度
    double dist_to_vcur = std::sqrt(rel_pose_of_vcur_[0] * rel_pose_of_vcur_[0] +
                                    rel_pose_of_vcur_[1] * rel_pose_of_vcur_[1]);

    for (const auto& entry : edges) {
        int neighbor_id = entry.vertex_id;
        const Pose2D& edge_rel_pose = entry.rel_pose;

        double dx = rel_pose_of_vcur_[0] - edge_rel_pose[0];
        double dy = rel_pose_of_vcur_[1] - edge_rel_pose[1];
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < min_dist) {
            min_dist = dist;
            nearest_vertex_id = neighbor_id;
            pose_on_edge = edge_rel_pose;
        }
    }

    bool changed = false;

    // 如果所有的边都太远，或者离目标点的距离还不如离当前原点的距离小，则退出
    if (min_dist >= dist_to_vcur || min_dist >= 5.0 || nearest_vertex_id < 0) {
        return false;
    }

    Pose2D rel_pose_to_vertex = getRelPose(rel_pose_of_vcur_, pose_on_edge);

    if (require_match) {
        // 先对齐栅格，然后再做配准
        LocalGrid cur_grid_transformed = cur_grid_.copy();
        Pose2D rel_pose_to_vertex_inv = graph_.inverseTransform(rel_pose_to_vertex[0], rel_pose_to_vertex[1], rel_pose_to_vertex[2]);
        cur_grid_transformed.transform(rel_pose_to_vertex_inv[0], rel_pose_to_vertex_inv[1], -rel_pose_to_vertex_inv[2]);

        auto tf_result = graph_.getTransformToVertex(nearest_vertex_id, cur_grid_transformed);

        if (tf_result.success) {
            Pose2D corr_inv = graph_.inverseTransform(tf_result.x, tf_result.y, tf_result.theta);
            Pose2D final_pose = applyPoseShift(rel_pose_to_vertex, corr_inv);
            
            double diff = std::sqrt((final_pose[0] - rel_pose_to_vertex[0]) * (final_pose[0] - rel_pose_to_vertex[0]) +
                                    (final_pose[1] - rel_pose_to_vertex[1]) * (final_pose[1] - rel_pose_to_vertex[1]));

            if (diff < local_jump_threshold_) {
                ROS_INFO("Edge reattach: from vertex %d to vertex %d (match_dist=%.2f)",
                         last_vertex_id_, nearest_vertex_id, diff);
                rel_pose_of_vcur_ = graph_.inverseTransform(final_pose[0], final_pose[1], final_pose[2]);
                last_vertex_id_ = nearest_vertex_id;
                edge_reattach_cnt_++;
                last_successful_match_time_ = current_stamp_;
                rel_poses_stamped_.clear();
                rel_poses_stamped_.push_back({current_stamp_, rel_pose_of_vcur_});
                changed = true;
            }
        }
    }

    if (!changed && !require_match) {
        ROS_INFO("Edge reattach (no match): from vertex %d to vertex %d", last_vertex_id_, nearest_vertex_id);
        rel_pose_of_vcur_ = graph_.inverseTransform(rel_pose_to_vertex[0], rel_pose_to_vertex[1], rel_pose_to_vertex[2]);
        last_vertex_id_ = nearest_vertex_id;
        edge_reattach_cnt_++;
        rel_poses_stamped_.clear();
        rel_poses_stamped_.push_back({current_stamp_, rel_pose_of_vcur_});
        changed = true;
    }

    if (changed) {
        need_to_change_vcur_ = false;
        if (has_rel_pose_vcur_to_loc_) {
            Pose2D inv_pose_on_edge = graph_.inverseTransform(pose_on_edge[0], pose_on_edge[1], pose_on_edge[2]);
            rel_pose_vcur_to_loc_ = applyPoseShift(inv_pose_on_edge, rel_pose_vcur_to_loc_);
        }
    }

    return changed;
}

// ============================================================================
// reattachByLocalization: 根据定位结果切换节点
// 对应 Python: TopoSLAMModel.reattach_by_localization()
// ============================================================================
bool TopoSLAMModel::reattachByLocalization(double iou_threshold_val,
                                           double localized_stamp) {
    const auto& vertex_ids = localization_results_.vertex_ids_matched;
    const auto& rel_poses = localization_results_.rel_poses;
    if (vertex_ids.empty() || rel_poses.empty()) return false;
    if (last_vertex_id_ < 0) return false;

    if (!rel_poses_stamped_.empty() && localized_stamp < rel_poses_stamped_.front().timestamp) {
        ROS_WARN("Old localization! Ignore it");
        return false;
    }

    rel_pose_vcur_to_loc_ = getRelPoseFromStamp(localized_stamp);
    has_rel_pose_vcur_to_loc_ = true;
    const Pose2D rel_pose_after_localization = getRelPose(rel_pose_vcur_to_loc_, rel_pose_of_vcur_);
    const Pose2D rel_since_loc = getRelPoseSinceLocalization();
    const size_t n = std::min(vertex_ids.size(), rel_poses.size());

    for (size_t i = 0; i < n; ++i) {
        const int vid = vertex_ids[i];
        if (vid < 0 || vid >= graph_.numVertices()) continue;
        const Pose2D& loc_rel = rel_poses[i];

        Pose2D inv_loc_rel = graph_.inverseTransform(loc_rel[0], loc_rel[1], loc_rel[2]);
        Pose2D pred_rel_pose_vcur_to_v = applyPoseShift(rel_pose_vcur_to_loc_, inv_loc_rel);
        Pose2D rel_pose_robot_to_loc = getRelPose(rel_since_loc, loc_rel);
        double iou = cur_grid_.getIoU(graph_.getVertex(vid).grid,
                                      rel_pose_robot_to_loc[0],
                                      rel_pose_robot_to_loc[1],
                                      rel_pose_robot_to_loc[2]);

        Pose2D vcur_to_v = getRelPose(graph_.getVertex(last_vertex_id_).pose_for_visualization,
                                      graph_.getVertex(vid).pose_for_visualization);
        Pose2D cur_to_v = getRelPose(vcur_to_v, rel_pose_of_vcur_);
        double dst = std::sqrt(cur_to_v[0] * cur_to_v[0] + cur_to_v[1] * cur_to_v[1]);
        if (dst > drift_coef_ * (current_stamp_ - last_successful_match_time_) + 10.0) {
            ROS_INFO("Vertex %d is too far to match", vid);
            continue;
        }

        if (iou > iou_threshold_val || need_to_change_vcur_) {
            ROS_INFO("Localization reattach: to vertex %d (IoU=%.3f, need_change=%s)",
                     vid, iou, need_to_change_vcur_ ? "true" : "false");
            last_successful_match_time_ = localized_stamp;

            if (mode_ == "mapping") {
                graph_.addEdge(last_vertex_id_, vid,
                               pred_rel_pose_vcur_to_v[0],
                               pred_rel_pose_vcur_to_v[1],
                               pred_rel_pose_vcur_to_v[2]);
            }

            last_vertex_id_ = vid;
            need_to_change_vcur_ = false;
            Pose2D pred_rel_pose = applyPoseShift(loc_rel, rel_pose_after_localization);
            rel_pose_of_vcur_ = pred_rel_pose;

            Pose2D inv_pred_rel_pose_vcur_to_v =
                graph_.inverseTransform(pred_rel_pose_vcur_to_v[0], pred_rel_pose_vcur_to_v[1], pred_rel_pose_vcur_to_v[2]);
            rel_pose_vcur_to_loc_ = applyPoseShift(inv_pred_rel_pose_vcur_to_v, rel_pose_vcur_to_loc_);
            has_rel_pose_vcur_to_loc_ = true;

            rel_poses_stamped_.clear();
            rel_poses_stamped_.push_back({current_stamp_, rel_pose_of_vcur_});
            return true;
        }
    }

    return false;
}

// ============================================================================
// addNewVertex: 创建新的拓扑节点
// 对应 Python: TopoSLAMModel.add_new_vertex()
// ============================================================================
void TopoSLAMModel::addNewVertex(const std::vector<int>& vertex_ids,
                                  const std::vector<Pose2D>& rel_poses) {
    int new_id = graph_.addVertex(
        global_pose_for_visualization_,
        cur_desc_,
        cur_grid_
    );

    Pose2D pose_stamped = getRelPoseFromStamp(current_stamp_);
    Pose2D new_rel_pose_of_vcur = getRelPose(pose_stamped, rel_pose_of_vcur_);

    // 添加从上一个节点到新节点的边
    if (last_vertex_id_ >= 0) {
        graph_.addEdge(last_vertex_id_, new_id,
                       pose_stamped[0], pose_stamped[1], pose_stamped[2]);
    }

    rel_pose_of_vcur_ = new_rel_pose_of_vcur;
    if (has_rel_pose_vcur_to_loc_) {
        rel_pose_vcur_to_loc_ = getRelPose(pose_stamped, rel_pose_vcur_to_loc_);
    }

    // 如果有定位匹配结果, 添加回环边
    const size_t n = std::min(vertex_ids.size(), rel_poses.size());
    for (size_t i = 0; i < n; ++i) {
        int vid = vertex_ids[i];
        if (vid < 0 || vid >= graph_.numVertices()) continue;
        if (!has_rel_pose_vcur_to_loc_) continue;

        Pose2D inv_rel = graph_.inverseTransform(rel_poses[i][0], rel_poses[i][1], rel_poses[i][2]);
        Pose2D pred_rel_pose = applyPoseShift(rel_pose_vcur_to_loc_, inv_rel);
        graph_.addEdge(new_id, vid, pred_rel_pose[0], pred_rel_pose[1], pred_rel_pose[2]);
    }

    last_vertex_id_ = new_id;
    need_to_change_vcur_ = false;
    rel_poses_stamped_.clear();
    rel_poses_stamped_.push_back({current_stamp_, rel_pose_of_vcur_});
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

    global_pose_for_visualization_ = global_pose;

    // Step A: Odometry integration
    // =============================================
    Pose2D grid_shift = Pose2D::Zero();
    if (odom_initialized_) {
        // FIX: parameter order was reversed (from=new, to=old).
        // Correct: from=old_pose, to=new_pose → gives forward increment.
        grid_shift = getRelPose(odom_pose_, cur_odom_pose);
    }
    updateRelPoseByOdom(cur_odom_pose);

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
    localizer_.updateCurrentState(global_pose, cur_desc_, cur_grid_, current_stamp_);

    // 记录带时间戳的相对位姿
    rel_poses_stamped_.push_back({current_stamp_, rel_pose_of_vcur_});

    // =============================================
    // 步骤 D: 初始定位 (仅首帧)
    // =============================================
    if (last_vertex_id_ < 0) {
        if (mode_ == "localization") {
            initLocalization();
            return;
        } else {
            // mapping 模式: 直接创建第一个顶点
            addNewVertex({}, {});
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

    // =============================================
    // 步骤 F: 回环检测 (mapping 模式)
    // =============================================
    if (mode_ == "mapping" && localization_is_fresh) {
        std::vector<int> vertex_ids = localization_results_.vertex_ids_matched;
        std::vector<Pose2D> rel_poses = localization_results_.rel_poses;
        if (vertex_ids.size() == rel_poses.size()) {
            bool has_last_vertex = false;
            for (int id : vertex_ids) {
                if (id == last_vertex_id_) {
                    has_last_vertex = true;
                    break;
                }
            }
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
                dists.push_back(std::sqrt(rp[0] * rp[0] + rp[1] * rp[1]));
            }
            if (findLoopClosure(vertex_ids, dists)) {
                ROS_INFO("Found loop closure. Add new vertex to close loop");
                addNewVertex(vertex_ids, rel_poses);
                return;
            }
        }
    }

    // =============================================
    // 步骤 G: 当前节点切换判定
    // =============================================

    // G.1: 沿边匹配切换
    bool changed = reattachByEdge(true);

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

    if (!inside_vcur || cur_iou_ < iou_threshold_ || rel_dist > max_edge_length_) {
        need_to_change_vcur_ = true;
        // 打印因为什么原因想切换/创建顶点
        if (!inside_vcur) {
            ROS_INFO("Moved outside vcur %d", last_vertex_id_);
        } else if (cur_iou_ < iou_threshold_) {
            ROS_INFO("Low IoU %.3f < %.3f", cur_iou_, iou_threshold_);
        } else {
            ROS_INFO("Too far from location center (dist=%.2f > %.2f)", rel_dist, max_edge_length_);
        }

        if (!changed) {
            // 判断 localization 是否过旧
            if (current_stamp_ - localization_results_.timestamp < 5.0) {
                // 尝试根据定位结果直接跳过去
                changed = reattachByLocalization(cur_iou_, localization_results_.timestamp);
                
                // 如果定位也没能跳成功
                if (!changed && mode_ == "mapping") {
                    ROS_INFO("No proper vertex to change. Add new vertex");
                    if (localization_is_fresh) {
                        addNewVertex(localization_results_.vertex_ids_matched,
                                     localization_results_.rel_poses);
                    } else {
                        addNewVertex({}, {});
                    }
                }
            } else {
                // 定位数据太旧了
                if (mode_ == "mapping") {
                    ROS_INFO("No recent localization. Add new vertex");
                    addNewVertex({}, {});
                } else {
                    ROS_WARN("No recent localization");
                }
            }

            // 在 localization 模式的最终兜底
            if (!changed && mode_ == "localization") {
                reattachByEdge(false);
            }
        }
    }

    // 重新计算并输出状态（因为位姿可能在切换节点时被重置）
    rel_dist = std::sqrt(rel_pose_of_vcur_[0] * rel_pose_of_vcur_[0] +
                         rel_pose_of_vcur_[1] * rel_pose_of_vcur_[1]);
    ROS_INFO("vtx=%d, rel_pose=(%.1f,%.1f,%.2f), dist=%.1f, IoU=%.3f, total_vtx=%d",
             last_vertex_id_,
             rel_pose_of_vcur_[0], rel_pose_of_vcur_[1], rel_pose_of_vcur_[2],
             rel_dist, cur_iou_, graph_.numVertices());
}

} // namespace prism_topomap
