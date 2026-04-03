/**
 * @file topo_slam_model.h
 * @brief 核心拓扑 SLAM 算法类
 *
 * 对应原 Python 文件: scripts/prism_topomap.py → TopoSLAMModel
 * 管理整个 SLAM 主循环: 里程计积分, 观测处理, 节点切换, 回环检测
 */
#pragma once

#include "prism_topomap/topological_graph.h"
#include "prism_topomap/local_grid.h"
#include "prism_topomap/localizer.h"
#include "prism_topomap/inference_client.h"
#include "prism_topomap/utils.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <mutex>
#include <memory>

namespace prism_topomap {

class TopoSLAMModel {
public:
    /**
     * @brief 构造函数
     * @param config             YAML 配置
     * @param inference_client   Python 推理服务客户端
     * @param path_to_load_graph 加载已有图的路径 (空=从零开始)
     * @param path_to_save_graph 保存图的路径
     * @param path_to_save_logs  保存日志的路径
     */
    TopoSLAMModel(const YAML::Node& config,
                   std::shared_ptr<InferenceClient> inference_client,
                   const std::string& path_to_load_graph = "",
                   const std::string& path_to_save_graph = "",
                   const std::string& path_to_save_logs = "");

    /**
     * @brief 主更新入口 (每帧调用)
     * 对应 Python: TopoSLAMModel.update()
     */
    void update(const Pose2D& global_pose,
                const Pose2D& cur_odom_pose,
                const sensor_msgs::PointCloud2& cloud_msg,
                const PointCloud& cur_cloud,
                bool has_image_front,
                bool has_image_back,
                const sensor_msgs::Image& image_front,
                const sensor_msgs::Image& image_back,
                const PointCloud* cur_curbs);

    /**
     * @brief 相对位姿校正
     * 对应 Python: TopoSLAMModel.correct_rel_pose()
     */
    void correctRelPose();

    /**
     * @brief 保存图
     */
    void saveGraph();

    // === 供 Node 层访问的接口 ===
    TopologicalGraph& graph() { return graph_; }
    Localizer& localizer() { return localizer_; }
    const LocalGrid& curGrid() const { return cur_grid_; }
    const Vertex& lastVertex() const { return graph_.getVertex(last_vertex_id_); }
    int lastVertexId() const { return last_vertex_id_; }
    const Pose2D& relPoseOfVcur() const { return rel_pose_of_vcur_; }
    bool foundLoopClosure() const { return found_loop_closure_; }
    const std::vector<int>& loopClosurePath() const { return path_; }
    double curIoU() const { return cur_iou_; }
    double currentStamp() const { return current_stamp_; }
    void setCurrentStamp(double stamp) { current_stamp_ = stamp; }
    const std::string& mode() const { return mode_; }
    double localizationFrequency() const { return localization_frequency_; }

    // 导航
    std::vector<int> getPathToMetricGoal(double x, double y);

private:
    // === 核心内部方法 ===
    void initParamsFromConfig(const YAML::Node& config);

    void processObservations(const sensor_msgs::PointCloud2& cloud_msg,
                             const PointCloud& cur_cloud,
                             bool has_image_front,
                             bool has_image_back,
                             const sensor_msgs::Image& img_front,
                             const sensor_msgs::Image& img_back,
                             const PointCloud* cur_curbs,
                             double x, double y, double theta);

    void updateRelPoseByOdom(const Pose2D& cur_odom_pose);

    void initLocalization();

    bool findLoopClosure(const std::vector<int>& vertex_ids,
                         const std::vector<double>& dists);
    bool checkPathCondition(int u, int v);
    bool isInsideVcur() const;

    bool reattachByEdge(bool require_match = true);
    bool reattachByLocalization(double iou_threshold, double localized_stamp, bool force_reattach = false);

    void addNewVertex(const std::vector<int>& vertex_ids,
                      const std::vector<Pose2D>& rel_poses);

    // === 位姿历史查询 ===
    Pose2D getRelPoseFromStamp(double timestamp) const;
    Pose2D getRelPoseSinceLocalization() const;

    // === 成员变量 ===
    std::shared_ptr<InferenceClient> inference_client_;
    TopologicalGraph graph_;
    Localizer localizer_;
    LocalGrid cur_grid_;

    // 当前状态
    int last_vertex_id_ = -1;
    Pose2D rel_pose_of_vcur_ = Pose2D::Zero();
    Pose2D odom_pose_ = Pose2D::Zero();
    bool odom_initialized_ = false;
    Pose2D global_pose_for_visualization_ = Pose2D::Zero();
    std::vector<float> cur_desc_;

    // 带时间戳的相对位姿历史
    struct StampedPose {
        double timestamp;
        Pose2D pose;
    };
    std::vector<StampedPose> rel_poses_stamped_;

    // 状态标志
    bool found_loop_closure_ = false;
    bool need_to_change_vcur_ = false;
    std::vector<int> path_;
    double current_stamp_ = 0.0;
    double cur_iou_ = 0.0;
    double localization_time_ = 0.0;
    double last_successful_match_time_ = 0.0;
    LocalizedState localization_results_;

    // 统计
    int edge_reattach_cnt_ = 0;
    int rel_pose_cnt_ = 0;
    int iou_cnt_ = 0;

    // 参数 (从 config 读取)
    std::string mode_;
    double iou_threshold_;
    double localization_frequency_;
    double rel_pose_correction_frequency_;
    double max_edge_length_;
    double drift_coef_;
    double localization_timeout_;
    double floor_height_;
    double ceil_height_;
    float pointcloud_quantization_size_;
    double grid_resolution_;
    double grid_radius_;
    double max_grid_range_;
    double reg_score_threshold_;
    double inline_reg_score_threshold_;
    double local_jump_threshold_;
    std::string map_frame_;
    int top_k_;
    int start_location_ = -1;
    Pose2D start_local_pose_ = Pose2D::Zero();
    bool has_start_local_pose_ = false;

    // 保存路径
    std::string path_to_load_graph_;
    std::string path_to_save_graph_;
    std::string path_to_save_logs_;

    std::mutex mutex_;
};

} // namespace prism_topomap
