/**
 * @file results_publisher.h
 * @brief ROS 可视化发布类
 *
 * 对应原 Python 文件: prism_topomap_node.py 中的 ResultsPublisher 类
 * 以及散布在 PRISMTopomapNode 中的各种 publish 方法
 */
#pragma once

#include "prism_topomap/topological_graph.h"
#include "prism_topomap/local_grid.h"
#include "prism_topomap/utils.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <toposlam_msgs/TopologicalPath.h>

namespace prism_topomap {

class ResultsPublisher {
public:
    ResultsPublisher(ros::NodeHandle& nh, const std::string& map_frame);

    // === 发布拓扑图可视化 ===
    void publishGraph(const TopologicalGraph& graph, int last_vertex_id);

    // === 发布当前节点 ===
    void publishLastVertex(const Vertex& last_vertex, int last_vertex_id,
                           const ros::Time& stamp);

    // === 发布相对位姿 ===
    void publishRelPose(const Pose2D& rel_pose, const ros::Time& stamp);

    // === 发布当前栅格 ===
    void publishCurGrid(const LocalGrid& grid, const ros::Time& stamp);

    // === 发布当前节点栅格 ===
    void publishLastVertexGrid(const LocalGrid& grid, const ros::Time& stamp);

    // === 发布定位结果 ===
    void publishLocalizationResults(const TopologicalGraph& graph,
                                     const std::vector<int>& matched_ids,
                                     const std::vector<Pose2D>& rel_poses,
                                     const std::vector<int>& unmatched_ids);

    // === 发布回环检测结果 ===
    void publishLoopClosureResults(const TopologicalGraph& graph,
                                    const std::vector<int>& path,
                                    const Pose2D& global_pose);

    // === 发布拓扑路径 ===
    void publishTopologicalPath(const TopologicalGraph& graph,
                                 const std::vector<int>& path,
                                 const ros::Time& stamp);

    // === 发布导航子目标 ===
    void publishSubgoal(const Pose2D& subgoal, const ros::Time& stamp);

    // === freeze/unfreeze ===
    void freeze();
    void unfreeze();

    // === 从里程计发布 TF (与 Python 一致: 使用消息中的 frame_id) ===
    void publishTfFromOdom(const nav_msgs::Odometry::ConstPtr& msg);

private:
    // 辅助: 将 LocalGrid 转为 OccupancyGrid 消息
    nav_msgs::OccupancyGrid gridToOccMsg(const LocalGrid& grid,
                                          const ros::Time& stamp) const;

    ros::Publisher graph_viz_pub_;
    ros::Publisher last_vertex_pub_;
    ros::Publisher last_vertex_id_pub_;
    ros::Publisher rel_pose_pub_;
    ros::Publisher local_grid_pub_;
    ros::Publisher cur_grid_pub_;
    ros::Publisher matched_points_pub_;
    ros::Publisher unmatched_points_pub_;
    ros::Publisher loop_closure_pub_;
    ros::Publisher freeze_pub_;
    ros::Publisher path_pub_;
    ros::Publisher path_marker_pub_;
    ros::Publisher pointgoal_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    std::string map_frame_;
};

} // namespace prism_topomap
