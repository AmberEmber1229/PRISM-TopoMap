/**
 * @file results_publisher.cpp
 * @brief ROS 可视化发布类的实现
 *
 * 对应原 Python prism_topomap_node.py 中的可视化发布逻辑
 */
#include "prism_topomap/results_publisher.h"
#include <tf/transform_datatypes.h>

namespace prism_topomap {

// ============================================================================
// 构造函数: 创建所有 Publisher
// ============================================================================
ResultsPublisher::ResultsPublisher(ros::NodeHandle& nh, const std::string& map_frame)
    : map_frame_(map_frame) {
    graph_viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "/topological_map", 1, true);
    last_vertex_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(
        "/last_vertex_grid", 1, true);
    last_vertex_id_pub_ = nh.advertise<std_msgs::Int32>(
        "/last_vertex_id", 1, true);
    rel_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
        "/rel_pose", 1, true);
    local_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(
        "/local_grid", 1, true);
    cur_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(
        "/cur_grid", 1, true);
    matched_points_pub_ = nh.advertise<visualization_msgs::Marker>(
        "/matched_points", 1, true);
    unmatched_points_pub_ = nh.advertise<visualization_msgs::Marker>(
        "/unmatched_points", 1, true);
    loop_closure_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "/loop_closure", 1, true);
    freeze_pub_ = nh.advertise<std_msgs::Bool>(
        "/freeze_command", 1, true);
    path_pub_ = nh.advertise<toposlam_msgs::TopologicalPath>(
        "/topological_path", 100, true);
    path_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "/topological_path_markers", 1, true);
    pointgoal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
        "/navigation_subgoal", 1, true);
}

// ============================================================================
// gridToOccMsg: LocalGrid → nav_msgs::OccupancyGrid
// ============================================================================
nav_msgs::OccupancyGrid ResultsPublisher::gridToOccMsg(
    const LocalGrid& grid, const ros::Time& stamp) const {

    nav_msgs::OccupancyGrid msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.info.resolution = static_cast<float>(grid.resolution());
    msg.info.width = grid.gridSize();
    msg.info.height = grid.gridSize();
    msg.info.origin.position.x = -grid.radius();
    msg.info.origin.position.y = -grid.radius();
    msg.info.origin.orientation.w = 1.0;

    const cv::Mat& occ = grid.getLayer("occupancy");
    msg.data.resize(grid.gridSize() * grid.gridSize());
    for (int r = 0; r < grid.gridSize(); ++r) {
        for (int c = 0; c < grid.gridSize(); ++c) {
            uint8_t val = occ.at<uint8_t>(r, c);
            int8_t occ_val = -1;  // unknown
            if (val == 0) occ_val = -1;      // unknown
            else if (val == 1) occ_val = 0;  // free
            else if (val == 2) occ_val = 100; // occupied
            msg.data[r * grid.gridSize() + c] = occ_val;
        }
    }

    return msg;
}

// ============================================================================
// publishGraph
// ============================================================================
void ResultsPublisher::publishGraph(const TopologicalGraph& graph,
                                     int last_vertex_id) {
    visualization_msgs::MarkerArray marker_array;
    ros::Time now = ros::Time::now();

    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    int marker_id = 0;

    for (int i = 0; i < graph.numVertices(); ++i) {
        const auto& v = graph.getVertex(i);

        visualization_msgs::Marker vertex_marker;
        vertex_marker.header.frame_id = map_frame_;
        vertex_marker.header.stamp = now;
        vertex_marker.ns = "vertices";
        vertex_marker.id = marker_id++;
        vertex_marker.type = visualization_msgs::Marker::SPHERE;
        vertex_marker.action = visualization_msgs::Marker::ADD;
        vertex_marker.pose.position.x = v.pose_for_visualization[0];
        vertex_marker.pose.position.y = v.pose_for_visualization[1];
        vertex_marker.pose.position.z = 0.0;
        vertex_marker.pose.orientation.w = 1.0;

        double size = (i == last_vertex_id) ? 0.7 : 0.3;
        vertex_marker.scale.x = size;
        vertex_marker.scale.y = size;
        vertex_marker.scale.z = size;

        if (i == last_vertex_id) {
            vertex_marker.color.r = 1.0; vertex_marker.color.g = 0.0;
            vertex_marker.color.b = 0.0; vertex_marker.color.a = 1.0;
        } else {
            vertex_marker.color.r = 0.0; vertex_marker.color.g = 1.0;
            vertex_marker.color.b = 0.0; vertex_marker.color.a = 1.0;
        }
        marker_array.markers.push_back(vertex_marker);

        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = map_frame_;
        text_marker.header.stamp = now;
        text_marker.ns = "labels";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = v.pose_for_visualization[0];
        text_marker.pose.position.y = v.pose_for_visualization[1];
        text_marker.pose.position.z = 0.5;
        text_marker.scale.z = 0.4; // 稍微放大一点字体
        text_marker.color.r = 1.0; text_marker.color.g = 1.0;
        text_marker.color.b = 1.0; text_marker.color.a = 1.0;
        
        // 拼接节点索引与绝对位姿
        std::stringstream ss;
        ss << i << ": (" << std::fixed << std::setprecision(1) 
           << v.pose_for_visualization[0] << ", " 
           << v.pose_for_visualization[1] << ", " 
           << std::setprecision(2) << v.pose_for_visualization[2] << ")";
           
        // 查找上一节点的边以拼接相对位姿
        if (i > 0 && graph.hasEdge(i - 1, i)) {
            Pose2D rel = graph.getEdge(i - 1, i);
            ss << "\nrel: (" << std::fixed << std::setprecision(1) 
               << rel[0] << ", " << rel[1] << ", " 
               << std::setprecision(2) << rel[2] << ")";
        }
        
        text_marker.text = ss.str();
        marker_array.markers.push_back(text_marker);
    }

    for (int u = 0; u < graph.numVertices(); ++u) {
        for (const auto& entry : graph.getEdgesFrom(u)) {
            int v = entry.vertex_id;
            if (v <= u) continue;

            visualization_msgs::Marker edge_marker;
            edge_marker.header.frame_id = map_frame_;
            edge_marker.header.stamp = now;
            edge_marker.ns = "edges";
            edge_marker.id = marker_id++;
            edge_marker.type = visualization_msgs::Marker::LINE_STRIP;
            edge_marker.action = visualization_msgs::Marker::ADD;
            edge_marker.scale.x = 0.1;
            edge_marker.color.r = 0.0; edge_marker.color.g = 0.5;
            edge_marker.color.b = 1.0; edge_marker.color.a = 0.8;

            geometry_msgs::Point p1, p2;
            p1.x = graph.getVertex(u).pose_for_visualization[0];
            p1.y = graph.getVertex(u).pose_for_visualization[1];
            p2.x = graph.getVertex(v).pose_for_visualization[0];
            p2.y = graph.getVertex(v).pose_for_visualization[1];
            edge_marker.points.push_back(p1);
            edge_marker.points.push_back(p2);

            marker_array.markers.push_back(edge_marker);
        }
    }

    graph_viz_pub_.publish(marker_array);
}

// ============================================================================
// publishLastVertex
// ============================================================================
void ResultsPublisher::publishLastVertex(const Vertex& last_vertex,
                                          int last_vertex_id,
                                          const ros::Time& stamp) {
    auto msg = gridToOccMsg(last_vertex.grid, stamp);
    last_vertex_pub_.publish(msg);

    std_msgs::Int32 id_msg;
    id_msg.data = last_vertex_id;
    last_vertex_id_pub_.publish(id_msg);
}

// ============================================================================
// publishRelPose
// ============================================================================
void ResultsPublisher::publishRelPose(const Pose2D& rel_pose,
                                       const ros::Time& stamp) {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.pose.position.x = rel_pose[0];
    msg.pose.position.y = rel_pose[1];
    msg.pose.position.z = 0.0;
    msg.pose.orientation = tf::createQuaternionMsgFromYaw(rel_pose[2]);
    rel_pose_pub_.publish(msg);
}

// ============================================================================
// publishCurGrid / publishLastVertexGrid
// ============================================================================
void ResultsPublisher::publishCurGrid(const LocalGrid& grid,
                                        const ros::Time& stamp) {
    auto msg = gridToOccMsg(grid, stamp);
    cur_grid_pub_.publish(msg);
}

void ResultsPublisher::publishLastVertexGrid(const LocalGrid& grid,
                                              const ros::Time& stamp) {
    auto msg = gridToOccMsg(grid, stamp);
    local_grid_pub_.publish(msg);
}

// ============================================================================
// publishLocalizationResults
// Matches original Python: publishes single Marker with type=POINTS
// ============================================================================
void ResultsPublisher::publishLocalizationResults(
    const TopologicalGraph& graph,
    const std::vector<int>& matched_ids,
    const std::vector<Pose2D>& /*rel_poses*/,
    const std::vector<int>& unmatched_ids) {

    ros::Time now = ros::Time::now();

    // Matched points - single Marker with POINTS type (same as original Python)
    visualization_msgs::Marker matched_marker;
    matched_marker.header.frame_id = map_frame_;
    matched_marker.header.stamp = now;
    matched_marker.type = visualization_msgs::Marker::POINTS;
    matched_marker.id = 0;
    matched_marker.action = visualization_msgs::Marker::ADD;
    matched_marker.scale.x = 0.5;
    matched_marker.scale.y = 0.5;
    matched_marker.scale.z = 0.5;
    matched_marker.color.r = 0.0;
    matched_marker.color.g = 1.0;
    matched_marker.color.b = 0.0;
    matched_marker.color.a = 1.0;
    matched_marker.pose.orientation.w = 1.0;

    for (int vid : matched_ids) {
        if (vid < 0 || vid >= graph.numVertices()) continue;
        const auto& v = graph.getVertex(vid);
        geometry_msgs::Point p;
        p.x = v.pose_for_visualization[0];
        p.y = v.pose_for_visualization[1];
        p.z = 0.1;
        matched_marker.points.push_back(p);
    }
    matched_points_pub_.publish(matched_marker);

    // Unmatched points - single Marker with POINTS type
    visualization_msgs::Marker unmatched_marker;
    unmatched_marker.header.frame_id = map_frame_;
    unmatched_marker.header.stamp = now;
    unmatched_marker.type = visualization_msgs::Marker::POINTS;
    unmatched_marker.id = 0;
    unmatched_marker.action = visualization_msgs::Marker::ADD;
    unmatched_marker.scale.x = 0.4;
    unmatched_marker.scale.y = 0.4;
    unmatched_marker.scale.z = 0.4;
    unmatched_marker.color.r = 0.5;
    unmatched_marker.color.g = 0.5;
    unmatched_marker.color.b = 0.5;
    unmatched_marker.color.a = 0.7;
    unmatched_marker.pose.orientation.w = 1.0;

    for (int vid : unmatched_ids) {
        if (vid < 0 || vid >= graph.numVertices()) continue;
        const auto& v = graph.getVertex(vid);
        geometry_msgs::Point p;
        p.x = v.pose_for_visualization[0];
        p.y = v.pose_for_visualization[1];
        p.z = 0.1;
        unmatched_marker.points.push_back(p);
    }
    unmatched_points_pub_.publish(unmatched_marker);
}

// ============================================================================
// publishLoopClosureResults
// ============================================================================
void ResultsPublisher::publishLoopClosureResults(
    const TopologicalGraph& graph,
    const std::vector<int>& path,
    const Pose2D& global_pose) {

    visualization_msgs::MarkerArray markers;
    ros::Time now = ros::Time::now();
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(del);

    int id = 0;

    // 路径线
    if (path.size() >= 2) {
        visualization_msgs::Marker line;
        line.header.frame_id = map_frame_;
        line.header.stamp = now;
        line.ns = "loop_closure_path";
        line.id = id++;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        line.scale.x = 0.3;
        line.color.r = 1.0; line.color.g = 0.0;
        line.color.b = 1.0; line.color.a = 1.0;  // 紫色

        for (int vid : path) {
            geometry_msgs::Point p;
            p.x = graph.getVertex(vid).pose_for_visualization[0];
            p.y = graph.getVertex(vid).pose_for_visualization[1];
            line.points.push_back(p);
        }
        markers.markers.push_back(line);
    }

    loop_closure_pub_.publish(markers);
}

// ============================================================================
// publishTopologicalPath
// ============================================================================
void ResultsPublisher::publishTopologicalPath(
    const TopologicalGraph& graph,
    const std::vector<int>& path,
    const ros::Time& stamp) {

    toposlam_msgs::TopologicalPath msg;
    // TopologicalPath 消息格式取决于 toposlam_msgs 的定义
    // 假设包含 vertex_ids 字段
    // msg.vertex_ids = path;
    path_pub_.publish(msg);

    // 路径可视化
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(del);

    if (path.size() >= 2) {
        visualization_msgs::Marker line;
        line.header.frame_id = map_frame_;
        line.header.stamp = stamp;
        line.ns = "nav_path";
        line.id = 0;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        line.scale.x = 0.15;
        line.color.r = 0.0; line.color.g = 1.0;
        line.color.b = 1.0; line.color.a = 1.0;  // 青色

        for (int vid : path) {
            geometry_msgs::Point p;
            p.x = graph.getVertex(vid).pose_for_visualization[0];
            p.y = graph.getVertex(vid).pose_for_visualization[1];
            line.points.push_back(p);
        }
        markers.markers.push_back(line);
    }
    path_marker_pub_.publish(markers);
}

// ============================================================================
// publishSubgoal
// ============================================================================
void ResultsPublisher::publishSubgoal(const Pose2D& subgoal,
                                        const ros::Time& stamp) {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.pose.position.x = subgoal[0];
    msg.pose.position.y = subgoal[1];
    msg.pose.orientation = tf::createQuaternionMsgFromYaw(subgoal[2]);
    pointgoal_pub_.publish(msg);
}

// ============================================================================
// freeze / unfreeze
// ============================================================================
void ResultsPublisher::freeze() {
    std_msgs::Bool msg;
    msg.data = true;
    freeze_pub_.publish(msg);
}

void ResultsPublisher::unfreeze() {
    std_msgs::Bool msg;
    msg.data = false;
    freeze_pub_.publish(msg);
}

// ============================================================================
// publishTfFromOdom
// ============================================================================
void ResultsPublisher::publishTfFromOdom(double x, double y, double theta,
                                          const ros::Time& stamp,
                                          const std::string& odom_frame) {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    transform.setRotation(q);

    tf_broadcaster_.sendTransform(
        tf::StampedTransform(transform, stamp, map_frame_, odom_frame));
}

} // namespace prism_topomap
