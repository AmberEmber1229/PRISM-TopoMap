/**
 * @file prism_topomap_node.h
 * @brief ROS C++ 主节点类
 *
 * 对应原 Python 文件: scripts/prism_topomap_node.py → PRISMTopomapNode
 * 负责: ROS 订阅/发布, 时间同步, 数据缓冲, 调用核心算法
 */
#pragma once

#include "prism_topomap/topo_slam_model.h"
#include "prism_topomap/results_publisher.h"
#include "prism_topomap/inference_client.h"
#include "prism_topomap/utils.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <deque>
#include <vector>
#include <memory>
#include <string>
#include <cstdint>

namespace prism_topomap {

class PRISMTopomapNode {
public:
    PRISMTopomapNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    void run();

private:
    // === ROS 回调 ===
    void pcdCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void processPcdQueue();
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void gtPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void gtOdomPoseCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void frontImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void backImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void curbDetectionCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    // === 定时器回调 ===
    void localizeTimerCallback(const ros::TimerEvent& event);

    // === 时间同步 ===
    struct StampedPose {
        double timestamp;
        Pose2D pose;
    };
    struct StampedImage {
        double timestamp;
        sensor_msgs::Image image;
    };
    struct StampedCloud {
        double timestamp;
        PointCloudPtr cloud;
    };

    struct SyncResult {
        bool valid;
        Pose2D global_pose;
        Pose2D odom_pose;
        bool has_img_front;
        bool has_img_back;
        sensor_msgs::Image img_front;
        sensor_msgs::Image img_back;
        bool has_curbs;
        PointCloudPtr curbs;
        std::string global_source;
        std::string odom_source;
        std::string failure_reason;
        double dt_gt;
        double dt_odom;
        double dt_front;
        double dt_back;
        double dt_curbs;
        double latest_gt_stamp;
        double latest_odom_stamp;
        double latest_front_stamp;
        double latest_back_stamp;
        double latest_curbs_stamp;
    };

    SyncResult getSyncPoseAndImages(double timestamp);
    Pose2D interpolatePose(const StampedPose& left, const StampedPose& right,
                           double timestamp);

    // === 导航 ===
    Pose2D getNavigationSubgoal();

    // === 成员变量 ===
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // 核心组件
    std::shared_ptr<InferenceClient> inference_client_;
    std::unique_ptr<TopoSLAMModel> topo_slam_model_;
    std::unique_ptr<ResultsPublisher> results_publisher_;

    // ROS 订阅/发布/定时器
    ros::Subscriber pcd_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber gt_pose_sub_;
    ros::Subscriber front_image_sub_;
    ros::Subscriber back_image_sub_;
    ros::Subscriber curb_sub_;
    ros::Subscriber goal_sub_;
    ros::Timer localize_timer_;
    ros::Timer rel_pose_timer_;

    // 传感器数据缓冲区
    std::vector<StampedPose> gt_poses_;
    std::vector<StampedPose> odom_poses_;
    std::deque<sensor_msgs::PointCloud2::ConstPtr> pcd_queue_;
    std::deque<StampedImage> rgb_buffer_front_;
    std::deque<StampedImage> rgb_buffer_back_;
    std::vector<StampedCloud> curb_clouds_;

    // 配置参数
    YAML::Node config_;
    std::string pcd_topic_;
    std::string odom_topic_;
    std::string image_front_topic_;
    std::string image_back_topic_;
    std::string curb_topic_;
    std::string gt_topic_;
    std::string pcd_fields_;
    Eigen::Matrix3f pcd_rotation_;
    bool subscribe_to_images_ = false;
    bool subscribe_to_curbs_ = false;
    bool use_gt_pose_ = true;
    bool use_odom_ = false;
    bool use_gt_pose_pose_stamped_ = false;
    std::string map_frame_;
    bool publish_tf_from_odom_ = false;
    // 0 disables algorithm-level point-cloud sampling. Positive values are
    // an explicit performance mode and change the algorithm input sequence.
    double pcd_process_interval_ = 0.0;
    FlowTraceConfig trace_config_;

    // 导航
    bool has_metric_goal_ = false;
    double metric_goal_x_ = 0.0, metric_goal_y_ = 0.0;
    std::vector<int> path_to_goal_;

    // 状态
    int frame_cnt_ = 0;
    uint64_t pcd_rx_count_ = 0;
};

} // namespace prism_topomap
