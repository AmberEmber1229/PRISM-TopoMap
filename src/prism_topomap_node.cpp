/**
 * @file prism_topomap_node.cpp
 * @brief ROS C++ 主节点实现 + main() 入口
 *
 * 逐方法对应原 Python prism_topomap_node.py 中的 PRISMTopomapNode 类
 */
#include "prism_topomap/prism_topomap_node.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>

namespace prism_topomap {

// ============================================================================
// 构造函数
// ============================================================================
PRISMTopomapNode::PRISMTopomapNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {

    // ================================================================
    // 1. 加载配置文件
    // ================================================================
    std::string config_file;
    pnh_.param<std::string>("config_file", config_file, "scout_rosbag.yaml");
    std::string pkg_path = ros::package::getPath("prism_topomap");
    std::string config_path = pkg_path + "/config/" + config_file;
    ROS_INFO("Loading config: %s", config_path.c_str());
    config_ = YAML::LoadFile(config_path);

    // ================================================================
    // 2. Read config (matching nested YAML structure)
    // ================================================================
    // --- input.pointcloud ---
    auto input_config = config_["input"];
    auto pointcloud_config = input_config["pointcloud"];
    pcd_topic_ = pointcloud_config["topic"].as<std::string>("/rslidar_points");
    pcd_fields_ = pointcloud_config["fields"].as<std::string>("xyz");
    subscribe_to_curbs_ = pointcloud_config["subscribe_to_curbs"].as<bool>(false);

    // Point cloud rotation matrix
    pcd_rotation_ = Eigen::Matrix3f::Identity();
    if (pointcloud_config["rotation_matrix"]) {
        auto rot = pointcloud_config["rotation_matrix"];
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                pcd_rotation_(r, c) = rot[r][c].as<float>();
            }
        }
    }

    // --- input.odometry ---
    auto odometry_config = input_config["odometry"];
    odom_topic_ = odometry_config["topic"].as<std::string>("/odom");
    use_odom_ = odometry_config["use_odom"].as<bool>(true);

    // --- input.gt_pose ---
    use_gt_pose_ = input_config["subscribe_to_gt_pose"].as<bool>(true);
    if (use_gt_pose_ && input_config["gt_pose"]) {
        auto gt_pose_config = input_config["gt_pose"];
        gt_topic_ = gt_pose_config["topic"].as<std::string>("/odom_gt");
        std::string gt_type = gt_pose_config["type"].as<std::string>("Odometry");
        use_gt_pose_pose_stamped_ = (gt_type == "PoseStamped");
    } else {
        gt_topic_.clear();
        use_gt_pose_pose_stamped_ = false;
    }

    // --- input.subscribe_to_images ---
    subscribe_to_images_ = input_config["subscribe_to_images"].as<bool>(false);
    if (subscribe_to_images_) {
        if (input_config["image_front"]) {
            image_front_topic_ = input_config["image_front"]["topic"].as<std::string>("/image_front");
        }
        if (input_config["image_back"]) {
            image_back_topic_ = input_config["image_back"]["topic"].as<std::string>("");
        }
    }

    // --- visualization ---
    auto viz_config = config_["visualization"];
    map_frame_ = viz_config["map_frame"].as<std::string>("map");
    publish_tf_from_odom_ = viz_config["publish_tf_from_odom"].as<bool>(false);

    // --- curb detection topic ---
    if (subscribe_to_curbs_ && pointcloud_config["curb_detection_topic"]) {
        curb_topic_ = pointcloud_config["curb_detection_topic"].as<std::string>("/curb_detection");
    }

    // ================================================================
    // 3. 创建推理客户端
    // ================================================================
    inference_client_ = std::make_shared<InferenceClient>(nh_);

    // ================================================================
    // 4. 创建核心算法
    // ================================================================
    std::string path_to_load_graph;
    std::string path_to_save_graph;
    std::string path_to_save_logs;
    pnh_.param<std::string>("path_to_load_graph", path_to_load_graph, "");
    pnh_.param<std::string>("path_to_save_graph", path_to_save_graph, "");
    pnh_.param<std::string>("path_to_save_logs", path_to_save_logs, "");

    topo_slam_model_ = std::make_unique<TopoSLAMModel>(
        config_, inference_client_,
        path_to_load_graph, path_to_save_graph, path_to_save_logs);

    // ================================================================
    // 5. 创建可视化发布器
    // ================================================================
    results_publisher_ = std::make_unique<ResultsPublisher>(nh_, map_frame_);

    // ================================================================
    // 6. 设置 ROS 订阅
    // ================================================================
    pcd_sub_ = nh_.subscribe(pcd_topic_, 10,
                             &PRISMTopomapNode::pcdCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 100,
                              &PRISMTopomapNode::odomCallback, this);

    if (use_gt_pose_) {
        if (use_gt_pose_pose_stamped_) {
            gt_pose_sub_ = nh_.subscribe(gt_topic_, 100,
                                         &PRISMTopomapNode::gtPoseCallback, this);
        } else {
            gt_pose_sub_ = nh_.subscribe(gt_topic_, 100,
                                         &PRISMTopomapNode::gtOdomPoseCallback, this);
        }
    }

    if (subscribe_to_images_) {
        front_image_sub_ = nh_.subscribe(image_front_topic_, 10,
                                         &PRISMTopomapNode::frontImageCallback, this);
        if (!image_back_topic_.empty()) {
            back_image_sub_ = nh_.subscribe(image_back_topic_, 10,
                                             &PRISMTopomapNode::backImageCallback, this);
        }
    }

    if (subscribe_to_curbs_) {
        curb_sub_ = nh_.subscribe(curb_topic_, 10,
                                  &PRISMTopomapNode::curbDetectionCallback, this);
    }

    // 导航目标订阅
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1,
                              &PRISMTopomapNode::goalCallback, this);

    // ================================================================
    // 7. 定时器: 异步定位
    // ================================================================
    double loc_freq = topo_slam_model_->localizationFrequency();
    if (loc_freq > 0) {
        localize_timer_ = nh_.createTimer(ros::Duration(1.0 / loc_freq),
                                          &PRISMTopomapNode::localizeTimerCallback, this);
    }

    ROS_INFO("=== PRISM-TopoMap C++ node initialized ===");
    ROS_INFO("  Mode: %s", topo_slam_model_->mode().c_str());
    ROS_INFO("  PCD topic: %s", pcd_topic_.c_str());
    ROS_INFO("  Odom topic: %s", odom_topic_.c_str());
    ROS_INFO("  use_odom for rel_pose: %s", use_odom_ ? "true" : "false");
    ROS_INFO("  use_gt_pose: %s", use_gt_pose_ ? "true" : "false");
    if (use_gt_pose_) {
        ROS_INFO("  GT topic: %s", gt_topic_.c_str());
        ROS_INFO("  GT type: %s", use_gt_pose_pose_stamped_ ? "PoseStamped" : "Odometry");
    }
}

// ============================================================================
// 传感器回调
// ============================================================================

void PRISMTopomapNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO_ONCE("[DIAG] First odom msg received on topic: %s", odom_topic_.c_str());
    double stamp = msg->header.stamp.toSec();
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    tf::Quaternion q(msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
    double theta = tf::getYaw(q);
    odom_poses_.push_back({stamp, Pose2D(x, y, theta)});

    // 可选: 从里程计发布 TF
    if (publish_tf_from_odom_) {
        results_publisher_->publishTfFromOdom(x, y, theta, msg->header.stamp);
    }
    processPcdQueue();
}

void PRISMTopomapNode::gtPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO_ONCE("[DIAG] First GT PoseStamped msg received on topic: %s", gt_topic_.c_str());
    double stamp = msg->header.stamp.toSec();
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    tf::Quaternion q(msg->pose.orientation.x,
                     msg->pose.orientation.y,
                     msg->pose.orientation.z,
                     msg->pose.orientation.w);
    double theta = tf::getYaw(q);
    gt_poses_.push_back({stamp, Pose2D(x, y, theta)});
    processPcdQueue();
}

void PRISMTopomapNode::gtOdomPoseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO_ONCE("[DIAG] First GT Odometry msg received on topic: %s", gt_topic_.c_str());
    double stamp = msg->header.stamp.toSec();
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    tf::Quaternion q(msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
    double theta = tf::getYaw(q);
    gt_poses_.push_back({stamp, Pose2D(x, y, theta)});
    processPcdQueue();
}

void PRISMTopomapNode::frontImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    double stamp = msg->header.stamp.toSec();
    rgb_buffer_front_.push_back({stamp, *msg});
    if (rgb_buffer_front_.size() > 100) rgb_buffer_front_.pop_front();
}

void PRISMTopomapNode::backImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    double stamp = msg->header.stamp.toSec();
    rgb_buffer_back_.push_back({stamp, *msg});
    if (rgb_buffer_back_.size() > 100) rgb_buffer_back_.pop_front();
}

void PRISMTopomapNode::curbDetectionCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    double stamp = msg->header.stamp.toSec();
    PointCloudPtr cloud(new PointCloudXYZ);
    pcl::fromROSMsg(*msg, *cloud);
    curb_clouds_.push_back({stamp, cloud});
}

void PRISMTopomapNode::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    has_metric_goal_ = true;
    metric_goal_x_ = msg->pose.position.x;
    metric_goal_y_ = msg->pose.position.y;
    ROS_INFO("Received navigation goal: (%.1f, %.1f)", metric_goal_x_, metric_goal_y_);
}

// ============================================================================
// localizeTimerCallback: 定时异步定位
// ============================================================================
void PRISMTopomapNode::localizeTimerCallback(const ros::TimerEvent& /*event*/) {
    topo_slam_model_->localizer().localize();
}

// ============================================================================
// interpolatePose: 线性插值位姿
// 对应 Python: interpolate_pose()
// ============================================================================
Pose2D PRISMTopomapNode::interpolatePose(const StampedPose& left,
                                          const StampedPose& right,
                                          double timestamp) {
    double dt = right.timestamp - left.timestamp;
    if (dt < 1e-9) return left.pose;

    double t = (timestamp - left.timestamp) / dt;
    t = std::max(0.0, std::min(1.0, t));

    return Pose2D(
        left.pose[0] + t * (right.pose[0] - left.pose[0]),
        left.pose[1] + t * (right.pose[1] - left.pose[1]),
        left.pose[2] + t * normalize(right.pose[2] - left.pose[2])
    );
}

// ============================================================================
// getSyncPoseAndImages: 时间同步
// 对应 Python: get_sync_pose_and_images()
// ============================================================================
PRISMTopomapNode::SyncResult PRISMTopomapNode::getSyncPoseAndImages(double timestamp) {
    SyncResult result;
    result.valid = false;
    result.has_img_front = false;
    result.has_img_back = false;
    result.has_curbs = false;

    // 查找 GT 位姿 (插值)
    if (true) {
        const double kPoseSyncTolerance = 0.2;

        auto getNearestPose = [timestamp](const std::vector<StampedPose>& poses,
                                          Pose2D& out_pose,
                                          double& best_diff) -> bool {
            if (poses.empty()) return false;
            int best_idx = 0;
            best_diff = std::abs(poses[0].timestamp - timestamp);
            for (int i = 1; i < static_cast<int>(poses.size()); ++i) {
                double diff = std::abs(poses[i].timestamp - timestamp);
                if (diff < best_diff) {
                    best_diff = diff;
                    best_idx = i;
                }
            }
            out_pose = poses[best_idx].pose;
            return true;
        };

        if (use_gt_pose_) {
            if (gt_poses_.empty()) return result;

            if (gt_poses_.size() == 1) {
                double gt_diff = std::abs(gt_poses_[0].timestamp - timestamp);
                if (gt_diff > kPoseSyncTolerance) return result;
                result.global_pose = gt_poses_[0].pose;
            } else {
                int gt_idx = -1;
                for (int i = 0; i < static_cast<int>(gt_poses_.size()) - 1; ++i) {
                    if (gt_poses_[i].timestamp <= timestamp && gt_poses_[i + 1].timestamp >= timestamp) {
                        gt_idx = i;
                        break;
                    }
                }

                if (gt_idx >= 0) {
                    result.global_pose = interpolatePose(gt_poses_[gt_idx], gt_poses_[gt_idx + 1], timestamp);
                } else {
                    double gt_diff = std::numeric_limits<double>::max();
                    if (!getNearestPose(gt_poses_, result.global_pose, gt_diff) || gt_diff > kPoseSyncTolerance) {
                        return result;
                    }
                }
            }
        } else {
            double odom_diff = std::numeric_limits<double>::max();
            if (!getNearestPose(odom_poses_, result.global_pose, odom_diff) || odom_diff > kPoseSyncTolerance) {
                return result;
            }
        }

        if (use_odom_) {
            double odom_diff = std::numeric_limits<double>::max();
            if (!getNearestPose(odom_poses_, result.odom_pose, odom_diff) || odom_diff > kPoseSyncTolerance) {
                return result;
            }
        } else {
            result.odom_pose = result.global_pose;
        }

        result.valid = true;
    } /* legacy sync logic retained for reference:
        if (gt_poses_.size() < 2) return result;

    // 找到最近的两个 GT 位姿
    int idx = -1;
    for (int i = 0; i < static_cast<int>(gt_poses_.size()) - 1; ++i) {
        if (gt_poses_[i].timestamp <= timestamp && gt_poses_[i + 1].timestamp >= timestamp) {
            idx = i;
            break;
        }
    }

    if (idx < 0) {
        // 使用最近的位姿
        double best_diff = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(gt_poses_.size()); ++i) {
            double diff = std::abs(gt_poses_[i].timestamp - timestamp);
            if (diff < best_diff) {
                best_diff = diff;
                idx = i;
            }
        }
        if (idx >= 0 && best_diff < 1.0) {
            result.global_pose = gt_poses_[idx].pose;
        } else {
            return result;
        }
    } else {
        result.global_pose = interpolatePose(gt_poses_[idx], gt_poses_[idx + 1], timestamp);
    }

    // 查找里程计位姿 (最近的)
    if (!odom_poses_.empty()) {
        double best_diff = std::numeric_limits<double>::max();
        int best_idx = 0;
        for (int i = 0; i < static_cast<int>(odom_poses_.size()); ++i) {
            double diff = std::abs(odom_poses_[i].timestamp - timestamp);
            if (diff < best_diff) {
                best_diff = diff;
                best_idx = i;
            }
        }
        result.odom_pose = odom_poses_[best_idx].pose;
    }

    result.valid = true;
    } */

    // 匹配前视图像
    if (subscribe_to_images_ && !rgb_buffer_front_.empty()) {
        double best_diff = std::numeric_limits<double>::max();
        int best_idx = -1;
        for (int i = 0; i < static_cast<int>(rgb_buffer_front_.size()); ++i) {
            double diff = std::abs(rgb_buffer_front_[i].timestamp - timestamp);
            if (diff < best_diff) {
                best_diff = diff;
                best_idx = i;
            }
        }
        if (best_idx >= 0 && best_diff < 0.5) {
            result.has_img_front = true;
            result.img_front = rgb_buffer_front_[best_idx].image;
        }
    }

    // 匹配后视图像
    if (subscribe_to_images_ && !rgb_buffer_back_.empty()) {
        double best_diff = std::numeric_limits<double>::max();
        int best_idx = -1;
        for (int i = 0; i < static_cast<int>(rgb_buffer_back_.size()); ++i) {
            double diff = std::abs(rgb_buffer_back_[i].timestamp - timestamp);
            if (diff < best_diff) {
                best_diff = diff;
                best_idx = i;
            }
        }
        if (best_idx >= 0 && best_diff < 0.5) {
            result.has_img_back = true;
            result.img_back = rgb_buffer_back_[best_idx].image;
        }
    }

    // 匹配路沿点云
    if (subscribe_to_curbs_ && !curb_clouds_.empty()) {
        double best_diff = std::numeric_limits<double>::max();
        int best_idx = -1;
        for (int i = 0; i < static_cast<int>(curb_clouds_.size()); ++i) {
            double diff = std::abs(curb_clouds_[i].timestamp - timestamp);
            if (diff < best_diff) {
                best_diff = diff;
                best_idx = i;
            }
        }
        if (best_idx >= 0 && best_diff < 0.5) {
            result.has_curbs = true;
            result.curbs = curb_clouds_[best_idx].cloud;
        }
    }

    return result;
}

// ============================================================================
// getNavigationSubgoal: 获取导航子目标
// 对应 Python: get_navigation_subgoal()
// ============================================================================
Pose2D PRISMTopomapNode::getNavigationSubgoal() {
    if (!has_metric_goal_ || topo_slam_model_->lastVertexId() < 0) {
        return Pose2D::Zero();
    }

    path_to_goal_ = topo_slam_model_->getPathToMetricGoal(metric_goal_x_, metric_goal_y_);

    if (path_to_goal_.size() < 2) {
        // 已经在目标节点, 直接返回目标位置
        return Pose2D(metric_goal_x_, metric_goal_y_, 0.0);
    }

    // 下一个节点的位置作为子目标
    int next_vid = path_to_goal_[1];
    const auto& v = topo_slam_model_->graph().getVertex(next_vid);
    return v.pose_for_visualization;
}

// ============================================================================
// pcdCallback: 点云回调 (主入口)
// 对应 Python: PRISMTopomapNode.pcd_callback()
// ============================================================================
void PRISMTopomapNode::pcdCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ROS_INFO_ONCE("[DIAG] First PCD msg received on topic: %s", pcd_topic_.c_str());
    pcd_queue_.push_back(msg);
    if (pcd_queue_.size() > 50) {
        pcd_queue_.pop_front();
    }
    processPcdQueue();
}

// ============================================================================
// processPcdQueue: 统一消费点云缓冲队列 (非阻塞同步)
// ============================================================================
void PRISMTopomapNode::processPcdQueue() {
    while (!pcd_queue_.empty()) {
        auto msg = pcd_queue_.front();
        double stamp = msg->header.stamp.toSec();

        // 对齐 Python 逻辑：防堆积丢弃过旧数据，修复时间戳对不齐的问题
        double dt = ros::Time::now().toSec() - stamp;
        if (dt > 0.5) {
            ROS_WARN_THROTTLE(2.0, "[DIAG] Dropping stale Lidar frame (dt=%.3f sec > 0.5). Stamp: %.3f, Now: %.3f", 
                              dt, stamp, ros::Time::now().toSec());
            pcd_queue_.pop_front();
            continue;
        } else if (dt < -1.0) {
            // 时间戳跳变或者严重超前，静默抛弃或打印
            ROS_WARN_THROTTLE(2.0, "[DIAG] Frame in the future (dt=%.3f sec). Stamp: %.3f, Now: %.3f", 
                              dt, stamp, ros::Time::now().toSec());
        }

        // 1. 时间同步
        auto sync = getSyncPoseAndImages(stamp);
        if (!sync.valid) {
            // 没有获得匹配结果 (通常是因为里程计由于处理延迟或者还没启动，仍未抵达)
            // 退出循环，保留该帧在队列里等待未来的里程计回调触发消费
            ROS_WARN_THROTTLE(5.0, "Waiting for pose data to catch up... gt_poses=%lu, odom_poses=%lu",
                              gt_poses_.size(), odom_poses_.size());
            break;
        }

        // 成功获取同步数据，弹出队列并开始处理
        pcd_queue_.pop_front();
        frame_cnt_++;

        // 2. Parse point cloud
        PointCloudPtr cur_cloud = getXyzCoordsFromMsg(*msg, pcd_fields_, pcd_rotation_);
        if (!cur_cloud || cur_cloud->empty()) {
            ROS_WARN("Empty pointcloud, skipping");
            continue; 
        }

        // 3. Set timestamp
        topo_slam_model_->setCurrentStamp(stamp);

        // 4. Call core algorithm
        PointCloudPtr curbs_ptr = sync.has_curbs ? sync.curbs : PointCloudPtr();

        topo_slam_model_->update(
            sync.global_pose,
            sync.odom_pose,
            *msg,
            cur_cloud,
            sync.has_img_front,
            sync.has_img_back,
            sync.img_front,
            sync.img_back,
            curbs_ptr
        );

        // 5. 发布可视化
        ros::Time ros_stamp = msg->header.stamp;
        results_publisher_->publishGraph(topo_slam_model_->graph(),
                                          topo_slam_model_->lastVertexId());

        if (topo_slam_model_->lastVertexId() >= 0) {
            results_publisher_->publishLastVertex(
                topo_slam_model_->lastVertex(),
                topo_slam_model_->lastVertexId(),
                ros_stamp);

            results_publisher_->publishRelPose(
                topo_slam_model_->relPoseOfVcur(), ros_stamp);

            results_publisher_->publishLastVertexGrid(
                topo_slam_model_->lastVertex().grid, ros_stamp);
        }

        results_publisher_->publishCurGrid(topo_slam_model_->curGrid(), ros_stamp);

        // 发布定位结果
        auto loc_state = topo_slam_model_->localizer().getLocalizedState();
        if (!loc_state.vertex_ids_matched.empty()) {
            results_publisher_->publishLocalizationResults(
                topo_slam_model_->graph(),
                loc_state.vertex_ids_matched,
                loc_state.rel_poses,
                loc_state.vertex_ids_unmatched);
        }

        // 发布回环
        if (topo_slam_model_->foundLoopClosure()) {
            results_publisher_->publishLoopClosureResults(
                topo_slam_model_->graph(),
                topo_slam_model_->loopClosurePath(),
                sync.global_pose);
        }

        // 发布导航路径
        if (has_metric_goal_) {
            Pose2D subgoal = getNavigationSubgoal();
            results_publisher_->publishSubgoal(subgoal, ros_stamp);
            if (!path_to_goal_.empty()) {
                results_publisher_->publishTopologicalPath(
                    topo_slam_model_->graph(), path_to_goal_, ros_stamp);
            }
        }

        ROS_INFO_THROTTLE(10.0, "Processed %d frames", frame_cnt_);
    }
}

// ============================================================================
// run
// ============================================================================
void PRISMTopomapNode::run() {
    // 等待 Python 推理服务
    if (!inference_client_->waitForServices(60.0)) {
        ROS_ERROR("Cannot connect to Python inference service! Make sure inference_service_node.py is running.");
        return;
    }

    ROS_INFO("=== PRISM-TopoMap C++ node running ===");
    ros::spin();
}

} // namespace prism_topomap

// ============================================================================
// main() 入口
// ============================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "prism_topomap_cpp_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    prism_topomap::PRISMTopomapNode node(nh, pnh);
    node.run();

    return 0;
}
