/**
 * @file utils.h
 * @brief Pose utilities and PCL-based point cloud processing
 *
 * Corresponds to original Python: scripts/utils.py
 * All 2D poses represented as Eigen::Vector3d [x, y, theta]
 * All point clouds stored as pcl::PointCloud<pcl::PointXYZ>::Ptr
 */
#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <string>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace prism_topomap {

// ============================================================================
// Type definitions
// ============================================================================

/// 2D pose: [x, y, theta]
using Pose2D = Eigen::Vector3d;

/// PCL point cloud types
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloudXYZ::Ptr;

/// Runtime-only controls for unified [FLOW] trace logging.
/// These fields never participate in SLAM decisions.
struct FlowTraceConfig {
    bool enabled = false;
    int every_n_processed_frames = 1;
    int descriptor_head_size = 4;
    bool registration_candidates = true;
};

/// Optional diagnostic counters produced while parsing one PointCloud2.
struct CloudParseStats {
    size_t raw_points = 0;
    size_t parsed_points = 0;
    size_t finite_points = 0;
    size_t invalid_points = 0;
    bool rotation_applied = false;
    bool has_sample = false;
    pcl::PointXYZ sample_before;
    pcl::PointXYZ sample_after;
};

// ============================================================================
// Angle utilities
// ============================================================================

/**
 * @brief Normalize angle to [-π, π]
 */
double normalize(double angle);

// ============================================================================
// 2D rotation
// ============================================================================

/**
 * @brief Rotate (x, y) around origin by angle radians
 * @return rotated (x_new, y_new)
 */
Eigen::Vector2d rotate2D(double x, double y, double angle);

// ============================================================================
// Pose operations
// ============================================================================

/**
 * @brief Compute relative pose from 'from' to 'to'
 */
Pose2D getRelPose(const Pose2D& from, const Pose2D& to);

/**
 * @brief Apply relative shift to a pose
 */
Pose2D applyPoseShift(const Pose2D& pose, const Pose2D& shift);

// ============================================================================
// Point cloud processing (PCL-based)
// ============================================================================

/**
 * @brief Apply 3x3 rotation matrix to point cloud xyz
 */
PointCloudPtr rotatePcd(const PointCloudPtr& points,
                        const Eigen::Matrix3f& rotation_matrix);

/**
 * @brief Apply 2D transform (x, y, theta) to point cloud
 */
PointCloudPtr transformPcd(const PointCloudPtr& points,
                           double x, double y, double theta);

/**
 * @brief Extract xyz from ROS PointCloud2 message and apply rotation
 *
 * @param msg      ROS PointCloud2 message
 * @param fields   "xyz" or "xyzrgb" (only xyz coords are kept)
 * @param rotation 3x3 rotation matrix applied to xyz
 * @return PCL point cloud pointer
 */
PointCloudPtr getXyzCoordsFromMsg(const sensor_msgs::PointCloud2& msg,
                                  const std::string& fields,
                                  const Eigen::Matrix3f& rotation,
                                  CloudParseStats* stats = nullptr);

/**
 * @brief Remove floor and ceiling points.
 *
 * Supports 'auto' mode: pass NaN for floor_height or ceil_height to
 * enable automatic detection via Z-axis histogram (matching original Python).
 *
 * @param cloud        Input point cloud
 * @param floor_height Floor threshold (points below removed). NaN = auto.
 * @param ceil_height  Ceiling threshold (points above removed). NaN = auto.
 * @return Filtered point cloud
 */
PointCloudPtr removeFloorAndCeil(const PointCloudPtr& cloud,
                                 float floor_height,
                                 float ceil_height);

/**
 * @brief VoxelGrid downsampling
 *
 * @param cloud     Input point cloud
 * @param leaf_size Voxel leaf size in meters
 * @return Downsampled point cloud
 */
PointCloudPtr voxelDownsample(const PointCloudPtr& cloud, float leaf_size);

/**
 * @brief Rotate point cloud around X axis (vertical rotation)
 */
PointCloudPtr rotateVertical(const PointCloudPtr& cloud, double angle);

} // namespace prism_topomap
