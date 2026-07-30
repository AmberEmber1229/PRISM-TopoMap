/**
 * @file utils.cpp
 * @brief Pose utilities and PCL-based point cloud processing
 *
 * Migrated from Eigen::MatrixXf to pcl::PointCloud<pcl::PointXYZ>::Ptr
 * for full PCL ecosystem compatibility (VoxelGrid, PassThrough, etc.)
 */
#include "prism_topomap/utils.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include <ros/ros.h>

namespace prism_topomap {

// ============================================================================
// normalize
// ============================================================================
double normalize(double angle) {
    while (angle < -M_PI) angle += 2.0 * M_PI;
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    return angle;
}

// ============================================================================
// rotate2D
// ============================================================================
Eigen::Vector2d rotate2D(double x, double y, double angle) {
    double cos_a = std::cos(angle);
    double sin_a = std::sin(angle);
    return Eigen::Vector2d(
        x * cos_a + y * sin_a,
       -x * sin_a + y * cos_a
    );
}

// ============================================================================
// getRelPose
// ============================================================================
Pose2D getRelPose(const Pose2D& from, const Pose2D& to) {
    Eigen::Vector2d rotated = rotate2D(
        to[0] - from[0],
        to[1] - from[1],
        from[2]
    );
    return Pose2D(rotated[0], rotated[1], normalize(to[2] - from[2]));
}

// ============================================================================
// applyPoseShift
// ============================================================================
Pose2D applyPoseShift(const Pose2D& pose, const Pose2D& shift) {
    double x     = pose[0];
    double y     = pose[1];
    double theta = pose[2];
    double rel_x     = shift[0];
    double rel_y     = shift[1];
    double rel_theta  = shift[2];

    double cos_neg_theta = std::cos(-theta);
    double sin_neg_theta = std::sin(-theta);

    double new_x = x + rel_x * cos_neg_theta + rel_y * sin_neg_theta;
    double new_y = y - rel_x * sin_neg_theta + rel_y * cos_neg_theta;
    double new_theta = theta + rel_theta;

    return Pose2D(new_x, new_y, new_theta);
}

// ============================================================================
// rotatePcd: Apply 3x3 rotation to PCL point cloud
// ============================================================================
PointCloudPtr rotatePcd(const PointCloudPtr& points,
                        const Eigen::Matrix3f& rotation_matrix) {
    // Build 4x4 transform from 3x3 rotation
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.linear() = rotation_matrix;

    PointCloudPtr result(new PointCloudXYZ);
    pcl::transformPointCloud(*points, *result, transform);
    return result;
}

// ============================================================================
// transformPcd: Apply 2D transform (x, y, theta) to PCL point cloud
// ============================================================================
PointCloudPtr transformPcd(const PointCloudPtr& points,
                           double x, double y, double theta) {
    PointCloudPtr result(new PointCloudXYZ);
    result->resize(points->size());

    float cos_t = static_cast<float>(std::cos(theta));
    float sin_t = static_cast<float>(std::sin(theta));

    for (size_t i = 0; i < points->size(); ++i) {
        const auto& p = (*points)[i];
        auto& q = (*result)[i];
        q.x =  p.x * cos_t + p.y * sin_t + static_cast<float>(x);
        q.y = -p.x * sin_t + p.y * cos_t + static_cast<float>(y);
        q.z =  p.z;
    }
    return result;
}

// ============================================================================
// getXyzCoordsFromMsg: ROS PointCloud2 → PCL, with rotation
// Uses pcl::fromROSMsg for efficient conversion.
// ============================================================================
PointCloudPtr getXyzCoordsFromMsg(const sensor_msgs::PointCloud2& msg,
                                  const std::string& fields,
                                  const Eigen::Matrix3f& rotation,
                                  CloudParseStats* stats) {
    PointCloudPtr cloud(new PointCloudXYZ);

    // Convert ROS message to PCL (extracts xyz automatically)
    pcl::fromROSMsg(msg, *cloud);

    if (stats) {
        stats->raw_points = static_cast<size_t>(msg.width) * msg.height;
        stats->parsed_points = cloud->size();
        stats->rotation_applied = !rotation.isIdentity(1e-6f);
    }

    if (cloud->empty()) {
        return cloud;
    }

    // Remove NaN points
    PointCloudPtr clean(new PointCloudXYZ);
    clean->reserve(cloud->size());
    for (const auto& p : *cloud) {
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
            clean->push_back(p);
        }
    }

    if (stats) {
        stats->finite_points = clean->size();
        stats->invalid_points = cloud->size() - clean->size();
        if (!clean->empty()) {
            stats->has_sample = true;
            stats->sample_before = clean->front();
        }
    }

    // Apply sensor rotation matrix
    if (!rotation.isIdentity(1e-6f)) {
        clean = rotatePcd(clean, rotation);
    }

    if (stats && stats->has_sample && !clean->empty()) {
        stats->sample_after = clean->front();
    }

    return clean;
}

// ============================================================================
// removeFloorAndCeil: Remove ground and ceiling points
//
// Restores the original Python 'auto' mode:
//   heights = linspace(-4.0, 4.0, 41)
//   if floor_height == 'auto':
//       bins = [len(cloud[(z > h) & (z < heights[i+1])]) for ...]
//       floor_index = argmax(bins[:20]) + 1
//       floor_height = heights[floor_index]
//   if ceil_height == 'auto':
//       ceil_index = floor_index + 5 + argmax(bins[floor_index+5:])
//       ceil_height = heights[ceil_index]
//   return cloud[(z > floor_height) & (z < ceil_height)]
// ============================================================================
PointCloudPtr removeFloorAndCeil(const PointCloudPtr& cloud,
                                 float floor_height,
                                 float ceil_height) {
    if (!cloud || cloud->empty()) {
        return PointCloudPtr(new PointCloudXYZ);
    }

    // Auto mode: detect floor/ceiling from Z-axis histogram
    const int NUM_BINS = 40;
    const float Z_MIN = -4.0f;
    const float Z_MAX = 4.0f;
    const float BIN_WIDTH = (Z_MAX - Z_MIN) / NUM_BINS;

    bool auto_floor = std::isnan(floor_height);
    bool auto_ceil  = std::isnan(ceil_height);

    if (auto_floor || auto_ceil) {
        // Build Z histogram
        std::vector<int> bins(NUM_BINS, 0);
        for (const auto& p : *cloud) {
            if (!std::isfinite(p.z)) continue;
            int idx = static_cast<int>((p.z - Z_MIN) / BIN_WIDTH);
            if (idx >= 0 && idx < NUM_BINS) {
                bins[idx]++;
            }
        }

        int floor_index = 0;
        if (auto_floor) {
            // Find peak in first 20 bins (ground plane)
            int max_count = 0;
            for (int i = 0; i < std::min(20, NUM_BINS); ++i) {
                if (bins[i] > max_count) {
                    max_count = bins[i];
                    floor_index = i;
                }
            }
            floor_index += 1;  // One bin above the ground peak
            floor_height = Z_MIN + floor_index * BIN_WIDTH;
            ROS_DEBUG("Auto floor detected: %.2f m (bin %d)", floor_height, floor_index);
        }

        if (auto_ceil) {
            // Find floor_index if not already set
            if (!auto_floor) {
                floor_index = 0;
                while (floor_index < NUM_BINS - 6 &&
                       (Z_MIN + floor_index * BIN_WIDTH) < floor_height) {
                    floor_index++;
                }
            }
            // Find ceiling peak starting 5 bins above floor
            int start = floor_index + 5;
            int max_count = 0;
            int ceil_index = start;
            for (int i = start; i < NUM_BINS; ++i) {
                if (bins[i] > max_count) {
                    max_count = bins[i];
                    ceil_index = i;
                }
            }
            ceil_height = Z_MIN + ceil_index * BIN_WIDTH;
            ROS_DEBUG("Auto ceiling detected: %.2f m (bin %d)", ceil_height, ceil_index);
        }
    }

    // Apply PassThrough filter on Z axis
    PointCloudPtr filtered(new PointCloudXYZ);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(floor_height, ceil_height);
    pass.filter(*filtered);

    return filtered;
}

// ============================================================================
// voxelDownsample: PCL VoxelGrid downsampling
// ============================================================================
PointCloudPtr voxelDownsample(const PointCloudPtr& cloud, float leaf_size) {
    if (!cloud || cloud->empty()) {
        return PointCloudPtr(new PointCloudXYZ);
    }

    PointCloudPtr filtered(new PointCloudXYZ);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*filtered);

    return filtered;
}

// ============================================================================
// rotateVertical: Rotate point cloud around X axis
// ============================================================================
PointCloudPtr rotateVertical(const PointCloudPtr& cloud, double angle) {
    PointCloudPtr result(new PointCloudXYZ);
    result->resize(cloud->size());

    float cos_a = static_cast<float>(std::cos(angle));
    float sin_a = static_cast<float>(std::sin(angle));

    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& p = (*cloud)[i];
        auto& q = (*result)[i];
        q.x =  p.x * cos_a + p.z * sin_a;
        q.y =  p.y;
        q.z = -p.x * sin_a + p.z * cos_a;
    }
    return result;
}

} // namespace prism_topomap
