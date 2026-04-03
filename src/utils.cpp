/**
 * @file utils.cpp
 * @brief 位姿工具函数和点云处理工具的实现
 *
 * 逐函数对应原 Python utils.py, 使用 Eigen 进行向量化计算
 */
#include "prism_topomap/utils.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath>
#include <algorithm>
#include <vector>

namespace prism_topomap {

// ============================================================================
// normalize: 角度归一化到 [-π, π]
// 对应 Python:
//   def normalize(angle):
//       while angle < -np.pi: angle += 2*np.pi
//       while angle > np.pi:  angle -= 2*np.pi
//       return angle
// ============================================================================
double normalize(double angle) {
    while (angle < -M_PI) angle += 2.0 * M_PI;
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    return angle;
}

// ============================================================================
// rotate2D: 2D坐标旋转
// 对应 Python:
//   def rotate(x, y, angle):
//       x_new = x * np.cos(angle) + y * np.sin(angle)
//       y_new = -x * np.sin(angle) + y * np.cos(angle)
//       return x_new, y_new
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
// getRelPose: 计算相对位姿
// 对应 Python:
//   def get_rel_pose(x, y, theta, x2, y2, theta2):
//       rel_x, rel_y = rotate(x2 - x, y2 - y, theta)
//       return [rel_x, rel_y, normalize(theta2 - theta)]
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
// applyPoseShift: 位姿叠加
// 对应 Python:
//   def apply_pose_shift(pose, rel_x, rel_y, rel_theta):
//       x, y, theta = pose
//       new_x = x + rel_x * np.cos(-theta) + rel_y * np.sin(-theta)
//       new_y = y - rel_x * np.sin(-theta) + rel_y * np.cos(-theta)
//       new_theta = theta + rel_theta
//       return [new_x, new_y, new_theta]
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
// rotatePcd: 对点云 xyz 坐标应用旋转矩阵
// 对应 Python:
//   def rotate_pcd(points, rotation_matrix):
//       points_xyz = points[:, :3]
//       points_xyz_rotated = points_xyz @ rotation_matrix
//       points_rotated = points.copy()
//       points_rotated[:, :3] = points_xyz_rotated
//       return points_rotated
// ============================================================================
PointCloud rotatePcd(const PointCloud& points,
                     const Eigen::Matrix3f& rotation_matrix) {
    PointCloud result = points;  // 拷贝全部列
    // 仅旋转前3列: result[:, :3] = points[:, :3] * rotation_matrix
    result.leftCols(3) = points.leftCols(3) * rotation_matrix;
    return result;
}

// ============================================================================
// transformPcd: 对点云应用 2D 变换
// 对应 Python:
//   def transform_pcd(points, x, y, theta):
//       points_transformed = points.copy()
//       points_transformed[:, 0] = points[:, 0]*cos(theta) + points[:, 1]*sin(theta)
//       points_transformed[:, 1] = -points[:, 0]*sin(theta) + points[:, 1]*cos(theta)
//       points_transformed[:, 0] += x
//       points_transformed[:, 1] += y
//       return points_transformed
// ============================================================================
PointCloud transformPcd(const PointCloud& points,
                        double x, double y, double theta) {
    PointCloud result = points;
    float cos_t = static_cast<float>(std::cos(theta));
    float sin_t = static_cast<float>(std::sin(theta));

    // 先旋转
    Eigen::VectorXf new_col0 =  points.col(0) * cos_t + points.col(1) * sin_t;
    Eigen::VectorXf new_col1 = -points.col(0) * sin_t + points.col(1) * cos_t;

    // 再平移
    result.col(0) = new_col0.array() + static_cast<float>(x);
    result.col(1) = new_col1.array() + static_cast<float>(y);

    return result;
}

// ============================================================================
// getXyzCoordsFromMsg: 从 ROS PointCloud2 消息提取点云
// 对应 Python:
//   def get_xyz_coords_from_msg(msg, fields, rotation):
//       points_numpify = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
//       ...
//       points_xyz = rotate_pcd(points_xyz, rotation)
//       return points_xyz
// ============================================================================
PointCloud getXyzCoordsFromMsg(const sensor_msgs::PointCloud2& msg,
                               const std::string& fields,
                               const Eigen::Matrix3f& rotation) {
    // 计算点数
    int n_points = msg.width * msg.height;
    if (n_points == 0) {
        return PointCloud(0, 3);
    }

    int n_cols = 3;  // xyz
    if (fields == "xyzrgb") {
        n_cols = 6;
    }

    PointCloud points(n_points, n_cols);

    // 使用 PointCloud2 迭代器高效提取数据
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

    for (int i = 0; i < n_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
        points(i, 0) = *iter_x;
        points(i, 1) = *iter_y;
        points(i, 2) = *iter_z;
    }

    // 如果是 xyzrgb, 还需要提取 RGB
    if (fields == "xyzrgb") {
        // 尝试读取 rgb 打包字段
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(msg, "r");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(msg, "g");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(msg, "b");
        for (int i = 0; i < n_points; ++i, ++iter_r, ++iter_g, ++iter_b) {
            points(i, 3) = static_cast<float>(*iter_r);
            points(i, 4) = static_cast<float>(*iter_g);
            points(i, 5) = static_cast<float>(*iter_b);
        }
    }

    // 应用旋转矩阵
    points = rotatePcd(points, rotation);

    return points;
}

// ============================================================================
// removeFloorAndCeil: 移除地面和天花板点
// 对应 Python:
//   def remove_floor_and_ceil(cloud, floor_height, ceil_height):
//       return cloud[(cloud[:, 2] > floor_height) * (cloud[:, 2] < ceil_height)]
// (简化版: 仅支持固定阈值, 不支持 'auto' 模式)
// ============================================================================
PointCloud removeFloorAndCeil(const PointCloud& cloud,
                              float floor_height,
                              float ceil_height) {
    // 先计算满足条件的点数
    std::vector<int> valid_indices;
    valid_indices.reserve(cloud.rows());

    for (int i = 0; i < cloud.rows(); ++i) {
        float z = cloud(i, 2);
        if (z > floor_height && z < ceil_height) {
            valid_indices.push_back(i);
        }
    }

    // 构建结果矩阵
    PointCloud result(valid_indices.size(), cloud.cols());
    for (size_t i = 0; i < valid_indices.size(); ++i) {
        result.row(i) = cloud.row(valid_indices[i]);
    }
    return result;
}

// ============================================================================
// rotateVertical: 绕 x 轴旋转点云
// 对应 Python:
//   def rotate_vertical(cloud, angle):
//       cloud_rotated = cloud.copy()
//       cloud_rotated[:, 0] = cloud[:, 0]*cos(angle) + cloud[:, 2]*sin(angle)
//       cloud_rotated[:, 2] = -cloud[:, 0]*sin(angle) + cloud[:, 2]*cos(angle)
//       return cloud_rotated
// ============================================================================
PointCloud rotateVertical(const PointCloud& cloud, double angle) {
    PointCloud result = cloud;
    float cos_a = static_cast<float>(std::cos(angle));
    float sin_a = static_cast<float>(std::sin(angle));

    result.col(0) =  cloud.col(0) * cos_a + cloud.col(2) * sin_a;
    result.col(2) = -cloud.col(0) * sin_a + cloud.col(2) * cos_a;

    return result;
}

} // namespace prism_topomap
