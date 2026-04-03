/**
 * @file utils.h
 * @brief 位姿工具函数和点云处理工具 (Eigen 实现)
 *
 * 对应原 Python 文件: scripts/utils.py
 * 所有 2D 位姿以 Eigen::Vector3d [x, y, theta] 表示
 * 所有点云以 Eigen::MatrixXf (Nx3) 表示
 */
#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <string>
#include <sensor_msgs/PointCloud2.h>

namespace prism_topomap {

// ============================================================================
// 类型定义
// ============================================================================

/// 2D 位姿: [x, y, theta]
using Pose2D = Eigen::Vector3d;

/// 点云矩阵: 每行 [x, y, z], 大小 Nx3
using PointCloud = Eigen::MatrixXf;

// ============================================================================
// 角度工具
// ============================================================================

/**
 * @brief 将角度归一化到 [-π, π] 范围
 * 对应 Python: normalize(angle)
 */
double normalize(double angle);

// ============================================================================
// 2D 旋转
// ============================================================================

/**
 * @brief 将 (x, y) 绕原点旋转 angle 弧度
 * 对应 Python: rotate(x, y, angle)
 * @return 旋转后的 (x_new, y_new)
 */
Eigen::Vector2d rotate2D(double x, double y, double angle);

// ============================================================================
// 位姿运算
// ============================================================================

/**
 * @brief 计算从 from 到 to 的相对位姿
 * 对应 Python: get_rel_pose(x1,y1,theta1, x2,y2,theta2)
 *
 * @param from 参考位姿 [x1, y1, theta1]
 * @param to   目标位姿 [x2, y2, theta2]
 * @return 相对位姿 [rel_x, rel_y, rel_theta]
 */
Pose2D getRelPose(const Pose2D& from, const Pose2D& to);

/**
 * @brief 在当前位姿上叠加一个相对位移
 * 对应 Python: apply_pose_shift(pose, rel_x, rel_y, rel_theta)
 *
 * @param pose  当前位姿 [x, y, theta]
 * @param shift 相对位移 [rel_x, rel_y, rel_theta]
 * @return 新位姿 [new_x, new_y, new_theta]
 */
Pose2D applyPoseShift(const Pose2D& pose, const Pose2D& shift);

// ============================================================================
// 点云处理
// ============================================================================

/**
 * @brief 对点云的 xyz 坐标应用 3x3 旋转矩阵
 * 对应 Python: rotate_pcd(points, rotation_matrix)
 *
 * @param points         Nx3 或 Nx6 点云
 * @param rotation_matrix 3x3 旋转矩阵
 * @return 旋转后的点云 (仅旋转前3列, 保留后续列)
 */
PointCloud rotatePcd(const PointCloud& points,
                     const Eigen::Matrix3f& rotation_matrix);

/**
 * @brief 对点云应用 2D 变换 (x平移, y平移, 旋转)
 * 对应 Python: transform_pcd(points, x, y, theta)
 */
PointCloud transformPcd(const PointCloud& points,
                        double x, double y, double theta);

/**
 * @brief 从 ROS PointCloud2 消息提取 xyz 坐标并旋转
 * 对应 Python: get_xyz_coords_from_msg(msg, fields, rotation)
 *
 * @param msg      ROS PointCloud2 消息
 * @param fields   "xyz" 或 "xyzrgb"
 * @param rotation 3x3 旋转矩阵
 * @return Nx3 点云 (或 Nx6 如果 fields="xyzrgb")
 */
PointCloud getXyzCoordsFromMsg(const sensor_msgs::PointCloud2& msg,
                               const std::string& fields,
                               const Eigen::Matrix3f& rotation);

/**
 * @brief 去除地面和天花板的点
 * 对应 Python: remove_floor_and_ceil(cloud, floor_height, ceil_height)
 *
 * 支持 floor_height/ceil_height 为固定值。
 * (原Python的'auto'模式在C++中暂不支持, 使用固定值)
 *
 * @param cloud        Nx3 点云
 * @param floor_height 地面高度阈值 (低于此值的点被去除)
 * @param ceil_height  天花板高度阈值 (高于此值的点被去除)
 * @return 过滤后的点云
 */
PointCloud removeFloorAndCeil(const PointCloud& cloud,
                              float floor_height,
                              float ceil_height);

/**
 * @brief 绕 x 轴旋转点云 (垂直旋转)
 * 对应 Python: rotate_vertical(cloud, angle)
 */
PointCloud rotateVertical(const PointCloud& cloud, double angle);

} // namespace prism_topomap
