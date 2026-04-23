/**
 * @file local_grid.h
 * @brief 局部占据栅格类
 *
 * 对应原 Python 文件: scripts/local_grid.py
 * 使用 OpenCV cv::Mat 存储栅格层 (代替 numpy array)
 * 使用 cv::warpAffine 进行仿射变换
 */
#pragma once

#include "prism_topomap/utils.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <map>
#include <vector>
#include <memory>

namespace prism_topomap {

class LocalGrid {
public:
    // ========================================================================
    // 构造函数
    // 对应 Python: LocalGrid.__init__()
    // ========================================================================
    LocalGrid(double resolution = 0.1,
              double radius = 18.0,
              double max_range = 8.0,
              double floor_height = 0.0,
              double ceil_height = 1.0,
              double obstacles_attenuation = 0.9,
              double curbs_attenuation = 0.99,
              const std::vector<std::string>& layer_names = {"occupancy", "density_map", "height_map"},
              const std::string& save_dir = "");

    // ========================================================================
    // 拷贝
    // 对应 Python: LocalGrid.copy()
    // ========================================================================
    LocalGrid copy() const;

    // ========================================================================
    // 核心计算方法
    // ========================================================================

    /**
     * @brief 光线投影填充可见区域
     * 对应 Python: raycast_grid(n_rays=1000, center_point=None)
     *
     * 从中心点向外发射 n_rays 条射线, 将射线末端障碍物之前的区域
     * 标记为已知可通行(值=1), 障碍物保持值=2
     */
    void raycastGrid(int n_rays = 1000,
                     int center_i = -1, int center_j = -1);

    /**
     * @brief 从点云更新栅格 + 坐标变换
     * 对应 Python: update_from_cloud_and_transform()
     *
     * 流程:
     * 1. 先对已有栅格做仿射变换 (累积里程计增量)
     * 2. 去除 NaN 和超范围点
     * 3. 点云投影到栅格坐标
     * 4. 光线投影 (raycast)
     * 5. 更新 density_map / height_map
     */
    void updateFromCloudAndTransform(const PointCloudPtr& points_xyz,
                                     double x = 0.0, double y = 0.0,
                                     double theta = 0.0);

    /**
     * @brief 更新路沿层
     * 对应 Python: update_curbs_from_cloud()
     */
    void updateCurbsFromCloud(const PointCloudPtr& points_xyz);

    /**
     * @brief 对所有栅格层做仿射变换
     * 对应 Python: transform(x, y, theta)
     */
    void transform(double x, double y, double theta);

    /**
     * @brief 判断某位置是否在已知区域内
     * 对应 Python: is_inside(x, y, theta)
     */
    bool isInside(double x, double y, double theta) const;

    /**
     * @brief 计算与另一个栅格的 IoU
     * 对应 Python: get_iou(other, rel_x, rel_y, rel_theta, ...)
     */
    double getIoU(const LocalGrid& other,
                  double rel_x, double rel_y, double rel_theta,
                  bool save = false, int cnt = 0) const;

    /**
     * @brief 栅格像素坐标到度量坐标的变换矩阵
     * 对应 Python: get_tf_matrix_xy(trans_i, trans_j, rot_angle)
     */
    Eigen::Matrix4d getTfMatrixXY(double trans_i, double trans_j,
                                   double rot_angle) const;

    // ========================================================================
    // 内部 helper
    // ========================================================================

    /**
     * @brief 对单层栅格做仿射变换
     * 对应 Python: get_transformed_grid(grid, x, y, theta)
     */
    cv::Mat getTransformedGrid(const cv::Mat& grid,
                               double x, double y, double theta) const;

    // ========================================================================
    // 序列化
    // ========================================================================
    void save(const std::string& save_dir) const;
    static LocalGrid load(const std::string& save_dir);

    // ========================================================================
    // 访问接口
    // ========================================================================
    const cv::Mat& getLayer(const std::string& name) const;
    cv::Mat& getMutableLayer(const std::string& name);
    int gridSize() const { return grid_size_; }
    double resolution() const { return resolution_; }
    double radius() const { return radius_; }
    double maxRange() const { return max_range_; }
    double floorHeight() const { return floor_height_; }
    double ceilHeight() const { return ceil_height_; }
    const std::vector<std::string>& layerNames() const { return layer_names_; }

private:
    double resolution_;
    double radius_;
    double max_range_;
    double floor_height_;
    double ceil_height_;
    double obstacles_attenuation_;
    double curbs_attenuation_;
    int grid_size_;                              // 2 * (int)(radius/resolution)
    std::vector<std::string> layer_names_;
    std::map<std::string, cv::Mat> layers_;      // 各层栅格数据
    std::string save_dir_;
};

} // namespace prism_topomap
