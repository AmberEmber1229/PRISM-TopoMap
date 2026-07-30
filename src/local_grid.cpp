/**
 * @file local_grid.cpp
 * @brief 局部占据栅格类的实现
 *
 * 逐方法对应原 Python local_grid.py
 * 使用 OpenCV 进行栅格仿射变换和图像 I/O
 */
#include "prism_topomap/local_grid.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sys/stat.h>
#include <chrono>

namespace prism_topomap {

// 辅助: 创建目录 (跨平台)
static void mkdirIfNotExists(const std::string& path) {
    struct stat st;
    if (stat(path.c_str(), &st) != 0) {
#ifdef _WIN32
        mkdir(path.c_str());
#else
        mkdir(path.c_str(), 0755);
#endif
    }
}

// ============================================================================
// 构造函数
// 对应 Python: LocalGrid.__init__()
// ============================================================================
LocalGrid::LocalGrid(double resolution,
                     double radius,
                     double max_range,
                     double floor_height,
                     double ceil_height,
                     double obstacles_attenuation,
                     double curbs_attenuation,
                     const std::vector<std::string>& layer_names,
                     const std::string& save_dir)
    : resolution_(resolution),
      radius_(radius),
      max_range_(max_range),
      floor_height_(floor_height),
      ceil_height_(ceil_height),
      obstacles_attenuation_(obstacles_attenuation),
      curbs_attenuation_(curbs_attenuation),
      layer_names_(layer_names),
      save_dir_(save_dir) {
    // grid_size = 2 * int(radius / resolution)
    grid_size_ = 2 * static_cast<int>(radius_ / resolution_);

    // 初始化各层为零矩阵
    for (const auto& name : layer_names_) {
        if (name == "height_map") {
            layers_[name] = cv::Mat::zeros(grid_size_, grid_size_, CV_32F);
        } else {
            layers_[name] = cv::Mat::zeros(grid_size_, grid_size_, CV_8U);
        }
    }

    // 创建保存目录
    if (!save_dir_.empty()) {
        mkdirIfNotExists(save_dir_);
    }
}

// ============================================================================
// copy: 深拷贝
// 对应 Python: LocalGrid.copy()
// ============================================================================
LocalGrid LocalGrid::copy() const {
    LocalGrid grid_copy(resolution_, radius_, max_range_,
                        floor_height_, ceil_height_,
                        obstacles_attenuation_, curbs_attenuation_,
                        layer_names_, save_dir_);
    grid_copy.layers_.clear();
    for (const auto& name : layer_names_) {
        grid_copy.layers_[name] = layers_.at(name).clone();
    }
    return grid_copy;
}

// ============================================================================
// raycastGrid: 光线投影填充可见区域
// 对应 Python:
//   def raycast_grid(self, n_rays=1000, center_point=None):
//       grid_raycasted = self.layers['occupancy'].copy()
//       if center_point is None:
//           center_point = (self.grid_size // 2, self.grid_size // 2)
//       for sector in range(n_rays):
//           angle = sector / n_rays * 2 * np.pi - np.pi
//           ii = center + sin(angle) * arange(...)
//           jj = center + cos(angle) * arange(...)
//           ... 找到最后一个障碍物, 之前全标1 ...
//       self.layers['occupancy'] = grid_raycasted
// ============================================================================
void LocalGrid::raycastGrid(int n_rays, int center_i, int center_j) {
    cv::Mat& occ = layers_["occupancy"];
    cv::Mat raycasted = occ.clone();

    if (center_i < 0) center_i = grid_size_ / 2;
    if (center_j < 0) center_j = grid_size_ / 2;

    int half_size = grid_size_ / 2;

    for (int sector = 0; sector < n_rays; ++sector) {
        double angle = static_cast<double>(sector) / n_rays * 2.0 * M_PI - M_PI;
        double sin_a = std::sin(angle);
        double cos_a = std::cos(angle);

        // 沿射线方向的栅格坐标
        int last_obst = -1;
        std::vector<std::pair<int, int>> ray_cells;
        ray_cells.reserve(half_size);

        for (int k = 0; k < half_size; ++k) {
            int ii = center_i + static_cast<int>(sin_a * k);
            int jj = center_j + static_cast<int>(cos_a * k);

            // 边界检查
            if (ii <= 0 || ii >= grid_size_ || jj <= 0 || jj >= grid_size_) {
                break;
            }

            ray_cells.push_back({ii, jj});

            if (occ.at<uint8_t>(ii, jj) > 0) {
                last_obst = static_cast<int>(ray_cells.size()) - 1;
            }
        }

        // 标记可见区域
        if (last_obst >= 0) {
            for (int k = 0; k < last_obst; ++k) {
                raycasted.at<uint8_t>(ray_cells[k].first, ray_cells[k].second) = 1;
            }
        } else {
            for (const auto& cell : ray_cells) {
                raycasted.at<uint8_t>(cell.first, cell.second) = 1;
            }
        }
    }

    occ = raycasted;
}

// ============================================================================
// getTransformedGrid: 对单层栅格做仿射变换
// 对应 Python:
//   def get_transformed_grid(self, grid, x, y, theta):
//       minus8 = [[1,0,radius/resolution],[0,1,radius/resolution],[0,0,1]]
//       tf_matrix = [[cos(-theta), sin(-theta), y/resolution],
//                    [-sin(-theta), cos(-theta), x/resolution],
//                    [0, 0, 1]]
//       plus8 = [[1,0,-radius/resolution],[0,1,-radius/resolution],[0,0,1]]
//       tf_matrix_shifted = minus8 @ tf_matrix @ plus8
//       return warpAffine(grid, tf_matrix_shifted[:2], grid.shape)
// ============================================================================
cv::Mat LocalGrid::getTransformedGrid(const cv::Mat& grid,
                                      double x, double y, double theta) const {
    double r = radius_ / resolution_;
    double neg_theta = -theta;
    double cos_t = std::cos(neg_theta);
    double sin_t = std::sin(neg_theta);

    // minus8: 平移到中心
    Eigen::Matrix3d minus8 = Eigen::Matrix3d::Identity();
    minus8(0, 2) = r;
    minus8(1, 2) = r;

    // tf_matrix: 旋转 + 平移
    Eigen::Matrix3d tf_mat;
    tf_mat << cos_t,  sin_t,  y / resolution_,
             -sin_t,  cos_t,  x / resolution_,
              0.0,    0.0,    1.0;

    // plus8: 平移回去
    Eigen::Matrix3d plus8 = Eigen::Matrix3d::Identity();
    plus8(0, 2) = -r;
    plus8(1, 2) = -r;

    // 组合变换
    Eigen::Matrix3d combined = minus8 * tf_mat * plus8;

    // 转为 OpenCV 2x3 仿射矩阵
    cv::Mat affine_mat(2, 3, CV_64F);
    affine_mat.at<double>(0, 0) = combined(0, 0);
    affine_mat.at<double>(0, 1) = combined(0, 1);
    affine_mat.at<double>(0, 2) = combined(0, 2);
    affine_mat.at<double>(1, 0) = combined(1, 0);
    affine_mat.at<double>(1, 1) = combined(1, 1);
    affine_mat.at<double>(1, 2) = combined(1, 2);

    cv::Mat result;
    cv::warpAffine(grid, result, affine_mat,
                   cv::Size(grid.cols, grid.rows),
                   cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
    return result;
}

// ============================================================================
// transform: 对所有层做仿射变换
// 对应 Python:
//   def transform(self, x, y, theta):
//       for layer_name in self.layer_names:
//           self.layers[layer_name] = self.get_transformed_grid(...)
// ============================================================================
void LocalGrid::transform(double x, double y, double theta) {
    for (const auto& name : layer_names_) {
        layers_[name] = getTransformedGrid(layers_[name], x, y, theta);
    }
}

// ============================================================================
// updateFromCloudAndTransform
// 对应 Python: LocalGrid.update_from_cloud_and_transform()
// ============================================================================
void LocalGrid::updateFromCloudAndTransform(const PointCloudPtr& points_xyz,
                                            double x, double y, double theta,
                                            LocalGridUpdateStats* stats) {
    const auto flow_start = std::chrono::steady_clock::now();
    if (stats) {
        *stats = LocalGridUpdateStats();
        stats->input_points = points_xyz ? points_xyz->size() : 0;
        stats->curbs_layer_exists = layers_.count("curbs") > 0;
    }

    auto finish_stats = [&]() {
        if (!stats) return;

        const cv::Mat& occ = layers_["occupancy"];
        cv::Mat mask;
        cv::compare(occ, 0, mask, cv::CMP_EQ);
        stats->unknown_cells = cv::countNonZero(mask);
        cv::compare(occ, 1, mask, cv::CMP_EQ);
        stats->free_cells = cv::countNonZero(mask);
        cv::compare(occ, 2, mask, cv::CMP_EQ);
        stats->occupied_cells = cv::countNonZero(mask);

        if (layers_.count("density_map")) {
            stats->density_nonzero_cells = cv::countNonZero(layers_["density_map"]);
            double min_val = 0.0;
            cv::minMaxLoc(layers_["density_map"], &min_val, &stats->density_max);
        }
        if (layers_.count("height_map")) {
            cv::Mat nonzero_mask;
            cv::compare(layers_["height_map"], 0, nonzero_mask, cv::CMP_NE);
            stats->height_nonzero_cells = cv::countNonZero(nonzero_mask);
            double min_val = 0.0;
            cv::minMaxLoc(layers_["height_map"], &min_val, &stats->height_max);
        }
        stats->elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - flow_start).count();
    };

    // 1. Transform existing grid (accumulate odometry increment)
    transform(x, y, theta);

    if (!points_xyz || points_xyz->empty()) {
        finish_stats();
        return;
    }

    // 2. Range filter: [-max_range, max_range] on X and Y
    float mr = static_cast<float>(max_range_);
    PointCloudPtr in_range(new PointCloudXYZ);
    in_range->reserve(points_xyz->size());
    for (const auto& p : *points_xyz) {
        const bool finite = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
        if (stats && finite) stats->finite_points++;
        const bool in_bounds = finite &&
            p.x > -mr && p.x < mr && p.y > -mr && p.y < mr;
        if (in_bounds) {
            in_range->push_back(p);
        } else if (stats && finite) {
            stats->out_of_range_points++;
        }
    }

    if (stats) stats->in_range_points = in_range->size();

    if (in_range->empty()) {
        finish_stats();
        return;
    }

    // 3. Separate obstacle points (remove floor and ceiling)
    PointCloudPtr obstacles = removeFloorAndCeil(in_range, floor_height_, ceil_height_);
    if (stats) stats->obstacle_points = obstacles->size();

    int grid_radius = static_cast<int>(radius_ / resolution_);

    // 4. Project all points to grid coordinates
    auto toGridCoord = [&](const pcl::PointXYZ& p, int& gi, int& gj) -> bool {
        gi = static_cast<int>(std::round(p.x / resolution_)) + grid_radius;
        gj = static_cast<int>(std::round(p.y / resolution_)) + grid_radius;
        return (gi >= 0 && gi < grid_size_ && gj >= 0 && gj < grid_size_);
    };

    std::vector<std::pair<int, int>> all_ij, obst_ij;
    std::vector<float> all_z_values;
    all_ij.reserve(in_range->size());
    all_z_values.reserve(in_range->size());

    for (const auto& p : *in_range) {
        int gi, gj;
        if (toGridCoord(p, gi, gj)) {
            all_ij.push_back({gi, gj});
            all_z_values.push_back(p.z);
        }
    }

    obst_ij.reserve(obstacles->size());
    for (const auto& p : *obstacles) {
        int gi, gj;
        if (toGridCoord(p, gi, gj)) {
            obst_ij.push_back({gi, gj});
        }
    }

    // 5. Fill occupancy layer
    layers_["occupancy"] = cv::Mat::zeros(grid_size_, grid_size_, CV_8U);
    for (const auto& p : all_ij) {
        layers_["occupancy"].at<uint8_t>(p.first, p.second) = 1;
    }
    raycastGrid();
    for (const auto& p : obst_ij) {
        layers_["occupancy"].at<uint8_t>(p.first, p.second) = 2;
    }

    // 6. Update density_map
    if (layers_.count("density_map")) {
        cv::Mat density_cur = cv::Mat::zeros(grid_size_, grid_size_, CV_32F);
        for (const auto& p : obst_ij) {
            density_cur.at<float>(p.first, p.second) += 1.0f;
        }
        layers_["density_map_cur"] = density_cur;
        layers_["density_map"].convertTo(layers_["density_map"], CV_32F);
        layers_["density_map"] = layers_["density_map"] * obstacles_attenuation_ + density_cur;
    }

    // 7. Update height_map
    if (layers_.count("height_map")) {
        layers_["height_map"] = cv::Mat::zeros(grid_size_, grid_size_, CV_32F);
        for (size_t i = 0; i < all_ij.size(); ++i) {
            float& cur = layers_["height_map"].at<float>(all_ij[i].first, all_ij[i].second);
            cur = std::max(cur, all_z_values[i]);
        }
    }

    finish_stats();
}

// ============================================================================
// updateCurbsFromCloud
// 对应 Python: LocalGrid.update_curbs_from_cloud()
// ============================================================================
void LocalGrid::updateCurbsFromCloud(const PointCloudPtr& points_xyz) {
    if (!points_xyz || points_xyz->empty()) return;

    float mr = static_cast<float>(max_range_);
    int grid_radius = static_cast<int>(radius_ / resolution_);

    std::vector<std::pair<int, int>> curb_ij;
    for (const auto& p : *points_xyz) {
        if (!std::isfinite(p.x)) continue;
        if (p.x <= -mr || p.x >= mr || p.y <= -mr || p.y >= mr) continue;

        int gi = static_cast<int>(std::round(p.x / resolution_)) + grid_radius;
        int gj = static_cast<int>(std::round(p.y / resolution_)) + grid_radius;
        if (gi >= 0 && gi < grid_size_ && gj >= 0 && gj < grid_size_) {
            curb_ij.push_back({gi, gj});
        }
    }

    cv::Mat curbs_cur = cv::Mat::zeros(grid_size_, grid_size_, CV_8U);
    for (const auto& p : curb_ij) {
        curbs_cur.at<uint8_t>(p.first, p.second) = 1;
    }

    if (layers_.count("curbs")) {
        cv::Mat curbs_float;
        layers_["curbs"].convertTo(curbs_float, CV_32F);
        cv::Mat curbs_cur_float;
        curbs_cur.convertTo(curbs_cur_float, CV_32F);
        cv::Mat result = curbs_float * curbs_attenuation_ + curbs_cur_float;
        result.convertTo(layers_["curbs"], CV_8U);
    }
}

// ============================================================================
// isInside: 判断位置是否在已知区域内
// 对应 Python:
//   def is_inside(self, x, y, theta):
//       i = int((x + self.radius) / self.resolution)
//       j = int((y + self.radius) / self.resolution)
//       return (self.layers['occupancy'][i, j] > 0)
// ============================================================================
bool LocalGrid::isInside(double x, double y, double /*theta*/) const {
    int i = static_cast<int>((x + radius_) / resolution_);
    int j = static_cast<int>((y + radius_) / resolution_);
    if (i < 0 || i >= grid_size_ || j < 0 || j >= grid_size_) {
        return false;
    }
    return (layers_.at("occupancy").at<uint8_t>(i, j) > 0);
}

// ============================================================================
// getIoU: 计算与另一个栅格的 IoU
// 对应 Python: LocalGrid.get_iou()
// ============================================================================
double LocalGrid::getIoU(const LocalGrid& other,
                         double rel_x, double rel_y, double rel_theta,
                         bool save, int cnt) const {
    // 旋转相对位姿
    // rel_x_rotated = -rel_x * cos(rel_theta) - rel_y * sin(rel_theta)
    // rel_y_rotated =  rel_x * sin(rel_theta) - rel_y * cos(rel_theta)
    double cos_t = std::cos(rel_theta);
    double sin_t = std::sin(rel_theta);
    double rx = -rel_x * cos_t - rel_y * sin_t;
    double ry =  rel_x * sin_t - rel_y * cos_t;

    // 变换当前栅格的 occupancy
    cv::Mat cur_transformed = getTransformedGrid(layers_.at("occupancy"), rx, ry, rel_theta);

    // 二值化: > 0 → 1
    cv::Mat cur_bin;
    cv::threshold(cur_transformed, cur_bin, 0, 1, cv::THRESH_BINARY);

    cv::Mat v_bin = other.layers_.at("occupancy").clone();
    cv::threshold(v_bin, v_bin, 0, 1, cv::THRESH_BINARY);

    // 确保数据类型一致
    cur_bin.convertTo(cur_bin, CV_32F);
    v_bin.convertTo(v_bin, CV_32F);

    // intersection = sum(v * cur)
    cv::Mat intersection_mat;
    cv::multiply(v_bin, cur_bin, intersection_mat);
    double intersection = cv::sum(intersection_mat)[0];

    // union = sum(v | cur)
    cv::Mat union_mat;
    cv::max(v_bin, cur_bin, union_mat);
    double union_val = cv::sum(union_mat)[0];

    if (union_val < 1e-6) return 0.0;

    // 可选: 保存调试数据
    if (save && !save_dir_.empty()) {
        std::string dir = save_dir_ + "/" + std::to_string(cnt);
        mkdirIfNotExists(dir);
        // 保存对齐可视化图
        cv::Mat aligned(grid_size_, grid_size_, CV_8UC3, cv::Scalar(0, 0, 0));
        for (int r = 0; r < grid_size_; ++r) {
            for (int c = 0; c < grid_size_; ++c) {
                aligned.at<cv::Vec3b>(r, c)[0] = static_cast<uint8_t>(cur_bin.at<float>(r, c) * 255);  // R
                aligned.at<cv::Vec3b>(r, c)[1] = static_cast<uint8_t>(v_bin.at<float>(r, c) * 255);    // G
            }
        }
        cv::imwrite(dir + "/grid_aligned.png", aligned);
    }

    return intersection / union_val;
}

// ============================================================================
// getTfMatrixXY: 从栅格像素变换到度量坐标的 4x4 矩阵
// 对应 Python: LocalGrid.get_tf_matrix_xy(trans_i, trans_j, rot_angle)
// ============================================================================
Eigen::Matrix4d LocalGrid::getTfMatrixXY(double trans_i, double trans_j,
                                          double rot_angle) const {
    // plus8: 平移到中心
    Eigen::Matrix4d plus8 = Eigen::Matrix4d::Identity();
    plus8(0, 3) = radius_;
    plus8(1, 3) = radius_;

    // minus8: 平移回去
    Eigen::Matrix4d minus8 = Eigen::Matrix4d::Identity();
    minus8(0, 3) = -radius_;
    minus8(1, 3) = -radius_;

    // tf_matrix: 旋转 + 像素坐标到度量坐标的平移
    double cos_a = std::cos(rot_angle);
    double sin_a = std::sin(rot_angle);
    Eigen::Matrix4d tf_mat = Eigen::Matrix4d::Identity();
    tf_mat(0, 0) =  cos_a;   tf_mat(0, 1) = sin_a;
    tf_mat(1, 0) = -sin_a;   tf_mat(1, 1) = cos_a;
    tf_mat(0, 3) = trans_i * resolution_;
    tf_mat(1, 3) = trans_j * resolution_;

    return minus8 * tf_mat * plus8;
}

// ============================================================================
// save / load: 序列化
// 对应 Python: LocalGrid.save() / load_local_grid()
// ============================================================================
void LocalGrid::save(const std::string& save_dir) const {
    mkdirIfNotExists(save_dir);

    for (const auto& kv : layers_) {
        const std::string& name = kv.first;
        const cv::Mat& layer = kv.second;

        if (name == "height_map") {
            // 保存为 npz 格式的替代: 保存为二进制 float 文件
            // 为了与 Python 端兼容, 使用 OpenCV 的 FileStorage
            cv::FileStorage fs(save_dir + "/" + name + ".yaml", cv::FileStorage::WRITE);
            fs << "data" << layer;
            fs.release();
        } else {
            cv::Mat save_mat;
            layer.convertTo(save_mat, CV_8U);
            cv::imwrite(save_dir + "/" + name + ".png", save_mat);
        }
    }

    // 保存 metadata
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "layer_names" << YAML::Value << YAML::BeginSeq;
    for (const auto& name : layer_names_) {
        out << name;
    }
    out << YAML::EndSeq;
    out << YAML::Key << "resolution" << YAML::Value << resolution_;
    out << YAML::Key << "radius" << YAML::Value << radius_;
    out << YAML::Key << "max_range" << YAML::Value << max_range_;
    out << YAML::Key << "floor_height" << YAML::Value << floor_height_;
    out << YAML::Key << "ceil_height" << YAML::Value << ceil_height_;
    out << YAML::Key << "obstacles_attenuation" << YAML::Value << obstacles_attenuation_;
    out << YAML::Key << "curbs_attenuation" << YAML::Value << curbs_attenuation_;
    out << YAML::EndMap;

    std::ofstream fout(save_dir + "/metadata.yaml");
    fout << out.c_str();
    fout.close();
}

LocalGrid LocalGrid::load(const std::string& save_dir) {
    // 读取 metadata
    YAML::Node meta = YAML::LoadFile(save_dir + "/metadata.yaml");

    std::vector<std::string> layer_names;
    for (const auto& n : meta["layer_names"]) {
        layer_names.push_back(n.as<std::string>());
    }

    LocalGrid grid(
        meta["resolution"].as<double>(),
        meta["radius"].as<double>(),
        meta["max_range"].as<double>(),
        meta["floor_height"].as<double>(),
        meta["ceil_height"].as<double>(),
        meta["obstacles_attenuation"].as<double>(),
        meta["curbs_attenuation"].as<double>(),
        layer_names
    );

    // 加载各层
    for (const auto& name : layer_names) {
        if (name == "height_map") {
            cv::FileStorage fs(save_dir + "/" + name + ".yaml", cv::FileStorage::READ);
            if (fs.isOpened()) {
                fs["data"] >> grid.layers_[name];
                fs.release();
            }
        } else {
            cv::Mat img = cv::imread(save_dir + "/" + name + ".png", cv::IMREAD_GRAYSCALE);
            if (!img.empty()) {
                grid.layers_[name] = img;
            }
        }
    }

    return grid;
}

// ============================================================================
// 访问接口
// ============================================================================
const cv::Mat& LocalGrid::getLayer(const std::string& name) const {
    return layers_.at(name);
}

cv::Mat& LocalGrid::getMutableLayer(const std::string& name) {
    return layers_.at(name);
}

} // namespace prism_topomap
