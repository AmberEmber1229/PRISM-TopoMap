/**
 * @file topological_graph.cpp
 * @brief 拓扑图类的实现
 *
 * 逐方法对应原 Python topo_graph.py
 */
#include "prism_topomap/topological_graph.h"

#include <queue>
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <sys/stat.h>

// JSON 序列化: 使用 nlohmann/json (header-only)
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace prism_topomap {

// 辅助: 创建目录
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
// ============================================================================
TopologicalGraph::TopologicalGraph(
    std::shared_ptr<InferenceClient> inference_client,
    double inline_reg_score_threshold,
    double grid_resolution,
    double grid_radius,
    double max_grid_range,
    int descriptor_dim)
    : inference_client_(inference_client),
      inline_reg_score_threshold_(inline_reg_score_threshold),
      grid_resolution_(grid_resolution),
      grid_radius_(grid_radius),
      max_grid_range_(max_grid_range),
      descriptor_dim_(descriptor_dim) {
    // 创建 FAISS 索引 (L2 距离, 暴力搜索)
    faiss_index_ = std::make_unique<faiss::IndexFlatL2>(descriptor_dim_);
}

// ============================================================================
// addVertex: 添加顶点
// 对应 Python:
//   def add_vertex(self, global_pose_for_visualization, descriptor, grid):
//       self.adj_lists.append([])
//       vertex_dict = { 'pose_for_visualization': ..., 'grid': grid.copy(),
//                       'descriptor': descriptor }
//       self.vertices.append(vertex_dict)
//       self.index.add(descriptor)
//       return len(self.vertices) - 1
// ============================================================================
int TopologicalGraph::addVertex(const Pose2D& global_pose,
                                const std::vector<float>& descriptor,
                                const LocalGrid& grid) {
    double x = global_pose[0], y = global_pose[1], theta = global_pose[2];
    int idx = static_cast<int>(vertices_.size());
    ROS_INFO("\n\n\n ADD NEW VERTEX (%.1f, %.1f, %.2f) idx=%d \n\n\n", x, y, theta, idx);

    Vertex v;
    v.pose_for_visualization = global_pose;
    v.grid = grid.copy();
    v.descriptor = descriptor;

    vertices_.push_back(std::move(v));
    adj_lists_.push_back({});

    // 添加到 FAISS 索引
    addToIndex(descriptor);

    return idx;
}

// ============================================================================
// getVertex
// ============================================================================
const Vertex& TopologicalGraph::getVertex(int vertex_id) const {
    return vertices_.at(vertex_id);
}

// ============================================================================
// addEdge: 添加无向边
// 对应 Python:
//   def add_edge(self, i, j, x, y, theta):
//       if i == j: return
//       if j in [x[0] for x in self.adj_lists[i]]: return
//       self.adj_lists[i].append((j, [x, y, theta]))
//       self.adj_lists[j].append((i, inverse_transform(x, y, theta)))
// ============================================================================
void TopologicalGraph::addEdge(int i, int j, double x, double y, double theta) {
    if (i == j) return;

    // 检查边是否已存在
    for (const auto& entry : adj_lists_[i]) {
        if (entry.vertex_id == j) return;
    }

    double xi = vertices_[i].pose_for_visualization[0];
    double yi = vertices_[i].pose_for_visualization[1];
    double xj = vertices_[j].pose_for_visualization[0];
    double yj = vertices_[j].pose_for_visualization[1];
    ROS_INFO("\nAdd edge (%.1f,%.1f)->(%.1f,%.1f) rel_pose=(%.1f,%.1f,%.2f)\n",
             xi, yi, xj, yj, x, y, theta);

    adj_lists_[i].push_back({j, Pose2D(x, y, theta)});
    Pose2D inv = inverseTransform(x, y, theta);
    adj_lists_[j].push_back({i, inv});
}

// ============================================================================
// hasEdge / getEdge / getEdgesFrom
// ============================================================================
bool TopologicalGraph::hasEdge(int u, int v) const {
    for (const auto& entry : adj_lists_[u]) {
        if (entry.vertex_id == v) return true;
    }
    return false;
}

Pose2D TopologicalGraph::getEdge(int u, int v) const {
    for (const auto& entry : adj_lists_[u]) {
        if (entry.vertex_id == v) return entry.rel_pose;
    }
    return Pose2D(0, 0, 0);  // 不应到达这里
}

const std::vector<AdjEntry>& TopologicalGraph::getEdgesFrom(int u) const {
    return adj_lists_[u];
}

// ============================================================================
// inverseTransform: 位姿逆变换
// 对应 Python:
//   def inverse_transform(self, x, y, theta):
//       x_inv = -x * cos(theta) - y * sin(theta)
//       y_inv =  x * sin(theta) - y * cos(theta)
//       theta_inv = -theta
// ============================================================================
Pose2D TopologicalGraph::inverseTransform(double x, double y, double theta) {
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);
    double x_inv = -x * cos_t - y * sin_t;
    double y_inv =  x * sin_t - y * cos_t;
    return Pose2D(x_inv, y_inv, -theta);
}

// ============================================================================
// getTransformToVertex: 沿边配准
// 对应 Python: TopologicalGraph.get_transform_to_vertex()
//
// 将当前栅格和候选节点栅格发送到 Python 做配准,
// 然后在 C++ 端通过 getTfMatrixXY 计算位姿变换
// ============================================================================
TransformResult TopologicalGraph::getTransformToVertex(int vertex_id,
                                                        const LocalGrid& grid) {
    ROS_INFO("Trying to match vertex %d", vertex_id);

    TransformResult result;
    result.success = false;

    const LocalGrid& cand_grid = vertices_[vertex_id].grid;

    // 调用 Python 推理服务进行栅格配准 (inline 类型)
    auto reg_result = inference_client_->gridRegistration(
        grid.getLayer("occupancy"),
        cand_grid.getLayer("occupancy"),
        "inline"  // 沿边配准
    );

    if (reg_result.success && reg_result.score > inline_reg_score_threshold_) {
        // 使用 getTfMatrixXY 将像素级变换转为度量坐标变换
        Eigen::Matrix4d tf_matrix = cand_grid.getTfMatrixXY(
            reg_result.trans_i, reg_result.trans_j, reg_result.rot_angle);

        result.success = true;
        result.x = tf_matrix(0, 3);
        result.y = tf_matrix(1, 3);

        // 从旋转矩阵提取 theta (绕 z 轴)
        // rotvec 的 z 分量就是 yaw
        Eigen::Matrix3d rot = tf_matrix.block<3, 3>(0, 0);
        result.theta = std::atan2(rot(1, 0), rot(0, 0));

        return result;
    }

    return result;  // success = false
}

// ============================================================================
// getPathWithLength: Dijkstra 最短路
// 对应 Python: TopologicalGraph.get_path_with_length()
// ============================================================================
PathResult TopologicalGraph::getPathWithLength(int u, int v) const {
    PathResult result;
    result.found = false;
    result.length = std::numeric_limits<double>::infinity();

    int n = static_cast<int>(adj_lists_.size());
    if (u < 0 || u >= n || v < 0 || v >= n) {
        return result;
    }

    std::vector<double> distances(n, std::numeric_limits<double>::infinity());
    std::vector<int> prev_nodes(n, -1);
    distances[u] = 0.0;

    // 最小堆: (distance, node_id)
    using PII = std::pair<double, int>;
    std::priority_queue<PII, std::vector<PII>, std::greater<PII>> heap;
    heap.push({0.0, u});

    while (!heap.empty()) {
        auto [current_distance, current_node] = heap.top();
        heap.pop();

        // 找到目标
        if (current_node == v) {
            result.found = true;
            result.length = distances[v];
            // 回溯路径
            std::vector<int> path;
            int cur = current_node;
            while (cur != u) {
                path.push_back(cur);
                cur = prev_nodes[cur];
            }
            path.push_back(u);
            std::reverse(path.begin(), path.end());
            result.path = path;
            return result;
        }

        // 已访问过更短路径, 跳过
        if (current_distance > distances[current_node]) {
            continue;
        }

        // 遍历邻居
        for (const auto& entry : adj_lists_[current_node]) {
            double weight = std::sqrt(
                entry.rel_pose[0] * entry.rel_pose[0] +
                entry.rel_pose[1] * entry.rel_pose[1]);
            double tentative = current_distance + weight;

            if (tentative < distances[entry.vertex_id]) {
                distances[entry.vertex_id] = tentative;
                prev_nodes[entry.vertex_id] = current_node;
                heap.push({tentative, entry.vertex_id});
            }
        }
    }

    return result;  // found = false
}

// ============================================================================
// FAISS 索引操作
// ============================================================================
void TopologicalGraph::addToIndex(const std::vector<float>& descriptor) {
    if (descriptor.empty()) return;

    // FAISS 要求输入为 float*, 行数 1, 列数 descriptor_dim_
    // 如果描述符是 [1, dim] 形状 (从 Python 来), 取前 dim 个
    const float* data = descriptor.data();
    int n_vectors = 1;
    // 如果 descriptor 大小正好是 dim 的整数倍, 只添加第一行
    faiss_index_->add(n_vectors, data);
}

std::pair<std::vector<float>, std::vector<int>>
TopologicalGraph::searchIndex(const std::vector<float>& query, int top_k) const {
    int actual_k = std::min(top_k, static_cast<int>(faiss_index_->ntotal));
    if (actual_k <= 0) {
        return {{}, {}};
    }

    std::vector<float> distances(actual_k);
    std::vector<faiss::idx_t> indices(actual_k);

    faiss_index_->search(1, query.data(), actual_k,
                         distances.data(), indices.data());

    // 转换 faiss::idx_t → int
    std::vector<int> int_indices(indices.begin(), indices.end());
    return {distances, int_indices};
}

// ============================================================================
// saveToJson: 保存图为 JSON
// 对应 Python: TopologicalGraph.save_to_json()
// ============================================================================
void TopologicalGraph::saveToJson(const std::string& output_path) const {
    mkdirIfNotExists(output_path);

    json j;
    json vertices_json = json::array();

    for (int i = 0; i < static_cast<int>(vertices_.size()); ++i) {
        const auto& v = vertices_[i];

        // 保存栅格到子目录
        v.grid.save(output_path + "/" + std::to_string(i));

        // 构建顶点 JSON
        json v_json;
        v_json["pose_for_visualization"] = {
            v.pose_for_visualization[0],
            v.pose_for_visualization[1],
            v.pose_for_visualization[2]
        };

        // 描述符: 确保是一维
        std::vector<float> desc = v.descriptor;
        if (desc.size() > static_cast<size_t>(descriptor_dim_)) {
            desc.resize(descriptor_dim_);
        }
        std::vector<double> desc_double(desc.begin(), desc.end());
        v_json["descriptor"] = desc_double;

        vertices_json.push_back(v_json);
    }

    // 邻接表转 JSON
    json edges_json = json::array();
    for (const auto& adj_list : adj_lists_) {
        json adj_json = json::array();
        for (const auto& entry : adj_list) {
            json edge_entry = json::array();
            edge_entry.push_back(entry.vertex_id);
            edge_entry.push_back({entry.rel_pose[0], entry.rel_pose[1], entry.rel_pose[2]});
            adj_json.push_back(edge_entry);
        }
        edges_json.push_back(adj_json);
    }

    j["vertices"] = vertices_json;
    j["edges"] = edges_json;

    std::ofstream fout(output_path + "/graph.json");
    fout << j.dump(2);
    fout.close();

    ROS_INFO("Graph saved to %s (vertices: %d)", output_path.c_str(), numVertices());
}

// ============================================================================
// loadFromJson: 从 JSON 加载图
// 对应 Python: TopologicalGraph.load_from_json()
// ============================================================================
void TopologicalGraph::loadFromJson(const std::string& input_path) {
    // 读取 graph.json
    std::ifstream fin(input_path + "/graph.json");
    if (!fin.is_open()) {
        ROS_ERROR("Cannot open graph file: %s/graph.json", input_path.c_str());
        return;
    }
    json j;
    fin >> j;
    fin.close();

    // 解析顶点
    vertices_.clear();
    adj_lists_.clear();

    for (int i = 0; i < static_cast<int>(j["vertices"].size()); ++i) {
        const auto& v_json = j["vertices"][i];

        Vertex v;
        auto pose = v_json["pose_for_visualization"];
        v.pose_for_visualization = Pose2D(pose[0], pose[1], pose[2]);

        // 加载栅格
        v.grid = LocalGrid::load(input_path + "/" + std::to_string(i));

        // 加载描述符
        auto desc_json = v_json["descriptor"];
        v.descriptor.resize(desc_json.size());
        for (size_t k = 0; k < desc_json.size(); ++k) {
            v.descriptor[k] = desc_json[k].get<float>();
        }

        vertices_.push_back(std::move(v));

        // 添加到 FAISS 索引
        addToIndex(vertices_.back().descriptor);
    }

    // 解析邻接表
    for (const auto& adj_json : j["edges"]) {
        std::vector<AdjEntry> adj_list;
        for (const auto& entry_json : adj_json) {
            AdjEntry entry;
            entry.vertex_id = entry_json[0].get<int>();
            auto pose_json = entry_json[1];
            entry.rel_pose = Pose2D(pose_json[0], pose_json[1], pose_json[2]);
            adj_list.push_back(entry);
        }
        adj_lists_.push_back(adj_list);
    }

    ROS_INFO("Graph loaded (vertices: %d)", numVertices());
}

} // namespace prism_topomap
