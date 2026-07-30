/**
 * @file topological_graph.h
 * @brief 拓扑图类 (含 FAISS 索引)
 *
 * 对应原 Python 文件: scripts/topo_graph.py
 */
#pragma once

#include "prism_topomap/local_grid.h"
#include "prism_topomap/inference_client.h"
#include "prism_topomap/utils.h"
#include <faiss/IndexFlat.h>
#include <vector>
#include <string>
#include <memory>
#include <Eigen/Dense>

namespace prism_topomap {

// ============================================================================
// 拓扑图顶点
// ============================================================================
struct Vertex {
    Pose2D pose_for_visualization;   // 全局可视化位姿 [x, y, theta]
    LocalGrid grid;                   // 局部栅格
    std::vector<float> descriptor;    // 位置识别描述符 (256 or 512 维)
};

// ============================================================================
// 邻接表项
// ============================================================================
struct AdjEntry {
    int vertex_id;
    Pose2D rel_pose;    // [x, y, theta] — 从当前顶点到 vertex_id 的相对位姿
};

// ============================================================================
// 配准结果
// ============================================================================
struct TransformResult {
    bool success;
    bool service_success = false;
    double x, y, theta;
    double score = 0.0;
    double trans_i = 0.0;
    double trans_j = 0.0;
    double rot_angle = 0.0;
    double elapsed_ms = 0.0;
};

// ============================================================================
// 路径结果
// ============================================================================
struct PathResult {
    bool found;
    std::vector<int> path;
    double length;
};

// ============================================================================
// 拓扑图类
// ============================================================================
class TopologicalGraph {
public:
    /**
     * @brief 构造函数
     * @param inference_client   Python 推理服务客户端
     * @param inline_reg_score_threshold  沿边配准分数阈值
     * @param grid_resolution    栅格分辨率
     * @param grid_radius        栅格半径
     * @param max_grid_range     最大点云范围
     * @param descriptor_dim     描述符维度 (256=MinkLoc3D, 512=MSSPlace)
     */
    TopologicalGraph(std::shared_ptr<InferenceClient> inference_client,
                     double inline_reg_score_threshold = 0.5,
                     double grid_resolution = 0.1,
                     double grid_radius = 18.0,
                     double max_grid_range = 8.0,
                     int descriptor_dim = 256);

    // === 顶点操作 ===
    int addVertex(const Pose2D& global_pose,
                  const std::vector<float>& descriptor,
                  const LocalGrid& grid);
    const Vertex& getVertex(int vertex_id) const;

    // === 边操作 ===
    void addEdge(int i, int j, double x, double y, double theta);
    bool hasEdge(int u, int v) const;
    Pose2D getEdge(int u, int v) const;
    const std::vector<AdjEntry>& getEdgesFrom(int u) const;

    // === 位姿逆变换 ===
    static Pose2D inverseTransform(double x, double y, double theta);

    // === 沿边配准 (通过 Python 推理服务) ===
    TransformResult getTransformToVertex(int vertex_id, const LocalGrid& grid);

    // === Dijkstra 最短路 ===
    PathResult getPathWithLength(int u, int v) const;

    // === FAISS 索引操作 ===
    void addToIndex(const std::vector<float>& descriptor);
    std::pair<std::vector<float>, std::vector<int>>
    searchIndex(const std::vector<float>& query, int top_k) const;

    // === 序列化 ===
    void saveToJson(const std::string& output_path) const;
    void loadFromJson(const std::string& input_path);

    // === 访问接口 ===
    int numVertices() const { return static_cast<int>(vertices_.size()); }
    int indexSize() const { return static_cast<int>(faiss_index_->ntotal); }
    int undirectedEdgeCount() const {
        size_t directed_count = 0;
        for (const auto& entries : adj_lists_) directed_count += entries.size();
        return static_cast<int>(directed_count / 2);
    }
    double inlineRegistrationThreshold() const { return inline_reg_score_threshold_; }
    const std::vector<Vertex>& vertices() const { return vertices_; }
    const std::vector<std::vector<AdjEntry>>& adjLists() const { return adj_lists_; }

private:
    std::vector<Vertex> vertices_;
    std::vector<std::vector<AdjEntry>> adj_lists_;

    // FAISS 索引 (C++ 原生)
    std::unique_ptr<faiss::IndexFlatL2> faiss_index_;
    int descriptor_dim_;

    // Python 推理服务客户端
    std::shared_ptr<InferenceClient> inference_client_;

    // 配准参数
    double inline_reg_score_threshold_;
    double grid_resolution_;
    double grid_radius_;
    double max_grid_range_;
};

} // namespace prism_topomap
