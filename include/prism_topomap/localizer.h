/**
 * @file localizer.h
 * @brief 定位管理器类
 *
 * 对应原 Python 文件: scripts/localization.py
 * 管理异步定位流程: FAISS 检索 → 栅格配准 → 位姿估计
 * 通过 mutex 保护共享状态, 供主循环和定时器线程安全访问
 */
#pragma once

#include "prism_topomap/topological_graph.h"
#include "prism_topomap/local_grid.h"
#include "prism_topomap/inference_client.h"
#include "prism_topomap/utils.h"
#include <mutex>
#include <vector>
#include <string>

namespace prism_topomap {

// ============================================================================
// 定位输出状态
// ============================================================================
struct LocalizedState {
    std::vector<int> vertex_ids_matched;       // 匹配成功的顶点 ID
    std::vector<Pose2D> rel_poses;             // 各匹配顶点的相对位姿
    std::vector<int> vertex_ids_unmatched;     // 检索到但配准失败的顶点 ID
    Pose2D global_pose;                        // 定位时刻的全局位姿
    double timestamp = 0.0;                    // 定位时刻时间戳
};

// ============================================================================
// 定位管理器类
// ============================================================================
class Localizer {
public:
    /**
     * @brief 构造函数
     * @param graph               拓扑图引用
     * @param inference_client    Python 推理服务客户端
     * @param registration_score_threshold 配准分数阈值
     * @param top_k               FAISS 检索 top-k
     * @param save_dir            调试数据保存目录 (空=不保存)
     */
    Localizer(TopologicalGraph& graph,
              std::shared_ptr<InferenceClient> inference_client,
              double registration_score_threshold = 0.6,
              int top_k = 5,
              const std::string& save_dir = "",
              const FlowTraceConfig& trace_config = FlowTraceConfig());

    /**
     * @brief 执行一次定位
     * 对应 Python: Localizer.localize()
     *
     * 由 ROS Timer 异步调用:
     * 1. 获取当前快照 (descriptor, grid, timestamp)
     * 2. FAISS 检索 top-k 候选
     * 3. 对每个候选做栅格配准
     * 4. 写入定位结果
     */
    void localize();

    /**
     * @brief 更新当前观测状态 (线程安全)
     * 对应 Python: Localizer.update_current_state()
     *
     * 由主回调线程调用, 更新最新的描述符/栅格/位姿
     */
    void updateCurrentState(const Pose2D& global_pose,
                            const std::vector<float>& descriptor,
                            const LocalGrid& grid,
                            double timestamp,
                            int frame_id = -1,
                            bool trace_detailed = false);

    /**
     * @brief 获取最新定位结果 (线程安全)
     * 对应 Python: Localizer.get_localized_state()
     */
    LocalizedState getLocalizedState() const;

    // 统计计数
    int cnt = 0;
    int n_loc_fails = 0;
    double localized_stamp = 0.0;

private:
    // === 内部快照结构 ===
    struct Snapshot {
        Pose2D global_pose;
        std::vector<float> descriptor;
        LocalGrid grid;
        double timestamp;
        int frame_id;
        bool trace_detailed;
    };

    Snapshot getCurrentState();

    void writeLocalizedState(const std::vector<int>& matched_ids,
                             const std::vector<Pose2D>& rel_poses,
                             const std::vector<int>& unmatched_ids,
                             const Pose2D& global_pose,
                             double stamp);

    TopologicalGraph& graph_;
    std::shared_ptr<InferenceClient> inference_client_;
    double reg_score_threshold_;
    int top_k_;
    std::string save_dir_;
    FlowTraceConfig trace_config_;

    // 线程保护的共享状态
    mutable std::mutex mutex_;
    Pose2D global_pose_ = Pose2D::Zero();
    std::vector<float> descriptor_;
    LocalGrid grid_;
    double stamp_ = 0.0;
    int frame_id_ = -1;
    bool trace_detailed_ = false;
    bool initialized_ = false;

    // 定位输出
    LocalizedState localized_state_;
};

} // namespace prism_topomap
