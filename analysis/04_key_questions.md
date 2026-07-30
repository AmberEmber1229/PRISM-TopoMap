# PRISM-TopoMap — Key Design Questions & Answers

## Q1: 为什么要设计两种架构（纯Python + C++/Python混合）？

**答案**: 这是从纯 Python 原型向生产级 C++ 系统的演进过程。

- **纯 Python 架构** 是原始论文的原型实现 [VERIFY: scripts/prism_topomap_node.py]，开发迭代快，所有算法（含深度学习推理）在同一个 Python 进程中，调试方便
- **混合架构** 将计算密集的主循环（里程计积分、点云投影、Dijkstra）移到 C++ [VERIFY: CMakeLists.txt:130-166]，仅将深度推理（需要 PyTorch/MinkowskiEngine）留在 Python [VERIFY: scripts/inference_service_node.py]

两种架构共享相同的算法逻辑和配置文件格式，纯 Python 版本可以用于验证混合架构的 C++ 实现是否等价。

**设计权衡:**

| 维度 | 纯 Python | C++/Python 混合 |
|------|----------|------------------|
| 开发速度 | 快 | 较慢（需要编译） |
| 运行性能 | 受 GIL 限制 | 主循环无 GIL |
| 部署复杂度 | 低 | 中（两个进程） |
| 调试难度 | 低 | 中（跨进程通信） |
| 依赖管理 | pip/conda | catkin + pip/conda |

---

## Q2: 为什么定位模块是异步的？

**答案**: 主循环需要高频（~10Hz）运行以保持栅格的实时更新和里程计积分，而定位模块中的深度学习推理（栅格配准）可能需要几十到几百毫秒。

[VERIFY: prism_topomap_node.py:527]

```python
rospy.Timer(rospy.Duration(self.localization_frequency), self.localize)
```

`localization_frequency` 典型值 0.5-2.0 Hz（每 0.5-2 秒一次定位）。

**后果**: 
- 定位结果始终有延迟（"localization_is_fresh" 检查）
- 主循环用 `localization_is_fresh` 位判断结果是否比最新采集的数据新
- 定位可能在机器人已经移动后返回，需要通过 `rel_pose_vcur_to_loc` 对齐时间戳 [VERIFY: prism_topomap.py:540]

---

## Q3: 为什么用 IoU 来判断节点切换而不是纯距离？

**答案**: 纯距离无法反映实际的环境重叠程度。例如在走廊中，机器人可能走了很远但视觉特征仍然相似；在房间中，可能走了几米就出现了新的不重叠区域。

[VERIFY: prism_topomap.py:566-582]

IoU 基于局部占据栅格的重叠度，直接反映了"两个视角看到的是否是同一区域"：

```
当 IoU < iou_threshold (典型值 0.5-0.8):
  → 当前观测与节点观测差异大
  → 可能进入了新区域
  → 需要创建新节点或切换到其他已有节点
```

节点切换使用三个条件（三者取一即触发）:
1. `!isInsideVcur()`: 移出已知区域
2. `iou < iou_threshold`: 观测重叠不足
3. `distance > max_edge_length`: 离中心太远

---

## Q4: reattachByEdge 和 reattachByLocalization 有什么区别？

**答案**: 它们解决不同的切换场景。

| 方法 | 触发时机 | 搜索范围 | 配准模型 | 用途 |
|------|---------|---------|---------|------|
| `reattachByEdge` | 每次 `update()` 调用 | 仅当前节点的邻居 | 沿边配准模型 (inline) | 沿已知路径在相邻节点间平滑切换 |
| `reattachByLocalization` | 判定需要换节点后 | 定位返回的所有匹配节点 | 全局定位配准模型 | 跳转到图中任意匹配节点（回环恢复） |

[VERIFY: topo_slam_model.cpp:720-780]

**优先级**: `reattachByEdge` 先于 `reattachByLocalization`，因为它利用已知的图拓扑结构，搜索结果更可靠。

---

## Q5: 沿边配准和全局定位配准有何不同？

**答案**: 两套配准模型使用不同的配置和阈值，针对不同的初始对齐质量。

[VERIFY: prism_topomap.py:131-149]

| 配置项 | 全局定位配准 | 沿边配准 |
|--------|------------|---------|
| 配置段 | `scan_matching` | `scan_matching_along_edge` |
| 分数阈值 | `registration_score_threshold` (典型 0.6) | `inline_registration_score_threshold` (典型 0.5) |
| 用途 | 从 top-k FAISS 候选中筛选 | 验证沿边切换的位姿 |
| 特征检测器 | 可配 ORB/SIFT/Harris | 推荐 `HarrisWithDistance` |
| 额外检查 | 无 | `jump_threshold` (防止跳变过大) |

沿边配准的初始值来自图结构中已有的边位姿，更准确；全局配准则从零开始搜索。

---

## Q6: drift_coef 参数的作用是什么？

**答案**: 防止里程计漂移导致定位位姿跳变过大，产生错误的节点切换。

[VERIFY: prism_topomap.py:369-372]

```python
dst > self.drift_coef * (self.current_stamp - self.last_successful_match_time) + 10
```

公式解释：

```
允许最大距离 = drift_coef × Δt + 10 (米)
其中:
  Δt = 自上次成功匹配以来的时间
  drift_coef = 里程计漂移系数 (典型: 0.2 慢速机器人, 1.0 汽车)
  10 米 = 基础容差
```

物理意义: 如果里程计声称机器人移动了 X 米, 那么实际位置最多偏离 `drift_coef × 时间 + 10` 米。如果定位给出的位姿偏离超过这个范围, 视为错误匹配。

---

## Q7: 为什么栅格变换里 theta 要取反？

**答案**: 栅格的仿射变换是"逆向"的——机器人向前移动意味着环境相对机器人向后移动。

[VERIFY: scripts/local_grid.py:128]

```python
tf_matrix = np.array([
    [np.cos(-theta), np.sin(-theta), y / self.resolution],
    [-np.sin(-theta), np.cos(-theta), x / self.resolution],
    [0, 0, 1]
])
```

注意 `tf_matrix` 中使用 `-theta`:
- 如果机器人向前移动 (机器人坐标系x增加)，环境向后移动 → 栅格反向平移
- 如果机器人逆时针转动 (+θ)，环境顺时针转动 → 栅格反向旋转

同样, `grid_shift` 的计算使用 `from=NEW, to=OLD` 方向（反向）[VERIFY: topo_slam_model.cpp:623-625]。

---

## Q8: FAISS 为什么使用暴力搜索 IndexFlatL2 而不是近似搜索？

**答案**: 拓扑图的规模通常在几十到几百个节点，使用 L2 距离的暴力搜索已经足够快。

[VERIFY: scripts/models.py:20]

```python
index = faiss.IndexFlatL2(256)  # MinkLoc3D: 256维
index = faiss.IndexFlatL2(512)  # MSSPlace: 512维
```

**复杂度分析**:
- 暴力搜索: O(N × dim)，N = 节点数, dim = 256/512
- 近似搜索 (如 IVF): O(log N × dim)，但有召回损失
- 当 N < 1000 时，暴力搜索延迟 < 1ms，近似搜索的优势不明显
- 暴力搜索保证 100% 召回率，对定位可靠性至关重要

---

## Q9: 节点创建时的描述符来自哪一帧？

**答案**: 来自触发 `addNewVertex` 调用的那一帧。

[VERIFY: prism_topomap.py:397-398]

```python
def add_new_vertex(self, vertex_ids, rel_poses):
    new_vertex_id = self.graph.add_vertex(self.global_pose_for_visualization, 
                                          self.cur_desc,  # 当前帧的描述符
                                          self.cur_grid)  # 当前帧的栅格
```

`cur_desc` 和 `cur_grid` 由 `processObservations()` 在每帧更新 [VERIFY: prism_topomap.py:488-494]。这意味着新节点的描述符和栅格来自"此刻看到的环境"，而不是累积或平滑后的结果。

---

## Q10: 定位失败时系统如何降级？

**答案**: 系统有多层降级策略。

**第1层**: 定位超时（仅在 mapping 模式初始定位阶段）:
```python
if time.time() - start_time > self.localization_timeout:
    add_new_vertex([], [])  # 创建新节点，不连边
```
[VERIFY: prism_topomap.py:436-439]

**第2层**: 定位旧数据:
- `localization_is_fresh = False` → 跳过回环检测 [VERIFY: prism_topomap.py:555-556]
- 如果定位 5 秒未更新 → mapping 模式直接加新节点，localization 模式报警 [VERIFY: prism_topomap.py:600-605]

**第3层**: 无配准匹配:
- `reattachByEdge(require_match=False)` → 仅用图结构（边的相对位姿）切换，不做配准验证 [VERIFY: prism_topomap.py:608-609]

**第4层**: localization 模式的最终兜底:
- 如果所有其他方法都失败，`reattachByEdge(require_match=False)` 作为最终尝试 [VERIFY: topo_slam_model.cpp:775-778]

---

## Verification Checklist

- [x] Q1: 两种架构对应 CMakeLists.txt 中的两个构建目标 [VERIFY: CMakeLists.txt:130-166]
- [x] Q2: 异步定位 Timer [VERIFY: prism_topomap_node.py:527]
- [x] Q3: IoU/距离/inside 三条件 [VERIFY: prism_topomap.py:573-574]
- [x] Q4: 两种切换方法 [VERIFY: topo_slam_model.cpp:720-780]
- [x] Q5: 两套配准配置 [VERIFY: prism_topomap.py:131-149]
- [x] Q6: drift_coef 含义 [VERIFY: prism_topomap.py:370]
- [x] Q7: theta 取反 [VERIFY: scripts/local_grid.py:129]
- [x] Q8: FAISS 索引维度 [VERIFY: scripts/models.py:20,30]
- [x] Q9: 节点创建使用 cur_desc [VERIFY: prism_topomap.py:398]
- [x] Q10: 降级策略 [VERIFY: prism_topomap.py:436-439, 555-556, 600-609]

*分析时间: 2026-05-06 | 项目版本: 0.0.0 | 分支: main*
