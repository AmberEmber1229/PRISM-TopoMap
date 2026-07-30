# PRISM-TopoMap Code Review Report — Python vs C++ 对比

## 1. Data Structure Audit

### 1.1 Vertex (LocationNode)

**Python**: [scripts/topo_graph.py:61-74](scripts/topo_graph.py#L61-L74)
```python
vertex_dict = {
    'pose_for_visualization': [x, y, theta],
    'grid': grid.copy(),         # LocalGrid 深拷贝
    'descriptor': descriptor     # np.ndarray (256 or 512,)
}
self.vertices.append(vertex_dict)
```

**C++**: [include/prism_topomap/topological_graph.h:23-27](include/prism_topomap/topological_graph.h#L23-L27)
```cpp
struct Vertex {
    Pose2D pose_for_visualization;   // Eigen::Vector3d [x, y, theta]
    LocalGrid grid;                   // cv::Mat 栅格层
    std::vector<float> descriptor;    // 256 or 512 dims
};
```

| 要求 | Python | C++ | 一致性 |
|------|--------|-----|--------|
| NodeID | `vertices` list index (隐式) | `vertices_` vector index (隐式) | 一致 |
| Descriptor | `np.ndarray` 256/512 floats | `std::vector<float>` 256/512 | 一致 |
| Scan2D | `grid.copy()` → `LocalGrid` with `layers` dict of `np.ndarray` | `grid.copy()` → `LocalGrid` with `layers_` map of `cv::Mat` | 一致 |
| 3D 点云缓存 | 无 — `update_from_cloud_and_transform()` 后仅保留栅格 | 同 Python | 一致 |

**Scan2D 流程对比**:
- Python [scripts/local_grid.py:73-106](scripts/local_grid.py#L73-L106): `update_from_cloud_and_transform()` → `transform()` → 去顶去底 → `round(xy / resolution)` → `raycast_grid()` → occupancy/density/height 更新
- C++ [src/local_grid.cpp](src/local_grid.cpp): `updateFromCloudAndTransform()` → `transform()` → `removeFloorAndCeil()` → 坐标投影 → `raycastGrid()` → 同

**一点差异**: Python 的 `grid.copy()` 复制 `layers` dict 中全部层；C++ 的 `LocalGrid::copy()` 也复制全部层。两者行为一致，但都有冗余 — 见 §1.3。

### 1.2 Edge

**Python**: [scripts/topo_graph.py:103-112](scripts/topo_graph.py#L103-L112)
```python
def add_edge(self, i, j, x, y, theta):
    self.adj_lists[i].append((int(j), [x, y, theta]))
    self.adj_lists[j].append((int(i), self.inverse_transform(x, y, theta)))
```
边存储为 `(target_id, [x, y, theta])` 元组，邻接表 `list` of `list`。

**C++**: [include/prism_topomap/topological_graph.h:32-35](include/prism_topomap/topological_graph.h#L32-L35) + [src/topological_graph.cpp:102-120](src/topological_graph.cpp#L102-L120)
```cpp
struct AdjEntry { int vertex_id; Pose2D rel_pose; };
// addEdge 双向添加，同 Python
adj_lists_[i].push_back({j, Pose2D(x, y, theta)});
adj_lists_[j].push_back({i, inverseTransform(x, y, theta)});
```

| 要求 | Python | C++ | 一致性 |
|------|--------|-----|--------|
| SourceID | 邻接表 index 隐式 | 同 | 一致 |
| TargetID | `j` (int) | `vertex_id` (int) | 一致 |
| RelativePose (2D) | `[x, y, theta]` | `Pose2D` [x, y, theta] | 一致 |
| 无向边 | 双向添加 + `inverse_transform` | 同 | 一致 |

### 1.3 内存评估

| 组件 | 单顶点 | 100 顶点 | 节点创建后是否使用 |
|------|--------|---------|------------------|
| descriptor | 1 KB | 100 KB | FAISS 检索 |
| occupancy (360×360 uint8) | 130 KB | 13 MB | **IoU, 配准** |
| density_map (360×360 float) | 518 KB | 52 MB | 仅可视化 |
| height_map (360×360 float) | 518 KB | 52 MB | 不参与任何算法 |
| curbs (360×360 uint8) | 130 KB | 13 MB | 仅户外场景 |

**Python 和 C++ 都存储全部 4-5 层**。在 `add_vertex` / `addVertex` 时调用 `grid.copy()`，完整复制所有 layers。

**建议**: 两个版本均可考虑在 `addVertex` 时仅持久化 `occupancy` 层 + `descriptor`，将每顶点内存从 1.3MB 降至 131KB（10× 降低）。

---

## 2. State Machine Flow Audit — Python vs C++ 逐级对比

### 2.1 论文定义的优先级

```
Priority 1: 原地更新     — IoU > threshold → stay, T_cur = T_cur_old * o_t
Priority 2: 邻居切换     — P1 failed, ORB match with neighbors → switch
Priority 3: 触发回环     — P2 failed, scan match with V_loc_candidates → switch + edge
Priority 4: 创建新节点   — all failed → new node + connect to prev + localized
```

### 2.2 实际执行顺序

**Python** [scripts/prism_topomap.py:516-605](scripts/prism_topomap.py#L516-L605) 和 **C++** [src/topo_slam_model.cpp:639-807](src/topo_slam_model.cpp#L639-L807) **执行顺序完全一致**：

| 步骤 | Python | C++ | 对应优先级 |
|------|--------|-----|-----------|
| 1. 里程计积分 | `get_rel_pose(*cur_odom_pose, *self.odom_pose)` → `update_rel_pose_of_vcur_by_odom()` | `getRelPose(cur_odom_pose, odom_pose_)` → `updateRelPoseByOdom()` | 前置 |
| 2. 观测处理 | `process_observations()` | `processObservations()` | 前置 |
| 3. 回环检测 | `find_loop_closure()` → 如果检测到 → `add_new_vertex()` + return | `findLoopClosure()` → 如果检测到 → `addNewVertex()` + return | **独立于 1-4** |
| 4. 沿边切换 | `changed = reattach_by_edge(require_match=True)` | `changed = reattachByEdge(true)` | **Priority 2 先执行** |
| 5. IoU/距离检查 | `is_inside_vcur()` + `get_iou()` + `dst > max_edge_length` | `isInsideVcur()` + `getIoU()` + `rel_dist > max_edge_length_` | **Priority 1 在此检查** |
| 6. 定位切换 | `reattach_by_localization(iou, timestamp)` | `reattachByLocalization(iou, timestamp)` | Priority 3 |
| 7. 创建新节点 | `add_new_vertex(vertex_ids, rel_poses)` | `addNewVertex(vertex_ids, rel_poses)` | Priority 4 |

**关键发现**: Python 也是 **Priority 2 → 1 → 3 → 4** 的执行顺序，C++ 完全复制了 Python 的行为。这不是 C++ 的 bug，而是 Python 原版的设计选择。

### 2.3 Priority 1: 原地更新 — 逐行对比

**Python** [scripts/prism_topomap.py:560-568](scripts/prism_topomap.py#L560-L568):
```python
inside_vcur = self.is_inside_vcur()
iou = self.cur_grid.get_iou(self.last_vertex['grid'],
    *self.graph.inverse_transform(*self.rel_pose_of_vcur), save=False, cnt=self.iou_cnt)
dst = np.sqrt(self.rel_pose_of_vcur[0] ** 2 + self.rel_pose_of_vcur[1] ** 2)
if not inside_vcur or iou < self.iou_threshold or dst > self.max_edge_length:
    self.need_to_change_vcur = True
```

**C++** [src/topo_slam_model.cpp:779-783](src/topo_slam_model.cpp#L779-L783):
```cpp
bool inside_vcur = isInsideVcur();
cur_iou_ = cur_grid_.getIoU(graph_.getVertex(last_vertex_id_).grid,
    inv_rel_pose[0], inv_rel_pose[1], inv_rel_pose[2], false, iou_cnt_++);
double rel_dist = std::sqrt(rel_pose_of_vcur_[0]*rel_pose_of_vcur_[0] + ...);
if (!inside_vcur || cur_iou_ < iou_threshold_ || rel_dist > max_edge_length_) {
    need_to_change_vcur_ = true;
```

**完全一致**。三个条件（inside + IoU + distance）完全相同，`inverse_transform` 预处理也相同。

**与论文的差异**: 论文说 Priority 1 "成功则 stay, T_cur *= o_t"，但代码中 T_cur 的更新（里程计积分）在第 1 步就已经完成，不管 Priority 1 是否成功。论文的表述重点在于"原地更新意味着不切换节点"，这正是 `need_to_change_vcur = False` 时的行为。

### 2.4 Priority 2: 邻居切换 — 逐行对比

**Python** [scripts/prism_topomap.py:285-345](scripts/prism_topomap.py#L285-L345):
```python
def reattach_by_edge(self, require_match=True):
    for vertex_id, pose_to_vertex in self.graph.adj_lists[self.last_vertex_id]:
        pose_diff = sqrt((e_x - T_x)^2 + (e_y - T_y)^2)  # T_cur vs edge_pose
    if min(pose_diffs) < dist_to_vcur and min(pose_diffs) < 5:
        nearest_vertex_id = neighbours[argmin(pose_diffs)]
    
    # 栅格对齐到候选节点坐标系 → 配准
    cur_grid_transformed = self.cur_grid.copy()
    cur_grid_transformed.transform(inv(rel_pose_to_vertex))
    corr_x, corr_y, corr_theta = self.graph.get_transform_to_vertex(
        nearest_vertex_id, cur_grid_transformed)
    if corr_x is not None and diff < self.local_jump_threshold:
        self.rel_pose_of_vcur = self.graph.inverse_transform(x, y, theta)
        self.last_vertex_id = nearest_vertex_id
```

**C++** [src/topo_slam_model.cpp:290-376](src/topo_slam_model.cpp#L290-L376):
```cpp
bool TopoSLAMModel::reattachByEdge(bool require_match) {
    for (const auto& entry : edges) {
        double dist = sqrt((e_x - T_x)^2 + (e_y - T_y)^2);
    }
    if (min_dist < dist_to_vcur && min_dist < 5.0 && nearest_vertex_id >= 0) {
        // same logic
    }
    // 栅格对齐 → 配准 → inverseTransform → 切换
}
```

**完全一致**，包括:
- 邻居遍历和距离计算
- 硬编码的 5.0m 距离上限 (两版本都有)
- 栅格变换 + `getTransformToVertex` 配准流程
- `local_jump_threshold` 跳变检查
- `require_match=false` 的无条件切换兜底

**配准方式**: 两版本都使用栅格配准（occupancy grid registration），通过 `inline_registration_pipeline`（config 中配置为 `feature2d` + detector）。论文提到 ORB 匹配，代码中 config 推荐 `HarrisWithDistance`。不是纯 ORB，但同属 2D 特征配准范畴。

### 2.5 Priority 3: 定位回环 — 逐行对比

**Python** [scripts/prism_topomap.py:347-395](scripts/prism_topomap.py#L347-L395):
```python
def reattach_by_localization(self, iou_threshold, localized_stamp):
    for i, v in enumerate(vertex_ids):
        pred_rel_pose_vcur_to_v = apply_pose_shift(
            self.rel_pose_vcur_to_loc, inverse(rel_poses[i]))
        iou = self.cur_grid.get_iou(vertex.grid, rel_pose_robot_to_loc)
        if dst > drift_coef * (dt) + 10:  # 漂移检查
            continue
        if iou > iou_threshold or self.need_to_change_vcur:
            if self.mode == 'mapping':
                self.graph.add_edge(self.last_vertex_id, v, *pred_rel_pose)
            self.last_vertex_id = v
            self.rel_pose_of_vcur = apply_pose_shift(rel_poses[i], rel_after_loc)
            return True
```

**C++** [src/topo_slam_model.cpp:382-451](src/topo_slam_model.cpp#L382-L451):
```cpp
bool TopoSLAMModel::reattachByLocalization(double iou_threshold_val, double localized_stamp) {
    for (size_t i = 0; i < n; ++i) {
        Pose2D inv_loc_rel = graph_.inverseTransform(loc_rel);
        Pose2D pred_rel_pose_vcur_to_v = applyPoseShift(rel_pose_vcur_to_loc_, inv_loc_rel);
        double iou = cur_grid_.getIoU(graph_.getVertex(vid).grid, ...);
        if (dst > drift_coef_ * (dt) + 10.0) continue;
        if (iou > iou_threshold_val || need_to_change_vcur_) {
            if (mode_ == "mapping") graph_.addEdge(last_vertex_id_, vid, ...);
            last_vertex_id_ = vid;
            rel_pose_of_vcur_ = applyPoseShift(loc_rel, rel_after_localization);
            return true;
        }
    }
}
```

**完全一致**。包括漂移检查公式 `drift_coef * Δt + 10`、IoU 阈值检查、mapping 模式下添加边的条件。

**一个共同的问题**: `add_edge` / `addEdge` 在此处**没有距离一致性校验**。定位给出的 `pred_rel_pose` 可能和两个顶点的全局坐标距离不一致，导致错误边。

### 2.6 Priority 4: 创建新节点 — 逐行对比

**Python** [scripts/prism_topomap.py:397-418](scripts/prism_topomap.py#L397-L418):
```python
def add_new_vertex(self, vertex_ids, rel_poses):
    new_id = self.graph.add_vertex(global_pose_for_visualization, cur_desc, cur_grid)
    pose_stamped, new_rel_pose = self.get_rel_pose_from_stamp(current_stamp)
    if self.last_vertex is not None:
        self.graph.add_edge(last_vertex_id, new_id, *pose_stamped)
    self.rel_pose_of_vcur = new_rel_pose
    # 定位回环边
    for v, rel_pose in zip(vertex_ids, rel_poses):
        if self.rel_pose_vcur_to_loc is not None and rel_pose is not None:
            pred_rel_pose = apply_pose_shift(
                self.rel_pose_vcur_to_loc, inverse(rel_pose))
            self.graph.add_edge(new_id, v, *pred_rel_pose)
```

**C++** [src/topo_slam_model.cpp:457-508](src/topo_slam_model.cpp#L457-L508):
```cpp
void TopoSLAMModel::addNewVertex(const vector<int>& vertex_ids, const vector<Pose2D>& rel_poses) {
    int new_id = graph_.addVertex(global_pose_for_visualization_, cur_desc_, cur_grid_);
    Pose2D pose_stamped = getRelPoseFromStamp(current_stamp_);
    if (last_vertex_id_ >= 0) {
        graph_.addEdge(last_vertex_id_, new_id, pose_stamped[0], ...);
    }
    rel_pose_of_vcur_ = new_rel_pose_of_vcur;
    for (size_t i = 0; i < n; ++i) {
        Pose2D inv_rel = graph_.inverseTransform(rel_poses[i]);
        Pose2D pred_rel_pose = applyPoseShift(rel_pose_vcur_to_loc_, inv_rel);
        graph_.addEdge(new_id, vid, pred_rel_pose[0], ...);
    }
}
```

**完全一致**。两个版本的核心逻辑完全相同：
1. 用当前 global_pose + desc + grid 创建新节点
2. 从 rel_poses_stamped 历史中计算 pose_stamped（作为与上一节点的边）
3. 遍历定位匹配结果，用 `rel_pose_vcur_to_loc` 计算回环边的预测位姿

**共同缺陷**: 步骤 3 添加回环边时**没有距离一致性校验**。Python 的注释 `#if np.sqrt(pred_rel_pose[0] ** 2 + pred_rel_pose[1] ** 2) < 5:` 被注释掉了，C++ 也没有实现。

### 2.7 findLoopClosure — 独立的回环检测

两版本在节点切换之前（Step F）都有一个独立的 `findLoopClosure`：

**Python** [scripts/prism_topomap.py:228-255](scripts/prism_topomap.py#L228-L255):
```python
def find_loop_closure(self, vertex_ids, dists):
    for i in range(len(vertex_ids)):
        for j in range(len(vertex_ids)):
            path, path_len = self.graph.get_path_with_length(u, v)
            if path_len > 5 and path_len > 2 * dst_through_cur and check_path_condition(u, v):
                self.add_new_vertex(vertex_ids, rel_poses)  # 创建新节点闭合回环
                return True
```

**C++** [src/topo_slam_model.cpp:238-267](src/topo_slam_model.cpp#L238-L267):
```cpp
bool findLoopClosure(const vector<int>& vertex_ids, const vector<double>& dists) {
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < n; ++j)
            if (path_len > 5.0 && path_len > 2.0 * dst_through_cur && checkPathCondition(u, v))
                addNewVertex(vertex_ids, rel_poses);  // 同
```

**完全一致**。判断条件 `path_len > 5 && path_len > 2 * dst_through_cur` 相同。触发后调用 `addNewVertex` 而非直接连边 — 在回环位置创建一个新节点。

---

## 3. Summary

### 3.1 数据结构对比

| 检查项 | Python | C++ | 一致性 |
|--------|--------|-----|--------|
| Vertex: descriptor (非 3D 点云) | `np.ndarray` | `std::vector<float>` | 一致 |
| Vertex: Scan2D (2D 栅格) | `LocalGrid.layers` dict | `LocalGrid.layers_` map | 一致 |
| Edge: target + rel_pose(2D) | `(int, [x,y,theta])` | `AdjEntry{int, Pose2D}` | 一致 |
| 3D 点云不缓存 | 投影后丢弃 | 同 | 一致 |
| 全部栅格层深拷贝 | `grid.copy()` 全层 | `grid.copy()` 全层 | 一致（有冗余） |

### 3.2 状态机流程对比

| 步骤 | Python | C++ | 一致性 |
|------|--------|-----|--------|
| 执行顺序 | 回环检测 → Priority 2 → 1 → 3 → 4 | 同 | **完全一致** |
| Priority 1: 三条件 | inside + IoU + distance | 同 | 完全一致 |
| Priority 2: 邻居距离上限 | 硬编码 `5` | 硬编码 `5.0` | 完全一致（都有同样问题） |
| Priority 2: 配准流程 | transform → get_transform_to_vertex | 同 | 完全一致 |
| Priority 3: 漂移检查 | `drift_coef * dt + 10` | 同 | 完全一致 |
| Priority 3: 边添加无校验 | 注释 `#if ... < 5:` 被注释 | 无校验 | 完全一致（都有同样缺陷） |
| Priority 4: 回环边添加 | `apply_pose_shift(rel_pose_vcur_to_loc, inv(rel))` | 同 | 完全一致 |
| findLoopClosure | `path_len > 5 && > 2*dst` | 同 | 完全一致 |

### 3.3 与论文的差异（两版本共有）

| 差异 | Python | C++ | 说明 |
|------|--------|-----|------|
| 执行顺序 2→1→3→4 而非 1→2→3→4 | 是 | 是 | 先尝试沿边切换，再检查是否还在当前节点 |
| 邻居距离上限硬编码 5 | 是 | 是 | 应与 `max_edge_length` 同步 |
| 定位回环边无距离校验 | 是 | 是 | `add_edge` 调用前无距离一致性检查 |

### 3.4 改进建议

| # | 建议 | 位置 | 影响 |
|---|------|------|------|
| 1 | addNewVertex 回环边添加距离一致性校验 | C++ topo_slam_model.cpp:498 / Python prism_topomap.py:414 | 防止错误回环边 |
| 2 | 仅持久化 occupancy 层 | addVertex 两个版本 | 每顶点内存 1.3MB→131KB |
| 3 | reattachByEdge 中硬编码 5.0 应引用 max_edge_length | 两个版本 | 配置一致性 |
| 4 | `findLoopClosure` 中硬编码 5.0 和 2.0 也应参数化 | 两个版本 | 可调参 |

**总结**: C++ 版本是 Python 版本的**高保真翻译**。所有算法逻辑（包括执行顺序、条件检查、hard-coded 常量、以及已知缺陷）都完全一致。与论文的偏差源自 Python 原版的设计选择，C++ 正确地复制了这些选择。
