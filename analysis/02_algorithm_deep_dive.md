# PRISM-TopoMap — Algorithm Deep Dive

## 目录

1. [SLAM 主循环 (`update()`)](#1-slam-主循环-update)
2. [点云观测处理 (`processObservations`)](#2-点云观测处理-processobservations)
3. [里程计积分 (`updateRelPoseByOdom`)](#3-里程计积分-updaterelposebyodom)
4. [定位管道 (Localizer::localize)](#4-定位管道-localizerlocalize)
5. [回环检测 (`findLoopClosure`)](#5-回环检测-findloopclosure)
6. [节点切换决策树](#6-节点切换决策树)
7. [Dijkstra 最短路径](#7-dijkstra-最短路径)

---

## 1. SLAM 主循环 (`update`)

### 1.1 函数签名

**Python**: [prism_topomap.py:516-609](scripts/prism_topomap.py)
**C++**: [topo_slam_model.cpp:606-790](src/topo_slam_model.cpp)

### 1.2 伪代码

```
Algorithm: SLAM Main Update Loop
Input: global_pose, cur_odom_pose, cloud_msg, cur_cloud, images, cur_curbs
Output: 更新拓扑图状态 (节点切换/新增/回环闭合)

  Step A: 里程计积分
    grid_shift = getRelPose(cur_odom_pose, odom_pose_)  // from NEW to OLD
    updateRelPoseByOdom(cur_odom_pose)
    rel_pose_of_vcur_ += odom_increment

  Step B: 观测处理
    processObservations(...)  // 描述符提取 + 栅格更新

  Step C: 更新定位器快照
    localizer.updateCurrentState(global_pose, cur_desc, cur_grid, timestamp)
    rel_poses_stamped_.push_back({timestamp, rel_pose_of_vcur_})

  Step D: 初始定位 (仅首帧, localization模式)
    if last_vertex_id_ < 0:
      if mode == "localization": initLocalization(); return
      else: addNewVertex({}, {}); return   // 建第一个节点

  Step E: 获取定位结果
    localization_results_ = localizer.getLocalizedState()
    localization_is_fresh = (timestamp >= last_rel_pose_time)

  Step F: 回环检测 (mapping模式 + 定位新鲜)
    if mode == "mapping" and localization_is_fresh:
      vertex_ids = [matched_ids] + [last_vertex_id]  (如果不在列表中)
      dists = [√(x²+y²) for each rel_pose]
      if findLoopClosure(vertex_ids, dists):
        addNewVertex(vertex_ids, rel_poses); return

  Step G: 节点切换判定
    changed = reattachByEdge(require_match=true)

    // IoU和距离判定
    cur_iou = cur_grid.getIoU(current_vertex.grid, inv_rel_pose)
    inside_vcur = current_vertex.grid.isInside(rel_pose_of_vcur_)
    rel_dist = √(rel_pose_of_vcur_.x² + rel_pose_of_vcur_.y²)

    if (!inside_vcur || cur_iou < iou_threshold || rel_dist > max_edge_length):
      need_to_change_vcur = true
      if !changed:
        if localization_is_fresh:
          changed = reattachByLocalization(cur_iou, timestamp)
          if !changed and mode == "mapping":
            addNewVertex(matched_ids, rel_poses)
        else:
          if mode == "mapping": addNewVertex({}, {})
          else: warn("No recent localization")
      if !changed and mode == "localization":
        reattachByEdge(require_match=false)  // 无条件切换
```

### 1.3 逐步骤详解

#### Step A: 里程计积分
[VERIFY: topo_slam_model.cpp:618-631]

```cpp
// grid_shift: 从 OLD→NEW 的反向变换 (因为栅格需要逆向移动)
Pose2D grid_shift = Pose2D::Zero();
if (odom_initialized_) {
    grid_shift = getRelPose(cur_odom_pose, odom_pose_);  // from=NEW, to=OLD
}
updateRelPoseByOdom(cur_odom_pose);
```

`grid_shift` 用于栅格的仿射变换（栅格坐标系固定，需要"反向移动"来补偿机器人运动）。注意 Python 中 `process_observations` 传入的 theta 取反：`-theta` [prism_topomap.py:492]。

#### Step G (核心): 节点切换判定

这是整个 SLAM 系统最复杂的控制流部分，涉及多层决策：

```
                  ┌──────────────────┐
                  │ 在每个新帧判断     │
                  └────────┬─────────┘
                           │
                  ┌────────▼─────────┐
                  │ reattachByEdge   │  尝试沿已有边切换到邻居节点
                  │ (require_match)  │
                  └────────┬─────────┘
                           │
                  ┌────────▼──────────────────────────┐
                  │ three conditions checked:          │
                  │ 1. isInsideVcur()?                 │
                  │ 2. cur_iou >= iou_threshold?       │
                  │ 3. rel_dist <= max_edge_length?    │
                  └────────┬──────────────────────────┘
                           │
                    ┌──────┴──────┐
                    │ all three OK│  any FAIL
                    ▼              ▼
              ┌──────────┐  ┌──────────────────────┐
              │ STAY in   │  │ need_to_change_vcur  │
              │ current   │  │ = true               │
              │ vertex    │  └──────────┬───────────┘
              └──────────┘             │
                              ┌────────▼──────────┐
                              │ reattachByEdge     │
                              │ already succeeded? │
                              └──┬──────────┬──────┘
                                 │YES       │NO
                                 ▼          ▼
                            ┌────────┐ ┌────────────────────────┐
                            │ DONE   │ │ localization recent?    │
                            └────────┘ │ (< 5 seconds old)       │
                                       └───┬──────────┬─────────┘
                                           │YES       │NO
                                           ▼          ▼
                                   ┌──────────────┐ ┌──────────────┐
                                   │reattachBy    │ │ if mapping:  │
                                   │Localization  │ │ addNewVertex │
                                   └──┬───────┬───┘ │ if loc: warn │
                                      │YES    │NO    └──────────────┘
                                      ▼       ▼
                                  ┌──────┐ ┌──────────────────┐
                                  │ DONE │ │ if mapping:      │
                                  └──────┘ │ addNewVertex     │
                                           │ if loc:          │
                                           │ reattachByEdge   │
                                           │ (require_match   │
                                           │  = false)         │
                                           └──────────────────┘
```

---

## 2. 点云观测处理 (`processObservations`)

[VERIFY: topo_slam_model.cpp:133-166] / [prism_topomap.py:473-496](scripts/prism_topomap.py)

```
Algorithm: processObservations
Input: cloud_msg, cur_cloud, images, cur_curbs, x, y, theta
Output: cur_desc_ (描述符), cur_grid_ (更新后的栅格)

  1. 描述符提取 (通过 Python 推理服务 或 本地调用)
     ├─ 构建输入: 点云坐标 + 特征(全1) + 可选图像
     ├─ MinkowskiEngine 稀疏量化: quantize(coords, feats, quantization_size)
     ├─ PlaceRecognition 模型前向传播 → descriptor [256 or 512 dims]
     └─ 返回 cur_desc_

  2. 栅格更新 (C++ 本地计算)
     ├─ cur_grid_.updateFromCloudAndTransform(cur_cloud, x, y, -theta)
     │   ├─ transform(x, y, theta): 对已有栅格层做仿射变换 (里程计补偿)
     │   ├─ 去除 NaN 和超出 max_range 的点
     │   ├─ remove_floor_and_ceil: 过滤地板和天花板点
     │   ├─ 点云投影到栅格坐标: ij = round(xy / resolution) + [grid_radius, grid_radius]
     │   ├─ raycastGrid: 从中心向外发射射线, 填充可通行区域
     │   ├─ 障碍物标记为 2
     │   ├─ density_map 衰减更新: density *= attenuation + current_density
     │   └─ height_map 更新: np.maximum.at(height_map, ij, z)
     └─ 可选: updateCurbsFromCloud(cur_curbs) — 路沿衰减叠加
```

### 2.1 光线投影 (raycastGrid)

[VERIFY: scripts/local_grid.py:52-71]

```
Algorithm: raycastGrid
Input: n_rays, center_point (默认栅格中心)
Effect: 修改 self.layers['occupancy']

  for sector in 0..n_rays:
    angle = sector/n_rays * 2π - π
    沿角度发射射线: i = center + sin(angle)*[0..radius]
                    j = center + cos(angle)*[0..radius]
    只保留在栅格范围内的像素
    检查射线上的 occupancy 值:
      if 有障碍物 (值>0):
        将障碍物之前的像素标记为 1 (已知可通行)
      else:
        整条射线标记为 1
```

光线投影的效果：将障碍物遮挡后的"阴影区域"保持为 0 (未知)，障碍物前方为 1 (可通行)，障碍物处为 2。

---

## 3. 里程计积分 (`updateRelPoseByOdom`)

[VERIFY: topo_slam_model.cpp:118-127] / [prism_topomap.py:498-514](scripts/prism_topomap.py)

```
Algorithm: updateRelPoseByOdom
Input: cur_odom_pose [x, y, θ]
Effect: 更新 rel_pose_of_vcur_ (累加相对位移)

  if not odom_initialized:
    odom_pose_ = cur_odom_pose; return

  Δ_odom = getRelPose(odom_pose_, cur_odom_pose)  // 里程计增量
  rel_pose_of_vcur_ = applyPoseShift(rel_pose_of_vcur_, Δ_odom)
  odom_pose_ = cur_odom_pose
```

数学推导：

```
已知:
  T_vcur^robot(t-1): 机器人在上一帧相对于当前节点的位姿
  T_odom(t-1)^odom(t): 里程计增量

求:
  T_vcur^robot(t): 机器人在当前帧相对于当前节点的位姿

解:
  T_vcur^robot(t) = T_vcur^robot(t-1) ⊕ T_odom(t-1)^odom(t)

其中 ⊕ 为位姿复合运算 (applyPoseShift):
  x_new = x + Δx·cos(-θ) + Δy·sin(-θ)
  y_new = y - Δx·sin(-θ) + Δy·cos(-θ)
  θ_new = θ + Δθ
```

### 3.1 位姿复合公式推导

[VERIFY: scripts/utils.py:86-91]

```
设 T_vcur^robot = [x, y, θ] 表示 robot 在 vcur 坐标系下的位姿
设 Δ = [Δx, Δy, Δθ] 表示 robot 的新相对位移

我们需要将 Δ (在 robot 自身坐标系下) 变换到 vcur 坐标系:
  Δ_in_vcur = Rot₂(-θ) · [Δx, Δy]ᵀ
              = [Δx·cos(-θ) + Δy·sin(-θ), -Δx·sin(-θ) + Δy·cos(-θ)]ᵀ

最终:
  x_new = x + Δx·cos(θ) - Δy·sin(θ)    (等价于 x + Δx·cos(-θ) + Δy·sin(-θ))
  y_new = y + Δx·sin(θ) + Δy·cos(θ)
  θ_new = θ + Δθ
```

---

## 4. 定位管道 (`Localizer::localize`)

[VERIFY: scripts/localization.py:109-188](scripts/localization.py) / [src/localizer.cpp](src/localizer.cpp)

### 4.1 流程概览

```
Algorithm: Localizer.localize
触发: ROS Timer, 异步于主循环, 频率 = localization_frequency
Input: (从 shared state 读取) descriptor, grid, timestamp
Output: 写入 shared state: vertex_ids_matched, rel_poses, vertex_ids_unmatched

  Step 0: 获取当前状态快照
    snapshot = getCurrentState()  // mutex保护的 descriptor, grid, timestamp

  Step 1: FAISS 检索 (粗定位)
    dists, pred_i = faiss_index.search(descriptor, top_k)
    // 返回 L2 距离最小的 top_k 个候选节点 ID

  Step 2: 栅格配准 (精定位) — 对每个候选
    for each candidate_idx in pred_i:
      cand_grid = graph.getVertex(idx).grid
      // 调用配准模型: ICP / GeoTransformer / Feature2D
      transform, score = registration_pipeline.infer(ref_grid, cand_grid)

  Step 3: 分数过滤
    if score >= registration_score_threshold:
      matched_ids.append(idx)
      // 像素级→度量级变换
      tf_matrix = cand_grid.getTfMatrixXY(trans_i, trans_j, rot_angle)
      rel_poses.append([tf_matrix.tx, tf_matrix.ty, tf_matrix.θ])
    else:
      unmatched_ids.append(idx)

  Step 4: 写入定位结果
    writeLocalizedState(matched_ids, rel_poses, unmatched_ids, global_pose, stamp)
```

### 4.2 FAISS 最近邻搜索
[VERIFY: scripts/localization.py:123]

使用 L2 距离的暴力搜索（`IndexFlatL2`），返回最近的 `top_k` 个节点。对于数百个节点的规模，暴力搜索足够高效。

### 4.3 栅格配准模型类型

支持三种配准模型（在 config 中配置）：[VERIFY: scripts/models.py:37-62](scripts/models.py)

| 模型 | 实现方式 | 适用场景 |
|------|---------|---------|
| `icp` | RANSAC + ICP (点云) | 通用，但需要较好的初始值 |
| `geotransformer` | 深度学习 (GeoTransformer 网络) | 高精度，仅支持 CUDA |
| `feature2d` | 2D 特征检测 (ORB/SIFT/Harris) + 匹配 | 推荐用于沿边配准 |

### 4.4 异步定位的优势

定位模块与主循环异步运行的原因：
1. **深度学习推理耗时**: 配准模型推理可能是几十到几百毫秒
2. **不阻塞传感器处理**: 主循环可以持续更新栅格和里程计
3. **容错**: 定位失败不会直接阻断 SLAM 进程

---

## 5. 回环检测 (`findLoopClosure`)

[VERIFY: prism_topomap.py:228-255](scripts/prism_topomap.py) / [topo_slam_model.cpp:228-257](src/topo_slam_model.cpp)

```
Algorithm: findLoopClosure
Input: vertex_ids (定位匹配到的节点), dists (对应距离)
Output: found_loop_closure (bool), path_ (旧图路径)

  for i, j in all pairs of vertex_ids:
    u = vertex_ids[i]; v = vertex_ids[j]
    if u < 0 or v < 0: continue

    path, path_len = graph.getPathWithLength(u, v)
    if path is None: continue

    dst_through_cur = dists[i] + dists[j]  // 通过当前位置的距离
    // 判断条件:
    // 1. path_len > 5 (已有路径足够长)
    // 2. path_len > 2 * dst_through_cur (已有路径 >> 通过当前位置的路径)
    // 3. checkPathCondition(u, v) (路径不是绕路)
    if path_len > 5 and path_len > 2*dst_through_cur and checkPathCondition(u,v):
      found_loop_closure = True
      path_ = path
      return True

  return False
```

### 5.1 回环检测几何示意图

```
        u ───────── long path ─────────→ v
        │                              │
        │   dst_through_cur[i]         │ dst_through_cur[j]
        │                              │
        └──────── robot_position ──────┘
             (current observation)

条件: path_len(u↔v in old graph) > 2 * (dist_to_u + dist_to_v)
即: 图中已有路径显著长于通过当前位置的"捷径"
→ 当前位置是一个新的观察视角, 将 u 和 v 连接起来
```

### 5.2 checkPathCondition

[VERIFY: prism_topomap.py:178-206](scripts/prism_topomap.py) / [topo_slam_model.cpp:208-222](src/topo_slam_model.cpp)

检查路径是否过于绕路（通过比较路径长度与直线距离）:

```
straight_length = √(total_rel_pose_x² + total_rel_pose_y²)
return path_len > 3*straight_length || straight_length < 10
```

---

## 6. 节点切换决策树

### 6.1 reattachByEdge — 沿边切换

[VERIFY: topo_slam_model.cpp:280-366] / [prism_topomap.py:285-344](scripts/prism_topomap.py)

```
Algorithm: reattachByEdge
Input: require_match (是否要求配准成功)
Output: changed (是否成功切换)

  // 遍历当前节点的所有邻居
  for each neighbor (vertex_id, edge_rel_pose) in graph.adj_lists[last_vertex_id]:
    // 计算预测位置 (沿边) 与实际位置的偏移
    diff = |rel_pose_of_vcur_ - edge_rel_pose|
    找到距离最小的邻居

  // 如果当前位置离自己原点比离任何邻居都近, 放弃
  if min_distance >= distance_to_self_origin or min_distance >= 5.0:
    return false

  // 计算配准初始值
  rel_pose_to_vertex = getRelPose(rel_pose_of_vcur_, pose_on_edge)

  if require_match:
    // 将当前栅格变换到候选节点坐标系
    cur_grid_transformed = cur_grid_.transform(inv(rel_pose_to_vertex))
    // 调用配准精修
    tf_result = graph.getTransformToVertex(nearest_id, cur_grid_transformed)
    // 检查跳变是否过大
    if diff < local_jump_threshold:
      切换到邻居节点; return true
  else:
    // 不要求配准, 直接切换
    切换到邻居节点; return true

  return false
```

### 6.2 reattachByLocalization — 定位切换

[VERIFY: topo_slam_model.cpp:372-441] / [prism_topomap.py:347-395](scripts/prism_topomap.py)

```
Algorithm: reattachByLocalization
Input: iou_threshold, localized_stamp
Output: changed

  // 对齐定位时刻的位姿
  rel_pose_vcur_to_loc = getRelPoseFromStamp(localized_stamp)
  rel_pose_after_localization = getRelPose(rel_pose_vcur_to_loc, rel_pose_of_vcur_)

  for each (vertex_id, loc_rel_pose) in localized state:
    // 预测当前节点到候选节点的相对位姿
    pred_rel_pose_vcur_to_v = applyPoseShift(
        rel_pose_vcur_to_loc, inverse(loc_rel_pose))

    // 计算将当前栅格变换到候选节点坐标系的相对位姿
    rel_pose_robot_to_loc = getRelPose(rel_pose_since_localization, loc_rel_pose)

    // 计算 IoU
    iou = cur_grid.getIoU(candidate.grid, rel_pose_robot_to_loc)

    // 漂移检查 (防止因定位错误导致的大跳变)
    dst_to_candidate = distance(current_vertex, candidate, rel_pose_of_vcur)
    if dst_to_candidate > drift_coef * time_since_last_match + 10:
      continue

    // IoU 条件
    if iou > iou_threshold or need_to_change_vcur:
      if mode == "mapping":
        graph.addEdge(last_vertex_id, vertex_id, pred_rel_pose)
      // 切换节点
      更新 rel_pose_of_vcur_; return true

  return false
```

### 6.3 切换判定的三个条件

| 条件 | 含义 | 典型阈值 | 结果 |
|------|------|---------|------|
| `!isInsideVcur()` | 机器人移出了当前节点的已知区域 | 栅格范围外 | 必须切换 |
| `cur_iou < iou_threshold` | 当前观测与节点观测重叠不足 | 0.5-0.8 | 视为新区域 |
| `rel_dist > max_edge_length` | 机器人离节点中心太远 | 10m | 强制断开 |

---

## 7. Dijkstra 最短路径

[VERIFY: topo_graph.py:132-169](scripts/topo_graph.py) / [topological_graph.cpp:206-267](src/topological_graph.cpp)

### 7.1 标准 Dijkstra 实现

```
Algorithm: Dijkstra (getPathWithLength)
Input: start(u), goal(v)
Output: {path[], length}

  distances[i] = ∞ for all i
  distances[u] = 0
  heap = [(0, u)]  // (distance, node)

  while heap:
    current_dist, curr = heap.pop()
    if curr == v: return path
    if current_dist > distances[curr]: continue  // lazy deletion

    for neighbor, edge_pose in adj_list[curr]:
      weight = √(edge_pose.x² + edge_pose.y²)  // 欧几里德距离
      tentative = current_dist + weight
      if tentative < distances[neighbor]:
        distances[neighbor] = tentative
        prev[neighbor] = curr
        heap.push(tentative, neighbor)

  return (not found)
```

### 7.2 复杂度分析

- **时间复杂度**: O((V + E) log V)，其中 V 是节点数，E 是边数
- **空间复杂度**: O(V) — distances 和 prev 数组
- 拓扑图通常有数百节点，Dijkstra 执行时间可忽略

### 7.3 应用场景

1. **回环检测**: 查找 `u→v` 的已有路径长度 `path_len`
2. **导航**: 查找从当前节点到目标节点最近节点的路径
3. **边权重**: 使用欧几里德距离（非角度），因为角度差对路径规划意义较小

---

## Verification Checklist

- [x] update() 主循环: Python [prism_topomap.py:516-609] vs C++ [topo_slam_model.cpp:606-790]
- [x] processObservations: Python [prism_topomap.py:473-496] vs C++ [topo_slam_model.cpp:133-166]
- [x] updateRelPoseByOdom: Python [prism_topomap.py:498-514] vs C++ [topo_slam_model.cpp:118-127]
- [x] localize: Python [localization.py:109-188] vs C++ Localizer::localize()
- [x] findLoopClosure: Python [prism_topomap.py:228-255] vs C++ [topo_slam_model.cpp:228-257]
- [x] reattachByEdge: Python [prism_topomap.py:285-344] vs C++ [topo_slam_model.cpp:280-366]
- [x] reattachByLocalization: Python [prism_topomap.py:347-395] vs C++ [topo_slam_model.cpp:372-441]
- [x] Dijkstra: Python [topo_graph.py:132-169] vs C++ [topological_graph.cpp:206-267]

*分析时间: 2026-05-06 | 项目版本: 0.0.0 | 分支: main*
