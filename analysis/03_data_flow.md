# PRISM-TopoMap — Data Flow Analysis

## 1. 完整数据流图

```
┌──────────────────────────────────────────────────────────────────────────┐
│                           PRISM-TopoMap Data Flow                         │
│                                                                          │
│  ┌──────────────────┐                                                     │
│  │ 传感器输入        │                                                     │
│  │ (ROS Topics)     │                                                     │
│  └──────┬───────────┘                                                     │
│         │                                                                 │
│  ┌──────┴──────────────────────────────────────────────────────────────┐ │
│  │ Phase 1: Message Ingestion & Buffering                               │ │
│  │                                                                      │ │
│  │ PointCloud2 ──► pcdCallback ──► cur_cloud []                        │ │
│  │ Odometry    ──► odomCallback ──► odom_poses[]                       │ │
│  │ Image(front)──► frontImageCallback ──► rgb_buffer_front[]            │ │
│  │ Image(back) ──► backImageCallback ──► rgb_buffer_back[]              │ │
│  │ Curbs       ──► curbDetectionCallback ──► curb_clouds[]              │ │
│  │ GT Pose     ──► gtPoseCallback ──► gt_poses[]                        │ │
│  │                                                                      │ │
│  │ 所有消息带时间戳存入队列/列表                                          │ │
│  └──────────────────────────┬───────────────────────────────────────────┘ │
│                             │                                             │
│  ┌──────────────────────────┴───────────────────────────────────────────┐ │
│  │ Phase 2: Time Synchronization (getSyncPoseAndImages)                  │ │
│  │                                                                       │ │
│  │  Step 2.1: 提取点云时间戳 t_pcd                                       │ │
│  │  Step 2.2: 在 odom_poses / gt_poses 中查找最近的位姿                    │ │
│  │  Step 2.3: 时间戳差异 < delta (0.05s)                                  │ │
│  │  Step 2.4: 插值位姿 (如果两个方向都有数据)                               │ │
│  │  Step 2.5: 查找最近的图像帧                                             │ │
│  │                                                                       │ │
│  │  Output: { global_pose, odom_pose, img_front, img_back, curbs }       │ │
│  └──────────────────────────┬────────────────────────────────────────────┘ │
│                             │                                             │
│  ┌──────────────────────────┴────────────────────────────────────────────┐ │
│  │ Phase 3: Core SLAM Processing (TopoSLAMModel::update)                  │ │
│  │                                                                        │ │
│  │  3a. 里程计积分                                                         │ │
│  │    odom_pose(t-1), odom_pose(t) ──► Δodom ──► rel_pose_of_vcur_         │ │
│  │                                                                        │ │
│  │  3b. 观测处理 (processObservations)                                     │ │
│  │    PointCloud ──► [Python/C++ Inference] ──► descriptor (256/512 dims)  │ │
│  │    PointCloud ──► LocalGrid.updateFromCloudAndTransform()              │ │
│  │      ├─ transform(Δodom): 补偿里程计漂移                                │ │
│  │      ├─ 点云投影: 3D→2D grid coordinates                                │ │
│  │      ├─ raycastGrid: 填充可通行区域                                     │ │
│  │      └─ density/height map 更新                                        │ │
│  │                                                                        │ │
│  │  3c. 定位器快照更新                                                     │ │
│  │    descriptor + grid + timestamp → localizer.updateCurrentState()      │ │
│  │                                                                        │ │
│  │  3d. 定位查询结果                                                       │ │
│  │    localizer.getLocalizedState() → {matched[], unmatched[], rel_poses[]}│ │
│  │                                                                        │ │
│  │  3e. SLAM 决策                                                          │ │
│  │    ├─ 回环检测 → 添加新节点连接回路                                      │ │
│  │    ├─ 沿边切换 → 重定位到邻居节点                                        │ │
│  │    ├─ 定位切换 → 跳转到定位匹配的节点                                    │ │
│  │    └─ 添加新节点 → 拓展拓扑图                                            │ │
│  └──────────────────────────┬─────────────────────────────────────────────┘ │
│                             │                                              │
│  ┌──────────────────────────┴──────────────────────────────────────────────┐│
│  │ Phase 4: Visualization Publishing (ResultsPublisher)                     ││
│  │                                                                          ││
│  │  publishGraph()         ──► MarkerArray (顶点+边+文字)                    ││
│  │  publishLastVertex()    ──► Marker + TF "vcur"→"map"                     ││
│  │  publishRelPose()       ──► TF "current_state"→"vcur"                    ││
│  │  publishCurGrid()       ──► OccupancyGrid (当前帧栅格)                    ││
│  │  publishLocalGrid()     ──► OccupancyGrid (当前节点栅格)                  ││
│  │  publishLocalizationResults() ──► 匹配/未匹配节点 Marker                   ││
│  │  publishLoopClosureResults() ──► 回环路径 MarkerArray                     ││
│  │  publishTopologicalPath() ──► TopologicalPath + 路径标记                  ││
│  │  publishSubgoal()       ──► TF + PoseStamped (导航目标)                   ││
│  │  publishTfFromOdom()    ──► TF (可选: 里程计直接发布)                      ││
│  └──────────────────────────────────────────────────────────────────────────┘│
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. 异步定位数据流

主循环和定位模块通过 mutex 保护的共享状态进行通信：

```
┌─────────────────────────────┐     ┌──────────────────────────────┐
│  Main Loop (高频 ~10Hz)      │     │  Localizer Thread (低频 ~2Hz) │
│                              │     │                               │
│  updateCurrentState() ───────┼──►  │  ┌─────────────────────────┐ │
│    descriptor ───────────────┼──►  │  │ descriptor (copy)       │ │
│    grid ─────────────────────┼──►  │  │ grid (copy)             │ │
│    timestamp ────────────────┼──►  │  │ timestamp               │ │
│                              │     │  └───────────┬─────────────┘ │
│                              │     │              │               │
│                              │     │              ▼               │
│                              │     │  ┌─────────────────────────┐ │
│                              │     │  │ FAISS search (top-k)    │ │
│                              │     │  └───────────┬─────────────┘ │
│                              │     │              │               │
│                              │     │              ▼               │
│                              │     │  ┌─────────────────────────┐ │
│                              │     │  │ For each candidate:     │ │
│                              │     │  │  GridRegistration       │ │
│                              │     │  └───────────┬─────────────┘ │
│                              │     │              │               │
│  getLocalizedState() ◄───────┼─────│─ writeLocalizedState()     │ │
│    vertex_ids_matched ◄──────┼─────│  vertex_ids_unmatched     │ │
│    rel_poses ◄───────────────┼─────│  timestamp                │ │
│    timestamp ◄───────────────┼─────│                            │ │
└─────────────────────────────┘     └──────────────────────────────┘
```

**关键设计点:**
- 主循环不等待定位完成 — 持续更新栅格和里程计
- `localization_is_fresh` 位用于判断定位结果是否在新数据采集之后
- 定位结果可能过时（在定位请求发出后到结果返回期间，机器人已移动）

---

## 3. C++/Python 混合架构中的跨进程数据流

```
┌────────────────────────────────┐     ROS Service      ┌────────────────────────┐
│ C++ Main Node                  │◄════════════════════►│ Python Inference Node   │
│ (prism_topomap_cpp_node)       │                      │ (inference_service_node)│
│                                │                      │                         │
│ InferenceClient                │                      │ InferenceServiceNode    │
│                                │                      │                         │
│ getDescriptor() ───────────────┼── GetDescriptor ───►│ handle_get_descriptor() │
│  cloud_msg (PointCloud2)       │   .srv               │  ├─ pc2→np array       │
│  images (optional)             │                      │  ├─ ME quantize        │
│  ◄──── descriptor (float[]) ───┼── Response ──────────│  └─ model.forward()    │
│                                │                      │                         │
│ gridRegistration() ────────────┼── GridRegistration ─►│ handle_grid_reg()      │
│  ref_grid (uint8[])            │   .srv               │  ├─ bytes→np→Tensor    │
│  cand_grid (uint8[])           │                      │  ├─ pipeline.infer()   │
│  registration_type ("inline"   │                      │  └─ return trans,score │
│   or "localization")           │                      │                         │
│  ◄── trans_i, trans_j,        │── Response ──────────│                         │
│       rot_angle, score ────────│                      │                         │
└────────────────────────────────┘                      └─────────────────────────┘
```

**栅格传输格式:**
- C++ 端: `cv::Mat (CV_8U)` → 序列化为 `uint8[]` 
- Python 端: `uint8[]` → `np.frombuffer()` → reshape → `torch.Tensor`
- 传输数据量: 360×360 = 129,600 bytes (标准配置下)

---

## 4. 描述符提取数据流

```
PointCloud2 (ROS Msg)
    │
    ▼
┌──────────────────────────────────────┐
│ 解析点云坐标                          │
│ Python: ros_numpy → np.array [N, 3]  │
│ C++:   pcl::fromROSMsg → PCL cloud   │
└──────────────┬───────────────────────┘
               │
               ▼
┌──────────────────────────────────────┐
│ MinkowskiEngine 稀疏量化              │
│ quantize(coords, feats, voxel_size)  │
│ → quantized_coords (稀疏张量)         │
└──────────────┬───────────────────────┘
               │
               ▼
┌──────────────────────────────────────┐
│ 可选: 图像预处理                      │
│ Image → cv_bridge → BGR→RGB → CHW    │
└──────────────┬───────────────────────┘
               │
               ▼
┌──────────────────────────────────────┐
│ Place Recognition Model Forward      │
│ MinkLoc3D: 256-dim descriptor        │
│ MSSPlace:  512-dim descriptor        │
└──────────────┬───────────────────────┘
               │
               ▼
       float[] descriptor
```

**描述符维度:**
- MinkLoc3D (纯点云): 256 维 [VERIFY: scripts/models.py:20]
- MSSPlace (多模态): 512 维 [VERIFY: scripts/models.py:30]

---

## 5. 拓扑图序列化数据流

### 保存 (saveToJson)
[VERIFY: topo_graph.py:171-188](scripts/topo_graph.py) / [topological_graph.cpp:305-357](src/topological_graph.cpp)

```
graph.saveToJson(output_path)
    │
    ├─ For each vertex i:
    │   ├─ grid.save(output_path + "/" + str(i))
    │   │   ├─ occupancy.png (PNG image)
    │   │   ├─ density_map.png
    │   │   ├─ height_map.npz
    │   │   └─ metadata.yaml (resolution, radius, etc.)
    │   └─ vertex descriptor → graph.json
    │
    └─ graph.json:
        {
          "vertices": [
            {"pose_for_visualization": [x, y, theta],
             "descriptor": [d1, d2, ..., dN]},
            ...
          ],
          "edges": [
            [[neighbor_id, [x, y, theta]], ...],
            ...
          ]
        }
```

### 加载 (loadFromJson)
[VERIFY: topo_graph.py:45-58](scripts/topo_graph.py) / [topological_graph.cpp:363-415](src/topological_graph.cpp)

```
graph.loadFromJson(input_path)
    │
    ├─ graph.json → 解析顶点和边
    ├─ For each vertex i:
    │   └─ LocalGrid::load(input_path + "/" + str(i))
    │       ├─ 加载各层 PNG/NPZ 文件
    │       └─ 读取 metadata.yaml
    └─ For each descriptor:
        └─ faiss_index.add(descriptor)  // 重建索引
```

---

## 6. 关键数据变换路径

### 6.1 栅格仿射变换链
[VERIFY: scripts/local_grid.py:122-143](scripts/local_grid.py)

```
原始栅格坐标 (pixel space)
    │
    ▼  plus8 (平移: +radius/resolution)
度量空间 (以栅格中心为原点)
    │
    ▼  tf_matrix (旋转-θ + 平移 x/resolution, y/resolution)
变换后的度量空间
    │
    ▼  minus8 (平移: -radius/resolution)
变换后的栅格坐标 (pixel space)
```

变换矩阵: `T = minus8 · [cos(-θ) sin(-θ) y/res; -sin(-θ) cos(-θ) x/res; 0 0 1] · plus8`

### 6.2 像素配准结果→度量位姿
[VERIFY: scripts/local_grid.py:180-196](scripts/local_grid.py)

```
配准模型输出: (trans_i, trans_j, rot_angle)  // 像素空间的平移和旋转
    │
    ▼ getTfMatrixXY(trans_i, trans_j, rot_angle)
4×4 齐次变换矩阵
    │
    ▼
度量位姿: [matrix(0,3), matrix(1,3), atan2(matrix(1,0), matrix(0,0))]
```

---

## Verification Checklist

- [x] 时间同步: Python [prism_topomap_node.py:656-682] vs C++ PRISMTopomapNode::getSyncPoseAndImages()
- [x] 异步定位 mutex 保护: Python [localization.py:34,69-75,77-85] vs C++ Localizer mutex_
- [x] 描述符提取流程: Python [prism_topomap.py:474-491] vs inference_service_node.py [handle_get_descriptor:128-196]
- [x] 栅格配准流程: Python [localization.py:143] registration_pipeline.infer() vs inference_service_node.py [handle_grid_registration:201-271]
- [x] 图序列化: Python [topo_graph.py:171-188] vs C++ [topological_graph.cpp:305-415]
- [x] 栅格变换链: Python [local_grid.py:122-143] vs C++ LocalGrid::getTransformedGrid()

*分析时间: 2026-05-06 | 项目版本: 0.0.0 | 分支: main*
