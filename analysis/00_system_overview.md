# PRISM-TopoMap — System Overview

## 项目概要

PRISM-TopoMap 是一个基于拓扑图的在线 SLAM 系统，支持大规模室内/室外环境的轻量级拓扑地图构建与定位。该系统由 [Kirill Muravyev 等人提出](https://arxiv.org/abs/2404.01674)，发表于 IEEE RA-L 2025。

核心思想：用**拓扑图**（graph of locations）替代传统的度量地图（metric map），每个节点存储局部占据栅格（local occupancy grid）和位置识别描述符（place recognition descriptor），边存储节点间的相对位姿。

## 架构概览

```
┌─────────────────────────────────────────────────────────────────────┐
│                      PRISM-TopoMap 系统架构                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │                    传感器输入层 (ROS Topics)                    │  │
│  │  PointCloud2 │ Odometry │ Image(front) │ Image(back) │ Curbs  │  │
│  └──────────────────────┬───────────────────────────────────────┘  │
│                         │                                           │
│  ┌──────────────────────┴───────────────────────────────────────┐  │
│  │              主节点 (两种架构可选)                              │  │
│  │  ┌─────────────────────┐  ┌─────────────────────────────┐    │  │
│  │  │ 纯Python架构         │  │ C++/Python 混合架构          │    │  │
│  │  │ prism_topomap_node.py│  │ prism_topomap_cpp_node       │    │  │
│  │  └──────────┬───────────┘  └──────────┬───────────────────┘    │  │
│  └─────────────┼──────────────────────────┼────────────────────────┘  │
│                │                          │                           │
│  ┌─────────────┴──────────┐  ┌───────────┴──────────────────────┐  │
│  │   核心算法层             │  │   Python 推理服务 (仅混合架构)    │  │
│  │  TopoSLAMModel          │  │   inference_service_node.py      │  │
│  │  ├─ TopologicalGraph    │  │   ├─ 描述符提取 (/prism/         │  │
│  │  ├─ Localizer           │  │   │   get_descriptor)            │  │
│  │  └─ LocalGrid           │  │   └─ 栅格配准 (/prism/           │  │
│  └─────────────────────────┘  │       grid_registration)          │  │
│                                └──────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │                    可视化/输出层                               │  │
│  │  RViz Markers │ OccupancyGrid │ TF │ TopologicalPath         │  │
│  └──────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

## 两种架构对比

| 特性 | 纯 Python 架构 | C++/Python 混合架构 |
|------|---------------|-------------------|
| **主循环节点** | `scripts/prism_topomap_node.py` | `src/prism_topomap_node.cpp` |
| **核心算法** | Python (`prism_topomap.py`) | C++ (`src/topo_slam_model.cpp`) |
| **ML 推理** | 直接调用 PyTorch/MinkowskiEngine | 通过 ROS Service 调用 `inference_service_node.py` |
| **FAISS** | Python (`faiss` 包) | C++ (`faiss::IndexFlatL2`) |
| **点云处理** | NumPy 数组操作 | PCL (`pcl::PointCloud`) |
| **栅格变换** | `cv2.warpAffine` (Python) | `cv::warpAffine` (C++) |
| **序列化** | Python `json` + `numpy` | `nlohmann/json` + OpenCV |
| **性能** | 受 GIL 限制 | 理论上更快，ML 推理在独立进程 |
| **依赖** | Python: PyTorch, MinkowskiEngine, FAISS, OpenCV, scipy, skimage | C++: PCL, Eigen, OpenCV, FAISS, yaml-cpp, nlohmann/json + Python 推理服务 |
| **编译** | 无需编译 | 需要 `catkin_make` |

### 架构选择的原因

混合架构的设计动机（来自 [CMakeLists.txt:1-5](CMakeLists.txt) 和 [package.xml:6-8](package.xml)）：

1. **C++ 主循环**：利用 C++ 的执行效率处理高频传感器数据、Dijkstra 最短路径、点云投影等计算密集型操作
2. **Python 推理服务**：深度学习模型（MinkLoc3D/MSSPlace, GeoTransformer）依赖 PyTorch/MinkowskiEngine，这些库的 Python API 更成熟
3. **解耦设计**：ML 推理作为一个独立的 ROS Service 节点，可以单独重启或部署到不同机器

## 模块清单

### 纯 Python 架构模块

| 文件 | 类/功能 | 行数 | 复杂度 |
|------|---------|------|--------|
| [prism_topomap_node.py](scripts/prism_topomap_node.py) | `PRISMTopomapNode` — ROS 主节点, 传感器订阅、时间同步、可视化发布 | ~787 | 高 |
| [prism_topomap.py](scripts/prism_topomap.py) | `TopoSLAMModel` — SLAM 核心算法: 节点管理、回环检测、切换判定 | ~615 | 极高 |
| [topo_graph.py](scripts/topo_graph.py) | `TopologicalGraph` — 拓扑图数据结构, FAISS 索引, Dijkstra | ~188 | 中 |
| [local_grid.py](scripts/local_grid.py) | `LocalGrid` — 局部占据栅格, 光线投影, IoU计算, 仿射变换 | ~247 | 中 |
| [localization.py](scripts/localization.py) | `Localizer` — 异步定位管理器, FAISS检索+配准 | ~188 | 中 |
| [models.py](scripts/models.py) | 模型工厂函数: `get_place_recognition_model`, `get_registration_model` | ~62 | 低 |
| [utils.py](scripts/utils.py) | 2D位姿运算, 点云坐标提取, 地板/天花板过滤 | ~97 | 低 |
| [inference_service_node.py](scripts/inference_service_node.py) | Python 推理服务节点 (仅混合架构使用) | ~338 | 中 |
| [gt_map.py](scripts/gt_map.py) | 地面真值地图 | - | 低 |
| [tf_manager.py](scripts/tf_manager.py) | TF 变换管理 | - | 低 |

### C++ 混合架构模块

| 头文件 | 源文件 | 类 | 行数 | 复杂度 |
|--------|--------|-----|------|--------|
| [topo_slam_model.h](include/prism_topomap/topo_slam_model.h) | [topo_slam_model.cpp](src/topo_slam_model.cpp) | `TopoSLAMModel` | ~791 | 极高 |
| [topological_graph.h](include/prism_topomap/topological_graph.h) | [topological_graph.cpp](src/topological_graph.cpp) | `TopologicalGraph` | ~417 | 中 |
| [local_grid.h](include/prism_topomap/local_grid.h) | [local_grid.cpp](src/local_grid.cpp) | `LocalGrid` | ~150+ | 中 |
| [localizer.h](include/prism_topomap/localizer.h) | [localizer.cpp](src/localizer.cpp) | `Localizer` | ~120+ | 中 |
| [inference_client.h](include/prism_topomap/inference_client.h) | [inference_client.cpp](src/inference_client.cpp) | `InferenceClient` | ~100+ | 低 |
| [results_publisher.h](include/prism_topomap/results_publisher.h) | [results_publisher.cpp](src/results_publisher.cpp) | `ResultsPublisher` | ~95+ | 低 |
| [utils.h](include/prism_topomap/utils.h) | [utils.cpp](src/utils.cpp) | 位姿运算, PCL点云处理 | ~125+ | 低 |
| [prism_topomap_node.h](include/prism_topomap/prism_topomap_node.h) | [prism_topomap_node.cpp](src/prism_topomap_node.cpp) | `PRISMTopomapNode` | ~140+ | 高 |

### Build System & Config

| 文件 | 用途 |
|------|------|
| [CMakeLists.txt](CMakeLists.txt) | CMake 构建：catkin + Eigen + OpenCV + PCL + FAISS |
| [package.xml](package.xml) | ROS 包元数据 |
| `srv/GetDescriptor.srv` | 自定义 ROS Service: 描述符提取请求/响应 |
| `srv/GridRegistration.srv` | 自定义 ROS Service: 栅格配准请求/响应 |

### 启动文件

| Launch 文件 | 用途 |
|-------------|------|
| [build_map_by_iou_habitat.launch](launch/build_map_by_iou_habitat.launch) | Habitat 仿真环境 SLAM (纯 Python) |
| [build_map_by_iou_scout_rosbag.launch](launch/build_map_by_iou_scout_rosbag.launch) | Scout 机器人 rosbag SLAM (纯 Python) |
| [build_map_by_iou_scout_rosbag_hybrid.launch](launch/build_map_by_iou_scout_rosbag_hybrid.launch) | Scout 机器人 rosbag SLAM (混合架构) |
| [hybrid_mapping.launch](launch/hybrid_mapping.launch) | 通用混合架构 SLAM |
| [habitat_mp3d_localization.launch](launch/habitat_mp3d_localization.launch) | Habitat 仿真环境纯定位 |
| [navigation_habitat.launch](launch/navigation_habitat.launch) | Habitat 仿真环境导航 |

## 系统运行模式

```
运行模式
├── SLAM (mapping)
│   ├── 从零建图: mode="mapping", 无预加载图
│   └── 增量建图: mode="mapping", 加载已有图
└── 纯定位 (localization)
    └── 加载预建图, 不修改图结构
```

两种模式的核心差异：
- **SLAM 模式**: 可以添加新节点(`addNewVertex`)、添加边(`addEdge`)、检测回环
- **Localization 模式**: 只做节点匹配和切换(`reattachByEdge`, `reattachByLocalization`)，不修改图

## 核心数据流总览

```
传感器数据 (PointCloud2 + Odometry + Image)
    │
    ▼
┌─────────────────────────────────────────────┐
│ 1. 时间同步 (getSyncPoseAndImages)          │
│    对齐点云、里程计、图像的时间戳              │
└──────────────────┬──────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────┐
│ 2. 观测处理 (processObservations)           │
│    ├─ 描述符提取: 点云→MinkowskiEngine量化    │
│    │    → PlaceRecognition模型 → desc向量    │
│    ├─ 栅格更新: 点云投影→光线投影→占据/密度图  │
│    └─ 路沿更新: 路沿点云衰减叠加               │
└──────────────────┬──────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────┐
│ 3. 里程计积分 (updateRelPoseByOdom)          │
│    ├─ 相对里程计增量累加到 rel_pose_of_vcur   │
│    └─ 记录带时间戳的位姿历史                   │
└──────────────────┬──────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────┐
│ 4. 定位器更新 (localizer.updateCurrentState) │
│    异步定位线程读取最新的 desc + grid          │
└──────────────────┬──────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────┐
│ 5. 定位查询 (localizer.localize)             │
│    ├─ FAISS 检索 top-k 候选节点               │
│    ├─ 栅格配准 (ICP/GeoTransformer/Feature2D) │
│    └─ 写入匹配/未匹配结果                      │
└──────────────────┬──────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────┐
│ 6. SLAM 主循环决策                           │
│    ├─ 回环检测 (findLoopClosure)              │
│    ├─ 沿边切换 (reattachByEdge)               │
│    ├─ IoU/距离判定 → 是否需要换节点             │
│    ├─ 定位切换 (reattachByLocalization)        │
│    └─ 添加新节点 (addNewVertex)                │
└──────────────────┬──────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────┐
│ 7. 可视化发布                                 │
│    拓扑图 MarkerArray, TF, OccupancyGrid      │
└─────────────────────────────────────────────┘
```

---

## Verification Checklist

- [x] 所有模块均从实际代码中识别 [VERIFY: 文件目录树]
- [x] 两种架构均已覆盖 [VERIFY: CMakeLists.txt 中同时安装 Python 脚本和 C++ 可执行文件]
- [x] 数据流步骤与实际代码匹配 [VERIFY: prism_topomap.py:516-609, topo_slam_model.cpp:606-790]

*分析时间: 2026-05-06 | 项目版本: 0.0.0 | 分支: main*
