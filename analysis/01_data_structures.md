# PRISM-TopoMap — Data Structures Analysis

## 数据结构总览

```
┌──────────────────────────────────────────────────────────────┐
│                    核心数据结构关系图                           │
│                                                              │
│  TopologicalGraph                                            │
│  ├── vertices_: vector<Vertex>                               │
│  │   └── Vertex                                              │
│  │       ├── pose_for_visualization: Pose2D [x,y,θ]         │
│  │       ├── grid: LocalGrid                                 │
│  │       └── descriptor: vector<float> (256 or 512 dims)     │
│  ├── adj_lists_: vector<vector<AdjEntry>>                    │
│  │   └── AdjEntry { vertex_id, rel_pose: Pose2D }           │
│  └── faiss_index_: IndexFlatL2 (C++) / faiss.IndexFlatL2 (Py)│
│                                                              │
│  LocalGrid                                                   │
│  ├── layers_: map<string, cv::Mat / np.ndarray>              │
│  │   ├── "occupancy": uint8   — 占据栅格 (0=未知,1=可通行,2=障碍)│
│  │   ├── "density_map": float — 障碍物密度衰减图              │
│  │   ├── "height_map": float  — 高度图                       │
│  │   └── "curbs": uint8       — 路沿衰减图 (户外)             │
│  ├── resolution_, radius_, max_range_                        │
│  └── grid_size_ = 2 * radius / resolution                    │
│                                                              │
│  Localizer (异步定位管理器)                                    │
│  ├── 当前快照: descriptor, grid, timestamp (mutex保护)        │
│  └── 定位结果: LocalizedState                                │
│      ├── vertex_ids_matched: vector<int>                     │
│      ├── rel_poses: vector<Pose2D>                           │
│      └── vertex_ids_unmatched: vector<int>                   │
│                                                              │
│  TopoSLAMModel (核心状态机)                                    │
│  ├── rel_pose_of_vcur_: Pose2D   — 机器人在当前节点的相对位姿 │
│  ├── rel_pose_vcur_to_loc_: Pose2D — 定位时刻的位姿快照       │
│  ├── odom_pose_: Pose2D          — 上一帧里程计              │
│  ├── cur_grid_: LocalGrid        — 当前帧观测栅格            │
│  └── rel_poses_stamped_: vector<StampedPose> — 带时间戳历史  │
└──────────────────────────────────────────────────────────────┘
```

---

## 1. Pose2D — 2D 位姿

**Python** — 普通三元组列表 `[x, y, theta]`:
[VERIFY: scripts/utils.py:82-91]

```python
# 所有位姿表示为 [x, y, theta]
# theta 范围为 [-π, π]

def normalize(angle):
    while angle < -np.pi:
        angle += 2 * np.pi
    while angle > np.pi:
        angle -= 2 * np.pi
    return angle

def get_rel_pose(x, y, theta, x2, y2, theta2):
    """计算 from→to 的相对位姿"""
    rel_x, rel_y = rotate(x2 - x, y2 - y, theta)
    return [rel_x, rel_y, normalize(theta2 - theta)]

def apply_pose_shift(pose, rel_x, rel_y, rel_theta):
    """在位姿上叠加相对偏移"""
    x, y, theta = pose
    new_x = x + rel_x * np.cos(-theta) + rel_y * np.sin(-theta)
    new_y = y - rel_x * np.sin(-theta) + rel_y * np.cos(-theta)
    new_theta = theta + rel_theta
    return [new_x, new_y, new_theta]
```

**C++** — `Eigen::Vector3d`:
[VERIFY: include/prism_topomap/utils.h:28]

```cpp
using Pose2D = Eigen::Vector3d;  // [x, y, theta]
```

关键位姿运算函数完全对应 Python 版本:
[VERIFY: include/prism_topomap/utils.h:57-65]

```cpp
double normalize(double angle);
Eigen::Vector2d rotate2D(double x, double y, double angle);
Pose2D getRelPose(const Pose2D& from, const Pose2D& to);
Pose2D applyPoseShift(const Pose2D& pose, const Pose2D& shift);
```

---

## 2. Vertex — 拓扑图顶点

**Python** — `vertex_dict` 字典:
[VERIFY: scripts/topo_graph.py:66-73]

```python
vertex_dict = {
    'pose_for_visualization': [x, y, theta],  # 全局可视化位姿
    'grid': grid.copy(),                        # LocalGrid 副本
    'descriptor': descriptor                    # np.ndarray (256 or 512,)
}
```

**C++** — `struct Vertex`:
[VERIFY: include/prism_topomap/topological_graph.h:23-27]

```cpp
struct Vertex {
    Pose2D pose_for_visualization;   // 全局可视化位姿 [x, y, theta]
    LocalGrid grid;                   // 局部栅格 (深拷贝存储)
    std::vector<float> descriptor;    // 位置识别描述符 (256=MinkLoc3D, 512=MSSPlace)
};
```

字段说明：
- **pose_for_visualization**: 节点的全局绝对位姿（仅用于可视化和度量路径查找，不参与定位优化）
- **grid**: 节点处观测到的局部占据栅格。存储时调用 `grid.copy()` 做深拷贝，保证后续修改不影响已存储节点
- **descriptor**: 通过 Place Recognition 模型提取的全局描述符向量。用于 FAISS 检索做粗定位

---

## 3. LocalGrid — 局部占据栅格

### Python 实现
[VERIFY: scripts/local_grid.py:10-38]

```python
class LocalGrid:
    def __init__(self, resolution=0.1, radius=18.0, max_range=8.0,
                 floor_height=0.0, ceil_height=1.0,
                 obstacles_attenuation=0.9, curbs_attenuation=0.99,
                 layer_names=['curbs', 'occupancy', 'height_map', 'density_map', 'density_map_cur'],
                 save_dir=None):
        self.grid_size = 2 * int(radius / resolution)  # 360x360 for default
        self.layers = {}  # dict of np.ndarray
```

### C++ 实现
[VERIFY: include/prism_topomap/local_grid.h:22-148]

```cpp
class LocalGrid {
    double resolution_, radius_, max_range_;
    double floor_height_, ceil_height_;
    double obstacles_attenuation_, curbs_attenuation_;
    int grid_size_;  // 2 * (int)(radius/resolution)
    std::map<std::string, cv::Mat> layers_;  // OpenCV Mat 替代 numpy array
};
```

### 栅格层说明

| 层名 | 数据类型 | 含义 | 用途 |
|------|---------|------|------|
| `occupancy` | uint8 | 0=未知, 1=已知可通行, 2=障碍物 | 光线投影、IoU计算、配准输入 |
| `density_map` | float32 | 障碍物密度(衰减累积) | 当前帧可视化 |
| `density_map_cur` | float32 | 当前帧密度快照 | 密度图更新中间层 |
| `height_map` | float32 | 各像素最大高度 | 高度信息保存 |
| `curbs` | uint8 | 路沿衰减累积 | 户外场景路沿信息 |
| `curbs_cur` | uint8 | 当前帧路沿快照 | 路沿层更新中间层 |

### 核心方法

| 方法 | 功能 | Python | C++ |
|------|------|--------|-----|
| `copy()` | 深拷贝整个栅格 | [local_grid.py:40-50](scripts/local_grid.py) | [local_grid.h:42](include/prism_topomap/local_grid.h) |
| `raycastGrid(n_rays)` | 光线投影填充可见区域 | [local_grid.py:52-71](scripts/local_grid.py) | [local_grid.cpp](src/local_grid.cpp) |
| `updateFromCloudAndTransform(cloud, x, y, theta)` | 点云投影+栅格变换 | [local_grid.py:73-106](scripts/local_grid.py) | [local_grid.cpp](src/local_grid.cpp) |
| `transform(x, y, theta)` | 对所有层做仿射变换 | [local_grid.py:141-143](scripts/local_grid.py) | [local_grid.cpp](src/local_grid.cpp) |
| `getTransformedGrid(grid, x, y, theta)` | 单层栅格仿射变换 | [local_grid.py:122-139](scripts/local_grid.py) | [local_grid.cpp](src/local_grid.cpp) |
| `isInside(x, y, theta)` | 检查位姿是否在已知区域内 | [local_grid.py:145-150](scripts/local_grid.py) | [local_grid.h:89](include/prism_topomap/local_grid.h) |
| `getIoU(other, rel_x, rel_y, rel_theta)` | 计算变换后栅格的IoU | [local_grid.py:152-178](scripts/local_grid.py) | [local_grid.h:95-97](include/prism_topomap/local_grid.h) |
| `getTfMatrixXY(trans_i, trans_j, rot_angle)` | 像素坐标→度量坐标变换矩阵 | [local_grid.py:180-196](scripts/local_grid.py) | [local_grid.h:103-104](include/prism_topomap/local_grid.h) |

### IoU 计算详解
[VERIFY: scripts/local_grid.py:152-178]

IoU 用于判断两个栅格的观测重叠程度，是节点切换判定的核心指标：

```
IoU = |occupancy_grid_A ∩ transformed_occupancy_grid_B| /
      |occupancy_grid_A ∪ transformed_occupancy_grid_B|
```

步骤:
1. 将当前栅格按相对位姿做仿射变换: `cur_grid_transformed = warpAffine(cur_grid, T(rel_pose))`
2. 二值化: `>0 → 1`
3. 计算交集: `sum(A * B)` (两栅格都为1的像素数)
4. 计算并集: `sum(A | B)` (至少一个为1的像素数)
5. 返回 `intersection / union`

---

## 4. AdjEntry — 邻接表项

**Python** — `(vertex_id, [x, y, theta])` 元组:
[VERIFY: scripts/topo_graph.py:111-112]

```python
# self.adj_lists[i] = [(j1, [x1,y1,theta1]), (j2, [x2,y2,theta2]), ...]
self.adj_lists[i].append((int(j), [x, y, theta]))
```

**C++** — `struct AdjEntry`:
[VERIFY: include/prism_topomap/topological_graph.h:32-35]

```cpp
struct AdjEntry {
    int vertex_id;      // 邻居节点 ID
    Pose2D rel_pose;    // 从当前节点到邻居节点的相对位姿 [x, y, theta]
};
```

---

## 5. FAISS 索引

### Python 版本
[VERIFY: scripts/models.py:20-30]

```python
# MinkLoc3D: 256维 L2索引
index = faiss.IndexFlatL2(256)
# MSSPlace: 512维 L2索引
index = faiss.IndexFlatL2(512)
```

### C++ 版本
[VERIFY: include/prism_topomap/topological_graph.h:115-116]

```cpp
std::unique_ptr<faiss::IndexFlatL2> faiss_index_;
int descriptor_dim_;  // 256 or 512
```

FAISS 索引的作用：存储所有顶点的描述符，支持高效的 L2 距离最近邻搜索（暴力搜索）。由于拓扑图规模通常在数百个节点量级，`IndexFlatL2` 的暴力搜索足够快。

**关键操作:**
- `add(descriptor)`: 添加顶点时调用 [topo_graph.py:73](scripts/topo_graph.py) / [topological_graph.cpp:81](src/topological_graph.cpp)
- `search(query, top_k)`: 定位时检索最相似的 k 个候选 [localization.py:123](scripts/localization.py) / 通过 Localizer 间接调用

---

## 6. LocalizedState — 定位输出

**Python** — dict 结构:
[VERIFY: scripts/localization.py:98-107]

```python
result = {
    'vertex_ids_matched': deepcopy(self.vertex_ids_matched),  # 配准成功的节点ID列表
    'rel_poses': deepcopy(self.rel_poses),                    # 对应的相对位姿列表
    'vertex_ids_unmatched': deepcopy(self.vertex_ids_unmatched), # 检索到但配准失败的节点
    'global_pose_for_visualization': (x, y, theta),           # 定位时刻的全局位姿
    'timestamp': self.localized_stamp                         # 定位时刻的时间戳
}
```

**C++** — `struct LocalizedState`:
[VERIFY: include/prism_topomap/localizer.h:24-30]

```cpp
struct LocalizedState {
    std::vector<int> vertex_ids_matched;
    std::vector<Pose2D> rel_poses;
    std::vector<int> vertex_ids_unmatched;
    Pose2D global_pose;
    double timestamp = 0.0;
};
```

---

## 7. InferenceClient — 推理服务客户端 (C++)

[VERIFY: include/prism_topomap/inference_client.h:24-99]

封装与 Python 推理服务节点 `inference_service_node.py` 的通信:

```cpp
struct DescriptorResult {
    bool success;
    std::vector<float> descriptor;  // 256 or 512 维
};

struct RegistrationResult {
    bool success;
    double score;
    double trans_i, trans_j, rot_angle;  // 像素级变换参数
};
```

通过两个 ROS Service 调用:
- `/prism/get_descriptor` → `InferenceClient::getDescriptor()`
- `/prism/grid_registration` → `InferenceClient::gridRegistration()`

---

## 8. TopoSLAMModel — 核心状态机

[VERIFY: include/prism_topomap/topo_slam_model.h:28-188]

```cpp
class TopoSLAMModel {
    // === 核心组件 ===
    std::shared_ptr<InferenceClient> inference_client_;
    TopologicalGraph graph_;
    Localizer localizer_;
    LocalGrid cur_grid_;

    // === 当前状态 ===
    int last_vertex_id_ = -1;           // 当前所在节点ID
    Pose2D rel_pose_of_vcur_;           // 机器人在当前节点的相对位姿
    Pose2D rel_pose_vcur_to_loc_;       // 定位时刻的 rel_pose 快照
    Pose2D odom_pose_;                  // 上一帧里程计位姿
    std::vector<StampedPose> rel_poses_stamped_;  // 带时间戳的位姿历史

    // === 状态标志 ===
    bool found_loop_closure_ = false;
    bool need_to_change_vcur_ = false;

    // === 参数 ===
    std::string mode_;                  // "mapping" or "localization"
    double iou_threshold_;              // 节点分离 IoU 阈值
    double max_edge_length_;            // 最大边长度
    double drift_coef_;                 // 里程计漂移系数
    double localization_timeout_;       // 初始定位超时
};
```

对应的 Python 版本:
[VERIFY: scripts/prism_topomap.py:19-96]

---

## 关键数据结构交互图

```
┌─────────────────────────────────────────────────────────────┐
│                      每帧数据流                              │
│                                                             │
│  ros::PointCloud2  ──► PointCloudPtr (cur_cloud)            │
│  ros::Odometry     ──► Pose2D (cur_odom_pose)               │
│  ros::Image        ──► sensor_msgs::Image (front/back)      │
│                                                             │
│           │  │  │                                           │
│           ▼  ▼  ▼                                           │
│                                                             │
│  TopoSLAMModel::update()                                    │
│  ├─ processObservations()                                   │
│  │  ├─ InferenceClient::getDescriptor() → cur_desc_         │
│  │  └─ LocalGrid::updateFromCloudAndTransform()              │
│  ├─ updateRelPoseByOdom() → rel_pose_of_vcur_               │
│  ├─ Localizer::updateCurrentState(desc, grid, ts)           │
│  ├─ Localizer::localize() [异步]                            │
│  │  └─ FAISS search → TopK → GridRegistration → LocalizedState│
│  ├─ findLoopClosure(vertex_ids, dists)                      │
│  ├─ reattachByEdge() / reattachByLocalization()             │
│  └─ addNewVertex() [if needed]                              │
│                                                             │
│           │                                                 │
│           ▼                                                 │
│  ResultsPublisher → RViz (拓扑图, TF, Grid)                  │
└─────────────────────────────────────────────────────────────┘
```

---

## Verification Checklist

- [x] Pose2D 定义: Python `[x,y,theta]` [VERIFY: scripts/utils.py:82-91] vs C++ `Eigen::Vector3d` [VERIFY: include/prism_topomap/utils.h:28]
- [x] Vertex 结构: Python dict [VERIFY: scripts/topo_graph.py:66-73] vs C++ struct [VERIFY: include/prism_topomap/topological_graph.h:23-27]
- [x] LocalGrid 层结构: Python np.ndarray dict [VERIFY: scripts/local_grid.py:30-35] vs C++ cv::Mat map [VERIFY: include/prism_topomap/local_grid.h:146]
- [x] FAISS 索引: Python `faiss.IndexFlatL2` [VERIFY: scripts/models.py:20] vs C++ `faiss::IndexFlatL2` [VERIFY: include/prism_topomap/topological_graph.h:115]
- [x] IoU 计算: Python [VERIFY: scripts/local_grid.py:152-178] vs C++ [VERIFY: include/prism_topomap/local_grid.h:95-97]
- [x] LocalizedState: Python dict [VERIFY: scripts/localization.py:98-107] vs C++ struct [VERIFY: include/prism_topomap/localizer.h:24-30]

*分析时间: 2026-05-06 | 项目版本: 0.0.0 | 分支: main*
