# PRISM-TopoMap 真实运行代表案例

源日志：`/home/tom/host_catkin_ws/src/PRISM-TopoMap/data_flow.log`

行号均指源日志物理行。解析时逐行读取、移除 ANSI 颜色，并将同一物理行中的多个
logger/`[FLOW]` 片段拆开；被 stdout/stderr 插入截断的尾字段不作臆测。

## 案例 A：FRAME 1 创建首节点

FRAME 1 的点云 STAMP 为 `1517156141.444385767`。第一次同步因尚无 odom 返回
`WAITING_FOR_POSE`（L21）；同一点云随后使用 ODOM
`(-0.036507,-0.003464,0.022191)` 成功同步（L27）。

本帧 28,800 个原始点中有 28,635 个有限点。描述符服务在 `0.759 ms` 后临时失败，
返回 `success=false`、`dim=0`（L34）。因此写入 Localizer 的快照和新建 vertex 0
都明确记录 `descriptor_dim=0`，FAISS 大小仍为 0（L36、L44）。这是本次运行首帧的
服务现象，不是算法规定首节点必须没有描述符：其余 241 帧的 C++ 服务结果均成功且为
256 维。

ODOM 尚未初始化，delta、grid_shift 和 rel_pose 前后值均为 `(0,0,0)`（L29）。
快照 STAMP 为 `1517156141.444386`；首节点分支不读取定位结果，日志也没有证据把这个
descriptor_dim=0 的快照关联到某个后续 CONSUME，因此消费帧记为“日志未记录”。

描述符失败没有阻断 LocalGrid 和首节点创建。栅格为 `360×360`，unknown/free/occupied
为 `126677/2638/285`；vertex 0 保存全局位姿
`(-0.0365,-0.0035,0.022191)`。最终状态为：

```text
vertex_before=-1
vertex_after=0
decision=FIRST_VERTEX
graph_vertices=1
graph_edges=0
```

关键证据：

```text
L34 [FRAME=1][STAGE=DESCRIPTOR] action=SERVICE_RESULT success=false elapsed_ms=0.759 dim=0 ...
L35 [FRAME=1][STAGE=GRID] ... occupancy_unknown=126677 occupancy_free=2638 occupancy_obstacle=285 ...
L36 [FRAME=1][STAGE=LOCALIZER_SNAPSHOT] action=WRITE descriptor_dim=0 ... graph_vertices=0
L44 [FRAME=1][STAGE=VERTEX] event=CREATE ... new_id=0 ... descriptor_dim=0 ... faiss_after=0
L47 [FRAME=1][STAGE=SUMMARY] vertex_before=-1 vertex_after=0 decision=FIRST_VERTEX ...
```

## 案例 B：FRAME 162 → 163 的跨帧定位与 KEEP

这个案例按相同的快照/结果时间戳 `1517156201.641442` 建立关联，完整展示：

```text
FRAME 162 写快照
  → ROS Timer 读取该快照
  → FAISS 返回 5 个候选
  → 5 次栅格配准（1 matched + 4 unmatched）
  → LocalizedState 写回非零 result_stamp
  → FRAME 163 消费同一 result_stamp
  → KEEP vertex 6
```

FRAME 162 在 L5373 写入 256 维描述符和 `occupancy_nonzero=15462` 的快照。该帧主循环
在 L5374 消费的是更早的 `result_stamp=1517156201.341012`，不能把它和刚写入的快照
混为一谈。帧处理结束后，Timer 在 L5394 读取 FRAME 162 的快照；这一行的末尾被并发
WARN 插入截断，但 FRAME、LOC_STAMP、STAGE、`action=TIMER_READ` 和 descriptor 字段
完整可识别。

FRAME 162 的传感器同步使用 ODOM `(6.475265,8.676470,2.787530)`，
dt_odom=`0.054347 s`；28,800 个原始点中 27,644 个有限、1,156 个无效（L5367–L5368）。
ODOM delta=`(0.1169,-0.0045,0.011412)`，rel_pose 从
`(0.4629,0.0148,0.144683)` 更新到 `(0.5792,0.0272,0.156095)`；grid_shift 为
`(-0.1168,0.0059,-0.011412)`（L5369）。C++ 在 `112.164 ms` 后收到 256 维描述符；
LocalGrid 的 unknown/free/occupied 为 `114138/13609/1853`，更新 `3.927 ms`
（L5371–L5372）。

FAISS 在 `0.013 ms` 内从 6 个索引项返回：

```text
[5:0.523608, 4:1.922655, 3:3.354603, 2:4.591693, 0:5.271492]
```

candidate 5 的配准服务成功，score=`0.689512 ≥ 0.600000`，得到
`metric_pose=(5.2375,-0.0907,0.759385)`；candidate 4、3、2、0 均为 unmatched。
L5411 随后生成 `matched=1 unmatched=4 result_stamp=1517156201.641442`。

下一成功帧 FRAME 163 在 L5420 消费完全相同的 result_stamp，age=`0.399102 s`，
定位候选为 `[5]`。几何回环组合均未满足条件，沿现有边切换也因候选节点并不更近而拒绝。
保持检查为 `inside=true`、IoU=`0.688722 ≥ 0.3`、rel_dist=`0.5884 ≤ 5.0 m`，
所以 `vertex_before=vertex_after=6`、`decision=KEEP`（L5429、L5432）。

关键证据：

```text
L5373 [FRAME=162][STAMP=1517156201.641442][STAGE=LOCALIZER_SNAPSHOT] action=WRITE ...
L5394 [FRAME=162][LOC_STAMP=1517156201.641442][STAGE=LOCALIZER_SNAPSHOT] action=TIMER_READ ...
L5399 [FRAME=162][LOC_STAMP=1517156201.641442][STAGE=FAISS] ... candidates=[5:0.523608,...]
L5402 [FRAME=162][LOC_STAMP=1517156201.641442][STAGE=REGISTRATION] candidate=5 ... score=0.689512 ... result=MATCHED
L5404–L5410 candidates 4/3/2/0 ... result=UNMATCHED
L5411 [FRAME=162][LOC_STAMP=1517156201.641442][STAGE=LOCALIZATION_RESULT] matched=1 unmatched=4 result_stamp=1517156201.641442
L5420 [FRAME=163][STAGE=LOCALIZATION_RESULT] action=CONSUME result_stamp=1517156201.641442 age=0.399102 matched=1 unmatched=4
L5429 [FRAME=163][STAGE=DECISION] step=KEEP_CHECK vertex=6 inside=true iou=0.688722 rel_dist=0.5884 ...
L5432 [FRAME=163][STAGE=SUMMARY] vertex_before=6 vertex_after=6 decision=KEEP ...
```

## 案例 C：FRAME 106 创建新节点和顺序边

FRAME 106 的点云 STAMP 为 `1517156181.042201281`。ODOM 增量使节点内相对位姿从
`(4.9748,-0.1739,-0.031325)` 变为 `(5.3720,-0.1940,-0.033822)`。该帧消费较早的
定位结果 `result_stamp=1517156180.642200`，其中 matched/unmatched=`0/1`，没有可用于
定位切换的候选。

这份消费结果来自 FRAME 105：L2971 写入快照，L2991 Timer 读取，L2992 的 FAISS
从一个索引项返回 `[0:2.393546]`；candidate 0 的配准服务失败、score=`0.0`，
所以 L2995 生成 `matched=0 unmatched=1`，再由 FRAME 106 在 L3004 消费，
age=`0.400001 s`。

FRAME 106 同步到 ODOM `(10.337358,0.232558,0.032699)`，dt_odom=`0.054464 s`；
28,800 个原始点中 28,487 个有限、313 个无效。C++ 描述符服务成功返回 256 维向量，
耗时 `117.811 ms`；Python 关联记录量化出 1,346 个点/体素记录，前向 `5.558 ms`、
总计 `112.613 ms`（L2997–L3001、L3080）。LocalGrid 的
unknown/free/occupied=`126034/3172/394`，总更新 `4.148 ms`（L3002）。

保持检查中 `inside=true`、IoU=`0.610981 ≥ 0.3`，但
`rel_dist=5.3755 m > max_edge_length=5.0 m`，因此距离条件触发 `NEED_CHANGE`。
沿边切换以 `NOT_CLOSER_THAN_CURRENT` 拒绝，定位切换以 `NO_CANDIDATES` 拒绝。

mapping 分支随后创建 vertex 2，节点数 `2→3`、FAISS 大小 `1→2`，并成功加入
顺序边 `1→2`：

```text
new vertex global_pose = (10.3374,0.2326,0.032699)
edge rel_pose          = (5.3720,-0.1940,-0.033822)
edge length            = 5.3755 m
still_added            = true
```

设置当前节点为 2 后节点内相对位姿清零；最终
`vertex_before=1 vertex_after=2 decision=NEW_VERTEX graph_vertices=3 graph_edges=2`。

关键证据：

```text
L3004 [FRAME=106][STAGE=LOCALIZATION_RESULT] action=CONSUME ... matched=0 unmatched=1
L3010 [FRAME=106][STAGE=DECISION] step=KEEP_CHECK ... iou=0.610981 rel_dist=5.3755 max_edge_length=5.0000
L3011 [FRAME=106][STAGE=DECISION] decision=NEED_CHANGE ... reason_distance=true
L3013 [FRAME=106][STAGE=DECISION] step=LOCALIZATION_REATTACH result=REJECT reason=NO_CANDIDATES
L3022 [FRAME=106][STAGE=VERTEX] event=CREATE ... new_id=2 ... faiss_before=1 faiss_after=2
L3024 [FRAME=106][STAGE=EDGE] EDGE_TYPE=SEQUENTIAL source=1 target=2 ... still_added=true
L3031 [FRAME=106][STAGE=SUMMARY] vertex_before=1 vertex_after=2 decision=NEW_VERTEX ...
```

## 未选择的案例

本次 242 个成功帧的最终决策只有 `FIRST_VERTEX`、`KEEP` 和 `NEW_VERTEX`；8 个
`STAGE=EDGE` 事件全是成功加入的 `SEQUENTIAL` 边。全文没有观察到
`EDGE_SWITCH`、`LOCALIZATION_SWITCH`、`LOOP_CLOSURE`、`EDGE_TYPE=LOOP` 或
`LOOP_NEW_VERTEX`。因此不构造这些类型的“真实案例”。
