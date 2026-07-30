# PRISM-TopoMap 双架构日志验证报告

> 基线说明：本报告描述的是源码修改前的 `6b99c4d` 运行和代码状态。后续经用户授权实施的混合架构修改不回写本报告结论；源码行号以该分析基线为准。

## 1. 分析范围与运行版本

本报告只做代码、配置、launch 与日志的只读审计。临时解析脚本和中间数据位于 `/tmp`；没有修改源码、配置、launch、CMake 或已有日志。

安全检查结果：

| 项目 | 结果 |
|---|---|
| 项目路径 | `/home/tom/host_catkin_ws/src/PRISM-TopoMap` |
| 内层仓库分支 / HEAD | `test1` / `6b99c4d`（完整 SHA `6b99c4dabaa25b861bac719f8a2261c5dd9e71d4`，提交 `log1`） |
| 内层初始状态 | clean，`git status --short` 与 `git diff --stat` 均为空 |
| 外层仓库 | `master@f1c0262`，开始前已有 ` M PRISM-TopoMap` |
| 外层子模块记录 / 当前 checkout | `9dc7753f61290ef30df8bbe43ba5d78631a4816d` / `6b99c4d...`，该差异为分析前既有状态 |
| `pure_python_flow_trace.log` | 14,001,589 bytes，51,330 个物理行 |
| `full_trace.log` | 13,864,171 bytes，55,912 个物理行 |

实际审阅了任务指定的 Python 与 C++ 文件，包括 `scripts/prism_topomap_node.py`、`scripts/prism_topomap.py`、`scripts/localization.py`、`scripts/topo_graph.py`、`scripts/local_grid.py`、`scripts/models.py`、`scripts/tf_manager.py`、`src/prism_topomap_node.cpp`、`src/topo_slam_model.cpp`、`src/localizer.cpp`、`src/topological_graph.cpp`、`src/local_grid.cpp`、`src/inference_client.cpp`、`src/results_publisher.cpp`、对应头文件、`scripts/inference_service_node.py`、服务定义、两套 launch 与 `config/scout_rosbag.yaml`。

## 2. 日志解析方法与限制

解析过程先去除 ANSI 转义码，再按每个 `[FLOW]` 起点拆分同一物理行中的多个记录，分别解析 header 与 payload；所有跨架构比较以归一到小数点后 6 位的 ROS 点云时间戳为主键。解析得到：

- Python：42,404 条结构化 `[FLOW]` 记录；
- 混合架构：44,061 条结构化 `[FLOW]` 记录；
- 共同 `SUMMARY` 时间戳：727 个；
- 点云、descriptor、grid 等阶段独立配对，因此各字段的有效样本数会略高或略低于 727。

限制如下：

1. stdout、stderr 和多个 logger 会把一个 `[FLOW]` payload 插断；拆分能恢复同一行的多个记录，但不能补造被插入文本截断的字段。
2. 统计优先使用完整结构化记录；若关键记录损坏，则以原始事件 token、前后图不变量和最终计数交叉验证，并在相应位置明确说明。
3. 图边按无向边计数，不重复计算双向邻接表。
4. 两份日志终止 ROS 时间不同，最终图规模不能被当作严格等时性能比较。
5. 日志没有导出逐像素栅格，也没有提供两套架构在同一正确候选上的充分 registration 配对，因此不能宣称逐像素或完整 registration 等价。

## 3. 两套运行的基础统计

| 指标 | 纯 Python | C++/Python 混合 |
|---|---:|---:|
| 接收点云 | 3,422 | 3,808 |
| 已处理帧 | 2,683 | 约 1,030（末个完整 summary 为 FRAME 1029） |
| ROS 范围 | 1517156141.444386 ～ 1517156483.526704 | 1517156141.444386 ～ 1517156521.724593 |
| 最终顶点 | 39 | 74 |
| 最终无向边 | 38 | 139 |
| 最终 FAISS 大小 | 39 | 73 |
| FIRST_VERTEX | 1 | 1 |
| 普通 NEW_VERTEX | 38 | 27 |
| LOOP_NEW_VERTEX / LOOP_DETECTED | 0 | 46 |
| EDGE_SWITCH | 11 | 34 |
| LOCALIZATION_SWITCH | 0 | 10 |

混合运行比 Python 多覆盖约 38 秒 ROS 时间，但其节点过密并不只是运行更久：74 个节点中有 46 个由 `LOOP_NEW_VERTEX` 产生，超过普通阈值扩图的 27 个。

纯 Python 的 38 条边全部是顺序边。混合架构的 139 条最终边可由图不变量精确复核：

- 73 条顺序边：每个非首节点一条；
- 62 条新加入的回环候选边；
- 4 条真正新增的 localization 附加边；
- 合计 `73 + 62 + 4 = 139`。

混合日志还记录 45 个 `KEEP_EXISTING` 回环候选和 63 个拒绝项（16 个 `PREDICTED_LENGTH`、47 个 `GEOMETRY_INCONSISTENT`）。10 次 localization edge 事件中，4 次新增、3 次命中既有边、3 次是被 `TopologicalGraph::addEdge()` 拒绝的自环；该函数在 `src/topological_graph.cpp:102-108` 明确禁止自环。

## 4. 纯 Python 定位线程健康状态

### 结论 A1：异步定位 Timer 在第五次调用中因日志格式化异常永久退出

- **源码**：`scripts/localization.py:194-205,212-235`；`scripts/prism_topomap_node.py:614-651`。
- **日志**：`pure_python_flow_trace.log`，ROS stamp / FRAME `1517156149.343489 / 73`。
- **短摘录**：`transform=list(transform)`；`TypeError: 'NoneType' object is not iterable`。
- **调用栈**：`rospy/timer.py:240` → `scripts/prism_topomap_node.py:623 localize()` → `scripts/localization.py:235 localize()`。
- **解释**：registration 推理可返回 `transform=None`，分数分支本身允许该结果，但 trace 记录无条件执行 `list(transform)`。Timer wrapper 没有异常隔离，故 `Thread-16` 结束。异常前有 5 次 `TIMER_BEGIN`/FAISS；其中 4 次写出 `RESULT=COMPLETED`，第五次在 FAISS 后异常。异常后 `TIMER_BEGIN`、全局定位 FAISS、全局 registration 和 `RESULT=COMPLETED` 均为 0。
- **置信度**：高。

### 结论 A2：主循环在 Timer 死亡后长期重复消费最后一次定位结果

- **源码**：`scripts/prism_topomap.py:727-752`。
- **日志**：`pure_python_flow_trace.log`，最后有效 LOC_STAMP `1517156147.343636 / localization_frame=53`；例如 consumer FRAME 287、stamp `1517156172.042562`。
- **短摘录**：`LOC_STAMP=1517156147.343636 ... current_minus_localized_stamp=24.698926 fresh=true`。
- **解释**：2,683 个处理帧均消费 localization state；最后一个结果被消费 2,630 次。全程 `fresh=true` 共 277 次、`fresh=false` 共 2,406 次；创建 vertex 1 并清理相对位姿历史后，旧结果变为 stale，但仍被反复读取到日志终止。最终 Python 图主要由 odom/IoU、顺序扩图和 11 次沿边切换形成，没有持续全局定位或回环闭合。
- **置信度**：高。

因此，假设 A 被确认：本次纯 Python 运行不是健康的“完整算法基准”，不能用它的最终 RViz 图直接证明混合实现错误或正确。

## 5. 混合架构 descriptor 与 FAISS 索引一致性

### 结论 B1：首帧是 ROS service 客户端调用失败，仍创建了空 descriptor 顶点

- **源码**：`src/topo_slam_model.cpp:1166-1179` 首节点分支；`src/topo_slam_model.cpp:162-175` descriptor 失败处理；`src/topological_graph.cpp:65-83,281-289` 顶点与索引写入。
- **日志**：`full_trace.log`，stamp / FRAME `1517156141.444386 / 1`。
- **短摘录**：`SERVICE_RESULT success=false ... dim=0`；`event=CREATE ... new_id=0 ... descriptor_dim=0 ... faiss_before=0 faiss_after=0`。
- **解释**：首个调用在 C++ 客户端层失败；日志中没有对应的服务端 `SERVICE-1`，服务端首次成功请求是下一帧 `1517156141.844400`。模型清空当前 descriptor，但 mapping 首节点分支不验证 descriptor，仍调用 `addVertex()`。图从此为 1 个顶点、0 个 FAISS row。
- **置信度**：高。

### 结论 B2：FAISS row 被直接解释为 vertex ID，首帧缺失造成永久错位

- **源码**：`include/prism_topomap/topological_graph.h:123-129` 仅保存 vertices 与 `faiss_index_`，无 row→vertex 映射；`src/topological_graph.cpp:292-307` 返回原始 row label；`src/localizer.cpp:179-223` 将 label 直接传给 `graph_.getVertex(idx)`；`src/topological_graph.cpp:372-408` 重建索引时也跳过空 descriptor 且不建映射。
- **日志**：`full_trace.log`，stamp / FRAME `1517156172.042562 / 82`。
- **短摘录**：节点 1 创建时 `graph_vertices=2 ... faiss_size=1`；随后查询自身 descriptor 得 `candidates=[0:0.000000]`，并对 `candidate=0` 配准。
- **解释**：实际 FAISS row 0 存的是 graph vertex 1 的 descriptor，但 Localizer 把 label 0 解释成 graph vertex 0。自描述符距离 0 却指向前一顶点，构成可复现的完整 ID 错位证据，不只是计数推断。
- **置信度**：高。

第二条因果复核发生在 LOC_STAMP `1517156181.042201 / FRAME 106`：节点 2 的 descriptor 位于 FAISS row 1，查询自身得到 `[1:0.000000,0:2.302958]`，随后 row 1 被解释成 graph vertex 1。这组错误候选直接进入第 6 节的首个永久拓扑分叉。

维度防护也不充分：`addToIndex()` 只拒绝空向量，没有检查长度是否等于 `descriptor_dim_`；`searchIndex()` 同样没有 query 长度检查。非空错误维度可能把不匹配的内存范围交给 FAISS。图加载重建也没有显式 row→vertex identity。

假设 B 被确认并加强：根因不是“首帧模型推理失败”，而是**一次 service transport/client 失败未被节点创建事务吸收，继而暴露了 FAISS row 与 graph ID 隐式同一的设计缺陷**。

## 6. 第一处永久控制流分叉

### 第一处数值差异

共同首帧 `1517156141.444386` 已出现小的 grid 统计差异：Python 为 `unknown/free/occupied=126680/2635/285`，混合为 `126677/2638/285`。点云 raw、finite、in-range、obstacle 完全一致，差异只有 3 个 free/unknown cell，不是拓扑立即分裂的证据。两套首帧全局位姿来源也已经不同：Python 是 GT buffer，混合是旧 odom。

### 第一处决策差异，但尚未形成持续数量差异

- **日志**：共同 stamp `1517156180.642200`；Python FRAME 370，混合 FRAME 105。
- **短摘录**：Python `rel_dist=5.005524 ... decision=NEW_VERTEX`；混合 `rel_dist=4.9778 ... decision=KEEP`。
- **解释**：Python 在此先创建 vertex 2；混合到下一共同 stamp 才越过阈值，节点数随后短暂对齐。这是处理频率、odom 同步和位姿来源差异造成的阈值时序变化，不是节点数永久拉开的时刻。
- **置信度**：高。

### 第一处导致拓扑永久不同的控制流差异

关键事件按 ROS 时间排序如下：

| 顺序 | ROS stamp / FRAME | 事件 |
|---:|---|---|
| 1 | `1517156180.642200` / hybrid 105 | 混合 vertex 1 的 `rel_dist=4.9778`，先 KEEP；Python 已创建 vertex 2。 |
| 2 | `1517156181.042201` / hybrid 106 | 混合以 `rel_dist=5.3755` 创建 vertex 2；Python同一 stamp 已在 vertex 2 内，仅 `rel_dist=0.401449`。两边节点数重新对齐。 |
| 3 | LOC_STAMP `1517156181.042201` / hybrid 106 | 错位 FAISS 返回 `[1:0.000000,0:2.302958]`。 |
| 4 | 同一 LOC_STAMP | candidate 1：score `0.648673`、metric pose `(1.0540,-0.0477,-0.039505)`；candidate 0：score `0.718640`、pose `(2.9721,-0.1561,-3.129775)`，两者均被标为 matched。 |
| 5 | `1517156181.442070` / hybrid 107 | 消费上述快照，age `0.399868`、fresh=true；候选成为 `[1:1.0551,0:2.9762,2:0]`。 |
| 6 | 同一 frame | `(u=1,v=2)` 的 path `5.3755`，through-current `1.0551`，三个门槛均通过，记录 `LOOP_DETECTED`。 |
| 7 | 同一 frame | 先创建 vertex 3，再无条件加入顺序边 `2→3`，长度 `0.4446`。 |
| 8 | 同一 frame | `3→1` 与 `3→0` 后验几何校验分别以 ratio `3.8810`、`4.2736` 拒绝；`3→2` 只是已经存在的顺序边。 |
| 9 | 同一 frame | 仍 `SET_CURRENT vertex=3`，最终 `graph_vertices=4 graph_edges=3 faiss_size=3`；Python仍为 3 顶点、2 边。 |

### 关键证据

- **源码**：`src/topo_slam_model.cpp:258-275,282-360,1233-1267,747-764,815-941`。
- **日志**：`full_trace.log`，LOC_STAMP `1517156181.042201 / FRAME 106` 与 stamp `1517156181.442070 / FRAME 107`。
- **短摘录**：`event=LOOP_DETECTED u=1 v=2` → `event=CREATE ... new_id=3` → `EDGE_TYPE=SEQUENTIAL source=2 target=3` → 两条 `action=REJECT reason=GEOMETRY_INCONSISTENT` → `event=SET_CURRENT vertex=3`。
- **解释**：错误索引候选通过的是候选对之间的图路径捷径门槛，不是“新节点到候选边”的最终几何一致性。节点和顺序边已经永久写入后，候选回环边才被拒绝；拒绝边没有回滚回环事件本身。
- **置信度**：高。

## 7. 回环检测、节点创建与边校验因果链

`findLoopClosure()` 在 `src/topo_slam_model.cpp:282-360` 遍历定位候选对，条件主要是：

1. 图路径长度大于 5 m；
2. 图路径大于经当前位姿距离的 2 倍；
3. `checkPathCondition()` 通过。

`checkPathCondition()` 位于 `src/topo_slam_model.cpp:258-275`，其中 straight length 小于 10 m 会直接接受；它没有验证待创建节点到每个 localization candidate 的全局几何一致性。`update()` 在 `src/topo_slam_model.cpp:1261-1264` 一旦返回 true 就调用 `addNewVertex()`。

`addNewVertex()` 的提交顺序为：

1. `src/topo_slam_model.cpp:747-764` 创建并索引新顶点；
2. `src/topo_slam_model.cpp:815-834` 无条件建立上一顶点到新顶点的顺序边；
3. `src/topo_slam_model.cpp:841-923` 才检查和添加候选回环边；
4. `src/topo_slam_model.cpp:928-941` 无论候选边是否全被拒绝，都将新顶点设为 current。

在 45 个能完整关联候选结果的 LOOP_NEW_VERTEX 事件中，至少 11 个没有增加任何非平凡回环边：候选不是被拒绝，就是仅命中已经存在的边。另 1 个 LOOP 事件受日志插断，不能补造其候选结果。由此可确认“错误边被拒绝”不等于“错误回环节点被抑制”。

### 结论 C

- **源码**：`src/topo_slam_model.cpp:258-360,747-941,1261-1264`。
- **日志**：首个完整事件为 `full_trace.log` stamp / FRAME `1517156181.442070 / 107`；全局计数 `LOOP_DETECTED=46`。
- **短摘录**：`decision=LOOP_NEW_VERTEX ... graph_vertices=4 graph_edges=3`，但该事件的两条非顺序候选都为 `GEOMETRY_INCONSISTENT`，第三条 `KEEP_EXISTING`。
- **解释**：混合图的额外节点主要由 46 次 LOOP_NEW_VERTEX 产生；事件判定与边几何校验不是一个原子决策，导致“空回环节点”和很短的顺序边簇。
- **置信度**：高。

假设 C 被确认。

## 8. 节点和边来源统计

### 纯 Python

| 类别 | 数量 | 口径 |
|---|---:|---|
| FIRST_VERTEX | 1 | 首节点 |
| 普通 NEW_VERTEX | 38 | summary decision |
| LOOP_NEW_VERTEX | 0 | summary decision |
| 顺序边 | 38 | `EDGE ... SEQUENTIAL`，最终边数复核 |
| localization 附加边 | 0 | 无 LOCALIZATION_SWITCH |
| EDGE_SWITCH | 11 | decision token |
| LOCALIZATION_SWITCH | 0 | decision token |

### 混合架构

| 类别 | 数量 | 口径 |
|---|---:|---|
| FIRST_VERTEX | 1 | 首节点 |
| 普通 NEW_VERTEX | 27 | `74 - 1 - 46`，与完整 summary token 交叉检查 |
| LOOP_NEW_VERTEX / LOOP_DETECTED | 46 | 原始 `event=LOOP_DETECTED` token；部分 summary 被 logger 插断 |
| 顺序边 | 73 | 每个非首节点一条，最终图不变量复核 |
| localization 附加边事件 | 10 | 4 新增、3 既有、3 自环拒绝 |
| 回环候选新增 | 62 | action=ADD |
| 回环候选既有 | 45 | action=KEEP_EXISTING |
| 回环候选拒绝 | 63 | 16 predicted-length + 47 geometry |
| EDGE_SWITCH | 34 | decision token |
| LOCALIZATION_SWITCH | 10 | decision token |

46 个 LOOP_NEW_VERTEX 占全部 73 个非首节点的 63.0%，是混合图节点密度异常的直接结构来源；普通 NEW_VERTEX 反而少于纯 Python。但由于 Python Timer 早死、运行时间不同、处理频率不同，不能把 `27` 与 `38` 直接解释为算法精度优劣。

## 9. 实验条件差异

两套日志均加载 `config/scout_rosbag.yaml`。关键算法文本配置相同：

| 参数 | 值 | 文件 |
|---|---:|---|
| `iou_threshold` | 0.3 | `config/scout_rosbag.yaml:22` |
| `max_edge_length` | 5.0 | `config/scout_rosbag.yaml:23` |
| `localization_frequency` | 2.0 | `config/scout_rosbag.yaml:24` |
| `localization_timeout` | 0.0 | `config/scout_rosbag.yaml:25` |
| `drift_coef` | 0.3 | `config/scout_rosbag.yaml:27` |
| `pcd_process_interval` | 0.3 | `config/scout_rosbag.yaml:28` |
| model / weights / `top_k` / quantization | MinkLoc3D / 同一路径 / 5 / 0.2 | `config/scout_rosbag.yaml:30-35` |
| grid resolution / radius / range | 0.1 / 18 / 15 | `config/scout_rosbag.yaml:36-39` |
| registration threshold | 0.6 | `config/scout_rosbag.yaml:40-46` |
| inline threshold / jump | 0.5 / 1.0 | `config/scout_rosbag.yaml:47-55` |

但运行语义并不相同：

### D1：同一 localization 参数被解释成不同单位

- **源码**：Python `scripts/prism_topomap_node.py:542-548` 直接用 `rospy.Duration(2.0)`，即周期 2 秒；C++ `src/prism_topomap_node.cpp:173-180` 用 `1.0 / 2.0`，即周期 0.5 秒。
- **日志**：`pure_python_flow_trace.log` 启动记录为 `localization_timer_period=2.0`，稳态 LOC_STAMP 示例为 `1517156143.044111 / FRAME 10`、`1517156145.344010 / FRAME 32`；`full_trace.log` 从 LOC_STAMP `1517156141.844400 / FRAME 2` 起以约 0.5 秒 wall timer 读取快照，且至少有 699 条完整定位结果。
- **短摘录**：Python `localization_timer_period=2.0`；C++ 构造实际周期为 `1.0 / loc_freq`。
- **解释**：混合架构名义异步定位频率是 Python 的 4 倍，直接改变候选可用时机、回环入口次数和服务负载。
- **置信度**：高。

### D2：仅混合路径执行 0.3 秒点云间隔节流

- **源码**：C++ `src/prism_topomap_node.cpp:611-623`；Python 点云回调 `scripts/prism_topomap_node.py:828-892` 没有相同 interval gate。
- **日志**：`full_trace.log` 在首帧后 stamp `1517156141.544229 / FRAME=UNASSIGNED` 即记录节流；`pure_python_flow_trace.log` stamp `1517156141.444386 / FRAME 1` 明确记录无 interval gate。
- **短摘录**：混合 `result=SKIPPED_INTERVAL interval=0.300000`；Python `interval_throttle_present=false`。
- **解释**：Python 处理 2,683/3,422，混合约 1,030/3,808。处理时间戳和 odom 累积步长不同，能改变 5 m 阈值跨越时刻、grid accumulation、定位快照和最终拓扑，绝非只影响性能。
- **置信度**：高。

### D3：global pose 与 odom 同步不同

- **源码**：Python 点云时刻插值 `scripts/prism_topomap_node.py:699-769`，GT 由 `scripts/tf_manager.py:94-122` 发布；C++ 从可用 buffer 取样并在无 GT 时回退 odom，见 `src/prism_topomap_node.cpp:333-418`。
- **日志**：首帧 Python `global_pose_source=GT_POSE_BUFFER ... odom_delta=0 ... INTERPOLATED_ODOM`；混合 `GT ... no data ... Falling back to /odom`，首帧 `dt_odom=0.056930`。混合有效样本的旧 odom age 中位数 0.046548 s、P95 0.055556 s、最大 0.057374 s；Python对应 delta 全为 0。
- **解释**：global geometry gate、节点可视位置和 RViz 图都会受影响。日志中的混合样本在点云时刻之前约 47 ms；代码策略是选当前可用样本，并非显式要求“旧样本”。
- **置信度**：高。

假设 D 被确认。launch 分别为 `launch/build_map_by_iou_scout_rosbag.launch:1-21` 与 `launch/build_map_by_iou_scout_rosbag_hybrid.launch:1-40`；YAML 相同不能消除上述运行时语义差异。

## 10. LocalGrid、descriptor 与 odom 数值对比

共同时间戳、同阶段的统计如下。`N` 因记录损坏和阶段可用性而异。

| 指标 | 配对 N | 结果 |
|---|---:|---|
| raw points | 732 | 732/732 完全相同 |
| finite points | 732 | 732/732 完全相同 |
| in-range points | 734 | 734/734 完全相同 |
| obstacle points | 732 | 732/732 完全相同 |
| occupancy unknown 绝对差 | 728 | 中位 24，P95 125，最大 415；完全相同 13 次 |
| occupancy occupied | 725 | 722 次完全相同，最大差 1 |
| density nonzero 绝对差 | 可配对样本 | 中位约 86,670，受不同处理频率与累积/插值强烈影响 |
| height nonzero | 722 | 721 次完全相同，最大差 1 |
| descriptor L2 绝对差 | 735 | 中位 0.003493，P95 0.015183，最大 0.040646 |
| descriptor head | 有效配对样本 | 各分量 P95 约 0.005；一条被 logger 插断的异常字段已排除 |
| 首次永久分叉前 IoU 绝对差 | 95 | 中位 0.001503，P95 0.013060，最大 0.029740 |
| Python odom 对点云时间差 | 2,683 | 0（插值） |
| 混合 odom 对点云绝对 age | 1,025 | 中位 0.046548 s，P95 0.055556 s，最大 0.057374 s |

### 变换代码审计

- Python warp 与变换矩阵：`scripts/local_grid.py:179-200,209-253`；
- C++ warp 与变换矩阵：`src/local_grid.cpp:163-202,416-497`。

坐标矩阵、角度方向、IoU 对齐和 pixel→metric 形式一致，没有日志证据支持 x/y、i/j 对调、theta 符号反向、内部栅格转置或 warp 方向反向等严重错误。

但实现并非数值完全相同：

- Python `cv2.warpAffine` 使用默认线性插值，C++ 在 `src/local_grid.cpp:199-201` 明确用 `cv::INTER_NEAREST`；
- Python 在 `scripts/local_grid.py:100-108` 使用 `np.round`（tie-to-even），C++ 在 `src/local_grid.cpp:297-302` 使用 `std::round`（半值远离零）；
- 处理频率和 odom sampling 不同，导致 density accumulation 的大差异；
- occupancy unknown/free 有小但稳定的统计差异。

### 结论 F

- **源码**：上述 LocalGrid 位置及 `scripts/models.py`、`scripts/inference_service_node.py` 的同模型推理路径。
- **日志**：共同时间戳 727 个 summary；例 `1517156141.444386 / FRAME 1` 点数完全一致，descriptor L2 为 9.521950（Python）而混合首帧无 descriptor，下一共同成功样本则紧密接近。
- **短摘录**：共同成功样本中 raw/finite/in-range/obstacle 一致，descriptor L2 差的 P95 仅 0.015183。
- **解释**：没有 gross coordinate/descriptor 错误证据；LocalGrid 和 descriptor 差异不是已观察节点爆炸的首要根因。但现有日志不足以证明逐像素等价，插值与 rounding 差异确实需要后续对齐测试。
- **置信度**：高（排除严重错误）；中（“不是任何边界事件的诱因”仍不能完全排除）。

由于 Python 全局定位在 8 秒左右死亡、混合候选又从首节点起错位，现有日志没有足够的“同一快照、同一正确候选” registration 样本用于公平对照。

## 11. 已确认问题

1. **P0：混合架构 graph vertex ID 与 FAISS row ID 失去一致性。** 首帧空 descriptor 节点造成永久错位，且实现没有显式映射、维度校验或加载重建 identity。
2. **P0：回环节点创建不是事务性决策。** 回环候选只需先通过路径捷径条件就提交节点与顺序边；后续边全部拒绝也不会回滚。
3. **P0：纯 Python Timer 可被 trace 格式化异常杀死。** 本次基准从 `1517156149.343489` 后没有新的异步全局定位。
4. **P0：对照实验语义不一致。** 同一 `localization_frequency=2.0` 分别表示 2 s 周期和 2 Hz；仅混合执行 0.3 s 点云节流；GT/odom 与同步策略不同。
5. **P1：零时间戳 freshness 语义不一致。** 见第 12 节。
6. **P1：localization reattach 的几何保护不足。** `src/topo_slam_model.cpp:650-704` 在 `need_to_change_vcur_` 为 true 时允许绕过 IoU。日志 stamp `1517156328.134530` 的 localization edge `29→24`，预测 1.5338 m、全局直线 6.6898 m、ratio 4.3616，仍记录 action=ADD。Python `scripts/prism_topomap.py:498-505` 有相同继承行为；在 FAISS ID 错位时风险尤其高。
7. **P2：部分 edge 日志把自环写成候选成功语义，实际底层拒绝。** 这会污染事件统计，但最终图不变量正确。

`src/results_publisher.cpp:82-207` 只遍历已存图进行可视化，不创建节点或边。因此 RViz 节点簇主要反映模型已提交的图状态，不是 publisher 重复绘制造成的拓扑膨胀。Python publisher 的 grid 转置路径 `scripts/prism_topomap_node.py:363-405` 可能影响栅格显示方向，但不会改变内部拓扑决策。

## 12. 被否定或证据不足的假设

### E：freshness 差异被确认，但不是本次节点爆炸的首要根因

- **源码**：C++ `src/topo_slam_model.cpp:1186-1198` 把 `localized_stamp <= 0` 判为 fresh；Python `scripts/prism_topomap.py:727-752` 在有 pose history 时把 0 判为 stale。
- **日志**：`full_trace.log` stamp / FRAME `1517156141.844400 / 2`。
- **短摘录**：`result_stamp=0.000000 age=-1.000000 fresh=true ... has_time_reference=false`；Python首帧则 `LOC_STAMP=0 ... fresh=false`。
- **解释**：C++ 不会在 stamp 0 时建立时间参考，但会进入 fresh mapping 分支；候选为空时通常不会直接触发 loop，后续 fallback 又可能把 `current_stamp - 0` 当作 stale。它是真实控制流差异，可能改变降级/新节点时机，但本次首个永久分叉已有更直接的错位候选证据。
- **置信度**：高（差异存在）；中（本次直接影响程度）。

### 被否定或尚无证据支持

- **“首帧 Python inference 模型返回失败”**：被日志否定。首帧没有服务端 handler 记录，是 C++ `descriptor_client_.call()` 失败；下一请求服务端成功。
- **“严重坐标轴/角度/warp 方向错误导致节点爆炸”**：当前日志和矩阵审计不支持；点数、descriptor 与早期 IoU 都接近。
- **“混合图的 74 个节点主要来自普通距离阈值”**：被计数否定；46 个来自 LOOP_NEW_VERTEX，普通 NEW_VERTEX 只有 27 个。
- **“回环边拒绝就意味着回环节点被拒绝”**：被首个完整分叉和至少 11 个空回环事件否定。
- **“RViz publisher 重复创建了节点簇”**：被代码审计否定；publisher 只读图。
- **“纯 Python 最终图是健康全功能基准”**：被 Timer 异常和异常后零全局定位记录否定。
- **“两套 LocalGrid 已逐像素等价”**：证据不足；现有日志只有统计量，且插值/rounding 明确不同。

## 13. 根因优先级与置信度

| 优先级 | 根因 | 对现象的作用 | 置信度 |
|---:|---|---|---|
| 1 | 首帧空 descriptor 顶点 + 无 FAISS row→vertex 映射 | 从第一个可索引节点起制造确定性错误候选；直接进入错误 registration、loop 与 switch | 高 |
| 2 | 回环判断先提交节点/顺序边，候选边后验校验且不回滚 | 将错误候选放大为 46 个 LOOP_NEW_VERTEX；即使边拒绝也保留节点簇 | 高 |
| 3 | 对照实验 timer、PCD cadence、global pose、odom sampling 不同 | 改变候选可用时机、阈值跨越、全局几何与 grid 累积；妨碍公平对照 | 高 |
| 4 | Python Timer 因 `None` trace 异常早死 | 使 Python“基准”失去持续全局定位，最终图不具可比性 | 高 |
| 5 | localization reattach 可绕过 IoU 且缺少同级全局几何拒绝 | 可把错误候选变成错误 current/附加边，放大索引错位后果 | 中高 |
| 6 | stamp 0 freshness 语义不一致 | 改变空定位状态的分支语义，但缺少其直接制造本次首个永久分叉的证据 | 中 |
| 7 | LocalGrid 插值/rounding 与采样差异 | 可影响边界 IoU/registration；当前没有严重坐标错误证据 | 中 |

主因链条为：

`首帧 service 调用失败` → `vertex 0 无 descriptor 但已提交` → `FAISS row 与 graph ID 永久错位` → `错误候选获得 registration match` → `弱路径门槛触发 LOOP_DETECTED` → `先创建节点和短顺序边` → `候选边后验拒绝但节点不回滚` → `节点过密/节点簇分裂，并可能伴随错误切换`。

## 14. 当前不能下结论的事项

1. 不能从统计量证明两套 occupancy、height、density 栅格逐像素一致；需要同点云、同 odom delta、同初始 grid 的逐像素导出。
2. 不能公平比较完整 global registration 数值等价性；必须先修复 Python Timer 和混合 FAISS identity，并统一候选集合与快照。
3. 不能用本次最终 39 vs 74 节点直接评判哪种架构“更正确”；运行时长、处理 cadence、global pose 和异步定位健康状态均不同。
4. 不能确定所有 46 个 LOOP_NEW_VERTEX 中恰有多少是索引错位单独造成；日志能证明最早事件和多个后续事件，但不同运行条件也会改变候选集合。
5. 不能由最终 RViz 外观量化轨迹真值误差；需要与统一 GT 或离线评价基准对齐。
6. 不能确认首帧 service transport 失败的外部原因（连接瞬态、并发启动或其他 ROS 状态）；日志只确定调用失败而服务已被发现。

后续公平等价性比较必须先依赖修改计划中的 P0 项：修复 descriptor/index identity、阻止空回环节点、恢复 Python Timer 健康，并统一 timer/cadence/global pose/odom 同步语义。
