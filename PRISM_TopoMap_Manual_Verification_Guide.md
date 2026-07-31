# PRISM-TopoMap 本次修改的手动验证指南

本指南只针对当前 C++/Python 混合架构。请不要覆盖原来的 `full_trace.log` 和 `pure_python_flow_trace.log`。建议先做首帧短测，再做完整 rosbag 回归。

路径说明：宿主机工作空间是 `/home/tom/host_catkin_ws`；Docker 容器内对应路径是 `/home/docker_prism/catkin_ws`，即你当前使用的 `~/catkin_ws`。以下命令若在 Docker 内执行，请把 `/home/tom/host_catkin_ws` 换成 `~/catkin_ws`。

## 1. 本次需要验证的行为

1. 推理服务启动后即使空闲较长时间，首个 descriptor 和 registration 请求仍能成功；如果首次传输失败，同一请求会重建客户端并最多重试一次。registration 明确返回 `NO_TRANSFORM` 时不重试。
2. 无效 descriptor 不能创建拓扑节点。
3. FAISS row 通过显式映射返回真实 graph vertex ID；正常 mapping 运行中 `graph_vertices == faiss_size == faiss_identity`。
4. “可能存在回环”只会成为候选。回环检测所依据的两个触发端点必须都由顺序边或通过几何检查的回环边覆盖；无关候选不能促成 `LOOP_NEW_VERTEX`。
5. 默认关闭混合架构独有的 `0.3 s` PCD interval；性能采样只能通过显式 launch 参数开启。
6. Scout rosbag 的回环边同时检查全局距离绝对误差、距离比例和朝向误差；启动日志会打印实际门槛。
7. `need_change` 不再绕过定位候选的 IoU 和几何检查；当前节点自身不能成为 localization switch 目标。
8. `EDGE_SWITCH` 和新节点顺序边都必须与 GT 全局位姿一致；顺序边长度上限为 5.5 m，给 5 m 建点门槛保留一帧超调。
9. 回环的 current 端点只有在顺序边预检通过后才算被覆盖。
10. Scout 严格要求 GT；GT 尚未到达时丢弃该点云，不再用 ODOM 创建首节点。
11. 仅由 IoU 触发的切换必须连续 3 帧低于门限，且相对当前节点至少移动 1.0 m；距离超过 5 m或移出有效区域不受这两个门槛影响。
12. 当前节点附近 0.5 m内检测到回环时直接复用当前节点并提交回环边，不再创建厘米级 `LOOP_NEW_VERTEX`。
13. 与机器人相距不超过 1.0 m的旧候选，在 registration、位姿一致性和最低 IoU 均通过时允许以 `CLOSE_GEOMETRY` 模式复用，避免先拒绝切换再创建重复节点。

## 2. 构建

在任意终端执行：

```bash
cd /home/tom/host_catkin_ws
catkin_make --pkg prism_topomap -j2
source /home/tom/host_catkin_ws/devel/setup.bash
```

预期最后出现：

```text
[100%] Built target prism_topomap_cpp_node
```

如果曾出现 `tf_manager.py` 无法导入 `flow_trace`，可在 Docker 内先检查：

```bash
cd ~/catkin_ws
python3 -B -c "import sys; sys.path.append('$HOME/catkin_ws/src/PRISM-TopoMap/scripts'); from flow_trace import FlowTracer; print('flow_trace import OK')"
```

当前修复后的预期输出是 `flow_trace import OK`。

## 3. 首帧与长空闲连接短测

这个测试专门复核原日志中的首帧 service 调用失败。建议先启动节点，等待 180 秒后再播放 bag，以覆盖比原运行更长的空闲时间。

### 终端 0：ROS master 与仿真时间

先启动：

```bash
roscore
```

然后在另一个终端、启动节点之前执行：

```bash
rosparam set /use_sim_time true
```

如果你已经按原有流程启动了 roscore 并设置仿真时间，可以跳过这一步。

### 终端 A：启动混合架构并保存节点日志

```bash
source /home/tom/host_catkin_ws/devel/setup.bash
cd /home/tom/host_catkin_ws/src/PRISM-TopoMap
roslaunch prism_topomap build_map_by_iou_scout_rosbag_hybrid.launch \
  trace_data_flow:=true \
  trace_every_n_processed_frames:=1 \
  trace_descriptor_head_size:=4 \
  trace_registration_candidates:=true \
  pcd_process_interval:=0.0 \
  2>&1 | tee hybrid_after_registration_fix_startup.log
```

看到以下信息后先不要播放 bag：

```text
Descriptor service is ready (persistent=false, transport_retries=1).
Registration service is ready (persistent=false, transport_retries=1).
PCD performance sampling: disabled
```

保持节点空闲至少 180 秒。

### 终端 B：短时间播放 rosbag

继续使用你原来验证该工程时的 rosbag 命令和参数，只把运行时长限制在约 20～30 秒。例如：

```bash
source /home/tom/host_catkin_ws/devel/setup.bash
rosbag play --clock --duration=30 <你的_rosbag_绝对路径>
```

如果你的原命令包含 `--start`、remap 或播放速率，请保留原参数。短测结束后先等待约 5 秒，让日志完成刷新，再在终端 A 按一次 `Ctrl-C`。

### 首帧通过标准

在项目目录执行：

```bash
rg -n "PCD performance sampling|HANDLER_BEGIN|REGISTRATION_SERVICE|SERVICE_RESULT|event=CREATE|event=SKIP_CREATE|\\[FAISS\\]" \
  hybrid_after_registration_fix_startup.log | head -n 160
```

正常首帧应满足：

- 服务端出现 `FRAME=SERVICE-1 ... action=HANDLER_BEGIN`，stamp 与 C++ FRAME 1 相同；
- C++ 出现 `SERVICE_RESULT success=true transport_success=true call_id=1`；
- `attempts=1` 表示首调直接成功；
- 若出现 `attempts=2`，表示首次传输仍失败，但同一帧重连恢复成功。请保留完整日志，我会继续定位首次失败原因；
- 首个 `event=CREATE` 应为 `descriptor_dim=256 faiss_before=0 faiss_after=1 faiss_identity=1`；
- 不应出现首节点 `descriptor_dim=0`。
- 首条 `type=localization` 的 `REGISTRATION_SERVICE action=SERVICE_RESULT` 应为
  `transport_success=true attempts=1`；如果为 `attempts=2`，表示首次传输失败后已在同一请求恢复。
- `result=NO_TRANSFORM transport_success=true attempts=1` 表示服务正常响应但算法没有找到变换，不属于连接故障，也不会触发传输重试。
- 不应再出现旧格式的 `GridRegistration service call failed!`；新的不可恢复传输故障会明确记录为
  `result=TRANSPORT_FAILURE transport_success=false attempts=2`。

如果两次 transport 都失败，预期安全行为是：

```text
event=SKIP_CREATE reason=INVALID_DESCRIPTOR
decision=WAIT_DESCRIPTOR
graph_vertices=0 ... faiss_size=0 faiss_identity=0
```

下一次 descriptor 成功后才应创建第一个节点。即使发生这种情况，也不应再产生 graph/FAISS ID 偏移。

## 4. 节点复用与位姿一致性完整回归

请重新启动终端 A，并使用新的文件名；不要覆盖上一轮已经生成的
`full_trace_after_loop_consistency_fix.log`：

```bash
source /home/tom/host_catkin_ws/devel/setup.bash
cd /home/tom/host_catkin_ws/src/PRISM-TopoMap
roslaunch prism_topomap build_map_by_iou_scout_rosbag_hybrid.launch \
  trace_data_flow:=true \
  trace_every_n_processed_frames:=1 \
  trace_descriptor_head_size:=4 \
  trace_registration_candidates:=true \
  pcd_process_interval:=0.0 \
  2>&1 | tee full_trace_after_vertex_reuse_fix.log
```

在另一个终端用原参数播放同一个 bag。建议先播放到约 245 秒，快速覆盖上一轮约 217 秒之后出现的密集回环节点区段；若节点增长正常，再完整播放到你之前采用的约 320 秒。停止 rosbag 后等待约 5 秒，让节点处理完队列，再停止 roslaunch。

最终需要分析的文件是 `full_trace_after_vertex_reuse_fix.log`。

## 5. 完整运行的快速人工检查

### 5.1 PCD 限流确实关闭

```bash
rg -n "PCD performance sampling|SKIPPED_INTERVAL" full_trace_after_vertex_reuse_fix.log
```

预期只有 `PCD performance sampling: disabled`，没有 `SKIPPED_INTERVAL`。

### 5.2 descriptor 与索引身份

```bash
rg -n "SERVICE_RESULT|event=SKIP_CREATE|event=CREATE|event=SET_CURRENT|\\[FAISS\\] Added" \
  full_trace_after_vertex_reuse_fix.log | head -n 200
```

正常新图应始终满足：

```text
graph_vertices == faiss_size == faiss_identity
```

新增节点的典型顺序应是：

```text
[FAISS] Added row=N vertex=N
event=CREATE ... new_id=N ... faiss_after=N+1 faiss_identity=N+1
event=SET_CURRENT ... faiss_size=N+1 faiss_identity=N+1
```

加载含空 descriptor 的旧图时允许 `graph_vertices > faiss_size`，但必须出现明确的 `Loaded vertex ... remains unindexed`，且 `[FAISS] Added row=... vertex=...` 显示显式映射。

### 5.3 自描述符不再指向前一个节点

重点查看第二个、第三个节点创建后的 FAISS 查询：

```bash
rg -n "new_id=1|new_id=2|STAGE=FAISS" full_trace_after_vertex_reuse_fix.log | head -n 120
```

如果查询快照正好来自新节点自身，距离 `0.000000` 的候选 ID 应等于该节点 ID。例如节点 1 的自身 descriptor 应返回 `1:0.000000`，不能再返回 `0:0.000000`。

### 5.4 回环节点必须有有效的新回环边

```bash
rg -n "Loop edge validation|LOOP_CANDIDATE_DETECTED|LOOP_PRECHECK|LOOP_TRIGGER_PRECHECK|LOOP_REUSE_CURRENT|LOOP_REJECTED|LOOP_DETECTED|LOOP_NEW_VERTEX|valid_loop_edges" \
  full_trace_after_vertex_reuse_fix.log
```

预期规则：

- 启动阶段应打印 `max_abs_distance_error=1.500`、`max_distance_ratio=2.000`、`max_yaw_error=0.500`；
- `LOOP_CANDIDATE_DETECTED` 只表示检测到候选，不代表已经建点；
- 创建新回环节点的候选必须出现同 FRAME 的 `LOOP_TRIGGER_PRECHECK`，其中 `trigger_u/trigger_v` 与候选事件一致；直接复用当前节点的分支改为记录 `EDGE_TYPE=LOOP_REUSE_CURRENT`；
- `action=REJECT` 或 `u_covered=false` / `v_covered=false` 时，应出现
  `LOOP_REJECTED reason=TRIGGER_ENDPOINTS_NOT_COVERED`，同一 FRAME 不得出现
  `LOOP_DETECTED result=CONFIRMED` 或 `decision=LOOP_NEW_VERTEX`；
- 被拒绝的回环帧若随后独立满足普通 5 m 距离扩图条件，可以出现
  `decision=NEW_VERTEX`，这不属于回环建点；
- `LOOP_DETECTED result=CONFIRMED commit=REUSE_CURRENT` 前必须出现
  `EDGE_TYPE=LOOP_REUSE_CURRENT ... action=ADD/KEEP_EXISTING`；需要创建新节点的确认事件前仍必须出现
  `LOOP_TRIGGER_PRECHECK ... u_covered=true v_covered=true ... action=ACCEPT`；
- 已确认事件、预检事件和候选事件的 `trigger_u/trigger_v` 必须一致；
- 对应 `SET_CURRENT` 的 `valid_loop_edges` 可为 1（另一端由顺序边覆盖）或 2（两个端点都由回环边覆盖），但不能靠触发端点之外的候选通过。

重点检查上一轮异常区段是否还会形成固定 0.5 秒间隔的密集节点：

```bash
rg -n "decision=LOOP_NEW_VERTEX|event=LOOP_REJECTED|LOOP_TRIGGER_PRECHECK" \
  full_trace_after_vertex_reuse_fix.log
```

预期 FRAME 编号不必与上一轮完全一致，但不应再出现类似“约 13.5 秒连续创建
15 个回环节点”的簇。

### 5.5 current 切换与顺序边

```bash
rg -n "require_gt_pose|global_source=ODOM|SAME_AS_CURRENT|POSE_INCONSISTENT|LOCALIZATION_SWITCH|EDGE_SWITCH|SEQUENTIAL_PRECHECK|WAIT_VERTEX_CONSISTENCY" \
  full_trace_after_vertex_reuse_fix.log
```

预期规则：

- 启动日志出现 `require_gt_pose: true`，所有已处理 FRAME 的
  `global_source` 都应为 `GT`；
- 可以出现 `reason=SAME_AS_CURRENT`，但不得再出现
  `event=LOCALIZATION_SWITCH from=N to=N`；
- `need_change=true` 的候选仍必须通过标准 IoU，或通过近距离复用的最低 IoU，并同时通过位姿一致性检查；
- 被记录为 `reason=POSE_INCONSISTENT` 的 localization/edge switch 不得改变
  `vertex_after`；
- 每个 `event=CREATE` 之前必须有同 FRAME 的
  `EDGE_TYPE=SEQUENTIAL_PRECHECK ... action=ACCEPT`，首节点除外；
- `SEQUENTIAL_PRECHECK action=REJECT` 后不得在同 FRAME 创建节点；
- 正常顺序边的 `predicted_length` 与 `global_direct_length` 应接近，且二者均不超过 5.5 m。

重点复核旧日志 FRAME 394 和 FRAME 1802 附近：原来的 `1→1`、`20→20`
自切换必须变成 `reason=SAME_AS_CURRENT`，随后应按普通 5 m 规则创建新节点。

### 5.6 IoU 迟滞、近距离回环和旧节点复用

```bash
rg -n "Vertex reuse policy|WAIT_IOU_CONFIRMATION|WAIT_IOU_MIN_DISTANCE|LOOP_REUSE_CURRENT|reuse_mode=CLOSE_GEOMETRY|event=CREATE" \
  full_trace_after_vertex_reuse_fix.log
```

预期规则：

- 启动日志打印 `iou_confirm_frames=3`、`iou_min_creation_distance=1.000`、`loop_reuse_current_distance=0.500`、`localization_reuse_min_iou=0.100` 和 `localization_reuse_max_distance=1.000`；
- 第一次和第二次单纯跌破 IoU 门限分别进入 `WAIT_IOU_CONFIRMATION`，不得在同 FRAME 出现 `event=CREATE`；
- 连续低 IoU 已达到 3 帧但 `rel_dist < 1.0` 时进入 `WAIT_IOU_MIN_DISTANCE`，不得建点；
- 超过 5 m或 `inside=false` 时仍可直接进入 `NEED_CHANGE`，不得被上述等待状态阻塞；
- 旧日志 stamp `1517156385.431628` 附近应优先出现
  `EDGE_TYPE=LOOP_REUSE_CURRENT ... action=ADD` 和
  `decision=LOOP_REUSE_CURRENT`，不应再出现一条约 0.056 m的顺序边；
- 旧日志 stamp `1517156423` 附近若再次匹配到空间上很近的旧节点，应优先出现
  `LOCALIZATION_SWITCH ... reuse_mode=CLOSE_GEOMETRY`；同一 FRAME 不应再创建重复节点；
- `CLOSE_GEOMETRY` 仍必须通过 registration、漂移、边位姿和机器人位姿一致性检查。任何 `POSE_INCONSISTENT` 候选都不得切换。

### 5.7 RViz 观察

请重点观察：

- 起步静止阶段是否仍快速产生节点；
- 原先首次永久分叉附近是否仍出现很短的顺序边节点簇；
- 回环发生时是否确实出现至少一条非顺序回环边；
- current vertex 是否频繁跳到明显不相邻的位置。

RViz 外观只作为辅助，最终仍以完整日志的 graph、FAISS 和回环事件链为准。

## 6. 可选性能采样对照

只有 `pcd_process_interval:=0.0` 的完整运行完成后，再考虑性能对照。可另跑：

```bash
roslaunch prism_topomap build_map_by_iou_scout_rosbag_hybrid.launch \
  trace_data_flow:=true \
  trace_every_n_processed_frames:=1 \
  pcd_process_interval:=0.1 \
  2>&1 | tee full_trace_after_fix_interval_0p1.log
```

或将参数改为 `0.3`，输出到 `full_trace_after_fix_interval_0p3.log`。这些是性能实验，不应替代默认关闭限流的正确性验证。

## 7. 请交回的文件

优先提供：

1. `full_trace_after_vertex_reuse_fix.log`；
2. 本轮实际播放到的 bag 时间（约 245 秒或约 320 秒）；
3. 如果无 interval 时出现明显积压，再提供 `0.1` 或 `0.3` 的对照日志；
4. 运行时使用的完整 rosbag 命令，以及节点启动后等待了多少秒才开始播放。

把文件放在 `/home/tom/host_catkin_ws/src/PRISM-TopoMap/` 下，然后告诉我文件名。我会继续做首帧、FAISS identity、回环预检、节点/边来源和共同时间戳的自动对照分析。
