# PRISM-TopoMap 修改计划（仅供审阅）

> 实施状态（2026-07-31）：已在当前混合架构工作空间实施首帧 descriptor 与 registration 传输恢复、无效 descriptor 建点保护、FAISS 显式身份映射、默认关闭 0.3 秒 PCD 限流、回环触发端点一致性保护。第二轮完整日志确认回环节点由 24 个降至 8 个，但又暴露 localization/self switch、沿边切换和未校验顺序边会破坏 5 m 相对位姿基准；现已禁止同节点 localization switch，取消 `need_change` 对 IoU 的绕过，并为 localization、EDGE_SWITCH、顺序边和回环 current 端点统一增加 Scout 距离/比例/朝向门槛。第三轮日志进一步确认所有 20 次 IoU 建点都发生在第一次跌破 0.3 的单帧，节点 25 由近距离回环强制创建，节点 27–30 则由反向视角下的旧节点复用失败形成重复链；现已增加三帧 IoU 确认、IoU-only 最小建点距离、近距离回环直接复用当前节点，以及近距离 `CLOSE_GEOMETRY` 旧节点复用。Scout 还要求首个 GT 到达后才处理点云。纯 Python Timer 修复仍保留为外部原始 Python 工作空间待办。

本文件保留计划行为、修改边界和验证方法，不包含代码、伪代码或 diff。上方已实施项目来自用户后续授权；未标记实施的项目仍需根据新日志再决定。依赖顺序是：先找到并修复首帧 descriptor service 调用失败的直接原因，再补强索引身份约束和回环节点创建逻辑，最后统一实验条件并做数值等价性比较。

## P0：先定位并修复首帧 descriptor service 调用失败

- **已验证问题：** 混合架构首帧 `/prism/get_descriptor` 调用在客户端失败；服务端没有收到这一请求。第二个点云请求才成为服务端 `SERVICE-1` 并成功返回 descriptor。
- **证据：** `full_trace.log` 显示服务在 wall time `1785313911.839` 已被发现，首点云到达和失败发生在 `1785314046.450～1785314046.453`，两者相隔约 135 秒；首个调用只耗时约 1.2 ms，且没有 stamp `1517156141.444386` 的 `DESCRIPTOR_PY` handler 记录。下一次 stamp `1517156141.844400` 才出现服务端 `SERVICE-1 result=OK`。`src/inference_client.cpp:21-25` 使用 persistent service client，`93-107` 在调用失败后重建该 persistent client。
- **根因：** 长空闲后的 persistent TCPROS 客户端连接失效已得到强证据支持。修复前服务就绪约 135 秒后，首个 descriptor 调用在约 1.2 ms 内失败且未进入 Python handler；改用 non-persistent client 后，即使服务空闲约 469 秒，FRAME 1 仍以 `attempts=1` 进入 `DESCRIPTOR_PY` 并成功返回 256 维 descriptor。重试未被使用，因此恢复来自连接生命周期修复，而不是重试掩盖。相同短测还发现 persistent registration client 的首个调用在约 0.97 ms 内发生传输失败并在重建后恢复，故 registration client 也采用相同的 non-persistent 与一次有界传输重试策略。
- **涉及文件与函数：** `src/inference_client.cpp` 的构造、`waitForServices()`、`getDescriptor()` 和重连逻辑；`include/prism_topomap/inference_client.h`；`scripts/inference_service_node.py` 的 service 注册与 `handle_get_descriptor()`；混合 launch 的节点启动顺序；必要的 ROS service 连接诊断。
- **目标行为：** 推理服务完成模型加载并真正可调用后，主节点才进入点云处理；首次真实 descriptor 请求与后续请求同样可靠。即使连接瞬态失效，同一首帧也应在有界重连后得到明确成功或明确失败，不应直接进入建图。
- **计划修改范围：** 先增加最小诊断并做隔离复现：比较 persistent 与 non-persistent 首次调用、不同空闲时间后的首次调用、服务节点 PID/URI 是否变化、请求是否进入 handler；根据复现结果修复连接生命周期或启动握手。随后增加“首个真实请求的有界重连/重试”和服务端可调用性检查，但不能用无限重试掩盖服务崩溃。
- **不应改变的行为：** descriptor 模型、点云内容、量化参数、正常请求的推理结果，以及后续稳定调用的性能路径。
- **风险：** 单纯增加 sleep 可能偶然掩盖连接问题；盲目重试可能重复昂贵推理；关闭 persistent 模式可能增加每帧连接开销；启动健康检查不能用与真实请求完全不同的空请求得出假阳性。
- **单元级验证：** 使用可控 mock service 覆盖首次连接断开、服务重启、长时间空闲、首次 call 返回 false、handler 返回 success=false 和重连成功；验证错误类型可区分且重试次数有上限。
- **日志级验证：** 客户端每次失败应记录连接状态、attempt、服务 URI/可用性和“请求是否得到响应”；服务端记录单调 request ID。首帧必须能判断是未到 handler、handler 异常还是模型返回失败。
- **rosbag 回归验证：** 在播放前分别等待 0、10、60、180 秒，多次运行同一 bag；首帧 descriptor 均应成功，且服务端 request 1 与 C++ FRAME 1 的 stamp 一致。另做服务重启故障注入，验证有界恢复。
- **完成判据：** 首帧失败原因能够稳定复现并被归类；修复后多种等待时长下首帧成功率达到验收要求，日志不再出现无服务端请求记录的客户端首调失败。
- **与其他计划项的依赖：** 这是首要任务。确认并修复直接原因后，再实施下一项索引身份防御；不能用 identity 映射替代对首帧 service 故障本身的修复。

## P0：在 service 修复后补强 descriptor 与顶点索引的一致性防线

- **已验证问题：** 首帧 descriptor 失败导致 graph vertex 0 未进入 FAISS，后续 row 被直接当作 vertex ID，发生永久错位。
- **证据：** `full_trace.log` FRAME 1、stamp `1517156141.444386` 为 `descriptor_dim=0, faiss 0→0`；FRAME 82、LOC_STAMP `1517156172.042562` 查询节点 1 自身 descriptor 却返回 `candidate=[0:0.000000]`。代码 `src/topological_graph.cpp:65-83,281-307,372-408` 和 `src/localizer.cpp:179-223` 没有 row→vertex 转换。
- **根因：** 图和索引默认假定“每个顶点都有合法 descriptor，因此 FAISS row 永远等于 vertex ID”。任何一次 descriptor 缺失都会破坏这个隐式前提。
- **涉及文件与函数：** `include/prism_topomap/topological_graph.h`；`src/topological_graph.cpp` 的 `addVertex()`、`addToIndex()`、`searchIndex()`、加载重建；`src/localizer.cpp` 的候选解释；`src/topo_slam_model.cpp` 的节点创建前置条件。
- **目标行为：** 正常情况下所有新节点都带合法 descriptor；异常情况下也不能让后续 FAISS 候选错指 graph vertex。每个 FAISS 结果必须能验证其真实 vertex 身份。
- **计划修改范围：** 首选策略是 descriptor 无效时不提交建图节点，等待同一帧重试或下一有效帧；同时添加显式 FAISS row→vertex identity 作为防御和旧图兼容机制；统一维度、有限值、运行时添加、加载重建和检索校验。
- **不应改变的行为：** descriptor 正常时的 vertex 编号、FAISS 距离、`top_k` 排序、合法图保存格式和正常顺序扩图。
- **风险：** 延迟首节点会改变初始化时间；旧图可能包含空 descriptor；显式映射若未覆盖保存/加载会引入新的错位。不能因为有映射就容许正常运行频繁产生空 descriptor 节点。
- **单元级验证：** 覆盖首节点空、首次重试成功、中间节点空、错误维度、NaN/Inf、加载重建和多次重建；验证每个自描述符查询返回其真实 vertex ID。
- **日志级验证：** 正常运行必须满足 graph vertex 创建均有合法 descriptor；若兼容旧图中的不可索引节点，日志必须显式区分 indexed/unindexed，并验证 mapping 唯一。
- **rosbag 回归验证：** 在上一计划项修复后重跑 bag，首节点应同时使 graph 与 FAISS 从 0 变为 1；再故障注入 descriptor 失败，确认不会造成后续 ID 错位。
- **完成判据：** 正常 bag 不产生空 descriptor 节点；任何单次 descriptor 故障都不能破坏 FAISS 与 vertex identity；保存/加载前后的候选 ID 一致。
- **与其他计划项的依赖：** 排在首帧 service 根因修复之后，是回环和 localization switch 修复的前置防线。

## P0：让回环触发、节点创建和实际提交的边指向同一闭环

- **已验证问题：** 第一轮预检已经避免“所有候选边都失败仍然建点”，但它只要求任意候选边成功，没有验证成功的边是否就是触发回环判断的那一对节点。结果可能由节点 `u/v` 触发回环，却由第三个无关候选通过预检，随后仍为 `u/v` 记作已确认回环并创建节点。
- **证据：** `full_trace_after_fix.log` 共创建 51 个节点，其中 24 个来自 `LOOP_NEW_VERTEX`；节点 26～40 在约 13.5 秒内连续产生 15 个回环节点，多次恰好相隔 0.5 秒。24 次已确认事件中，仅 8 次能由顺序边和已接受回环边共同覆盖两个触发端点。FRAME 2171 的触发对是 `25/20`，端点 20 被拒、25 只是顺序节点，却由无关节点 23 的候选边通过预检；FRAME 2211 的触发对 `27/22` 均被拒，却由节点 31/21 的候选边使事务继续。
- **根因：** `findLoopClosure()` 只返回布尔值，丢失了触发节点 `u/v`；`addNewVertex()` 又对完整定位候选集合做“任意一条成功”判断。检测依据、建点依据和最终边提交对象不是同一个闭环约束。
- **涉及文件与函数：** `src/topo_slam_model.cpp` 的 `checkPathCondition()`、`findLoopClosure()`、`addNewVertex()`、`update()`；对应头文件和图边查询/添加接口。
- **目标行为：** 回环检测必须返回明确的触发节点 `u/v`。创建中的新节点必须能同时连接这两个端点：如果某端点是当前顺序节点，由必然提交的顺序边覆盖；否则必须有一条指向该端点、且通过距离和朝向检查的回环边。无关候选不能替触发端点通过事务。
- **已实施范围：** `findLoopClosure()` 现在返回带 `u/v`、旧路径和距离信息的候选对象；`addNewVertex()` 在提交图修改前只预检触发端点并计算 `u_covered/v_covered`，任一端点未覆盖即记录 `TRIGGER_ENDPOINTS_NOT_COVERED`，节点数和边数保持不变。普通 5 m 扩图仍走独立的 `NEW_VERTEX` 规则。Scout 配置另将回环边的全局直线距离绝对误差限制为 1.5 m、距离比例限制为 2.0（0.5 m 以下不做比例判断）、朝向误差限制为 0.5 rad；这些门槛未在其他数据集配置中默认启用。
- **不应改变的行为：** 正常距离阈值扩图、合法顺序边、确实有有效新回环边的节点创建，以及无向图语义。
- **风险：** 预检过严可能漏掉真实回环；预检时必须使用与最终加边完全相同的位姿和阈值；定位快照在检查期间不能发生语义变化。
- **单元级验证：** 覆盖候选全拒绝、只有自环、只有已有边、一条有效、多条混合、重复候选和阈值边界；候选全失败时断言节点数、边数和 current 均不变。
- **日志级验证：** 每个 `LOOP_NEW_VERTEX` 前必须出现同 FRAME 的 `LOOP_TRIGGER_PRECHECK action=ACCEPT u_covered=true v_covered=true`；`LOOP_TRIGGER_PRECHECK action=REJECT` 后不得在同 FRAME 确认回环或以 `LOOP_NEW_VERTEX` 建点。该帧若随后独立满足普通距离扩图条件，仍允许以 `NEW_VERTEX` 建点。最终确认事件必须携带同一 `trigger_u/trigger_v`。
- **rosbag 回归验证：** 重点复核旧运行 FRAME 2171、2211 对应的路线区段，确认无关候选不再促成建点；统计 `LOOP_CANDIDATE_DETECTED`、触发端点预检接受/拒绝、`LOOP_NEW_VERTEX` 和节点时间间隔，检查节点 26～40 类似的 0.5 秒密集簇是否消失。
- **完成判据：** 不再存在“由一对节点触发回环，却由其他候选边促成建点”的控制路径；所有确认回环都能从实际提交的边重建出检测所声称的闭环。
- **与其他计划项的依赖：** 必须在 descriptor/FAISS identity 正确后验证，否则预检使用的仍可能是错误候选。

## P0：修复纯 Python 异步定位 Timer（仅在原始 Python 工作空间实施）

- **已验证问题：** registration 返回 `transform=None` 时，trace 格式化执行 `list(transform)` 抛异常，`rospy.Timer` 线程永久退出。
- **证据：** `pure_python_flow_trace.log` FRAME 73、stamp `1517156149.343489` 的调用栈；`scripts/localization.py:194-235` 与 `scripts/prism_topomap_node.py:614-651`。
- **根因：** 可失败推理结果与 trace 序列化契约不一致；Timer callback 没有异常隔离和健康状态。
- **涉及文件与函数：** 原始 Python 工作空间中的 `scripts/localization.py::Localizer.localize()` 和 `scripts/prism_topomap_node.py` timer callback。本 PRISM-TopoMap 混合架构工作空间只保留问题记录和回归接口，不在这里实施 Python 源码修改。
- **目标行为：** `None` transform 被安全记录为失败结果；单次失败不会终止 Timer；纯 Python 基准能持续运行全局定位。
- **计划修改范围：** 将本项迁移到用户指定的原始 Python 工作空间：统一 registration 失败契约、修正 trace 序列化、设置 callback 异常边界和健康日志。本工作空间不修改这些 Python 文件。
- **不应改变的行为：** 有效 transform 的阈值判断、matched/unmatched 分类、快照锁语义和正常 timer 周期。
- **风险：** 两个工作空间版本可能漂移；过宽异常捕获可能隐藏真实缺陷；修复后必须记录原始 Python 仓库 commit 才能作为基准。
- **单元级验证：** 在原始 Python 工作空间注入 `None`、异常、低分和高分 transform，确认 Timer 连续调用。
- **日志级验证：** 单次失败后仍出现下一次 `TIMER_BEGIN`/FAISS/完成或明确失败结果，不得有未处理线程异常。
- **rosbag 回归验证：** 在原始 Python 工作空间运行超过原异常 stamp 30 秒，验证 LOC_STAMP 持续推进；产出日志供本工作空间对照分析。
- **完成判据：** 原始 Python 工作空间无法复现 `NoneType is not iterable`，且完整 bag 中 Timer 持续健康。
- **与其他计划项的依赖：** 本仓库混合架构修复不等待本项；但最终 Python/C++ 公平等价性比较必须等待外部工作空间完成并提供新日志。

## P0：统一 localization timer、global pose 与 odom 同步语义

- **已验证问题：** 同一 `localization_frequency=2.0` 在 Python 表示 2 秒周期，在 C++ 表示 2 Hz；Python 使用点云时刻插值和 GT，混合运行回退到约 47 ms 旧的 odom。
- **证据：** `scripts/prism_topomap_node.py:542-548,699-769`；`src/prism_topomap_node.cpp:173-180,333-418`；两份日志的 timer 和 SYNC 记录。
- **根因：** 配置单位和 pose 同步契约未跨语言统一；相同 YAML 文本被不同解释，GT 缺失又采取静默降级。
- **涉及文件与函数：** `config/scout_rosbag.yaml` 的 timer/pose 参数；混合 node 的 timer 创建和 pose buffer 同步；对应 launch。纯 Python 侧的必要修改仍在原始 Python 工作空间实施。
- **目标行为：** 同一参数代表相同 timer 周期；对照运行使用相同 global pose 来源和点云时刻 odom；若必需 GT 缺失，应明确失败而不是静默改变实验条件。
- **计划修改范围：** 明确 timer 参数的单位并统一实际周期；规定 GT 必需或允许降级的模式；统一 pose 插值/采样和容差；启动日志输出最终生效值。PCD 0.3 秒限流单独由下一计划项处理。
- **当前实施进度：** Scout 混合配置已启用 `require_gt_pose`；GT buffer 为空时点云不会进入算法，也不会再用 ODOM 创建首节点。timer 单位和原始 Python 工作空间的同步语义仍保留为后续跨工作空间任务。
- **不应改变的行为：** 用户明确选择的运行模式、合法 topic 配置和与算法无关的发布内容。
- **风险：** timer cadence 改变会影响负载；严格 GT 策略可能中止现有 launch；插值要正确处理角度环绕。
- **单元级验证：** 验证 timer 实际周期、GT 缺失策略、点云时刻线性/角度插值和超时边界。
- **日志级验证：** 双架构输出相同 `timer_period`、`global_source` 和 odom delta 分布；参数名和单位一致。
- **rosbag 回归验证：** 固定启动顺序和 pose source，比较两套定位快照时间与点云时刻 pose。
- **完成判据：** timer 和 pose 条件不再构成未声明的实验差异。
- **与其他计划项的依赖：** 实际等价性回归在首帧 service、索引和回环修复之后执行；Python 侧依赖外部工作空间。

## P1：取消用混合架构独有的 0.3 秒限流掩盖节点爆增

- **已验证问题：** `pcd_process_interval=0.3` 只在 C++ 混合路径执行，纯 Python 没有相同门槛。它最初用于缓解启动阶段节点爆增，但会改变处理点云集合、odom 累积步长、栅格更新和节点阈值时机，不能作为拓扑正确性的长期修复。
- **证据：** `src/prism_topomap_node.cpp:611-623` 记录并执行 `SKIPPED_INTERVAL`；`scripts/prism_topomap_node.py:828-892` 无对应 interval gate，日志明确为 `interval_throttle_present=false`。本次 Python 处理 2,683/3,422 帧，混合约处理 1,030/3,808 帧。
- **根因：** 限流是在首帧索引错位和过早创建回环节点尚未定位时加入的症状性保护。它降低错误决策的触发频率，却没有修复错误候选或错误节点提交。
- **涉及文件与函数：** `src/prism_topomap_node.cpp` 的点云队列和 interval gate；`include/prism_topomap/prism_topomap_node.h`；`config/scout_rosbag.yaml`；混合 launch 和采样诊断。原始 Python 工作空间只作为参考行为，不在本仓库修改。
- **目标行为：** 在“与原始 Python 等价”的运行配置中，混合架构不再默认使用独有的 0.3 秒算法限流；节点稳定性来自索引和回环逻辑正确，而不是少处理点云。如生产环境需要降采样，应作为独立、显式的性能模式。
- **计划修改范围：** 先在前述 P0 修复完成后，以 0.3 秒、0.1 秒和禁用 interval 三组运行确认节点不再爆增；等价性配置优先禁用混合端独有限流。若确需性能降采样，保留可选参数但默认关闭，明确它会改变算法输入，并将其与“正确性修复”分开验收。
- **不应改变的行为：** 点云消息年龄保护、队列上限、异常数据过滤和正常回调安全；生产用户仍可显式选择性能采样模式。
- **风险：** 取消限流会提高 descriptor service、LocalGrid 和 callback 负载，可能暴露队列积压或实时性问题；不能在 P0 修复前直接取消并用节点爆增结果评判方案。
- **单元级验证：** 覆盖 interval 禁用、0.1、0.3、时间戳相等/倒退和 rosbag 时间跳变；验证禁用时没有 interval 原因丢帧。
- **日志级验证：** 启动时记录 sampling mode 和有效 interval；分别统计接收、处理、因消息年龄丢弃、因性能采样丢弃的帧，避免把不同丢帧原因混在一起。
- **rosbag 回归验证：** 在完成首帧 service、identity 和回环修复后，先禁用 interval 重跑完整 bag；验证启动阶段不再爆增，再与 0.1/0.3 秒结果比较负载、节点/边和第一分叉。
- **完成判据：** 禁用 0.3 秒限流时不再复现由错误回环造成的启动爆增；等价性配置与原始 Python 采样语义一致；性能模式的拓扑影响被单独记录。
- **与其他计划项的依赖：** 实施和验收依赖前三个混合架构 P0 修复。它应在最终 LocalGrid、IoU 和拓扑等价性比较之前完成。

## P1：统一“尚无定位结果”的 freshness 状态机

- **已验证问题：** C++ 将 `localized_stamp<=0` 判为 fresh，但不建立 time reference；Python 在已有 pose history 时把 stamp 0 判为 stale。
- **证据：** `full_trace.log` FRAME 2、stamp `1517156141.844400` 为 `result_stamp=0 age=-1 fresh=true has_time_reference=false`；Python首帧同状态为 `fresh=false`。代码 `src/topo_slam_model.cpp:1186-1198` 与 `scripts/prism_topomap.py:727-752`。
- **根因：** freshness 同时承担“结果存在”“时间可对齐”“未过期”三种语义，零值 sentinel 的解释不同。
- **涉及文件与函数：** 两套 TopoSLAM update/consume localization 逻辑；localizer result 数据结构；stale fallback 与相对位姿历史管理。
- **目标行为：** 明确区分 NO_RESULT、VALID_FRESH、VALID_STALE 和 INVALID；只有带有效 timestamp 的结果才能进行 motion compensation 或回环候选消费。
- **计划修改范围：** 统一状态定义、初始化值、age 计算、time-reference 建立和 fallback 入口；日志直接输出状态枚举及判定原因。
- **不应改变的行为：** 有效新鲜结果的候选使用、真正 stale 结果的降级策略、正常相对位姿累积。
- **风险：** 启动阶段节点创建时机可能改变；旧代码依赖空候选但 fresh 的隐式行为。
- **单元级验证：** 覆盖 stamp 0/None、空 history、非空 history、等于最早 history、略早/略晚、matched 为空和非空的组合。
- **日志级验证：** 两套架构对相同状态输出相同 freshness 枚举；不再出现 `age=-1 fresh=true` 但没有 time reference 的矛盾状态。
- **rosbag 回归验证：** 比较启动前几秒的回环入口、降级和首节点行为；确认不新增启动节点簇。
- **完成判据：** 零时间戳在两边控制流一致且有明确语义；所有消费结果均能说明是否存在有效时间参考。
- **与其他计划项的依赖：** 依赖 P0 实验语义定义；应在最终拓扑等价性回归前完成。

## P1：为 localization reattach 与节点切换补齐几何一致性保护

- **已验证问题：** `need_to_change_vcur_` 为 true 时可绕过 IoU 接受候选；`full_trace_after_loop_consistency_fix.log` 的 26 次 localization switch 中有 15 次 `from==to`。FRAME 394 的 `1→1` 把已累计到 5.0758 m 的相对位姿重置为 0.954 m，直接造成后续节点 `1→2` 的全局距离 7.82 m、边长却只有 3.40 m。40 次 EDGE_SWITCH 中 9 次切换后的相对位姿与 GT 明显冲突；最终 50 条边中有 12 条不满足 Scout 几何门槛。
- **证据：** `full_trace.log` stamp `1517156328.134530`；`src/topo_slam_model.cpp:650-704`；Python继承逻辑 `scripts/prism_topomap.py:498-505`。
- **根因：** “必须离开当前节点”被等同于“任意定位候选都可接受”，而 global geometry 只记录、不作为同级拒绝条件。
- **涉及文件与函数：** 两套 `reattachByLocalization()` / `reattach_by_localization()`；localization edge 添加与 current vertex 更新；相关 drift/IoU 条件。
- **目标行为：** 必须切换 current 时仍只能选择通过 descriptor、registration、IoU/几何一致性和时间一致性的候选；没有合格候选时进入显式降级而非强行附着。
- **已实施范围：** 当前节点自身候选直接以 `SAME_AS_CURRENT` 拒绝；`need_change` 不再替代 IoU；localization edge、切换后的机器人位姿和 EDGE_SWITCH 均必须通过统一距离、比例、朝向及 5 m 归属范围检查。首次 drift limit 不再使用 `current_stamp-0`。新节点提交前预检顺序边并限制 Scout 顺序边最大 5.5 m；失败时不修改图。回环 current 端点只有在该顺序边预检通过后才算覆盖。
- **不应改变的行为：** 合法沿边切换、真实重定位、drift 随时间放宽的既有设计意图。
- **风险：** 全局 pose 本身不可靠时会误拒绝；门槛必须与 GT/odom source 策略联动；过度收紧会导致 current 长时间失配。
- **单元级验证：** 覆盖 IoU 高/低、need-change true/false、全局比例一致/不一致、自环、既有边、多候选排序。
- **日志级验证：** 每个 LOCALIZATION_SWITCH 都有完整的候选验证结果；被标为 ADD 的边实际存在且非自环；几何不一致候选不能改变 current。
- **rosbag 回归验证：** 重点复核 stamp `1517156328.134530` 及后续 current 轨迹，统计错误附加边、切换振荡和恢复时间。
- **完成判据：** 不再出现严重全局几何不一致却因 need-change 被接受的候选；事件日志与实际图增量一致。
- **与其他计划项的依赖：** 必须在 FAISS identity 修复后验证；参数标定依赖统一 global pose 来源。

## P1：对齐 LocalGrid 的数值语义并建立逐像素基准

- **已验证问题：** 同点云统计接近，但 Python warp 默认线性插值、C++ 使用 nearest；`np.round` 与 `std::round` 的半值规则也不同。现有日志没有逐像素数据。
- **证据：** `scripts/local_grid.py:100-108,179-200`；`src/local_grid.cpp:163-202,297-302`；共同时间戳 occupancy unknown 差的中位数为 24、最大 415，早期 IoU 差 P95 为 0.013060。
- **根因：** 两套实现只对齐了公式和参数，没有固定插值、边界、rounding、dtype 与写回规则的跨语言数值契约。
- **涉及文件与函数：** 两套 LocalGrid 的 point-to-cell、warp、occupancy update、IoU、transform matrix；测试夹具与调试导出接口。
- **目标行为：** 在相同点、相同初始 grid、相同 pose delta 下，各层逐像素结果满足预先定义的精确或容差标准；差异超限可定位到具体阶段。
- **计划修改范围：** 先冻结参考语义，再统一 interpolation、rounding、border、shape/axis、dtype 和阈值；增加只在测试/诊断启用的逐层导出与 checksum。
- **不应改变的行为：** 栅格坐标定义、地图范围、障碍高度语义，以及已确认正确的变换方向；未经基准验证不调整算法阈值。
- **风险：** 插值方式变化会改变历史 IoU/registration 分数；逐像素完全相同可能受库版本和浮点实现限制，应区分离散层精确与连续层容差。
- **单元级验证：** 使用半栅格边界点、正负坐标、旋转、平移、边界裁剪和空 grid，逐层比较 occupancy/density/height/IoU。
- **日志级验证：** 对指定共同 timestamp 输出各层 hash、非零数、最大差与差异像素数；禁止仅用全局计数宣称等价。
- **rosbag 回归验证：** 在统一 cadence/odom 后导出固定采样帧，逐像素比较并检查 IoU 决策边界。
- **完成判据：** 离散 occupancy/height 达到约定精确标准，连续 density 达到数值容差；任何剩余差异都有明确来源。
- **与其他计划项的依赖：** 依赖 P0 实验条件统一；不阻塞索引/回环正确性修复，但阻塞最终“数值等价”结论。

## P2：强化图、索引、Timer 与事件日志的不变量可观测性

- **已验证问题：** 当前日志可观察到 graph/FAISS 数量，却没有 row→vertex identity；logger 插断导致部分 summary 和 edge 类型字段损坏；自环事件声明与实际图行为不一致。
- **证据：** 混合日志 46 个 raw `LOOP_DETECTED` 中部分 summary 无法结构化恢复；10 个 localization edge 事件含 3 个底层拒绝自环；最终计数只能依赖图不变量交叉复核。
- **根因：** trace 更偏向过程文本，缺少原子事件 ID、提交结果和跨组件一致性字段；多个 logger 写同一输出造成物理行交织。
- **涉及文件与函数：** 两套 flow trace helper；TopologicalGraph 添加/查询；Localizer Timer；TopoSLAMModel 的 decision/commit；inference service 日志。
- **目标行为：** 每个帧、定位快照、回环事务和边提交都有稳定 event ID；日志能直接回答候选来源、映射、验证、实际图增量与 Timer 健康。
- **计划修改范围：** 定义单记录字段契约和结果枚举；记录 graph vertices、indexed/unindexed、FAISS mapping checksum、edge before/after、实际 addEdge 返回值、timer callback sequence；减少跨 logger 的同一事件拆分。
- **不应改变的行为：** 算法决策、默认非 trace 性能和现有 ROS topic。
- **风险：** 高密度日志会影响时序；敏感路径需支持采样和关闭；日志 schema 变更会影响现有解析工具。
- **单元级验证：** schema 必填字段、event ID 唯一性、失败路径、并发写入和 addEdge 返回值一致性测试。
- **日志级验证：** 单次运行可无歧义重建所有节点/边增量；`final = initial + committed deltas`；损坏记录率为 0 或有明确丢弃计数。
- **rosbag 回归验证：** 在 trace 开/关下比较算法输出，确认观测不改变拓扑；自动生成不变量审计摘要。
- **完成判据：** 无需用源码补造字段即可复核 graph、FAISS、loop 和 switch 全链条；日志声明与实际容器状态一致。
- **与其他计划项的依赖：** schema 应吸收前述 P0/P1 的新状态，但实施可分阶段；建议在大规模回归前完成核心不变量字段。

## P2：建立双架构 rosbag 决策级回归门禁

- **已验证问题：** 当前两份运行在 timer、处理 timestamp、pose source、Timer 健康和 FAISS identity 上均不一致，无法作为持续等价性基线。
- **证据：** 验证报告第 3、4、5、9、10 节；727 个共同 summary timestamp 只能覆盖不同采样集合的交集。
- **根因：** 缺少固定启动、故障注入、同时间戳数据导出和决策级验收标准。
- **涉及文件与函数：** 测试/launch 资源、日志解析工具、固定 bag 运行脚本与 CI 配置；不要求改变算法生产路径。
- **目标行为：** 对同一 bag，两套架构处理相同 timestamp，并按阶段比较 point、pose、descriptor、grid、候选、registration、decision、graph delta；已知容差和允许差异明确版本化。
- **计划修改范围：** 建立短程确定性 fixture 与完整 bag 两级测试；加入首帧 service 失败、单次 registration None、GT 缺失和错误 descriptor 维度故障场景；产出机器可读摘要和人工时间线。
- **不应改变的行为：** 生产 mapping 参数与部署方式；测试不能依赖人工查看 RViz 判定通过。
- **风险：** GPU/FAISS/ROS 调度可能引入非确定性；完整 bag 成本较高；需要把算法容差与基础设施波动分开。
- **单元级验证：** 解析器、timestamp matching、无向边去重、事件因果关联和容差计算均有固定 fixture。
- **日志级验证：** 自动检查 Timer 连续性、descriptor/index identity、零空回环节点、自环声明、图增量守恒和第一分叉。
- **rosbag 回归验证：** 短 bag 每次提交运行，完整 bag 定期运行；同时保存共同 timestamp 数、第一数值差异和第一控制流差异。
- **完成判据：** P0/P1 修复后，两套运行可在相同实验条件下给出可重复的阶段级比较；任何拓扑分叉都能自动定位到首个 ROS stamp。
- **与其他计划项的依赖：** 最终门禁依赖全部 P0；LocalGrid 像素门禁依赖相应 P1；观测 schema 完成后可提高诊断完整度。

## 建议实施与复核顺序

1. 首先隔离复现首帧 descriptor service 调用失败，确认是 persistent 连接、服务重启还是其他传输时序，并修复直接原因。
2. 在首帧正常成功的基础上，补上 descriptor 有效性检查和 FAISS row→vertex identity 防线，确保任意异常都不会再次造成 ID 错位。
3. 将回环流程改为“保存触发节点对→只预检这两个端点→确认两个端点均被覆盖→创建节点并提交同一组边”，消除由无关候选促成的 LOOP_NEW_VERTEX。
4. 完成前三步后，禁用混合架构独有的 0.3 秒限流重跑 bag；只有在正确性稳定后，才单独评估可选性能采样模式。
5. 统一 localization timer 单位、global pose 和 odom 同步；随后处理 freshness 与 localization reattach。
6. 纯 Python Timer 修复保留为独立待办，只在原始 Python 工作空间实施；完成后提供新日志作为健康基准。
7. 最后进行 LocalGrid 逐像素对齐、日志 schema 强化和完整 rosbag 回归门禁。

混合架构的前三步可以不等待原始 Python 工作空间而先行修复。只有首帧 service、identity、回环创建、0.3 秒限流、timer/pose 条件以及外部 Python Timer 都完成后，才能对两套架构的 descriptor、registration、回环和最终拓扑做公平等价性评价；在此之前不建议依据最终 RViz 外观调整阈值。
