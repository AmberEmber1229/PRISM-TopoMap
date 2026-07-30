# PRISM-TopoMap 新日志定位执行链复核

上一份基于无 `/clock` 的 `prism_data_flow_trace.log` 所得运行结论已废弃。本报告只以
项目目录中最新的 `data_flow.log` 为本次运行依据。

## 1. 原始日志身份

- 绝对路径：`/home/tom/host_catkin_ws/src/PRISM-TopoMap/data_flow.log`
- 大小：2,248,429 字节。
- 物理记录：8,990 行（8,989 个换行符，末行无换行）。
- 物理首条 logger：L1，wall time `1785290356.144806558`，
  ROS time `1517156141.482713852`。
- 物理末条 logger：L8989，wall time `1785290445.678336390`，
  ROS time `1517156231.021635375`。
- 全文非零 ROS time 范围：
  `1517156141.452424307–1517156231.021635375`。
- 成功帧点云 STAMP：
  `1517156141.444385767–1517156230.939493656`，跨度 `89.495108 s`。

日志共有 8,852 个可解析的 logger 第二时间戳，其中 8,842 个非零；ROS time 明确正常推进。
文件内容、大小和时间范围均与上一份无 `/clock` 日志不同。

## 2. 全文流式搜索统计

解析器逐行读取全文，并移除 ANSI 颜色；没有把全文读入一个字符串，也没有输出全文。
同一物理行里的多个 logger/`[FLOW]` 片段会分别处理。

| 搜索模式（区分大小写） | 行数 | 出现次数 |
|---|---:|---:|
| `Starting localization` | 178 | 178 |
| `FAISS index is empty` | 61 | 61 |
| `FAISS` | 240 | 240 |
| `gridRegistration` | 139 | 139 |
| `[INFER] gridRegistration` | 70 | 70 |
| `[INFER-PY] gridRegistration` | 69 | 69 |
| `registration score` | 464 | 464 |
| `Vertex .* registration score` | 464 | 464 |
| `LOCALIZATION_RESULT` | 358 | 358 |
| `LOC_STAMP` | 942 | 942 |
| `writeLocalizedState` | 0 | 0 |
| `LOOP CLOSURE` | 0 | 0 |
| `n_localized=` | 9 | 9 |

`writeLocalizedState` 为 0 只说明日志没有输出函数名，不能推翻 119 条带非零
`result_stamp` 的结构化 `LOCALIZATION_RESULT` 生成记录。9 条 `n_localized=` 都来自
新节点诊断且值为 0；它们只描述创建这些节点时用于重挂接的候选数量，不代表此前没有定位、
配准或非空结果消费。

## 3. 新旧格式合并和交错容错

`tools/analyze_prism_flow_log.py` 同时识别：

- `[FLOW] ... action=TIMER_READ`、`STAGE=FAISS`、`STAGE=REGISTRATION` 和
  `STAGE=LOCALIZATION_RESULT`；
- `Starting localization from stamp ...`；
- `FAISS index is empty`；
- `[INFER]`、`[INFER-PY] gridRegistration ...`；
- `Vertex X registration score: ...`。

旧式证据统一映射为 `TIMER_LOCALIZE_START`、`FAISS_EMPTY/FAISS_RESULT`、
`REGISTRATION_CALL/REGISTRATION_RESULT`。当新旧两种格式描述同一 LOC_STAMP、candidate
和相近 score 时，保留字段更完整的 `[FLOW]` 事件，旧式行只作旁证；旧式事件仅在结构化行
被并发输出截断时补位。本日志共补齐 26 个逻辑事件，避免把同一次调用重复统计两次。

解析器得到 7,318 个含 `[FLOW]` 的物理行、7,422 个 `[FLOW]` 片段。36 个片段在
stdout/stderr 插入点被截断，保留为 parse warning；完整的邻近旧式证据可补齐事件类型，
但不会补造丢失的数值字段。

归一化后的关键事件数为：

| 逻辑事件 | 数量 |
|---|---:|
| `TIMER_LOCALIZE_START` | 179 |
| `FAISS_EMPTY` | 61 |
| `FAISS_RESULT` | 118 |
| `REGISTRATION_CALL` | 466 |
| `REGISTRATION_RESULT` | 466 |
| `localization_result_generated` | 119 |
| `localization_result_consumed` | 238 |

## 4. 五层执行结论

1. **Timer 已执行。** 178 条无条件旧式 `Starting localization` 和 179 个归一化
   `TIMER_LOCALIZE_START` 直接证明回调进入了 `Localizer::localize()`。结构化与旧式计数
   的一个差值来自交错截断后的互补证据，不应把两套格式相加。
2. **FAISS 已检索。** 61 个周期明确为空索引，118 个周期返回非空候选。FRAME 162
   从 index_size=6 返回候选 5、4、3、2、0（L5399）。
3. **GridRegistration service 已调用。** 归一化后有 466 组候选调用/结果；
   全文还有 C++ `[INFER]` 70 条、Python `[INFER-PY]` 69 条和 candidate score
   464 条互相佐证。FRAME 162 的 candidate 5 得分 `0.689512` 并匹配，其余四项未匹配。
4. **LocalizedState 已写回。** 119 个 `LOCALIZATION_RESULT` 生成事件带非零
   `result_stamp`。例如 FRAME 162 在 L5411 写回
   `result_stamp=1517156201.641442 matched=1 unmatched=4`。源码没有打印函数名，所以
   `writeLocalizedState` 字符串计数为 0 是日志覆盖形式，不是未执行。
5. **主循环已消费非空结果。** 可完整解析的 238 条 CONSUME 中，155 条的 matched 或
   unmatched 非零。FRAME 163 在 L5420 消费 FRAME 162 的同一 result_stamp，
   age=`0.399102 s`，并在 L5432 得到 `decision=KEEP`。

因此，新日志与上一份无 `/clock` 日志的差异首先是**运行行为差异**（ROS time 推进并触发
Timer），其次包含**日志覆盖与解析差异**（新旧格式并存、少量交错截断，需要去重和补位）。
目标文件身份已经核验，不属于日志选择错误。

## 5. FRAME 162 → 163 的关联证据

| 阶段 | 证据 |
|---|---|
| 快照写入 | L5373，FRAME 162，STAMP=`1517156201.641442` |
| Timer 读取 | L5394，FRAME 162，LOC_STAMP=`1517156201.641442` |
| FAISS | L5399，5 个非空候选 |
| 配准 | L5402–L5410，1 matched + 4 unmatched |
| 写回 | L5411，result_stamp=`1517156201.641442` |
| 后续消费 | L5420，FRAME 163，相同 result_stamp，age=`0.399102` |
| 主循环决策 | L5429、L5432，KEEP vertex 6 |

本次全文没有观察到 `EDGE_SWITCH`、`LOCALIZATION_SWITCH`、`LOOP CLOSURE`、
`LOOP_CLOSURE`、`EDGE_TYPE=LOOP` 或 `LOOP_NEW_VERTEX`。文档和代表案例不构造这些事件。
