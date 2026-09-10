# health_monitor 设计文档（2026-09-08，对抗校验修订版）

> 状态：待用户评审（已经 17 代理五视角审查+逐条对抗核实，9 项确认全部吸收）
> 前提结论：ROS1 `/statistics` 通道对本项目不可用（roscpp 未实现；rospy 为订阅端语义），
> 已源码验证（ros_comm noetic-devel）。故采用 ShapeShifter 主动订阅方案。

## 1. 背景与目标

车载 Jetson 上新增 **C++ 健康监测节点**：自动发现 ROS 图中全部话题，测量每个话题的
**频率 / 新鲜度 / 流量 / 消息类型**，1Hz 汇总发布到 `/diagnostics`。

- **零侵入**：不改任何现有代码/配置/启动文件（rebuild_all.sh 全量段自动编入 src/ 新包，
  无需改它；workflow.md 工作记录条目不属业务侵入）
- **无判据**：节点只出量测与状态分类，健康判定留给消费方（用户后续自建）
- **旁路观测**：不接任何控制链（安全兜底仍是 canbus T1 带内急停）
- 入口：手动 `roslaunch health_monitor health_monitor.launch`

## 2. 范围

**做**：新包 `src/health_monitor`（发现/订阅/采样/量测/发布）。

**不做**：阈值判据表、HMI/monitor 集成、start_l4.sh/HMI 卡片入口、任何 .msg 新增
（出参用 ROS 自带 `diagnostic_msgs/DiagnosticArray`，不动 can_msg 家族避开 md5 连坐）。

## 3. 包结构（node/逻辑类分层，抄 canbus 习惯）

```
src/health_monitor/
├── CMakeLists.txt              # catkin；依赖 roscpp + topic_tools + diagnostic_msgs
├── package.xml
├── README.md                   # 包说明 + 实车验证清单（树内惯例）
├── include/hz_meter.h          # HzMeter 纯头文件实现（inline，uint64 纳秒，ROS 无关）
├── include/state_classify.h    # 状态分类纯函数（可单测）
├── include/health_monitor.h    # HealthMonitor 类声明
├── src/health_monitor_node.cpp # 仅 ROS 接线：main / 参数 / spinOnce 主循环
├── src/health_monitor.cpp      # 发现、订阅管理、重载荷轮转、评估发布
├── launch/health_monitor.launch
├── test/hz_meter_test.cpp      # 纯数学单测：不依赖 ROS，g++ 直接编跑
└── test/state_classify_test.cpp
```

## 4. 机制规格

### 4.1 话题自动发现（以发布者表为唯一存在口径）

- `discover_period_s`（默认 5s）Timer 调 **`ros::master::getTopics(V_TopicInfo&)`**
  （roscpp 提供，返回 name + datatype，**只含已 advertise 的话题**，即发布者表）
- ⚠ 不使用 getSystemState 联合口径（roscpp 亦无此封装）：本节点自己的订阅会把
  被监控话题钉死在 master 的订阅表里，使"消失移除"永不触发、干净退出的发布者
  永久 stale 陈尸。pubs 数量不采集（getTopics 不提供），`hardware_id` 改载 datatype
- **存在性判定**：话题出现在 getTopics 表 → 登记；连续 2 个发现周期缺席 → 退订并
  移除记录（2 周期宽限防 master 抖动）。纯订阅者话题天然不进表、不订阅
- 排除表 `exclude_topics`，默认 `[/rosout, /rosout_agg, /diagnostics, /clock]`；
  **/diagnostics 永久排除**（防自环），不得由参数放开
- `max_topics`（默认 128，实车规模约 50 留余量）上限保护，超限 printf 一次并跳过新增

### 4.2 订阅分类（发现期定型，无首发全量订阅）

- 发现时 datatype 已知，直接分类，**不经过"先常订再看首条消息类型"**：
  - datatype ∈ `heavy_types`（默认 `{sensor_msgs/PointCloud2, sensor_msgs/Image,
    sensor_msgs/CompressedImage}`，参数可配）→ 进 4.3 重载荷轮转池，**从不首发常订**
    （本车重载荷 = 5 路雷达点云 + 2 路 1080p20 JPEG 相机流）
  - 其余 → 常订，queue=3（queue=1 在 ≥spinOnce 频率的话题上会系统性丢帧致 hz 偏低；
    3 兼顾积压重放失真上限）+ `ros::TransportHints().tcpNoDelay()`
- 订阅形式：`topic_tools::ShapeShifter` 泛型回调（roscpp 通用 boost::function 重载），
  跳过反序列化
- 回调（单线程，免锁）：`uint64 ns = ros::Time::now().toNSec()` 与
  `uint32 bytes = msg->size()` 成对入 HzMeter

### 4.3 重载荷占空比采样（轮流制）

- 轮转池按**话题名字典序**，每 `sample_period_s`（默认 10s）轮一个，开
  `sample_on_s`（默认 2s）订阅窗后退订。本车池 7 话题（5 雷达+2 相机）→ 每话题
  约 70s 一窗；单次突发只有一路
- 窗口开启时 `reset()` HzMeter（防旧窗数据混入）；窗关闭后保留末窗快照供报告
  （state=sampling，hz 为末窗实测，stale_age 持续增长但不触发 stale）
- 窗口结束仍 0 帧 → state=no_data（见 4.5）
- 话题在窗内被移除（4.1 两周期缺席）→ 立即关窗并清理待关窗动作；轮转指针按
  **话题名**（非下标）解析：增删后取字典序 ≥ 当前名的首个现存话题，到尾回绕

### 4.4 HzMeter（include/hz_meter.h，纯头文件 inline，无任何 ROS 头）

- 存储：`std::deque<std::pair<uint64_t, uint32_t>>`——(纳秒时间戳, 字节数) 成对，
  保证截断与字节同步回退
- `tick(uint64_t ns, uint32_t bytes)`：成对入队，双上限截断：窗口 >100 条 或
  跨度 >64s 时 pop_front（时间与字节一起弹）
- `hz()`：n≥2 且跨度>0 时 `(n-1)/跨度`，否则 0
- `stale_age_s(now)` / `window_msgs()` / `traffic_bps()`（**窗口字节和÷窗口跨度**，
  与截断后窗口严格一致）
- `reset()`：清空全部窗口状态

### 4.5 评估与发布（1Hz）

`report_period_s`（默认 1.0s）Timer 组 `diagnostic_msgs/DiagnosticArray` 发
`/diagnostics`（queue=10）。每话题一个 `DiagnosticStatus`，**评估顺序：
sampling/event 判定优先于 stale**：

| state | 触发 | level |
|---|---|---|
| ok | 窗口≥2 帧且 stale_age < 生效 stale 阈值 | OK(0) |
| stale | 非 event/sampling 话题，曾有数据但 stale_age ≥ 生效阈值 | WARN(1) |
| event | 话题在 `event_topics` 表内（事件驱动，合法静默） | OK(0)，stale_age 照常上报 |
| sampling | 重载荷轮转窗口外 | OK(0)，hz 为末窗快照 |
| no_data | 发现后超过 `no_data_grace_s`（默认 30s）仍 0 帧 | WARN(1) |
| new | 发现后不足 2 个发现周期 | WARN(1)，信息不足 |

- **event_topics**（参数，默认含本图实证的 8 个事件/条件驱动话题）：
  `/task_plan_msg`（仅云端下发/块切换时发布，task_plan_node.cpp:399-407）、
  `/cloud/task/task_status`、`/v2nCommandFeedback`（仅指令到达时，task_plan_core）、
  `/cloud/task/task_info`、`/cloud/task/remote_signal`、`/cloud/msg/command_msg`
  （fms_agent 三个 MQTT 脚本话题，云端不下发即静默可达小时级）、
  `/cam0/status`、`/cam7/status`（latch+状态翻转才发，camera_publish.cpp:87）。
  这些话题不适用 stale 判级——工程先例：hmi_config 对 /cloud 组也不用频率判健康
- **stale 阈值适用范围**：稳态频率 ≥ 1/stale_s 的话题；低频/事件话题用 event_topics
  或后续消费方自行判读（stale_age 原始值始终在 KeyValue 里）
- `name`=话题名；`hardware_id`=datatype；`message`="state=... hz=%.1f ..."
- KeyValue：`hz / window_msgs / stale_age_s / traffic_bps / datatype / state`
- 末尾一条汇总：`name=/health_monitor/summary`，含话题总数、常订数、轮转池大小
  与当前轮转位置
- 异常（getTopics 失败等）printf 一次，不刷屏

### 4.6 线程模型

单线程（canbus main 风格）：main 里 `ros::Rate(100)` + `spinOnce()` 主循环；
发现/采样/报告全用 Timer。所有回调同线程，无锁。

## 5. 参数（绝对键，全部默认值 + ros::param::get，失败 printf 一次并沿用默认）

| 键 | 默认 | 说明 |
|---|---|---|
| /health_monitor/discover_period_s | 5.0 | 发现轮询周期 |
| /health_monitor/report_period_s | 1.0 | /diagnostics 发布周期 |
| /health_monitor/stale_s | 5.0 | 新鲜度阈值（超过判 stale；适用稳态话题） |
| /health_monitor/no_data_grace_s | 30.0 | 发现后 0 帧宽限，超过判 no_data |
| /health_monitor/sample_period_s | 10.0 | 重载荷轮转步进 |
| /health_monitor/sample_on_s | 2.0 | 重载荷采样窗宽 |
| /health_monitor/exclude_topics | 见 4.1 | 排除表（/diagnostics 恒在） |
| /health_monitor/heavy_types | 见 4.2 | 重载荷类型集合 |
| /health_monitor/event_topics | 见 4.5 | 事件驱动话题表（不做 stale 判级） |
| /health_monitor/max_topics | 128 | 订阅数上限 |

## 6. 开销预算

常订话题为全部小/中消息（重载荷 7 话题已入轮转池，常订集合均 <1KB，
/perception 100Hz、/navigation_msg 50Hz 等）≈ 1-2% 单核（跳过反序列化）；
轮转采样摊销 ≈0.1%；发现轮询为本地 XMLRPC，忽略不计。总预算 <3% 单核、内存 <10MB。

## 7. 代码风格

按 `src/canbus/VIBE_CODING_GUIDE.md`：C++11；4 空格；类/函数左括号换行、
if/for 同行；类与方法 PascalCase（`HealthMonitor`/`HzMeter`）；ROS 回调
`XxxCallBack`；pub/sub snake_case 带 `_pub/_sub`；话题绝对名；所有成员显式
初始化；tcpNoDelay；注释中文业务+英文短标签；不复制历史缺陷（未初始化、
重复 include 等）。

## 8. 验证（按诚实分级）

**本机（无 ROS1，jazzy 不可编 ROS1 代码）**：
1. 桩编译：roscpp（含 ros::master::getTopics 桩）/ topic_tools / ShapeShifter /
   diagnostic_msgs 头桩 + xmlrpcpp / std_msgs 依赖，包两个 TU
   （health_monitor_node.cpp / health_monitor.cpp）编译链接过（沿用 workflow 08-30
   桩编译纪律）
2. `test/hz_meter_test.cpp` 纯数学单测（独立 g++ 编译运行）：已知间隔→hz、
   双上限窗口截断、stale 计算、reset 隔离、**截断后 traffic_bps 不含被弹出字节**
   （成对回退正确性）

**实车清单（记入包 README，上车时执行）**：
- `rostopic echo /diagnostics` 与 `rostopic hz <话题>` 抽查对照（**≥60s 长窗**，
  容差 ±10%，消除 queue/瞬时抖动）
- 两种发布者死亡路径都要测：`kill <pid>`（SIGTERM 干净注销 → 条目在下个发现
  周期从报告移除）；`kill -9 <pid>`（注册残留 → 无数据 → no_data/stale）
- 事件话题（如 /task_plan_msg）待命时 state=event 且 level=OK，stale_age 持续增长
- 重载荷话题轮转日志与 sampling 态展示（5 雷达+2 相机，~70s 轮一圈）
- `top` 对比起停前后 CPU 差（预期 <3% 单核）
- 话题数对照：与**发布者表口径**一致（注意 `rostopic list` 为 pubs+subs 并集，
  会多出纯订阅话题，属预期差异）

## 9. 交付物

- 上节包结构全部文件
- workflow.md 日志条目（工作记录，非业务文件）

## 10. 已知取舍

- 采样制下重载荷话题频率只有窗口内快照，**测不出间歇性掉频**——雷达断流已有
  带内兜底（pnc dlidar>3s→sensorstate→急停 + CenterPoint 健康检查），取舍接受
- `new` 态（刚发现）与 `no_data`（长期 0 帧）会以 WARN 出现：图刚起时有一片
  new WARN 属瞬态；advertise-从不发布的话题会长期 no_data WARN（提示死通道，
  消费方可按 state 过滤）
- event_topics 默认表是按本图实证列出的：图演化后新的事件话题需人工补表
  （stale_age 原始值始终可查，误标后果限于 level 位）
- queue=3 在极端积压后突发重放会把周期测歪（上限 3 帧），对 hz 的影响方向为
  略高估瞬时频率；稳态量测不受影响
- 话题从发布者表消失即从报告移除（2 发现周期宽限，不留尸检记录）；发布者
  SIGKILL 残留注册的场景由 no_data/stale 覆盖
