# health_monitor —— 话题健康监测节点

自动发现 ROS 图中全部话题（ros::master::getTopics 发布者表口径），量测每个话题的
频率/新鲜度/流量/消息类型，1Hz 发布 `diagnostic_msgs/DiagnosticArray` 到
`/diagnostics`。旁路观测节点，不接任何控制链；健康判定留给消费方。

## 最近记录

09-08新增旁路健康观测；现行机制和参数如下，不改变车辆控制。旧开发过程见工程workflow归档。

## 注意事项与容易疏忽的点

- ROS master发布者表不同于rostopic list的发布/订阅并集，数量差异未必是漏检。
- 事件话题可合法静默，不能按周期话题强判故障；重载荷话题轮转采样不代表持续订阅。
- stale/no_data要区分曾收到后断流与从未收到；干净注销与kill -9注册残留表现不同。
- `/diagnostics`只是观测输出，不接安全制动链；预期CPU值必须在Orin实测。

## 机制

- 小/中话题：topic_tools::ShapeShifter 常订（queue=3，跳过反序列化，回调只记
  时间戳与字节数）
- 重载荷（sensor_msgs/{PointCloud2,Image,CompressedImage}，本车=5 雷达+2 相机）：
  按字典序轮流开 2s 采样窗，从不常订
- 状态分类（详见代码 state_classify.h）：ok / stale / event / sampling / no_data / new；
  event 话题（事件驱动，合法静默）不判 stale
- 单线程 Timer，回调免锁

## 参数（绝对键，均有内置默认值）

| 键 | 默认 | 说明 |
|---|---|---|
| /health_monitor/discover_period_s | 5.0 | 发现轮询周期 |
| /health_monitor/report_period_s | 1.0 | /diagnostics 发布周期 |
| /health_monitor/stale_s | 5.0 | 新鲜度阈值（稳态话题） |
| /health_monitor/no_data_grace_s | 30.0 | 发现后 0 帧宽限 |
| /health_monitor/sample_period_s | 10.0 | 重载荷轮转步进 |
| /health_monitor/sample_on_s | 2.0 | 采样窗宽 |
| /health_monitor/exclude_topics | [/rosout,/rosout_agg,/diagnostics,/clock] | 排除表 |
| /health_monitor/heavy_types | [PointCloud2,Image,CompressedImage] | 重载荷类型 |
| /health_monitor/event_topics | 见代码（8 个本图实证话题） | 事件驱动话题 |
| /health_monitor/max_topics | 128 | 订阅数上限 |

## 使用方法

在ROS1工程根、消息依赖已满足时执行：

```bash
catkin_make --pkg health_monitor -j4
source devel/setup.bash
roslaunch health_monitor health_monitor.launch
```

另一终端source相同工作空间后，用 `rostopic echo /diagnostics` 和 `rostopic hz <实际话题>`
对照，参数见上表。HMI或已有launch运行此节点时不要再重复启动。

## 本机回归（无 ROS 依赖）

catkin_make --pkg health_monitor 后直接运行（或 g++ 编译 test/*.cpp）：
- hz_meter_test / state_classify_test：ALL PASS

## 实车验证清单

- rostopic echo /diagnostics 与 rostopic hz <话题> 抽查对照（≥60s 长窗，±10%）
- kill <pid>（SIGTERM 干净注销）→ 条目 2 个发现周期内从报告移除；
  kill -9（注册残留）→ state=no_data/stale
- 事件话题（如 /task_plan_msg）待命时 state=event 且 level=OK，stale_age_s 持续增长（证明有末次消息且在老化；恒 -1.00 说明开机至今从未发布）
- 重载荷轮转日志（5 雷达+2 相机约 70s 一圈）与 sampling 态
- top 对比起停前后 CPU 差（预期 <3% 单核）
- 话题数对照：与发布者表口径一致（rostopic list 为并集，多出纯订阅话题属预期）
