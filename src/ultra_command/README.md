# ultra_command — 312 任务族监控矩形障碍物检测

## 功能（需求原文映射）

| task_plan 当前执行路径（/task_plan_msg.pathList 精确匹配） | 监测矩形 | rosparam 输出 |
|---|---|---|
| `pudong_air/312_316_01`、`pudong_air/312_cargo_01` | `pudong_air/left1` + `pudong_air/left2` | 障碍物落入任一矩形 → `/ultra/status/safe = 1`；两矩形均无 → `0` |
| `pudong_air/312_charge_01` | `pudong_air/right` | 障碍物落入 → `1`；无 → `0` |
| 其它路径 / 待命 / 云端停止指令后 | 不监测 | 不写参数（保持最后值） |

- 矩形数据 = `pnc/path/pudong_air/{left1,left2,right}.csv`（4 顶点闭合多边形，
  与任务路径 csv 同坐标系：定位 xAxis/yAxis 地图系）。
  经全局参数 `path_dir`（`robot_path_plan.launch` 设置）定位，每周期热读+失败自动重试。
- 障碍物来源 = `/perception`（`robot::perception`），取 `objs[i].x/y` 中心点做
  点在多边形内判定（射线法，与 perception_convert `IsPointInExclusion` 同款算法；
  不做障碍物尺寸/外形展开）。
- 节点 10Hz 主循环；监控激活期间每周期 ZOH 重发 `rosparam::set`（参数被外部改写可自愈）。

## 结构

```
src/ultra_command/
├── CMakeLists.txt              # catkin, 依赖 roscpp/std_msgs/geometry_msgs + robot(消息)
├── package.xml
├── launch/ultra_command.launch # 须与 pnc 栈同 master(path_dir 依赖)
└── src/
    ├── ultra_command_node.cpp     # ROS 薄壳: 订阅 /task_plan_msg /perception /cloud/msg/command_msg
    └── ultra_command_comply.{h,cpp} # 纯逻辑(零 ROS 依赖): 模式匹配/矩形加载/判内/老化
```

## 实现决策（规格未明说处的取值，改动前先读）

1. **非目标任务期间不写参数**：规格只定义三个任务激活时的行为。任务结束/切到
   其它任务时保持最后值不动（消费者按其自身语义处理）。若需要"退出即清 0"，
   改 node 主循环 `ZONE_MODE_NONE` 分支补一次 `set(...,0)` 即可。
2. **感知静默 1.0s 按"无障碍"（写 0）**：perception_convert 对**零 markers 帧**
   早退静默不发消息；markers 非空但无存活 CUBE（全为标签、或全被感知排除区
   过滤掉）时会发布 **objs 为空**的 `/perception` 帧——本节点对空列表同样按
   无障碍判 0，两种上游行为输出一致。若只盯最后一帧，`safe=1` 会永久粘滞，
   故无新帧超过 1.0s（对齐 canbus 指令老化窗）判 0。
   **已知取舍（对抗校验 09-10 确认）**：感知链死亡且有障碍在区时 1.0s 后错报 0，
   而 sensorstate 兜底链（canbus EmergencyStop / control 缓停）要 dlidar>3.0s
   才生效，存在 ~2s 无保护窗；若消费方语义要求故障时宁可不报 0，可把
   `kObstacleStaleSec` 对齐 3.0s（代价：清空场景回 0 延迟 1→3s）——待用户定夺。
3. **中心点判定**：只判障碍物中心点是否在矩形内，不按 dx/dy 尺寸扩展
   （与 perception_convert 排除区过滤同口径）。
4. **精确字符串匹配**：`312_316_02/03`、`312_cargo_02/03`、`312_charge_02/03/04`
   等同族变体**不**触发监控；pathList 多条目时**按列表顺序首个命中元素决定模式**
   （与族别无关，混合列表谁在前谁胜出；现役 yaml 均为单路径，无行为差异）。
   注意 task8（charge_01 任务）末块 action 路径为 `sx040901`，该块期间模式回 NONE。
5. **启动即写 `safe=0` 一次**：保证消费者任何时刻都能读到该参数
   （对齐 task_plan 启动写 `/cloud/suggestspeed`）。
6. **矩形加载失败按"无障碍"处理**（写 0）+ 醒目打印；加载成功前不阻断。
   目录 glob 扫描是 pnc 感知侧已知风险（workflow 风险 9），本包**不用目录扫描**，
   三个文件名写死；csv 坏行（含 nan/inf 字面量行，stod 不抛异常）按 isfinite
   校验跳过，顶点不足 3 判加载失败。
7. **云端停止指令即时退监控**：订阅 `/cloud/msg/command_msg`（0-停止 1-暂停
   2-运行），`commandState==0` 时清空监控模式。原因（对抗校验 09-10 确认）：
   task_plan 停车分支只 `clearTaskPool()`，终态空 pathList 的 `/task_plan_msg`
   因发布门控**不会发出**，不订阅则模式闩死在 LEFT/RIGHT、对已取消任务持续
   写参数直到下一个云任务。暂停/继续不清模式（任务仍在执行）。

## 已知边界（对抗校验 09-10 轮结论）

1. **晚启动/中途重启失明（分钟级）**：`/task_plan_msg` 非 latch 事件驱动，节点
   晚于任务下发启动或目标块执行中重启时，模式停留 NONE，**当前块剩余执行时长内
   零监控**（312_charge_01 单块 ~1.2km/2m/s ≈ 10 分钟，316/cargo 块 3~5 分钟），
   且启动写 0 会覆盖重启前可能正确的 1。节点会对"有 /perception 无
   /task_plan_msg"打印一次告警。**硬性要求：随 pnc 栈一起启动、任务中禁止重启**。
   根治需 pnc 侧一行改动（`/task_plan_msg` advertise 加 latch，已验证 path_plan
   对重复消息幂等）——动 pnc 待用户批准。
2. **启动接入未做（实车生效的唯一阻塞项）**：start_l4.sh / launch/control.launch /
   HMI 组件表均无本节点（三者属 workflow 保护清单）。候选：control.launch 追加
   include（推荐，随 pnc 同 master 启动同时消解边界 1）/ HMI 组件 / start_l4 终端。
3. **/perception 是感知排除区过滤后的流**：落在 pnc/config 任一 csv 排除多边形
   内的障碍物对本节点**不可见**。当前两个排除多边形与三矩形无重叠（已逐坐标
   核对），但**监控矩形不得与日后新增的排除区 csv 重叠**，否则 safe 恒 0 静默致盲。
4. **输出无去抖/滞回**：障碍物中心在矩形边界徘徊（感知噪声量级）时 safe 最高以
   ~5Hz 方波抖动；检出/闪断交替时 ~0.6Hz 翻转（实测）。规格字面为即时反映，未加
   滤波；若消费方对边沿敏感（告警/继电器），需加 N 帧一致窗或滞回（约 0.3s 置 1）。
5. **进程死亡 = 参数冻结末值**：节点崩溃后 `/ultra/status/safe` 停在最后写入值，
   HMI process_manager / health_monitor 均不覆盖本节点（零 advertise 对
   health_monitor 不可见）。随启动接入一并决策；人工恢复 = `rosparam set
   /ultra/status/safe 0` 后重启节点。
6. **矩形与任务自身作业几何重叠（待用户确认语义）**：实测三矩形紧贴任务停止点
   正前方 1.2~1.5m 起的走廊，任务路径自身穿越矩形（312_316_01 有 106 个路径点在
   left1 内、cargo_01 131 点、charge_01 33 点在 right）——恢复行驶必穿过矩形。
   若挂靠目标/常驻设备落在矩形内被激光雷达上报，作业段 safe 恒 1，"无障碍→0"
   分支不可达。这是矩形数据的语义问题非代码缺陷：若意图是监控侧向通道需重画
   矩形；若"确认前方就绪"正是目的则符合预期。建议回放一次 312 任务 rosbag 核实。
7. **消费方契约未验证**：`/ultra/status/safe` 目前全仓零读者（消费方在仓外车载
   环境）。若消费方仅启动读一次则 ZOH 无意义需事件化通道；若周期轮询需确认其
   周期容忍"待命保持最后值"语义。部署时与消费方联调确认。
8. `/perception` 头 stamp 恒 0（perception_convert 不填），老化计时用回调接收
   时刻 `ros::Time::now()`。

## 构建与部署

- 车载：同步 `src/ultra_command/` 整目录 → `cd 工程根 && JOBS=4 ./rebuild_all.sh`
  （全量段自动编入；**不可**在干净 build/devel 上单独 `catkin_make --pkg
  ultra_command`，依赖 robot 包消息先生成）。
- 启动：`roslaunch ultra_command ultra_command.launch`（或并入现有启动编排，
  见已知边界 2；start_l4.sh/HMI 属运维保护清单，本包未改动，接入方式需用户拍板）。
- 编后核对：
  - `rosparam get /ultra/status/safe` 存在且为 0/1；
  - 启动日志出现 `监控矩形加载成功 ... (5/5/5 顶点)`；
  - 下发 312_charge_01 任务后日志出现 `监控模式切换: 0 -> 2`；
  - 在 right 矩形内放置障碍物（或 rostopic pub 模拟）→ 参数翻 1，移走（>1s）→ 回 0；
  - 云端下发停止指令 → 日志出现 `收到云端停止指令, 退出监控模式`；
  - **与消费方联调**：确认其读取时机/周期/类型期望（见已知边界 7）。
- 本机（无 ROS）验证方法：消息桩从真实 .msg 机械生成 + `g++ -fsyntax-only` 过
  node；comply 为零 ROS 依赖纯逻辑，直接编可执行跑单测（见 workflow.md 日志）。
