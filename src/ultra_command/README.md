# ultra_command — 312 任务族监控矩形障碍物检测

## 最近核对与历史摘要

09-21 按用户要求，目标路径匹配由 pudong_air/312_316_01、pudong_air/312_cargo_01、
pudong_air/312_charge_01 分别更新为 pudong_air/312_316、pudong_air/312_cargo、
pudong_air/312_charge；_01_01 初始化监控路径及判据保持。
09-21 启动方式变更：不再由 `launch/control.launch` include 捆绑启动，改为 HMI"自定义指令"
卡片独立启停（组3、参与一键启动，健康=master 节点表计数）；`start_l4.sh` 救急路径单独补了
`roslaunch ultra_command ultra_command.launch` 终端行。
09-21 因 CAN 速度不准确，节点改订阅 `/navigation_msg`，取 `gpsSpeed`（m/s）作为
初始化起步判据。原阈值、任务闩锁、感知/矩形处理保持；无需修改消息定义和 CSV。
导航未知时仍保持 safe=1；本模块沿用原速度缓存，不新增独立车速超时机制。
节点 C++11 桩编译及 29 项导航接线/初始化回归通过，见
[速度来源专项](../pnc/tests/navigation_speed/README.md)。需重编 ultra_command 并重启，本轮未部署车辆。

09-17监控矩形文件恢复；09-11加入初始化路径模式，旧task_plan latch配套修复曾被回滚。
09-19文档核对确认：当前converter已处理真实空帧、planning已有safe消费者、部分任务已引用
`_01_01`，但task_plan仍非latch。以下说明按现树修正，完整历史见工程workflow归档。

## 注意事项与容易疏忽的点

- 本模块的固定区域初始化监控，与planning手动切自动的车身周边观察是两套逻辑；
  前者阈值严格>0.5m/s，后者连续两帧D车速>0.2m/s，不能混用退出条件。
- /perception含历史目标，真实空帧到发布列表清空有时序窗口；断流不能伪造空心跳。
- 仅用中心点判矩形，没有车身/障碍物外形展开；排除区内目标在上游已被滤掉。
- 启动未知任务会保持safe=1，不能只看现场无障碍就清零；还要核对任务、矩形和消息时效。
- pathList精确字符串匹配、首个命中决定模式；相同路径的新task_id也会重置初始化状态。
- 导航速度替换未增加或改变该既有独立模块的挡位规则。

## 功能（需求原文映射）

| task_plan 当前执行路径（/task_plan_msg.pathList 精确匹配） | 监测矩形 | rosparam 输出 |
|---|---|---|
| `pudong_air/312_316`、`pudong_air/312_cargo` | `pudong_air/left1` + `pudong_air/left2` | 障碍物落入任一矩形 → `/ultra/status/safe = 1`；连续三帧均无 → `0` |
| `pudong_air/312_charge` | `pudong_air/right` | 障碍物落入 → `1`；连续三帧均无 → `0` |
| `pudong_air/312_316_01_01`、`pudong_air/312_cargo_01_01`（初始化监控） | 同上 left1+left2 | **起步前**（尚未出现车速>0.5m/s）：区内有障碍 → `1`，无 → `0`；**首次车速>0.5m/s 后**：无论区内是否有障碍恒 `0` |
| `pudong_air/312_charge_01_01`（初始化监控） | `pudong_air/right` | 同上，以 right 为监测区 |
| 其它路径 / 待命 / 云端停止指令后 | 不监测 | 写 `0`（清除上一任务状态） |

- 矩形数据 = `pnc/path/pudong_air/{left1,left2,right}.csv`（4 顶点闭合多边形，
  与任务路径 csv 同坐标系：定位 xAxis/yAxis 地图系）。
  经全局参数 `path_dir`（`robot_path_plan.launch` 设置）定位，每周期热读+失败自动重试。
- 障碍物来源 = `/perception`（`robot::perception`），取 `objs[i].x/y` 中心点做
  点在多边形内判定（射线法，与 perception_convert `IsPointInExclusion` 同款算法；
  不做障碍物尺寸/外形展开）。
- 节点 10Hz 主循环；每周期 ZOH 重发 `rosparam::set`（参数被外部改写可自愈）。

## 结构

```
src/ultra_command/
├── CMakeLists.txt              # catkin, 依赖 roscpp/std_msgs/geometry_msgs + robot(消息)
├── package.xml
├── launch/ultra_command.launch # 须与 pnc 栈同 master(path_dir 依赖)
└── src/
    ├── ultra_command_node.cpp     # ROS 薄壳: 订阅 task/perception/navigation/cloud command
    └── ultra_command_comply.{h,cpp} # 纯逻辑(零 ROS 依赖): 模式匹配/矩形加载/判内/老化/起步闩锁
```

## 实现决策（规格未明说处的取值，改动前先读）

1. **非目标任务期间写 0**：任务结束、切到其它任务或云端停止后主动清除上一
   任务状态，避免 rosparam 永久保留旧值。
2. **感知静默 1.0s 按故障态（写 1）**：perception_convert 已改为零 markers
   也发布 `objs` 为空的 `/perception`，并填写 header stamp；因此空场景能正常清 0，
   无新帧只表示感知链不可用。未收到或超过 1.0s 均保守写 1，不再与 sensorstate
   的 3.0s 故障判定形成错报 0 的保护空窗。
3. **中心点判定**：只判障碍物中心点是否在矩形内，不按 dx/dy 尺寸扩展
   （与 perception_convert 排除区过滤同口径）。
4. **精确字符串匹配**：`312_316_02/03`、`312_cargo_02/03`、`312_charge_02/03/04`
   等同族变体**不**触发监控；pathList 多条目时**按列表顺序首个命中元素决定模式**
   （与族别无关，混合列表谁在前谁胜出；现役 yaml 均为单路径，无行为差异）。
   注意 task8（charge_01 任务）末块 action 路径为 `sx040901`，该块期间模式回 NONE。
5. **启动先写 `safe=1`**：任务状态、感知和矩形尚未确认时保持故障态。
   2026-09-19核对：当前task_plan发布器没有latch，不能保证晚启动/重启立即恢复任务。
   收到明确的非目标任务/停止后才按对应规则写0；旧“latch已修复”描述不适用于现树。
6. **矩形加载失败按故障态处理**（写 1）+ 醒目打印；加载成功后仍需新感知帧确认。
   目录 glob 扫描是 pnc 感知侧已知风险（workflow 风险 9），本包**不用目录扫描**，
   三个文件名写死；csv 坏行（含 nan/inf 字面量行，stod 不抛异常）按 isfinite
   校验跳过，顶点不足 3 判加载失败。
7. **云端停止指令即时退监控**：本节点直接订阅 `/cloud/msg/command_msg` 清空模式，
   不依赖当前非latch的task_plan发布器。暂停/继续不清模式（任务仍在执行）。
8. **普通监控危险即时、解除防抖**：任一帧检出障碍立即写 1；恢复 0 必须连续
   三帧 `/perception` 确认无障碍，10Hz 输入下约 0.3s。主循环重复读取同一帧不计数。
9. **初始化监控（_01_01 路径，2026-09-11 新增）的起步判据**：
   - 车速源 = `/navigation_msg.gpsSpeed`（float32 m/s）。
     未收到有效车速时按故障态写 1；最新车速会缓存，导航与 task 回调先后顺序不影响
     起步状态。保持原缓存规则，不做独立车速老化；CAN 心跳不能证明导航速度新鲜；
   - 阈值为**严格大于 0.5 m/s**（恰 0.5 不触发）；起步闩锁**单向**，此后降速/
     停车不恢复起步前监测；闩锁按**完整命中路径 + task_id** 标识一次任务激活。
     两个 INIT_LEFT 路径互切、或同路径但 task_id 改变时均重新初始化；相同任务消息重复
     送达不会误复位；
   - 起步前收到有效感知帧后直接反映当前区域状态：有障碍 1、无障碍 0，不使用
     普通模式的三帧解除；矩形未加载、感知超时或车速未知仍按故障态写 1；
   - 起步后**无条件 0，不依赖感知/矩形状态**。节点重启时闩锁随进程复位；任务
     与车速两者均到达后不受回调先后顺序影响，车速尚未到达期间保持故障态 1。

## 当前注意边界与历史核对

1. **/perception 是感知排除区过滤后的流**：落在 pnc/config 任一 csv 排除多边形
   内的障碍物对本节点**不可见**。09-11记录中的两个排除多边形与三矩形当时无重叠；当前配置需重新核对，
   **监控矩形不得与日后新增的排除区 csv 重叠**，否则 safe 恒 0 静默致盲。
2. **进程被 SIGKILL / 主机失电时 rosparam 无原子故障通知**：正常退出先写 1，
   launch 以 1s 延迟自动拉起；但不可捕获的退出到重启成功之间仍可能短暂保留末值。
3. **矩形与任务自身作业几何重叠（待用户确认语义）**：实测三矩形紧贴任务停止点
   正前方 1.2~1.5m 起的走廊，任务路径自身穿越矩形（312_316_01 有 106 个路径点在
   left1 内、cargo_01 131 点、charge_01 33 点在 right）——恢复行驶必穿过矩形。
   若挂靠目标/常驻设备落在矩形内被激光雷达上报，作业段 safe 恒 1，"无障碍→0"
   分支不可达。这是矩形数据的语义问题非代码缺陷：若意图是监控侧向通道需重画
   矩形；若"确认前方就绪"正是目的则符合预期。建议回放一次 312 任务 rosbag 核实。
4. **消费方已在仓内**：planning兼容前向链和新前向适配器读取 `/ultra/status/safe`；
   新链独立读缓存参数，兼容链有早退条件，R仍按原规则。不能沿用旧“全仓零读者”结论，
   也不能据有一个读者就认定所有挡位/路径均强制停车。
5. **初始化路径已被部分任务引用**：09-19核对task5/task18等YAML已有`_01_01`，
   task5当前task_sum为4。是否进入初始化模式还取决于实际下发的任务与完整pathList；
   不要再把“路径存在但YAML尚未引用”当当前结论。

## 使用方法

- 在ROS1工程根先构建robot消息依赖，再执行 `catkin_make --pkg ultra_command -j4`、
  `source devel/setup.bash`。干净工作空间不能跳过robot；当前rebuild_all脚本的已知问题见根README。
- 启动（09-21 起）：HMI 页面"规划控制"组"自定义指令"卡片独立启停，参与一键启动
  （组3 与 pnc 并行，path_dir 缺失时本节点每周期重试加载，先后无碍）；
  `start_l4.sh` 救急路径有独立终端行。注意本节点仍须与 pnc 栈同 master
  （path_dir 全局参数依赖）。停止卡片=节点退出前写 `safe=1`，行驶中停车属预期。
  只启动 pnc 而未启动本节点时，`/ultra/status/safe` 参数不存在，planning 侧缺省 0
  不停车——矩形监控静默缺失且无任何告警，成套启动请用一键启动（或救急脚本）。
  单独调试仍用 `roslaunch ultra_command ultra_command.launch`。
- 编后核对：
  - 待命时 `rosparam get /ultra/status/safe` 为 0；任务状态未知时为 1；
  - 启动日志出现 `监控矩形加载成功 ... (5/5/5 顶点)`；
  - 下发 312_charge 任务后日志出现 `监控模式切换: 0 -> 2`；
  - 在 right 矩形内放置障碍物（或 rostopic pub 模拟）→ 参数立即翻 1，移走并连续
    三帧无障碍（约 0.3s）→ 回 0；断开 `/perception` 超过 1s → 回 1；
  - 云端下发停止指令 → 日志出现 `收到云端停止指令, 退出监控模式`，参数清 0；
  - 初始化监控（_01_01 任务）：起步前区内有障碍 → 1；`rostopic pub` 一条
    `gpsSpeed>0.5` 的 `/navigation_msg`（或实车起步）→ 日志出现 `车速首次超过
    0.5m/s, 初始化监控退出`，此后即使区内有障碍参数也保持 0；
  - 切换 cargo_01_01 ↔ 312_316_01_01 或更换 task_id → 必须重新进入起步判定；
  - 部署任务前核对实际下发的任务块是否使用 `_01_01` 路径及车端配置是否一致；
  - **与消费方联调**：确认其读取时机/周期/类型期望（见已知边界 4）。
- 本机（无 ROS）验证方法：消息桩从真实 .msg 机械生成 + `g++ -fsyntax-only` 过
  node；comply 为零 ROS 依赖纯逻辑，直接编可执行跑单测（见 workflow.md 日志）。
