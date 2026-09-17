# robot_control 模块说明

该目录实现 `control_node`：20 Hz 接收规划、定位、CAN、任务和交通灯状态，
生成 `/control_msg`，并发布用于诊断的 `/acc`。这里记录文件职责和主流程，便于后续按
业务边界定位修改点。代码格式和改动纪律仍以 `src/pnc/README.md` 为准。

## 文件职责

| 文件 | 职责 |
|---|---|
| `control_node.cpp` | ROS 薄入口：订阅、发布、20 Hz 调度 |
| `control_comply.h` | 控制编排接口、输入快照和跨周期状态 |
| `control_comply.cpp` | 路径预处理、围栏、主控制流程和横向控制入口 |
| `longitudinal_acc_control.inc` | 可选 ACC/AEB 计算，由主实现包含进同一翻译单元 |
| `longitudinal_speed_control.*` | 期望加速度、基础油门/刹车分配 |
| `lateral_reverse_control.*` | R 挡预瞄点与转弯半径控制 |
| `stanley_controller/*` | D 挡 Stanley 与纯追踪前馈融合 |

## 运行数据流

```text
/plan_path_msg ──路径过滤/样条/曲率──┐
/navigation_msg ──车辆位姿与速度─────┤
/can_msg ──挡位/反馈转角──────────────┤
/path_plan_status、/task_plan_msg ────┼─> VehicleControl() ─> /control_msg
/camera/tl_status ──交通灯去抖────────┤
/perception ──可选 ACC/AEB────────────┘

fence.csv ─> FenceAlarm() ─> FenceWarning ─> 下一周期安全覆盖
```

`control_node` 当前每周期的调用顺序是：

1. `VehicleControl()` 计算本周期控制量；
2. `FenceAlarm()` 更新电子围栏状态；
3. `PublishMessage()` 发布控制量；
4. 发布 `acc_msg` 诊断信息。

因此 `FenceWarning` 对控制量的影响天然晚一个控制周期。此处只记录既有行为，整理代码时
不要为了“看起来更合理”而调整调用顺序。

## VehicleControl 主流程

主函数内各段按覆盖优先级排列：

1. 路径安全停车、任务完成或 N 挡停车；
2. 两处场景化交通灯停车；
3. 空路径停车；
4. 计算相对路径位姿、期望速度和期望加速度；
5. 可选 ACC/AEB 诊断计算与基础纵向控制；
6. R 挡几何控制或 D 挡 Stanley 控制；
7. 曲率限速、传感器缓停和 GNSS 急停；
8. D 挡速度给定上升斜坡,再按原标定比例换算成油门；
9. 电子围栏与横向偏差停车覆盖；
10. 输出限幅及 R 挡油门覆盖。

后面的步骤可能覆盖前面的控制字段。修改某一业务条件前，应同时检查该条件之后所有对
`mControlData` 的赋值，不能只看局部代码。

## 起步速度给定斜坡（2026-09-16）

用户确认底层电机为速度环，油门与最终车速的比例已经实车标定。
保留 D 挡 `speed_cmd × 18`、R 挡 `mSpeed × 10`（上限 45%），以及原来的
0.5m/s 跟随间隙。只平滑 D 挡速度给定的上升，下降与停车指令立即生效。

`/robot/control/launch_speed_slope` 为速度给定上升斜率，默认 **0.3m/s²**，
按现有 20Hz 控制周期每次增加 0.015m/s。持续目标不低于 1m/s、底层速度环能跟随时，
速度给定约 3.35s 升至 1m/s；这里描述给定变化率，实车瞬态还需现场验证。
参数每周期读取；非正数、非有限值回退至 0.3。可运行时设置：

```bash
rosparam set /robot/control/launch_speed_slope 0.3
```

速度给定使用每个控制对象独立的 double 状态，最后才量化为 uint8 油门。
较小 slope 也能逐步累积，不会因每周期不足 1% 而卡在零。
原 5% 最小油门保留为目标约束，先换算到目标速度再走斜坡，起步期间不直接跳到 5%。
N 挡、任务停车、路径安全停车、空路径、交通灯停车、GNSS 急停、围栏与横向偏差停车
均清除斜坡历史；恢复后从零起步。R 挡保持原输出并清除 D 挡历史。
首次在 D 挡接入且车辆已运动时，从当前反馈车速衔接。

此前的 `throttle_slope`、`launch_speed_gap`、`launch_min_throttle` 三参数方案已回滚，
这些旧参数不再读取，也不更改原有标定。验证入口：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 src/pnc/tests/control/verify.py --output /tmp/control_speed_verify
```

测试驱动真实控制源码与真实消息生成的 ROS 桩，覆盖斜坡累积、停车/换挡恢复、
低速保底和稳定输出；可加 `--baseline-control <旧 control_comply.cpp/.h 所在目录>`
进行新旧标定输出对照。本轮 C++11 控制节点编译/链接、57 项检查通过，
与回滚基准的五条既有编译告警一致。本机验证不能替代 ROS1/车载编译和路测。

## 常用业务修改入口

- 改路径点接收、过滤或样条：`ControlComply::SetPathPlanData`
- 改停车优先级和安全覆盖：`ControlComply::VehicleControl`
- 改围栏判定：`ControlComply::FenceAlarm`
- 改 D 挡横向算法：`LatController::calculate`
- 改 R 挡横向算法：`GeometricConstrol::LateralControlTrack1`
- 改基础油门/刹车分配：`SpeedControl::SpeedTrack`
- 改 ACC/AEB：`ControlComply::LongitudinalFeedforwardControl`
- 改最终消息发送方式：`ControlComply::PublishMessage`

## 修改时必须保留的既有语义

- ROS 话题使用绝对名，include 顺序不调整。
- 不重命名既有标识符，包括 `GeometricConstrol`、`setTaskPlanData` 等历史名称。
- `VehicleControl()` 内逻辑顺序是控制优先级的一部分。
- `PublishMessage()` 在油门和刹车同时非零时会拆成两条消息发布。
- D/R 挡使用不同的横向控制器，不能通过表面相似直接合并。
- D 挡 ×18、R 挡 ×10 为用户确认的速度环标定比例,起步调参只调整速度给定斜率。
- 多处算法含跨周期 `static` 状态；移动实现时必须保持函数和静态对象语义不变。
- ACC/AEB 使用 `.inc` 是为了保持原翻译单元；不要直接将它加入 CMake 独立编译。
- `mControlData` 字段赋值是对外协议行为，清理时不能删除。

## 已知但本轮未修改的问题

- `SpeedControl::m_acc_last` 未在构造函数中初始化，而
  `AccelerationCalculateBySpeed_P()` 最终会把它赋给 `desireAcc`。该发布字段会随
  对象初始内存变化；基础油门/刹车当前不使用传入的 `tAcc`，但外部仍可观察该字段。
- `ControlComply::mPathid` 和 `mPathsafety` 也未在构造函数中初始化，首条有效路径消息
  到来前读取它们存在风险。

这些属于既有行为和独立缺陷，当前整理没有擅自初始化；若要修复，应单独做业务影响确认和
新旧场景验证。
