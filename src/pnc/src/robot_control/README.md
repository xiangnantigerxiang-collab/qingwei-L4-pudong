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
8. 将速度指令换算成油门；
9. 电子围栏与横向偏差停车覆盖；
10. 输出限幅及 R 挡油门覆盖。

后面的步骤可能覆盖前面的控制字段。修改某一业务条件前，应同时检查该条件之后所有对
`mControlData` 的赋值，不能只看局部代码。

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
