# robot_path_plan 模块说明

`path_plan_node` 每秒执行 10 次规划，接收任务、定位、CAN 和感知输入，发布
`/refer_path_msg`、`/plan_path_msg`、`/path_plan_status`。编码风格遵循
[`src/pnc/README.md`](../../README.md)：4 空格、`if(condition) {` 同行大括号、原有命名、中文业务注释。

## 目录与职责

| 文件 | 职责 |
|---|---|
| `path_plan_node.cpp` | ROS 接线、参数热读、传感器心跳和周期调度 |
| `path_plan_comply.h` | 输入输出接口、快照、跨帧状态与阶段函数声明 |
| `path_plan_comply.cpp` | 唯一编译入口，保持 include 依赖顺序 |
| `path_plan_geometry.inc` | 矩形区域判断、坐标转换、方向与距离辅助 |
| `path_plan_task.inc` | 初始化、任务接收、任务切换时的历史复位 |
| `task/global_path.inc` | CSV 拼接、车辆连接段、起止点与进度索引 |
| `task/operation_path.inc` | 挂钩、入库、装卸路径调度与生成 |
| `task/airport_stop.inc` | 飞机位配置、占用状态、停车索引 |
| `task/stop_speed.inc` | 动作/入库/挂钩/普通行驶的停车限速和完成判据 |
| `path_plan_perception.inc` | 输入快照、感知过滤、前后扫描坐标转换、云端指令 |
| `path_plan_reference.inc` | 周期任务推进、参考路径编排与状态发布 |
| `reference/path_generation.inc` | 20 点采样、末端延长、挡位分流、旧轨迹复用 |
| `reference/collision_safety.inc` | 路径风险筛选、四帧历史、前向过滤、碰撞限速 |
| `path_plan_output.inc` | 最终速度平滑、安全覆盖、声光参数和消息发布 |
| `safety/startup_observation.inc` | 周边/T 路口观察、等待区判定 |
| `path_plan_experimental.inc` | 休眠 lattice/换道入口、旧测试限速；未调用的可视化发布接口于 09-14 由用户删除 |

13 个 `.inc` 均通过 `path_plan_comply.cpp` 编译，不能单独加入 CMake。几何辅助在最前，
原有五个业务分片仍按任务、感知、参考路径、最终输出、实验算法的顺序包含；
`task/`、`reference/`、`safety/` 由对应业务分片继续包含。

业务状态继续由 `PathPlanComply` 持有，阶段函数通过私有接口组织；此次没有新增持久
状态、改变成员顺序或引入其他线程。`adaptiveHook.*`、`collisioncheck/`、`common/`
等专用算法保持独立；`lattice/`、`lattice_plan/`、`reference_line/`、`speedplan/`
的现有编译关系保持不变。

## 每周期的业务顺序

1. `PathPlanProcess()`：补充作业路径，更新最近点，按任务类型计算停车限速与完成状态。
2. `PublishReferPath()`：检查任务/路径有效性，采样参考点，按挡位更新参考路径，发布中间结果。
3. `PublishPlanPath()`：更新观察窗，处理人工/短路径退化，起步判定，复制参考路径，
   平滑加速，叠加急停/断网，生成声光，处理等待区和云端暂停，发布最终结果。
4. `PublishPathPlanStatus()`：发布前面阶段更新后的任务和诊断状态。

最终输出阶段的速度和灯光有顺序覆盖关系。`CopyReferencePath()` 仅复制原有六个字段，
不能改为整包赋值。起步判定写入的 `safety` 随后会被参考路径覆盖，起步未通过时的
零速由 `ApplyWaitingAreaStop()` 保证。人工/短路径退化仅发布停车路径，不发布声光消息；
观察窗在这两个早退判断之前仍会更新。

## 任务、路径与停车

`SetTaskPlanData()` 每次收到事件都重载飞机位配置和路径。判断 `task_id` 是否切换后
才覆盖旧任务：切换调用 `ResetTaskHistory()`，同任务的块推进和重复消息保留历史窗、
去抖计数和云端暂停。所有情况都保留急停闩锁。

`calcuGlobalPath()` 的阶段顺序为：

```text
LoadTaskPaths → FindTaskStartPoint → ConnectVehicleToPath → FindTaskStopPoint
     CSV 拼接       寻找起点           生成连接段              寻找停车点
```

初始最近点搜索排除最后 10 点，停车点搜索从第 4 点开始，连接段只在距离原路径严格
介于 1~2.5 m 时生成。周期内 `UpdatePathInfo()` 只向前推进；其上界是会随候选点更新的
`keyPoint + 80`，不是固定的 80 点范围。

`LimitSpeedByDistanceToStop()` 将业务分流到四个规则函数：

| 函数 | 返回速度与完成判据 |
|---|---|
| `UpdateActionTaskStatus()` | 动作任务返回零速，按挂钩状态或无停车点哨兵判断完成 |
| `LimitParkTaskSpeed()` | 按入库剩余距离限速；越过目标方向或不足 0.2 m 判到位 |
| `LimitHookTaskSpeed()` | 使用 `center_distance`，保留托盘连接反馈和距离反增迟滞 |
| `LimitDrivingTaskSpeed()` | 飞机位占用时服从临停线；到最终停车点才结束任务 |

`distance2Stop` 先记录任务终点距离，飞机位临停和挂钩感知距离随后只更新
`remain_distance_`。两者数值可能不同，这是保留的既有发布语义。

## 参考路径与碰撞

`BuildReferencePathPoints()` 向前采样最多 20 点，末尾不足时沿末端航向补点。
R 挡只检查后向扫描目标到后轴定位点的距离，严格小于 2.5 m 时停车。

其他挡位先生成前向路径，再依次执行 `CollectReferenceRiskObjects()` 的矩形比较、
`UpdateReferenceRiskHistory()` 的历史保留、`FilterForwardRiskObjects()` 的前方
±90° 过滤，以及 `ApplyReferenceCollisionSpeed()` 的分段限速和三次安全去抖。

路径间距小于 0.1 m 的目标进入风险列表；临时复用 `object.height` 保存间距。
四帧窗口取最近的非空结果，前向过滤的 `history_risk_vec_filter` 则是每次重建的局部变量。
各减速阈值按原顺序依次覆盖，不能改成互斥的 `else-if`。

必须区分三个提前结束检查的分支：

- 感知目标为空：不推进风险窗，不更新 `distance2Object`。
- 四帧窗口无风险：写入 `100 + 感知目标数`。
- 风险目标全部在后方：写入 `200 + 风险目标数`。

这三个分支仍发布参考路径，但不执行后续安全计数和 ultra 参数读取。

## 校验与已知问题

2026-09-14 检查了用户的 planning 整理：两组可视化函数与声明同步删除，原调用均为注释；
`/test_trajs`、`/planning/obstacles` 不再声明，旧 RViz 配置仍留有后者的显示项，但原本也没有
活动发布。另删除若干调试日志/注释、调整书写格式和订阅声明位置。前后版本的 387 组状态、
实际发布消息和参数事件一致，未发现该次整理引入业务逻辑错误。随后全包格式统一单独以开工
快照验证，193 个自有源码文件的代码 token 和预处理指令一致；审查记录见
[`tests/planning/review_20260914.md`](../../tests/planning/review_20260914.md)。

可复现的编译/差分测试位于 [`tests/planning`](../../tests/planning/README.md)。测试使用
真实业务源码、真实 CSV 和几何算法；ROS 通信层由桩代替，robot 消息桩从 `.msg`
机械生成。它不能替代车载 ROS1 编译及实车/rosbag 验证。

本次校验确认以下行为已存在于重构前，未改变其业务规则：

- `/ultra/status/safe` 只在前向碰撞限速末尾读取：空感知、无风险、后向目标和 R 挡
  都会跳过。若期望 ultra 对全部路径强制停车，现有读取位置不能满足。
- `handleDrivingPath()` 内装载、卸载、入库分支含同一 `taskType` 同时相等/不等的条件，
  内层生成代码不可达；挂钩路径仍会周期生成。
- `LoadPathFile()` 用 `while (!feof)` 且不检查 `fscanf` 返回值；多 CSV 直接拼接时
  各段 `dist_origin` 不连续。此次只归拢加载代码，没有改变数据解析和里程语义。
- 飞机位配置为空时，回退对象 `id` 未初始化，用于调试打印时存在未初始化读。
- `task_id == 4` 仍调用测试分段限速；当前树没有旧文档所称的“该任务清空感知”代码。
- 现有 `printf` 中仍有格式与参数类型不匹配的告警；休眠算法也保留其原有告警和缺陷。

修改这些规则时，应另行明确期望行为，并更新相应回归断言，不能把本次的基线等价
结论当作全部原有业务规则正确的证明。
