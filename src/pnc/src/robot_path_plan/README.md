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
| `path_plan_task.inc` | 初始化、单点限速配置加载、任务接收、任务切换时的历史复位 |
| `task/global_path.inc` | CSV 拼接、车辆连接段、起止点与进度索引 |
| `task/operation_path.inc` | 挂钩、入库、装卸路径调度与生成 |
| `task/airport_stop.inc` | 飞机位配置、占用状态、停车索引 |
| `task/stop_speed.inc` | 动作/入库/挂钩/普通行驶的停车限速和完成判据 |
| `path_plan_perception.inc` | 输入快照、感知过滤、前后扫描坐标转换、云端指令 |
| `path_plan_reference.inc` | 周期任务推进、参考路径编排与状态发布 |
| `reference/path_generation.inc` | 20 点采样、末端延长、挡位分流、旧轨迹复用 |
| `reference/collision_safety.inc` | 新/旧感知避障入口、兼容路径与独立前扫描停车规则 |
| `safety/perception_safety.h/.inc` | 按 ID 缓存真实观测、连续碰撞判断、确认/迟滞与舒适限速 |
| `safety/perception_safety_adapter.inc` | 参数与话题输入、路径缓存、限速/安全标志叠加 |
| `path_plan_output.inc` | 最终速度平滑、安全覆盖、单点区域限速、声光参数和消息发布 |
| `safety/startup_observation.inc` | 周边/T 路口观察、等待区判定 |
| `path_plan_experimental.inc` | 休眠 lattice/换道入口、旧测试限速；未调用的可视化发布接口于 09-14 由用户删除 |

15 个 `.inc` 均通过 `path_plan_comply.cpp` 编译，不能单独加入 CMake。几何辅助在最前，
原有五个业务分片仍按任务、感知、参考路径、最终输出、实验算法的顺序包含；
新感知核心与适配紧接几何辅助包含；其余 `task/`、`reference/`、`safety/` 由对应业务分片继续包含。

业务状态继续由 `PathPlanComply` 持有，阶段函数通过私有接口组织；09-12 重构没有新增持久
状态、改变成员顺序或引入其他线程。`adaptiveHook.*`、`collisioncheck/`、`common/`
等专用算法保持独立；`lattice/`、`lattice_plan/`、`reference_line/`、`speedplan/`
的现有编译关系保持不变。

## 每周期的业务顺序

1. `PathPlanProcess()`：补充作业路径，更新最近点，按任务类型计算停车限速与完成状态。
2. `PublishReferPath()`：检查任务/路径有效性，采样参考点，按挡位更新参考路径，发布中间结果。
3. `PublishPlanPath()`：更新观察窗，处理人工/短路径退化，起步判定，复制参考路径，
   平滑加速，叠加急停/断网，生成声光，处理等待区和云端暂停，应用单点区域限速，
   发布时叠加闸机安全标志。
4. `PublishPathPlanStatus()`：发布前面阶段更新后的任务和诊断状态。

最终输出阶段的速度和灯光有顺序覆盖关系。`CopyReferencePath()` 仅复制原有六个字段，
不能改为整包赋值。起步判定写入的 `safety` 随后会被参考路径覆盖，起步未通过时的
零速由 `ApplyWaitingAreaStop()` 保证。人工/短路径退化仅发布停车路径，不发布声光消息；
观察窗在这两个早退判断之前仍会更新。

## 单点区域限速（2026-09-17）

规划初始化时，通过现有 `path_dir` 参数读取 `speed_limit.csv`，默认位置为
`src/pnc/path/speed_limit.csv`。每行三个数值，无表头：

```csv
96.14,-405.63,0.5
91.13,-393.80,0.5
```

`x,y` 使用与 `/navigation_msg.xAxis/yAxis` 相同的坐标系，单位为米；
`speedlimit` 单位为 **m/s**，与 `/plan_path_msg.desireSpeed` 一致，允许为 0。
每周期比较当前定位点与所有配置点的距离，**距离 ≤ 5 m** 时，最终输出
`desireSpeed = min(原规划 desireSpeed, 区域 speedlimit)`。区域重叠取所有命中限速的最小值；
离开区域后使用原规划结果，不保持该区域限速。

限速在最终速度平滑及其他停车条件之后执行，只降低 `/plan_path_msg.desireSpeed`，
保留更低速度和零速，不修改 `safety`、参考路径、任务状态、声光或闸机规则。
人工/短路径提前返回本来就是零速，仍按原流程输出。

空文件不增加限速；文件不存在时打印启动提示并保持原规划。空白行忽略，格式错误、
非有限数值或负限速的行跳过，并在加载结束时报告有效点数和无效行数。
文件只在初始化时读一次，**修改或清空文件后需重启规划节点生效**；任务切换不重载文件。
每周期 O(N) 遍历内存缓存、比较平方距离，不开方、不读文件、不访问参数服务器，
新增空间为 O(N)，限速计算本身不分配内存。用户的原 CSV 内容未修改。

复现测试见 [`tests/planning`](../../tests/planning/README.md#单点区域限速验证)。

## 闸机状态输入（2026-09-17）

`path_plan_node` 订阅 `/gantry_state`（`gantry_detect/gantry_state`，队列 1，TCP_NODELAY），
回调调用 `SetGantryState(active, gantry_open)`，仅更新一个布尔状态，主循环保持 10Hz。
未收到消息时不生效；之后采用最新一条消息的状态，不另加超时、去抖或任务复位规则。

| active | gantry_open | 最终 `/plan_path_msg.safety` |
|---|---|---|
| false | 任意 | 保留原规划结果 |
| true | true | 保留原规划结果 |
| true | false | 置为 1 |

消息的实际字段名为 `safety`，类型为 `bool`；没有修改 `.msg`。
正常输出和人工/短路径退化输出均通过 `PublishFinalPlanPath()` 发布。
闸机只覆盖本次发布的标志，发布后恢复原内部值，防止开闸/失效后在退化路径中残留，
同时保留其他停车条件的 `safety`。路径点、速度、声光、参数写入和任务逻辑保持原样。
新增计算为常数次布尔操作，不复制路径数组、不新增线程或定时器。

编译使用实际生成的 `gantry_detect/gantry_state.h`，替换了之前不存在的
`gantry_detect/gantry_detect.h` 引用。头文件目录和消息生成依赖仅挂到 `path_plan_node`。
当前 `src/gantry_detect/` 已由同日其他改动移为实体目录，根目录不再有模块或软链接；
部署时同步该实体包到车载工作空间 `src/`，先生成 gantry 消息、
重编 `robot` 并重启规划节点。检测模块自身代码、参数和整车启动脚本没有修改。

验证结果：未输入闸机状态时，修改前后 387 组完整消息、状态和参数事件一致；
闸机检查经同日复核扩展至 1,806 项，覆盖开闭/失效切换、回调接线、其他安全原因、
退化路径、任务复位及不同安全原因的连续交接；三种错误清零变异均被测试检出。
详见 [`tests/planning`](../../tests/planning/README.md)。本机为 ROS 桩验证，未部署 Orin。

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

planning 在 `/perception` 和 `/perception/planning` 两个入口直接剔除 `type=3/4`
（左/右车道外），保留 `type=0/1/2`（本车道/左一/左二）继续执行既有尺寸、几何检查。
有效新帧将同 ID 改为车道外时，立即移除其旧轨迹，不能继续外推；兼容链也清掉该 ID 的
旧风险历史。起步/T 区只看到过滤后的感知，观察窗和其他独立停车规则保持。

planning 在 `/perception` 和 `/perception/planning` 两个入口剔除同时满足
`dx > 1 && dy > 1 && (dx/dy > 3 || dy/dx > 3)` 的细长框；尺寸恰好 1 m 或比例恰好 3
不因本规则剔除，原有其他过滤条件仍生效。利用正尺寸将比例判断写成乘法比较，
在原遍历中执行，无额外容器分配。转换节点的发布、独立前后扫描输入不变；
规划新链中此前有效的已确认观测仍按原漏检外推上限过期，被剔除帧不会刷新它。

`BuildReferencePathPoints()` 向前采样最多 20 点，末尾不足时沿末端航向补点。
R 挡只检查后向扫描目标到后轴定位点的距离，严格小于 2.5 m 时停车。

收到首个有效 `/perception/planning` 后，前向检查启用新的真实观测避障：使用真实框角点、
固定 ID 缓存、不同帧确认、`vx/vy` 连续预测和按路径外廓判断，普通风险降低目标速度，
同一 ID 连续三帧真实观测满足紧迫碰撞才置 `safety`。前两帧限速为当前 CAN 车速的 60%，
其他更低限速优先；重复规划、漏检外推不累计紧急票数。普通目标确认参数和低分数策略保持，
紧急三帧另行计数。原 `/perception` 仍供起步和 T 区观察使用。
新链启用后不再使用旧的“任意附近目标按中心距离限速”规则，避免绕过路径筛选。
超声停车在新前向链中独立读取缓存参数，不随“感知为空/风险消失”提前跳过；
这项调整防止感知解除时放开另一停车来源。R 挡和独立前扫描原规则保留。
参数、复杂度及适用边界见 [感知避障说明](safety/README.md)。

首个有效新话题到来前保持以下兼容逻辑；启用后断流超过 0.6 秒停车，不退回旧历史帧。
兼容逻辑在其他挡位先生成前向路径，再依次执行 `CollectReferenceRiskObjects()` 的矩形比较、
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

- 兼容链的 `/ultra/status/safe` 只在前向碰撞限速末尾读取，空感知、无风险和后向目标会跳过；
  09-17 新前向链已改为独立缓存读取。R 挡仍按原规则，不在此次改动范围内。
- `handleDrivingPath()` 内装载、卸载、入库分支含同一 `taskType` 同时相等/不等的条件，
  内层生成代码不可达；挂钩路径仍会周期生成。
- `LoadPathFile()` 用 `while (!feof)` 且不检查 `fscanf` 返回值；多 CSV 直接拼接时
  各段 `dist_origin` 不连续。此次只归拢加载代码，没有改变数据解析和里程语义。
- 飞机位配置为空时，回退对象 `id` 未初始化，用于调试打印时存在未初始化读。
- `task_id == 4` 仍调用测试分段限速；当前树没有旧文档所称的“该任务清空感知”代码。
- 现有 `printf` 中仍有格式与参数类型不匹配的告警；休眠算法也保留其原有告警和缺陷。

修改这些规则时，应另行明确期望行为，并更新相应回归断言，不能把本次的基线等价
结论当作全部原有业务规则正确的证明。
