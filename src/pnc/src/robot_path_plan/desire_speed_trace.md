# Planning desireSpeed 编号对照（历史记录）

2026-09-21：用户调试完成，以下50处临时输出已全部从运行源码删除。
本文仅供查阅此前的调试日志；当前代码不再输出这些编号及对应的三列速度。

以下为调试阶段的记录。全部编号是固定的代码位置，按实际业务调用链编排，不使用运行时计数器。
每条语句采用 `std::cout << "number x " << 对应速度变量 << " " << mNavData.gpsSpeed << " " << mVehicleData.vehicleSpeed << std::endl;`。
编号后依次输出期望速度、GPS速度和车辆速度，各字段之间用一个空格分隔，
例如 `number 49 1.46 1.3 1.32`。速度单位为 m/s。下表“输出变量”列列出期望速度变量，
所有位置都随后输出同样的GPS速度和车辆速度字段，只读取当前消息值。

正常周期从 `PathPlanProcess()` 的 **14** 开始，经参考路径和安全限速，最终到 **49** 发布、
**50** 恢复内部快照。同一条执行路径按编号递增；互斥分支跳号，下一周期重新从 14 开始。
1～11 属于初始化、复位或消息回调，按事件发生时输出，不保证与其他回调的到达顺序一致。
12～13 为作业路径失败处理，位置在周期速度接收之前；现有作业调度的不可达条件未改动。

**38** 是 `/refer_path_msg.desireSpeed` 的实际发布值；**49** 是
`/plan_path_msg.desireSpeed` 的实际发布值。**50** 仅表示发布后恢复内部速度，不能当作本次发送值。
闸机可能只叠加 `safety` 而不改速度，停车仍须结合既有 `plan safety:` 和消息 `safety` 判断。

常见流程（方括号表示条件分支）：

- D 挡、专用感知、无前扫描碰撞：14 → 15 → 16 → 17 → 23 → [24] → 33 → 34 → 35 → 36 → 37 → 38 → 40 → [41 或 42] → [43] → [44] → [45] → 46 → 47 → [48] → 49 → 50。
- D 挡兼容碰撞处理：23/[24] 后进入 25，接着 26 或 27→28→29→30，再经 [31]、[32] 回到后续主链。专用感知的独立前扫描也可能调用这一段，然后继续 33～36。
- R 挡：14 → 15 → 16 → 17 → 21 → [22] → 38，再进入输出阶段。
- 手动或短路径的最终输出：39 → 47 → [48] → 49 → 50。

10、11、17、27～30、35、36 在相应条件检查结束后打印；条件未命中时仍输出当前保留值。
其余赋值点在对应分支内打印。没有新增或重排条件、修改公式、重复调用计算函数或改变消息字段。
全部 44 处有效的 `desireSpeed` / `mDesireSpeed` 显式赋值均已覆盖，另包含任务整包复制、
输入、碰撞入口、速度快照及两处发布值，共 50 个位置。9～11 是兼容感知回调的局部计算，原本不发布。

| 编号 | 业务位置 | 输出变量 | 函数与文件 |
|---|---|---|---|
| 1 | 初始化规划速度 | `mDesireSpeed` | `InitParameter()` · [path_plan_task.inc](path_plan_task.inc) |
| 2 | 初始化任务速度 | `mTaskPlanData.desireSpeed` | `InitParameter()` · [path_plan_task.inc](path_plan_task.inc) |
| 3 | 参数复位规划速度 | `mDesireSpeed` | `ResetParameter()` · [path_plan_task.inc](path_plan_task.inc) |
| 4 | 参数复位任务速度 | `mTaskPlanData.desireSpeed` | `ResetParameter()` · [path_plan_task.inc](path_plan_task.inc) |
| 5 | 收到的任务速度 | `task_plan_t.desireSpeed` | `SetTaskPlanData()` · [path_plan_task.inc](path_plan_task.inc) |
| 6 | 接纳并复制任务速度 | `mTaskPlanData.desireSpeed` | `SetTaskPlanData()` · [path_plan_task.inc](path_plan_task.inc) |
| 7 | 新任务清空参考速度 | `mReferPath.desireSpeed` | `SetTaskPlanData()` · [path_plan_task.inc](path_plan_task.inc) |
| 8 | 任务复位参考速度 | `mReferPath.desireSpeed` | `resetTask()` · [path_plan_task.inc](path_plan_task.inc) |
| 9 | 兼容感知回调局部速度初值（不发布） | `desireSpeed` | `SetPerceptionData()` · [path_plan_perception.inc](path_plan_perception.inc) |
| 10 | 兼容感知回调距离限制后（不发布） | `desireSpeed` | `SetPerceptionData()` · [path_plan_perception.inc](path_plan_perception.inc) |
| 11 | 兼容感知回调安全检查后（不发布） | `desireSpeed` | `SetPerceptionData()` · [path_plan_perception.inc](path_plan_perception.inc) |
| 12 | 停止点路径为空清零 | `mDesireSpeed` | `UpdateStopPoint()` · [task/operation_path.inc](task/operation_path.inc) |
| 13 | 装载路径加载失败清零 | `mDesireSpeed` | `GenerateLoadPath()` · [task/operation_path.inc](task/operation_path.inc) |
| 14 | 周期规划接收任务速度 | `mDesireSpeed` | `PathPlanProcess()` · [path_plan_reference.inc](path_plan_reference.inc) |
| 15 | 地图限速后 | `mDesireSpeed` | `PathPlanProcess()` · [path_plan_reference.inc](path_plan_reference.inc) |
| 16 | 停止距离限速后 | `mDesireSpeed` | `PathPlanProcess()` · [path_plan_reference.inc](path_plan_reference.inc) |
| 17 | R挡限速检查后 | `mDesireSpeed` | `PathPlanProcess()` · [path_plan_reference.inc](path_plan_reference.inc) |
| 18 | N挡清零 | `mDesireSpeed` | `PathPlanProcess()` · [path_plan_reference.inc](path_plan_reference.inc) |
| 19 | 任务完成清零 | `mDesireSpeed` | `PathPlanProcess()` · [path_plan_reference.inc](path_plan_reference.inc) |
| 20 | 任务完成或短路径清空参考速度 | `mReferPath.desireSpeed` | `PublishReferPath()` · [path_plan_reference.inc](path_plan_reference.inc) |
| 21 | R挡参考速度赋值 | `mReferPath.desireSpeed` | `UpdateReverseReferencePath()` · [reference/path_generation.inc](reference/path_generation.inc) |
| 22 | R挡后向障碍停车 | `mReferPath.desireSpeed` | `UpdateReverseReferencePath()` · [reference/path_generation.inc](reference/path_generation.inc) |
| 23 | 前向参考速度取小 | `mReferPath.desireSpeed` | `UpdateForwardReferencePath()` · [reference/path_generation.inc](reference/path_generation.inc) |
| 24 | 任务4分段限速后 | `mReferPath.desireSpeed` | `UpdateForwardReferencePath()` · [reference/path_generation.inc](reference/path_generation.inc) |
| 25 | 兼容碰撞限速入口 | `mReferPath.desireSpeed` | `ApplyReferenceCollisionSpeed()` · [reference/collision_safety.inc](reference/collision_safety.inc) |
| 26 | 车道外距离混速后 | `mReferPath.desireSpeed` | `ApplyReferenceCollisionSpeed()` · [reference/collision_safety.inc](reference/collision_safety.inc) |
| 27 | 10至25m碰撞距离分支检查后 | `mReferPath.desireSpeed` | `ApplyReferenceCollisionSpeed()` · [reference/collision_safety.inc](reference/collision_safety.inc) |
| 28 | 小于20m碰撞距离分支检查后 | `mReferPath.desireSpeed` | `ApplyReferenceCollisionSpeed()` · [reference/collision_safety.inc](reference/collision_safety.inc) |
| 29 | 小于15m碰撞距离分支检查后 | `mReferPath.desireSpeed` | `ApplyReferenceCollisionSpeed()` · [reference/collision_safety.inc](reference/collision_safety.inc) |
| 30 | 小于10m碰撞距离分支检查后 | `mReferPath.desireSpeed` | `ApplyReferenceCollisionSpeed()` · [reference/collision_safety.inc](reference/collision_safety.inc) |
| 31 | 兼容碰撞链超声停车 | `mReferPath.desireSpeed` | `ApplyReferenceCollisionSpeed()` · [reference/collision_safety.inc](reference/collision_safety.inc) |
| 32 | 兼容碰撞链safety停车 | `mReferPath.desireSpeed` | `ApplyReferenceCollisionSpeed()` · [reference/collision_safety.inc](reference/collision_safety.inc) |
| 33 | 独立前扫描检查后保留原速度上限 | `mReferPath.desireSpeed` | `CheckTrackedReferenceSafety()` · [safety/perception_safety_adapter.inc](safety/perception_safety_adapter.inc) |
| 34 | 专用感知限速后 | `mReferPath.desireSpeed` | `CheckTrackedReferenceSafety()` · [safety/perception_safety_adapter.inc](safety/perception_safety_adapter.inc) |
| 35 | 专用感知safety覆盖后 | `mReferPath.desireSpeed` | `CheckTrackedReferenceSafety()` · [safety/perception_safety_adapter.inc](safety/perception_safety_adapter.inc) |
| 36 | 专用感知链超声覆盖后 | `mReferPath.desireSpeed` | `CheckTrackedReferenceSafety()` · [safety/perception_safety_adapter.inc](safety/perception_safety_adapter.inc) |
| 37 | 前向碰撞检查后恢复任务/终点上限 | `mReferPath.desireSpeed` | `PublishReferPath()` · [path_plan_reference.inc](path_plan_reference.inc) |
| 38 | 参考路径实际发布值 | `mReferPath.desireSpeed` | `PublishReferPath()` · [path_plan_reference.inc](path_plan_reference.inc) |
| 39 | 手动或短路径停车值 | `mPlanPath.desireSpeed` | `PublishStoppedPath()` · [path_plan_output.inc](path_plan_output.inc) |
| 40 | 参考速度复制到规划输出 | `mPlanPath.desireSpeed` | `CopyReferencePath()` · [path_plan_output.inc](path_plan_output.inc) |
| 41 | 挂托盘慢加速平滑后 | `mPlanPath.desireSpeed` | `SmoothPlanSpeed()` · [path_plan_output.inc](path_plan_output.inc) |
| 42 | 普通加速平滑后 | `mPlanPath.desireSpeed` | `SmoothPlanSpeed()` · [path_plan_output.inc](path_plan_output.inc) |
| 43 | 急停清零 | `mPlanPath.desireSpeed` | `ApplyEmergencyAndNetworkStop()` · [path_plan_output.inc](path_plan_output.inc) |
| 44 | 断网清零 | `mPlanPath.desireSpeed` | `ApplyEmergencyAndNetworkStop()` · [path_plan_output.inc](path_plan_output.inc) |
| 45 | 云端暂停清零 | `mPlanPath.desireSpeed` | `PublishPlanPath()` · [path_plan_output.inc](path_plan_output.inc) |
| 46 | 发布出口地图限速后 | `mPlanPath.desireSpeed` | `PublishPlanPath()` · [path_plan_output.inc](path_plan_output.inc) |
| 47 | 独立停车覆盖前的速度快照 | `mPlanPath.desireSpeed` | `PublishFinalPlanPath()` · [path_plan_output.inc](path_plan_output.inc) |
| 48 | 起步观察停车清零 | `mPlanPath.desireSpeed` | `PublishFinalPlanPath()` · [path_plan_output.inc](path_plan_output.inc) |
| 49 | 最终规划消息实际发布值 | `mPlanPath.desireSpeed` | `PublishFinalPlanPath()` · [path_plan_output.inc](path_plan_output.inc) |
| 50 | 发布后恢复原速度，防止停车覆盖残留 | `mPlanPath.desireSpeed` | `PublishFinalPlanPath()` · [path_plan_output.inc](path_plan_output.inc) |

新增打印复用现有变量，不增加状态、容器、循环或算法调用。使用 `std::endl` 会增加同步输出和刷新开销，
本机业务回归不代表车端时延不变；未测量车端性能。编译和验证记录见[workflow](../../../../workflow.md)。
