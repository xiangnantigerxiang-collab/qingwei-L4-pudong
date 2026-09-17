# planning / robot_perception_convert 交叉复核（2026-09-17）

后续状态：同日用户要求“三帧紧急确认、前两帧限速 60%、忽略车道外目标”后，
第 1 项相邻段漏查已随该改动修复并加入回归；第 2–5 项仍未修改。
下文保留审查当时的复现事实，当前规则与验证见 [策略修改验证](obstacle_policy_verification_20260917.md)。

本次是业务审查与故障复现，没有修改生产源码、参数、消息、路径 CSV、control 或 monitor。
结论：常规地图定位模式下，新增单点限速的取小、空文件、停车优先级均通过；但发现两项
前向避障错误、一项限速的模式兼容问题，以及两项已有的几何语义问题，不能把原回归通过
等同于两个模块的全部业务正确。以下均区分了触发条件、实际结果和修复方向。

## 1. P1：首次安全余量相交后提前退出，会漏掉后续段的紧急物理碰撞

位置：[perception_safety.inc](../../src/robot_path_plan/safety/perception_safety.inc)，
`PerceptionSafety::Evaluate()` 的第 350–370 行。

算法首先找带普通安全余量的相交，再仅在这一段检查紧急物理相交，随后无条件 `break`。
普通余量的相交可能发生在前一段尾部，真正车身相交则发生在下一段。此时下一段不会被检查，
是否置 `safety` 取决于同一条路线的离散点间距。

复现条件：自车 `(100,100)`、heading=90°、车速 0.8 m/s；静态框中心 `(103.29,100)`，
沿行驶方向长 1.2m、横向宽 1m，默认外廓和制动参数。独立计算的物理净距约 0.39m，
含 0.05m 紧急余量的接触距离约 0.34m，算法紧急距离为 1.02m。

| 实际主流程输入路线 | 输出 safety | 最终 desireSpeed |
|---|---:|---:|
| 同一直线，原始点间距 1m | 1 | 0 |
| 同一直线，原始点间距 0.1m，内部抽取约 0.3m 段 | 0 | 0.792 m/s |

独立核心测试也复现了 0.3m 分段漏判，1/0.4/0.5m 分段正确急停。随后通过真实
`PublishReferPath → CheckForwardReferenceSafety → PublishPlanPath` 再次确认，
不是测试独立几何函数造成的假象。此问题来自此前新增的感知避障，和 CSV 限速无关。

修复方向：普通限速的最早相交与紧急物理相交分别判定；找到普通相交后，仍检查紧急距离
以内的后续段，直到发现紧急相交或已经越过紧急检查范围。可复用现有几何缓存和单次遍历，
无需增加目标跟踪、文件操作或堆分配，也无需解除既有 128 段上限。

## 2. P1：ACC 切换只关闭输入，未关闭规划端的新感知超时决策

位置：[path_plan_node.cpp](../../src/robot_path_plan/path_plan_node.cpp) 第 36–38 行，
[collision_safety.inc](../../src/robot_path_plan/reference/collision_safety.inc) 第 4–6 行，
[perception_safety.inc](../../src/robot_path_plan/safety/perception_safety.inc) 第 302–311 行。

先收到有效 `/perception/planning` 后，`HasInput()` 持续为真。把
`/robot/control/accswitch` 从 0 切为 1 时，真实回调直接返回，规划仍执行新感知检查。
即使上游持续发送新的空观测，超过 0.6s 也会被判定断流。

真实回调复现：100.0s 接收正常空帧；开启 ACC；100.7s 再送新空帧，得到
`reason=5(STALE_INPUT), safety=1, desireSpeed=0`。正常前进输出 `Path_Id=5` 时，
control 的既有 safety 分支会要求满制动。这与 ACC 开关原本交由 control 处理纵向控制的
接线矛盾；开关保持默认 0 不触发此案例。不是 CSV 限速引入的问题。

修复方向：统一输入门控和决策门控，明确 ACC 模式的避障归属；切回规划模式时处理新帧
有效性与速度状态，避免残留旧状态。不能通过伪造新感知时间戳来掩盖真实断流。
其他急停、闸机、云暂停和区域限速应继续独立生效。

## 3. P2：非静态挂钩模式下，地图限速点会与局部定位坐标混用

位置：[path_plan_output.inc](../../src/robot_path_plan/path_plan_output.inc) 第 63 行，
以及 [navigation_node.cpp](../../src/robot_navigation/navigation_node.cpp) 第 69–82 行。

`ADAPTIVEHOOK && PalletType != 0` 时，navigation 将 `/navigation_msg.xAxis/yAxis`
改成挂钩局部坐标；converter 仍使用 `/localization` 的地图坐标。限速函数直接使用
`mNavData`，没有区分这两种坐标系。

调用真实 navigation 的任务、定位、挂钩回调并捕获发布，再输入真实 planning：
地图位置为 `(96.14,-405.63)`，局部定位变为约 `(-1,-0.001)`。在地图原位置设 0.2m/s
测试限速，规划原目标为 0.5m/s，最终仍输出 0.5m/s，未命中区域。测试限速仅在隔离夹具
配置，用户 CSV 未修改。

当前 launch 为 `pallettype=0`，这项缺陷在默认模式不触发；当前用户 CSV 两项均为
0.5m/s，也不同于本复现使用的更低限速。但该模式属于代码已有分支，后续启用并配置更低
区域限速时会出现问题。新增区域限速尚未覆盖它。

修复方向：地图区域限速使用独立保留的原始地图定位，不能直接改掉挂钩循迹需要的局部
`mNavData`。同样应避免把地图系感知与局部系自车位姿混入起步/T 区几何判断。

## 4. P2：运动 heading 已正确输出，但仍有消费者将其当作物理框朝向

位置：[perception_object_tracker.inc](../../src/robot_perception_convert/perception_object_tracker.inc)
第 363–376 行、[perception_msg_convert.cpp](../../src/robot_perception_convert/perception_msg_convert.cpp)
第 210–213、285–288 行，以及根目录 `monitor/ros_visualizer.py:608–612`。

converter 按用户要求通过 `atan2(vx,vy)` 输出北零顺时针的运动 heading，新规划链正确使用
`polygons` 保存的物理角点。原 `/perception` 则清空角点，monitor 及 planning 尚未收到首个
有效新观测时的兼容碰撞链，仍用 `heading` 旋转矩形。

真实 converter＋HDMap SDK 复现：自车朝东，输入 Marker 宽 1m、长 4m、yaw=90°，
世界角点实际范围为横向 4m、纵向 1m；首次零速回退 heading 约为 0°。
实际 monitor 的 `snapshot()` 把同一对象解释为横向 1m、纵向 4m，框旋转了 90°。
目标运动方向与物理朝向不同、或停车后保持上次运动 heading 时，同样不能用运动航向恢复框。

这会造成显示框与 type/新规划的物理几何看起来不一致，并影响兼容碰撞分支的矩形。
它不是 20% 面积门槛或 vx/vy 的计算公式错误；新规划链已通过角点避免此问题。
修复需保持用户要求的运动 heading，另行提供/消费物理几何；不能把运动 heading 改回框朝向。
`object.msg` 中 heading 的旧“box rotation”注释也已过时，本轮未改消息或 monitor。

## 5. P2：扫描雷达的纵向偏移固定减地图 x，未随车头方向旋转

位置：[path_plan_perception.inc](../../src/robot_path_plan/path_plan_perception.inc)
第 125–126 行（前扫描）及第 228–229 行（后扫描）。

前扫描的 `-1.5`、后扫描的 `-0.9` 位于地图 x 平移项。若它们表示代码注释中的车体纵向
安装偏移，应在局部坐标里合并后随 heading 一起旋转，固定减地图 x 不符合该约定。

真实前扫描回调复现：自车 `(100,100)` 朝北，雷达目标在局部前方 6m，当前输出
`(98.5,106)`；按现有 -1.5m 纵向偏移旋转应为 `(100,104.5)`。
这会改变路侧目标的横向位置及碰撞距离。属于原有问题，不是本轮限速或新跟踪改动。
修复前须核对实车前/后雷达外参的轴向与符号，不能借此随意重标定距离。

## 保留下来的误停来源

起步/T 路口仍消费旧 `/perception` 的两秒历史，T 区判断按目标中心落入硬编码区域，
不检查 `type` 或新链的真实帧确认。输入新的空观测，同时保留一个旧 T 区历史对象时，
复现得到新链 `reason=0(CLEAR)`，最终却被等待区规则压为 `desireSpeed=0`。

这是此前为保持原业务而保留的独立停车条件，本身不证明 T 区应该放行；但说明新的
前向避障不能消除所有起步/T 区误停。要改变它，需要单独明确这些区域的实际观察规则，
不能直接删掉其他安全条件。3.4m 默认保护宽度也仍在使用，type=车道外不等于不侵入保护外廓。

## 已通过的检查与范围

| 检查 | 本轮结果 |
|---|---|
| 单点区域限速：空文件、5m 边界、重叠、零速/安全来源 | 284 项通过 |
| 闸机与其他安全原因交接 | 1,806 项通过 |
| planning 感知集成及比例 3 过滤 | 52 项通过 |
| 新感知核心 C++11 重新编译 | 4,795 项通过 |
| 既有业务完整快照 | 387 组与上轮已核验版本逐字节一致 |
| converter 跟踪/速度/航向/ID | 19,733 项；ASan/UBSan 同样通过 |
| converter 真实发布链＋HDMap SDK | 4,414 项；ASan/UBSan 同样通过 |
| converter 实际 CMake 目标、C++11、SDK 动态链接 | 通过 |
| 新增组合反例 | 上述问题已复现，未修复 |

旧回归重新运行前核对当前规划源码与上轮编译 SHA 一致；新限速前后原差分仅允许初始化
新增一次 `get:path_dir`，未命中区域时其他状态/消息/参数不变。当前这轮 387 组比较以已包含
该启动读取的上轮版本为基线。control、参数、消息和 CSV 没有业务改动；开工前后对
PNC/monitor 已有 1,502 个文件核对，编译/检查结束时字节均未变，随后仅新增本报告并更新 workflow。

以上是主机真实源码＋消息桩的结果，未包含 ROS1/Orin 调度、整车 CPU 总负载或车辆动态。
这轮不能认定实车急刹/滞后的全部原因均已找全；旧用例通过也不能覆盖新增反例。

原始日志及复现代码：`/tmp/planning_convert_review_20260917_152858/`。
复现源码、日志与结果另存工程同级 `planning_convert_review_20260917_152858.tar.gz`，
避免 `/tmp` 清理后失去审查证据。
主要文件为 `planning_edge_review.cpp` / `planning_edge.log`、`core_edge_review.cpp`、
`convert_edge_review.cpp` / `convert_edge.log`、`monitor_geometry.json` 和 `result.json`。
建议优先修复第 1、2 项，并将这批反例变为正式回归；第 3 项补齐坐标系切换，
第 4、5 项按明确的几何字段与外参约定修复。本轮未修改控制标定，也未部署车辆。
