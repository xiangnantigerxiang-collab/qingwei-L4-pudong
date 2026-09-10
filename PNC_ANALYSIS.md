# PNC_ANALYSIS.md — pnc 模块深度分析报告

> 生成：2026-08-29。方法：20-agent 工作流（8 子系统深读 → 话题契约/安全链/死代码三路交叉核查 → 5 批对抗实证 → 完备性批判 + 3 项补漏：时序/状态机恢复/数据资产），1078 次工具调用，全部结论以当前代码行号落点。
> 标记约定：**【实证】**=第二独立 agent 复读代码确认（25 条高/中问题走此流程，21 CONFIRMED / 4 PARTIAL）；**【待核】**=单 agent 结论未及对抗复核；**【修正】**=推翻 workflow.md 旧结论。
> 路径基准：`src/pnc/`（下文省略）；行号为当前副本实测。

---

## 0. 执行摘要

1. **pnc 实际形态**：纯 CSV 循迹 + 逐帧矩形碰撞分级减速 + 简化纵向控制（油门=速度×18）。lattice/速度规划/参考线平滑**整套 Apollo 移植链编译但休眠**（唯一调用点 path_plan_comply.cpp:1543 被注释，函数内自注"后面的代码有 bug":2642）。
2. **最危险新发现——冻结油门单点失效**【待核·gap2 代码核实】：control_node 或 can_comm_node 崩溃 → can_comm_node 忙循环(kHz 级)无限重发最后一条指令，canbus `OnControlCommand` 无时间戳(canbus_core.cpp:338-358) → **车辆按冻结油门持续行驶且无解除路径**（planningAlive/sensorstate/fence 此时全部正常维持）。can_comm_node 还存在忙循环缺陷：声明 20Hz 但循环体无 `loop_rate.sleep()`(can_comm_node.cpp:34-40)。
3. **两处输入健壮性缺口会直接崩节点**【实证】：任务 yaml/路径 CSV 缺失即段错误（task14/15 引用不存在的 aircraft2/aircraft3.csv；`ParseTaskFile` YAML 无 try/catch；两处 `LoadPathFile` fopen 无判空）+ **FILE\* 全链无 fclose**（约 990 次任务消息后 fd 耗尽崩溃）。
4. **急停 latch 死锁**【实证·gap2】：emergencyStop 置位后唯一复位依赖 `/robot/serial/rs232/`——全工作区无发布者 → 一次 CAN 急停后自动驾驶永久锁 0 速；重启 path_plan 又撞上 task_plan 发布门控互锁（NOTASK 状态永不触发重发，需云端整单重发且从 block0 重跑）。
5. **【修正】负向偏差"三重漏判"旧结论不成立**（详见 §4.2）：主停车判断 :579 用的 biaDistance 已是无符号垂距（:495 hypot 覆盖带符号值），负向偏差**能**停车；另两处（:738 重复行、BiaDisLimitSpeed 未取 fabs）确实存在但**整体位于零调用的死函数 SpeedJudge 内**，运行时无影响。原定"安全修复"降级为"死代码清理决策"。
6. **实车正在踩的坑**【实证】：fence.csv 不覆盖在用的 path_pudong0517（task3/task4，452-642 行共 191 点越栏 0.1~1.6m → 必中途刹 70%）；task4 是真实任务却命中 `task_id==4` 感知旁路（**全程关闭避障**+测试限速残留）；max_vehicle_speed=15(km/h 注释) 与 m/s 直接 min → 上限等效 54km/h 形同虚设。
7. **控制纵向链大面积失效**【实证】：`InitParameter` 从未被调用 → config.yaml 控制参数全不加载（R 挡几何控制器用默认 wheelbase=2.0 而非实车 1.6）；模糊 PID 结果被未初始化成员覆盖(desireAcc 恒 0)；最小二乘标定表被 ×18 覆盖；can_comm 油门斜率保护"先赋值后比较"恒不生效；超速刹车实际只有 1%（每周期清零重加）。
8. **时延链**：驱动 CAN 帧实际 **5Hz**（canbus 0.2s 定时器锁存，control 20Hz 指令 75% 被丢弃）+ safety 3 帧去抖 300ms → 感知→CAN 典型 ~0.35s(1.5m@15km/h)、最坏 ~0.85s(3.5m)。
9. **对云上报两处反了**【实证】：转向灯 turning 字段左右互换（:2069-2076）；deviceId 三处硬编码不一致（心跳 A03 / 反馈"拖A0002" / fms 主题"拖A0005"）。
10. **休眠算法启用前不可修复项清单**见 §2.3/§2.4（30km/h 硬编码、static 跨帧残留、Frenet 符号两套相反、front 障碍不减速等 26 项缺陷）。

---

## 一、模块全景

### 1.1 六节点（全部单线程 ros::spinOnce，全库无 AsyncSpinner/std::thread【核实】）

| 节点 | 可执行/频率 | 职责 | 入口 |
|---|---|---|---|
| task_plan_node | 20Hz 主循环 + 10Hz 心跳 T1 | 任务块状态机、云端 FMS 交互 | task_plan_node.cpp:237 |
| path_plan_node | 10Hz 主循环 + 10Hz 传感器 T1 | 路径加载/跟踪/碰撞分级减速/safety | path_plan_node.cpp:222 |
| control_node | 20Hz | 横纵向控制、围栏、多级停车 | control_node.cpp:100 |
| navigation_node | 50Hz 定时重发（ros::init 名=serial_send_node） | ivlocmsg→navigation_msg 整形、挂车模式反算 | navigation_node.cpp:142 |
| can_comm_node | 声明 20Hz **实际忙循环 kHz 级**【实证】 | task+control+声光 → can_comm_msg 聚合 | can_comm_node.cpp:34 |
| perception_msg_convert | 100Hz spin 事件驱动 | /box 车体系→地图系 → /perception | perception_msg_convert.cpp:177 |

启动链：工作区 launch/control.launch 聚合 pnc 5 个子 launch（包内 robot_start.launch 无人调用）；**全部 launch 无 respawn/required**。canbus 先于 pnc 约 13-19s 上线（start_l4.sh）。

### 1.2 端到端数据流（实测核实）
```
FMS ─MQTT→ fms_agent ─/cloud/task/task_info→ task_plan(读 param/task<id>.yaml)
task_plan ─task_plan_msg(事件)→ path_plan(10Hz: CSV加载+碰撞分级减速+safety)
path_plan ─plan_path_msg/path_plan_status(10Hz)→ control(20Hz: Stanley/R挡几何+×18油门)
control ─control_msg(20Hz)→ can_comm(忙循环聚合) ─can_comm_msg→ canbus(5Hz 发CAN)
ASENSING ─/localization(ivlocmsg,UTM-固定偏移384337.17/3449370.17 硬编码于驱动)→ navigation(50Hz ZOH 重发)
CenterPoint ─/box(MarkerArray 车体系)→ perception_convert ─/perception→ path_plan/control
auto_couple ─/hook_position→ navigation/path_plan(ADAPTIVEHOOK 反算)
```

### 1.3 旧结论修正表（本次实证推翻/细化 workflow.md 08-19 记录）

| 旧结论 | 修正 |
|---|---|
| 负向偏差三重漏判（待修#2） | :579 已用无符号值能停；另两处在死函数内。→ 降级为死代码清理 |
| task_plan 20Hz 无条件重发 task_plan_msg | 事件驱动发布（:289-296）；但每条消息全量重置规划状态仍在（:787-829） |
| 云端停车/暂停对车辆无实际作用 | **实际生效**：path_plan 侧 resetTask(:831-842)/暂停置 desireSpeed=0(:2104-2107) |
| /cloud/msg/command_msg 双类型冲突 | 冲突双方之一 cloud_pub.py 未被 launch（潜伏地雷，非现行故障） |
| refer_path_msg 无人订阅 | simview(view 包)在订，MD5 一致 |
| UpdatePathInfo 80 点窗口≈8m | 80 点×0.21m 实际≈**17m**；≈8m 的是 20 点下发窗口 |
| left_circle.csv 74 行 17.6m 跳变 | 文件尾空行被外部工具误解析为 (0,0)；C 消费者只是末点重复一次 |

---

## 二、子系统分析

### 2.1 task_plan（任务规划，5 文件 1394 行）

**主链**：/cloud/task/task_info → SetTaskInfo(task_plan_comply.cpp:56，车速>0.5 拒绝否则整池重载) → task<id>.yaml 解析(:256-305) → TaskManage 状态机(:350-425，20Hz) 按 path_plan 上报 taskExecuStatus 推进任务块 → PublishTaskPlanMsg(:191-240) 下发路径名/停点/速度/挡位/挂钩指令。10Hz T1 上报心跳（deviceId=A03）。

**任务块类型**：1/2/3/5/7 行驶类→等 TASKFINISHED+停留>20 周期；6 DOACTION→按 subAction（挂钩 epsERR1<185、脱钩>250、等待 isfull）；0 NOTHING 换挡块→立即跳过。

**主要问题**：
- 【实证·高】yaml 解析无异常保护（:268 LoadFile + node.cpp:51 伪造 task_id=1000 必触发 BadFile→回调内 terminate）；param/ 现有 task1-6,8,9,12,14,15,18，**task14/15 引用缺失 CSV 见 §7**
- 【实证·高】末块完成后 PublishTaskPlanMsg 提前 return（:226-232）→ **云端永远收不到整体完成报文**，procedure 停在 2；TaskStatus.fail_code/fail_reason 全程序从未赋值
- 【实证·高】限速钳位单位混用 :207-212（max_vehicle_speed=15 km/h 注释 vs tSpeed m/s → 上限 54km/h）
- 【实证·高】任务无超时/重试：COUNTERPOINT/ASKPALLETPOS/CALCUBACKPATH 落 default 恒 rtn=0 **永久卡块**（:333-334）；WAITING/HOOKOPERATION 条件不满足同样无限等待
- 【实证·中】IsChangeTask 去重被注释（:82）→ 同 id 重发从 block0 重跑（断点续跑失效）；到达判定不校验块号（:340-348），状态滞后>1s 会连续跳块
- 【实证·中】手动模式不阻断状态机（:179-186 原 return 被注释）
- 【实证·中】心跳 ts：hour+=8 无 %24 回卷（每日 8h 窗口生成非法时戳；长度恒 17 字符，非长度抖动）；deviceId 三处不一致（A03/拖A0002/拖A0005）
- 【实证·中】远程指车 UTM 原点硬编码 (238162.61964,3539857.08307)（node.cpp:59-60）与驱动侧偏移 (:384337.17,3449370.17) **两套并存**——目标点整体漂移（该链路当前因 task1000.yaml 缺失先崩，未投入使用）
- 【待核·中】RunningMsgCallBack 伪造 task1000、/cloud/suggestspeed 初值 100 无读者、Warning/Notify 空回调

**死代码**：IsChangeTask、frame_transform.cc 250 行（仅 LatlonToUtmXY 在用）、wait_task 仅死函数内判断。

### 2.2 path_plan（路径规划主链，path_plan_comply.cpp 2832 行）

**主链**：SetTaskPlanData(:767-829，**每条消息**重读 aircraft YAML+全部路径 CSV+复位 mKeypoint/InitSafetyCheck) → PathPlanProcess(:1324) → handleDrivingPath → UpdatePathInfo(:2493，[mKeypoint,+80 点)≈17m 前向窗口单调推进) → LimitSpeedByDistanceToStop(:2323，remain>10m 返回 remain 本身；<0.5m 置 TASKFINISHED) → PublishReferPath(:1438，20 点≥0.3m 下发窗口，不足沿末航向延长 30m；D 挡逐帧矩形碰撞检测 :1688-1899) → PublishPlanPath（加速度低通 0.3/0.7 + 各停车源置 0）。

**碰撞分级减速**：前向 ±90° 过滤(:1013-1023) → 障碍 OBB×路径逐点车体 OBB(3.6×3.4 按托盘) CheckRelation <0.1m 入风险表 → 4 帧去抖 → 方向过滤(4 帧) → dist 级联 25/20/15/10 ÷5/6/7/8 ×0.5+0.5v；**dist<6.5 且 v>1.5 或 dist<4.5 → safety=1**(:1873-1876)，需连续 3 帧确认(:1882-1894)。

**主要问题**：
- 【实证·高】LoadPathFile fopen 无判空(:2273-2275) + **feof/fscanf 末点重复** + **全链无 fclose**（FILE\* 泄漏，~990 次任务消息后 fd 耗尽段错误【gap1 核实】）
- 【实证·高】`task_id==4` 感知旁路(:1417-1422)：每帧清空三路感知=**该任务全程无避障**；同 id 叠加 getLaneLimitSpeedTest 测试残留(:2572-2606，static 首帧锁死+插值被注释+硬编码分界点 6.25,-65.5)
- 【实证·中】handleDrivingPath **四组恒假内层条件**(:846-893，外层 `==X` 内层 `!=X`)：UpdateStopPoint/GenerateLoadPath/GenerateParkPath 永不执行 → taskType 2/3/7 退化为普通 CSV 跟踪（task9 的 park4 停点偏 17.3m 即此症状）
- 【实证·中】碰撞 OBB 朝向失真：未来参考点车体框只更新 x/y，heading 恒用当前值(:1715-1727) → 弯道碰撞距离系统性失真；history_risk_vec_filter **漏写 static**(:1767) 第二级去抖完全失效
- 【实证·中】CheckRelation 仅 4×4 顶点对最小距离(collision_detection.cpp:84-115)，顶点对边构型距离高估 → 减速偏晚
- 【实证·中】safety 带速度条件：4.5~6.5m 且 v≤1.5 时以 dist/8 低通速度**继续蠕动逼近**（对着飞机顶进行为需确认是否有意）
- 【实证·中】AccSwitch=1 时感知回调 return 但**不清空旧数据**（陈旧障碍继续参与判定）；且 ACC 输出 mPlanspeed 无消费者 → accswitch=1 = 关避撞无替代【安全缺口】
- 【实证·中】SetPerceptionData 的快速急停链全无效(:1060-1094)：局部变量+死参数 /canbus/brake（每条 perception 消息白付一次跨进程 param RPC）
- 【实证·中】R 挡后向停车 2.5m 阈值未扣后悬与障碍半尺寸，printf 写 0.2 与实际不符(:1512-1531)
- 【待核·中】每帧 printf 数十条（钩挂帧最坏 ~260 条）；起步观察 30 帧窗初值全 true → 每次起步固定 ~3s 禁止（实现依赖初值，无注释）

**死代码**（详见 §8）：lattice 调用注释块(:1541-1554)、速度规划注释(:1637-1639，planspeed 固定 8.0)、上一帧轨迹复用块(:1556-1631 恒假)、换道/绕行全链（TrajectoryMove/LaneChange/bypass_enable 参数无读者）、SetPerceptionData2/pubObstacles、JudgeTaskPlanMsgChanged、is_take_new_task 恒真保护分支。

### 2.3 休眠规划算法（lattice_plan + robot_path_plan/lattice + speedplan）

三套**编译进二进制但运行期零调用**的算法：

| 模块 | 原理 | 启用条件/阻断项 |
|---|---|---|
| src/lattice_plan（Apollo 裁剪） | Frenet 采样：1 纵向四项式 × 11 横向五次曲线(d∈±3m) → 0.05s×15s 组合 → 顺序累加代价（**名为 DP 实无层间转移**） | 恢复 :1543 注释即可调，但：巡航速度硬编码 30km/h、内部自注"后面的代码有bug"(:2642)、static 跨帧残留(trajectory_cost.cpp:14/:48/:94、trajectory_combiner.cpp:146)、全碰撞时返回 idx=5 已碰轨迹、CalculateMinNumber 强制 left_side_pass 覆盖侧向决策 |
| robot_path_plan/lattice/my_lattice_planner（自研 724 行） | 上一帧裁剪拼接 + end_l∈{0,+2.0} 五次闭式解 + OBB-SAT 筛选 | **CarModel 全仓库无构造点**、obstacles_ 数据源被注释(node:39)、参考线 dkappa 恒 0；缺陷：曲率公式量纲错(:41)、位置残差项多乘 2(:212)、曲率可行性检查被注释(:320) |
| src/speedplan（规则式 431 行） | κ 四段折线限速 + 前后向加速度约束平滑 + BESIDE 障碍 5 段插值 | path_final 需填 κ/length（主流程不填）；FRONT 正前方障碍**只赋值不减速**(:301-311)、vehSpeed 重复 ÷3.6、obsInfo 永不 clear（内存增长） |

**两套 Frenet 符号约定相反**（cartesian_frenet_conversion.cpp:34 用 rθ−θ vs my_lattice_planner.cpp:411 用 θ−rθ）——同时启用会 d' 反号。同名头文件三份（math_utils.h×3、pose2d.h×2、geometry_utils.h×2）。vehicle_param_config.h 车参（宽 1.56/传动比 17/"长城车辆参数"注释）来自**另一车型**。

### 2.4 参考线与决策（reference_line/pathdecision/pathmatch/collisioncheck）

整链休眠（唯一入口 :1543）：QP 样条平滑（OSQP，x/y 独立求解，走廊 0.5m）→ PathMatcher 投影 → LaneChangeDecision（零实例化）。**存活部分只有 CollisionCheck 的几何谓词**（GetRect/CheckRelation/isRectIntersect，被主链碰撞检测使用）。

休眠链内已发现的缺陷（启用前必修）：reference_line.cpp:294-305 复制粘贴错误（end_index 用 it_lower+循环双 i++）、:738 lower_bound end() 越界、:111 单点参考线 uint32 下溢、qp_spline 短段除零/NaN、航向差未归一化、PathMatcher 构造函数**声明无定义**（真实实例化即链接错误——侧面证明从未被构造）、QP 权重硬编码与配置宏全不生效、OSQP workspace 每次 c_malloc 从不释放（恢复后 10Hz 稳定泄漏）。

### 2.5 control（控制，15 文件 3093 行）

**D 挡**：Stanley(φ×0.4+atan(0.4·0.6·e/v)) 0.7 + 纯追踪前馈 0.3 融合（k 曲率自适应被 :29 硬编码 0.3 覆盖）→ LPF 0.8 → slew 8°/周期 → ±22° → **×(-1.1) 增益**(:953)。setParameters(1.6,22,8) 每周期硬编码(:948)。
**R 挡**：独立几何链——预瞄固定 3m、R=(x²+y²)/2y 按 10×横向误差修正、δ=atan2(±1.6,R)、**夹 ±40°（超机械限 22°）**、无滤波无斜率。
**纵向**：delta>0.5 钳 +0.5m/s/周期 → 曲率限速 0.3/sqrt(Σκ/60)（直线→300m/s 即不限）→ **throttle=speed_cmd×18**(:561) → 超速(delta<-1.5) 刹车"渐增"实际每周期清零后只到 1%。

**主要问题**：
- 【实证·高】**InitParameter 从未被调用**（control_node.cpp:74-121 无调用点）→ config.yaml 控制参数体系整体失效：R 挡几何控制器用**默认 wheelbase=2.0**（实车 1.6）、acc PID 全 0、油门标定表空；且 launch 未设 control_para_file，补调用也会 LoadFile("") 崩
- 【实证·高】横向停车 :579-580 阈值 5.5m/4.0m+0.3rad 硬编码（printf 文案写 2.5m 误导；config control_lane_div=1.5 在死函数内）
- 【实证·高·gap3】biaAngle 未归一化 ±180°(:494)：向西行驶时差值≈2π，配合偏差 4.0~5.5m **误触发停车**（低危但真实）
- 【实证·中】超速制动失效：:912 每周期清零 brake + :540-543 渐增只到 1% → delta<-1.5 实际只 1% 刹车+滑行，低速末端靠 safety/TASKFINISHED 兜底
- 【实证·中】PublishMessage 油门/刹车双发覆盖(:604-619)：两者同>0 拆两条消息，canbus 按最后一条覆盖 → 油门 X→0→X 振荡
- 【实证·中】红灯停车两组硬编码地理框(:398-418，x∈1553-1559/425-432+heading 80-100)——**全部 77 个路径 CSV 的 |x|≤348.6，条件恒假=死分支**（前站残留）；/camera/tl_status 无发布者
- 【实证·中】R 挡油门=mSpeed×10 用原始 mSpeed 不响应减速链；GNSS 故障刹 100 时 R 挡仍算非零油门（靠双发清零巧合兜住）
- 【待核·中】lat_controller.cpp:59-62 sign=fabs(cross)/cross 压线时 **NaN** 直达发布；getNearestIndex 首个局部极小即 break——回折/闭环路径投影错段（left_circle 首尾距 0.32m 必触发）
- 【实证·中】CSpline 样条 `new float[]` 每帧泄漏(Spline.cpp:13-15)；mPathid==20 与 else 分支逐行相同(:169-197)

**负向偏差三处定论**（详见 §4.2）：主判断已无符号化能停；:738/:740 重复行+BiaDisLimitSpeed 未取 fabs 确在，但位于零调用 SpeedJudge 内；BiaAngleLimitSpeed :684-688 数组错读（激活即恒返 0）。

**死代码**：SpeedJudge/BiaAngleLimitSpeed/BiaDisLimitSpeed、lateral_control.cpp 全文件（模糊 PID 横向 5×5 表）、SpeedTrack0/RATIO=9、mPlanspeed（ACC 死端）、isWithinFence（空函数体 UB）、552-571 旧油门公式。

### 2.6 navigation / can_comm / trans（定位预处理与 CAN 桥）

**navigation**：驱动侧已完成 UTM 局部化（ASENSING_INS_node.cpp:361-365 减固定偏移，**proj4 全工程零调用=死依赖**），本节点仅整形+50Hz ZOH 重发。挂车模式状态机：ADAPTIVEHOOK→HookEnable=1 改用挂点极坐标反算（**PalletType==0 出厂默认时又被全局定位覆盖**——语义反直觉）。
问题【待核·中】：断流后 50Hz 重发陈旧位姿（无 header/时间戳，下游无法判龄）；启动初期发全 0 位姿；navigation_msg 的 longitudinal_accelerate/lateral_accelerate/faultCode/localization_status **全链无生产者**（task_plan 心跳却消费 longitudinal_accelerate 判加减速→云端永远收"keep"）；rtkState 硬编码 "no_fixed"；经纬度 float32 承载 float64（~0.8m 量化，埋雷）；launch 5 参数（serial_port/refer_lon.../save_path）**全部零读取**（旧 Gps2plane 躯壳）。

**can_comm**：纯指令聚合器（挡位/挂钩←task，纵向/转向←control，声光←sound_light 恒零值消息）。问题【实证·高】：**忙循环**（无 sleep，kHz 级发布+每圈 param RPC）；**油门斜率恒不生效**（:51 先赋值 :62 再比较，差恒 0）；vehicle_wheel_co 缺参时 ratio=0.0 静默清零转向（单独启动 robot_canbus.launch 即触发）；无任何新鲜度判据（配合 §0.2 冻结油门问题）。

**trans**：CSV→ReferenceLine 装配，唯一调用在死链 LatticePlan 内。

### 2.7 perception_msg_convert（感知转换）

/box(车体系 CUBE+TEXT) → θ=90−heading 旋转到地图系 → CUBE 过滤(:123-126，**已在源码、部署机二进制为 ros1_hub 旧版未含——待重编**，与 workflow.md 待办一致) → 感知边界过滤（21 点多边形=浦东站覆盖区，实测与 0517/051501 路线 95-100% 重合；**注释写"局部坐标系"是错的，实为地图系**）→ /perception。

问题：
- 【实证·高】**障碍粘滞**：空 MarkerArray 早退不发布(:105-108) → 障碍消失后 path_plan 的 mPerception.objs 保留旧位置；且实编的 centerpoint_ros.cc 在雷达异常时 `ShutdownPublisher()` 彻底停发——双重粘滞源
- 【实证·高】边界文件路径写死部署机绝对路径 /home/nvidia/...(:162)：加载失败仅 WARN 后**无过滤模式**（换机/换站即致盲或全放行）
- 【高·gap3 数学推导，待复核】障碍航向公式 :144 `fmod(yaw+heading+90,360)` 与位置旋转(θ=90−h, CCW)不自洽，正确式应为 h−ψ——ψ=0 时框朝向偏 90°，细长障碍碰撞盒朝向错误（近方形障碍影响小）
- 【实证·中】契约字段缺失：vx/vy 恒 0（ACC 把所有障碍当静止）、id 恒 0、header.stamp 恒 0（帧龄不可审计）、置信度丢弃；dx/dy 注释与实际相反（链路自洽但误导）
- 【待核·中】无定位新鲜度检查：/localization 断流时用陈旧位姿投影（运动时障碍位置系统性偏移≈车速×龄）；启动初期 mGPS 全 0 → 障碍投到地图原点
- perception_msg_convert1.cpp 未编译（死文件，误入即重复定义 main）

### 2.8 common / msg / 构建

- 活跃公共库仅 2 个自研库：pubalgor（几何/字节序/Bezier，含**硬编码场地高程 z=38.79m**）与 spline（CSpline；delete 而非 delete[] 形式 UB、短路径 sd[-1] 越界写）
- **三方库全死链**：osqp（唯一调用链尽头是 :1543 注释）、qpoases（唯一引用方 17 个未编译的 smooth_spline 文件）、proj4（只 include 零调用）、Eigen（头文件级）、jsoncpp（零 Json:: 调用）、lib/ 5 个预编译 .so 零链接、serial 依赖零使用
- ARM 分支配置隐患：qpoases GLOB 同时匹配 AArch64 与 x86-64 归档；proj4 点名 x86 归档而同目录 AArch64 版被忽略（靠零引用侥幸）
- msg：44 个生成，零引用 3 个（task_plan_status/vehicle_task_status/perception_msg）；hook_position.msg 重复列出两次；ivmsglocpos.h **手抄头三份**（MD5 当前与 ivlocmsg 一致=同步地雷）；pnc 与 canbus 双份维护 can_msg/can_comm_msg（现役同步正常），canbus 侧 hook_position.msg 落后 2 字段（无使用者）
- CMakeLists：GBK 编码（grep 需 -a）；GLOB SRC_FILES 死变量；ARCH_ARM/X86 宏零引用；control_comply 库把两个 .h 当源文件

---

## 三、接口契约核查（14 节点全图）

### 3.1 跨包 MD5 实测（用 ros1_hub devel 生成头逐条核对）

| 话题 | 双方 | 结果 |
|---|---|---|
| /can_msg | robot↔canbus | ✅ 一致 (f00c5ecb) |
| /can_comm_msg | robot↔canbus | ✅ 一致 (659b83be) |
| /hook_position | robot↔auto_couple(center_position) | ✅ 一致 (9ff0c300)，但**名字不同靠字段巧合**，任一方改动即静默断链 |
| /cloud/task/task_info 等 5 组 | robot↔fms_agent | ✅ 一致 |
| ivmsglocpos | 手抄头×3↔ivlocmsg | ✅ 一致 (0ccfe435)，同步地雷 |
| /cloud/msg/command_msg | fms CommandMsg vs robot v2nCommandFeedback | ❌ 不兼容——但冲突方 cloud_pub.py 未被 launch（潜伏） |
| hook_position | canbus 侧副本 | ❌ 落后 2 字段（canbus 无使用，陈旧副本） |

### 3.2 孤儿话题（收无人发/发无人收）

**收无人发**：/palletpos、/scan_bbox_result（**前向扫描链整体断裂**：lidar_perception 发的是 /perception_front_bbox 又无人订——双重孤儿）、/robot/serial/rs232/（急停解除入口死）、/baselink_in_trailer、/camera/tl_status、/cloud/msg/airport_msg（飞机位让行 :2439-2447 永不触发）、/cloud/msg/running_msg|warning_msg|notify_msg（发布者未 launch）。
**发无人收**：acc（control 20Hz 调试）、test_trajs、planning/obstacles（publisher 建立后发布调用被注释）、/sampled_path、/optimal_path（类未实例化）。
**rosparam 死通道**：/cloud/suggestspeed（写无读者）、/canbus/brake、/canbus/horn、/sound/play（写无读者）；/robot/lanechangecmd、/robot/speed、/robot/control/accswitch（读无写者）；**/robot/planning/netcheck 唯一写入者 netcheck.sh 在 start_l4.sh:24-25 被注释——断网停车保护实际关闭**。

### 3.3 其余契约问题（节选，全 40 条见工作流日志）

- 【实证·高】sound_light_msg 恒零值（mSoundLightData 无任何赋值点）→ can_comm 灯光使能恒 0，实际灯光走 /canbus/light 参数——两条并行声光通道一条死一条半死
- 【实证·高】navigation_msg.longitudinal_accelerate 无生产者被心跳消费（云端加减速状态上报失效）
- 【实证·中】队列语义不一致：path_plan 订 navigation_msg 用 q10（50Hz 输入+10Hz 循环，每周期空转 4 次覆盖）而 control 用 q1
- 【实证·中】/perception 感知契约退化：只填 x/y/dx/dy/heading 五字段

---

## 四、安全链全景

### 4.1 23 条停车/减速/急停链（触发→效果→端到端时延）

| # | 链 | 触发 | 效果 | 时延 |
|---|---|---|---|---|
| C1 | 前向碰撞分级减速 | 距离级联 25/20/15/10m | dist/N×0.5+0.5v | ~0.55s |
| C2 | 前向碰撞停车 | dist<6.5(v>1.5)或<4.5，**3 帧去抖** | safety=1→brake100 | ~0.7s |
| C3 | R 挡后向停车 | backdist<2.5m（无去抖） | brake100 | ~0.45s |
| C4 | 起步观察 | 四周<0.95m/挂盘<1.45m，30 帧窗 | 禁起步+灯 7 | ~3s 窗 |
| C5 | CAN 急停 | emergencyStop **latch** | 刹 100（**无解除**） | ~0.4s |
| C6 | GNSS 丢失 | 定位静默>2.0s（bit3） | 刹 100 | ~2.4s |
| C7 | 雷达/相机故障 | 感知静默>3.0s / cam0/7 掉线 | 缓停 dcc=8 | ~3.4s（盲行~12m） |
| C8 | 托盘脱落 | hookstate==1 | 缓停 | ~0.5s |
| C9 | 电子围栏 | 前角出 fence.csv 多边形 | 刹 70 | ~0.3s |
| C10 | 横向偏差停车 | biaDistance>5.5 或 >4.0+0.3rad | 刹 70 | ~0.3s |
| C11 | 终点限速/停车 | remain<0.5m→FINISHED | 渐停 | ~0.35s |
| C12 | 任务完成/N 挡/手动 | taskExecuStatus==2 且 |v|<0.5 | 刹 80 | ~0.35s |
| C13 | 空路径 | mPathList 空 | 刹 50 | ~0.25s |
| C14 | 云端命令 | commandState 0/1 | resetTask/暂停 | ~0.35s（**生效**，修正旧结论） |
| C15 | 断网 | /robot/planning/netcheck | 弱刹（**实际关闭**：写入者被注释） | — |
| C16 | 规划心跳丢失 | canbus 侧 0.5s 静默 | 刹 70 | ~0.7s（**只覆盖 path_plan，不覆盖 control/can_comm**） |
| C17 | 信号灯 | 硬编码地理框（**恒假=死链**，tl_status 无发布者） | 刹 30/50 | — |
| C18 | 曲率限速 | Σκ 前 60 点 | v=min(v,0.3/√) | 同控制周期 |
| C19 | 指令斜坡 | 加速+0.5m/s/周期、转向 slew 8° | — | 20Hz |
| C20 | 感知近距 brake 参数 | dist<5 | **死链**（/canbus/brake 无读者） | — |
| C21 | ACC/AEB | accswitch | **无效**：感知链被跳过且输出无消费者 | — |
| C22 | 定位预处理 | 挂车模式反算 | — | 50Hz |
| C23 | 感知转换 | /box→/perception | —（空帧粘滞见 §2.7） | — |

### 4.2 负向偏差三处定论（修正 workflow.md 待办#2）

1. **:579-580 停车判断**：biaDistance 唯一有效赋值在 :493-495（`distanceTo`=hypot **恒≥0** 的垂足投影距离），BiaAngleCalculate :871-874 的带符号值每周期被覆盖（死计算）→ **左右偏差均触发，漏判不存在**。残留真问题：biaAngle=fabs(heading差) 未归一化 ±180°（§2.5）。
2. **:737-741 限速重复行**：`:738-739` 与 `:740-741` 原样重复且未取 fabs——确认存在，但整个 SpeedJudge(:712-751) 全工程零调用（仅声明 control_comply.h:104）→ **死代码，运行时零效果**。
3. **BiaDisLimitSpeed(:693-710) 未取 fabs**：负偏差被 FuzzyDataProcess 钳到断点 0 → 返回最高限速（语义=负向不限速），但唯一调用点 SpeedJudge:732 死代码 → **不可达**。附带：BiaAngleLimitSpeed(:674-691) 数组错读（speed_angle_limitX 读进 angle[] 且 speed[] 恒 0，激活即恒返 0 限速）。

**结论**：原"安全修复"降级为"死函数清理决策"（删除或修复后接入）。真正值得做的是把 :579 阈值参数化（printf 文案同步改）+ biaAngle 归一化。

### 4.3 安全缺口（12 项，按严重度）

- 【高】**控制指令断流无检测**（冻结油门，§0.2）：canbus OnControlCommand 只缓存无时间戳；pnc 无 control 输出心跳；C16 只看 path_plan
- 【高】accswitch=1 = 关闭避撞且无替代（ACC 开环）
- 【高】emergencyStop latch 无解除 + path_plan 重启互锁（§0.4）
- 【中】感知漏检无冗余（唯一来源 /perception；侧后目标不进碰撞检测，仅 R 挡补后向）
- 【中】定位跳变无检测（navigation_msg 无突变阈值；navigation_node 死亡时 GNSS 看门狗监听其**输入**而非输出→不触发；唯一兜底 5.5m 偏差停车）
- 【中】规划超时无逐帧监控（LoadPathFile 同步 IO 阻塞只靠 canbus 0.5s 兜底）；control 无 plan_path_msg 新鲜度检查（path_plan 停发时 control 沿旧路径行驶）
- 【中】减速方向无斜坡+弱停（desireSpeed=0 无 safety 标志时仅 throttle=0+brake≤5%，下坡/惯量可能停不住）
- 【中】路径突变无连续性检查（新路径直线连接段 1~2.5m 无障碍复核；task_plan 层 >0.5m/s 禁换任务保护在 path_plan 层无对应）
- 【中】task_plan 层速度保护单侧（path_plan 只打印不拒绝）
- 【低】信号灯/等待区/T 路口坐标硬编码且部分恒假；can_msg 断流 pnc 侧无检测（mVehicleSpeed 冻结继续用）；fence 加载失败即崩（§0.3）

---

## 五、时序与时延预算

**实测频率**：task_plan 20Hz / path_plan 10Hz（帧内最坏 20-40ms）/ control 20Hz / navigation 50Hz（先 publish 后 spinOnce，数据固定滞后 1 拍）/ can_comm **忙循环 kHz 级** / canbus 驱动帧 **5Hz**。

**端到端**（15km/h=4.17m/s）：/box→perception_convert→path_plan(≤100ms)→safety 3 帧去抖(300ms)→control(≤50ms)→can_comm→canbus 5Hz(≤200ms)：典型 **~0.35s(1.5m)**，最坏 **~0.85s(3.5m)**（不含 CenterPoint 检测与执行器）。

**关键量化**：
- 驱动 CAN 5Hz：20Hz 控制指令 **75% 被丢弃**，紧急刹车序列被抽稀为 5Hz 阶跃（canbus_node.cpp:222-223 定时器+queue=1 锁存）
- param RPC 风暴：control 每周期 **12 次**、path_plan 9-13 次、navigation 50 次/s、can_comm 无界 → 全车稳态 ≥400-500 RPC/s 打 roscore，can_comm 忙循环进一步放大延迟
- path_plan 单线程最坏 spinOnce≈30-60ms（SetTaskPlanData 重读 YAML+CSV 与 ≤5 个 navigation+≤4 个 perception 回调同帧排队）——**10Hz 预算被占 30-60%**
- ADAPTIVEHOOK 任务每 10Hz 帧全量重建路径+最坏 ~260 条 printf
- **全链消息零时间戳**（在用 msg 仅 perception.msg 声明 header 且从不填充）：数据龄不可测、convert 用最新位姿转换任意旧 box（运动时障碍偏移≈车速×龄）
- printf 无节流：path_plan 普通帧 55-60 条、control 12-20 条

---

## 六、状态机与故障恢复

**三台状态机**：task_plan 任务块（11 类块型，出口依赖 path_plan 上报+epsERR1/isfull）→ path_plan taskExecuStatus {0/1/2}（TASKFINISHED 是吸收态，NOTASK 转移被注释 :1387）→ control 无显式状态机（消费型）。

**无出口卡死态**：COUNTERPOINT/ASKPALLETPOS/CALCUBACKPATH 落 default 恒 0；HOOKOPERATION 机械停在 185-250 死区（三方阈值还各不相同：task_plan >250 / path_plan >245 / canbus 240）；WAITING 无 remote_signal 永久等；taskExecuStatus==1 且到达条件永不满足（无看门狗）。

**kill -9 重启影响矩阵**（要点）：
| 节点 | 即时后果 | 恢复 |
|---|---|---|
| task_plan | 车继续跑当前块直至自行 FINISHED（失控窗口=剩余路程）；心跳断 | **无状态查询 API**，只能云端整单重发且从 block0 重跑 |
| path_plan | canbus 0.5s 后刹 70/挂 N+EPB（安全）；control 仍用冻结路径算转向 | **与 task_plan 发布门控互锁死锁**（NOTASK 永不触发重发） |
| control | **can_comm 缓存+忙循环无限重发冻结指令，canbus 5Hz 继续执行——最危险组合** | InitParameter 不被调用问题在重启后同样存在 |
| can_comm | 同上（canbus cmd_ 冻结） | 重启即恢复（20Hz 内） |
| navigation | 冻结位姿继续控制，GNSS 看门狗不触发（监听输入），兜底只剩 5.5m 偏差停车 | 重启初期再发全 0 位姿 |
| perception_convert | mPerception 冻结（旧障碍永久保留），3s 后缓停 | mGPS 全 0 窗口 |

**冷启动危险窗口**：navigation 全 0 位姿（任务先到则 calcuGlobalPath 用 (0,0) 定位）；convert mGPS 全 0（障碍投原点）；IsGreenLight 前 5 帧必非绿；起步观察 30 帧全 true 固定 ~3s 禁止；canbus startupAge≤10s 监督盲窗+batteryPower 默认 80；task_plan 启动强写 suggestspeed=100（0.1s 窗口）。

---

## 七、数据资产审计（77 CSV + task yaml + 围栏/边界）

- **【实证·高】task14/task15 的 task0（type=1 循迹）引用不存在的 aircraft2.csv/aircraft3.csv** → FMS 派发即 path_plan 段错误；task2/task14 引用的 pathwuxi003.csv 同缺（type=6 不加载，隐形地雷）
- **【实证·高】fence.csv（11 点）不覆盖在用的 path_pudong0517**（task3/task4/task4_lane_speed_limit）：452-642 行 191 点越东南斜边 0.1~1.6m → 前角出栏即刹 70 且无恢复，**任务必然中途刹停**；旧站资产（path04302/path_zhg/path0506001）大段在栏外，误派发即围栏刹停
- **曲率可行性**：最小转弯半径下限 1.6/tan22°=**3.96m**；实测 left_circle R≈2.34、park_back 2.36、go_start 2.52、path_pudong0517 2.71、park 族 2.78~3.93、path0506001/2 ≈2.4 —— **倒车入库族与环线在 22° 转向限内不可完整跟踪**（实车表现=切弯→偏差增大→5.5m 停车）；达标：aircraft_back 5.45、051501 5.98、sx040901 4.06、track 4.27
- heading 列大面积未填（aircraft_go/back、051501 中段=0）：当前唯一活消费是末端延伸取 size-2 行（尾行已填，未损坏）；**重启 lattice 或新增航向判断立即得 0° 假航向**
- **heading 列双语义**：正驶=行进方向，倒车族（park\*/park_hook/path_hook1）=车头朝向（差≈180°），仅凭 yaml gear=3 隐式约定，无文件头说明
- **坐标系统一性**：唯一活平面=UTM51N−(238162.61964,3539857.08307)（task_plan 远程指车链）；**实际定位用的偏移在驱动侧 (384337.17,3449370.17)**——两组数字不同源（一个经 LatlonToUtmXY 动态算，一个驱动写死），gis_utils 还藏第三套死原点 BASE_GAUSS。task5/6 停点与 aircraft_parking_port.yaml 三 port 距 0.01-0.09m（同源验证 ✓）
- **单位对账**：速度全链 m/s（yaml 注释"#3km/h"全部失实；2 实为 7.2km/h）；心跳上报 ×3.6 转 km/h；角度双轨（speed_angle0..4=度 vs biaAngle=rad）；转向传动比 22（死代码 RATIO=9）
- yaml 寻址固定拼接 "task{id}.yaml"：param/1/task1.yaml（新）不会覆盖 param/task1.yaml（旧）——FMS 发 task1 走旧内容；5 个带后缀副本纯靠手工 cp 切换，极易切错
- 数据毛刺：park_hook/path_hook1 各 9 行同点重复（人为停顿编码）；aircraft_go 末步 0.777m；051501 有 15 处短步会被 control 过滤丢弃
- forbiddenarea.csv（2 点无法成域）与 fence0430.csv（旧站围栏）零读者

---

## 八、死代码与依赖盘点

**休眠主链**（编译进二进制、运行期零调用）：
- lattice 整链（:1543 注释）：lattice_plan 12 文件 + reference_line 9 文件 + pathmatch + pathdecision + collisioncheck 死部分 + trans + common 的 Apollo 子库（math/surface/path/curve1d/line/polynomial_xd/qp_spline_corridor）+ **OSQP 唯一调用链**
- 速度规划（:1637 注释）：speedplan 2 文件
- my_lattice_planner + collision_check_with_bbox（CMake 显式编译，零调用；残留调试路径 /home/cqw/...）
- 换道/绕行全链（TrajectoryMove/LaneChange/bypass_enable/mOriginPath/mVirtualPath/enableBypass 字段恒 0）
- control：SpeedJudge/BiaAngleLimitSpeed/BiaDisLimitSpeed、lateral_control.cpp 全文件（模糊 PID）、SpeedTrack0、mPlanspeed（ACC）、isWithinFence、InitParameter（**失效的参数体系**）
- can_comm 油门斜率（恒假分支）；task_plan IsChangeTask；frame_transform 250 行；SetPerceptionData2/pubObstacles/pubLatticeTrajs
- 未编译文件 21 个：smooth_spline 17 + piecewise_jerk 2 + speed_data + perception_msg_convert1

**死依赖**：osqp/qpoases/proj4/jsoncpp/Eigen(有效算法路径)/lib/ 5 个 .so/serial 依赖/ARCH 宏——全部可从链接行移除（保留 include 不影响）。

**死消息**：生成后零引用 3 个、未列入 3 个、msg/bak/ 10 个、hook_position.msg 重复列出、canbus 侧陈旧副本 1 个。

---

## 九、硬编码 Top 清单（42 项全表见工作流日志，此处行为关键项）

| 值 | 位置 | 影响 |
|---|---|---|
| 油门×18 / R 挡×10 上限 45 | control_comply.cpp:561/:597 | 覆盖全部闭环控制 |
| 停车阈值 5.5/4.0/0.3 | :579-580 | 绕过 config control_lane_div |
| Stanley(1.6,22,8)+增益 -1.1 | :948/:953 | 无标定依据；-1.1 反向增益经验补丁 |
| R 挡限幅 ±40° | geometric_control.cpp:258 | 超机械限 22° 且未乘传动比 |
| 曲率限速 0.3/√(Σκ/60) | :626/:664 | config 六档曲率表零引用 |
| 碰撞盒 3.6×3.4（按托盘） | path_plan_comply.cpp:1692 | 车宽 1.47 却按 3.4 算；:1036 又用 2.0——三套口径 |
| 减速分级表 25/20/15/10/6.5/4.5 | :1865-1876 | 全魔法数 |
| task_id==4 感知旁路 / task_id 1000 | :1417/:51 | 测试后门 |
| 红绿灯两框 1553-1559/425-432 | control:398-418 | **恒假死分支**（前站坐标） |
| T 路口/等待区多边形/分界点(6.25,-65.5)/装载点(-33.96,-24.92) | path_plan:2151/2247/2577/657 | 场地专属，换站静默失效 |
| 感知边界绝对路径 /home/nvidia/... | perception_msg_convert.cpp:162 | 跨机部署即失效 |
| UTM 原点两套 | task_plan_node.cpp:59-60 / 驱动:361 | 同上 |
| deviceId A03/拖A0002/拖A0005 | node.cpp:163,221,230 | 云端路由错车 |
| epsERR1 185/250/245 | 三处不一致 | 240-250 死区行为随节点不同 |
| 高程 z=38.79 | pubalgor.cpp:335+ | 场地魔法数 |
| config.yaml 149 行中约 45 键零消费 | — | 参数体系名存实亡 |

---

## 十、问题统计

| 来源 | 高 | 中 | 低 | 实证情况 |
|---|---|---|---|---|
| 8 深读 agent | 33 | 74 | 37 | 高/中去重 107 条，25 条对抗实证（21 CONFIRMED / 4 PARTIAL），82 条待核 |
| 3 补漏 agent | 16 | 26 | 9 | 代码核实（未走独立对抗复核） |

4 条 PARTIAL 的更正已并入上文相应条目（云端命令实际生效 / ts 长度恒 17 / command_msg 无并存冲突 / task_plan 事件驱动发布）。

---

## 十一、修复优先级建议

**P0 安全（建议实车前处理）**
1. 指令通道新鲜度：canbus OnControlCommand 加时间戳老化(>0.5s 刹车) + can_comm_node 补 sleep 修忙循环（同一改动同时解决冻结油门+param 风暴）
2. 输入健壮性四件套：LoadPathFile fopen 判空+fclose、ParseTaskFile try/catch、task14/15 yaml 修复或下线、CalcuPathCurve at(size-13) 边界
3. emergencyStop 解除路径（rs232 话题接入或面板复位）+ path_plan 重启互锁（task_plan 改持续重发当前块）
4. task_id==4 感知旁路移除（改 yaml 显式开关）；fence.csv 扩栏覆盖 0517 或任务回栏内
5. CUBE 过滤重编部署（已在 workflow.md 待办#3，顺带解决 TEXT 幻影）

**P1 正确性**
6. max_vehicle_speed 单位修正（×3.6 或注释改 m/s 并给 4.17）
7. /perception 空帧发布清空（障碍粘滞）
8. can_comm 油门斜率"先赋值后比较"修复；评估 control 20Hz→canbus 直发（消除 5Hz 抽稀）
9. turning 左右互换、deviceId 统一、biaAngle 归一化、biaDistance 阈值参数化
10. InitParameter 接入决策（接入=行为变化需实车标定 wheelbase/油门表；不接入=删除并接受 ×18）
11. 感知障碍航向公式复核（h−ψ vs h+ψ+90，数学推导指向错误）

**P2 债务**
12. 死代码清理决策：lattice 家族/speedplan/SpeedJudge/模糊 PID/死依赖（osqp/qpoases/proj4/jsoncpp/lib 5 .so）——建议单独一次"只删不改编译验证"
13. 硬编码收敛（红灯坐标删除、T 路口/等待区/分界点进 config、边界文件路径参数化）
14. ACC 开关语义（accswitch=1 关避撞——加互锁或补闭环）
15. 曲率不可行路径处置（left_circle/park 族重新测绘或改多步挪车）

**P3 卫生**
16. msg/CMake 清理、printf 节流（ROS_DEBUG）、rosparam IPC 收敛为话题、消息加 header.stamp

---

*附：完整 195 条问题明细、23 链安全表全文、42 项硬编码表、14 节点话题矩阵存于工作流产出（journal），必要时可重新生成。*
