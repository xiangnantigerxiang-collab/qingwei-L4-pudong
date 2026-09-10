# workflow.md — 工作延续文档（基线版 2026-09-05）

> 开工先读。**基线声明：本树=实车部署版（用户 09-05 确认运行未发现问题），此后一切修改以此为准。**
> **09-10 基线重置（用户宣言「忘掉过去，今后在这个版本的基础上改动」）：pnc 以本树现态为新基底**（=09-05 基线 + 感知排除区重写 + config 两 csv + 312 任务族 yaml；**09-08~09-09 pnc 轮整体作废**）。pnc 快照 `../pnc_userbase_20260910.tar.gz`（9.7M/1630 文件）。
> 基线快照 `../baseline_vehicle_deployed_20260905.tar.gz`（838 文件）；6 路深读原始报告 `../baseline_sweep_reports_20260905.md`（file:line 细节以此为准）。
> 完成工作在「日志」追加结论级条目，同步「现状」「待办」。非 git，回滚靠 `../` 快照。本机无 ROS：C++ 桩编译验证（方法见「设施」）。

## 一、工程速览

机场 L4 无人电动牵引车整车软件（浦东机场，云端 FMS→循迹→自动挂/脱托盘）。车载 Jetson Orin（aarch64，ROS1+Py3.8，`/home/nvidia/qingwei-L4-No2`），本目录为其重建副本。

数据流：
- FMS ─MQTT(39.105.47.169:1883)→ fms_agent(8 节点,VehicleId=A03) → /cloud/... → pnc（task_plan 20Hz → path_plan 10Hz → control 20Hz → can_comm → canbus）
- 5×RoboSense → 拼接 → CenterPoint(TensorRT) → /box → perception_convert(100Hz) → /perception → pnc
- 2×LakiBeam → auto_couple → /hook_position（倒车对挂）；ASENSING RTK/IMU → /localization → navigation(50Hz) → /navigation_msg
- canbus：socketcan can0 250k（socketcan_bridge 桥 /can_send、/can_recv），发 0x184/0x284，收 0x185/0x285/0x0C02A0A2
- 入口两条：start_l4.sh（12 终端串行，救急；第 10 步 simview 指向工程外 /home/nvidia/zyd/0522）或 HMI（14 组件 5 分组，日常）

关键事实：纯路径跟踪（lattice/速度规划休眠）；throttle=speed_cmd×18（R 挡 ×10 上限 45）；路径=预录 CSV（pnc/path/，launch 参数 path_dir）；安全兜底=sensorstate 故障位缓停/急停、fence.csv 围栏、横向偏差>5.5m 停、急停闩锁；车辆 1680kg/轴距 1.6m/15km/h/传动比 22/限角 ±22°；CenterPoint 须在其 build/ 启动（模型 `../model/`）。

包：robot(pnc 6 节点,45 消息)｜CUDA-CenterPoint(独立 cmake,非 catkin)｜driver(rslidar/lakibeam/ASENSING/cam_geac/lidar_perception)｜canbus(18 消息)｜fms_agent(24 消息+Python,env/ venv 必需)｜auto_couple(GBK)｜simview(包名 view)｜ivlocmsg（必留）。

关键文件：pnc 主链=path_plan_comply.cpp(stub+5 .inc)+task_plan_core.{h,cpp}+control_comply.cpp；canbus=canbus_comply.{h,cpp}+canbus_node.cpp；**can_msg/can_comm_msg 三份 .msg（canbus/pnc/simview）必须同步改**（md5 变→所有订阅节点整体重编）。

## 二、现状（2026-09-10 pnc 基线重置）

- **canbus（用户自研版）**：T1 10Hz=EmergencyStop 汇总（sensorstate≠0 ∨ netcheck≠0 ∨ !planning_alive ∨ 指令老化>1.0s）+ hook/pallet 状态机（±5 带 >1s：4=up/3=down；中段 1s 不动→1=block；pallet 无 block；2 永不产出）+发布 /ehb_msg（仅此一者，10Hz）；T2 20Hz=VehicleComm+发布 /can_msg（09-07 晚用户自 T1 移回，恢复基线 20Hz；曾 20→10 引发 HMI 健康误报，hmi_config min_hz 已复位 20）；**0x284 byte2 四分支顺序覆盖**（:90 0x05 需 cmd=5∧status∉{1,4}；:95 0x0A 需 cmd=6∧status≠3；:100 0x01 需 status=4∧pallet≠3；:101 0x02 需 status=3∧pallet≠4；后写覆盖，完整真值表见 src/canbus/README.md）——语义=挂/脱钩两相续接+机构自收敛（:100/:101 不受 hookCmd 门控）；限值 config.cfg（182/254/213/254）成对校验→rosparam 四键；速度=高压公式 25.8/0.66（实车未发现问题）；**EHB**：docs/ehb-can.md 为唯一协议源，九帧旁听/接收解析（EHB 发 3 帧+VCU→EHB 6 帧）入 mEHBMsg，T1 10Hz 发 /ehb_msg（详见 src/canbus/README.md）
- **pnc**：判据统一 hookStatus（完成 output.inc:443/:452、平滑 :79、task_plan OperationStateJudge、心跳映射 4/3 直传 1→2）；task_plan 20Hz+心跳 10Hz（块推进去抖：DOACTION 2.5s/行驶 1s；/task_plan_msg 事件驱动发布）；path_plan 10Hz（T1 写 sensorstate：lidar 断>3s+2/相机断>2s+4/GNSS 断>2s+8/任一故障再+1，且 alive=1）；SetTaskPlanData 每条消息全量重载（复位仍限真实切换）；**perception_convert=排除区新架构（09-10 基线）**：LoadBoundary 扫 config/ 全部 .csv 各成一多边形、IsPointInExclusion 命中即剔（与旧包容边界「内=放行」语义整体反转），config/ 现=charge.csv（44 点走廊）+trajectory_optimized.csv（21 点闭环轨迹草稿，**会被误当排除区，风险 9**）；WAITING 子动作（subaction 7，isfull==1 放行）定义完备但现役 yaml 零使用
- **ultra_command（09-10 新包，晚二轮对抗校验后加固）**：312 任务族监控矩形障碍物检测——/task_plan_msg.pathList 精确匹配 pudong_air/312_316_01 或 312_cargo_01（监控 left1+left2）/312_charge_01（监控 right）激活，/perception 障碍物**中心点**射线法判落入→10Hz ZOH 写 `/ultra/status/safe` 1/0；**订阅 /cloud/msg/command_msg，commandState==0 即退监控**（task_plan 停车分支只 clearTaskPool 无终态发布，不订则闩死）；矩形=pnc/path/pudong_air/{left1,left2,right}.csv（全局 path_dir 每周期热读+失败重试，**不用目录 glob**，nan/inf 坏行 isfinite 跳过）；**感知静默>1.0s 按 0**（空场景/全被排除区过滤时 /perception 静默或空 objs，无此老化则 safe=1 永久粘滞；感知死亡错报 0 的 ~2s 窗与 sensorstate 3.0s 兜底不对齐——fail-safe 方向待用户定）；非目标任务/待命/停止后不写参数保持最后值；02/03 变体不触发；CMake -std=c++11+add_dependencies(消息生成)。本机纯逻辑单测 10 组+桩编译过；**待用户决策**：启动接入（唯一阻塞实车生效）、矩形与作业走廊重叠语义（实测任务路径自身穿矩形，挂靠目标若在区内则作业段 safe 恒 1）、消费方契约（全仓零读者）、去抖、pnc 侧 /task_plan_msg latch 建议（根治中途启动失明分钟级窗口）
- **契约**：hookStatus=HOOKSTATUS_E 两侧锁定；can_msg 三包逐字节一致、can_comm_msg 仅注释差（md5 同）；fms↔pnc 现役 TaskInfo/TaskStatus/v2nCommandFeedback 一致；CommandMsg/v2nHeartBeatValue 副本漂移但死定义；/hook_position pnc↔auto_couple 7 字段 md5 巧合一致（改字段即断）；struct_type 两份仅差无读者的 GEAR_P
- **已下线（勿恢复）**：一键标定、堵转保护、相机伸缩杆、0x201 蜂鸣、CAN 断流+faultCode 置 1、path_plan_status 订阅+/planning/alive 清零、ADAPTIVEHOOK 循迹 0x0A、param hookstate、control 脱钩缓停、simview(rviz)
- **死通道群**（写而无读者）：/canbus/horn、/canbus/brake、/sound/play、/cloud/suggestspeed、/acc（/canbus/time 唯一写者已由用户 09-08 晨删除，通道全灭）；无发布者：/camera/tl_status、/palletpos、/cloud/msg/{running,warning,notify,airport}_msg；accswitch 无写者（ACC 分支死，其输出也不回写 mControlData）
- **待用户决策**：HMI 标定卡片孤儿（超时报失败）；位置限值管道（pnc 每圈热读但读值零消费——task_plan mPositionLimits 无读者、path_plan HookPos* 零引用）；rebuild_all.sh 缺陷修复（风险 14）
- **已验证**：车载编译+部署+运行 OK（09-05 用户确认）；Firefox WebGL 三开关已解；geometry_utils inline 修复有效

## 三、关键风险与边界（现行已知未修；细节与 file:line 见归档报告）

1. **控制双发拆包**：control 油门刹车同>0 时拆两条消息（throttle-only 先、brake-only 后），canbus 整包缓存取最后一条→**该情形油门被消息顺序清零**（control_comply.cpp:456-465+canbus_node.cpp:197-201）
2. can_comm 忙循环（无 sleep+每圈 param RPC）；control 死亡→最后缓存全速重发，**兜底=canbus 老化 1.0s 急停**（can_comm_node.cpp:34-40；**忙循环已修**：09-10 补主循环尾 loop_rate.sleep()，/can_comm_msg 现规 20Hz[消费方仅 canbus，老化窗口 1.0s 内；HMI/monitor 零引用该话题]。param RPC 每圈热读仍在，属 ZOH 设计保留）
3. **转向数学**：getNearestIndex 首个距离回升即 break（回折/闭环路径投影错段）；calcSteer cross==0→NaN 直达输出；biaAngle 未 ±180 回卷（西向差值近 360°）
4. control fence.csv fopen 无判空→缺失即段错误；超速制动峰值仅 5%（brake 每周期 +1 上限 5）
5. m_acc_last 未初始化→desireAcc 未初始化读（经 /acc，无订阅者未上链）；模糊 PID EC>0.3 误写 indexE=6
6. **灯光左右疑似反**：canbus 3=左/4=右 vs path_plan 写 3 配注释"目标在右侧"——转向灯方向待实车核对
7. canbus：desireGear∉{2,3,4} 时 can184[6] 栈未初始化发出；estop 后 N 挡刹车被钳 20；whistle 注释 bit3 vs 夜间代码 +4=bit2
8. 急停闩锁解除依赖 can_msg.emergencyStop 先归零（rs232 解除分支被闩锁短路）
9. **感知（09-10 排除区新架构）**：LoadBoundary 无白名单扫 config/ 全部 .csv + catch(...){} 静默——**任何 csv 掉入该目录即成排除区**（trajectory_optimized.csv 轨迹草稿已实际中招：21 点闭环被当多边形、stod 吞第三列尾串不报错 → 312_316 行驶透镜区 ~140×330m 障碍物静默全剔）；目录绝对路径仍写死车载（副本 fail→无过滤放行）；空 MarkerArray 早退不清空→旧障碍粘滞；obj.vx/vy 恒 0（ACC 按静止处理）。**车载部署必删旧 config/perception_boundary.csv**（旧包容区被当排除区加载=作业区感知致盲）；charge.csv 走廊约 700m（(-344,1088)~(-121,384) 折返闭合，312_charge 终点 (-250,1138) 在内）整条充电车道排障，范围是否有意待用户确认
10. navigation：PalletType 恒 0→挂车反算恒被全局定位覆盖；rtkState 恒 "no_fixed"；断流无限重发旧位姿
11. path_plan：每条 task_plan_msg 全量重载（CSV 重读+关键点重搜）；task_id=1000 双重 ParseTaskFile；**task_id==4 清感知+getLaneLimitSpeedTest 测试后门（P0④）**
12. MQTT 明文凭据 3 套（公网 39.105.47.169 admin/qw2026MQTT；161.189.186.1 两套）；MqttClient qos=0 无锁；fms 健康计数 6 容 1 且漏统计 2 节点
13. 心跳字段：hookState=1 永不产出；drivingState 仅 0/3（longitudinal_accelerate 无生产者）；heading−90 可为负；hour+=8 跨日不进位
14. **rebuild_all.sh**：say/die/$JOBS 未定义、无 set -e、无 cd 锚定、`sudo rm -rf ./build ./devel` 相对路径——**非工程根运行会误删当前目录的 build/devel 且失败分支全部 no-op 继续跑**；CenterPoint 失败时旧 exe 已删
15. /robot/serial/rs232/ 尾斜杠话题（与发布端名可能不匹配）；robot_canbus.launch 单独启动时 config.yaml 不加载→wheelAngle×ratio=0 转向恒 0

## 四、车载部署（操作）

1. **同步**：pnc 整目录（含 .inc 与 geometry_utils.h 的 inline 修复）+canbus 整目录（含 config.cfg/msg/）+ASENSING/auto_couple/lidar_perception 构建修复 6 文件+src/ultra_command 整目录（09-10 新包，随全量段编入；启动接入方式待定）。车载无需删除项。
2. **重建**：`cd 工程根 && JOBS=4 ./rebuild_all.sh`（现行=用户分级版：rslidar_sdk -DENABLE_TRANSFORM=ON→ivlocmsg→auto_couple→lidar_perception→robot→canbus→清白名单全量；CenterPoint 独立 cmake 串行；**必须从工程根跑+导出 JOBS，原因见风险 14**；白名单缓存坑已由末段清空规避）
3. **编后核对**：7 可执行+centerpoint_ros_node；`rosparam get /canbus/hookposition`=182/254/213/254；`rostopic echo /can_msg`（hookPos/palletPos 随动、epsERR1/2 恒 0）；健康态不急停/故障位+断网急停；kill control→~1s 刹停；挂脱钩全流程（两相续接+到位停发）；云端心跳 4/3；**速度对 GPS 实测**；**转向灯方向核对**（风险 6）；config 改值重启生效；相机/蜂鸣不动作属预期
4. 入口二选一：start_l4.sh 或 HMI
5. **pnc 感知排除区（09-10 基线必做）**：同步 src/pnc/src/robot_perception_convert 两文件 + config/charge.csv 后，**必须删除车载 config/perception_boundary.csv**（旧包容区被当排除区加载=作业区感知致盲，且完全静默）；trajectory_optimized.csv 处置定案前**不得**留在车载 config/；编后核对启动日志 `Loaded N exclusion polygons` 数量与预期一致

## 五、设施与方法（本机）

- /tmp/canbus_user_verify（canbus 桩+GATE/EXEC/T1/config 测试）、/tmp/pnc_stub（46 msg 桩，6 可执行编译链接过）——重启即失，重建要点见 `../workflow_full_backup_20260905.md` 09-04 条目
- 桩编译要点：`-include cstdint`（个别 `-include array`）；4 个 TU 本机 boost 需 c++14（车载无碍）
- 快照（`../`）：baseline_vehicle_deployed（09-05 基线）、pnc_userbase_20260910（**09-10 pnc 新基底，用户宣言，9.7M/1630 文件**）、docs_before_slim（09-05）、pnc_before_hookstatus_hb/pnc_before_inline_fix（09-04）、canbus 三枚（09-03）、before_codex_merge（09-02）、pnc_taskplan_*（08-30）；备份 md：workflow_full_backup_20260905、baseline_sweep_reports_20260905

## 六、待办（优先级）

0. ~~车载编译部署~~ ✅ 09-05 完成；编后精核对（四.3，尤其风险 6 灯光方向）仍建议走一遍
1. 决策：感知排除区三项（trajectory_optimized.csv 去留/迁移 path/、charge.csv 走廊范围确认、LoadBoundary 白名单+坏行告警加固）；HMI 标定卡片去留；限值管道去留；rebuild_all.sh 修缺陷（补 say/die/JOBS/set -e/cd 锚定）；ultra_command 五项——①启动接入（唯一阻塞实车生效：control.launch include 推荐/HMI 组件/start_l4 终端，均触保护清单）②矩形与作业走廊重叠语义（任务路径自身穿矩形，挂靠目标在区内则作业段 safe 恒 1——回放 rosbag 或重画矩形）③/ultra/status/safe 消费方契约（全仓零读者，读取时机/周期/类型未知）④感知死亡 fail-safe 方向（1.0s 错报 0 vs 3.0s 对齐 sensorstate）⑤输出去抖（消费方边沿敏感则加滞回）；另 pnc 侧一行建议：/task_plan_msg advertise 加 latch（根治晚启动失明分钟级窗口，path_plan 幂等已验证）；task5.yaml task_sum:3 与 4 块定义不符（task3 脱钩块永不装载，业务确认后改 4 或删死块）
2. monitor 实车验收（scan 外参→CPU→下线 start_l4.sh 侧 simview）
3. 【需授权】修复候选=PNC_ANALYSIS P0 剩余（can_comm sleep/fence fopen 判空/急停 latch/task_id==4/fence 0517）**+本轮新发现**（风险 1 双发拆包/3 转向数学/5 m_acc_last+indexE）
4. MQTT 安全整改
5. 硬编码收敛（红绿灯框恒假可一并清/装载点/UTM 两套/油门×18）
6. 休眠代码去留；建议 git init

## 七、保护清单（清理/修改严禁触碰）

| 项 | 原因 |
|---|---|
| src/fms_agent/env/、.git、data/tmp_path.txt | fms.sh source / 唯一版本史 / 运行时读取 |
| src/CUDA-CenterPoint/build/ 与 model/ | start_l4.sh 原地启动+相对路径 |
| src/ivlocmsg/ | pnc 定位消息在用 |
| src/driver/cam_geac/demo/ 可执行 | rb_camera.sh 按名调用 |
| src/pnc/path/、param/、3rd-party/arm/*/lib* | 数据/配置/预编译库正本 |
| 根 record_*.sh、fms.sh、start_l4.sh、launch/ | 启动运维脚本 |
| 工程外快照与备份 md | 非 git 唯一回滚点 |

## 八、纪律

- 本机≠车载路径，部署需同步；HMI/monitor 无鉴权严禁公网映射；"全部停止"≠硬件急停
- PNC_ANALYSIS.md 行号为重构前基准；其 §8 qpoases 结论已证伪；其 §0.7/§2.5 部分结论已过时（InitParameter 已删/死代码已清/冻结油门已有老化兜底——见归档报告 deploy-chain 对照）
- auto_couple GBK：grep 加 `-a`；动 pnc 前必读 src/pnc/README.md（CLAUDE.md 强制）
- /hook_position 与 auto_couple 消息类型名不同字段同（MD5 巧合，改字段即断链）
- fms_agent 编译口径：纯消息+Python，catkin_make 自动生成即完成，scripts 原地跑

## 九、日志（结论级；细节查备份 md/归档报告）

- **09-10（晚二）ultra_command 对抗校验轮（ultracode：6 视角评审×3 异构镜头核实+完备性批评家，52 代理/315 万 tokens）+修复**：19 原始发现→13 去重→**10 确认/3 驳回/批评家补查 5**（补查含 1 项 P1）。**确认即修 4 项（全在包内）**：①[P1] 云端停止指令取消任务后监控闩死——task_plan 停车分支(task_plan_core.cpp:424-427)只 clearTaskPool，终态空 pathList 的 /task_plan_msg 因发布门控(task_plan_node.cpp:373-381)永假不发出，模式闩在 LEFT/RIGHT 对已取消任务持续写参数→node 增订 /cloud/msg/command_msg，commandState==0 即 SetTaskPlanPaths({})（不动 pnc；上游两停车时机行为不一致仍存，属 pnc 自身缺陷记录在案）②[P2] LoadPolygonCsv 接受 nan/inf 字面量（stod 不抛异常，NaN 顶点使射线法相邻边静默失效，/tmp 实验证实）→isfinite 校验跳过③[P3] CMake 未设 -std=c++11（stod 依赖，c++98 编译实验复现失败）→补 add_compile_options+std_msgs/geometry_msgs 传递依赖（perception.msg Header/object.msg Point）+add_dependencies(${catkin_EXPORTED_TARGETS})（对齐 auto_couple，消干净并行构建消息头竞态；此项原判「镜头分歧驳回」但三票均承认该加，作廉价加固）④[P2] 中途(重)启动失明分钟级+README「窗口小」量化失实（312_charge_01 单块 ~10 分钟）→补「有 /perception 无 /task_plan_msg」一次性告警+README 如实量化+硬性要求随栈启动；根治=pnc 一行 latch 待批。**确认未修（需用户拍板，已入待办 1/README 已知边界）**：启动接入缺失（P2，实车生效唯一阻塞）/感知死亡 1.0s 错报 0 的 ~2s 窗 vs sensorstate 3.0s（fail-safe 方向）/批评家五连：**矩形与作业走廊几何重叠（P1 级：实测任务路径自身穿矩形——312_316_01 106 点在 left1/cargo 131 点/charge 33 点在 right，挂靠目标若在区内则作业段 safe 恒 1，「无障碍→0」分支不可达，语义需确认）**、消费方契约全仓零读者、进程死亡参数冻结无监督、输出无去抖（边界徘徊实测 5Hz 方波）。**驳回 3**（记录）：add_dependencies 竞态（严重度分歧但已顺手加固）/package.xml 缺 std_msgs（同上加固）/safe 充电期粘滞（D1+D4 已文档化精确重述）。**上游顺带发现（不属本包）**：task5.yaml task_sum:3 但定义 4 块（task3 脱钩 subaction=6 永不装载，3/3 确认；也可能系故意禁用，业务确认）；rebuild_all say/die/JOBS 未定义=风险 14 复确认（新证：JOBS 未设时裸 -j 使 catkin_make argparse exit 2 即刻中止，全部 -j 行静默失败——比原记载「无限并行」更糟但机制不同）。README 决策 2/4 论据修正（空 objs 帧实际会发布——非 CUBE-only 或全被排除区过滤时；混合条目=列表首命中非 LEFT 优先，实验锁定）。**验证**：单测扩至 **10 组全过**（+nan/inf 坏行剔除、混合条目语义锁定）+node TU 桩重编（+v2nCommandFeedback/v2nFeedbackValue 桩 9/3 字段核对合）零警告；桩在 /tmp/ultra_stub（重启即失，gen_msgs.py 可再生）

- **09-10（晚）ultra_command 新包（312 任务族监控矩形障碍物检测）**：需求=task_plan 执行 pudong_air/312_316_01 或 312_cargo_01 时监测 /perception 障碍物是否落入矩形 pudong_air/left1+left2，执行 312_charge_01 时监测 right；落入→rosparam `/ultra/status/safe`=1，无→0。交付 src/ultra_command（catkin 包名 ultra_command，与 pnc 并列；node 薄壳+comply 零 ROS 纯逻辑分层，消息跨包依赖 robot 包——auto_couple 消费 ivlocmsg 同款模式，rebuild_all 全量段自动编入，**干净构建下不可单独 --pkg ultra_command**）。坐标系核对：矩形 csv 与任务路径同目录同系（定位 xAxis/yAxis 地图系），/perception 障碍物已由 perception_convert 转到同系，中心点直接可比。**规格未明说处的实现决策（详见包 README）**：①非目标任务/待命不写参数保持最后值（消费者语义未知，最保守读法；退出即清 0 的一行改法已记）②**感知静默>1.0s 按 0**——perception_convert 空障碍列表早退**不发消息**（风险 9 同源行为），只盯最后一帧则 safe=1 永久粘滞，老化窗对齐 canbus 指令 1.0s；感知链死亡由 sensorstate→急停链兜底③中心点判内（射线法逐顶点同 IsPointInExclusion，不做 dx/dy 尺寸展开，与排除区过滤同口径）④精确字符串匹配：02/03 同族变体不触发；pathList 多条目首匹配 LEFT 优先；task8（charge_01）末块 action 路径 sx040901 期间模式回 NONE。矩形经全局 path_dir 每周期热读+失败自动重试（先于 pnc 启动也不死），**不用目录 glob**（避风险 9 模式），加载失败按无障碍+醒目打印。10Hz 主循环 ZOH 重发（外部改写可自愈）；启动写 safe=0 一次（对齐 task_plan 启动写 /cloud/suggestspeed）。已知边界：/task_plan_msg 非 latch，晚于任务下发启动需等下次块推进同步。**验证**：comply 纯逻辑单测真实 CSV 跑 8 组全过（三矩形质心入区/区外=0/模式精确匹配含 02 不触发/多障碍/1.0s 老化往返/未加载不崩/坏目录失败/无尾斜杠幂等）；node TU 消息桩从真 .msg 机械生成（task_plan_msg 13/perception 2/object 11 字段核对合）+ros 桩 `-fsyntax-only -Wall -Wextra` 零警告；桩在 /tmp/ultra_stub（重启即失）。**车载部署**：同步 src/ultra_command 整目录+rebuild_all；启动接入（start_l4.sh/HMI 均保护清单，未动）与编后核对清单见包 README

- **09-10（下午）can_comm 忙循环修复（新基底首笔改动）**：can_comm_node.cpp:40 主循环尾补 `loop_rate.sleep()`（README 架构约定「频率分档 can_comm 20Hz」的既知漏项例外，同款写法对齐 task_plan_node:382/path_plan_node:264）。效果：循环从 CPU 满速空转规 20Hz，/can_comm_msg 由全速重发→20Hz（消费方仅 canbus_node queue1，canbus 老化兜底窗口 1.0s 远在范围内；HMI/monitor 经 grep 零引用该话题，无健康监控连带）；每圈 param RPC（/canbus/brake 等低频指令热读）随循环降频，ZOH 语义不变。**验证**：本机无 noetic（/opt/ros 只剩 jazzy/ROS2），按桩编译法做微桩——ros/ros.h 桩（仅本 TU API 面：init/ok/spinOnce/Rate/NodeHandle.subscribe 函数指针推导重载/advertise/publish）+4 消息桩**从真 .msg 脚本机械生成**（铁律 5，can_comm_msg 17/17 字段抽查合）+pubalgor 空桩，真 can_comm_node.cpp+真 can_comm_comply.h 过 `g++ -fsyntax-only -std=c++14`；桩在 /tmp/cancomm_stub（重启即失）。**车载部署**：pnc 整目录同步已含此文件，rebuild_all 重编 robot 包即生效（四.5 感知排除区清单与此独立）

- **09-10 pnc 基线重置（用户宣言「忘掉过去，今后在这个版本的基础上改动」）**：全量 diff vs 09-05 基线=src/pnc **仅 robot_perception_convert 两文件不同**（其余逐字节一致，全部 src mtime 统一 09-04 12:16:43=外部 zip 整树覆盖指纹；09-09 用户 11 文件+6 修复随覆盖消失——6 修复因对应用户缺陷代码同失而无需补，**唯 can_comm sleep 已验证改进丢失[风险 2 复活，待重补]**）；canbus/hmi/monitor/launch 未受覆盖，各自保留 09-05 后演进。**感知重写=zyd 版**（版本链：基线→zip0909→zipcsv[`/home/zyd/0aqw/A03/` 外来版]→现树[用户仅改一行路径回车载]）：单包容多边形→多排除多边形，LoadBoundary 扫 config/ 全部 .csv（文件名排序/<3 顶点忽略/dirent），IsPointInBoundary(内=放行)→IsPointInExclusion(内=剔除)**语义整体反转**；编译面过（algorithm/fstream 在 .h:8-10，dirent.h 新增）。**P0×2**：①config/trajectory_optimized.csv=312_316 路线闭环优化草稿（21 点首尾相同，第三列恒 1 非 heading 列），被目录 glob 当排除多边形加载（stod 吞尾串+catch(...){} 静默）→行驶透镜区 ~140×330m 障碍物静默全剔；②车载部署不删旧 perception_boundary.csv 则旧包容区反转为排除区=作业区致盲。P1：目录 glob 无白名单/无格式校验；charge.csv 走廊 ~700m（312_charge 终点在内）整条充电车道排障待确认。param/task{4,5,6,8,18}.yaml=312 任务族（cargo/charge/316，引用路径全存在；sx040901 死引用无害[DOACTION 块 path_plan 不加载 CSV]；WAITING=7 仍零使用）。src/ 下两个 perception zip 副本建议移出源码树。**本轮零代码改动**；快照=../pnc_userbase_20260910.tar.gz

- **09-09 用户 pnc 大改审查+风险修复轮**（用户 09-08~09-09 自改 11 文件 ~1700 行 vs 09-05 基线；can_comm 补 loop_rate.sleep 落实 20Hz 分档✓、task_plan 删 max_vehicle_speed 钳制系**用户有意**[原钳制单位错位本就不生效,残留 config.yaml 死参数与 node:19 注释已清]、path_plan 碰撞段重写门槛 0.1→0.5 对齐注释）。双审查代理+逐条亲证后发现并修复 6 处：①②task.inc updateAirPortStopIndex/updateAirPortInfo 两处 range-for **按值遍历写拷贝**（机位索引投影与占用状态全失效,起步可楔死 0 速）→改 `auto &`；③碰撞段滑窗被删致**风险空帧不再早退→急停去抖计数器每空帧清零,间歇检出永远凑不满 3 帧**→恢复 4 帧滑窗（在场判定+最近非空快照回取,保留用户 0.5 门槛与新结构,不恢复 dist_to_refline 死分支）；④恢复 distance2Object 的 100+/200+ 哨兵赋值（发布铁律,含 objs 空早退补 100+）；⑤sensorstate 的 +1 总标志位挪回位累加后（否则心跳 HB.sensorsState 恒 0 上云）；⑥spinOnce 挪回主循环顶（README 骨架,消除输入快照 100ms 滞后）。知悉未动：navigation status==2 改 early-return（用户有意,GNSS 降级首报 2s）、全目录 Allman→K&R 重排（与 README 目录风格相反,未回退）、min_speed 死变量、.height 死存储（保留供调试）。**验证：括号平衡+上下文复读过；未桩编译（46 msg 桩已失,重建成本高）——车载 rebuild 前述 6 文件需整体同步**：robot_path_plan{node.cpp,reference.inc,task.inc}+config.yaml+task_plan_node.cpp（task_plan_core.{h,cpp} 用户已改）

- **09-08 晚 health_monitor 新包（话题健康监测,C++/单线程/零侵入）**：需求=监控全部话题+点云占空比采样+1Hz 出 /diagnostics,消费方用户后续自建。**前置 spike 推翻 /statistics 路线**（roscpp 未实现该特性,rospy 为订阅端语义,ros_comm noetic 源码验证）→ ShapeShifter 主动订阅方案。设计 spec 经 ultracode 17 代理五视角对抗校验（27 发现→9 确认全吸收）,关键修正：发现用 ros::master::getTopics 发布者表口径（getSystemState 并集会被自身订阅钉死+roscpp 无此封装）;重载荷池=PointCloud2+Image+CompressedImage（否则 2 路 1080p20 JPEG 被常订,预算破产）;event 话题 8 个实证入默认表不判 stale（/task_plan_msg 等,待命不误报）;HzMeter (ns,bytes) 成对存储防 traffic 单调发散;常订 queue=3 防 100Hz 丢帧;kill/kill -9 两种死亡路径分测。交付：src/health_monitor 新包（node/逻辑分层,纯逻辑 hz_meter+state_classify 可独立单测）;本机验证=桩编译(/tmp/hm_stub)+纯逻辑单测全绿,**实车清单见包 README 未执行**。车载部署：同步 src/health_monitor 整目录,rebuild_all 全量段自动编入,roslaunch 即起

- **09-08 晚 calib 整包删除（用户自删 src/calib 后核验零残留）**：calib_logger+test_control 两包，上轮已证零入口引用（无 launch/HMI/脚本/文档引用，仅手动 rosrun/roslaunch）。本轮全树精确扫描（calib_logger/test_control/src/calib）**零残留+反向依赖零**（无任何剩余包 package.xml/CMakeLists 依赖之，catkin 构建不受影响；rebuild_all 分级段本就不含 calib；宽扫 calib 命中均为无关词——fms_agent 的 pip 包 camera_calibration、CenterPoint PTQ 量化校准）。本轮零代码改动（仅 workflow.md），上轮测试结论（test_full 80/0 等）仍有效。两包定性：横向=Stanley+纯跟踪试验场（调参从未回流生产 control）、纵向=油门/刹车响应数据采集（acc 列恒 0 占位）。**车载部署**：删 src/calib 整目录+重建即可，无其他文件需同步。另：核验时检测到 robot_path_plan/.path_plan_task.inc.swp（用户 vim 编辑中）——非残留，未动

- **09-08 data_logger 整包删除+全树关联清理（用户自删 src/data_logger 后委托清理）**：七处关联清除——①start_l4.sh 删 data_logger 终端步（"network"标题那步，现 12 终端，后续步骤序号前移）②根 launch/data_logger.launch 孤儿副本删除 ③HMI：hmi_config.py 删"行车记录"组件（15→14 组件）+README 组件说明行+VEHICLE_TEST.md 表格行/尾注/核对项同步 ④test_record_rosbag.py 断言改 assertNotIn（锁死不回归）⑤**calib_logger 运行时借用 data_logger 路径两处改自持**（getPath 改 "calib_logger"；其 config/log.yaml frequency 20→10——原代码读的是 data_logger 的 yaml(10Hz)，自家 yaml(20)系从未生效的死配置，对齐 10 保行为不变；改前若直接删包，calib_logger 启动会因 LoadFile("/config/log.yaml") 抛未捕获 InvalidNode 崩溃）。calib_logger 修改仅两处字符串字面量（类型不变），未桩编译，车载 rebuild_all 全量段会实证。CLEANUP_REPORT.md 的 data_logger 字样系 09-02 时点历史记录，保留。验证：hmi test_full **80/0**（单独串行跑；曾出现一次 6 失败系我把 chaos_test 与之并行造成的进程互扰，单独重跑即复绿——**hmi 测试勿并行跑**）+ test_record_rosbag **24 OK** + frontend_test.js **72/0** + chaos_test 与改动零耦合（不 import hmi_config，纯进程层自包含，浸泡型未整跑）+ 全树无死角 grep 零残留（含无扩展名文件）。**同轮核验发现用户 09-08 晨自改两文件**（与本次清理零交集）：path_plan_node.cpp（11:33，风格回退+删注释死码+订阅移位非删除；两处语义变化——①navigation status==2 由"时间戳回拨 100s 即刻判障"改为"early-return 不更新时间戳，2s 后自然判障"②sensorstate 的 +1 总标志位移到位累加之前恒不触发（基线原在后本可产出 bit0；消费方只判 !=0，无影响））、canbus_comply.cpp（11:54，删 /canbus/time 唯一写入行）。用户自改件未桩编译，上车前注意。**车载部署**：同步删 src/data_logger、launch/data_logger.launch + 改 start_l4.sh、hmi/{hmi_config.py,README.md,VEHICLE_TEST.md,tests/test_record_rosbag.py}，重建+重启 HMI（⑤的 calib_logger 两文件随下条 calib 整删作废，车载无需同步）；devel 内旧产物随 rebuild_all 首段清除

- **09-07 晚十 dashboard 对抗校验轮（ultracode：6 视角审查×双怀疑论者核实,18 代理/113 万 tokens/327 工具调用）**：12 原始发现→**4 确认/2 分歧/6 P3,全数处置**。数据逻辑视角(对照两份真实 .msg 逐规则验算)零发现。①[P2]flash 动画 box-shadow 整段覆盖拟物凸影(600ms 动画>500ms 轮询,活值瓦片凸影常态缺失且蓝环永不完结;核实者在真实 Chromium 复现)→keyframes 两帧保留基础柔影只衰减蓝环,浏览器采样复核三影层并存 ②[P2]ehb_msg.msg:3 头注"随 /can_msg 同拍发布"系晚七漏改→改"T2 20Hz/相位不锁"(仅注释,md5 不变无需重编) ③④[P2 测试盲区]sections=null 降级 payload 与 ros/master off 态零覆盖——变异删守卫/改绿灯均 150 项全绿(降级态会崩 render 伪装总断连;ROS 挂显示绿灯)→补 5 断言 ⑤[分歧→加固]_fmt_value 非整型数组元素 TypeError 打死整个 /api/dashboard(现网 .msg 无此形态,但"改 .msg 重启跟上"是文档化演进路径)→逐字段 try 降级 str+d12 用例 ⑥[分歧→加固]FAU 位属性缺失按 False 计入"全部正常"(违反自家"无数据不得假正常"不变量;真实 catkin 类全字段必有故现网不触发)→三态判定(任一位不可读=无数据),mock 补 34 位对齐真实类形态 ⑦P3×5:.tn 对比度 2.69:1→--dim/nodata 占位类常驻→render 清类/lastVals 跨卡同名互扰→话题前缀键/poll 慢响应乱序回跳→防重入+15s 死锁逃逸/日期 09-08 笔误→09-07。**修复过程自踩两坑**:mock 字典括号错位 34 键漏外层(运行时自检抓回);spec sections[0] 系头部空节(payload 跳过而测试取 FAU 名单未跳过)。回归:test_dashboard **110/110**(d05 三态重写+d12 新)+frontend_dash **51/51**(+降级/off 态/fsum 闪烁/防重入)+test_full 72/edge 28/frontend 62/fuzz 2/chaos 11 全绿。**方法论:变异沙箱(agent 自改自测)+双怀疑论者分镜头(代码实证/触发路径)是抓真缺陷主力;核实员严重度分歧本身有价值——分歧项按"成本极低+违反成文不变量"取向加固;变异代理动过工程文件,修复前先六标记核对工作树纯净**

- **09-07 晚九 dashboard 布局紧凑化（用户认可晚八版后的追加需求）**：can 卡片收至原宽 1/3（160px,瓦片单列微缩 10/12px 字号）、ehb 收至 1/2（575px,瓦片 minmax 104px 约 4 列小号）;#wrap 两栏 grid→flex 靠左不拉伸,**右侧大片留白留给未来新增信息面板**（窄屏 flex-wrap 自动换行）。实现要点:卡片加 can/ehb 变体类承载差异化样式,**markDead/render 赋 className 会整体覆写须保类**——前端测试 5 处 className 断言同步。测试:frontend_dash **44/44**+test_dashboard **106/106**+浏览器截图目检（can 单列无截断/右侧留白约 40%/无重叠错位/ehb 故障条贯通）。车载部署仍=同步 monitor/ 目录

- **09-07 晚八 dashboard 瘦身瓦片化（用户需求:去垃圾信息/凹凸感/can 不显 raw/ehb 注释精简）**：payload 即瘦身形态——can 27→**22** 瓦片（隐藏 rawcommand/rawfeedback/epsERR1/2/faultCode——faultCode 写路径 09-03 已下线恒 0,用户确认隐藏）、ehb 63→**27** 瓦片（35 个 *_FAU_* bool 折叠为分节首行汇总条三态:绿全正常/红 N 项点名/灰无数据;隐藏 RollingCounter/CheckSum 四字段）。注释只留换算/单位（ehb 正则 ×0.1 MPa/×0.04 MPa/×0.1 km/h/0-100%;can 单位 m/s/%/°/A,含=枚举括注丢弃）;**译码仍用 .msg 原注释——先译码后裁剪,顺序倒置会杀掉全部 ehb 绿字译码（两轮对抗校验头号发现）**;can 补 CAN_ENUMS 显式译码（挡位/急停/按钮/状态机）+VHL_VehicleGear uint16 双字节字符译码（恰一字母才译）;分节标题精简（"帧4 制动请求"）。前端表格→凹凸拟物瓦片（凸卡+凹槽双向柔影）,删英文名副行/注释列/topic 行/页脚;停滞徽标改挂卡片标题（原锚点随 topic 行删除失效——校验轮发现）。**两轮换视角校验共修 9 处设计缺陷**（另:无数据假绿"全部正常"/can 单位丢失/档位裸数字/文档同步缺口）。测试:test_dashboard d05/d07 重写 **106/106**、frontend_dash_test 重写 **44/44**、回归 test_full 72/edge 28/frontend 62/fuzz 2/chaos 11 全绿;mock 全栈+浏览器截图目检（凹凸效果/故障红绿/译码绿字/单位小字全就位）。车载部署=同步 monitor/ 目录（纯 Python 无编译）。教训:①后台任务沙箱禁 listen,视觉验证服务须前台 nohup 派生 ②pkill/pgrep -f 模式自匹配本命令行（连翻两次车）,清理按端口 `fuser -k` ③换视角二次校验（验收者→实现走查）两轮各抓出不同缺陷,单轮自校不够

- **09-07 晚七 用户 canbus 侧自修+全树一致性复位**：用户自行将 /can_msg 发布自 T1 移回 T2Callback（20Hz,恢复基线编排;T1 仅余 /ehb_msg@10Hz）——从发布源头满足 HMI 原始 min_hz=20,与晚六"hmi_config 降 10"二选一,用户择源头修复,晚六方案随之作废复位。config.cfg 另加 vehicle1/vehicle3 双车注释（值未变,vehicle3=182/254/213/254 现行）。复核（vs canbus_before_ehb 快照 diff）:canbus_node.cpp 净变化=ehb 三行接线,无隐藏改动;pnc/monitor/dashboard 消费均频率无关。随之复位三处漂移:hmi_config min_hz 10→20（与 ros_test_config/车载原始值一致,监控紧度回基线）+canbus README 四处 T1/T2 描述与风险条+本文件现状。测试:hmi test_full **80/0**。**车载:canbus_node.cpp 属编译件,若仅在副本改则车载需同步+重编 canbus 包;hmi_config 晚六的 10 若已上车可回传 20（20Hz 实际下两值皆健康,20=基线紧度,建议统一）**

- **09-07 晚六 实车故障定位：HMI"CAN 总线启动超时未达健康"**（用户报 /can_msg echo 正常）。根因=hmi_config canbus 健康判据 `/can_msg min_hz=20`（ros_bridge 按 min_hz×0.8=16Hz 判）vs 实际 10Hz——用户把 can_msg/ehb_msg 发布自 T2 移 T1(10Hz) 时 HMI 阈值未同步,晚三风险预警应验。修复:hmi_config `/can_msg min_hz 20→10`（有效门槛 8Hz;/can_recv 20 不动,socketcan_bridge 未变）。验证:hmi test_full **80/0**。过程教训两条:①A/B 对照用的全局 sed 误伤 /can_recv//navigation_msg(20→10),靠全配置核对抓回——**配置文件禁用无锚点全局 sed** ②T17"current 为 config.cfg 实际值"硬编码 09-01 旧限值(185/240/130/240) vs 现行 182/254/213/254 系陈旧漂移(与本次无关,A/B 实证),已同步常量。**测试盲区:hmi mock 泵 /can_msg 50Hz,测不出"配置阈值 vs 实际频率"漂移**。车载部署:同步 hmi/hmi_config.py 后重启 HMI 即可,canbus 侧无改动

- **09-07 晚五 修复轮二遍对抗校验（5 视角+变异测试沙箱,12 代理）**：结论——**修复轮生产代码零缺陷**(decoder 63 字段×0..255 全扫描 0 错译;6 个指定变异 M1-M8 全被测试捕获),但暴露 5 条缺口全修:①[P1-测试缺口] monitor_server dashboard 启动整段 try/except 守卫零测试覆盖(删除后套件仍全绿,但端口被占场景守卫承重——沙箱实证无守卫则主服务死于 Errno 98)→新增 d11 占用守卫测试(预占 socket+断言主服务存活+stderr 告警) ②[P2-测试缺口]"二进制先于十进制"分支顺序无锁(交换后 6 个 2bit 字段值 10/11 会把垃圾字节译成"错误/无效"权威中文)→d03 补 D(10)/D(11) is None 顺序锁 ③[P2]DASHBOARD_PORT 空串静默关闭且遮蔽配置值→改空串=未设回退配置(不遮蔽不告警;非法值才告警+关闭) ④[P2]前端总断连(API 挂/JSON 坏)四点/龄/卡片残留最后一拍全绿——比部分故障更严重的总故障反而无指示→markDead(四点红+龄清空+卡片停滞),对齐 index.html ⑤[P2]README 称 .msg 失败 stderr 告警实为静默→dashboard.py 补 stderr 告警(文档变真而非改文档)。REFUTED 2:FakeMaster TIME_WAIT(SimpleXMLRPCServer 本有 SO_REUSEADDR,复跑不复现);空串无告警与主端口惯例同源(已被③以更优语义解决)。回归:test_dashboard **82/82**+frontend_dash_test **31/31**+test_full 72/edge 28/frontend 62/fuzz 2/chaos 11 全绿。**方法论:变异测试(沙箱撤销修复→测试必须红)是锁"修复被测试真正覆盖"的最直接手段,守卫类防御代码最易零覆盖**

- **09-07 晚四 dashboard 对抗审查与修复（ultracode 工作流:5 视角审查×逐发现对抗核实,15 代理/80 万 tokens）**：换思路弃自证脚本改独立对抗审查——16 条原始发现,**8 CONFIRMED/2 REFUTED**(其余去重合并)。修复全部 8 条:①枚举译码缺纯十进制键形态→VHL_EPB_ParkingRequest 值 2(请求释放)/3(无效)译不出(键正则 [01]{1,8}→\d{1,8}+三态匹配:hex/二进制等宽/十进制;二进制必须先于十进制否则"10"当 10) ②_fmt_value 不认 bytes——rospy(Py3) uint8[]=bytes,mock 用 list 致测试盲区,实车 faultCode/rawcommand/rawfeedback 显示 Python repr→补 bytes/bytearray/memoryview→hex ③.msg 非 UTF-8 直接炸穿 main() 含主 8081(DashboardApp 只捕 OSError;UnicodeDecodeError 是 ValueError)→utf-8 优先 GBK 兜底+except Exception 降级 msg_load_errors+monitor_server 整段 dashboard 启动守卫 ④译码文本污染:0xFF:忽略;64768-03 整段入文本→截去 ( ; , 后缀 ⑤DASHBOARD_PORT 空串→int('') ValueError 拖死主服务→安全解析(非法/越界告警+关闭) ⑥端口>65535 OverflowError 穿透 except OSError→并入守卫 ⑦前端话题点永不熄灭+冻结值伪装健康→三态(ok/stale/off,5s 阈值对齐 index.html)+停滞卡片半透明标注 ⑧$( "err")笔误。REFUTED 2:区间键 0x4~0x6 跳过系设计;空串行为与主端口 MONITOR_PORT 既有死代码同源(仍顺手修了新代码侧)。回归:test_dashboard 74/74+frontend_dash_test 24/24(新,DOM 桩驱动真实页面脚本)+test_full 72/edge 28/frontend 62/fuzz 2/chaos 11 全绿。**方法论结论:mock 类型契约与真实 rospy 不一致(bytes vs list)是本轮最大盲区来源,自证式校验测不到自己没想到的类型**

- **09-07 晚三 monitor dashboard 仪表板（8082,can_msg+ehb_msg 全字段中文展示）**：monitor 同进程第二端口（monitor.sh 不变,绑定失败仅告警;DASHBOARD_PORT 可改/置 0 关闭）;ros_visualizer +/ehb_msg 订阅+latest()/master_ok() 取数接口;新文件 dashboard.py+static/dashboard.html+tests/test_dashboard.py。**中文名称对齐策略**：ehb 启动时运行时解析 ../src/canbus/msg/ehb_msg.msg 注释（构造性对齐,改 msg 自动跟上）;can_msg 无中文注释→dashboard.py 内 CAN_LABELS monitor 侧命名（27 字段,改 can_msg 需同步）;英文注释（CAN BUS OFF）回退原文。附枚举译码（00:/0x0:/冒号空格三形态,值旁绿字）。验证:test_dashboard 56/56+全量回归（test_full 72/edge 28/frontend 62/fuzz 2/chaos 11）+monitor.sh 真实入口双端口冒烟过。**环境发现:本机 11311 被 root 的真实 rosmaster(noetic,/opt/ros/noetic,09-05 起)占用**——测试伪 master 已迁 21111~21113;本机其实装了 ROS(与"本机无 ROS"旧记录不符,桩编译流程不受影响,rospy 对 monitor 仍不可 import(未 source)）。同轮修正:上轮漏更的 README 运行逻辑 T1/T2 描述（发布移 T1 后过时 4 处）

- **09-07 晚二 EHB「接收信号需求」纳入（ehb_msg 52→63 字段,canbus 旁听九帧）**：接收方向全部 SA=0x58（VCU→EHB）——0x08FB1458 制动请求(4 信号,请求压力×0.04 MPa)/0x08FB1558 驻车请求(3)+补充项 0x0CFD0358 车速/0x0CFD0058 上电/0x0CFD0158 踏板/0x08FD0258 档位(ASCII);RecvCanData +6 case(共九),**canbus 只旁听解析不发送**;「其他液压管路压力」ID=暂无未纳入。要点:RollingCounter 声明 8bit/位跨 4bit(I02)→按位置 4bit 解析(显式假设,与 0-0xF 逻辑范围一致);CheckSum 求和宽度未定义(I06)→仅存不校验;档位 16bit ASCII 摆放未定义(I09)→存小端原始拼接;**4 补充项中文信号名→英文名(VHL_VehicleSpeed/HvPowerState/BrakePedalStatus/VehicleGear)系整理命名非原表,已注 msg/README**。同轮用户改动:ehb_msg+can_msg 发布自 T2 移至 T1(10Hz,ehb 先发)。验证:63 字段 python 交叉核对+定向 73/73+接线 7/7+随机往返 800×9 全过;期间校验器抓出并修正一处测试向量笔误(0x2F→0xF2 高4bit)

- **09-07 晚 EHB 依用户协议文件重做（唯一来源=src/canbus/docs/ehb-can.md,旧 SST 锚定版全废）**：用户整理的 ehb-can.md（xls 保真转写,含冲突编号 I01~I12 与逐行单元格账本）取代此前 SST 锚定法结论。ehb_msg.msg 重写（52 字段=信号名原文,diff 逐条核对）+RecvCanData 三 case 重写+**新增 /ehb_msg 发布**（时为 T2 20Hz,09-07 晚二随用户改动移至 T1 10Hz,topic 暂无读者）。纠正旧版三错：①ATB 15 故障位顺序（3.3=CanBusOff/3.6=BrakePressSensorError(旧版漏此字段)/3.7=LosOfEHBSTOCom/3.8=MotorCommuteFail=电机**校准**失败;删 EHB_ATB_Reserved）②EPB 5.1=RcError/5.2=CsError（旧版颠倒）③头注方向（0x08FB1458/0x08FB1558 才是整车发送的请求帧;FD 系列=车速/上电/踏板/档位,EHB 接收补充项）。验证 /tmp/ehb_verify：52 字段断言+0x185/285/0C02A0A2 回归 62/62+发布接线 3/3+**随机往返 500 帧全字段**+python 按 md 布局对 cpp/msg/测试三方机械交叉核对全过;大写字段名已对照 noetic genmsg 源码确认合法（BASE_RESOURCE_NAME_LEGAL_CHARS_P=^[A-Za-z][\w_]*$,尾注释合法）

- **09-07 晨 EHB 定稿〔本条 SST 锚定结论已被 09-07 晚条目取代,勿引用〕（名称/中文注释与 xls 完全对应,用户需求）**：SST 提取突破——**锚点扫描法**（前缀 `10 00 00 00`+尾 `01 00 0c 00` 交叉验证,CONTINUE 段首宽度标志剥离）恢复 201/367 串且偏移序=索引序;三帧结构定稿:FB67=STO 蓄能(20 信号:4×2bit 状态+5×8bit 压力原始+4bit 故障数+10 故障位,名称[72-91]/中文[44-53]/位序三方锚定)、FB68=ATB 主动制动(压力+状态+故障数+**15 故障位顺序系语义推断未锚定⚠**)、FB69=EPB 驻车(2bit+3bit+驻车液压+4bit+10 故障位锚定);接收方向真帧=0x0CFD0058/0x0CFD0158/0x0CFD0358(SA=0x58,初版"模板ID"误判实为此);ehb_msg.msg **字段名=xls 信号名原文+注释=xls 信号描述原文**,解析器/测试重写(EHB 35 断言+回归全绿);⚠ 字段名大写开头(EHB_*)ROS1 genmsg 接受(ROS2 才强制小写),若车载 catkin 意外拒绝则首字母小写一键替换

- **09-06 EHB 帧 ID 纠偏（用户指正）**：初版三帧 ID 锚在 xls 模板残留的示例串（0x0CFD00xx,模板备注列渗入）——**正主=P=2/SA=0x0E：0x08FB670E(FB67=64359,与行3 '64359-02' 自洽)/0x08FB680E(FB68)/0x08FB690E(FB69)**;canbus_comply.cpp 三 case+ehb_msg.msg 头注释+测试已改,EHB 31/31+回归全绿重验;教训入 canbus/README:**绑定漂移的 xls 里帧 ID 类关键值必须人工确认**
- **09-05 EHB 解析交付**（快照 ../canbus_before_ehb_20260905.tar.gz）：docs/ehb-can.xls(213KB WPS 变体 BIFF,本机无 xlrd/pip)→手写纯 Python OLE2+BIFF 解析器提取（SST 字符串头带 4 字节前缀的变体;绑定部分漂移但**数值单元格 100% 可靠**;**位布局破译**:起始位"字节.位"且 Excel 部分存成 ×10000(如 21000=2.1),验算全自洽）。产出:msg/ehb_msg.msg(帧1 0x0CFD0058 全展开 20 信号/帧2 0x0CFD0158/帧3 0x08FD0258,J1939 Intel 位序位1=LSB,SA=0x58,均 100ms DLC8)+canbus_comply.{h,cpp} mEHBMsg 与 RecvCanData 三 case 解析+CMake 注册;**桩验证 EHB 31/31+回归全绿**;⚠ 待用户对照 xls 目检一次:帧1 byte1 四个 2bit 含义/帧2·3 位图名称/故障位对应(候选名已列 README)。mEHBMsg **未发布**(需求只要求存入;外发= T2 加一行)。**同轮纠正**:09-05 晨"GATE 全绿"系陈旧二进制误报——用户 09-04 的 :100/:101 联锁改变三组期望,test_gate 已按部署版真值表重写(14 断言含 pallet 组合与自收敛)全过

- **09-05 基线重分析（本条目后现行版确立）**：用户报实车部署未发现问题→本树定为唯一基准（快照+归档）；ultracode 6 路深读（593k tokens/284 工具调用）产出：canbus 0x284 真值表与联锁语义、控制链新发现（双发拆包清油门/转向数学三处/fence fopen/m_acc_last）、全仓契约矩阵（死通道群清单）、rebuild_all 缺陷精查；修正自身文档偏差；关闭待办 0；新增风险清单 15 条。**对抗校验轮（3 代理×34 声明）**：32 CONFIRMED/2 REFUTED——驳回①"恰好 4 键"（节点实写 5 键，/canbus/time 每周期写，文档措辞本就指位置键无需改）②完成判定行号（深读代理报 :445/:454 有误，实为 **:443/:452**——早间据深读代理的"行号修正"被对抗轮纠正回原始值，已改回）；control-bridge 12/12 全确认（双发拆包/忙循环/fence fopen/刹车 5% 峰值/m_acc_last/indexE/转向数学/灯光反转/PalletType/感知粘滞全部代码实证）
- **09-05 晨** 精简全部文档（942→351 行），新建 canbus README；用户改动两处入基线：canbus_comply :100-101 挂/脱钩两相续接联锁（销到位后单独驱动鞍座至端态，含自动收敛特性）、rebuild_all.sh 分级版（依赖序+白名单清空）
- **09-04** 心跳迁移 hookStatus+param hookstate 清退；:79 平滑过渡判据统一；部署前验证 6 exe 桩编译过+geometry_utils ODR 修复；fms 契约查明；车载首跑 --pkg 撞消息生成→用户改依赖序分级后成功；Firefox WebGL 三开关终解
- **09-03** 晨 canbus_core 终局七轮（随晚间重写作废）；**晚** 用户自研重写 canbus+我修 5 处+六项下线+老化/config 化
- **09-02** 新车部署；两树统一本树唯一正本
- **09-01** 标定实车迭代/stall 极值版（后均删）；Eigen 泄漏修复；HMI 标定模块+去 rviz；rosbag 启动器
- **08-31** 任务切换需求实现；canbus 标定引入（后删）；monitor 右侧栏
- **08-30** pnc 专项（重构/CMake/control 四轮/话题全局化/task_plan core 化/规范文档）
- **08-28** monitor 交付；PNC_ANALYSIS 产出
- **08-25/26~27** HMI 交付+canbus_core 重构（后取代）；自启+config 收敛
- **08-19/20** 全量分析；清理 175M；MQTT 明文凭据（待整改）
