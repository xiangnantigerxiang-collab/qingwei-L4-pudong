# canbus：使用方法与现行协议契约

文档核对：2026-09-21。原始用户自研基线形成于09-03～09-05，后续EHB/周期变化如下。

> 原版 canbus_comply 架构；canbus_core 重构版已下线（历史见工程根workflow及文档归档）。
> 启动：`launch/canbus.sh`（modprobe+can0 250k → canbus.launch → socketcan_bridge[/can_send,/can_recv] + aeb_can_node）。

## 使用方法

在车端ROS1工程根构建 `canbus` 及消息依赖，source工作空间后启动：

```bash
catkin_make --pkg canbus -j4
source devel/setup.bash
bash launch/canbus.sh
```

该脚本初始化can0/250k并接入socketcan_bridge；HMI已启动CAN时不要重复运行。
`config.cfg`在启动时读取，改标定后重启。核对输出：

```bash
rostopic hz /can_msg
rostopic hz /ehb_msg
rostopic echo -n 1 /can_msg
```

/can_msg应由T2以20Hz发布，/ehb_msg由T1以10Hz发布；完整字段/协议见下文。

09-21起支持托盘反向码值：`pallet_position_min` 始终表示**上端**，
`pallet_position_max` 始终表示**下端**，不要求数值递增，也不自动交换。
当前配置为上端212、下端115；需重编canbus并重启节点才能使用新的校验规则。
启动后用 `rosparam get /canbus/palletposition` 核对实际加载的两值；配置被拒绝时会成对
回退默认213/254，仍会发布参数，因此“读取成功”不代表加载了配置文件中的标定。

到位条件保持 `abs(palletPos - 端点值) < 5`，T1连续命中11次才确认：
当前上端有效区间208～216对应 `palletStatus=4`，下端111～119对应 `palletStatus=3`。
反馈122距下端115为7，状态保持0；**0表示没有确认端态，并不能证明托盘仍在运动**。
只有确认122是实际最低端的稳定反馈后，才应据此重新标定下端；不能仅为消除状态0扩大容差。

## 注意事项与容易疏忽的点

- 09-07曾把can_msg移到10Hz造成HMI健康误报，最终恢复T2 20Hz；不能只改发布位置不查消费者阈值。
- 0x284同一字节的多个独立if存在后写覆盖，不能随手改成else-if；挂脱钩的自收敛仍是业务。
- EHB唯一协议源是docs/ehb-can.md；旧SST提取/推测已作废，制动请求×0.04与反馈×0.1不能混用。
- N挡仍可能把急停80钳到20；非法挡位未赋字节等属于已知边界，不能当风格复制。
- can_msg/can_comm_msg三包副本与monitor映射须一起核对，字段变化需重编实际消费者。
- /ehb_msg现在被monitor仪表板消费；消息声明或旧“无读者”记录不能作为删除依据。
- 09-19的D补刹在control实现，不要为了描述新需求改本模块CAN协议或R输出。

## 历史摘要

09-07按用户协议完成九帧EHB解析并恢复can_msg频率；09-03～09-05确立当前node/Comply
车载基线。较早的core架构和旧协议推断不恢复。下文保留仍有效的契约，原始过程见
[归档](../../docs/history/2026-09-19-before-docs/src/canbus/README.md)。

## 结构

| 文件 | 职责 |
|---|---|
| `src/canbus_node.cpp` | main（节点名 can_node，launch 重命名 canbus）+T1(10Hz)+T2(20Hz)+LoadPositionConfig；限值四全局变量 |
| `src/canbus_comply.{h,cpp}` | VehicleComm(组帧)+RecvCanData(解析)+相机死方法 |
| `src/struct_type.h` | pnc 同名副本（仅多无读者的 GEAR_P） |
| `msg/` | 26 个 .msg，**CMake 生成 18 个**（fault_msg/keypoints/object1/obstac/RoutingPath/task_plan_status/vehicle_task_status/visionObjects 未列不生成）；can_msg/can_comm_msg 与 pnc/simview(包名 view) 必须同步改 |
| `config.cfg` | 限值四键，当前178/252/212/115；内置回退182/254/213/254，启动读取 |

## 运行逻辑

- **T2 20Hz（VehicleComm+发布 /can_msg）**：手动模式发中性帧即返回；自动模式发 0x184（油门/刹车/转角 ±700 钳位/挡位+EPB：D,R=0x10|g，N=0x20|g）与 0x284（byte2 见下表；灯光 /canbus/light 3=左 4=右+夜间 +4=bit2；/canbus/time param）。约束链：**estop→Brake=80（唯一急停值，指令老化也并入此通道）**；Brake>0→油门/转角清零；N 挡油门清零且刹车钳 20（estop 的 80 也受此钳）。发布 /can_msg。
- **T1 10Hz（状态机+发布 /ehb_msg）**：EmergencyStop = sensorstate≠0 ∨ netcheck≠0 ∨ !planning_alive ∨ 指令老化(>1.0s)。状态检测：hookPos/palletPos 与对应端点绝对差<5，连续11个T1回调命中→端态（min对应4=up end，max对应3=down end）；中段持续不动→block=1（仅 hook）。状态机之后发布 /ehb_msg，T1/T2相位不锁定。
- **0x284 byte2 四分支（顺序执行，后写覆盖前写）**：

  | 历史行号（仅定位旧版） | 置值 | 条件 |
  |---|---|---|
  | :90 | 0x05（钩升+盘降） | hookCmd==HOOKOPERATION ∧ hookStatus∉{1,4} |
  | :95 | 0x0A（钩降+盘升） | hookCmd==DECOUPLING ∧ hookStatus≠3 |
  | :100 | 0x01（仅盘降） | hookStatus==4 ∧ palletStatus≠3 |
  | :101 | 0x02（仅盘升） | hookStatus==3 ∧ palletStatus≠4 |

  语义=**挂/脱钩两相续接+机构自收敛**：挂接 0x05→销顶(4)→0x01 落鞍→鞍底(3) 全停；脱钩 0x0A→销底(3)→0x02 升鞍→鞍顶(4) 全停。**注意**：:100/:101 不检查 hookCmd——自动模式下即使无任务，销在端而鞍未到位也会自动补完机构位形；且后写覆盖意味着"销=3∧盘≠4"期间 :101 的 0x02 会覆盖 :90 的 0x05（先升鞍再拉销的归一化次序），"销=4∧盘≠3"期间 0x01 覆盖 0x0A。全组合真值表见 工程同级 `baseline_sweep_reports_20260905.md` 的canbus节（历史文件可能不在当前副本）。
- **RecvCanData**：0x185→刹车 d2/挡位 d3低4/自动 d3.6/急停 d3.7/速度=d7×256+d6 ×3/25.8/60×π×0.66/3.6（高压版公式，实车未发现问题）；0x285→hookState d0.0/linkPallet d0.2/eab d0.3/按钮 d0.4-7/电量 d1/销位 d4/鞍座 d6（epsERR1/2 悬空恒 0）；0x0C02A0A2→转角=(d1×256+d0−15750)/10、epsCurrent、epsMode。
- **启动加载**：`ros::package::getPath("canbus")/config.cfg`，成对校验：hook仍要求0≤min<max≤255且差≥20；pallet两值各在0～255且绝对差≥20，允许反向，拒绝缺键、非整数或越界值（允许行尾`#`注释；重复键以最后一条为准，最后一条非法则该对回退）。坏对回退默认并printf，再发布 `/canbus/hookposition/min|max`、`/canbus/palletposition/min|max`。pnc侧每圈热读四键（task_plan_node:359-366/path_plan_node:245-248）但**读值零消费**（mPositionLimits/HookPos*无业务读者）——留作pallet判定预留。

## 契约（改前必查两侧）

- hookStatus/palletStatus 数值与 pnc `include/common/struct_type.h` HOOKSTATUS_E 锁定：0=移动、1=block、3=down end(销底=脱开)、4=up end(销顶=挂牢)。pnc 完成判定（path_plan_output.inc :443/:452，TASKFINISHED 赋值 :446/:455）、平滑过渡（:79）、心跳映射全依赖它；palletStatus 现役消费者仅 :100/:101 联锁
- 限值语义：min=顶端/max=底端。hook沿用码小=高；pallet允许反向码值，当前上端212、下端115。端态确认由T1 10Hz连续11次计数。
- can_msg.msg 三份同步（canbus/pnc/simview），md5 变→canbus+pnc+ASENSING+simview 整体重编

## EHB 现行协议

唯一协议来源为 [ehb-can.md](docs/ehb-can.md)，09-07以前的SST推断不再使用。

- 消息：`msg/ehb_msg.msg`（**63 字段**=发送表 52+接收表 11；命名报文字段名=ehb-can.md 信号名称原文（含 LosOf/Warn 原拼写），注释=信号描述原文+位号+枚举；**4 个补充项原表信号名为中文，英文名 VHL_VehicleSpeed/VHL_HvPowerState/VHL_BrakePedalStatus/VHL_VehicleGear 系整理命名非原表**）；成员 `mEHBMsg`，RecvCanData 九 case 解析，**T1 10Hz 发布于 `/ehb_msg`**（09-07 最终态：/can_msg 已移回 T2，仅 /ehb_msg 留 T1；monitor仪表板已订阅，另可rostopic echo/录包监控）
- 协议：SAE J1939 29 位 ID，Intel 位序（字节1~8、字节内位1~8、位1=最低位——ehb-can.md §2.2 显式整理约定，非原表文字，验收时确认）；各帧均 DLC=8
- **EHB 发送（整车接收，周期 100ms）**：
  - **帧1 `0x08FB670E` EHB_STORAGE 蓄能**（64359-01~20）：byte0 四个 2bit 状态 + byte1-5 五路压力 raw（×0.1 MPa，0xFE 无效）+ byte6 低 4bit 故障数/高 4bit 起 4 故障位 + byte7 六故障位（8.7-8.8 未分配）
  - **帧2 `0x08FB680E` EHB_AUTOBRAKE 主动制动**（64360-1~18）：byte0 实际压力 + byte1 低 2bit 状态（2.3-2.4 未分配）/高 4bit 故障数 + byte2-3 十五个故障位（3.1-4.7，**顺序=原表行序**；旧版语义推断顺序、EHB_ATB_Reserved 及漏掉的 BrakePressSensorError 均已纠正）
  - **帧3 `0x08FB690E` EHB_EPB 驻车**（64361-1~14）：byte0 状态 2bit+驻车 3bit（1.6-1.8 未分配）+ byte1 驻车液压 + byte2 低 4bit 故障数 + byte3 八故障位 + byte4 低 2bit（**5.1=RcError/5.2=CsError**，旧版颠倒已纠正）
- **整车/VCU(SA=0x58)→EHB（canbus 在 can0 旁听解析，不发送）**：
  - **帧4 `0x08FB1458` Vehicle_Brake_Request**（20ms，64276-01~04）：byte0 请求制动力 raw（**×0.04**=0~8 MPa，勿与反馈 ×0.1 混用）+ byte1 低 2bit 请求标志 + byte6 高 4bit 滚动计数器 + byte7 校验和
  - **帧5 `0x08FB1558` Vehicle_Parking_Request**（100ms，64277-01~03）：byte0 低 2bit 驻车请求（0 无/1 驻车/2 释放/3 无效）+ 计数器 + 校验和
  - **补充项**（无报文头，周期未提供）：`0x0CFD0358` 车速（byte5-6 小端 uint16 ×0.1 km/h，0xFFFF 忽略）/ `0x0CFD0058` 高压上电（byte3：0x00 断电/0x01 上电/0xFF 忽略）/ `0x0CFD0158` 制动踏板（byte4，0-100%）/ `0x08FD0258` 档位（byte5-6 ASCII R,N,D,P，0xFFFF 忽略）；「其他液压管路压力」ID=暂无，未纳入
- 原表冲突注记（ehb-can.md §6）：ID 与隐藏列 PGN/优先级/源地址不一致（I01，**以 D 列报文 ID 为准**——参数编号前缀 64359/64360/64361/64276/64277 与 ID 计算所得 PGN 自洽）；**RollingCounter 声明 8bit/位跨 4bit（I02）→按位置 4bit 解析（0~15，与逻辑范围一致），显式实现假设**；2bit 请求无效值 0x11 越界（I03）；故障数无效值 0xFE/0xE 并存（I04）；校验和求和宽度未定义（I06）→**仅存值不校验**；档位 ASCII 两字节摆放未定义（I09）→存原始 16bit 小端拼接
- 09-07历史验证涵盖九帧63字段、73项定向断言、7项接线和800组随机往返。
  原始工具/完整过程见文档归档；本轮只整理说明，没有重新运行协议测试。
- EHB/VHL字段保留协议名字（含大写与原拼写），不要为了格式统一修改消息字段。

## 已下线（勿加回）

/canbus/hookstate param、path_plan_status 订阅+/planning/alive 清零、相机 0x608/606、0x201 蜂鸣、CAN 断流检测+faultCode 置 1、ADAPTIVEHOOK 循迹 0x0A、一键标定、堵转保护。

## 风险（记录在案）

desireGear∉{2,3,4}（如 GEAR_P=1/异常值）时 can184[6] 以**栈未初始化值**发出；estop 刹车在 N 挡被钳到 20；头注释 whistle=bit3 与夜间代码 +4（bit2）不一致；/can_msg 20Hz（T2）与状态 10Hz（T1）相位不锁——基线既有特性（状态值每拍随 20Hz 双发，数值无碍）；09-07 曾随发布移入 T1 降为 10Hz，实车引发 HMI 健康误报（min_hz 阈值漂移），当晚用户移回 T2 恢复 20Hz，hmi_config min_hz 已复位 20。

## 验证

托盘标定专项：在工程根运行 `python3 src/canbus/tests/verify_position_config.py`。
脚本从真实.msg生成字段桩，复制实际canbus_node.cpp逐字编译，替换ROS与Comply为无I/O桩；
覆盖正反向标定、边界/非法输入、成对回退、启动参数发布、T1全码值扫描及去抖时序，
包括122在下端115标定下保持0。C++11编译、34组配置/启动用例和6422项状态断言通过。
此专项不验证CAN编解码或车端机械端点，车端仍需构建并重启后核对。

历史临时工具：/tmp/canbus_user_verify（桩编译+GATE/EXEC/T1/config行为测试，重建法见
workflow备份09-04条目）；/tmp/ehb_verify（EHB桩+断言）。临时目录可能已失效，
历史验证不能作为本轮重新运行的结果；近期部署待办见工程workflow。
