# canbus — CAN 收发（用户自研版，2026-09-03 起；基线 2026-09-05 实车部署版）

> 原版 canbus_comply 架构；canbus_core 重构版已下线（历史见工程根 workflow.md）。
> 启动：`launch/canbus.sh`（modprobe+can0 250k → canbus.launch → socketcan_bridge[/can_send,/can_recv] + aeb_can_node）。

## 结构

| 文件 | 职责 |
|---|---|
| `src/canbus_node.cpp` | main（节点名 can_node，launch 重命名 canbus）+T1(10Hz)+T2(20Hz)+LoadPositionConfig；限值四全局变量 |
| `src/canbus_comply.{h,cpp}` | VehicleComm(组帧)+RecvCanData(解析)+相机死方法 |
| `src/struct_type.h` | pnc 同名副本（仅多无读者的 GEAR_P） |
| `msg/` | 26 个 .msg，**CMake 生成 18 个**（fault_msg/keypoints/object1/obstac/RoutingPath/task_plan_status/vehicle_task_status/visionObjects 未列不生成）；can_msg/can_comm_msg 与 pnc/simview(包名 view) 必须同步改 |
| `config.cfg` | 限值四键（182/254/213/254），启动读取 |

## 运行逻辑

- **T2 20Hz（VehicleComm+发布 /can_msg）**：手动模式发中性帧即返回；自动模式发 0x184（油门/刹车/转角 ±700 钳位/挡位+EPB：D,R=0x10|g，N=0x20|g）与 0x284（byte2 见下表；灯光 /canbus/light 3=左 4=右+夜间 +4=bit2；/canbus/time param）。约束链：**estop→Brake=80（唯一急停值，指令老化也并入此通道）**；Brake>0→油门/转角清零；N 挡油门清零且刹车钳 20（estop 的 80 也受此钳）。发布 /can_msg（09-07 晚用户自 T1 移回，恢复基线 20Hz）。
- **T1 10Hz（状态机+发布 /ehb_msg）**：EmergencyStop = sensorstate≠0 ∨ netcheck≠0 ∨ !planning_alive ∨ 指令老化(>1.0s)。状态检测：hookPos/palletPos 入 min/max±5 带 >1s（10 tick）→端态（4=up end/3=down end，hook 与 pallet 同向）；中段 1s 不动→block=1（仅 hook）。状态机之后发布 /ehb_msg（09-07 两度调整的最终态：用户先把 /ehb_msg、/can_msg 一并自 T2 移入 T1，当晚又将 /can_msg 移回 T2 恢复 20Hz——T1 仅余 /ehb_msg@10Hz）。
- **0x284 byte2 四分支（顺序执行，后写覆盖前写）**：
  | # | 行 | 置值 | 条件 |
  |---|---|---|---|
  | :90 | 0x05（钩升+盘降） | hookCmd==HOOKOPERATION ∧ hookStatus∉{1,4} |
  | :95 | 0x0A（钩降+盘升） | hookCmd==DECOUPLING ∧ hookStatus≠3 |
  | :100 | 0x01（仅盘降） | hookStatus==4 ∧ palletStatus≠3 |
  | :101 | 0x02（仅盘升） | hookStatus==3 ∧ palletStatus≠4 |

  语义=**挂/脱钩两相续接+机构自收敛**：挂接 0x05→销顶(4)→0x01 落鞍→鞍底(3) 全停；脱钩 0x0A→销底(3)→0x02 升鞍→鞍顶(4) 全停。**注意**：:100/:101 不检查 hookCmd——自动模式下即使无任务，销在端而鞍未到位也会自动补完机构位形；且后写覆盖意味着"销=3∧盘≠4"期间 :101 的 0x02 会覆盖 :90 的 0x05（先升鞍再拉销的归一化次序），"销=4∧盘≠3"期间 0x01 覆盖 0x0A。全组合真值表见 `../baseline_sweep_reports_20260905.md` canbus 节。
- **RecvCanData**：0x185→刹车 d2/挡位 d3低4/自动 d3.6/急停 d3.7/速度=d7×256+d6 ×3/25.8/60×π×0.66/3.6（高压版公式，实车未发现问题）；0x285→hookState d0.0/linkPallet d0.2/eab d0.3/按钮 d0.4-7/电量 d1/销位 d4/鞍座 d6（epsERR1/2 悬空恒 0）；0x0C02A0A2→转角=(d1×256+d0−15750)/10、epsCurrent、epsMode。
- **启动加载**：`ros::package::getPath("canbus")/config.cfg`，成对校验（0≤min<max≤255 且窗≥20，坏对回退默认并 printf）→ rosparam 发布 /canbus/hookposition|min|max、/canbus/palletposition|min|max。pnc 侧每圈热读四键（task_plan_node:359-366/path_plan_node:245-248）但**读值零消费**（mPositionLimits/HookPos* 无业务读者）——读在值不在的死管道，留作 pallet 判定预留。

## 契约（改前必查两侧）

- hookStatus/palletStatus 数值与 pnc `include/common/struct_type.h` HOOKSTATUS_E 锁定：0=移动、1=block、3=down end(销底=脱开)、4=up end(销顶=挂牢)。pnc 完成判定（path_plan_output.inc :443/:452，TASKFINISHED 赋值 :446/:455）、平滑过渡（:79）、心跳映射全依赖它；palletStatus 现役消费者仅 :100/:101 联锁
- 限值极性：**码小=高**（min=顶端/max=底端）；带滞 1s 由 T1 10Hz 计
- can_msg.msg 三份同步（canbus/pnc/simview），md5 变→canbus+pnc+ASENSING+simview 整体重编

## EHB 解析（2026-09-07 晚二版：两张信号表全纳入；唯一协议来源=docs/ehb-can.md，旧 SST 锚定版作废）

- 消息：`msg/ehb_msg.msg`（**63 字段**=发送表 52+接收表 11；命名报文字段名=ehb-can.md 信号名称原文（含 LosOf/Warn 原拼写），注释=信号描述原文+位号+枚举；**4 个补充项原表信号名为中文，英文名 VHL_VehicleSpeed/VHL_HvPowerState/VHL_BrakePedalStatus/VHL_VehicleGear 系整理命名非原表**）；成员 `mEHBMsg`，RecvCanData 九 case 解析，**T1 10Hz 发布于 `/ehb_msg`**（09-07 最终态：/can_msg 已移回 T2，仅 /ehb_msg 留 T1；topic 暂无读者，rostopic echo/录包监控用）
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
- 验证：/tmp/ehb_verify 桩（重启失，stub 头+gen_msg_stub.py+verify_layout.py 重建即可）：九帧 63 字段 python 重解析交叉核对 + 定向断言 73/73（含 0x185/0x285/0x0C02A0A2 回归）+ T1/T2 发布接线 7/7 + 随机往返 800 组×9 帧全过
- 字段名大写开头（EHB_*/VHL_*）**已对照 noetic genmsg 源码确认合法**：`names.py BASE_RESOURCE_NAME_LEGAL_CHARS_P=^[A-Za-z][\w_]*$`（大小写均可，ROS2 才强制小写）；尾注释 `# ...` 经 `_strip_comments`（`split('#')[0]`）合法；字段行按空格二分、63 名无重复（MsgSpec 查重）——此三项均已核
- 提取方法存档（历史，结论已被 ehb-can.md 取代）：WPS 变体 BIFF 锚点扫描法恢复 SST 绑定——其"ATB 位序推断""接收方向真帧=FD 系列"判断与用户整理的 ehb-can.md 不符，勿再引用

## 已下线（勿加回）

/canbus/hookstate param、path_plan_status 订阅+/planning/alive 清零、相机 0x608/606、0x201 蜂鸣、CAN 断流检测+faultCode 置 1、ADAPTIVEHOOK 循迹 0x0A、一键标定、堵转保护。

## 风险（记录在案）

desireGear∉{2,3,4}（如 GEAR_P=1/异常值）时 can184[6] 以**栈未初始化值**发出；estop 刹车在 N 挡被钳到 20；头注释 whistle=bit3 与夜间代码 +4（bit2）不一致；/can_msg 20Hz（T2）与状态 10Hz（T1）相位不锁——基线既有特性（状态值每拍随 20Hz 双发，数值无碍）；09-07 曾随发布移入 T1 降为 10Hz，实车引发 HMI 健康误报（min_hz 阈值漂移），当晚用户移回 T2 恢复 20Hz，hmi_config min_hz 已复位 20。

## 验证

本机：/tmp/canbus_user_verify（桩编译+GATE/EXEC/T1/config 行为测试，重启失，重建法见 workflow 备份 09-04 条目）；/tmp/ehb_verify（EHB 桩+断言，重启失）。车载核对清单见 workflow「四、车载部署」。
