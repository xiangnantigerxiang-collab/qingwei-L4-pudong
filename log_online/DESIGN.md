# log_online 设计文档（事件数据记录与在线监控节点）

日期：2026-09-16 ｜ 状态：已实施（2026-09-16，终审后含修复波） ｜ 形态：仿 monitor 的根目录独立 Python 模块

## 1. 背景与目标

监管要求：车辆具备在线监控能力，并能记录**不安全事件或失效状况发生前至少 90 秒和发生后至少 90 秒**的关键数据。数据至少包含：

1. 无人驾驶行李牵引车编号
2. 控制模式
3. 车辆位置
4. 运行状态参数（至少：行驶方向、行驶速度、加速度）
5. 环境感知与响应状态
6. 灯光、信号实时状态
7. 外部视频监控情况

## 2. 范围

**做**：独立 Python 进程 `log_online`，10Hz 采集七类数据入内存环形缓冲；触发条件成立时落盘"前 ≥90s + 事件中 + 后 ≥90s"事件目录；内置 HTTP 状态页实时展示；磁盘上限自动轮转。

**不做（用户已决策）**：
- 视频只记录链路状态（在线/离线/帧率/最近帧年龄），**不存储视频流**；
- 触发仅限**急停与故障类**（emergencyStop / sensorstate / netcheck / faultCode），安全停车类、数据链失效类、云端链路类不触发（后续可扩）；
- 不修改 monitor / start_l4.sh / HMI（保护清单，只给手动接入说明）。

## 3. 形态与目录结构

单进程双线程：

- **主线程**：rospy 订阅回调（仅更新"最新值缓存"）+ `rospy.Timer` 10Hz tick（读参数→构建快照→状态机→写盘）；
- **HTTP 线程**（daemon）：ThreadingHTTPServer 状态页，默认 `0.0.0.0:8083`（`LOG_ONLINE_PORT` 可配；绑定失败仅 stderr 告警，不退出记录）；
- 共享：锁保护的最新值字典；快照为不可变 dict 拷贝。

```
log_online/
├── log_online.sh          # 启动包装（仿 monitor.sh：devel/setup.bash 存在才 source）
├── log_online_config.py   # 全部配置（见 §11）
├── recorder.py            # Recorder：订阅缓存 + tick + 环形缓冲 + 事件状态机 + 落盘/轮转
├── status_server.py       # StatusServer：/api/status /api/events + static 托管
├── static/index.html      # 状态页（原生 JS，2s 轮询）
├── DESIGN.md              # 本文档
├── README.md              # 运行/部署/语义对照（实施时写）
├── data/events/           # 事件数据根目录（可配，默认工程根下）
│   └── 20260916_181409_emergencystop/
│       ├── metadata.json
│       └── records.jsonl
└── tests/
    ├── mock_ros.py        # rospy/消息桩（仿 monitor tests/mock_ros.py 做法）
    ├── test_recorder.py
    ├── test_status.py
    └── run_smoke.py       # mock 全栈事件生命周期冒烟
```

## 4. 数据源与字段映射（字段均已对照 .msg 核实）

| 类别 | 来源 | 字段 |
|---|---|---|
| 编号 | `log_online_config.VEHICLE_ID`（对齐 fms_agent `config.py` 的 `VehicleId="A03"`） | 常量写入每条快照 |
| 控制模式 | `/can_msg`（robot/can_msg，q1） | `controlPanelState`、`eabPanelState` |
| 位置 | `/navigation_msg`（robot/navigation_msg，q1） | `lat lon xAxis yAxis heading rtkState`（altitude/gpsSpeed 在消息中存在但未纳入快照） |
| 方向/速度 | `/can_msg` | `curGear`（挡位枚举映射以 pnc `GEAR_*` 为准实施时核实）、`vehicleSpeed`(m/s) |
| 加速度 | 自算 + 对照 | `accel`=vehicleSpeed 差分 EMA（窗口 0.5s，墙钟 dt）；`accel_raw`=navigation_msg `longitudinal_accelerate`（已知无生产者，恒 0，留作对照） |
| 感知 | `/perception`（robot/perception，q1，100Hz 只取最新） | 每 obj：`id type x y dx dy heading height vx vy confidence`（type=hdmap 车道关系 0-4）；截断 `MAX_OBJECTS=64`；汇总 `obj_count`、`nearest_dist`（自车到 obj 中心平面距离最小值，无 obj 时 null） |
| 响应 | `/plan_path_msg`（q1）+ `/control_msg`（q1）+ 参数 | `safety desireSpeed`（plan）；`throttlePercent brakePercent wheelAngle desireSpeed desireAcc`（control）；`sensorstate`（/planning/sensorstate）、`ultra_safe`（/ultra/status/safe） |
| 灯光信号 | 参数（tick 时读） | `/canbus/light`、`/canbus/horn`（path_plan 写，10Hz 读一次 ZOH） |
| 视频 | 相机 CompressedImage（q1，只取最新） | `online`（最近帧年龄 <2s）、`last_frame_age_ms`、`fps`（滑动窗估计）；话题名实施时从 cam_geac demo 源码确认后写入 config 默认值，未配置时 `online=null`（disabled） |
| 故障 | `/can_msg` + 参数 | `emergencyStop`（can_msg）、`netcheck`（/robot/planning/netcheck）、`faultCode_hex`（can_msg.faultCode 字节→hex 串） |

所有订阅带 `tcpNoDelay()`。每条消息记录 `last_seen` 供链路健康判定（stale 阈值统一 2.0s）。

## 5. 快照 schema（records.jsonl 每行一个，10Hz）

```json
{
  "ts": 1760000000.123,
  "vehicle_id": "A03",
  "control":  {"controlPanelState": 0, "eabPanelState": 0},
  "position": {"lat": 31.15, "lon": 121.8, "x": 12.3, "y": -45.6, "heading": 270.0, "rtkState": "fixed"},
  "motion":   {"gear": 2, "speed": 1.23, "accel": 0.05, "accel_raw": 0.0},
  "perception": {"obj_count": 3, "nearest_dist": 8.2,
                 "objs": [{"id":1,"type":0,"x":10.0,"y":0.5,"dx":4.5,"dy":1.9,"heading":90.0,"height":1.5,"vx":0.0,"vy":0.0,"confidence":0.87}]},
  "response": {"safety": 0, "desireSpeed": 1.5, "throttle": 9, "brake": 0,
               "wheelAngle": 12.5, "desireAcc": 0.0,
               "sensorstate": 0, "ultra_safe": 0},
  "lights":   {"light": 3, "horn": 0},
  "video":    {"online": true, "last_frame_age_ms": 120, "fps": 15.0},
  "fault":    {"emergencyStop": 0, "netcheck": 0, "faultCode_hex": ""}
}
```

消息缺失/超 2s 未更新 → 对应子字段置 `null` 且该子对象加 `"stale": true`，快照照常产出。

## 6. 环形缓冲

`collections.deque`，每 tick append 当前快照并从头弹出 `ts < now - RING_SECONDS(120)` 的元素。内存典型 ~1-2MB；感知满载（MAX_OBJECTS=64）时单条可达 10-30KB，满环最坏 ~15-30MB（Jetson 可承受）。

## 7. 事件状态机

**触发条件**（tick 时对快照判定，任一成立即 trigger_active）：
`fault.emergencyStop != 0` ∨ `response.sensorstate != 0` ∨ `fault.netcheck != 0` ∨ `fault.faultCode_hex != ""`；
字段为 None（数据缺失/stale）按 0 处理**不触发**（避免开机无数据误开事件），stale 状态本身记录在数据里。

**状态与转移**：

| 状态 | 行为 | 转移 |
|---|---|---|
| IDLE | 仅维护环形缓冲 | trigger_active 连续 `DEBOUNCE_TICKS=5`（0.5s）→ RECORDING（建事件目录，写 metadata 初始版） |
| RECORDING | 落盘缓冲中 `ts ≥ trigger_start - PRE_WINDOW_S(90)` 的全部快照，随后每 tick 追加写 | trigger_active=0 → POST（开始计 post 计时） |
| POST | 继续每 tick 追加 | 累计 `POST_WINDOW_S(90)` 且期间无 trigger → 关闭事件 → IDLE；trigger_active 复活 → 回 RECORDING（post 计时清零，窗口扩展，segments 追加） |

- 事件类型：首个触发段的类型名（`emergencystop/sensorstate/netcheck/faultcode`），后续触发段类型记入 segments；
- 目录名 `YYYYmmdd_HHMMSS_<type>`，同秒冲突加 `_1` 后缀；
- 去抖计数在 trigger 消失时立即清零；
- 进程被 kill：已 flush 的 jsonl 保留，metadata `complete:false`（下次启动扫描发现未完成事件不改写，仅列入列表）。

## 8. 落盘格式

**records.jsonl**：每行一个快照（§5 schema）。事件开启时一次性写入 pre 段；之后每 `FLUSH_INTERVAL_S=1.0` flush 一次；关闭时 flush + fsync + 补写最终 metadata。

**metadata.json**：

```json
{
  "schema_version": 1,
  "vehicle_id": "A03",
  "event_dir": "20260916_181409_emergencystop",
  "trigger_segments": [{"start": 1760000000.0, "end": 1760000005.0, "types": ["emergencystop"]}],
  "record_start": 1759999910.0,
  "record_end": 1760000095.0,
  "pre_window_s": 90, "post_window_s": 90, "debounce_ticks": 5, "sample_hz": 10,
  "complete": true,
  "software": "log_online 1.0.0",
  "health_at_trigger": {"can": 0.02, "navigation": 0.01, "perception": 0.1},
  "stats": {"records": 1850, "bytes": 1980000}
}
```

## 9. 磁盘轮转

事件关闭后与进程启动时执行：遍历 `data/events/` 各事件目录累计大小（实现为每次检查时全量重扫——事件目录数小，代价可忽略），超过 `DISK_CAP_BYTES`（默认 1GiB=1073741824）→ 按目录名升序删除最旧直至总量 ≤ `cap × 0.9`。删除失败（rmtree 后复查仍存在）stderr 告警、不虚扣水位并继续。

## 10. 状态页（默认 8083）

- `GET /` → `static/index.html`
- `GET /api/status` → 当前最新快照（§5 schema）+ `health`（各链 last_seen 年龄/hz 估计）+ `trigger`（各条件实时值 + 状态机状态）+ `ring`（条数/时长）+ `disk`（used/cap）
- `GET /api/events?limit=20` → 最近事件 metadata 摘要列表（倒序）
- 页面三块：七类实时数据表、链路健康灯、最近事件表；原生 JS 2s 轮询；无框架无外部资源
- **纪律：仅内网使用，严禁公网映射**（同 monitor）

## 11. 配置项（log_online_config.py，全部带默认值）

| 键 | 默认 | 说明 |
|---|---|---|
| `VEHICLE_ID` | `"A03"` | 车辆编号 |
| `TOPIC_CAN/NAVIGATION/PERCEPTION/PLAN/CONTROL` | `/can_msg` `/navigation_msg` `/perception` `/plan_path_msg` `/control_msg` | 订阅话题 |
| `TOPIC_CAMERA` | `/cam0/compressed` | cam_geac 按 `cam{n}/compressed` 相对名发布（`mgr_camear_jpegenc.cpp:110`）；实车 `rostopic list` 核对后可改；空串=视频状态 disabled |
| `PARAM_SENSORSTATE/NETCHECK/ULTRA_SAFE/LIGHT/HORN` | `/planning/sensorstate` `/robot/planning/netcheck` `/ultra/status/safe` `/canbus/light` `/canbus/horn` | tick 热读参数 |
| `SAMPLE_HZ` | 10 | 快照频率 |
| `RING_SECONDS` | 120 | 环形缓冲时长（>90 需求 + 30s 余量） |
| `PRE_WINDOW_S` / `POST_WINDOW_S` | 90 / 90 | 需求窗口 |
| `DEBOUNCE_TICKS` | 5 | 触发去抖（0.5s） |
| `FLUSH_INTERVAL_S` | 1.0 | 落盘 flush 周期 |
| `DISK_CAP_BYTES` | 1073741824 | 事件数据磁盘上限 |
| `ROTATE_KEEP_RATIO` | 0.9 | 轮转目标水位 |
| `MAX_OBJECTS` | 64 | 单快照障碍物上限 |
| `STALE_AFTER_S` | 2.0 | 链路 stale 阈值 |
| `ACCEL_EMA_S` | 0.5 | 自算加速度平滑窗口 |
| `HTTP_HOST` / `HTTP_PORT` | `0.0.0.0` / 8083 | 状态页（env `LOG_ONLINE_PORT` 覆盖端口） |
| `DATA_DIR` | `<工程根>/log_online/data/events` | 事件根目录（ROOT 定位仿 monitor.sh） |

## 12. 错误处理与降级

| 情形 | 行为 |
|---|---|
| 任一消息断流 | 子字段 null + stale，记录不中断 |
| 相机无数据/未配置 | video.online=false 或 null(disabled) |
| 磁盘写失败 | stderr 告警，保持内存缓冲继续，下 tick 重试 |
| HTTP 端口占用 | 告警后状态页关闭，记录不受影响 |
| 进程 kill -9 | 已 flush 数据保留，metadata `complete:false` |
| 异常大感知包 | objs 截断至 MAX_OBJECTS，obj_count 记原始数 |

## 13. 测试计划（本机 python3 无 ROS 全可跑）

mock_ros 注入假 rospy/消息类（仿 monitor tests）：

1. 快照 schema 七类字段完整性与默认值；
2. 环形缓冲 120s 裁剪（旧元素弹出）；
3. 去抖：4 tick 触发不开事件、5 tick 开；
4. **pre 窗口：事件 records 首条 ts ≤ trigger_start − 90s**；
5. **post 窗口：records 末条 ts ≥ trigger_clear + 90s**；
6. 窗口扩展：post 期间再触发，post 计时归零、segments 正确；
7. 轮转：超限删最旧、水位 0.9；
8. faultCode 字节→hex；
9. 断流→null+stale；相机无数据→offline；
10. 加速度 EMA 数值正确性（固定速度序列）；
11. `/api/status` `/api/events` 结构断言；
12. 全栈冒烟：模拟 200s 数据流含一次 10s 急停，校验事件目录、行数、首末时间戳、complete=true。

## 14. 部署与验收

- 车载：同步 `log_online/` 整目录（纯 Python 零编译），`bash log_online/log_online.sh` 启动；
- 接入 start_l4.sh / HMI 为保护清单文件，**默认不改**；手动接入方式写入 README（等用户授权后可代改）；
- 验收标准：①人工触发一次急停，事件目录生成且 records 首/末时间戳距触发起止 ≥90s；②metadata 七类信息可查；③状态页实时值与 rostopic echo 一致；④磁盘轮转生效。

## 15. 已知边界与后续扩展口

- 视频不存流（用户决策）；后续如需：环形缓冲 JPEG 序列方案已在设计讨论中存档；
- 触发类仅急停/故障（用户决策）；安全停车/链路失效/云端类可在 TriggerEvaluator 处扩表；
- `accel_raw` 恒 0（上游无生产者，workflow 风险 13），自算 `accel` 为准；
- 节点启动后不足 90s 就触发时，pre 段只含启动以来的数据（`record_start` 如实反映，不伪造）；
- 挡位映射已核实（`include/common/struct_type.h:64-66`）：`GEAR_N=2、GEAR_R=3、GEAR_D=4`，页面按此映射显示文案。
