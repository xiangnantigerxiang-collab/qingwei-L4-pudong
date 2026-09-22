# log_online — 机场无人牵引车事件数据记录节点（README）

## 1. 模块职责

log_online 是与 monitor 并列的根目录独立 Python 模块（纯标准库 + rospy，零编译），满足监管对
"不安全事件发生前后各 ≥90s 关键数据可追溯" 的要求：单进程内以 10Hz 采集七类数据（编号、控制模式、
位置、运行状态、环境感知与响应、灯光信号、外部视频监控情况）入 120s 内存环形缓冲；当急停/故障类
触发条件成立（去抖 0.5s）时，落盘一个"前 90s + 事件中 + 后 90s"的事件目录；内置只读 HTTP 状态页
实时展示车况与链路健康；事件数据超过磁盘上限后自动轮转删旧。设计依据、字段核实出处与取舍决策
（视频只记链路状态不存流、触发仅急停与故障类等）见同目录 `DESIGN.md`。

## 2. 运行方式

```bash
bash log_online/log_online.sh          # 默认 0.0.0.0:8083
LOG_ONLINE_PORT=8090 bash log_online/log_online.sh   # 改端口
# 等价直启: cd log_online && python3 recorder.py    (状态页随进程启动)
```

- 启动脚本仿 `monitor/monitor.sh`：工程根存在 `devel/setup.bash` 才 source，然后 `exec python3 recorder.py`；
- 状态页地址 `http://<车载IP>:8083/`，接口 `/api/status`、`/api/events?limit=20`；
- 端口被占用时仅 stderr 告警并关闭状态页，**事件记录不受影响**；
- 环境变量 `LOG_ONLINE_PORT` 覆盖 `log_online_config.py` 的 `HTTP_PORT`。

**纪律：状态页只读但暴露车况明细，仅限内网调试/运维访问，严禁任何公网映射或端口转发（同 monitor）。**

## 3. 数据字段七类映射（抄自 DESIGN.md §4，字段均已对照 .msg 核实）

| 类别 | 来源 | 字段 |
|---|---|---|
| 编号 | `log_online_config.VEHICLE_ID`（对齐 fms_agent `config.py` 的 `VehicleId="A03"`） | 常量写入每条快照 |
| 控制模式 | `/can_msg`（robot/can_msg，q1） | `controlPanelState`、`eabPanelState` |
| 位置 | `/navigation_msg`（robot/navigation_msg，q1） | `lat lon x y heading rtkState`（altitude/gpsSpeed 在 /navigation_msg 中存在但未纳入快照） |
| 方向/速度 | `/can_msg` | `curGear`（挡位映射 `GEAR_N=2、GEAR_R=3、GEAR_D=4`，`include/common/struct_type.h:64-66`）、`vehicleSpeed`(m/s) |
| 加速度 | 自算 + 对照 | `accel`=vehicleSpeed 差分 EMA（窗口 0.5s，墙钟 dt）；`accel_raw`=navigation_msg `longitudinal_accelerate`（已知无生产者，恒 0，留作对照） |
| 感知 | `/perception`（robot/perception，q1，100Hz 只取最新） | 每 obj：`id type x y dx dy heading height vx vy confidence`（type=hdmap 车道关系 0-4）；截断 `MAX_OBJECTS=64`；汇总 `obj_count`、`nearest_dist`（自车到 obj 中心平面距离最小值，无 obj 时 null） |
| 响应 | `/plan_path_msg`（q1）+ `/control_msg`（q1）+ 参数 | `safety desireSpeed`（plan）；`throttlePercent brakePercent wheelAngle desireAcc`（control）；`sensorstate`（/planning/sensorstate）、`ultra_safe`（/ultra/status/safe） |
| 灯光信号 | 参数（tick 时读） | `/canbus/light`、`/canbus/horn`（path_plan 写，10Hz 读一次 ZOH） |
| 视频 | 相机 CompressedImage（q1，只取最新） | `online`（最近帧年龄 <2s）、`last_frame_age_ms`、`fps`（滑动窗估计）；只记链路状态**不存视频流**；未配置话题时 `online=null`(disabled) |
| 故障 | `/can_msg` + 参数 | `emergencyStop`（can_msg）、`netcheck`（/robot/planning/netcheck）、`faultCode_hex`（can_msg.faultCode 字节→hex 串） |

任一消息断流/超 2s 未更新：该子字段置 `null` 且子对象加 `"stale": true`，记录不中断。

## 4. 事件目录结构

根目录 `log_online/data/events/`（默认），事件目录名 `YYYYmmdd_HHMMSS_<类型>`，同秒冲突加 `_1` 后缀。
类型取首个触发段：`emergencystop / sensorstate / netcheck / faultcode`。

```
log_online/data/events/
└── 20260916_181409_emergencystop/
    ├── metadata.json      # 事件元信息（开启即写，关闭补终版 complete=true）
    └── records.jsonl      # 每行一个 10Hz 快照
```

**metadata.json 样例（实际按 `indent=2` 多行美化输出，下例压缩排版仅示意字段）：**

```json
{"schema_version": 1, "vehicle_id": "A03", "event_dir": "20260916_181409_emergencystop",
 "trigger_segments": [{"start": 1760000000.0, "end": 1760000005.0, "types": ["emergencystop"]}],
 "record_start": 1759999910.0, "record_end": 1760000095.0,
 "pre_window_s": 90.0, "post_window_s": 90.0, "sample_hz": 10, "debounce_ticks": 5,
 "complete": true, "software": "log_online 1.0.0",
 "health_at_trigger": {"can": 0.02, "navigation": 0.01},
 "stats": {"records": 1850, "bytes": 1980000}}
```

**records.jsonl 单行样例（实际为无空格紧凑 JSON）：**

```json
{"ts":1760000000.123,"vehicle_id":"A03",
 "control":{"controlPanelState":0,"eabPanelState":0,"stale":false},
 "position":{"lat":31.15,"lon":121.8,"x":12.3,"y":-45.6,"heading":270.0,"rtkState":"fixed","stale":false},
 "motion":{"gear":4,"speed":1.23,"accel":0.05,"accel_raw":0.0,"stale":false},
 "perception":{"obj_count":3,"nearest_dist":8.2,"objs":[{"id":1,"type":0,"x":10.0,"y":0.5,"dx":4.5,"dy":1.9,"heading":90.0,"height":1.5,"vx":0.0,"vy":0.0,"confidence":0.87}],"stale":false},
 "response":{"safety":0,"desireSpeed":1.5,"throttle":9,"brake":0,"wheelAngle":12.5,"desireAcc":0.0,"sensorstate":0,"ultra_safe":0,"stale":false},
 "lights":{"light":3,"horn":0,"stale":false},
 "video":{"online":true,"last_frame_age_ms":120.0,"fps":15.0,"stale":false},
 "fault":{"emergencyStop":0,"netcheck":0,"faultCode_hex":"","stale":false}}
```

窗口语义：事件 records 首条 `ts <= 首触发 - 90s`；末条 `ts >= 最后清除 + 90s`；post 窗内再触发则
回到记录态、post 计时清零并追加触发段（同一事件目录，窗口扩展）。进程被 kill 时已 flush 的数据保留，
该事件 `complete: false`，重启后不改写、仅列入 `/api/events` 列表。

## 5. 磁盘轮转

事件关闭后与进程启动时检查 `data/events/` 总占用：超过 `DISK_CAP_BYTES=1GiB`（1073741824）即按目录名
升序（时间序）删除最旧事件目录，直至总量降到 `1GiB × ROTATE_KEEP_RATIO=0.9`（约 0.9GiB）水位以下。
删除动作打印 `[log_online] 轮转删除旧事件 <目录> (<bytes> bytes)`。

## 6. 车载部署

1. **同步整目录**：`log_online/` 纯 Python 零编译，整目录同步到车载 `/home/nvidia/qingwei-L4-No2/log_online/`
   （含 `static/index.html`，状态页 404 时只剩接口不可用页面）；
2. **核对相机话题**：状态页视频链依赖 `TOPIC_CAMERA`（默认 `/cam0/compressed`，cam_geac 按 `cam{n}/compressed`
   相对名发布）。上车先核对，对不上就改 `log_online_config.py`：

   ```bash
   rostopic list | grep compressed    # 用实际话题名替换 TOPIC_CAMERA;视频链禁用则置空串 ""
   ```

3. **手动接入 start_l4.sh**（保护清单文件，**默认不改**；以下为建议命令行，需用户授权后代改），
   在 `start_l4.sh` 中仿现有条目追加一行：

   ```bash
   gnome-terminal -t "log_online" -- bash -c "cd /home/nvidia/qingwei-L4-No2; bash log_online/log_online.sh; exec bash"
   ```

   无桌面/调试场景可手动后台拉起：

   ```bash
   cd /home/nvidia/qingwei-L4-No2
   nohup bash log_online/log_online.sh >> log_online/log_online.out 2>&1 &
   ```

   HMI 侧无需改造，浏览器访问 `http://<车载IP>:8083/` 即可（仅内网）。

4. **验收四条**：
   - ① 人工触发一次急停（或等价置 `/can_msg` `emergencyStop`），`data/events/` 生成事件目录，
     records 首/末时间戳距触发起止各 ≥90s；
   - ② 该目录 `metadata.json` 中七类信息（编号/控制模式/位置/运动/感知/响应/灯光/视频/故障）可查；
   - ③ 状态页实时值与 `rostopic echo /can_msg`、`/navigation_msg` 等一致；
   - ④ 磁盘轮转生效：塞满超 1GiB 后最旧事件目录被删，总量回落到 ≤0.9GiB。

## 7. 本机测试（无 ROS，Python 3.8 标准库）

```bash
cd log_online
for f in tests/test_*.py; do python3 "$f" || exit 1; done   # 8 个单元测试文件全 OK
python3 tests/run_smoke.py                                   # 全栈冒烟 -> SMOKE PASS
python3 -m py_compile recorder.py status_server.py log_online_config.py
```

冒烟场景：模拟 0-400s 时间线，t=100 急停 10s、t=200 再触发 5s（落在上一事件 post 窗内=窗口扩展），
断言仅 1 个事件目录、2 个触发段、records 末条 ≥ 最后清除+90s。
