# qingwei L4 monitor（8081 + dashboard 8082）— Web 3D 可视化

替代 simview(rviz)。纯 Python stdlib+rospy，零 pip，部署=拷目录。Three.js r147 vendored（最后保有非 module examples/js 的版本，MIT，离线）。
功能：车位/朝向/速度、障碍、规划/路由线、停车点、托盘、地图三线、2 路 2D 补盲、4 路 3D 感知点云（rviz 未有过）；Orbit 视角+俯视 2D/3D 环视/跟随三预设；图层开关默认态照抄 robot.rviz。

## 部署

```bash
scp -r monitor/ nvidia@<orin-ip>:/home/nvidia/qingwei-L4-No2/
bash monitor/monitor.sh                # 0.0.0.0:8081 + dashboard 8082 同进程起；MONITOR_PORT/DASHBOARD_PORT 改端口；与 hmi(8080) 并存
```

## dashboard 仪表板（8082，2026-09-07 起，当日瓦片化瘦身+布局紧凑化）

- `/`（static/dashboard.html，2Hz 轮询）+ `/api/dashboard`；与主服务**同进程**同一 rospy 节点，复用 RosVisualizer 订阅与 stash（ros_visualizer `/ehb_msg` 订阅与 `latest()/master_ok()/ros_available()` 取数接口）；绑定失败仅告警不影响 8081；`DASHBOARD_PORT=0` 关闭
- **布局**：两卡靠左不拉伸（can ~160px 瓦片单列微缩、ehb ~575px 瓦片 4 列小号），右侧大片留白预留给未来新增信息面板（窄屏 flex-wrap 自动换行）；卡片带 `can/ehb` 变体类，JS 改 className 时须保留
- **凹凸拟物瓦片**（凸起卡片+凹陷值槽双向柔影），payload 即瘦身后形态：
  - 隐藏字段：can=rawcommand/rawfeedback（原始帧 hex）/epsERR1/2（悬空恒 0）/faultCode（写路径 09-03 已下线）；ehb=RollingCounter/CheckSum 四个 → **can 22 瓦片、ehb 27 瓦片**（原 27/63 行）
  - **故障位折叠**：35 个 `*_FAU_*` bool 聚合为分节首行汇总条三态——绿"全部正常"/红"N 项 · 点名"/灰"无数据"（**msg 缺或任一位属性不可读即"无数据"，不得假"全部正常"**；单字段格式化异常降级字符串，不炸端点）
  - 注释只留换算/单位：ehb=`_conv_note`（×0.1 MPa/×0.04 MPa/×0.1 km/h/0-100%），can=单位（m/s/%/°/A；含 `=` 的枚举括注丢弃）；**枚举译码仍用 .msg 原注释——先译码后裁剪，顺序不可倒**
  - 译码：ehb uint8 三种注释键形态（二进制等宽 `00:`/十六进制 `0x0:`/十进制 `0:`，区间 `0x4~0x6:` 跳过）；can 用 `CAN_ENUMS` 显式表（挡位/急停/按钮/状态机，**改 can_msg 字段语义需同步**）；`VHL_VehicleGear` 按 `_decode_gear` 出 R/N/D/P（uint16 双字节恰一个字母才译，摆放歧义不译）
  - 分节标题精简（"帧1 蓄能系统"/"帧4 制动请求"，去 0x ID/位号/括注）
- **中文名称对齐策略**：/ehb_msg 启动时运行时解析 `../src/canbus/msg/ehb_msg.msg` 注释（构造性对齐——改 .msg 重启 monitor 即跟上；读取 utf-8 优先 GBK 兜底）；/can_msg 的 msg 无中文注释，用 `dashboard.py` 内 `CAN_LABELS` 的 monitor 侧命名（语义出处 src/canbus/README.md，**can_msg 改字段需同步该表**）
- 数据龄徽章：ROS/master/两话题三态（绿=5s 内新鲜、黄=停滞、灰=无数据）；**停滞（age>5s，发布者死亡）瓦片半透明+卡片标题标"数据停滞"**，冻结旧值不再伪装成健康
- **失败隔离**：dashboard 任何启动失败只 stderr 告警，主 8081 服务不受影响——端口占用/越界、"DASHBOARD_PORT 非法值" 告警并关闭（空串=未设，回退配置值不遮蔽；值 0=显式关闭）；`.msg` 缺失/坏编码 stderr 告警 + API 带 `msg_load_errors` + 页面显示"无字段定义"。前端侧总断连（API 挂/JSON 损坏）四点全红+龄清空+卡片停滞，不残留最后一拍全绿

## 与 simview 语义对照（差异即约定）

- yaw=(90°−heading) 一致；障碍 dx/dy 互换一致（半透明+抬 h/2 为有意改进）；路径 x/y 不等长整帧丢弃一致；停车点 stopAngle 用于朝向（C++ 恒指东，增强）；托盘不含 C++ 死偏移 hook_xg−2.8
- **障碍物按 `/perception` objs 的 type 着色（3D CUBE 与降级 2D 同规则）**：0=红 / 1=橙 / 2=黄 / 其余（含字段缺失）墨绿 `0x339999`（历史默认色）。type 语义由上游决定：hdmap `LaneMapServer::ClassifyPerception`（09-15）车道占用 0-本道/1-左一/2-左二/3-左外/4-右外（外道落墨绿）；object.msg 旧注释为 0-车/1-行人/2-骑行/3-未知——两套语义下本配色均成立
- **置信度文字（09-16 字号减半）**：`object.msg` 的 `confidence` 以白色文字显示在框上方，保留目前减半后的显示尺寸和 `h+0.3` 锚点；缺失时不显示。3D 与运动信息共用字形纹理和一个绘制批次，2D 降级为 18px 直立文字。
- **障碍运动信息（09-16）**：在 confidence 上方从上到下显示 `id`、`speed`、`heading` 三行白字，只保留数值和单位（如 `42` / `5.00 m/s` / `36.87°`）。`speed=hypot(vx,vy)`，单位 m/s；heading 显示上游方位角，单位度。速度和航向保留两位小数，与 confidence 字号一致。3D 字形为 42px，按原来半字号的世界尺寸显示，行距仍为 0.9m；偏移在相机平面内计算，俯视/环视均保持直立和上下顺序。2D 字号为 18px，行距为 22px；随 lidar 图层显隐，confidence 缺失仍显示运动信息。
- 地图：map/ 下全部 .csv 按名排序全画；rosparam /robot/mapfile 指单文件时只画该张
- 2D 补盲按通道左蓝右绿（intensity 未参与着色）；不订阅 /planning/obstacles（死话题）

## 2D 补盲外参（上实车一次）

lakibeam 两路 frame_id 同为 `laser` 且工程内无 tf → 服务端外参变换，配置在 `monitor_config.py` 的 `SCAN_EXTRINSICS`（x/y 米 + yaw_deg）。标定：已知距离障碍对照页面弧位调至吻合。

## CPU/带宽（monitor_config.py 的 CLOUD 段）

4 路 PointCloud2 typed 订阅空转 12-40MB/s 是 CPU 大头，故：活动门控（30s 无浏览器拉取自动退订）/每路 1s 解析（parse_interval）/解包前 stride 预抽稀（默认 2）/0.2m 体素每路上限 8000 点。总 ≈380KB/帧@1Hz。占用偏高→调大 stride/parse_interval 或调粗 voxel；实测 `top`+hmi 面板 CPU。

09-16 实车卡顿排查后，障碍四行文字改为**单张静态字形纹理、单个批量绘制对象**。数值变化仅更新复用的顶点和 UV 缓冲，不再按障碍重画 canvas 或上传整张纹理。100 个障碍的本机浏览器对照：文字 RGBA 像素数据量由约 74.9MiB 降到约 0.90MiB，文字绘制调用由 200 次降至 1 次；纹理大小不随障碍数量增长。数字、单位、白色/描边、字号及位置保持现有要求。测试场景不含地图/点云，浏览器使用 SwiftShader；此结果说明资源开销下降，不代表 Orin 实测帧率或整车控制时延。纯前端更新，部署后强制刷新页面以替换缓存。记录见 [performance_verification_20260916.json](tests/performance_verification_20260916.json)。

## 设计否决（勿重提）

SSE/WebSocket 推送（base64+33% 或非 stdlib）；gzip（噪声态压缩率仅 20-30%，不划算）；路径版本号门控（10Hz receding horizon 每帧必变，反引入陈旧视图）。

## 排障（状态点灰/无数据）

```bash
curl -s -m5 http://127.0.0.1:8081/api/snapshot | head -c300   # 有 JSON=服务端正常→看浏览器侧；卡/空=服务端问题
grep -c getBigUint64 static/index.html                        # 应为 0
grep -c "3D 初始化失败" static/index.html                     # 应为 1（旧副本缺修复）
```

- 浏览器 F12+左上错误框自报：3D 失败→自动降级完整 2D（WebGL 修复法见 hmi/README「Firefox WebGL 排查」）；JS 错误带行号；红色横幅=/api/snapshot 不通
- roscore 未起=正常降级非故障（master 红+黄横幅，起栈自愈）
- 2D 渲染按数据变化重画限 10FPS（Jetson 防 CPU）

## 坐标系（前端 MonitorMath，单测锁定）

ROS(x东,y北,z上)→THREE(x右,y上,z南)：`three=(x_ros, z_ros, −y_ros)`；`rotation.y=yaw_ros` **不取负**（取负是最常见翻车点）；车辆/障碍 `yaw=normalize(90°−heading)·π/180` 与 simviewer 一致。

## 二进制协议（scan.bin / cloud.bin）

小端。头 20B：`QWMC`(4)+version u16+通道数 u16+总点数 u32+时刻ms u64；通道子头 8B：id u8+flags u8+保留 u16+点数 u32；点 12B：x_mm i32+y_mm i32+z_mm i16+intensity u8+pad u8。车体坐标（前端挂车辆组随位姿动）。Py/JS 双解码被同一 fixture 字节锁锁死（test_full t05 / frontend F4）。

## 测试（开发机；车载不部署 tests/）

```bash
python3 tests/test_full.py      # 74 项伪 ROS 全栈
python3 tests/test_dashboard.py # 110 项仪表板（解析对齐/枚举译码及顺序锁/瘦身契约三态/单字段降级/GBK/端口语义/占用守卫/全栈）
node tests/frontend_dash_test.js # 51 项仪表板前端无头（瓦片渲染/故障三态+闪烁/降级 sections=null/ros off 态/总断连全灭）
python3 tests/edge_test.py      # 28 项边界值
node tests/frontend_test.js     # 91 项前端检查，含字形/UV 还原、100 障碍纹理复用、图层与 2D 降级
python3 tests/chaos_test.py     # 11 项混沌/浸泡（约 2 分钟）
python3 tests/fuzz_proto.py     # 协议双解码对账 fuzz（50 轮）
bash monitor.sh                 # 起服务（8081+8082），无 ROS 环境横幅
```

注：测试的伪 master 端口已改 21111~21113（本机 11311 被 root 的真实 rosmaster 占用，见 workflow 09-07）。

## 文件

monitor.sh｜monitor_config.py（话题/外参/降采样/图层默认态/DASHBOARD_PORT）｜monitor_server.py（HTTP+4 API+dashboard 拉起）｜ros_visualizer.py（rospy 桥/快照/地图/二进制）｜dashboard.py（8082 仪表板：.msg 解析/中文名称/枚举译码/API）｜map/view.csv（拷自 simview）｜static/index.html（前端单页）｜static/dashboard.html（仪表板单页）｜three.min.js+OrbitControls.js｜tests/
