# workflow.md — 工作延续文档

> **用途**：每次开始工作前先通读本文件，了解历史工作与工程现状，保证工作可以延续。
> **维护约定**：每完成一项工作，在「二、工作日志」追加一节，并同步更新「三、当前状态」和「四、下一步」。
> 工作目录：`/home/mothotob/work/projects/claude-code/qingwei-L4-No2-pudong-rebuild/qingwei-L4-No2-pudong/`（非 git 仓库，下称"工程根目录"）

---

## 一、工程概况（2026-08-19 完成全量分析，结论可直接复用，无需重新分析）

### 1. 项目定位
**机场 L4 无人电动牵引车（拖车）整车软件**，场景为浦东机场，业务闭环：云端 FMS 调度 → 循迹行驶 → 自动挂接/脱开托盘。车载部署于 NVIDIA Jetson Orin（aarch64，ROS1 + Python3.8，车载路径 `/home/nvidia/qingwei-L4-No2`，本目录是其重建副本）。由 `/home/qwrf/mogu-master` 项目衍生（launch 中仍有残留引用）。

### 2. 数据流（核心链路）
```
云端FMS ─MQTT(39.105.47.169:1883)→ fms_agent(Python) → /cloud/task/... → pnc任务层
5×RoboSense雷达(2×RSAIRY左右+3×RSM1中前后) → 4路点云拼接 → CenterPoint(TensorRT FP16
+libspconv SCN) → /box(MarkerArray) → perception_convert(+/localization) → /perception → pnc规划
3×LakiBeam1 2D雷达 → auto_couple(反光板配对) → /hook_position → 倒车对挂路径
ASENSING RTK/IMU → /localization → navigation(50Hz)
pnc: task_plan → path_plan(10Hz) → control(20Hz) → can_comm → canbus(socketcan can0 250kbps)
     下发0x184/0x284/0x201/0x608，反馈0x185/0x285/0x401
辅助：simview(RViz可视化)、data_logger(10Hz行车CSV→logData/)、calib(标定工具)
```

### 3. 包清单（src/ 下，清理后 9 个）
| 包 | 职责 | 规模 |
|---|---|---|
| pnc（包名 robot） | 规划控制核心，6 节点；Apollo 裁剪移植（lattice/QP样条/OSQP） | ~2.95万行 C++ |
| CUDA-CenterPoint | 激光 3D 检测，nuScenes 10 类，模型在 model/（仅 3 个文件为有效） | ~4350行 |
| driver | 传感器驱动：rslidar_sdk、lakibeam、ASENSING_INS_Driver、cam_geac、lidar_perception | 厂商为主 |
| canbus | CAN 收发协议（~1280行） | |
| fms_agent | MQTT↔ROS 云端网关（~1160行 Python，env/ 为运行必需 venv） | |
| auto_couple | 自动挂接：双 2D 雷达反光板配对解算挂接点（~763行，源码 GBK 编码） | |
| simview / data_logger / calib | 可视化 / 行车记录 / 标定 | |
| ivlocmsg | 定位消息定义（pnc 在用，**必须保留**） | |

### 4. 关键事实（影响后续工作判断）
- **实际运行行为是"纯路径跟踪"**：pnc 主流程（path_plan_comply.cpp:1543）lattice 规划、速度规划调用均被注释，靠逐帧矩形碰撞检测分级减速停车；自研 my_lattice_planner 已实例化但未调用
- 控制油门走简化式 `throttle = speed×18`（control_comply.cpp:561）；横向 D 挡 Stanley+纯追踪、R 挡转弯半径法
- 路径=预录制 CSV（pnc/path/），无全局路由算法；云端路由实际取 fms_agent/data/tmp_path.txt（631点）平移
- 安全兜底链完整：传感器离线检测→缓停/急停、电子围栏 fence.csv、横向偏差>5.5m 停车、RS232 急停、CAN 断流 0.5s 故障
- 车辆参数：1680kg、轴距 1.6m、max 15km/h、转向传动比 22、限角 ±22°
- CenterPoint 由 start_l4.sh 在 build/ 目录直接跑可执行（模型用相对路径 `../model/`，**不能换目录启动**）

### 5. 关键文件速查
- 规划主逻辑：`src/pnc/src/robot_path_plan/path_plan_comply.cpp`（2832行）
- 控制主逻辑：`src/pnc/src/robot_control/control_comply.cpp`
- 推理核心：`src/CUDA-CenterPoint/src/centerpoint.cpp`；ROS 节点 `test/centerpoint_ros.cc`
- 挂接算法：`src/auto_couple/src/auto_couple_core.cpp`
- CAN 协议：`src/canbus/src/canbus_comply.cpp`
- MQTT 桥：`src/fms_agent/scripts/`（broker 硬编码在 utils/MqttClient.py）
- 启动链：`start_l4.sh`（11 个终端依次拉起）+ `launch/` 各 launch

---

## 二、工作日志

### 2026-08-19 ① 全量工程分析
4 路并行深度分析（pnc / driver+canbus / 感知 / fms+辅助包），产出上述「一、工程概况」。同时发现的问题：fms_agent/data/pwd.txt 明文 GitHub PAT、MQTT 明文公网 IP+密码、cam_geac1 冗余、大量 zip 快照、logData 490MB、net_check.log 频繁掉线记录等。

### 2026-08-19 ② 工程清理（不改动任何业务逻辑代码）
1.1G → **175M**，全部为删除操作，源码/launch/config/模型源零改动。删除：13 个 zip 快照（~273MB）、logData/ 行车 CSV（490MB）+ logs/ + net_check.log×2 + 测试 bag + trtexec 日志、cam_geac1/ 旧版相机 SDK（40MB，零引用）、CenterPoint data/ 离线评测数据（38MB）+ 未加载 plan 变体（26MB）、build_back/ + 各处 .o 编译中间产物（~55MB）、transfer/ 摆渡目录、编辑器残留、pwd.txt（安全隐患）。**启动链 25 项关键文件逐一核验通过**。明细见 `CLEANUP_REPORT.md`。

### 2026-08-20 清除 conti_radar_msgs
二次 grep 验证全工程零外部引用后整包删除（32KB，ARS408 风格消息定义，雷达驱动不在本工程）。删除后复查 "conti" 残留命中均为第三方库无关单词。CLEANUP_REPORT.md 已同步更新。

### 2026-08-25 Web HMI（替代 start_l4.sh 的测试人员启动界面）
新增 `hmi/` 目录（约 2600 行，详见 `hmi/README.md`），**未改动任何现有业务代码/脚本**：
- 形态：车端 Web 页面（纯 Python stdlib + rospy 可选导入，零第三方依赖，非 catkin 包，部署=拷目录），`bash hmi/hmi.sh` 后浏览器访问 Orin:8080
- 功能：一键启动（分组并行+健康门控；roscore 改显式守护、纠正 lidar_perception 顺序）/ 全部停止（逆序+二次确认）/ 单组件启停重启 / 日志查看下载 / 车辆状态面板（车速挡位模式急停、任务、RTK、障碍物距离、横向偏差、电量挂接、传感器健康、CAN、网络）
- 健康监测：话题频率滑窗（AnyMsg，不依赖 devel 消息包）、点云间歇采样（省 CPU）、fms 节点数统计、master xmlrpc 探活（检测重启自动重连）
- 本机（x86 无 ROS）已端到端验证：T1-T11 断言全过（状态机/降级恢复/日志轮转/两段式停止/路径穿越防护/缓存头/退出不带走组件），车载配置 14 组件加载校验通过，前端 JS `node --check` 通过
- 复查轮（同日）：修复 5 处 bug——挡位元素样式被 setVal 覆盖、掉线横幅永不显示（renderOffline 未渲染 banner）、单组件停止误加确认弹窗（与设计不符）、ros_bridge init_node 半途失败后重试死循环、配置缺 health 字段 KeyError；另验证端口占用报错路径。修复后 13 项回归 + 轮转单元测试 + 退出隔离测试全过
- 深度校验轮（同日）：构建伪 ROS 环境（`hmi/tests/`：伪 rospy+伪消息包+可编程话题泵+伪 master XML-RPC），首次运行时验证 ros_bridge 全部 ROS 侧代码，**抓到 1 个车载必现的致命 bug**：`xmlrpc.client.Transport(timeout=…)` 构造参数不存在（任何 Python 版本），导致 master 探活永远失败→roscore/fms 健康永挂、master 重启重连永不触发；本机无 ROS 的测试永远发现不了。已改用 `_TimeoutTransport` 子类（make_connection 上设超时）。修复后 47 项断言全过（频率健康/降级恢复/采样探测/节点统计/master 重启与失联/车辆字段全映射/HTTP 鲁棒/并发压力/坏配置拒绝）。tests/ 仅开发机用，车载不部署
- **未做车载实测**（本机无 ROS/CAN/传感器），部署后需按 README 验证；已知车载注意项：相机 sudo 首次需手动跑一次 rb_camera.sh、勿与 start_l4.sh 同时用、HMI 重启不收养旧进程

---

## 三、当前状态（截至 2026-08-25）

- 工程体积 **~175M**，src/ 下 9 个包（见上表）；另有新增 `hmi/`（Web 人机界面，约 2600 行）
- 已清理完毕：zip 备份 0、*.log 运行日志 0、编辑器残留 0、无引用冗余副本 0
- `CLEANUP_REPORT.md`：清理明细与保留依据（含每次删除的引用验证）
- 启动链完整性：✅ 已核验（start_l4.sh 全链 25 项）；HMI 启动链与其一致（roscore 显式化 + lidar_perception 顺序修正）
- HMI：本机验证完成，**车载部署+实测待做**（拷 hmi/ 到车载 → bash hmi/hmi.sh → 浏览器验证）
- 未编译验证过：本机非车载环境，未做过 catkin build（如需验证编译，注意本机是 x86，pnc 3rd-party 有 arm/x86 两套预编译库）

---

## 四、下一步（候选工作，按优先级）

0. **HMI 车载部署实测**：拷 hmi/ 到车载 → `bash hmi/hmi.sh` → 浏览器验证一键启动全链 + 车辆状态面板数据正确性；留意相机 sudo 首次手动、simview 路径、master 重连等 README 已知项
1. **MQTT 安全整改**（需用户授权改代码/配置）：MqttClient.py 明文公网 IP + 账号密码，建议改为环境变量/配置文件 + TLS；云端侧需同步改
2. **硬编码收敛**（业务逻辑，需谨慎）：红绿灯停车坐标、auto_couple 固定托盘坐标(-21.30,-18.92)、任务 UTM 偏移（task_plan_node.cpp:59-60）、油门=speed×18
3. **确认休眠代码去留**：lattice/速度规划被注释的调用、my_lattice_planner、增益调度"模糊PID"——是否删减或恢复启用需用户决策
4. **感知转换 bug 修复**：perception_msg_convert1.cpp 把 /box 文本 marker 也当障碍物（无类型过滤）
5. **建议 git init**：当前无版本管理（仅 fms_agent 内有 .git 指向 gitee），可建仓 + .gitignore（排除 env/、build/、model 大文件视需要）
6. 可选：rs_driver 厂商文档图 7.2MB（src/driver/rslidar_sdk/src/rs_driver/doc/）
7. 可选：auto_couple 源码 GBK→UTF-8 转码（注意会改变文件内容，需用户确认）

---

## 五、保护清单（后续任何清理/修改严禁触碰）

| 项 | 原因 |
|---|---|
| `src/fms_agent/env/` | fms.sh 第2行 source 依赖 |
| `src/CUDA-CenterPoint/build/centerpoint_ros_node` 及 build/ 目录结构 | start_l4.sh 在该目录直接启动，模型相对路径依赖 |
| `src/CUDA-CenterPoint/model/` 现存 3 文件 | plan 运行时加载，onnx 为重生成源 |
| `src/fms_agent/data/tmp_path.txt` | 云端路由服务运行时读取 |
| `src/ivlocmsg/` | pnc 定位消息在用 |
| `src/fms_agent/.git` | 唯一版本历史 |
| `src/driver/cam_geac/demo/` 全部可执行 | rb_camera.sh 多功能按名调用 |
| `src/pnc/path/`、`param/`、`3rd-party/arm/{osqp,proj4,qpoases}/lib*` | 业务数据/配置/预编译库正本 |
| 根目录 `record_*.sh`、`fms.sh`、`start_l4.sh`、`launch/` | 启动与运维脚本 |

---

## 六、注意事项

- 本机路径与车载不同：脚本内绝对路径均为 `/home/nvidia/qingwei-L4-No2`，在本机改动后部署需同步
- launch/ 中 all_demo.launch、all_ipc.launch、perception.launch 引用不存在的 `/home/qwrf/mogu-master` 工作区，属残留，勿误判为本工程有效启动方式；**有效启动方式只有 start_l4.sh**
- `localization_driver.launch` 引用的 rs232/tmp 包不在本工程（在车载工作区）
- auto_couple 源码注释为 GBK 编码，直接 cat 会乱码
- 历史行车日志（logData/）已删除，如需原始数据需从车载机取
