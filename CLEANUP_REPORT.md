# 工程清理报告

日期：2026-08-19
范围：仅删除运行产物 / 日志 / 备份快照 / 无引用冗余副本。**未修改任何业务逻辑代码**（.cpp/.h/.py/.launch/.yaml/.msg/脚本 均零改动）。
结果：**1.1G → 175M（释放约 925MB）**。

## 一、删除清单

### 1. zip 快照备份（13 个，约 273MB）
| 文件 | 说明 |
|---|---|
| src/canbus_0429.zip、canbus_back_0429.zip、canbus_back_0430.zip | canbus 旧版快照 |
| src/CUDA-CenterPoint.zip、CUDA-CenterPoint0508A03.zip、CUDA-CenterPoint0708.zip | 感知包历史快照 |
| src/pnc_0429.zip、pnc_back_0429.zip、simview_back_0508.zip、data_logger.zip | 各包旧版快照 |
| src/driver/ASENSING_INS_Driver_V1.02.zip | INS 驱动快照（目录正本保留） |
| src/pnc/param_back_0508.zip、src/pnc/path.zip | 参数/路径快照（正本目录保留） |

### 2. transfer/ 摆渡目录（292KB）
canbus_0523.zip、src_0523.zip、robot_control_0709/（7 月 9 日控制模块旧版源码副本，正式版在 src/pnc/src/robot_control/）。

### 3. 运行日志与调试数据（约 490MB）
- `logData/`：130+ 份行车日志 CSV（2026-06～07，data_logger 10Hz 输出）
- `logs/`：3 份模块启动日志（2026-05-17）
- `net_check.log`、`launch/net_check.log`：网络掉线记录
- `path.csv`：INS 驱动（ASENSING_INS_node.cpp:380）以 append 模式写的轨迹转储，运行时自动重建，代码未动
- `zhg.csv`：无任何引用的临时数据
- `src/driver/cam_geac/2026-04-29-15-45-53.bag`：测试 rosbag
- `src/CUDA-CenterPoint/model/rpn_centerhead_sim.8503.log`：trtexec 引擎构建日志（2.7MB）
- `src/canbus/src/notready.mp3`：无引用遗留音频
- `src/fms_agent/data/pwd.txt`：**内含明文 GitHub PAT，属安全隐患，已删除**（全工程无引用）

### 4. 编译中间产物（约 55MB）
- `src/CUDA-CenterPoint/build_back/`：整目录（带 CATKIN_IGNORE 的备份构建树）
- `src/CUDA-CenterPoint/build/CMakeFiles/`：.o 中间产物（`centerpoint_ros_node` 可执行完整保留，start_l4.sh 启动项不受影响）
- `src/CUDA-CenterPoint/build/centerpoint`：离线测试可执行（车端启动链只用 centerpoint_ros_node）
- `src/driver/cam_geac/demo/source/*.o`：12 个链接中间产物（5.9MB，厂商预编译可执行不受影响）
- `src/pnc/3rd-party/arm/source/{osqp-0.5.0,qdldl,qpOASES}/build/`：34MB 编译产物；预编译库正本在 `src/pnc/3rd-party/arm/{osqp,proj4,qpoases}/lib*`，均保留

### 5. 无引用冗余副本（约 117MB）
- `src/driver/cam_geac1/`（40MB）：4 月旧版相机 SDK，全工程（含所有 launch/脚本/源码）**零引用**
- `src/CUDA-CenterPoint/data/`（38MB）：nuScenes 离线评测数据（pkl 29MB + 测试点云 bin 10MB），仅被 `tool/export_neck_head.py`、`tool/eval_nusc.py` 离线工具引用，车端运行时零依赖
- `src/CUDA-CenterPoint/model/rpn_centerhead_sim.plan.8503`、`.plan.8503_b1`（26MB）：代码实际只加载 `rpn_centerhead_sim.plan` + `centerpoint.scn.onnx`（见 centerpoint_ros.cc:57、centerpoint.cpp:55），变体无引用

### 6. 编辑器残留
`src/pnc/path/.~lock.4102.csv#`、`src/fms_agent/scripts/.test_mqtt_send.py.swp`、`src/driver/lidar_perception/launch/.lidar_perception.launch.swp`

### 7. 无引用消息包（2026-08-20 追加）
- `src/conti_radar_msgs/`（32KB，大陆 ARS408 风格消息定义：Object/ObjectList/Cluster/ClusterList）：全工程（package.xml/CMakeLists/源码/launch/脚本）零外部引用，整包删除。删除后复查 "conti" 残留命中均为第三方库内部单词（qpOASES 注释 Contiguous、osqp CI 脚本 continuum.io），与本包无关。

## 二、特意保留（删除前已验证）

| 项 | 保留原因 |
|---|---|
| `src/fms_agent/env/`（31MB venv） | `fms.sh` 第 2 行 `source ./src/fms_agent/env/bin/activate` 启动依赖 |
| `src/fms_agent/.git` | 唯一版本历史（gitee 远端） |
| `src/CUDA-CenterPoint/model/*.onnx` | 模型源文件，重新生成 TensorRT plan 所需 |
| `src/driver/cam_geac/demo/` 全部可执行 | `rb_camera.sh` 多功能脚本按名调用（ros1_jpg/init/rtsp 等） |
| `src/fms_agent/data/tmp_path.txt` | cloud_routing_server.py 路由服务运行时读取 |
| `src/pnc/path/`、`param/`、`launch/` | 业务数据与配置 |

## 三、遗留建议（本次未动，属代码/配置需另行决策）

1. `rslidar_sdk/src/rs_driver/doc/` 厂商文档图片 7.2MB —— 属厂商 SDK，可选清理
2. MQTT 明文公网 IP + 账号密码（`fms_agent/scripts/utils/MqttClient.py`）—— 安全隐患，需改配置
3. 多处硬编码（红绿灯停车坐标、托盘坐标、UTM 偏移、油门=speed×18）—— 属业务逻辑，未触碰

## 四、完整性核验

`start_l4.sh` 启动链 25 项关键文件逐一核验存在：lidar_perception、cam_geac（rb_camera.sh/ros1_jpg_demo/init.sh）、ins、canbus.sh/launch、rslidar_sdk、centerpoint_ros_node + 双模型文件、start_auto_couple.launch（lakibeam + auto_couple）、control.launch（pnc robot 全套 launch/config/path）、fms.sh（env + fms_agent.launch + tmp_path.txt）、data_logger、simview。全部 ✓。
