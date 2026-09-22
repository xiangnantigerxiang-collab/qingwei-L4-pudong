# 驱动与近距感知使用说明

核对：2026-09-19。各驱动保留原有实现和上游说明，本页记录本工程接线与使用入口。
最近的整栈调整是09-17 HMI将3D感知与闸机捆绑启动；驱动本身的外参/网络/模型不因文档整理改变。

## 使用方法

以下从车端工程根开始，先 `source devel/setup.bash`，再按需选一个入口，不能与HMI重复启动：

| 模块 | 实际包名/启动 | 配置与说明 |
|---|---|---|
| RoboSense 3D | `roslaunch launch/start.launch`（整车接线） | [上游SDK与本工程说明](rslidar_sdk/README_CN.md)，编译需保留ENABLE_TRANSFORM=ON |
| LakiBeam 2D | `roslaunch launch/start_auto_couple.launch`（含两路2D与挂接） | [单设备用法](lakibeam/readme.md)，传感器IP/端口/输出名由launch给定 |
| ASENSING定位 | `roslaunch ins demo.launch` | [使用说明](ASENSING_INS_Driver_V1.02/README.md)，设备/dev/serial_gps |
| GEAC相机 | 进入 `src/driver/cam_geac` 执行 `bash rb_camera.sh ros1_jpg` | [使用说明](cam_geac/README.md)，依赖车端库、配置及工作目录 |
| 补盲感知 | `roslaunch lidar_perception lidar_perception.launch` | [README](lidar_perception/readme.md)，输入前扫描/后点云与挂接位置 |

构建按各包CMake依赖进行；RoboSense使用 `catkin_make --pkg rslidar_sdk -DENABLE_TRANSFORM=ON`。
补盲依赖auto_couple生成的消息，不能只拷旧头文件代替正常构建。使用 `rostopic info/hz <话题>`
核对实际发布者、类型和频率，再对照下游所订话题。

## 注意事项与容易疏忽的点

- 同一UDP端口/串口只能由正确实例消费；HMI与手动launch双启可表现为随机断流。
- 驱动frame_id、安装外参和下游地图坐标是不同层次；显示不对齐先核对变换链。
- LakiBeam两路即使frame_id相同，monitor仍可能使用自己的SCAN_EXTRINSICS，不要假定有tf。
- RoboSense的坐标变换选项属于当前部署契约，不能直接照上游默认构建替代。
- GEAC启动脚本会复制库到系统目录并初始化设备；源码目录中的demo可执行是运行依赖，不是可清理垃圾。
- 雷达/相机实际数据频率不能用规划/显示刷新频率代替；需检查原话题和消费端年龄。
- 下层rs_driver、第三方库、安装产物的README为原版参考，不批量改其协议或安装内容。
