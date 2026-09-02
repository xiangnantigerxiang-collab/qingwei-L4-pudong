# ROS topic 录制开关

本文件分别控制 HMI“感知数据录制”和“规控数据录制”每次启动时交给
`rosbag record` 的 topic。

- 每个录制分组都列出完整 topic，可分别设置，互不影响
- 值为 `0`：该分组本次不录制此 topic
- 值为 `1`：该分组本次录制此 topic
- 每次手动启动录制前都会重新读取本文件，无需重启 HMI
- topic 必须使用绝对名称（以 `/` 开头）
- 同一 topic 在同一个录制分组中只能出现一次；允许在两个分组中分别设置

## 感知数据录制

### ROS 系统

- /clock: 0
- /rosout: 0
- /rosout_agg: 0
- /tf: 0
- /tf_static: 0

### 激光雷达

- /rslidar_packets_left: 0
- /rslidar_packets_right: 0
- /rslidar_packets_mid: 0
- /rslidar_packets_front: 0
- /rslidar_packets_back: 0
- /rslidar_imu_data_left: 0
- /rslidar_imu_data_right: 0
- /rslidar_imu_data_mid: 0
- /rslidar_imu_data_front: 0
- /rslidar_imu_data_back: 0
- /rslidar_points_left: 0
- /rslidar_points_right: 0
- /rslidar_points_mid: 0
- /rslidar_points_front: 0
- /rslidar_points_back: 0
- /rslidar_points_rear: 0

### RTK/惯导

- /ASENSING_INS: 0
- /ivsensorgps: 0
- /ivsensorimu: 0
- /ivsensorodom: 0
- /localization: 0

### 2D 雷达与自动挂接

- /richbeam_lidar/scan: 0
- /richbeam_lidar/scan0: 0
- /richbeam_lidar/scan1: 0
- /richbeam_lidar/scan2: 0
- /scan: 0
- /scan0: 0
- /scan1: 0
- /scan2: 0
- /front_scan: 0
- /back_left_scan: 0
- /back_right_scan: 0
- /richbeam_lidar/pcd: 0
- /richbeam_lidar/pcd0: 0
- /richbeam_lidar/pcd1: 0
- /pcd: 0
- /pcd0: 0
- /pcd1: 0
- /bounding_box: 0
- /hook_position: 1
- /hook_position1: 0
- /back_pointcloud: 0
- /perception_front_bbox: 1
- /perception_back_bbox: 1
- /segment_front_input_pc: 0
- /segment_back_input_pc: 0

### 3D 感知与兼容链路

- /box: 0
- /perception: 1
- /lidar_pointcloud_rear: 0
- /lidar_pointcloud_rear_left: 0
- /lidar_pointcloud_rear_right: 0
- /lidar_pointcloud_front_left: 0
- /lidar_pointcloud_front_right: 0
- /scan_bbox_result: 0
- /scan_perception_result: 0
- /debug: 0

### 相机

- /cam0/compressed: 0
- /cam1/compressed: 0
- /cam2/compressed: 0
- /cam3/compressed: 0
- /cam4/compressed: 0
- /cam5/compressed: 0
- /cam6/compressed: 0
- /cam7/compressed: 0
- /cam0/status: 0
- /cam1/status: 0
- /cam2/status: 0
- /cam3/status: 0
- /cam4/status: 0
- /cam5/status: 0
- /cam6/status: 0
- /cam7/status: 0
- /camera/image/compressed: 0
- /camera/tl_status: 0

### CAN、规划与控制

- /can_recv: 0
- /can_send: 0
- /can_msg: 0
- /can_comm_msg: 0
- /navigation_msg: 0
- /task_plan_msg: 0
- /refer_path_msg: 0
- /plan_path_msg: 0
- /path_plan_status: 0
- /control_msg: 0
- /acc: 0
- /sound_light_msg: 0
- /palletpos: 0
- /robot/serial/rs232/: 0
- /test_trajs: 0
- /planning/obstacles: 0
- /sampled_path: 0
- /optimal_path: 0

### 云端网关

- /cloud/task/task_info: 0
- /cloud/task/remote_signal: 0
- /cloud/task/task_status: 0
- /cloud/msg/running_msg: 0
- /cloud/msg/command_msg: 0
- /cloud/msg/warning_msg: 0
- /cloud/msg/notify_msg: 0
- /cloud/msg/airport_msg: 0
- /v2nHeartBeat: 0
- /v2nCommandFeedback: 0
- /v2nRunningFeedback: 0

### 可视化

- /robot/simviewer/vehicle: 0
- /robot/simviewer/lidar: 0
- /robot/simviewer/map: 0
- /robot/simviewer/routing: 0
- /robot/simviewer/planning: 0
- /robot/simviewer/loadpos: 0
- /robot/simviewer/stoppose: 0
- /robot/simviewer/globalpath: 0

## 规控数据录制

### ROS 系统

- /clock: 0
- /rosout: 0
- /rosout_agg: 0
- /tf: 0
- /tf_static: 0

### 激光雷达

- /rslidar_packets_left: 0
- /rslidar_packets_right: 0
- /rslidar_packets_mid: 0
- /rslidar_packets_front: 0
- /rslidar_packets_back: 0
- /rslidar_imu_data_left: 0
- /rslidar_imu_data_right: 0
- /rslidar_imu_data_mid: 0
- /rslidar_imu_data_front: 0
- /rslidar_imu_data_back: 0
- /rslidar_points_left: 0
- /rslidar_points_right: 0
- /rslidar_points_mid: 0
- /rslidar_points_front: 0
- /rslidar_points_back: 0
- /rslidar_points_rear: 0

### RTK/惯导

- /ASENSING_INS: 0
- /ivsensorgps: 0
- /ivsensorimu: 0
- /ivsensorodom: 0
- /localization: 0

### 2D 雷达与自动挂接

- /richbeam_lidar/scan: 0
- /richbeam_lidar/scan0: 0
- /richbeam_lidar/scan1: 0
- /richbeam_lidar/scan2: 0
- /scan: 0
- /scan0: 0
- /scan1: 0
- /scan2: 0
- /front_scan: 0
- /back_left_scan: 0
- /back_right_scan: 0
- /richbeam_lidar/pcd: 0
- /richbeam_lidar/pcd0: 0
- /richbeam_lidar/pcd1: 0
- /pcd: 0
- /pcd0: 0
- /pcd1: 0
- /bounding_box: 0
- /hook_position: 0
- /hook_position1: 0
- /back_pointcloud: 0
- /perception_front_bbox: 0
- /perception_back_bbox: 0
- /segment_front_input_pc: 0
- /segment_back_input_pc: 0

### 3D 感知与兼容链路

- /box: 0
- /perception: 0
- /lidar_pointcloud_rear: 0
- /lidar_pointcloud_rear_left: 0
- /lidar_pointcloud_rear_right: 0
- /lidar_pointcloud_front_left: 0
- /lidar_pointcloud_front_right: 0
- /scan_bbox_result: 0
- /scan_perception_result: 0
- /debug: 0

### 相机

- /cam0/compressed: 0
- /cam1/compressed: 0
- /cam2/compressed: 0
- /cam3/compressed: 0
- /cam4/compressed: 0
- /cam5/compressed: 0
- /cam6/compressed: 0
- /cam7/compressed: 0
- /cam0/status: 0
- /cam1/status: 0
- /cam2/status: 0
- /cam3/status: 0
- /cam4/status: 0
- /cam5/status: 0
- /cam6/status: 0
- /cam7/status: 0
- /camera/image/compressed: 0
- /camera/tl_status: 0

### CAN、规划与控制

- /can_recv: 0
- /can_send: 0
- /can_msg: 1
- /can_comm_msg: 1
- /navigation_msg: 1
- /task_plan_msg: 1
- /refer_path_msg: 0
- /plan_path_msg: 1
- /path_plan_status: 1
- /control_msg: 1
- /acc: 1
- /sound_light_msg: 0
- /palletpos: 1
- /robot/serial/rs232/: 0
- /test_trajs: 0
- /planning/obstacles: 1
- /sampled_path: 0
- /optimal_path: 0

### 云端网关

- /cloud/task/task_info: 1
- /cloud/task/remote_signal: 1
- /cloud/task/task_status: 1
- /cloud/msg/running_msg: 1
- /cloud/msg/command_msg: 1
- /cloud/msg/warning_msg: 1
- /cloud/msg/notify_msg: 1
- /cloud/msg/airport_msg: 1
- /v2nHeartBeat: 1
- /v2nCommandFeedback: 1
- /v2nRunningFeedback: 1

### 可视化

- /robot/simviewer/vehicle: 0
- /robot/simviewer/lidar: 0
- /robot/simviewer/map: 0
- /robot/simviewer/routing: 0
- /robot/simviewer/planning: 0
- /robot/simviewer/loadpos: 0
- /robot/simviewer/stoppose: 0
- /robot/simviewer/globalpath: 0
