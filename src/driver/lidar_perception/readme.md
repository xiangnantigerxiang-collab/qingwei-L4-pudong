# lidar_perception 近距补盲感知

核对：2026-09-19。该模块处理近距扫描/点云，独立于CenterPoint的/box检测链。
较早编译修复明确依赖auto_couple生成消息，本轮不改算法或ROI。

## 使用方法

在ROS1工程根、auto_couple等依赖已生成后执行：

```bash
catkin_make --pkg lidar_perception -j4
source devel/setup.bash
roslaunch lidar_perception lidar_perception.launch
```

当前launch配置前输入 `/front_scan`、后输入 `/back_pointcloud`、挂接输入 `/hook_position1`，
输出 `/perception_front_bbox` 与 `/perception_back_bbox`；现场要用rostopic info/hz核对
上游实际话题是否存在。直接 `rosrun lidar_perception lidar_perception_scan_node` 不会自动加载该launch参数。

## 注意事项与容易疏忽的点

- 包含参数不代表上游已启动；没有输出先查输入名、消息类型与frame，不直接改停车距离。
- 不与HMI/start_l4中的同一节点双启。补盲输出与/perception/planning不是同一个来源。
- auto_couple/center_position.h由消息生成，干净构建必须保留正确依赖顺序。
- 前后ROI和安装变换有方向含义，不直接互换x正负或左右参数。
- catkin --pkg残留白名单会影响后续全量构建；完整构建需显式清空。
