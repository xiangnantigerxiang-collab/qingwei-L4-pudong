# ASENSING 定位驱动（工程接入）

核对：2026-09-19。目录名与ROS包名不同，实际包/节点为 `ins`；运行依赖车端串口设备。
本轮仅补文档，较早编译接入记录见工程workflow归档。

## 使用方法

在车端工程根执行 `source devel/setup.bash` 后启动 `roslaunch ins demo.launch`。
当前launch配置设备 `/dev/serial_gps`，启动前核对设备映射和权限；HMI已启动定位时勿重复开串口。
构建使用 `catkin_make --pkg ins -j4`，消息依赖须先生成。
使用 `rostopic info /localization`、`rostopic hz /localization` 检查实际接线和连续输出。

## 注意事项与容易疏忽的点

- 目录名字不能直接当catkin包名；设备名不同也不能改成任意ttyUSB来碰运气。
- 确认定位与地图使用同一坐标原点及heading约定，再判断规划横向偏差或障碍物错位。
- 旧导航链可能重发缓存定位，不能只看下游有频率就认定原始定位新鲜。
- 修改定位消息须同步PNC等消费者；本机桩没有验证串口、设备时间或实车定位质量。
