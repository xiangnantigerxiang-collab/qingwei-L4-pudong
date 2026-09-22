# auto_couple 自动挂接定位

核对：2026-09-19。ROS1包，用两路2D雷达等输入提供挂接目标位置；最终任务/挡位控制仍由PNC负责。
较早编译与接线记录已归入workflow历史，本轮没有改变挂接算法或外参。

## 使用方法

在车端工程根、ROS1及依赖已安装时执行：

```bash
catkin_make --pkg auto_couple -j4
source devel/setup.bash
roslaunch launch/start_auto_couple.launch
```

该工程launch同时启动两路lakibeam1和auto_couple_node，包含IP/端口、外参与target_type。
HMI“自动挂接+2D雷达”已运行时不要重复启动。检查 `/back_left_scan`、`/back_right_scan`
及 `/hook_position` 的实际类型、发布者、频率；输出类型以源码和 `rostopic info` 为准。

## 注意事项与容易疏忽的点

- 历史源码含GBK等编码，用 `rg -a` 搜索，避免被当二进制而漏掉引用；不批量转码。
- target_type和左右雷达外参决定目标语义，不能用monitor显示外参替代挂接计算外参。
- /hook_position存在跨包消息契约，旧记录中类型名不同但字段MD5相同；改字段前必须核对两侧，不能依赖巧合。
- 两路雷达的端口、传感器地址和输出名不能重复；检查真实输入后再判断算法未输出。
- auto_couple/center_position消息被lidar_perception编译依赖；改消息时同步构建下游。
- 不因D起停需求改变R挂接任务、速度或停止判据。
