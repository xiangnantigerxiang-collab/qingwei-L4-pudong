# LakiBeam 2D雷达

核对：2026-09-19。ROS包名为 `lakibeam1`。工程由自动挂接launch配置两路2D雷达，
原先通用安装说明已收敛为以下车端用法，历史原文保留在文档归档。

## 使用方法

在ROS1工程根执行：

```bash
catkin_make --pkg lakibeam1 -j4
source devel/setup.bash
roslaunch launch/start_auto_couple.launch
```

以上同时启动两路雷达与auto_couple。单设备独立调试可选包内：
`roslaunch lakibeam1 lakibeam1_scan.launch`（LaserScan），或
`roslaunch lakibeam1 lakibeam1_pcd.launch`（PointCloud2）；对应 `_view.launch` 含RViz。
不要与HMI或整车两路launch重复启动。

## 注意事项与容易疏忽的点

- 工程传感器IP/端口、output_topic、inverted和外参以根start_auto_couple.launch为准，
  包内示例不等于当前车辆配置。
- 两路分别检查 `/back_left_scan` 和 `/back_right_scan`，相同frame_id不代表安装位置相同。
- monitor另有显示外参，不能把网页显示对齐当挂接算法外参已校准。
- 改为PointCloud2模式后要同步消费者类型，不能只改launch可执行名。
