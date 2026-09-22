simview：保留的RViz工具（核对：2026-09-19）

使用方法
当前ROS包名是view，不是旧说明中的robot_simview。
在车端工程根、已构建工作空间中执行：
  source devel/setup.bash
  roslaunch view simview.launch
launch默认加载本包launch/view.csv，并设置/robot/simviewer/mapswitch为1。
地图路径由/robot/mapfile控制，RViz配置在launch/robot.rviz。

注意事项与容易疏忽的点
- HMI日常使用monitor替代simview；保留工具不代表需要恢复一键启动卡片。
- 根start_l4.sh仍含指向工程外/home/nvidia/zyd/0522的旧simview入口，不要误认为它运行本树。
- can_msg/can_comm_msg在本包有副本，改跨包字段必须同步，不能因UI已替代而删消息包。
- /planning/obstacles旧显示项对应已删除调试发布接口，不能据无数据判断现行规划故障。
- 不与另一整栈同时启动；本轮只纠正文档包名和用法，没有改launch或地图。

历史摘要
09-01起HMI改用Web monitor；更早手动运行过程已归档。
