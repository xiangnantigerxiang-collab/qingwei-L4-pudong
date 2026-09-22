# simview 路径显示

catkin 包名 `view`。车端依赖已构建并 source 工作空间后，可使用
`roslaunch view simview.launch` 启动；已有整栈/RViz会话时不要重复启动。
地图来自 `/robot/mapfile`，本包 launch 默认 `launch/view.csv`；根目录整栈 launch
可将其配置为 PNC `path/` 下的文件。先核对实际参数及文件是否存在。

2026-09-20 `draw.cpp::LoadMap()` 改为按行消费，兼容 3～5 列 CSV：
前三列仍为 x/y/heading，第四列地图限速、第五列旧标志只读取校验，不参与显示。
保留原 heading 转换和 0.1 m 点间距过滤。空行、BOM及整行注释可读取；
坏行跳过并统计，文件不存在或读取失败返回空地图并输出日志，不发生错列死循环。

同步源码后重编 `view` 并重启 simviewer。PNC 数据格式见
[路径说明](../pnc/path/README.md)。本机无 ROS/RViz，已将实际 `LoadMap()` 原文以 C++11
严格编译，并核对全部26个PNC文件迁移前后显示几何完全一致；未验证完整RViz节点或实车显示。
