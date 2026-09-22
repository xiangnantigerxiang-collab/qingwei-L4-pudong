# gantry_detect 闸机识别

核对：2026-09-20。当前为 `src/gantry_detect/` 实体ROS1包，使用C++14/PCL。
09-17已接入HMI的3D感知卡片、monitor及planning安全输出；旧“根目录包+软链接、无消费者”描述已失效。

## 使用方法

在车端工程根、ROS1及ivlocmsg等消息依赖已生成后执行：

```bash
catkin_make --pkg gantry_detect -j4
source devel/setup.bash
roslaunch gantry_detect gantry_detect.launch
```

需要调试显示可加 `rviz:=true`。HMI的3D感知卡片已捆绑启动本节点时，不再重复运行。
参数在 `config/gantry_detect.yaml`，由launch加载；改参数后重启。
当前输入为 `/localization`、`/rslidar_points_front`、`/rslidar_points_mid`，由右路配置
（当前mid）回调驱动。用下面命令检查实际接线和状态：

```bash
rostopic hz /gantry_state
rostopic echo -n 1 /gantry_state
```

输出包含active/gantry_open。planning在 `active && !gantry_open` 基础上增加固定闸机点与
当前参考线的位置约束：闸机到最近路径点<2m，且该点沿线位于自车前方0～8m（两端不含）。
开闸/无效或位置不符只解除闸机来源，位置失效时清除关闭缓存；详见
[planning规则](../pnc/src/robot_path_plan/README.md)。调试话题含合并点云、过滤点云与markers。

## 注意事项与容易疏忽的点

- “左/右”是配置键，实际话题是front/mid，不能凭名字猜雷达安装方位。
- 09-17审查记录：定位与缓存点云老化、驱动点云断流失效、NaN预清理及降采样后最小聚类点数需复核。
  当前仍有min_cluster_pts=1000配置，不能把原始点数当降采样后点数。
- ROI、点云安装变换和定位须同坐标系；检测不到目标可能被解释为打开，需回放真实闸杆验证。
- planning在有效位置范围内保持最近闸机状态且不另加超时；离开范围或参考线失效时清除，
  重入需新消息。接入安全输出不等于检测侧故障已解决。
- HMI捆绑健康分别看/box和/gantry_state，停止要收掉两个进程；不可只停roslaunch留后台CenterPoint。
- 修改gantry_state.msg要同步planning/monitor并重编消费者。源码和参数本轮均未修改。

历史检查仅为本机定向复现，未据此认定Orin整链已验收；待办见 [workflow](../../workflow.md)。
