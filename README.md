# 青威 L4 浦东工程

机场无人电动牵引车的软件工作空间。车端为 Jetson Orin / ROS1；本目录是车载工程的
重建副本，当前不使用 git。文档更新日期：**2026-09-22**。

编码前先读 [AGENTS.md](AGENTS.md) 和 [编码与技术风格记忆](docs/CODING_MEMORY.md)，
再读 [workflow.md](workflow.md) 的现状与待办。全部文档入口及维护规则见
[文档索引](docs/README.md)。

## 最近变化

- **09-22：control D 挡减速刹车获实车效果确认。** 用户反馈“刚刚实车测试效果很好”。
  当前版包含沿程曲率限速、电机给定平滑、正常末停和液压释放承接；R及横向算法保持。
  作为后续基线保留，参数、原有本机验证与本次反馈见[当前专项报告](src/pnc/tests/control/forward_speed_smooth_20260922.md)。
- **09-22：其他近期事项。** 完成围栏迁移冲突修复、planning转向灯复核及两份cargo轨迹限速列平滑；
  各自部署与验收状态见[workflow](workflow.md)，本次刹车反馈不替代这些功能的验收。
- **09-21～09-20：** 导航速度统一、曲率折中标定与4秒预瞄、D参考轨迹60点、HMI自定义指令独立、
  托盘反向标定。D横向速度相关权重已回滚至固定70%/30%，当前以回滚后为准。
- **日志整理：** 主日志只保留当前有效结论、近期重点及待办；较早中间过程见
  [09-22整理前归档](docs/history/2026-09-22-before-roadtest-log-cleanup/README.md)。

## 模块与使用入口

| 模块 | 用途 | 使用方法与注意事项 |
|---|---|---|
| PNC（catkin 包 `robot`） | 任务、规划、控制、导航、感知转换、控制转发 | [总说明](src/pnc/README.md)；[规划](src/pnc/src/robot_path_plan/README.md)、[安全](src/pnc/src/robot_path_plan/safety/README.md)、[控制](src/pnc/src/robot_control/README.md)、[感知转换](src/pnc/src/robot_perception_convert/README.md) |
| canbus | CAN 编解码与执行器状态机 | [README](src/canbus-vehicle-3/README.md) / [编码指南](src/canbus-vehicle-3/VIBE_CODING_GUIDE.md) |
| HDMap | 轨迹处理、车道空间查询和分类 SDK | [README](src/hdmap/README.md) |
| CUDA-CenterPoint | GPU 3D 障碍物检测 | [README](src/CUDA-CenterPoint/README.md) |
| 驱动与补盲 | 3D/2D 雷达、定位、相机、近距感知 | [驱动使用说明](src/driver/README.md) |
| gantry_detect | 闸机识别与 `/gantry_state` | [README](src/gantry_detect/README.md) |
| auto_couple / ivlocmsg | 挂接定位、定位消息定义 | [auto_couple](src/auto_couple/README.md) / [ivlocmsg](src/ivlocmsg/README.md) |
| fms_agent | 云端任务和状态桥接 | [README](src/fms_agent/README.md) |
| ultra_command | 指定任务路径的矩形区域监控 | [README](src/ultra_command/README.md) |
| health_monitor | 旁路话题健康观测 | [README](src/health_monitor/README.md) |
| HMI / monitor / log_online | 启停、可视化、事件记录 | [HMI](hmi/README.md)、[monitor](monitor/README.md)、[log_online](log_online/README.md) |
| simview | 保留的旧 RViz 工具 | [使用说明](src/simview/readme.txt) |

数据主链为：雷达 → CenterPoint `/box` → 感知转换 → `/perception/planning` →
规划 `/plan_path_msg` → 控制 `/control_msg` → can_comm → canbus。`/perception`
另供显示、历史目标和兼容逻辑；两个感知话题不能互相替代诊断。

## 使用方法

以下命令在**车端工程根目录**执行，运行前应已有与车辆匹配的 ROS1 环境、编译产物、
模型、地图和标定。开发机的 ROS 桩验证不能替代这些运行条件。

```bash
cd /home/nvidia/qingwei-L4-No2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
bash hmi/hmi.sh
```

浏览器访问车端 `8080`，由 HMI 按组件依赖启动。monitor 默认 `8081`、仪表板 `8082`；
独立事件记录器以 `bash log_online/log_online.sh` 启动，状态页为 `8083`。
单模块调试使用上表对应 README，先确认 HMI 未运行同名组件。
`start_l4.sh` 是另一条启动入口，不能与 HMI 同时启动整栈。

增量编译在已配置的 ROS1 工作空间中使用 `catkin_make --pkg <实际包名>`，并按依赖先后
生成消息/SDK；PNC 包名为 `robot`，simview 包名为 `view`。CenterPoint 独立 CMake 编译。
`--pkg` 会留下 catkin 白名单，全量构建需显式清除 `CATKIN_WHITELIST_PACKAGES`。
完整构建和模块依赖见各模块 README。

## 注意事项与容易疏忽的点

- 当前 `rebuild_all.sh` 会删除 `build/devel`，且缺少 `say/die/JOBS` 的完整定义和目录锚定。
  不能把它当作可靠的一键构建命令；本轮仅整理文档，未修改脚本。
- CenterPoint 必须在自己的 `build/` 运行，相对模型路径依赖该目录；不要清理现有模型和产物。
- `/perception` 为空不代表 `/perception/planning` 新鲜；显示障碍也不代表它通过了规划的
  车道/尺寸过滤。排查时同时看专用话题、type、时间戳和最终 safety 来源。
- 09-19 新增行驶逻辑只针对 D；R 挡后向扫描及原控制继续保留。起步观察当前源码为
  1m，历史 2m 测试差异仍待统一，本次没有改参数。
- `can_msg/can_comm_msg` 有三包副本，消息改动要同步消费者与重编；旧 `.so` 也要区分
  x86_64 与 Orin aarch64。修改 CSV、YAML 后是否需重启，以模块实际加载方式为准。
- HMI 的“全部停止”不等于硬件急停；页面服务仅用于内网。普通清理不得删除运行数据、
  标定、模型、虚拟环境或工程外回滚快照。

## 验证与历史

PNC 的 [规划测试](src/pnc/tests/planning/README.md) 和各模块使用章节列出可复现入口。
记录需区分静态检查、本机桩测试、真实 ROS1 编译、已部署和实车确认。
最新工作进展只更新 [workflow.md](workflow.md)，不要在多个 README 复制整段日志。
