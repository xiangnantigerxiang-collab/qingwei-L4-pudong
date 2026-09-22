# 文档索引与维护规则

更新：2026-09-22。工程总入口为 [根 README](../README.md)，编码约束见
[AGENTS.md](../AGENTS.md) 与 [编码与技术风格记忆](CODING_MEMORY.md)。

## 使用方法

开始任务：风格记忆 → workflow 当前状态/待办 → 模块 README → 相关完整函数或协议。
使用模块：根 README 模块表 → 模块“使用方法” → “注意事项与容易疏忽的点”。
查旧行为：先确认现行说明，再按日期查历史归档，不用旧行号直接定位当前代码。

## 按时间维护

| 时间距离 | 主文档保留内容 |
|---|---|
| 最近 7 天 | 需求、当前最终行为、关键条件/参数、源码入口、验证及未完成项；最近一天最详细 |
| 8～30 天 | 每事项 1～3 句：结论、是否被替代、证据链接；去掉中间尝试和重复测试数字 |
| 超过 30 天 | 按阶段归纳一行或一小段，完整过程留归档 |
| 仍有效的接口/约束/未解决问题 | 不按时间降级，保留在当前用法、注意事项或待办中 |

“最近”以本次维护日期计算，不硬编码为永久的重要性。README 说明当前怎么用；workflow
记录最近做了什么、现在是什么状态、还缺什么。相同事项只保留一份现行详细说明并相互链接。
回滚或被替代方案必须明确标为历史，不能与新值并列成多份“最新”。

2026-09-22按用户要求进一步合并近期同一事项的中间尝试：保留最终行为、证据和未解决问题，
不按轮次重复堆叠参数与通过计数。最新control D挡减速刹车已获用户实车“效果很好”反馈，
现行状态集中在[control专项报告](../src/pnc/tests/control/forward_speed_smooth_20260922.md)。
原始bag、历史测试日志和冻结交付包保持原样，补录反馈不把旧验证改写为新结果。

## 模块 README 的固定内容

每份工程维护的模块 README 应包含：职责/输入输出、最近有效变化、使用方法
（运行目录、依赖、构建、启动或 API、参数和观察结果）、验证入口、注意事项与易疏忽点。
库、测试目录、纯消息包写调用/运行/生成方法，不虚构不存在的节点启动命令。
驱动、算法上游文档可保留正文，在工程接入入口补本项目用法与限制。

注意事项必须来自已确认的需求、源码或历史问题，写清现状与证据；不要把旧问题默认写成
已修，也不要把历史基准测试数写成本次重跑结果。命令给出正确包名和工作目录。

## 文档分工与入口

| 文档 | 职责 |
|---|---|
| [根 README](../README.md) | 工程模块地图、使用入口和共性注意事项 |
| [workflow](../workflow.md) | 现状、按日期递减的记录、合并后的待办和保护清单 |
| [风格记忆](CODING_MEMORY.md) | 用户长期偏好、依据、技术约束及交付习惯 |
| [PNC](../src/pnc/README.md) / [canbus 指南](../src/canbus-vehicle-3/VIBE_CODING_GUIDE.md) | 模块编码细则，不恢复历史架构 |
| [control当前报告](../src/pnc/tests/control/forward_speed_smooth_20260922.md) | 当前D挡参数、本机验证和09-22用户实车反馈 |
| [规划测试](../src/pnc/tests/planning/README.md) | 测试入口、当前结果、历史夹具差异 |
| [驱动入口](../src/driver/README.md) | 上游雷达/相机驱动的本工程接线与用法 |
| [09-22整理前原文](history/2026-09-22-before-roadtest-log-cleanup/README.md) | 本次压缩前的1807行workflow及SHA-256，含近期各阶段完整经过 |
| [09-19整理前原文](history/2026-09-19-before-docs/README.md) | 更早完整README/workflow和SHA-256清单 |

其他模块按 [根 README 模块表](../README.md#模块与使用入口) 进入。
`PNC_ANALYSIS.md`、`CLEANUP_REPORT.md`、已有 design/plan/verification 文件是各自日期的历史
分析或专题证据，不是默认现行说明。旧报告原有实验数据不重写；被替代时补充历史标识与现行链接。

## 当前工程 README 清单

以下为工程维护入口；本次同步control相关现行说明和总索引，其他模块保留各自的更新日期。
上游驱动内核、依赖、安装副本及归档的排除范围见下一节。

| 文件 | 说明 |
|---|---|
| [README.md](../README.md) | 青威 L4 浦东工程 |
| [docs/README.md](README.md) | 文档索引与维护规则 |
| [hmi/README.md](../hmi/README.md) | qingwei L4 Web HMI（8080） |
| [log_online/README.md](../log_online/README.md) | log_online — 机场无人牵引车事件数据记录节点（README） |
| [monitor/README.md](../monitor/README.md) | qingwei L4 monitor（8081 + dashboard 8082）— Web 3D 可视化 |
| [src/CUDA-CenterPoint/README.md](../src/CUDA-CenterPoint/README.md) | CUDA-CenterPoint |
| [src/CUDA-CenterPoint/qat/README.md](../src/CUDA-CenterPoint/qat/README.md) | **Quantization for CenterPoint SparseConvolution** |
| [src/auto_couple/README.md](../src/auto_couple/README.md) | auto_couple 自动挂接定位 |
| [src/canbus-vehicle-3/README.md](../src/canbus-vehicle-3/README.md) | canbus：使用方法与现行协议契约 |
| [src/driver/ASENSING_INS_Driver_V1.02/README.md](../src/driver/ASENSING_INS_Driver_V1.02/README.md) | ASENSING 定位驱动（工程接入） |
| [src/driver/README.md](../src/driver/README.md) | 驱动与近距感知使用说明 |
| [src/driver/cam_geac/README.md](../src/driver/cam_geac/README.md) | GEAC相机：本工程使用入口 |
| [src/driver/cam_geac/demo/README.md](../src/driver/cam_geac/demo/README.md) | 一，适用版本 |
| [src/driver/cam_geac/tools/readme.txt](../src/driver/cam_geac/tools/readme.txt) | GEAC tools 工程使用说明（核对：2026-09-19） |
| [src/driver/lakibeam/readme.md](../src/driver/lakibeam/readme.md) | LakiBeam 2D雷达 |
| [src/driver/lidar_perception/readme.md](../src/driver/lidar_perception/readme.md) | lidar_perception 近距补盲感知 |
| [src/driver/rslidar_sdk/README.md](../src/driver/rslidar_sdk/README.md) | 1 **rslidar_sdk** |
| [src/driver/rslidar_sdk/README_CN.md](../src/driver/rslidar_sdk/README_CN.md) | 1 **rslidar_sdk** |
| [src/fms_agent/README.md](../src/fms_agent/README.md) | fms_agent 云端任务与状态桥接 |
| [src/gantry_detect/README.md](../src/gantry_detect/README.md) | gantry_detect 闸机识别 |
| [src/hdmap/README.md](../src/hdmap/README.md) | HDMap Server SDK |
| [src/health_monitor/README.md](../src/health_monitor/README.md) | health_monitor —— 话题健康监测节点 |
| [src/ivlocmsg/README.md](../src/ivlocmsg/README.md) | ivlocmsg 定位消息包 |
| [src/pnc/README.md](../src/pnc/README.md) | PNC：使用方法与编码规范 |
| [src/pnc/src/robot_control/README.md](../src/pnc/src/robot_control/README.md) | robot_control 模块说明 |
| [src/pnc/src/robot_task_plan/README.md](../src/pnc/src/robot_task_plan/README.md) | 任务编排与当前围栏契约 |
| [src/pnc/src/robot_path_plan/README.md](../src/pnc/src/robot_path_plan/README.md) | robot_path_plan 模块说明 |
| [src/pnc/src/robot_path_plan/safety/README.md](../src/pnc/src/robot_path_plan/safety/README.md) | 真实观测驱动的前向感知避障 |
| [src/pnc/src/robot_perception_convert/README.md](../src/pnc/src/robot_perception_convert/README.md) | 感知转换、目标跟踪与时序滤波 |
| [src/pnc/tests/perception/README.md](../src/pnc/tests/perception/README.md) | 感知测试入口与历史HDMap接入夹具 |
| [src/pnc/tests/planning/README.md](../src/pnc/tests/planning/README.md) | Planning验证：用法、现行结果与历史边界 |
| [src/pnc/tests/control/README.md](../src/pnc/tests/control/README.md) | D挡平顺控制、制动状态与非D差分验证 |
| [src/pnc/tests/navigation_speed/README.md](../src/pnc/tests/navigation_speed/README.md) | 导航实际速度来源、时效与异常检查 |
| [src/pnc/tests/task_fence/README.md](../src/pnc/tests/task_fence/README.md) | 围栏迁移与任务/规划联调验证 |
| [src/simview/readme.txt](../src/simview/readme.txt) | simview：保留的RViz工具（核对：2026-09-19） |
| [src/ultra_command/README.md](../src/ultra_command/README.md) | ultra_command — 312 任务族监控矩形障碍物检测 |

## 注意事项与容易疏忽的点

- 归档保存原文，包括已过时描述；只从归档恢复被需要的证据，不整体覆盖当前规则。
- `/tmp` 路径仅为当时验证产物，可能随清理消失；仓内脚本、报告和持久文档才是长期入口。
- `src/hdmap/sdk/share/hdmap/README.md` 是安装副本，源文件为 `src/hdmap/README.md`；
  重新安装 SDK 时同步，不手改安装产物。
- `3rd-party`、`src/driver/rslidar_sdk/src/rs_driver`、虚拟环境、缓存中的 README 是依赖
  原文，不作为本工程维护文档批量重写。它们的接入方法放在所属模块 README。
- 纯文档整理只查文档链接、源码事实和文件范围，不启动节点，也不改变业务/配置。
