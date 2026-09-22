# 2026-09-19 手动切自动起步周边观察

需求：手动转自动后检查车身周围 2 m；有障碍时 safety=1，不允许起步；无障碍时撤销本项
停车；起步后退出，继续常规避障。用户补充：连续两帧实际速度 >0.2 m/s 判为成功起步。

## 实现

- 新 `safety/startup_observation.h/.inc` 独立保存模式、真实物理框、原始地图定位和 CAN 状态。
  每次手动切自动（含节点首次看到自动）重新等待切换后的真实观测；起步后再次停车及任务
  切换不重新开启。倒车速度也按绝对值判定，阈值严格大于 0.2 m/s。
- `PlanningPerceptionCallBack()` 在 ACC 门控前把完整 `/perception/planning` 送入起步观察，
  不套用常规车道、方向、细长框、置信度及多帧确认策略。上游排除区仍保留。
- 车身使用已加载的空车/载荷外廓；障碍物使用真实角点。先检查矩形相交，再检查最近欧氏
  距离 ≤2 m，包含角部圆弧范围。现有默认保护外廓宽 3.4 m、长 3.6 m、前伸 2.3 m。
- `PublishFinalPlanPath()` 将本项 `safety=1`、零速叠加到最终发布，覆盖短路径提前返回；
  发布后恢复原内部值，清除本项不改变闸机、参考路径急停等安全来源。
- 感知/CAN/定位无效或超时（现有 input_timeout，默认 0.6 s）时保持起步停车；有效空帧
  立即允许起步。切换前/重复/乱序帧不刷新时效，坏几何不能部分提交后放行。
- 不新增线程/话题/参数/消息字段。观察期间 O(N)，预留 512 个矩形后复用缓存；退出后输入
  和判断均为 O(1)。不修改常规避障、控制标定、用户 CSV/YAML 或启动脚本。

## 验证结果

| 检查 | 结果 |
|---|---:|
| 独立核心 C++11，Wall/Wextra/Werror | 2,508 项通过 |
| 同套 ASan/UBSan | 2,508 项通过 |
| 新起步观察真实入口/发布/常规交接 | 144 项通过 |
| 单点限速 | 284 项通过 |
| 闸机安全叠加 | 1,806 项通过 |
| 常规感知集成 | 362 项通过 |
| 起步后不重入、旧等待删除与轨迹复用 | 756 项通过 |
| 终点规划/末端控制 | 2,031 / 110 项通过 |
| 规划/控制/公共算法编译 | 45 个翻译单元通过 |
| 已完成起步的旧业务完整输出差分 | 387 组逐字节一致 |
| 新核心 cppcheck warning/performance/portability | 无诊断 |

2,000 组随机几何用独立的角点绕数、线段相交及点到线段距离作为期望值，未用生产函数自证。
模式/时效检查包括 0.2 等号、速度脉冲、重复发布、同一时刻 CAN、断续帧、回切手动、重新
切自动、节点初次自动、无定位、输入过期、空帧、坏框、重复/乱序/未来时间戳等。
发布检查逐字段验证只改变 safety/速度，其他消息字段、内部状态、声光、发布次数及顺序保持。

开工备份：`/tmp/planning_startup_before_20260919_150907/before.tar.gz`。
核心与集成日志：`/tmp/planning_startup_final_20260919/result.json` 及 `integration/`。
差分记录：`/tmp/planning_startup_before_20260919_150907/ordinary_regression.json`；
轨迹 SHA-256：`b3c0afd5ef9b978540d57b9d262546c8223d71f75c1b28a6a20a6a75003c3e6d`。

## 其他业务影响复核

再次对照开工快照核对 5 个原生产文件的完整差分：仅增加独立观察对象/入口、在 CAN 和
原始导航回调中转交输入、在感知回调的 ACC 门控前转交真实框，以及最终发布的 safety/速度
叠加。新类不写原任务、路径、常规感知状态，不修改参数服务器；退出后的检查直接返回。
最终消息仍只发布一次，发布后恢复原 safety/速度。所有既有 include 的相对顺序保持。

PNC 中规划实现/测试目录之外的 **1,279 个既有文件哈希全部不变**，包括控制、任务规划、
感知转换、消息、用户路径/参数与构建配置。规划的任务推进、路径生成、常规新旧避障、
前后扫描、区域限速和停车速度实现也保持原字节；其原有调用顺序没有改变。

387 组差分验证的是**已完成起步观察后的常规行驶**，并非声称切入自动的输出仍和旧版相同。
有意新增的行为包括：首次收到自动状态时按新会话观察；等待切换后的真实帧；观察期间
全部方向/车道目标参与；缺失/过期/无效输入阻止起步。这些限制在起步完成后退出。
此处记录首轮交付时仍存在的 `AccSwitch` 输入门控与断流停车矛盾；同日后续根据行驶中
漏停反馈，已取消专用 `/perception/planning` 回调的 ACC 门控，见
[D 挡起停与漏停排查记录](d_comfort_verification_20260919.md)。

## 完整改动文件清单

生产代码（`src/pnc/src/robot_path_plan/`，7 个）：

- `path_plan_comply.cpp`
- `path_plan_comply.h`
- `path_plan_node.cpp`
- `path_plan_output.inc`
- `path_plan_perception.inc`
- `safety/startup_observation.h`（新增）
- `safety/startup_observation.inc`（新增）

测试代码（`src/pnc/tests/planning/`，7 个）：

- `gantry_safety.cpp`
- `regression.cpp`
- `startup_removal.cpp`
- `verify_point_speed_limit.py`
- `startup_observation.cpp`（新增）
- `startup_observation_integration.cpp`（新增）
- `verify_startup_observation.py`（新增）

说明与记录（4 个）：

- `src/pnc/src/robot_path_plan/README.md`
- `src/pnc/tests/planning/README.md`
- `src/pnc/tests/planning/startup_observation_verification_20260919.md`（本文件，新增）
- `workflow.md`

复现：

```bash
python3 -B src/pnc/tests/planning/verify_startup_observation.py --output /tmp/startup-observation-check
python3 -B src/pnc/tests/planning/verify.py --baseline /path/to/before/src/pnc --output /tmp/startup-driving-diff
```

## 部署边界

本机使用实际业务源码和从真实 `.msg` 生成的 ROS 桩；不代表 ROS1/Orin 运行结果。
需要同步 7 个规划生产文件（含新增头和 `.inc`）、重编 robot 并重启规划节点。
车端验收手动切自动、四周 2 m 障碍/清除、D/R 低速两帧退出、再停车不重入和重新切自动。
实际车身及挂托盘尺寸需与既有外廓配置一致；上游感知排除区内不发布的目标无法在此检测。

## 同日实车不放行反馈：诊断补充（用户已确认解决，未提供根因）

用户最初反馈切入自动后始终未起步，周边无障碍但显示 safety=1，随后明确表示问题已解决。
未提供具体处理方式和实车消息，不能把本机复现的输入失效情况认定为现场根因。

本机确认：有效的新空帧会立即撤销起步停车；缺失 `/perception/planning`、消息超时或
任何框无法解析时，原实现均返回停车。监控使用 `/perception`，两路数据不等价。
新增 100 周期真实发布检查：车速始终为 0，前三帧周边有目标，其后有效空帧持续放行。
这证明放行不需要先达到 0.2 m/s；该速度仅用于确认起步完成并退出观察。

本轮生产修改仅 `startup_observation.h/.inc`、`path_plan_comply.h`、`path_plan_output.inc`：
为原判定提供状态原因、目标 ID 和消息年龄，并在最终发布处打印原 safety、闸机、急停及
起步来源。未更改 2 m 几何、0.2 m/s 退出、超时/坏帧停车规则或其他来源的 safety。
常规输出和输入缺失仍按此前判据处理，没有用盲目清零绕过停车。

- C++11 核心及同套 ASan/UBSan：各 2,537 项通过。
- 起步发布集成：444 项通过；加其余专项总计 5,793 项，45 单元编译通过。
- 新核心 cppcheck 无诊断。结果：`/tmp/planning_startup_diagnostics_final_20260919/result.json`。
- 排查前快照：`/tmp/planning_startup_diagnosis_20260919_154115/before.tar.gz`。

`plan safety:` 日志中的 `startup` 区分 `clear`、`inactive`、`nearby_obstacle`、
`waiting_perception_planning`、`invalid_perception_box`、`invalid_perception_time`、
`invalid_or_missing_pose`、`invalid_can_speed`、`clock_mismatch`、
`perception_timeout`、`pose_timeout`、`can_timeout`。持续停车每秒打印一次，原因变化立即打印。
还输出目标 ID、缓存框数、感知/定位/CAN 年龄（-1 为无时间记录）以及独立 safety 来源。

现场首先需要这两条只读输出：

```bash
timeout 5s rostopic echo -n 1 /perception/planning
timeout 5s rostopic echo -n 1 /refer_path_msg/safety
```

新话题没有输出需核对感知转换节点版本/发布状态；参考路径 safety 已经为 true 时，应同步
检查原常规避障原因。进一步以新增日志辨认停车来源，不能将所有 safety 都归到起步检查。
