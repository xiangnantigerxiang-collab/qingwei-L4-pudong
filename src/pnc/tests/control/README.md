# 控制回归

更新：2026-09-22。测试链接真实控制源码，ROS桩由工程消息生成；只在输出目录生成夹具、日志和二进制。
本机没有ROS1，桩验证不覆盖车端调度或真实液压响应。历史阶段与当前结论分开记录。

## 当前D挡沿程限速与平顺控制

**2026-09-22用户反馈本轮减速刹车实车“效果很好”，当前版本作为后续基线。**
业务仅修改control的D挡，R及横向算法保持。本次为用户定性实车确认，没有新增bag或量化制动指标。
[当前参数、实车反馈、验证与部署说明](forward_speed_smooth_20260922.md)为现行详细记录；
下列计数是上轮已完成的本机验证，本次日志整理没有重跑测试。

在工程根运行，最后一条基线使用本轮冻结的完整control目录：

```bash
python3 -B src/pnc/tests/control/verify_forward_brake.py --output /tmp/control_forward_brake
python3 -B src/pnc/tests/control/verify_brake_state_revision.py --output /tmp/control_brake_state
python3 -B src/pnc/tests/control/verify_forward_speed_smooth.py --control-build /tmp/control_forward_brake --output /tmp/control_forward_smooth
python3 -B src/pnc/tests/control/verify_non_d_brake_compat.py --baseline-control <本轮改前完整control目录> --output /tmp/control_non_d
```

| 范围 | 上轮结果 |
|---|---|
| C++11节点构建、液压反馈及安全优先级 | 构建通过，11,288项断言通过 |
| 执行状态/跨挡/时序 | 20场景、10,685项断言通过 |
| 沿程预瞄/电机参考/正常末停/延迟电机响应 | 11场景、1,840项断言通过 |
| R/N冻结基线完整消息对照 | 220场景、8,925条完整消息一致，含48组D转非D场景 |
| 静态曲率标定 | 4,901点与冻结原版一致 |

基线、原始日志与部署三文件在工程同级 `control_d_smooth_20260922_103950/`；
历史bag回放和性能边界见专项报告。回放是同输入输出比较，不能换算为实车顿挫改善比例。

## 当前验证边界与旧夹具差异

- `verify_curve_history.py`的完整check仍要求“预瞄峰值立即成为当前位置限速”，不适用于当前沿程包络；
  当前只复用静态标定驱动，不把旧曲率全套计数当作本轮通过数。
- 旧`verify.py`、`launch_speed.cpp`、`brake_request.cpp`保留D×15、旧比例制动及先前策略断言；
  `--curve-preview`也含旧制动预期，不能作为当前全套验收入口。原先23项主套件差异属于历史基线记录。
- planning的`verify_map_speed_limit.py`在最新复核中有1项`terminal_control`旧末停断言FAIL；
  还原转向灯文件后仍复现，与新control末停行为的契约差异待核对。本次没有修改断言来消除失败。
- ROS桩/x86回放不覆盖Orin调度、实车载荷、液压死区及急停距离；用户本次好评不新增这些量化覆盖。
- 非D差分基线须包含`control_comply.cpp/.h`及全部依赖`.inc`，不能混用其他阶段快照。

## 前期记录索引（均为历史阶段）

| 阶段 | 记录与现行关系 |
|---|---|
| 09-22早期制动状态修复 | [状态专项](brake_state_revision_20260922.md)；跨挡/时钟等修复保留，观察期、释放/故障恢复等随后调整，以当前报告为准 |
| 09-21初版反馈制动 | [反馈专项](brake_feedback_20260921.md)；取代旧比例制动，但其统一观察/固定禁油等并非当前最终行为 |
| 09-21动态预瞄 | [4秒预瞄](curve_preview_4s_20260921.md)；名义4v范围保留，后续增加前方位置记忆和沿程包络 |
| 09-21曲率折中 | [1.4m/s参考点](curve_turn_midpoint_20260921.md)；当前静态C(k)保持该标定，取代偏慢的[1.0m/s版](curve_turn_1m_20260921.md) |
| 09-21抗噪/早期复核 | [抗噪修复](curve_noise_fix_20260921.md)当时获用户实车确认；[旧复核](curve_preview_review_20260921.md)按当时版本理解 |
| 导航速度统一 | [导航专项](../navigation_speed/README.md)；CAN不再提供控制实际车速，时效/异常各有独立用例 |

旧起步斜坡删除、规划下降授权、临时调试输出及多轮制动计数已从入口合并；原始过程保留在
[workflow原文归档](../../../../docs/history/2026-09-22-before-roadtest-log-cleanup/README.md)。
工程同级 `pnc_remove_launch_ramp_before_20260921_111448.tar.gz`、
`pnc_brake_request_before_20260921_094612.tar.gz` 等当轮快照保留，不覆盖当前源码。
历史报告的“待实车/未部署”描述当时阶段，最新D挡减速刹车状态以上方09-22用户反馈为准。
