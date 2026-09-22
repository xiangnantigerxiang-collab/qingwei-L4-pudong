# 导航实际车速切换回归（2026-09-21）

验证 PNC 和 ultra_command 的真实业务源码；ROS 桩由工程实际消息定义生成。
CAN 挡位/模式/载荷仍有效，仅实际速度弃用 CAN。HDMap 没有 CAN 实测速度入口，
其样条导数、路径目标速度不属于替换范围。

在工程根运行：

```bash
python3 -B src/pnc/tests/planning/verify_map_speed_limit.py --output /tmp/nav_planning
python3 -B src/pnc/tests/control/verify.py --output /tmp/nav_control
python3 -B src/pnc/tests/control/verify_curve_history.py --control-build /tmp/nav_control --output /tmp/nav_curve
python3 -B src/pnc/tests/navigation_speed/verify.py --planning-build /tmp/nav_planning --output /tmp/nav_speed
```

最后一条复用第一条的全量构建，检查对象新于生产源码，避免旧头文件/类布局混用。
临时矩形、日志、桩和二进制仅写输出目录，业务 CSV、参数和库不修改。

本轮通过：PNC 53 个翻译单元、6 个节点链接；C++11 控制节点和 ultra_command 节点编译；
153 项控制回归、695 项曲率检查；规划现有地图/闸机/末端/感知/左二过滤等回归通过。
历史起步完整套件仍含 2m 边界期望，当前源码为 1m，第一条明确只运行该套件的左二专项；
本轮未改该半径，也未宣称历史 2m 测试通过。

新增专项覆盖：

- 139 项规划断言：矛盾 CAN/导航输入下的状态车速、平滑权重、真实感知制动距离，
  两帧导航起步确认、重复时刻/阈值/无效值、导航断流、CAN 状态过期及 R→D；
  其中42项验证NaN/±Inf的最终停车、提前返回、R挡和恢复后不影响其他安全来源。
- D/R/减速共 18 组完整控制输出对照、216 条消息；CAN 为负数、0、99、NaN/Inf
  均不改变相同导航输入下的输出，控制消息及 can_comm 转发车速为导航值。
- 2 项独立制动期望；12 项时效和终点制动断言：新鲜 CAN 不掩盖导航断流，
  未介入积分清零、已介入补刹保持、导航不能替代 CAN 状态、普通终点缓刹/保持制动。
- 6组/210项导航异常与恢复断言：D/R分别输入NaN/±Inf，零驱动且转向/目标有限，
  CAN重发不能解除异常保护；导航恢复不解除其他safety，安全来源清除后恢复原控制。
- 29 项 ultra_command 断言：真实节点订阅导航且不订阅 CAN、三种初始化路径、
  导航缺失/NaN、严格 0.5m/s 边界、单向闩锁与新任务复位。

旧综合夹具 `planning/navigation_feedback.h` 同步发送 CAN 状态与对应导航速度，
保持其原测试场景；本目录另从两个真实入口分别发送矛盾输入，避免同值夹具掩盖来源错误。
曲率测试自行输出历史限速，不依赖生产中已删除的数值调试打印。
09-21后续按用户要求加入规划减速触发条件，本目录制动/时效夹具先发送真实`desireSpeed`下降沿，
继续独立验证导航来源和断流处理；仅设置实际超速已不构成积分请求。触发专项见
[控制回归](../control/README.md)。
本机无 ROS1，以上结果不覆盖 ROS 调度、车端编译和实车效果。
后续独立核对发现并修复异常导航值缺陷；186组/6197条控制消息、387组规划状态的
同输入前后差分结论、原始复现与已知边界见 [复核记录](review_20260921.md)。
