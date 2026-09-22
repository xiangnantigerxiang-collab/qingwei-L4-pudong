# 左一车道动态对向目标过滤验证（2026-09-18）

## 行为和范围

PNC 的 `/perception` 与 `/perception/planning` 两个入口使用同一方向过滤器。
仅 `type=1` 且地图绝对速度至少 0.2 m/s、与本车行驶方向夹角大于 135° 的目标被新增规则剔除。
同向慢车不使用相对速度判定；目标 `heading` 不参与运动方向计算。静止、低速、横穿/斜穿、
不确定方向保留；本车道和左二车道不受新规则影响。原 type=3/4 及尺寸过滤保持。

本车方向使用定位北向 0°、顺时针航向；D 挡正向、R 挡反转 180°。尚未收到定位、非有限
航向或未知挡位时保留目标；35 m/s 以上或非有限目标速度交由原输入校验处理。

剔除有效 ID 时同步删除新链旧轨迹/确认记录和旧链四帧风险历史，防止继续漏检外推。
新链在整帧校验完成后提交，坏帧/乱序/冲突重复 ID 不会部分删除旧障碍；全过滤有效帧继续
刷新心跳。其他目标、独立前后扫描及全局急停解除迟滞保持，已有急停不会因剔除一个目标
而越过原有解除条件。

生产改动共 6 个文件（含新增头文件）：

- `src/robot_path_plan/safety/lane_motion_filter.h`：方向分类和左一对向谓词；每帧计算一次方向。
- `src/robot_path_plan/safety/perception_safety.h/.inc`：观测输入接收方向过滤器，沿用已有 ID 清除表。
- `src/robot_path_plan/safety/perception_safety_adapter.inc`：根据定位/挡位建立过滤器并传入新链。
- `src/robot_path_plan/path_plan_perception.inc`：旧入口过滤及历史清理；定位回调记录已收到定位。
- `src/robot_path_plan/path_plan_comply.h`：辅助方法声明，初始化原有定位接收标志。

控制、转换、消息、配置、路径和第三方文件共 753 个保护文件哈希不变。上一轮终点减速与
1.5 倍轨迹生成的生产改动保持。输入原消息的字段、顺序及上游监控输出不被修改。

## 验证结果

| 验证 | 结果 |
| --- | --- |
| 真实规划、控制及公共算法编译 | 45 个翻译单元成功 |
| 感知核心 C++11，`-Wall -Wextra -Werror` | 5,360 项通过 |
| 同套 ASan/UBSan | 5,360 项通过，无诊断 |
| 真实规划入口和发布集成 | 362 项通过 |
| 单点区域限速 | 284 项通过 |
| 闸机安全覆盖 | 1,806 项通过 |
| 起步观察删除回归 | 756 项通过 |
| 终点减速/路线长度/路径边界 | 1,975 项通过 |
| 独立控制进程消费真实末端规划消息 | 36 项通过 |
| 修改前后既有业务差分 | 387 组完整消息、参数事件和状态逐字节一致 |
| 300 目标、1,000 次预热后 Observe/Evaluate | 0 堆分配，缓存 4,789,248 bytes |
| cppcheck（新增过滤器及观测核心） | 无诊断 |

核心加集成/既有专项共 10,579 项检查（不重复计入 sanitizer）。方向测试包含旋转、跨 360°、
0.2 m/s 和方向锥边界、旧 heading 与速度矛盾、同向慢车、其他车道、非法数据、旧轨迹清除、
重新进入确认和迟滞解除。集成测试将两个入口含对向目标时的完整发布消息与空场景比较，
同时逐字段比较加入其他停车/限速来源后的输出，避免把旧链已有的速度平滑当成过滤结果。

无 ROS1 的开发机使用由实际 `.msg` 生成的传输桩；整体因本机 Boost 要求采用 C++14，新增
核心单独按 C++11 严格编译。生产 CMake 未改。未启动实车节点、未部署 Orin；车端须重新
编译 robot 并重启规划节点后验证。

## 记录与复现

开工快照：工程同级 `planning_left_lane_before_20260918_171553.tar.gz`，包含 `src/pnc` 和
`workflow.md`。原始日志：`/tmp/pnc_left_lane_20260918_171553/`：

- `core_result.json`、`core.log`、`sanitize.log`、`allocations.log`；
- `build/result.json`：全部集成结果及最终生产源码 SHA256；
- `regression_result.json` 和 `regression_before/after/trace.txt`：既有业务字节差分；
- `scope_audit.json`、`before_sha256.json`：修改范围与保护文件哈希核对；
- `verify_remaining.py`：复用本轮完整编译对象执行最终集成及基线差分。

从工程根目录重跑全部集成：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 src/pnc/tests/planning/verify_point_speed_limit.py \
  --output /tmp/pnc-left-lane-check
g++ -std=c++11 -Wall -Wextra -Werror -O2 -I/tmp/pnc-left-lane-check/stubs \
  src/pnc/tests/planning/perception_safety.cpp -o /tmp/pnc-left-lane-check/core
/tmp/pnc-left-lane-check/core
g++ -std=c++11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined \
  -fno-omit-frame-pointer -I/tmp/pnc-left-lane-check/stubs \
  src/pnc/tests/planning/perception_safety.cpp -o /tmp/pnc-left-lane-check/core-sanitize
/tmp/pnc-left-lane-check/core-sanitize
g++ -std=c++11 -Wall -Wextra -Werror -Wno-mismatched-new-delete -O2 \
  -I/tmp/pnc-left-lane-check/stubs src/pnc/tests/planning/perception_allocations.cpp \
  -o /tmp/pnc-left-lane-check/allocations
/tmp/pnc-left-lane-check/allocations
```

将本轮快照解压到独立目录后，可用既有 `verify.py --baseline <解压后的 src/pnc>` 重跑
387 组严格差分。本轮差分输出 SHA256：
`df8c0c704b2a45414bb8793c1c700d20eef1352a30378742bf83a45e89b98044`。
该输出含临时夹具路径，改变验证目录会改变 SHA；前后字节一致性不受影响。
