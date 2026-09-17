# planning 重构校验

在没有 ROS1 的开发机上编译真实 planning 源码，并把相同业务输入分别送入原版和当前版，
对照完整发布消息、参数访问/写入/发布顺序、路径索引、任务状态和跨帧历史。
测试不访问 ROS master，不启动实车节点，也不修改路径或任务配置。

## 运行

从工程根目录执行，`--baseline` 指向需要对照的 **pnc 目录**：

```bash
python3 src/pnc/tests/planning/verify.py --baseline /path/to/baseline/pnc --output /tmp/planning-check
```

格式统一单独以格式化前的快照验证，覆盖全包自有 C/C++ 及预处理指令：

```bash
python3 src/pnc/tests/planning/check_structure.py --format-only /path/to/formatting_before/pnc src/pnc
python3 src/pnc/tools/format_cpp.py --clang-format /path/to/clang-format --check
python3 src/pnc/tests/compile_all.py --output /tmp/pnc-all-nodes
```

2026-09-14 格式化前快照为工程同级 `pnc_before_brace_style_20260914_192628.tar.gz`，
其中的 `src/pnc` 已包含用户本次 planning 整理。审查详情见 [review_20260914.md](review_20260914.md)。
`check_structure.py` 不加 `--format-only` 时仍是 09-12 重构专用检查，保留函数不得删除；
09-14 用户明确删除的两组调试接口会让该历史模式报出删除，不能将它当作新格式验收命令。

重构前快照为工程同级的 `planning_before_refactor_20260912_134949.tar.gz`，其中保存
`src/pnc/` 和 `workflow.md`；解压到单独目录即可提供基线。不要覆盖正在使用的工程。

依赖：Python3（新避障验证另用 PyYAML）、g++、Boost、yaml-cpp、jsoncpp，以及本包的 Eigen 和 x86 OSQP 预编译库。
本机 Boost 头要求 C++14，因此桩验证使用 `-std=c++14`；生产 CMake 的 C++11 设置保持原样。

## 单点区域限速验证

```bash
PYTHONDONTWRITEBYTECODE=1 python3 src/pnc/tests/planning/verify_point_speed_limit.py \
  --output /tmp/planning-point-speed-limit-check
```

编译真实 Comply/node 及其公共算法，运行 `point_speed_limit.cpp`、既有闸机和感知集成检查。
临时 CSV 仅写入输出目录下的 `fixtures`，不修改用户配置。覆盖 CRLF、空文件、非法行、
零限速、5 m 圆形边界、重叠取小、文件缓存、进出区域、平滑后限速，以及急停/断网/暂停/
起步/等待区/人工/短路径与闸机安全标志保持。检查完整发布消息，只有 `desireSpeed` 可以变化。
本机仍使用消息桩及 C++14，不能替代 ROS1/Orin 的编译和运行验证。

相对新增功能前的旧业务差分，启动时会多一次 `get:path_dir`，用于读取限速文件；
未命中限速区域时，其余参数事件、完整消息和状态应保持原样。

09-17 本机验证通过：41 个规划及公共算法单元编译，区域限速 284 项、闸机 1,806 项、
感知集成 52 项检查通过。修改前后 387 组旧业务快照，在严格核对 11 次夹具初始化各新增一次
`get:path_dir` 后，其余消息、状态及参数事件逐字节一致；211 个保护文件哈希不变。
日志与差分结果位于 `/tmp/planning_point_speed_limit_20260917_151638/`。

## 覆盖范围

09-17 后续按用户要求启用紧急三帧确认和车道外过滤。当前测试结果及边界见
[策略修改验证](obstacle_policy_verification_20260917.md)。核心 4,995 项、ASan/UBSan、
真实规划发布集成 257 项、区域限速 284 项、闸机 1,806 项均通过；与本轮开工快照的
387 组既有业务输出一致。六种故意恢复旧错误的隔离变异均被测试检出。
`perception_safety.cpp` / `perception_safety_integration.cpp` 已同步新规则，后文首期计数为历史记录。
可用现有 `verify.py` 加 `verify_perception_safety.py` 复现：以
`planning_policy_before_20260917_164328.tar.gz` 解出的 `src/pnc` 作为 `--baseline`，
将前者输出目录作为后者的 `--legacy-build`；区域限速继续使用 `verify_point_speed_limit.py`。

- `check_structure.py`：50 个未改实现的原有函数逐 token 比较；检查旧接口和成员声明
  顺序、消息定义、13 个 `.inc` 包含链，防止漏编译或重复定义。
- `generate_stubs.py`：ROS 传输/时间/参数桩；robot 消息逐字段从当前 `.msg` 生成，
  保留数值类型和数组形态，避免手写消息字段与实物不一致。
- `verify.py`：编译 CMake 中 planning 使用的 39 个共享算法单元，再分别编译原版/当前版
  Comply 和 node，链接节点及测试程序。算法及头文件的有效 token 相同才允许共享对象，
  从 09-14 起允许经过独立格式等价检查的空白变化。
- `../compile_all.py`：从真实 CMake 目标表取得 53 个翻译单元（含 `.cc`），编译并链接六个 PNC 节点。
  复用真实 `.msg` 生成器，补齐其他节点需要的 ROS/TF 编译桩，不启动 ROS 或车辆节点。
- `regression.cpp`：起步观察、人工/短路径/结束退化、急停/暂停/断网、前向碰撞与历史窗、
  后向 2.5 m 边界、ultra 早退边界、真实 CSV 及多段装载、样条连接、重复任务/任务切换、
  各任务停车阈值、机位临停、等待区、关键点推进、前后扫描转换、旧轨迹复用和固定种子组合。
- `gantry_safety.cpp`：调用真实闸机回调，并捕获最终发布消息；覆盖消息四种布尔组合、
  无消息/保持最新消息、开闭/失效切换、正常/倒车/人工/短路径/空路径/起步输出、
  急停/暂停/断网/已有安全标志及任务复位。逐字段检查仅发布的 `safety` 可变化，
  内部规划状态、声光、参数事件和发布次数/顺序均保持。由 `verify.py` 一并运行。

测试既有明确业务断言，也有完整状态差分。桩固定时间、记录参数事件，并在每次发布时
捕获消息，能发现“最后结果相同，但中间多发/漏发一条消息”这类顺序变化。

输出目录中的 `result.json` 保存快照数量和 SHA-256；编译、链接、运行日志分目录保存，
发生差异时生成 `trace.diff`。保留的历史问题见
[`robot_path_plan/README.md`](../../src/robot_path_plan/README.md#校验与已知问题)。

## 2026-09-12 最终校验结果

- 39 个共享算法单元和原版/当前版各自的 Comply、node 编译成功；节点及回归程序完整链接
  成功，并另行确认不使用链接垃圾回收时也无缺失符号。
- 387 个状态快照、发布消息和参数事件逐字节一致，所有业务断言通过；
  Comply 编译告警 64 → 62，没有新增编译告警。
- 50 个原有函数 token 一致；原有成员/接口/消息定义和节点接线、频率保持一致；
  13 个分片均恰好包含一次。
- 人工复查了提取函数的输入输出、早退位置、局部变量生命周期和安全覆盖顺序；
  修正新提取函数的局部变量遮蔽，修正固定 80 点搜索范围等旧注释。
- cppcheck 全依赖扫描遇到 Eigen 宏解析错误，未将其作为通过依据。另对展开业务分片、
  排除外部 include 的代码作定向检查，未发现新增 warning/error 级诊断；仍保留原有
  不可达作业分支、未初始化回退 id 和 printf 类型等诊断，详见模块说明。

本机最终记录：`/tmp/planning_refactor_verify/final_review/result.json` 及同目录编译/运行日志。
结构变化：`PublishReferPath` 499 → 35 行、`PublishPlanPath` 213 → 50 行、
`calcuGlobalPath` 137 → 20 行、`LimitSpeedByDistanceToStop` 173 → 26 行；具体计算已按
任务、参考路径、安全输出职责移入阶段函数。

## 验证边界

ROS 传输和外部消息由桩替代；本测试不覆盖真实 ROS1 消息序列化、catkin 并行生成、
硬件时序及车辆动态。它验证重构行为与当前基线一致，不证明原有停车规则均正确。
代码上车前仍需在 ROS1/车载工具链重编并回放/验收关键任务。

## 2026-09-17 闸机接入验证

开工快照：工程同级 `planning_before_gantry_20260917_103937.tar.gz`。
该快照含一条用户已有的、尚未完成接线的 `gantry_detect/gantry_detect.h` 引用；实际消息
是 `gantry_state.h`。差分用副本只移除了该未使用的不存在头文件，不改变旧业务代码；
原始快照完整保留。正式代码仅在节点包含正确的消息头，Comply 接收两个布尔值。

```bash
python3 src/pnc/tests/planning/verify.py \
  --baseline /tmp/planning_gantry_20260917_103937/before/pnc \
  --output /tmp/planning-gantry-check
```

39 个共享算法单元、前后 Comply/node 编译及节点完整链接通过；旧 387 组快照、完整消息
和参数事件逐字节一致，新增 1,266 项闸机检查通过。真实 PNC CMake 配置的依赖图确认
只有 `path_plan_node` 增加 gantry 消息目录/生成目标；catkin_pkg 包发现和拓扑排序确认
`ivlocmsg -> gantry_detect -> robot`，根模块通过 `src/gantry_detect` 相对软链接加入工作空间。

本机 Boost.Geometry 要求 C++14，桩验证沿用 `-std=c++14`；生产 CMake 仍为 C++11。
消息从真实 `.msg` 生成；CMake 检查使用模拟 catkin 配置，不代替 ROS1/Orin 构建。
原始日志：`/tmp/planning_gantry_20260917_103937/`，汇总见
[`gantry_verification_20260917.json`](gantry_verification_20260917.json)。

### 同日复核：无效/开闸不能清除其他 safety

按用户要求再次从原始基线编译差分，387 组状态/消息/参数事件一致。补充 60 个连续交接帧、
540 项断言，闸机测试共 1,806 项通过。新断言直接给出期望的发布值和内部值，覆盖
“关闸期间其他安全原因出现 → 开闸/失效 → 其他原因解除”，防止对照实例与被测实例
同时误清零却比较相同。隔离副本中三种错误（无效时直接赋 0、发布后恢复为 0、输入回调
清零已有标志）均被测试检出；正式业务源码保持原样。

同时核对 ROS Noetic `publish(const M&)` 的同步序列化调用链，确认发布后的内部原值恢复
不会改写本次已序列化消息。检查依据、适用边界和日志见
[`gantry_safety_review_20260917.md`](gantry_safety_review_20260917.md)。

## 2026-09-17 感知避障与性能验证

本次开工快照为工程同级 `planning_perception_before_20260917_140441.tar.gz`，解压后使用其中的 `src/pnc`。
先运行上述 `verify.py` 得到基线结果和公共算法对象，再执行：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 src/pnc/tests/planning/verify_perception_safety.py \
  --baseline /path/to/before/src/pnc \
  --legacy-build /tmp/planning-check \
  --output /tmp/perception-safety-check

PYTHONDONTWRITEBYTECODE=1 python3 src/pnc/src/robot_perception_convert/tests/verify_publish.py \
  --output /tmp/perception-publish-check
```

`--legacy-build` 可省略，此时自行编译公共算法，只执行新功能及性能测试，不重跑旧快照/闸机检查。
性能测试顺序交替执行旧、新版本各三轮；测试时应避免其他编译或高 CPU 工作并行运行。

| 检查 | 最终结果 |
|---|---|
| 独立 C++11 核心及 ASan/UBSan | 各 4,762 项通过 |
| 实际规划回调/发布/安全覆盖 | 31 项通过 |
| 基线旧业务完整消息/状态/参数事件 | 387 组逐字节一致 |
| 原闸机安全覆盖 | 1,806 项通过 |
| 感知真实 CMake C++11、SDK 动态链接与发布集成 | 4,414 项通过，同样通过 ASan/UBSan |
| 300 目标、1000 次预热后观测更新＋判断 | 堆分配 0 次，缓存 4,133,888 bytes |
| 新核心 cppcheck warning/performance/portability | 无诊断 |

核心测试包含独立凸多边形相交判据校对 2,000 个随机连续运动样本，不能只靠被测算法与自身比较。
业务覆盖真实帧确认、一次误检、历史鬼影、同向超车/切入、物理朝向与运动朝向不一致、
空车/载荷外廓、输入失效、急停解除迟滞、减速度/jerk 边界，以及已有零速/安全标志保持。
发布测试直接把实际 converter 输出喂给实际核心，验证两边角点轴向契约。

三轮中位数，真实旧/新规划代码输入更新＋前向检查耗时（主机，ms）：

| 目标数/分布 | 旧 P50 | 新 P50 | 旧 P99 | 新 P99 |
|---|---:|---:|---:|---:|
| 20/稀疏 | 0.059870 | 0.002784 | 0.067343 | 0.004066 |
| 100/稀疏 | 0.212085 | 0.011262 | 0.244614 | 0.014013 |
| 300/稀疏 | 0.678485 | 0.031853 | 0.794205 | 0.036788 |
| 300/密集 | 0.674334 | 0.025646 | 0.772244 | 0.028459 |
| 1000/稀疏 | 2.181631 | 0.051226 | 2.445566 | 0.055987 |
| 1000/密集 | 2.261661 | 0.080792 | 2.613327 | 0.089334 |

另以实际旧/新 converter 回调运行 20/100/300 目标、20/100 Hz、部分目标频繁新生共 6 组场景，
原 `/perception` 的 id/x/y/vx/vy/heading/confidence/type 输出校验和一致。
原始数据、源码 SHA 和适用边界见 [perception_safety_verification_20260917.json](perception_safety_verification_20260917.json)，
临时构建日志保存在 `/tmp/planning_perception_20260917_140441/`。

这里只证明覆盖场景与测量代码路径：未测 ROS1 序列化/调度、Orin 总 CPU 或整车响应。
旧快照一致性不涵盖工作区同时存在的相机心跳判断、任务 YAML 和路线 CSV 变更；这些变更未由本次回滚。
空车/载荷实际尺寸及最差载荷减速能力仍需实测标定，详见
[参数和部署边界](../../src/robot_path_plan/safety/README.md#参数与部署)。
