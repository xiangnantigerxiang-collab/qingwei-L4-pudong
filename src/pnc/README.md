# PNC：使用方法与编码规范

更新：2026-09-22。catkin 包名 **robot**。涉及本模块的代码阅读、修改、清理或重构前，
必须完整阅读本文件及 [工程风格记忆](../../docs/CODING_MEMORY.md)。用户当前明确要求优先，
不能让旧风格采样或局部历史缺陷覆盖后续明确规范。

## 当前功能与近期变化

- **09-22最新实车反馈：** 用户确认本轮control减速刹车“效果很好”，当前D挡实现作为后续基线。
  业务只改control的三个文件，R控制与横向算法保持；本次补录日志未再调参。
  完整行为、原有本机验证及本次反馈见[当前专项报告](tests/control/forward_speed_smooth_20260922.md)。
- **当前D纵向：** 名义新增预瞄为导航速度×4秒，保留已发现且未驶过的前方曲率位置，按沿程包络提前降速；
  静态曲率参考k=0.04时1.4m/s、直线5m/s需求及前后各2m几何抗噪保持。
  电机参考普通减速0.6m/s²、恢复0.4m/s²、内部jerk0.5m/s³，硬规划/安全上限立即抢占。
  D×13.5、当前速度+0.5及5%保底保持，R×10/45%保持，不恢复额外普通起步斜坡。
- **当前D制动：** 积分仅由原规划desireSpeed下降发起，默认门槛0.1m、上限0.2m；液压首请求3%、普通上限15%。
  根据真实液压及减速度协调接入/释放，大需求缩短观察并加快已响应后的增量；不再统一固定禁油1.8秒。
  正常末停锁存有效到位证据，手动D或N停稳且新鲜零反馈连续0.5秒可清D专属故障，独立急停保护保持。
  旧比例补刹dec×50/15、固定6m预瞄及仅手动D恢复是历史，不与当前版混用。
- **实际速度/横向：** PNC与ultra以导航gpsSpeed作为实际车速，CAN保留挡位/模式/液压状态；
  无效导航按安全优先级处理，不回退CAN速度。D横向速度相关权重已因实车效果更差回滚至固定70%/30%。
- **09-22围栏迁移修复：** task_plan拒绝不可达任务、截停取消依赖作业、保留车头角点保护；
  planning检查实际路径与任务代次，失联叠加停车；HMI围栏失联显示未知，control围栏逻辑已清除。
  需配套重编消息及节点，详见[task_plan说明](src/robot_task_plan/README.md)。本次刹车反馈不替代围栏验收。
- **其他近期planning：** D参考轨迹最多60点；四列CSV按点限速，两份cargo限速列09-22按相邻差≤0.01m/s平滑；
  D转向灯改为18m意图扫描/防抖/保持。闸机前向8m、左二车道过滤及D前向障碍减速继续保留。
  planning旧terminal_control断言与新末停行为有1项差异，仍待核对，不能称全工程回归全过。
  说明与部署入口见[规划README](src/robot_path_plan/README.md)和[workflow](../../workflow.md)。

较早各轮中间参数及完整计数已集中到[历史归档](../../docs/history/2026-09-22-before-roadtest-log-cleanup/README.md)，
当前control测试统一从[控制回归](tests/control/README.md)进入；不把旧阶段“全部通过”当作当前整套结果。

## 使用方法

在车端工程根目录、已有ROS1及相关依赖/消息后执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg robot -j4
source devel/setup.bash
```

干净工作空间需先构建依赖，尤其 `ivlocmsg`、`hdmap`、`gantry_detect` 的库/消息；
以 CMake/package.xml 为准。HDMap的x86库不能用于Orin，必须重编aarch64。
`--pkg` 会保留catkin白名单，后续全量构建显式清除 `CATKIN_WHITELIST_PACKAGES`。

整栈由HMI或工程根 `roslaunch launch/control.launch` 启动，不能与已运行组件重复启动。
ultra_command已从该launch解绑，由HMI独立卡片或start_l4.sh启动；各节点也可单独启动，但不会自动补齐全部输入：

| 内容 | 命令（已source工作空间） | 详细说明 |
|---|---|---|
| 任务 | `roslaunch robot robot_task_plan.launch` | `src/robot_task_plan/task_plan_core.*`及任务YAML |
| 规划 | `roslaunch robot robot_path_plan.launch` | [规划README](src/robot_path_plan/README.md) |
| 控制 | `roslaunch robot robot_control.launch` | [控制README](src/robot_control/README.md) |
| 导航 | `roslaunch robot robot_navigation.launch` | 输入定位与参考坐标见launch，勿自行重设原点 |
| 感知转换+控制转发 | `roslaunch robot robot_canbus.launch` | [转换README](src/robot_perception_convert/README.md)；该launch不是底盘canbus节点 |

规划参数来自 `param/perception_safety.yaml`，任务/路径来自 `param/`、`path/`；不要修改
用户文件来“让测试通过”。源码常量修改需重编，启动加载的CSV/YAML修改后需重启。
控制积分门槛的运行参数见控制README，先核对运行值是否覆盖源码默认值。

## 架构与职责

| 层次/目录 | 约定 |
|---|---|
| `*_node.cpp` | ROS接线、参数读取、回调转发与现有周期调度；保持薄入口 |
| `*_comply.h/.cpp` | 业务编排、输入快照、周期处理和输出；不为局部需求改整体架构 |
| `robot_task_plan/task_plan_core.*` | 当前纯业务core与有序事件出口，由node转换到ROS；不把这个结构推到其他模块 |
| `robot_path_plan/` | `.cpp`唯一入口，几何辅助及业务`.inc`按固定次序包含；task/reference/safety分职责 |
| `robot_control/` | 控制优先级与D/R分流；ACC `.inc`保持同一翻译单元 |
| navigation / perception_convert | 现行单文件节点及本地算法分片，沿用已有结构 |

`.inc`不得单独加入CMake。移动代码时保留命名空间、宏、include依赖、静态对象及状态生命周期。
canbus目前采用node/Comply，历史canbus_core已经下线，不能再称其为当前PNC结构模板。

## 命名与格式

| 对象 | 新代码沿用的习惯 |
|---|---|
| 类/方法 | PascalCase，编排类以Comply结尾；`Set*`、`*Process`、`Publish*`按职责命名 |
| 成员/输入参数 | 常用 `m` / `t` 前缀；已存在的无前缀或其他形式保持 |
| 局部变量/文件 | snake_case；节点入口 `*_node.cpp` |
| 回调/pub/sub | 保留已有CallBack/Callback拼法；发布/订阅器snake_case加_pub/_sub |
| 枚举/结构 | 沿用现有大写枚举与`*_S`，不重命名消息或旧字段 |

自有C/C++统一4空格，左大括号同行，条件关键字与括号之间不加空格：

```cpp
if(condition) {
    UpdateState();
} else {
    KeepPreviousState();
}
```

函数、类、结构体、命名空间遵循相同括号规则。标准为C++11；优先使用已有标准库能力，
不用更高标准特性或无关依赖。需要索引/相邻点/采样步进时使用索引循环。
第三方 `3rd-party/`、Eigen、unsupported不格式化；include/using不排序，不顺手改名。

根配置为 [`.clang-format`](.clang-format)。确有格式任务时，在工程根运行：

```bash
python3 src/pnc/tools/format_cpp.py --clang-format /path/to/clang-format --check
```

该工具验证有效token、预处理指令和include顺序；格式整理与业务修改分开验收。
09-14之前的Allman/Google混合格式仅为历史，不能据邻近旧示例覆盖当前明确格式。

## ROS、状态与算法习惯

- 话题使用绝对名；保持现有队列、TCP_NODELAY设置和单线程调度，不为局部功能引入线程池。
- 标称周期：planning 10Hz；control/task_plan/can_comm 20Hz；navigation 50Hz；转换主循环100Hz。
  实际发布还受回调驱动约束。**当前can_comm虽声明Rate(20)，循环缺sleep；不能称其已限为20Hz。**
- 新参数先初始化、校验单位和有效范围；保持现有启动读取/热读/getCached语义，避免高频文件I/O。
- 已有rosparam指令和ZOH重发属于协议；不能擅自换通道或把冻结旧值解释成新鲜输入。
- 既有函数static不随便移动；新增状态按实例生命周期组织，明确手动/自动、任务、挡位和超时重置。
- 几何优先直白标量公式；注明地图系/车体系，heading为北零顺时针，数学yaw换算为`90-heading`。
- 关键阈值写清单位与净距参考点；中文注释解释原因。诊断跟随本文件日志方式并按变化/限频输出，
  不复制过去的热循环printf噪声、缺少初始化或无校验读取。

## 注意事项与容易疏忽的点

1. 只在用户范围内改逻辑。09-19新增D功能不能直接共用到R；R已有末端修复也不能被误回滚。
2. `safety=1`有多个来源，普通限速只取小，不能因某一路清场就清零其他安全状态。
3. `/perception`有历史目标；`/perception/planning`是当帧物理框。监控空场景不等于专用输入健康。
4. 速度误差积分单位为m，不直接对应制动百分比；目标速度参考不等于实车液压减速度，需结合真实反馈检查。
5. `can_msg/can_comm_msg`在canbus/pnc/simview存在副本，字段更改必须全树查读写与ROS MD5。
6. `mControlData`等发布字段的赋值不可按“局部没读者”删除。GBK文件用`rg -a`，删除前查全链。
7. 空帧、断流、重复时间戳、真实帧确认不能混淆；缺输入不能伪造带新时间戳的健康空消息。
8. 地图、路径、YAML、外廓和当前标定D×13.5/R×10不顺手调整。当前起步1m与历史2m测试差异应明确记录。

## 验证方法

```bash
python3 src/pnc/tests/compile_all.py --output /tmp/pnc-all-nodes
python3 -B src/pnc/tests/control/verify_forward_brake.py --output /tmp/pnc-forward-brake
```

当前D平顺/状态及R/N差分的完整命令见[控制回归](tests/control/README.md)；旧verify.py包含过时业务断言。
规划入口与已知旧夹具差异见 [规划测试README](tests/planning/README.md)；当前感知发布使用
[转换模块验证](src/robot_perception_convert/README.md#验证)，不要默认旧HDMap接入夹具仍满足新空帧语义。
消息桩必须从真实`.msg`生成。本机部分Boost编译需C++14，生产仍为C++11；记录真实执行结果，
不得把历史计数或主机测试称为本轮/实车通过。整理前完整规范见
[归档](../../docs/history/2026-09-19-before-docs/src/pnc/README.md)。
