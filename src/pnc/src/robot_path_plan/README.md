# robot_path_plan 模块说明

该目录实现 `path_plan_node`：10 Hz 接收任务、定位、CAN 和感知数据，生成
`/refer_path_msg`、`/plan_path_msg` 与 `/path_plan_status`。这里记录现役调用链和文件
职责，便于后续按业务边界定位修改点。代码风格和改动纪律仍以 `src/pnc/README.md`
为准。

## 核心文件职责

| 文件 | 职责 |
|---|---|
| `path_plan_node.cpp` | ROS 薄入口：回调接线、传感器心跳、10 Hz 调度 |
| `path_plan_comply.h` | 规划编排接口、输入快照、输出消息与跨周期状态 |
| `path_plan_comply.cpp` | 几何辅助函数和五个职责分片的唯一编译入口 |
| `path_plan_task.inc` | 任务接收、CSV 拼接、作业路径、机场停车位 |
| `path_plan_perception.inc` | CAN/定位/感知输入、坐标转换、障碍距离 |
| `path_plan_reference.inc` | 周期主流程、20 点参考路径、D/R 挡碰撞判断 |
| `path_plan_output.inc` | 最终路径、安全覆盖、停车限速、关键点推进、可视化 |
| `path_plan_experimental.inc` | 休眠 lattice 入口及现役坐标/方向辅助函数 |
| `adaptiveHook.*` | 挂钩作业的自适应倒车路径 |
| `collisioncheck/*` | 车辆与障碍物矩形关系计算 |
| `common/` | 曲线、几何、路径、样条等通用算法 |
| `lattice/`、`lattice_plan/`、`reference_line/`、`speedplan/` | 已编译但主链未启用的实验规划算法 |

五个 `.inc` 仍由 `path_plan_comply.cpp` 按原函数顺序包含进同一翻译单元。它们不是独立
库，不能直接加入 CMake。

## 运行数据流

```text
/task_plan_msg ──CSV/作业路径/停车点──────────────┐
/navigation_msg ──当前位置、航向、RTK─────────────┤
/can_msg ──挡位、车速、急停、挂钩反馈────────────┤
/perception、前后扫描框 ──障碍物与起步观察───────┼─> PathPlanProcess()
/cloud/msg/command_msg ──停止/暂停/继续───────────┘
                                                       │
                                                       ├─> /refer_path_msg
                                                       ├─> /plan_path_msg
                                                       └─> /path_plan_status
```

`path_plan_node` 每周期调用顺序是：

1. `PathPlanProcess()` 推进全局路径关键点并计算停车速度上限；
2. `PublishReferPath()` 截取前方 20 点，执行 D/R 挡障碍判断；
3. `PublishPlanPath()` 叠加起步观察、急停、断网、等待区和云端暂停；
4. `PublishPathPlanStatus()` 发布任务进度与诊断状态。

后面的步骤可以覆盖前面的 `desireSpeed` 和 `safety`，调用顺序本身就是安全优先级。

## 任务到路径的主流程

`SetTaskPlanData()` 收到一条任务事件后：

1. 重新读取飞机位 YAML；
2. 判断是否切换了 `task_id`，真实切换时复位跨帧感知窗口和暂停状态；
3. 按 `pathList` 顺序读取并拼接 CSV；
4. 从当前定位寻找 `mKeypoint`，必要时生成车辆到原路径的连接段；
5. 在全局路径上计算 `mStopIndex` 和飞机位临时停车索引。

周期内 `UpdatePathInfo()` 只在当前关键点之后 80 点内寻找最近点，保证任务进度尽量单调
向前。`LimitSpeedByDistanceToStop()` 再根据普通行驶、挂钩、倒车入库或动作任务计算速度
上限和完成状态。

## 最终路径的安全覆盖顺序

`PublishPlanPath()` 的主要覆盖顺序如下：

1. 人工模式或参考路径不足时发布空路径和 0 速；
2. 新任务起步执行周边/T 路口观察；
3. 复制参考路径并对加速指令做低通；
4. 急停闩锁和断网状态压 0 速；
5. 等待区遇障碍压 0 速并报警；
6. 云端暂停再次压 0 速；
7. 写声光参数并发布 `/plan_path_msg`。

修改中间任一条件时，应继续向后检查所有 `mPlanPath.desireSpeed`、
`mPlanPath.safety` 和 `InitSafetyCheck` 的赋值。

## 常用业务修改入口

- 改任务接收、CSV 拼接或连接段：`path_plan_task.inc`
- 改挂钩/入库作业路径：`GenerateHookPath`、`GenerateParkPath`
- 改感知目标过滤和坐标转换：`path_plan_perception.inc`
- 改 D/R 挡碰撞检测：`PublishReferPath`
- 改停车距离和任务完成条件：`LimitSpeedByDistanceToStop`
- 改起步观察、急停、断网和暂停优先级：`PublishPlanPath`
- 改全局路径进度窗口：`UpdatePathInfo`
- 调试 lattice 前：先处理 `path_plan_experimental.inc` 中标记的既有缺陷

## 修改时必须保留的既有语义

- `.inc` 必须由 `path_plan_comply.cpp` 按当前顺序包含，不能独立编译。
- `PathPlanProcess`、`PublishReferPath`、`PublishPlanPath` 的调用和赋值顺序不可随意调整。
- `mKeypoint` 只向前搜索；闭环路径不能改成无界全局最近点。
- D/R 挡使用不同的障碍物来源和停车规则。
- `history_risk_vec`、`unsafe_vec`、`history_unsafe` 等跨帧窗口不能改成每帧局部变量。
- 成员声明顺序和既有标识符不调整；ROS 话题继续使用绝对名。
- 休眠 lattice、参考线和速度规划虽然主链未调用，仍有部分辅助函数被现役代码使用。

## 已知但本轮未修改的问题

- `task_id == 4` 会清空感知目标并启用测试限速。
- `emergencyStop` 是闩锁状态，解除依赖 `/robot/serial/rs232/`。
- 飞机位配置为空时，回退对象的 `id` 没有初始化；cppcheck 会报
  `uninitvar/uninitStructMember`，本轮保持既有行为。
- `handleDrivingPath()` 中除 `ADAPTIVEHOOK` 外的若干内层同值比较条件不可达。
- `LatticePlan()` 主链处于注释状态，函数内部仍标记“后面的代码有 bug”。
- 多处场地坐标、车辆尺寸、减速阈值和测试话题仍为硬编码。

这些都可能改变实车业务或安全行为，本轮只记录和解释，没有借整理之机修复。
