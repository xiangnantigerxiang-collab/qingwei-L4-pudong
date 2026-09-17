# planning 闸机 safety 复核（2026-09-17）

结论：本次接入未发现会把其他业务的 `safety=1` 误清为 0 的问题，未发现新增的其他业务逻辑变化。
本轮仅补测试和记录，生产代码未修改。本机 ROS 桩验证，不等同于 ROS1/Orin 实车验收。

## 发布规则与源码审查

当前实现等价于：

```text
最终发布 safety = 原规划 safety || (active && !gantry_open)
```

| 原规划 safety | active | gantry_open | 最终发布 safety |
|---|---|---|---|
| 0 | false | false / true | 0 |
| 0 | true | true | 0 |
| 0 | true | false | 1 |
| 1 | false | false / true | 1 |
| 1 | true | false / true | 1 |

- `SetGantryState()` 只更新自己的 `mGantryStop`，不写 `mPlanPath` 或 `mReferPath`。
- `PublishFinalPlanPath()` 先保存原规划值，闸机条件满足时才写 1；没有无效/开闸时写 0 的分支。
  发布后恢复的是进入函数前的原值，不是固定 0，因此其他安全原因仍保留。
- 正常、手动、短路径/空路径两条最终发布入口均经过该函数，其他发布、声光、速度和任务顺序不变。
- `mGantryStop` 不被任务复位清除；每条新消息覆盖闸机条件，解除只停止该项覆盖。
- 既有参考路径复制、碰撞去抖、任务复位等对 safety 的写入完整保留。本结论不宣称原系统各规则本身全部正确。

这里的“无效”按消息协议指 `active=false`；`gantry_open=true` 表示闸机打开。
启动未收到消息时不叠加；接收中断时保持最后一条消息的状态，没有新增超时自动清零。

## 发布后恢复原值是否影响已发布消息

实际调用为 `ros::Publisher::publish(const M&)`，没有传递共享消息指针。
核对 Noetic 官方源码：该重载创建空的 `SerializedMessage` 并传递序列化回调，
`Publisher::publish()` 同步调用 `TopicManager::publish()`；后者在缺少共享消息指针时
选择序列化，调用 `serfunc()` 取得字节缓冲后再交付消息。
因此需要发布时，序列化发生在本次 `publish()` 返回之前；随后恢复本地字段不会修改该缓冲。
规划使用单线程 `spinOnce()`，该恢复区间也不执行新的业务输入回调。

来源：[publisher.h](https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/include/ros/publisher.h#L94)、
[publisher.cpp](https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/publisher.cpp#L82)、
[topic_manager.cpp](https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/topic_manager.cpp#L659)。
这是官方实现的源码核对，未在本机运行 ROS1 网络传输。

## 编译、差分与连续交接测试

- 39 个共享算法单元、修改前后 Comply/node 重新编译，两个完整节点及测试程序链接成功。
- 与闸机接入前基线比较：387 组完整发布消息、参数事件、内部状态逐字节一致。
  基线仍只移除了原来未使用且不存在的 `gantry_detect/gantry_detect.h`，其他代码未调整。
- `gantry_safety.cpp` 通过 1,806 项检查；原 1,266 项之外，新增 5 类输出路径 × 12 个连续帧，
  共 540 项断言，检查回调不写路径、发布结果、发布后内部原值及其他字段/声光/事件顺序。
- 连续交接覆盖其他安全原因在关闸期间出现、开闸、失效、其他原因解除，再次关闸/解除；
  独立指定每帧期望值，避免两份当前实现的共同错误被差分比较掩盖。
- 去掉新增接口/订阅和发布包装后，既有输出逻辑、输入逻辑及节点接线/循环的 token 与基线一致。
  文件哈希核对：其他 PNC 业务代码、消息定义、gantry_detect 与 monitor 均未改变。

当前差分摘要为 `71b6c515ff344490c3e808f3236d3e59a23f384ed4e66ec28c364dc9d16b6744`。
与首轮不同是因为测试 trace 记录了临时 fixture 的绝对目录；归一化目录后，两轮 trace 也一致。
本机 Boost.Geometry 要求 C++14，桩编译沿用 C++14；生产 CMake 的 C++11 设置未变。

## 故障注入：测试能够检出所担心的问题

仅在 `/tmp` 隔离副本改源码、编译和执行，正式源码未应用这些变异。

| 人为错误 | 测试检出的首个错误 |
|---|---|
| 直接把 `mGantryStop` 赋给 safety，无效时清为 0 | 开闸时仍有其他安全原因，发布值与独立预期不符 |
| 发布后固定恢复为 0 | 关闸期间出现其他安全原因，发布后的内部值被错误清除 |
| 输入回调在无效时将内部 safety 清零 | 手动输出丢失原有 safety |

原始日志：`/tmp/planning_gantry_review_20260917_105121/`。
`verify/result.json` 是编译和回归结果；`scope_review.json` 是范围/token 复核；
`mutation_result.json` 记录三种故障注入的实际失败断言。
