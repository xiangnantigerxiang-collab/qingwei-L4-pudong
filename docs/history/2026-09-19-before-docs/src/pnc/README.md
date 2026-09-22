# pnc 模块编码风格与习惯规范

> 本文件由 2026-08-30 对 pnc 全部源码的风格采样提炼而成（含 6 节点目录 + 公共算法库，
> 采样覆盖命名/结构/ROS 惯例/算法实现/注释文化五个维度）。
> **用途：任何人（或 Claude）在本模块新增/修改代码前必读，新增代码风格必须与所在文件的上下文一致。**
> 标注【历史例外】的项是既有不一致，**保持现状、不要顺手改**——本模块的纪律是改动最小化。

---

## 一、架构模式（必须遵守的骨架）

每个 ROS 节点一个目录，固定三层结构：

```
robot_<模块>/
├── <模块>_node.cpp     # ROS 薄壳：main + 回调 + 订阅/发布，不含业务逻辑
├── <模块>_comply.h/.cpp # 业务编排类 XxxComply：Set* 接收 + *Process 周期逻辑 + Publish*
└── （专用算法类独立文件，文件名按功能命名）
```

- **node 三段式**：全局 comply 实例 → 一组极薄回调（1-3 行：置标志/转格式 + 调 `comply.Set*()`）→
  `main()`：ros::init → subscribe → advertise → `while(ros::ok()){ros::spinOnce(); comply.*Process(); pub.publish(...); loop_rate.sleep();}`
- **comply 类不持有 ROS 通信成员**（无 Subscriber/Publisher/NodeHandle）；发布时由调用方把
  `ros::Publisher` 作参数传入（如 `PublishMessage(ros::Publisher& tPub)`）
- **CMake 里 comply 编成库、node 是可执行**，二者分开（`add_library(xxx_comply)` + `add_executable(xxx_node)`）
- **`robot_path_plan/` 的 comply 实现按职责拆成 `.inc`**：入口仍只有
  `path_plan_comply.cpp`，先包含几何辅助，再按固定顺序原位包含任务、感知、参考路径、
  最终输出和实验算法五个业务分片。**2026-09-12** 按职责进一步分为
  `task/`、`reference/`、`safety/` 子目录，由对应业务分片继续包含；核心入口按阶段函数
  编排，状态仍集中在 Comply。`.inc` 不得单独加入 CMake，否则会改变同一翻译单元语义并产生重复定义。
- **【2026-08-30 起第二种许可骨架】`robot_task_plan/` 已改为 canbus 同款 core/事件流结构**：
  `<模块>_core.{h,cpp}`（零 ROS 业务库：输入镜像结构体 + `Set*`（输入镜像写入，可含轻逻辑，
  如 SetCanData 的自动模式重武装、SetTaskInfo 的任务装载）/`On*`（同步发事件）
  入口 + `TaskPlanSink` 有序事件流回传 发布/rosparam 写/ROS_INFO）+ `<模块>_node.cpp`
  （薄适配：话题接线、参数读取、事件→ROS 调用逐字段转换）。业务全在 core，node 不留逻辑。
  其余节点目录为传统结构（path_plan/control/can_comm 为 comply 三层；
  navigation/perception_convert 本就是单文件节点）；新增节点二选一，同一目录内不混用两代骨架。
- 心跳/定时用 `nh.createTimer(ros::Duration(0.1), T1Callback)`，回调里直接操作全局 `*_pub` 发布

## 二、命名规范（实测主导约定）

| 对象 | 约定 | 示例 |
|---|---|---|
| 类名 | PascalCase，业务编排类带 `Comply` 后缀 | `ControlComply`、`SpeedControl`、`PubAlgor` |
| 方法名 | PascalCase 动词开头，按职责分动词族 | 消息入口 `Set*`；周期 `*Process`；发布 `Publish*`；初始化 `InitParameter`；谓词 `Is*/Judge*` |
| 成员变量 | `m` 前缀 + PascalCase（【历史例外】公有状态成员部分无前缀，不改） | `mPathList`、`mNavData`、`mControlData` |
| 消息型参数 | `t` 前缀 PascalCase 或 snake_case+`_t` 后缀（两种并存） | `tDesireSpeed`、`can_msg_t` |
| 局部变量 | snake_case；中间量加 `_temp` 后缀 | `xyz_temp`、`rtn_value`、`sensorstate` |
| 回调函数 | `<消息名>CallBack`（**大写 B** 为主导；【历史例外】T1Callback/GNSSCallback/AirPortMsgCallback/PalletCoorCallback/pathStatusCallback/TlStatusCoorCallback 等约 6 处小写 b，不改） | `CanMsgCallBack` |
| 发布器/订阅器 | snake_case + `_pub`/`_sub` 后缀 | `control_pub`、`task_plan_sub` |
| 接收标志 | `rcv_*` 前缀 | `rcv_can_data` |
| 文件名 | 全小写下划线；节点入口固定 `<模块>_node.cpp` | `longitudinal_speed_control.cpp` |
| 头文件守卫 | `#ifndef <文件>_H` 无目录前缀为主；robot_control/ 已统一 `ROBOT_CONTROL_*_H_` 带前缀（防同名头碰撞） | `ROBOT_CONTROL_CONTROL_COMPLY_H_`、`TASK_PLAN_CORE_H` |
| C 风格结构体 | `typedef struct 小写tag {...} 全大写_S;`，成员 t 前缀（task_plan_core 08-30 起为 `struct TASKINFO_S {...};` 直写） | `TASKINFO_S`、`XYZ_COOR_S`、`CONTROL_PARAM_IN` |
| 枚举常量 | 全大写连写 | `GEAR_N`、`TASKFINISHED`、`ADAPTIVEHOOK` |

## 三、代码格式（2026-09-14 按用户要求统一）

- **自有 C/C++ 统一 4 空格缩进、左大括号同行、右大括号单独收尾**，
  `else` / `catch` 使用 `} else {` / `} catch(...) {`。条件关键字与括号之间不加空格：

  ```cpp
  if(mReferPath.x.size() < 5) {
      ROS_ERROR("publish plan path size less 5.");
      PublishStoppedPath(tPub1);
      return;
  }
  ```

- 函数、类、结构体、命名空间、循环和条件块均采用上述同行大括号；
  独立局部作用域和多维数组的元素初始化列表保留其语法对应的布局。
- 统一范围为 `src/`、`include/common/`、`tests/` 中的 C/C++ 及 `.inc` 文件；
  `3rd-party/`、`include/Eigen/`、`include/unsupported/` 的第三方内容保持原样。
- 根配置为 [`.clang-format`](.clang-format)，`robot_control/.clang-format` 继承根配置。
  **include 顺序有隐式依赖，禁止排序/分组；也不排序 using 声明，不增删大括号。**
- 推荐使用 [`tools/format_cpp.py`](tools/format_cpp.py)，它调用 clang-format 18，
  将条件末尾注释前的大括号一并归位，并检查有效 token、预处理指令与 include 顺序不变。
  保留原有续行符和模板 `> >` 的 token 边界；字符串和注释中的伪代码不作业务代码处理：

  ```bash
  python3 src/pnc/tools/format_cpp.py --clang-format /path/to/clang-format
  python3 src/pnc/tools/format_cpp.py --clang-format /path/to/clang-format --check
  ```

- 该规则替代此前 control/task_plan 的 2 空格 Google 与其他目录 4 空格 Allman 的混合格式。
- C++11；克制使用现代特性（手写索引 `for (int i...)` 远多于 range-for，243:44；
  需要索引/相邻点配对/步进采样时必须用索引循环）
- 【2026-08-30 晚更新】`robot_task_plan/task_plan_core` 与 canbus 包的 `canbus_core`
  已升级为**惯用 C++**（`std::function` 事件出口、`std::string` 事件字段、NSDMI、
  `struct X_S {}` 直写），两模块成员统一 m 前缀 PascalCase（canbus 原 Google 尾下划线
  已全改名）；这两个文件内新代码按此风格，其余节点目录仍守上一条

## 四、ROS 使用惯例

- **话题一律绝对名**（`/` 前缀，snake_case，扁平或 `域/名` 两层；全包 60 处已统一，勿再引入相对名）
- **单线程模型**：全部节点 `ros::spinOnce()`，全库无 AsyncSpinner/std::thread（不要引入）
- **频率分档**：path_plan 10Hz ｜ control/task_plan/can_comm 20Hz ｜ navigation 50Hz ｜
  perception_msg_convert 100Hz；主循环尾部必须 `loop_rate.sleep()`（【历史例外】can_comm_node 漏了）
- **订阅全部带 `ros::TransportHints().tcpNoDelay()`**；队列深度语义：指令/状态类 1、数据流 10、发布一律 10
- **回调签名**：robot:: 消息用 `const xxx::ConstPtr& msg`（转发时解引用 `*msg`）；
  大包/外来消息（MarkerArray/BoundingBoxArray/ivlocmsg）用 `const T& msg`
- **参数热读**：`ros::param::get` 放主循环内每圈重读，返回值不检查、目标变量预初始化
  （参数级零阶保持）；运行时状态键带 `/`（`/robot/xxx`、`/canbus/xxx`、`/planning/xxx`），
  launch 配置键裸名（`path_dir`、`task_file`）——两种并存是惯例
- **rosparam 当低频指令通道**（`/canbus/brake`、`/sound/play`、`alarmcmd`）——历史架构，勿改成话题
- **ZOH 重发**是常态：navigation 每圈无条件重发、path_plan 退化路径也照样 publish（清空+0 速）

## 五、算法实现习惯

- **跨帧状态**用函数内 `static` 局部变量（去抖计数、`static std::vector` 环形历史缓冲
  `erase(begin())+push_back`、状态锁）；一半用成员变量接力（`mKeypoint` 滑窗推进）——两种并存
- **标量几何**为主：裸 `double` + `hypot/atan2/cos/sin`；角度用方位角制（0=北、顺时针），
  与弧度互转的 `90.0 - heading`、`* M_PI / 180.0` 内联写法（Eigen 已引入但算法几乎不用）
- **魔法数直接内联**，重要处加中文尾注释说明量纲：
  `double base_to_front = 2.3; // 车辆后轴中心到前雷达的距离`；
  场地绝对坐标（如停靠点）直接写源码——已知债务，新增时同样处理但必须注释
- 哨兵值惯例：`-1` 表示无停止点、`100 + n`/`200 + n` 编码信息段位、`1e6/10000.0` 当初始最小距离

## 六、日志与注释文化

- **printf 主导**（全库 239 处 vs ROS_INFO 19 处）：带 `\n`、中文字符串、`====横幅====`、
  手写编号（`3333mKeyPoint`）调试噪声留在生产循环——**新增调试输出跟随所在文件用 printf**
- **中文注释解释"为什么"**（复用上一帧路径的理由、去抖 4 帧的含义），英文做小节标签
  （`// generate connect path`、`// smooth`）
- **自我声明式注释**是本模块特色：`// 下面的desiredSpeed没有用到`、`// 后面的代码有bug`
  ——发现死代码/已知缺陷时用注释如实标注，而不是删除或隐瞒
- **死代码整段注释保留**（而非删除）是该仓库历史习惯；08-30 起改为"经用户确认后删除"，
  未确认的保持注释原样
- 历史署名注释保留（`// zhangyu 20220213`），不删
- 斜线分节横幅：`//////////////////计算一个限速/////////////////`

## 七、改动纪律（历次整理形成的铁律）

1. **清理/格式化绝不改业务逻辑**——每次删除用 token 级等价校验（去注释去空白后逐 token 比对）
2. **不重命名任何既有标识符**，包括拼写错误（`GeometricConstrol`、`CallBack`、`recived_cloud_task`）
3. **不重排 include**（隐式依赖）；include 路径分层：本目录裸名 / 跨模块 `robot_path_plan/common/...`
   前缀 / 消息一律 `robot/<msg>.h` / 系统三方尖括号
4. **mControlData 等发布消息的字段赋值不删**（改变发布内容=行为变化）
5. 消息桩必须从真实 `.msg` 生成，禁止手写（08-28 教训）
6. auto_couple 等 GBK/ISO-8859 文件 grep 必须 `-a`，否则静默漏检
7. 修改后必须过编译验证：本机桩编译流程见 workflow.md（08-30 日志）

## 八、感知车道分类接入（2026-09-15）

`perception_msg_convert.cpp` 在坐标转换、排除区域过滤完成后，发布前调用
`mLaneMap.ClassifyPerception(mGPS, mPerception)`。转换与分类使用同一定位，
返回消息的 `objs[i].type` 为 0 本车道、1 左一、2 左二、3 左侧车道外、4 右侧车道外；
`object.msg` 已添加对应中文注释，消息字段不变。

节点启动时只加载一次 `hdmap` 包内 `map_processed`，可用全局参数
`/hdmap/map_processed_dir` 覆盖。地图加载失败报错退出，单帧分类失败报错并跳过该帧发布。
依赖 `hdmap`、`roslib`；`rebuild_all.sh` 已安排先编译 `hdmap` 再编译 `robot`。

本机运行 `python3 src/pnc/tests/perception/verify.py --output /tmp/pnc_hdmap_publish_verify`：
使用真实 `.msg` 生成的桩和真实 SDK，验证 C++11 感知目标、两种包导出配置的动态链接及
71 项实际回调检查。目标车载仍需执行 ROS1 catkin 构建与运行验证。
接口细节见 [HDMap 车道判断说明](../hdmap/docs/lane_occupancy.md)。

---

*采样基准：2026-08-30 代码状态；代码格式于 2026-09-14 按用户明确要求统一。
风格问题以本文件为准；本文件与代码冲突时，以【实际所在文件的上下文】为最终裁决。*
