# 障碍物车道占据判断

`LaneMapServer` 加载 `map_processed` 中的全部普通 CSV，每个文件代表一条车道中心线。
初始化一次后，每帧调用 `ClassifyPerception(navigation, perception)`，返回带车道分类的
`robot::perception`。实际路径由调用方传入，不依赖当前工作目录或写死的车载安装路径。

## 输入与输出契约

- 定位类型为 `robot::navigation_msg`。`xg/yg/zg` 在该消息中的字段名是
  `xAxis/yAxis/zAxis`，对应地图平面位置及高度；`heading` 是正北 0°、顺时针增加的方位角。
  当前车道中心线无高程，分类按二维地面投影计算。
- 感知类型为 `robot::perception`。`objs[i].x/y` 是障碍框的地图中心坐标，单位米；
  `dx/dy` 是框局部两轴的完整尺寸，构造角点时各取一半，不再对中心进行车体坐标转换。
- 根据 `perception_msg_convert.cpp` 的实际赋值，感知角度为
  `Hobj = (yaw_local_deg + Hego + 90) mod 360`。框的地图数学角为
  `yaw_world = (Hobj - 2*Hego) * pi/180`，东为 0、逆时针为正。
  这与地图中心线的 PNC heading 不同。适配层已处理，不要求 PNC 调用者再换算。
  **传入定位航向应与该帧感知转换时使用的航向一致**；SDK 不猜测两帧之间的车辆转动。
- 输出保持 header、数量、顺序、id、位置、尺寸、朝向、速度、高度和 polygons，
  仅修改返回副本的 `objs[i].type`。输入消息保持原值。

`type` 在 SDK 的 `lane_types.h` 中定义并逐项添加中文注释，保持 `uint8` 兼容：

| 数值 | 枚举 | 返回语义 |
|---:|---|---|
| 0 | `LANE_CURRENT` | 覆盖面积最大的候选车道为本车道 |
| 1 | `LANE_LEFT_ONE` | 覆盖面积最大的候选车道为左侧第 1 车道 |
| 2 | `LANE_LEFT_TWO` | 覆盖面积最大的候选车道为左侧第 2 车道 |
| 3 | `LANE_OUTSIDE_LEFT` | 左侧车道外；自车离道时表示障碍物在自车左侧 |
| 4 | `LANE_OUTSIDE_RIGHT` | 右侧车道外；自车离道时表示障碍物在自车右侧 |

**上述返回值属于车道关系编码**，使用此返回消息的调用者应按上表解释。
`robot/object.msg` 的 `type` 注释已同步为车道关系；字段类型与顺序保持不变。
PNC 的 `perception_msg_convert.cpp` 已在 `/perception` 发布前调用本接口。

## PNC 发布接入

`perception_msg_convert` 启动时通过 `ros::package::getPath("hdmap")` 定位包内
`map_processed`，仅加载一次地图和空间索引。可用 ROS 全局参数
`/hdmap/map_processed_dir` 指定其他地图目录。catkin install 会把处理后的 CSV
安装到 `share/hdmap/map_processed`，支持源码工作空间和 install 空间的默认路径。

在原有 CUBE 过滤、坐标转换和排除区域过滤之后、`perception_pub.publish(mPerception)`
之前执行 `mPerception = mLaneMap.ClassifyPerception(mGPS, mPerception)`。
坐标转换与分类共用该回调内的 `mGPS`，符合框朝向适配要求；当前 `spinOnce` 为单线程调度。
地图加载失败会打印目录与错误原因并退出节点；单帧分类失败会打印错误并跳过该帧发布。

PNC 增加 `hdmap` 与 `roslib` 包依赖，只给感知转换目标链接地图库。
根目录 `rebuild_all.sh` 已把 `hdmap` 放在 `robot` 之前构建；也可先执行
`catkin_make --pkg hdmap`，再构建 `robot`。本机验证命令：

```bash
python3 src/pnc/tests/perception/verify.py --output /tmp/pnc_hdmap_publish_verify
```

该测试从真实 `.msg` 生成 ROS 桩，编译真实 PNC CMake 感知目标并链接实际 `.so`，
覆盖 SDK 导出目标及 catkin 传统变量两种配置。71 项回调检查验证五类结果、最大覆盖、
反向行驶、自车离道、框朝向还原、原有过滤、发布前分类、异常帧和地图缓存。
本机没有 ROS1；这些检查不替代目标车载 catkin 构建与 ROS 通信验证。

## 判断规则

1. 中心线左右各扩展 2 米，默认总宽 4 米。折点斜接，锐角斜接长度超过半宽两倍时改斜切；
   开放中心线的首尾平截，不向端点之外无限延伸。闭合中心线的首尾也连接。
2. 自车定位点位于车道区域内则认为在车道上；重叠区域选最近中心线，距离相同时按
   朝向平行程度、文件名排序确定结果。车辆尺寸不在输入中，因此不判断整个自车车身占据。
3. 按“自车在道路上必处于最右侧车道”的业务前提，只建立本车道及其左侧两个邻道候选。
   每个障碍物先投影到本车道，以该位置沿自车行驶方向的局部横断面确定左右及邻道次序，
   因此能沿弯道判断，不按文件名或全局 x/y 排序。平行的对向车道也计入左侧序号；
   与当前道路横穿的车道不当作邻道。无道路拓扑信息时，邻接是几何估计。
4. 将障碍物视为有向地面矩形，计算其与三条候选车道的交面积，选择面积最大者。
   同一障碍物的总面积固定，比较面积等价于比较覆盖比例。弯道分段重叠只计一次。
   面积相等时选最近中心线，再按本车道、左一、左二稳定决胜；只有边界接触、面积为零时不算占据。
5. 没有正面积车道覆盖时，用自车朝向建立的横向坐标判断左右，左正右负。
   自车不在任何车道时不作邻道分类，即使障碍物在别的地图车道内，也只返回 3/4。
   车道外矩形跨自车前后轴线时按中心侧别，中心横向距离恰为零时统一返回 3。

## 配置与异常处理

```cpp
hdmap::LANE_OPTIONS_S options;
options.lane_width = 4.0;
hdmap::STATUS_S status = lane_map.LoadMap(map_directory, options);
if(!status.IsOk()) {
    // 向调用方报告 status.message；首次加载失败时不要进入正常分类流程。
    return;
}

try {
    robot::perception result = lane_map.ClassifyPerception(navigation, perception);
    // 将 result 交给需要车道关系的后续业务。
} catch(const std::runtime_error& error) {
    // 本帧失败，按调用方既有策略报告/处理，不能把原物体类别当成车道 type 使用。
}
```

`LANE_OPTIONS_S` 还提供 `grid_size=8m`、`neighbor_distance=12m`、`parallel_angle_deg=45°`。
前者只调内存与查询效率，后两者用于局部邻道几何筛选。宽度暂统一估算，不从 CSV 猜测可变宽度。

未加载地图、非有限定位、非有限框朝向、非正尺寸等返回明确失败；空感知返回空消息。
`ClassifyBoxes` 是不依赖 ROS 的状态码接口，提供所选车道文件下标和交面积供调试，失败时保持输出原值。
`ClassifyPerception` 将失败转换为异常以满足直接返回消息的接口形式。
地图重载只在全部读取、几何和索引完成后换入，失败保留旧地图。重载与查询按 PNC 当前单线程方式调度。

## 缓存与验证

加载阶段使用 Boost.Polygon 合并整条车道的区域，保留回环内洞，再裁成互不重叠的地图块。
区域并集使用 0.01 毫米定点网格，相邻分段共用接缝端点；运行时使用 double 计算交面积。
车道块和中心线段均有静态包围盒层次索引，另为每条车道缓存最近线段索引。
每帧仅查询相关块、裁剪局部多边形；没有目录扫描或 CSV I/O。索引以包围盒距离下界剪枝，
最近点最终在线段上投影，算法不以最近采样点代替投影点。

编译需要 C++11 和 Boost 头文件，`.so` 只依赖标准 C/C++ 运行库，不依赖 ROS 或 GEOS。
测试用的 robot 头可以由真实 catkin 生成目录提供；本机使用仓库生成器从真实 `.msg` 生成消息桩：

```bash
python3 src/pnc/tests/planning/generate_stubs.py src/pnc /tmp/hdmap_lane_message_stubs
cmake -S src/hdmap -B src/hdmap/build -DCMAKE_BUILD_TYPE=Release \
  -DHDMAP_ROBOT_INCLUDE_DIRS=/tmp/hdmap_lane_message_stubs
cmake --build src/hdmap/build -j2
ctest --test-dir src/hdmap/build --output-on-failure
python3 src/hdmap/tests/verify_lanes.py src/hdmap/build/hdmap_lane_probe \
  src/hdmap/map_processed --report src/hdmap/tests/lane_verification.json
src/hdmap/build/hdmap_perception_benchmark src/hdmap/map_processed
```

真实 ROS 环境可通过 `HDMAP_ROBOT_INCLUDE_DIRS` 传入生成消息及 ROS include 目录，
通过 `HDMAP_ROBOT_LIBRARIES` 提供测试需要的 ROS 库（如 `rostime`）。
纯 SDK 编译时两个选项都可省略；不依赖测试桩或 PNC 消息生成顺序。

2026-09-15 验证：原轨迹处理 956 项断言、车道 154 项检查、600 个真实消息结构的适配案例、
7 条地图全部 12,537 个中心线位置检查通过。独立 GEOS 以完整车道 buffer/intersection 对照
SDK 的分块面积，NumPy 全扫描对照空间索引，共 5,362 个场景的类别和车道一致，最大面积差
`6.173e-5 m²`；其中包含交叉道路、弯道、自交回环和实际地图，另检查 4/8/16 米分块结果一致。
验证报告见仓库 `tests/lane_verification.json`，基准测试结果见 `tests/lane_benchmark.txt`。
数值验证不代表 4 米估计宽度与真实道路边界一致；本机耗时也不代表 Orin 的耗时。

本机 x86_64 / Intel Core Ultra 5 338H / GCC 13.3 Release，完整 `ClassifyPerception`
包含消息适配和返回副本，预热 20 帧后各计时 300 帧：

| 每帧障碍物数 | P50 | P99 | 最大值 |
|---:|---:|---:|---:|
| 1 | 0.0024 ms | 0.0053 ms | 0.0061 ms |
| 10 | 0.0215 ms | 0.0313 ms | 0.0314 ms |
| 100 | 0.1849 ms | 0.2375 ms | 0.2479 ms |
| 300 | 0.5254 ms | 0.6408 ms | 0.6523 ms |

7 条地图一次加载约 289 ms，缓存 1,378 个车道块；这些时间不包含 ROS 通信。
AddressSanitizer、UndefinedBehaviorSanitizer 和 cppcheck 定向检查通过。
交付 `sdk/lib/libhdmap_server.so` 是本机 x86_64 产物，Orin 上按 README 的 CMake 命令重编。
