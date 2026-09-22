# HDMap Server SDK

C++11 共享库，供 PNC 在进程内调用。核心算法不依赖 ROS、Eigen 或第三方样条库。
新增车道服务采用 PNC 当前的 4 空格、同行大括号、PascalCase 函数与中文业务注释风格。
地图初始化阶段使用 Boost.Polygon 头文件完成区域并集；运行时没有 Boost 动态库依赖。

## 最近变化与注意事项

09-21 配合 PNC/ultra_command 导航速度替换，核查本库无 CAN 实际车速入口。
`ClassifyPerception()` 仅从导航读取位置和 heading；样条 `Segment::Speed()` 是几何参数
导数模长，供弧长积分/反解使用；`pnc_adapter` 的路径点 velocity 是目标路径字段。
这些量不替换成车辆实际速度。本轮未修改 HDMap 算法、地图、接口或 SDK 库。

09-15更新为7条源地图、12,537个处理点，并将车道服务接入PNC感知发布；09-14首期10条地图
属于历史。地图预处理与规划实际加载链是两件事，当前接入的是感知车道分类。

## 注意事项与容易疏忽的点

- 0.5m是沿样条的弧长间距，不是弯道相邻点的直线距离；末尾不足一整步默认不输出。
- 输出8列表头CSV不能交给PNC旧三/四列fscanf读取器；使用LoadProcessedTrajectory。
- SDK方向1/-1不是PNC挡位枚举；倒车车头heading不能用点序切线直接替代。
- 当前SDK安装库是主机架构，Orin必须重编；`sdk/share/hdmap/README.md`是安装副本，由源README安装更新。
- 类实例一次LoadMap并复用；改CSV不自动热更新，不能每帧重新加载地图。
- 只返回type的分类接口不改变其他字段；converter另有本车道20%门槛，不等同于SDK通用分类。
- 地图原点、位置精度和车辆可行性需要外部保证，数值插值误差小不代表实车定位精度。

## 架构与职责

```text
hdmap/
├── include/hdmap/
│   ├── hdmap_server.h          # SDK 门面：文件、目录、内存轨迹处理
│   ├── hdmap_types.h           # 点、角度制、方向、选项、结果与错误码
│   ├── coordinate.h            # PNC heading 与数学 yaw 转换
│   ├── trajectory_processor.h  # 可单独调用的轨迹处理阶段
│   ├── lane_map_server.h       # 车道缓存、障碍物分类、robot 消息适配
│   ├── lane_types.h            # 车道宽度/索引参数与 type 中文枚举
│   └── pnc_adapter.h           # 转为 PNC XYZ_COOR_S，逐字段适配
├── src/
│   ├── hdmap_server.cpp        # 流程编排
│   ├── coordinate.cpp          # 角度归一化和转换
│   ├── trajectory_processor.cpp
│   ├── lane_map_server.cpp     # 地图加载、自车匹配、邻道排序、覆盖面积比较
│   ├── lane_geometry.h/.cpp    # 私有：区域并集、分块裁剪、包围盒索引、线段投影
│   ├── cubic_spline.h/.cpp     # 私有：三次样条、弧长积分与反解
│   └── trajectory_csv.h/.cpp   # 私有：CSV 校验、目录扫描、原子导出
├── tools/hdmap_process.cpp     # 批处理命令行，参数解析与结果打印
├── examples/pnc_load_path.cpp  # 使用真实 PNC 头文件的动态链接示例
├── examples/pnc_classify_perception.cpp # navigation_msg/perception 调用示例
├── docs/lane_occupancy.md      # 障碍物车道判断的契约、配置与校验说明
├── tests/                     # C++ 回归、SciPy 独立验证、真实地图验证报告
├── map/                       # 用户提供的原始轨迹
├── map_processed/             # 当前生成的 7 个 CSV（2026-09-15）
└── sdk/                       # 本机安装产物：include、lib、bin、CMake 配置
```

`HdMapServer` 无地图缓存或 ROS 通信成员，调用不会改变其他实例。
读取、坐标处理、几何算法与 PNC 适配独立；样条内部类型不导出到 `.so` 公共符号中。
`LaneMapServer` 单独管理只读车道缓存与空间索引，提供障碍物车道关系判断；
邻道关系根据局部中心线几何推断，当前 CSV 不包含道路拓扑或路由信息。

## 障碍物车道关系接口（2026-09-15 新增）

```cpp
#include <robot/navigation_msg.h>
#include <robot/perception.h>
#include "hdmap/lane_map_server.h"

hdmap::LaneMapServer lane_map; // 实际接入时作为业务类成员，跨帧复用。
hdmap::STATUS_S status = lane_map.LoadMap("/absolute/path/map_processed");
// 初始化须检查 status.IsOk()；成功后每帧调用下行，失败异常由调用方处理。
robot::perception result = lane_map.ClassifyPerception(navigation, perception);
```

默认车道宽度 4 米；按障碍物有向矩形与车道的重叠面积选择覆盖最多的车道。
返回消息仅修改 `objs[i].type`：**0 本车道、1 左一、2 左二、3 左侧车道外、4 右侧车道外**。
自车离开车道时只返回 3/4。接口接收 `xAxis/yAxis/zAxis/heading`，对齐现有
`perception_msg_convert.cpp` 的感知 heading 编码；输入 `objs.x/y` 已是地图坐标。

地图加载和查询分离，分块缓存与包围盒索引在 `.so` 内，模板仅适配 ROS 消息，
可以直接接收 `robot::navigation_msg`、`robot::perception` 并返回 `robot::perception`。
完整约定、配置、异常处理及性能数据见 [车道判断说明](docs/lane_occupancy.md)。

PNC 的 `perception_msg_convert.cpp` 已在 `perception_pub.publish(mPerception)` 前调用分类。
节点启动时加载 `hdmap` 包的 `map_processed`，可通过 `/hdmap/map_processed_dir` 覆盖目录；
地图加载失败退出，单帧分类失败记录错误并跳过该帧发布。`type` 的消息注释已同步为车道关系。
当前发布检查见 `src/pnc/src/robot_perception_convert/tests/verify_publish.py`；旧tests/perception为09-15历史接入夹具，其空帧期望已被后续需求替代。

## 坐标和数据契约

| 字段 | 定义 |
|---|---|
| `x_axis` / `y_axis` | 原地图平面坐标，米；沿用 PNC 定位坐标原点，不平移、不旋转、不重新投影 |
| `heading` | PNC 车头方位角：+y 为 0°、+x 为 90°、顺时针增加，范围 `[0,360)` |
| `curvature` | 曲率绝对值，单位 `1/m`，与 `ControlComply::CalcuPathCurve` 的输出约定一致 |
| `signed_curvature` | 沿轨迹点序左转为正、右转为负，单位 `1/m`，供需要方向信息的算法使用 |
| `dist_origin` | 样条从起点到当前点的累计真实弧长，米；每个 CSV 独立从 0 开始 |
| `p2pDistance` | 到前一个输出点的直线距离，首点为 0；对应 PNC 同名字段 |
| `direction` | SDK 自身的前进 `1` / 倒车 `-1` 标记，**不是 PNC 挡位枚举值** |

PNC 方位角与数学 yaw 的关系：`yaw = (90 - heading) * pi / 180`。
PNC 内部 `planning::PathPoint::theta` 使用数学弧度，应通过 `Coordinate::HeadingToYaw` 转换。
地图点使用 `double`；转换到 `XYZ_COOR_S` 时才使用 `float`，会检查溢出并处理航向舍入到 360° 的边界。

当前 7 个源文件（2026-09-15 更新）的 heading 与 PNC 方位角约定一致，所以使用 PNC 模式归一化。
可通过 `MATH_YAW_RAD` / `MATH_YAW_DEG` 明确选择其他输入角度制。
坐标原点必须由地图生产方与 PNC 定位共同保证，SDK 不从 x/y 数值猜测坐标系。

当前 7 条轨迹均识别为前进；SDK 也支持车头方向与点序切线相反的倒车轨迹。
倒车输出 heading 保留车头方向语义；不能只用 `atan2(dy, dx)` 当作车头方向。
带符号曲率仍按点序定义，倒车时不把它直接当作车身转向曲率。

## 轨迹处理流程

1. **读取**：每个文件是一条有序轨迹，输入为 `x,y,heading` 三列，支持无表头或准确的同名表头。
   支持 UTF-8 BOM、CRLF、空行、`#` 注释行；拒绝空字段、额外列、非数值、NaN/Inf 和数值后多余字符。
   错误带文件名和行号。目录处理仅扫描直属普通 `.csv`，按名称排序，忽略隐藏文件和软链接。
2. **航向转换与行驶方向判断**：先统一为 PNC heading，再根据车头与位移的夹角判断前进/倒车。
   累积到至少 1 cm 位移再判断，避免静止抖动。至少 90% 有效里程须在选定方向的 ±60° 内；
   无法确定或混合换挡的轨迹报错，需先按行驶方向分文件。显式指定方向也做一致性校验。
3. **至少 1 米采样**：保留首点，后续候选点到上一保留点的平面距离达到阈值才入选。
   保留原始终点；若最后一段不足阈值，则向前合并尾部节点，确保相邻拟合节点均不少于阈值。
   不足两个满足条件的节点时返回错误，重复点不会进入样条方程。
4. **拟合**：以节点间累计弦长为参数，分别拟合 x(u)、y(u) 的夹持三次样条。
   原始首末 heading 转为行进切线，作为边界一阶导；中间节点保持位置、一阶导、二阶导连续。
   检查段内零切线尖点；不存在曲率定义时返回错误，不强行输出零曲率。
5. **等弧长插值**：对 `sqrt(x'(u)^2 + y'(u)^2)` 自适应积分，得到真实样条弧长。
   用带二分保护的牛顿迭代反解 `s -> u`，按 `s=0,0.5,1.0,...` 采样。
   **相邻点的样条弧长是 0.5 米；弯道上的直线距离通常略小于 0.5 米。**
6. **计算属性**：从解析一阶导计算 heading，从一、二阶导计算
   `signed_curvature = (x'y'' - y'x'') / (x'^2+y'^2)^(3/2)`，绝对值写入 `curvature`。
7. **导出**：17 位有效数字保存到新的 CSV；临时文件写完后原子发布。
   已存在的目标文件不覆盖，源图不会被改写。

默认严格保持 0.5 米间距，不能整除的尾段不输出，长度见 `PROCESS_REPORT_S::remaining_length`。
例如 10.2 米轨迹输出 `s=0..10.0`，余下 0.2 米记录在报告中。
设置 `include_endpoint=true` 可追加 `s=10.2` 的终点，此时只有最后一段不足 0.5 米。
整步附近使用最多 `1e-8` 米的浮点容差，防止理论 10 米被算成 `9.999999999999998` 时丢掉终点。

输出格式固定为：

```csv
x,y,heading,curvature,signed_curvature,dist_origin,p2p_distance,direction
```

## 使用方法：编译、生成 `.so` 和批处理

在工程根执行：

```bash
cmake -S src/hdmap -B src/hdmap/build \
  -DCMAKE_BUILD_TYPE=Release -DHDMAP_WITH_CATKIN=OFF \
  -DCMAKE_INSTALL_PREFIX="$PWD/src/hdmap/sdk" -DCMAKE_INSTALL_LIBDIR=lib
cmake --build src/hdmap/build -j4
ctest --test-dir src/hdmap/build --output-on-failure
cmake --install src/hdmap/build
src/hdmap/sdk/bin/hdmap_process src/hdmap/map src/hdmap/map_processed_new
```

安装结果：`sdk/lib/libhdmap_server.so -> libhdmap_server.so.1 -> libhdmap_server.so.1.0.0`，
公共头文件在 `sdk/include/hdmap`，批处理程序在 `sdk/bin`。
安装后的 CLI 通过相对 RPATH 查找同一 SDK 的共享库，可整体移动 SDK 目录。
重复批处理请指定新的输出目录；错误退出码为 1，命令行参数用法错误为 2。
可运行 `hdmap_process --help` 查看角度制、行驶方向、间距和终点选项。

**本次生成的 `.so` 是本机 x86_64 / GCC 13 版本。Jetson Orin 是 aarch64，须在车载环境按上述命令重新构建，
或使用配套 aarch64 工具链与 sysroot 交叉编译。** C++ SDK 的消费者应使用兼容的编译器、libstdc++ 与 C++ ABI。
不应将 x86_64 库直接复制为车载运行库。

包内也提供 `package.xml`。在 ROS1 catkin 构建中会自动启用 `HDMAP_WITH_CATKIN` 并导出头文件和
`hdmap_server` 库，并安装 `map_processed` 的 CSV。PNC 已添加 `hdmap` 和 `roslib` 依赖，
`rebuild_all.sh` 按 `hdmap`、`robot` 的顺序构建。本机只有 ROS2 Jazzy，已验证独立 SDK 和
使用 ROS 桩的 PNC CMake 感知目标，未执行真实 ROS1 工作空间构建。

## PNC 调用

推荐在地图预处理阶段生成 CSV，PNC 装载任务轨迹时直接加载处理结果：

```cpp
#include <cstdint>
#include "common/struct_type.h"
#include "hdmap/hdmap_server.h"
#include "hdmap/pnc_adapter.h"

hdmap::HdMapServer server;
hdmap::MapPointList points;
hdmap::DIRECTION_E direction = hdmap::DIRECTION_AUTO;
hdmap::STATUS_S status = server.LoadProcessedTrajectory(csv_file, points, direction);
if(!status.IsOk()) {
    // 向任务装载层报告 status.message，当前地图不得切换为失败结果。
    return;
}
std::vector<XYZ_COOR_S> path_list;
status = hdmap::ConvertToPncPath(points, path_list);
if(!status.IsOk()) return;
// path_list 交给路径管理层；实际挡位、任务速度仍由 PNC 任务配置赋值。
```

适配器将 `z_axis`、`velocity` 初始化为 0，分别由业务层维护换道标记和速度。
SDK 的曲率不会占用 `z_axis`；结构体通过逐字段赋值转换，不依赖二进制内存布局。
`direction` 仅供核对任务挡位，不能直接强转为 PNC 的 `GEAR_D` / `GEAR_R`。

**SDK 输出带表头和 8 列，必须使用 `LoadProcessedTrajectory`，不能直接送入原 PNC 的三/四列
`fscanf` 路径读取器。** 原控制侧 `CalcuPathCurve` 还存在按固定 0.1 米累加的历史逻辑，
正式接入应使用 SDK 曲率及实际里程，核对所有依赖点数/固定点距的业务窗口。
本次交付 SDK 和真实类型链接示例，现役 PNC 的加载链、速度规则、任务配置未切换。

独立 CMake 消费者：

```cmake
find_package(hdmap CONFIG REQUIRED)
target_link_libraries(your_pnc_target PRIVATE hdmap::hdmap_server)
```

配置时增加 `-DCMAKE_PREFIX_PATH=/path/to/hdmap/sdk`。
ROS1 catkin 消费者可在现有 `find_package(catkin REQUIRED COMPONENTS ...)` 中加入 `hdmap`，
在 `package.xml` 中加入 `<depend>hdmap</depend>`，并链接 `${catkin_LIBRARIES}`。
示例运行：

```bash
src/hdmap/build/hdmap_pnc_example src/hdmap/map_processed/lane1_spline.csv
```

需要按内存点串处理时调用 `ProcessTrajectory`；文件处理用 `ProcessFile` / `ProcessDirectory`。
需要单独使用阶段时调用 `TrajectoryProcessor` 的 `NormalizeHeadings`、`ResolveDirection`、
`SampleAnchorPoints`、`FitAndResample`；后三者接收已经转换为 PNC heading 的点。
这些阶段均允许输入输出使用同一容器。

所有可预期失败通过 `STATUS_S` 返回；读取/处理失败保持调用方输出和报告原值。
批处理返回逐文件结果，单个文件失败后继续其他文件，整体返回失败，成功文件仍保留。
批次启动前失败时不替换旧的结果数组，调用方必须先检查返回状态。

## 验证与已生成结果

2026-09-15 已对更新后的 7 条地图重新调用 SDK：25,277 个原始点生成 12,537 个输出点。
参数为 `--heading pnc --direction auto --anchor-interval 1 --sample-interval 0.5`，
结果位于 `map_processed/`；保持严格 0.5 米等弧长，末尾不足 0.5 米的部分不另补点。
独立 SciPy 全点校验通过：最大相邻弧长误差 `9.913e-9 m`，最大坐标差 `9.916e-9 m`，
heading 差 `1.156e-7°`，曲率差 `2.451e-9 /m`。全部源 CSV 的 SHA-256 在处理前后一致。
`tests/map_verification.json` 已更新为本批地图的验证报告。

| 当前轨迹 | 原始点数 | 拟合节点数 | 输出点数 | 方向 |
|---|---:|---:|---:|---|
| lane1 | 772 | 158 | 354 | 前进 |
| lane2 | 704 | 156 | 341 | 前进 |
| lane3 | 2108 | 472 | 1052 | 前进 |
| lane4 | 1145 | 257 | 555 | 前进 |
| lane_MID | 7166 | 1508 | 3413 | 前进 |
| lane_N2S | 7139 | 1509 | 3412 | 前进 |
| lane_S2N | 6243 | 1500 | 3410 | 前进 |

独立验证脚本仅测试环境需要 NumPy / SciPy：

```bash
python3 src/hdmap/tests/verify_maps.py src/hdmap/map src/hdmap/map_processed \
  --report src/hdmap/tests/map_verification.json
```

上述误差是数值实现交叉验证结果，不代表原始定位数据的物理精度。
三次样条穿过采样节点，没有执行定位去噪、障碍物约束或车辆最小转弯半径约束；
实车使用前仍需在目标环境完成 PNC 接入及道路验证。

09-14首期10条地图/956项及安装后异地链接验证属于历史；详细数值见
[整理前原文](../../docs/history/2026-09-19-before-docs/src/hdmap/README.md)，不与当前7条结果混用。
