# 感知转换与时序滤波

节点已接入 `HDMap 分类 → 两秒原始帧历史 → 时序滤波 → 置信度与尺寸筛选 → publish`。
`/perception` 发布的对象包含计算后的 `confidence`，所有实现、测试、说明均位于本目录。

## 发布链与历史缓存

`BoxMsgCallBack` 开始时记录接收时间并清理历史，要求已收到定位、定位接收时间距本帧
小于两秒且 `xAxis/yAxis/zAxis/heading` 有限。完成原有坐标转换、排除区域过滤及
HDMap 分类后，把带有接收时间 `header.stamp` 的原始分类帧保存到 `mPerceptionHistory`。
随后滤波，并在发布前校验时间及输出是否来自当前帧：

```cpp
robot::perception filtered_perception = FilterPerceptionHistory(mPerceptionHistory);
// 仅在发布前有效性校验通过、历史提交成功后筛选，允许筛选后的结果为空。
// 需包含 <limits>；尺寸参数按宽度 [下限, 上限]、长度 [下限, 上限] 排列（米）。
const double max_size = std::numeric_limits<double>::infinity();
mPerception = FilterPerceptionByConfidence(filtered_perception, 0.25, 0.0, max_size, 0.0, max_size);
perception_pub.publish(mPerception);
```

缓存只存 HDMap 分类后的原始帧，不回存滤波输出。空 MarkerArray、只有文字标记的帧，
以及障碍物全被排除区域过滤的帧，也会作为空观测进入历史，参与置信度分母。
时序窗口中的历史目标在最后一次真实观测达到两秒后退出；发布阶段还会剔除
`confidence < 0.25` 的目标，相等时保留。尺寸阈值由调用方传入，目前发布调用的
宽、长区间均为 `[0.0, +∞]`，不设额外尺寸限制；阈值筛选不改原始历史。

缓存按时间递增，回调开始和现有 100Hz 主循环均调用 `MaintainPerceptionHistory`，
移除距今 `>=2s` 的前缀。无输入或持续坏帧也会过期；删除任何历史后都会清除基于旧窗口
计算的本地 `mPerception` 缓存，等待下一真实有效帧重算。相同时间戳只保留最后一帧，
暂停的仿真时钟不会令历史增长；时间非法或回退时清空历史和计算结果，重新建立窗口。

无输入、定位缺失/过期或本帧处理失败时不发布消息；过期维护只清本地状态。
不能用新时间戳持续发布空帧来代替断流，因为 PNC 和 ultra_command 按消息接收时间
判断感知超时，这种空心跳会掩盖上游失效。真实有效的空观测仍正常入窗并发布滤波结果。

坐标转换与滤波均使用局部工作变量，失败不会把半转换数据覆盖到最近的发布结果。
HDMap 分类失败时跳过该帧；时序滤波失败时撤销本次缓存追加并跳过发布；
同时间戳旧帧仅在滤波成功后替换，下一帧仍可使用此前有效历史。若处理期间发生
时间回退、当前帧或定位达到两秒，或滤波返回的 header 并非当前帧，则清空缓存并停止本帧发布。
ROS 时间为零时还检查非空原始帧不能得到默认空结果，避免默认 header 的零时间戳绕过校验。
上述时间窗口均使用 ROS 时间；仿真时钟暂停时，不按墙上时间判定两秒过期。

## 调用接口

```cpp
#include "perception_temporal_filter.h"

std::vector<std::pair<robot::perception, double>> history;
// 调用方保存每帧 HDMap 分类结果，时间与 ros::Time::now() 使用同一时钟。
history.push_back(std::make_pair(classified_perception, ros::Time::now().toSec()));
robot::perception filtered_perception = FilterPerceptionHistory(history);
```

输入通过 const 引用读取，函数不修改调用方的历史数据，不保存跨调用的隐式状态。
调用方负责管理历史容器大小。历史中应保存 HDMap 分类后的原始观测，包括空感知帧；
滤波输出不应再作为新观测放回历史，否则会重复统计已经累积过的结果。

`perception_temporal_filter.h` 提供声明，`.inc` 在现有 `perception_msg_convert.cpp` 中
包含一次并编译。这样无需修改目录外的 CMake，也不需要给消息新增字段：
当前 `robot/object.msg` 已有 `float32 confidence`。

## 置信度与尺寸过滤

```cpp
#include "perception_confidence_filter.h"

// 示例：保留 confidence >= 0.25、横向宽 dx 在 [0.5, 3.0] 米、纵向长 dy 在 [1.0, 12.0] 米的对象。
robot::perception filtered_perception = FilterPerceptionByConfidence(perception, 0.25, 0.5, 3.0, 1.0, 12.0);
```

接口为 `robot::perception FilterPerceptionByConfidence(const robot::perception& tPerception,
double tConfidenceThreshold, double tMinWidth, double tMaxWidth, double tMinLength, double tMaxLength)`：
置信度达到阈值，且 `dx`、`dy` 分别位于宽度、长度闭区间内时保留，任一不满足就剔除。
等于上下边界时保留；上下限相等时，只保留恰等于该正尺寸的对象。
`dx` 按用户定义为横向宽，`dy` 为纵向长，单位米；直接比较尺寸字段，不根据 heading 交换长宽或换算包围框。

保留兼容重载：两参数调用仍只设置信度阈值；原四参数调用的后两项仍为最小宽度、最小长度，
两项上限均不限制。设置区间时，完整传入四项尺寸参数，按“宽下限、宽上限、长下限、长上限”排列。
输入通过 const 引用读取；输出保留消息头、对象顺序和对象所有原有字段，
不重新归一化置信度，不更改历史缓存。空输入或全部被筛除时，仍返回原消息头。

置信度阈值使用传入的有限 double 原值，不做截断、舍入或容差比较；对正常 `[0,1]` 置信度，
阈值为 0 时全部保留，为 1 时只保留值为 1 的对象。NaN/无穷置信度会被剔除；
NaN/无穷阈值抛出 `std::invalid_argument`，避免配置错误被静默转换成空感知。
尺寸下限须为有限非负数，上限不得小于下限，不允许 NaN；上限允许正无穷表示不设上限。
非法区间抛出 `std::invalid_argument`，不自动交换上下界；该检查在处理对象之前执行，空输入同样校验。
对象 `dx/dy` 为非有限值或非正数时直接剔除，即使尺寸下限为 0 也不保留非法尺寸。

实现位于 `perception_confidence_filter.inc`，由现有节点包含一次编译；声明也通过
`perception_msg_convert.h` 引入。筛选遍历一次对象数组，并预分配输出容量，
仅复制被保留的对象。比较耗时为 `O(N)`，另有保留对象（含 polygons）的复制开销。

`BoxMsgCallBack` 已在时序滤波和现有发布前有效性校验通过后调用该函数，阈值固定为
`0.25`，宽、长区间均为 `[0.0, +∞]`，之后立即发布。需要设置尺寸时，修改调用的第三至第六个参数。
全部被阈值筛除属于有效空结果，不会被误判为时钟异常而跳过发布。
历史缓存继续保存未经阈值筛除的原始分类帧，后续观测仍能与这些目标关联并累积置信度。

独立函数阶段已通过 23 项阈值、浮点边界、空输入、字段保留和异常数值检查，以及当时的
2,976 项发布回归；C++11 节点编译、真实 SDK 动态链接、ASan/UBSan 和 cppcheck 通过。
测试使用真实消息定义生成的 ROS 桩。运行本目录的 `tests/verify_publish.py` 可复验，
结果见 [confidence_verification_20260915.json](tests/confidence_verification_20260915.json)。

发布接入阶段已通过 3,043 项检查、ASan/UBSan、cppcheck、C++11 节点编译及真实 SDK 动态链接。
覆盖阈值等号边界、低置信度空结果发布和基于未发布原始观测的后续恢复，
结果见 [confidence_publish_verification_20260915.json](tests/confidence_publish_verification_20260915.json)。

尺寸扩展阶段新增 32 项尺寸检查，合计 3,075 项检查、ASan/UBSan、cppcheck 和 C++11 节点桩编译通过。
该阶段按最小宽度、最小长度筛选，发布调用中的两个尺寸下限均为 0，由使用者设置实际下限。
完整结果见 [size_filter_verification_20260915.json](tests/size_filter_verification_20260915.json)。

区间扩展阶段新增 28 项区间检查，合计 3,103 项检查、ASan/UBSan、cppcheck 和 C++11 节点桩编译通过。
现有两参数/四参数调用的下限语义保持，发布调用已使用六参数区间接口。
完整结果见 [size_range_verification_20260915.json](tests/size_range_verification_20260915.json)。

## 处理规则

1. 函数开始只读取一次 ROS 当前时间。保留 `0 <= now - timestamp < 2` 的帧；
   按“2 秒以上丢弃”处理，恰好 2 秒也丢弃。负数、非有限和未来时间戳被排除。
   按时间排序；同时间戳只保留输入向量中的最后一帧，避免重复缓存抬高置信度。
2. 按用户确认，仅用 `x/y/dx/dy` 构造地图轴对齐矩形，`dx/dy` 为完整边长。
   `heading`、`polygons` 和 `id` 不作为关联条件；`type` 的变化也不阻止同一目标关联。
3. 当前框与目标最近一次观测的中心距离必须 `<=1m`，且
   `intersection_area >= 0.5 * min(area_a, area_b)`；仅边界接触不算重叠。
   该比例对应“覆盖任意一框的 50%”，不是 IoU。
4. 多候选冲突时按中心距离更小、重叠比例更大、原始索引更小的顺序做一对一关联。
   一帧内每个目标至多匹配一次；同帧多个框分别处理，不将其反复计为同一目标的多次观测。
   中间漏帧时可与窗口内最后一次观测重新关联，漏帧不会增加该目标的得分。
5. 每帧权重为 `w = exp(-(now - timestamp) / 1.0)`，衰减时间常数为 1 秒。
   `confidence = 该目标出现帧的权重和 / 所有有效帧的权重和`，空帧也计入分母。
   最后限制到 `[0,1]` 并写入返回对象的 `confidence`，不采用输入对象已有的 confidence。
6. 每个关联目标输出一个对象，保留其最新观测的所有其他字段，包括位置、尺寸、
   HDMap 车道 type、heading、速度和 polygons；输出顺序按目标首次出现顺序。
   返回 header 取最新有效输入帧。没有有效输入帧时返回默认空消息。

该 confidence 表示所给时间窗口内的加权出现率，不是经过标定的检测概率。
只有一帧有效输入时，该帧各目标的 confidence 都为 1；窗口内连续出现的目标也为 1。
时序函数不对位置做平滑、不按置信度阈值删除对象；其输出随后在发布前按置信度和尺寸区间筛选。
几何与 type 均取目标最后一次观测，本函数不重新做 HDMap 匹配。

有效帧内的非有限中心、非正/非有限尺寸，以及超出空间索引整数范围的坐标，会抛出
`std::invalid_argument`，不返回部分滤波结果；过期帧在几何检查之前就被排除。
未来时间戳按当前这次调用判断，ROS 时间回退后也不会将未来帧纳入窗口。

## 计算效率

用 1 米空间哈希网格索引每个目标最新中心，每个输入框只查所在格和相邻八格，
先算中心距离平方，再算矩形交面积。框几何预计算；对象仅保存输入引用，返回时才复制。
目标移动时通过记录桶内位置、末项填补更新索引，避免每帧重新构建全部历史目标的网格。

若输入帧数为 H，有效帧数为 F，有效观测数为 N，第 f 帧在网格内比较 C_f 对框、
通过门限的候选数为 E_f，平均哈希查询下耗时为
`O(H + F log F + N + sum(C_f + E_f log(E_f + 1)))`，
附加空间为 `O(H + N + max(E_f))`。目标密集挤在同一格时，候选比较仍可能退化；
这里采用距离优先的贪心关联，不求解全局最优身份分配。

## 验证

在工程根执行：

```bash
python3 src/pnc/src/robot_perception_convert/tests/verify.py --output /tmp/perception_temporal_filter_verify
python3 src/pnc/src/robot_perception_convert/tests/verify_publish.py --output /tmp/perception_filter_publish_verify
```

第一条从当前真实 `.msg` 生成 ROS 桩，以 C++11 和严格警告编译，验证时间与几何边界、
归一化、最新字段保留、漏检、多目标冲突、100 组已知身份的随机历史，以及
3,000 次空间查询与穷举扫描的候选集合一致性；另运行 ASan/UBSan 和性能测试。
第二条编译实际 PNC CMake 感知目标并链接实际 SDK，捕获真实回调传给 Publisher 的消息副本。
覆盖置信度与类型的发布顺序、空帧分母、两秒退出、原始帧缓存、重复/回退时间戳、
100Hz 缓存大小、异常回滚与恢复、CUBE/排除区过滤及五种车道分类，并运行 ASan/UBSan。
另覆盖首次无定位、定位过期/无效、连续坏帧、无回调时实际主循环清理、部分历史过期、
处理期间时钟跳变、零时间戳默认空结果，以及故障时不发布空心跳、不污染有效快照。
发布阈值回归验证 `0.25` 等号保留、低置信度剔除、非空输入全部被筛除后的空结果发布，
以及未发布观测仍参与后续置信度累计。
尺寸检查覆盖横向宽与纵向长独立设置、联合筛选、相等及相邻浮点边界、空输入、
非法尺寸/下限、消息字段保留和两参数调用兼容性。
区间检查另覆盖宽/长上限、区间两端、上下限相等、无限上限、上下限反转、非法区间以及四参数兼容性。
日志及 JSON 结果位于各自指定的输出目录。

目录外 `src/pnc/tests/perception/verify.py` 属于仅接入 HDMap 时的历史校验，
其中“空 MarkerArray 不发布”的断言已被本次空观测入窗需求替代；当前发布链使用上面的新脚本验证。

本机测试使用 ROS 桩，不包含 ROS1 通信、Orin 编译或车载时延；生产实现只依赖现有
ROS 消息/时钟与 C++11 标准库。版本记录和结果保存在本目录，遵循用户限定的改动范围。

## 2026-09-15 独立函数阶段校验记录

C++11 严格警告编译、27,149 项时序滤波检查、ASan/UBSan、cppcheck 和原有 71 项发布回归通过。
当时仅增加函数声明/实现的两处 include，尚未接入发布；以下为该阶段的算法性能记录。
本机 x86_64 / Intel Core Ultra 5 338H / GCC 13.3，滤波以 `-O2` 编译，
每组预热 10 次、统计 100 次完整函数调用，单位毫秒：

| 历史帧数 | 每帧对象数 | 输入观测总数 | P50 ms | P99 ms | 最大 ms |
|---:|---:|---:|---:|---:|---:|
| 40 | 100 | 4000 | 0.2461 | 0.2644 | 0.3179 |
| 200 | 100 | 20000 | 0.8083 | 1.2731 | 1.2827 |
| 40 | 300 | 12000 | 0.5033 | 0.5609 | 0.5702 |
| 200 | 300 | 60000 | 2.4806 | 2.5786 | 2.7421 |

完整记录见 [verification_20260915.json](tests/verification_20260915.json)。这些耗时不包含 ROS 通信，且不代表 Orin 实测。

## 2026-09-15 发布接入校验记录

实际回调链 2,758 项检查、节点/滤波 ASan/UBSan、cppcheck、C++11 CMake 节点编译及真实 SDK 动态链接通过。
新增缓存经过连续 350 帧、100Hz 输入检查；暂停时钟重复输入不增长；回退时清空历史。
通过一次性时钟故障注入验证滤波异常回滚，随后有效帧可恢复正常发布。
本次未改独立滤波算法、消息、HDMap 和目录外文件。详见 [publish_verification_20260915.json](tests/publish_verification_20260915.json)。

## 2026-09-15 空帧与过期安全复核

确认已有空观测入窗、两秒边界剔除、重复时间戳合并和异常帧回滚；补全无回调/持续失败时的
本地过期清理、定位有效期检查、工作帧隔离及发布前时钟校验。
最终通过 2,976 项实际发布链检查、27,149 项算法检查、两套 ASan/UBSan、cppcheck、
C++11 CMake 节点编译和实际 HDMap SDK 动态链接。
问题、修复方式、验证范围和运行边界见 [safety_review_20260915.md](tests/safety_review_20260915.md)，
机器可读结果见 [safety_verification_20260915.json](tests/safety_verification_20260915.json)。
