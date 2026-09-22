# CSV 地图最高限速验证（2026-09-20）

## 需求与实施范围

`src/pnc/path/` 递归26个CSV、31,719行，第4列统一为10。用户没有另行确认单位，
已在工作过程中说明沿用planning和原区域限速的m/s口径：10 m/s = 36 km/h。
前三列原始文本、行数、行结束符及是否有末尾换行保持；原8个四列文件的旧第4列移为第5列。
旧值均为0，继续用于原z_axis/变道标志，不能将新限速误作旧标志。

规划复用当前行驶路径最近点索引，输出 `min(原规划速度, 当前地图限速)`，D/R均适用。
规划推进后和最终发布前各取小一次；不修改safety、车道类型、闸机8m门限、急停、
起步观察、控制D×18/R×10及制动参数，不修改ROS消息。
本次没有增加地图低限速区的前方预判/制动距离；限制的是规划速度，不能代表实车即时达速。

CSV读取不重采样。已有接入段会生成几何、每6点抽稀并执行0.1m参数步长样条；
新字段保留接入时越过的原始点和每个抽稀区间的最低限速，再单调传播到样条输出点。
周期参考路径另有采样和前视终点线性插值，限速直接读取原行驶路径索引，不随输出采样丢失。
纯生成、没有CSV来源的作业路径默认无限制，保持原任务速度；限制随所属路径复制和清除。

## 全工程读取链核对

按实际文件参数和调用链核对，未将名称里包含CSV但读取其他目录的模块一并改动。

| 真实读取方 | 实际输入/作用 | 本轮处理 |
|---|---|---|
| path_plan：LoadPathFile | path_dir与任务pathList拼成路线；作业辅助也调用 | 新逐行解析3/4/5列，第4列存地图限速，第5列存旧标志 |
| path_plan：LoadPointSpeedLimits | path_dir/speed_limit.csv | 兼容3/4列，第3列仍为区域限速，第4列不改变区域业务 |
| control：LoadPathFile | path_dir/fence.csv | 新逐行解析，围栏几何不受新增列影响 |
| simviewer：LoadMap | /robot/mapfile；整栈可指向robot/path | 新逐行解析3～5列，保留原显示转换/过滤 |
| ultra_command：LoadZones/LoadPolygonCsv | path_dir/pudong_air/{left1,left2,right}.csv | 原实现已逐行取x/y、忽略剩余列，验证后无需改源码 |

task_plan只下发路径名字，不读取这些CSV；HDMap读取其raw/processed目录，感知转换的
默认排除区读取pnc/config，均不是本轮path目录数据。监控及可选轨迹调试的相关解析
也按行提取需要的列，额外列不会引发错行。根launch配置的path/view.csv当前未提供，
simviewer现在遇到缺文件返回空地图并记录错误，未伪造或新增地图文件。

路径读取遇缺文件、坏行、非有限/负限速或底层读取失败，整段返回空；任务拼接不跨越
缺失段，作业辅助增加空路径访问保护。围栏失败清空本项数据；显示坏行跳过并统计。
同时移除原 `while(!feof) + fscanf` 在文件末尾多压入一个旧点的行为。

## 验证结果

持久复现入口（本机需已有yaml-cpp、PNC第三方库及本机架构HDMap SDK）：

```bash
python3 -B src/pnc/tests/planning/verify_map_speed_limit.py --output /tmp/pnc-map-speed-verify
```

本轮实际证据目录：`/tmp/pnc_map_speed_20260920_142627/`，最终入口输出在`final_verify/`。
本机没有ROS1，节点使用真实msg生成的ROS桩；Boost相关全量编译使用C++14，生产标准仍为C++11。

| 检查 | 结果 |
|---|---|
| PNC全量编译/链接 | 53翻译单元、6节点通过，全部公共对象按新布局重编 |
| map_speed_limit.cpp | 932项：新旧列数、坏文件、任务拼接、零限速、进度变化、D/R、多安全来源、无历史残留、接入/抽稀限速传递 |
| path_csv.cpp | C++11 -Wall -Wextra -Werror；常规与ASan/UBSan各106项，26文件31,719行 |
| 区域限速 / 闸机 | 284 / 6,581项通过 |
| 终点 / 感知集成 | 2,481 / 995项通过 |
| 起步左二及安全隔离 / 起步退出后不重入 | 615 / 756项通过 |
| 控制真实读取方法 | C++11，31,750项；26个文件及缺失/坏行/三四五列夹具 |
| simview真实LoadMap原文 | C++11严格编译；26文件迁移前后显示点坐标/方向完全相同，缺文件不崩溃 |
| ultra_command真实实现 | C++11；三个区域各5顶点相同，54,000次区域包含结果相同 |

旧起步完整套件仍有历史2m与当前1m差异，此次仅运行明确相关入口，不宣称旧完整套件通过。
ASan/UBSan覆盖CSV解析器，不宣称整个旧样条库通过sanitizer。
simview仅验证改动方法，未编译整个RViz节点；未在ROS1实机或车端运行。

### 既有业务差分

使用同一regression夹具，对比闸机8m版本与本轮最终实现，隔离夹具目录归一化后：

- 635条路径、319条状态、315条声光发布、48条速度记录及参数/发布事件均相同。
- 335份内部状态中330份完全一致；`same-task/new-task/repeat-message/connect-real-path`
  各去掉1个旧EOF重复末点，`multiple-csv`去掉2个，后一段最近点索引对应减1。
  程序化确认其余字段相同、留下的坐标/方向/里程逐点相同，索引仍指向同一实际位置。
- 终点测试此前2,483项，本轮2,481项，原因是两个CSV夹具不再包含读取器伪造的重复末点；
  没有删除测试断言或放宽业务期望。完整输出在`regression_final_comparison.json`。

## 时间与空间

N为源路径点数，M为样条输出点数。正常10Hz每周期新增两次O(1)索引读取与取小，
不增加扫描、文件I/O、参数服务器请求、容器或分配。路径载入仍逐行O(N)，
连接时一次性限速传递为O(N+M)，单调索引避免逐输出点全表搜索。

内部XYZ_COOR_S从32增加到36 bytes；额外内存为现有各路径向量capacity之和×4，
随向量原生命周期使用/复用，不按帧累计历史。20,000个有效点的字段净增80,000 bytes；
实际分配还取决于原vector容量。CSV读取移除原中间复制容器。

同输入宿主机对照：固定同一个CPU，前后交替5轮，每组预热100周期、采样2,000周期；
地图限速10、任务速度2，原安全输入一致。测PathPlanProcess、PublishReferPath、
PublishPlanPath，stdout重定向/dev/null，包含原printf计算但不含真实ROS传输。
规划主实现-O2，未改公共几何对象两边均-O0，各自使用兼容布局。
P50/P99为5轮中位数，最大值为所有轮的最大单周期值，单位μs：

| 路径点数 | 改前P50/P99/max | 改后P50/P99/max |
|---|---|---|
| 200 | 1.641 / 1.943 / 18.622 | 1.637 / 1.928 / 13.293 |
| 20,000 | 1.723 / 2.015 / 37.837 | 1.741 / 2.038 / 22.437 |

每2,000周期两边均44,000次原有分配、申请1,748,000 bytes，即本轮没有新增周期分配。
完整基准进程峰值RSS（两种规模在同一进程依次运行）5轮中位数16,152→16,296 KiB。
轻微波动和单次调度峰值不证明性能提升，也不能推算车载CPU百分比；未实测Orin、真实感知
密集场景或冷启动载入耗时。原始5轮数据、内存统计在`benchmark_result.json`。

## 文件、备份与生效

业务代码11份：`include/common/path_csv.h`（新增）、`struct_type.h`；
planning的`path_plan_comply.cpp/.h`、`path_plan_task.inc`、`path_plan_reference.inc`、
`path_plan_output.inc`、`task/global_path.inc`、`task/operation_path.inc`；
`robot_control/control_comply.cpp`；另包`src/simview/src/draw.cpp`。
另更新26份CSV，测试/说明文件列于工程同级部署目录清单。

开工备份：工程同级 `pnc_map_speed_before_20260920_142627.tar.gz`（pnc、simview及workflow）。
部署准备目录：工程同级`deployment_20260920_map_speed/`，包含同步包、逐文件校验和与清单。
由于内部C++结构布局改变，必须同步相关源码和CSV、完整重编robot，另重编view；
依赖和消息预先生成，重启规划/控制/显示节点后生效。不能混用旧.o/旧库或只换CSV。
此前`deployment_20260920_after1200/`是历史闸机/左二包，未覆盖或替换它；
本轮没有连接车辆、执行部署或修改根启动运维脚本。
