# 四列地图、废弃标志清理与自动写回验证（2026-09-20）

## 行为与修改边界

地图统一为`x,y,heading,speed`，speed单位m/s，缺少第四列时默认10。
planning真实`LoadPathFile`装载三列时补10，超过四列保留前四列；完整校验后保存到原文件。
已有四列保持原限速和文件内容，不因重新加载而写回。混合列数按行处理，BOM、空行、
整行注释、LF/CRLF和末行是否有换行均保留；前三列文本及有效第四列不重新格式化。

缺少坐标/航向、空字段、非有限数值、负限速等无法可靠补齐，整段失败且不保存半份结果。
需要转换时使用同目录临时文件流式写入，flush、保留读写执行权限、fsync、关闭，再rename；
前后核对源文件inode/大小/修改及状态变更时间，发现并行修改、只读或写入失败则保留原文件，
返回空路径。软链接保留链接并更新实际目标。节点需要地图及其目录的写权限。
任务拼接/作业路径继续使用既有空路径保护，不使用未保存的转换结果、不跨越缺失路线。

删除第五列业务读取、变道标志传递、相关成员及没有调用方的旧换道入口；保留通用
XYZ_COOR_S.z_axis及其几何算法用途，保留独立lattice功能。原前向路径赋值条件中的
Path_Id在此前始终设为5，删除多余标志分支不改变当前前向参考路径输出。

删除speed_limit.csv及LoadPointSpeedLimits/ApplyPointSpeedLimit、区域缓存和调用。
旧文件即使重新出现也不再生效；旧两处5m圆区域的0.5限速因此按要求取消。
第四列限速继续在原规划流程和最终发布取小，连接段/抽稀/样条保留最低限速；
闸机8m门控、其他safety、起步观察、D/R控制标定、急停和刹车参数均未修改。
未增加前方限速预判距离，地图数据在任务装载时读取，不是周期热更新。

## 地图审计与并行改动

- 当前24个CSV、30992条数据均为四列，前三列数值与本轮开工快照相同。
- 现存7份地图去第五列，原前四列文本和换行逐字节保留：pudong下aircraft_back、
  aircraft_go、go_start、go_straight、left_circle、park_back，以及pudong_air/312_316_01_01。
- speed_limit.csv按要求删除；path_pudong051501.csv由外部删除后，用户回复“是，保留删除”，
  已纳入部署删除清单。task222.yaml第6/16行仍引用它，使用该旧任务会因地图缺失加载失败；
  未擅自修改任务配置或恢复用户删除的地图。
- 工作期间检测到pudong_air/312_cargo_01_01.csv的格式及113行限速10→0.5的并行修改，
  原样保留并纳入部署包，坐标/航向数值不变。其余30879行speed仍为10。
  测试对实际地图验证有效四列和非负限速，不要求用户配置一律等于10。

## 编译与业务验证

本机没有ROS1，桩由真实msg生成。全量编译受宿主Boost影响用C++14，生产仍C++11；
控制真实实现及CSV读写辅助均另外以C++11编译。证据目录：
`/tmp/pnc_csv_four_20260920_145555/`，主结果`verify_final/result.json`。

```bash
python3 -B src/pnc/tests/planning/verify_map_speed_limit.py --output /tmp/pnc-csv-four-check
python3 -B src/pnc/tests/control/verify.py --output /tmp/pnc-control-check
```

| 检查 | 本轮实际结果 |
|---|---|
| PNC全量编译与链接 | 53个单元、6个节点；公共对象全部重新编译 |
| 地图限速/真实LoadPathFile与写回 | 1201项通过 |
| 闸机/终点规划/感知集成 | 6581 / 2481 / 995项通过 |
| 起步左二/安全隔离、退出不重入 | 615 / 756项通过 |
| 终点控制交接 | 113项，规划与控制分进程链接 |
| 四列解析 | C++11严格编译、ASan/UBSan各102项，24文件30992行 |
| 自动文件转换/保存辅助 | C++11严格编译、ASan/UBSan各79项 |
| 控制独立回归 | 153项，C++11控制节点编译链接通过 |
| 真实围栏读取 | 31032项，24份地图几何一致；三/五列、坏行/负限速/缺失安全返回，不写原文件 |

自动保存覆盖三列、四列、多列、混合行、零限速、文本额外列、BOM、CRLF、空行、注释、
无末行换行、重复读取inode/mtime不变、原权限、坏尾行禁止部分保存、只读文件、
目录不可写、加载期间源文件修改、软链接及失败后临时文件清理。
实际地图入口同时验证旧区域文件缺失、零限速、坏格式都不再影响原位置D/R输出，
第四列仍取小且其他安全来源保留。没有把删除区域限速后预期速度变化判为回归。

全量编译/集成完成后，因新增持久写回测试入口及并行地图编辑，单独重跑CSV解析/写回
普通及sanitizer用例，并合并至同一result.json；没有为测试驱动器的后续修改重复全量编译。
旧起步完整套件的2m期望与现行1m实现差异仍保留，只运行本次定向入口。
sanitizer仅覆盖CSV解析/写回，不宣称整个旧几何库通过sanitizer；未在车辆运行。

### 与本轮开工快照对照

`verify_compare.py`分别重编前后规划实现/节点及同一回放夹具；确认公共几何源码与
struct_type.h逐字节相同后复用本轮全量编译的公共对象。归一化隔离目录，并仅去除旧
区域初始化的11次path_dir读取事件后，完整trace逐字节一致：
635条路径、319条状态、315条声光消息（共1269条发布）、48条速度、335份内部状态相同。
不包含已明确删除的区域限速行为及用户并行调整地图限速产生的预期差异。
原脚本用子串计数把TASKSTATE也算入387，现改按行首STATE计数335，未修改trace。

静态核对PNC业务源码：无旧区域读取/应用/缓存、旧CSV变道开关/成员/函数残留；
相关接口所有调用同步调整。YAML、消息、launch、独立安全模块及控制标定代码与快照相同。
src/simview与src/ultra_command本轮源码未改；四列兼容由上一轮验证，本轮不冒充重跑其专项。

## 时间与空间

N为路径点数，K为旧区域点数，B为CSV字节数，L为最长行长度。
周期内移除O(K)区域扫描，第四列继续复用最近点O(1)取值；无新增周期遍历、文件I/O、
参数读取、缓存或堆分配。不把整个planning周期称为O(1)。

有效四列文件加载仍O(B)，新增一次stat；只有需要补列/截列时额外O(B)流式解析和写回，
并执行fsync/rename。额外内存按最长行O(L)及文件名长度有界，不缓存整份CSV文本；
临时磁盘占用O(B)，成功替换或失败时清除。原路径向量仍O(N)，转换状态只存在本次调用。
未测车端CPU或冷加载/写盘耗时，fsync可能增加该次任务装载延迟。

同输入宿主机测试固定CPU、前后交替5轮，每轮预热100周期后采样2000周期。
地图上限10、任务速度2，远离旧区域，原安全输入一致；测PathPlanProcess及两项发布，
stdout重定向/dev/null，含原printf计算、不含ROS传输。规划主实现-O2、两边公共几何-O0。
P50/P99为5轮中位数，max为全部轮次最大值，单位μs：

| 路径点数 | 改前P50/P99/max | 改后P50/P99/max |
|---|---|---|
| 200 | 1.629 / 1.920 / 19.266 | 1.594 / 1.880 / 12.251 |
| 20000 | 1.722 / 2.022 / 24.798 | 1.685 / 1.944 / 26.156 |

每2000周期两边均44000次既有分配、申请1748000 bytes；路径点仍36 bytes，
规划类2640→2544 bytes。完整基准进程RSS峰值5轮中位数16256→16296 KiB，
包括两种规模顺序运行。小幅波动不能换算为Orin负荷变化；最大延迟并未在所有规模下降。
原始数据见benchmark_result.json，业务差分见comparison_result.json。

## 变更文件与部署

本轮业务代码11份（相对src/pnc）：

1. include/common/path_csv.h
2. include/common/path_csv_upgrade.h（新增）
3. src/robot_path_plan/path_plan_comply.cpp
4. src/robot_path_plan/path_plan_comply.h
5. src/robot_path_plan/path_plan_task.inc
6. src/robot_path_plan/path_plan_reference.inc
7. src/robot_path_plan/path_plan_output.inc
8. src/robot_path_plan/path_plan_experimental.inc
9. src/robot_path_plan/reference/path_generation.inc
10. src/robot_path_plan/task/global_path.inc
11. src/robot_control/control_comply.cpp

测试改map_speed_limit、path_csv、terminal_stop、perception_safety_integration及三个验证
脚本，新增path_csv_upgrade，删除point_speed_limit及其入口；模块、路径、测试说明和
workflow同步更新。完整文件清单/哈希按本轮开工快照生成，区分本轮代码、现存并行修改及配套文件。

开工备份：工程同级pnc_csv_four_before_20260920_145555.tar.gz。
部署目录：工程同级deployment_20260920_csv_four/，含runtime.tar.gz、FILE_LIST.md、
DELETED_FILES.txt和SHA256SUMS。包内携带11份本轮代码、全部24份现存CSV；另附上一轮
struct_type.h、task/operation_path.inc、src/simview/src/draw.cpp三份依赖，合计38个运行文件。
这些配套文件本轮未修改，不计入本轮11份代码。历史部署包保持不动。

tar覆盖不会删除旧文件，必须按删除清单移除两份地图。PathPlanComply布局变化且包含
上一轮共享路径点布局，完整重编robot，携带view更新时一并重编view，并重启相关节点。
不要混用旧对象/库；节点写回三列地图需要目录/文件可写。task222.yaml的已删除地图引用
见前文，未擅自改任务列表。本轮未连接车辆或执行实际部署。
