# 工作进展与当前状态

更新：**2026-09-22**。本文件供任务延续使用；最近详细、较早简述，当前有效约束和未解决
问题始终保留。完整旧记录见 [整理前归档](docs/history/2026-09-19-before-docs/README.md)。
工程用法见 [README](README.md)，编码前完整阅读 [风格记忆](docs/CODING_MEMORY.md)。

## 当前基线与有效约束

- **2026-09-22 按用户确认完成control/D沿程限速、电机平顺与末停修复；本机验证通过，未部署/未实车验收。**
  业务仅control_comply.cpp/.h、forward_brake_control.inc；R/N220场景8925条完整输出与冻结原版一致。
  名义4v预瞄保留已发现的前方位置，沿程包络提前收速；静态曲率4901标定点及几何抗噪保持。
  正常电机参考减速0.6、恢复0.4m/s²、内部jerk0.5m/s³，硬规划/安全限制继续立即抢占。
  正常末停在回调锁存有效终点及位置，负距离重算不单独切80%；仍核验真实位移并保留超时/安全保护。
  大需求观察0.4s并核验近期电机减速度，响应后每0.6s最多+3%；小请求仍3%、普通上限15%、积分默认0.2m饱和。
  1.8s仅保护未决再加压；小请求无反馈不单凭速度误差锁100；手动D或N停稳零反馈0.5s可清D专属故障。
  下方旧“手动N不能恢复/所有释放固定等1.8s/普通请求均0.6s+每次1%”均已被本条取代。
  节点C++11构建、11288项反馈、10685项状态、1840项新增平顺断言通过。
  同bag8316帧大幅电机给定下降129→43、上升86→9，无新增液压；这不是新算法闭环实车验收。
  路径更新增加O(N)预计算，缓存增加8N bytes、对象增加240 bytes；微基准路径更新耗时增加，不能称CPU零增加。
  [当前说明与命令](src/pnc/tests/control/forward_speed_smooth_20260922.md)，
  快照/证据`../control_d_smooth_20260922_103950/`，改前归档同名`_before.tar.gz`。

- **2026-09-22 用户确认围栏迁移业务冲突后，生产修复完成，本机验证通过；未部署/未实车验收。**
  task_plan负责失败关闭、连续安全区间、截停后取消作业、原2.3m/±0.73m前角实时保护；ADAPTIVEHOOK/DOACTION例外保留。
  planning以新增task_fence_guard消息核对任务版本/几何版本/批准索引，检查最终生成路径，失联0.5s叠加safety停车。
  同代完成反馈避免旧TASKFINISHED推进，HMI失联1s显示未知，control不再启动写alarmcmd=0，D/R控制算法不变。
  同步19个生产文件并完整重编robot，重启task_plan/path_plan/control/HMI；CSV、YAML、launch、既有消息均未修改。
  围栏59项及原专项、53单元/6节点、D13462/状态9764/曲率17609、非D8925条和规划安全专项通过。
  下方原始“缓存O(1)/0.13微秒”“围栏已完整迁移”结论已被本条取代。任务校验缓存命中仍O(N)，缓存有界。
  说明见[task_plan](src/pnc/src/robot_task_plan/README.md)，[19文件部署包与清单](../pnc_fence_fix_delivery_20260922/部署说明.md)。

- **2026-09-22 按用户确认修改control/D制动生产代码，本机验证完成，未部署/未实车验收。**
  业务仅`control_comply.cpp/.h`、`forward_brake_control.inc`；保留此前围栏迁出及其他模块改动。
  已修复旧手动N反馈误锁新D停车、自动N/R在途请求遗漏、观察期时钟回退；新请求独立计时，
  最早未确认强请求不能被反复撤销/重发拖延。R原1～5%小补刹与断流维持的旧普通请求均保留来源分类。
  普通不足须新加速度估计连续确认0.3s（进入短缺>0.12、退出≤0.05m/s²）；积分默认0.1m门槛/0.2m上限保持。
  小请求2s无反馈先未确认、撤普通液压并禁油，不凭超时放油或仅此锁100；已有请求仍明显超速且持续不足则独立故障。
  取消规划下降/迟到一次正反馈不能洗掉失败证据；15%饱和不足的1s确认也仅由新估计推进。
  强制停车移动中无响应、真实未释放/持续断流/液压中时钟异常及独立急停仍保护；新鲜停稳保留保持量。
  **未知/故障人工解除仍要求手动D、停稳≤0.03m/s、新鲜零制动连续0.5s；手动N不能替代。**
  3%/15%/1.7s预测/1.8s保护及0.6s电机观察保留，最低可靠液压请求与剩余降速模型待实车标定。
  C++11节点构建通过；新增20场景9764断言、旧D13462项、独立15场景36断言通过；
  曲率17609项、4901标定点不变，R/N220场景8925条完整消息一致。直线5m/s、4s预瞄/抗噪、横向保持。
  新状态固定增加64 bytes（主机ABI），协调O(1)，没有新增路径扫描；性能边界见说明。
  同步三个业务文件，重编robot并重启control；不改参数YAML。
  [当前说明](src/pnc/tests/control/brake_state_revision_20260922.md)，
  [最终审计](../control_brake_state_evidence_20260922_082147/final_audit.json)，完整证据与备份见下方日志。

- **2026-09-22 原始围栏迁入版本（历史；缺口已按本页最新条目修复）。**
  按用户明确要求，在 `robot_task_plan` 模块中实现电子围栏校验，防止规划轨迹越出安全围栏：
  - **启动加载**：在 `task_plan_node.cpp` `main()` 中启动时自动获取 `path_dir`（参数服务器读取、从 `task_file` 推导、或兜底 ROS 包路径），调用 `LoadFenceFile` 读取 `path/fence.csv` 加载至内存并计算 AABB 包围盒（117 个顶点，范围 X:[-356.57, 169.54], Y:[-657.46, 1175.59]）；
  - **任务校验与终点截断**：在 `task_plan_core.cpp` 的 `SetTaskInfo`（常规任务与 1000 号远程指车任务）中，装载轨迹点与任务终点后调用 `ValidateTaskPoolWithFence`：
    - 若终点以前全部轨迹点都在电子围栏内，正常外发，不修改任何参数；
    - 若终点以前存在电子围栏外的点，将轨迹离开电子围栏前的最后一个在内点作为任务的停车终点（`stopX, stopY, stopAngle`）发送给 planning 模块；
  - **严格零修改磁盘文件**：即便终点更新，绝不修改磁盘上任何 YAML 或 CSV 文件，终点更新仅在内存对象（`mTaskPool` / `mTaskList` / `task_plan_msg`）中流转；
  - **空间换时间极小化 CPU 负荷**：
    - 预计算 AABB BBox：四次比较 $O(1)$ 快速排除围栏外延点；
    - 内存哈希缓存 (`mPathTrajectoryCache`)：路径首次加载后将其全部轨迹点与 `in_fence` 状态持久缓存至内存，缓存命中只免去重复读盘/部分几何计算，整次校验仍为 O(N) 扫描和临时空间；原0.13微秒结论不能用于该函数或车载性能；
  - **零 ROS 依赖核心架构保持**：`task_plan_core` 保持纯 C++ 零 ROS 依赖设计；
  - **全套测试验证**：
    - `compile_all.py`：53 个翻译单元全过，6 个可执行节点成功链接；
    - `test_task_plan_fence.cpp`：覆盖围栏加载与 AABB、围栏内正常外发、越界轨迹安全截断（实测 pudong/go_straight 越界点截断至第 365 点安全内点 [3.11, -40.48]）、缓存性能压测、动作任务容错、磁盘 YAML 逐字节无修改 6 大断言全数 PASS；
    - 历史测试回归：`verify_forward_brake.py` 13100 项制动断言、`verify_map_speed_limit.py` 12 项测试套件全部 PASS。
- **2026-09-22 control 模块电子围栏代码完整清除，全套回归验证通过。**
  按用户明确要求，电子围栏相关逻辑不再放在 control 模块中：
  - `control_comply.h/.cpp`：删除了 `LoadPathFile`、`pointInPolygon`、`local2global2`、`FenceAlarm` 4 个函数声明与定义，删除了成员变量 `mFenceList` 与状态 `FenceWarning`；
  - `control_node.cpp`：移除了启动时 `LoadPathFile("fence")` 及 20Hz 主循环中的 `FenceAlarm()` 调用；原始版本曾保留 `alarmcmd=0`；本页最新修复已删除此初始化并由 task_plan 接管告警；
  - `control_comply.cpp`：移除了 `VehicleControl()` 中的 `FenceWarning == 1` 停车制动块，移除了 `TerminalStopBrake()` 中的 `FenceWarning == 0` 条件；
  - `forward_brake_control.inc`：移除了 `ForwardTerminalStopBrake()` 中的 `FenceWarning == 0` 判定条件；
  - `path/fence.csv` 地图数据文件本身在 `path/` 目录下完整保留；
  - 严禁影响的其它业务逻辑经全量自动化测试验证 100% 保持：
    - `verify_forward_brake.py`：13,100 项前向制动断言全数 PASS；
    - `verify_curve_history.py`：17,609 项 D 挡曲率历史与动态预瞄断言全数 PASS；
    - `verify_non_d_brake_compat.py`：220 场景 / 8,925 条完整非 D（R/N）控制消息逐字段与改前基线完全一致；
    - `verify_map_speed_limit.py`：全量规划与控制交接测试全数 PASS（含 terminal_control 113 项交接、1201 项限速、6581 项闸机、2612 项末停、995 项感知、615 项起步、756 项观察等）；
  - 备份快照：`pnc_control_before_fence_removal_20260922_072700.tar.gz`。
- **2026-09-22 两份09-20旧bag制动辨识与代码复核：当时发现的逻辑缺陷已在同日生产修复，物理标定仍待完成。**
  用户明确bag用于了解电机回馈/液压非线性，不是新算法实车验收。191056严格匹配普通制动8次，
  接入0.328～0.553s、撤请求后结束1.012～1.525s，6/8先撤请求后实际接入；无独立持续3%试验。
  液压反馈全0时也有0.397s内车速0.818→0.094m/s的电机响应，固定1.8s零给定可能加重欠速。
  真实当前C++复现P1：已释放手动/N旧反馈遇首D正停车请求立即误锁100；自动N已发制动未登记，
  N切D可在迟到液压到来前恢复油门。另P2：液压未接入观察期时钟回退不重建requestTime。
  这些是独立源码反例，不声称完整序列已在旧bag实发；上轮13100断言未覆盖这些组合。
  两包8次真实源码回放、11组两版3982帧业务复现已完成；旧输入分叉后的故障不能统计为新算法失败率。
  **该分析阶段未改生产control或参数；上述逻辑缺陷已由随后用户授权的生产修改处理，见最新条目。** R/曲率既有约束保持。
  交付校验另发现并发删除control电子围栏，已保留分析起始源码/哈希及差异，并对最新源码重新复核制动反例；
  原8次bag回放对应起始版本，不能把其哈希称为围栏删除后的当前版本。
  [完整结论与证据](../brake_validation_20260922/验证结论.md)，[独立复现](../brake_validation_20260922/review/README.md)。
- **2026-09-22 monitor 电子围栏数据对齐（100%匹配PNC）与三态视觉语义升级（安全黄/警示带橙/违规红）。**
  用户更新了 `monitor/fence/fence.csv`，经校验与 `src/pnc/path/fence.csv` 逐字节完全一致（117边/118顶点，SHA-256完全相同）；
  机坪在役任务点落在围栏内的比率全面恢复至 100.0%（`312_cargo_01` 2332点、`312_charge_01` 5290点、`312_316_01` 1555点），彻底根除此前 29 点旧围栏下的误报警红框。
  视觉语义全盘采纳建议落地三态车框变色（`fenceState(x, y)` 与 `updateVehicleColor(p)`）：
  1. 状态 0（安全态 Safe）：自车定位中心落在任一围栏本体多边形内 -> 车体轮廓为黄色（`#ffff00`）；
  2. 状态 1（预警态 Warning）：自车定位中心越出本体红线、处于外扩 2.5m 黄色斜线警示带内 -> 车体轮廓变为橙色（`#ff8c00`），实现缓冲预警且与地面黄色斜线保持视觉认知一致；
  3. 状态 2（违规报警态 Violation）：自车定位中心完全越出外扩 2.5m 保护红线 -> 车体轮廓变为红色（`#ff0000`）；
  4. 2D Canvas 降级模式同步接入 `FENCE_VEH_HEX[fenceState(v.x, v.y)]` 三态变色；
  5. 状态变量由布尔值升级为 `vehFenceState = -1`，仅在三态翻转时切换材质颜色，杜绝高频 GC；
  6. 后端 `ros_visualizer.py` 的 `fence_payload()` 支持优先通过 `rospy.get_param` 读取 `/robot/fencefile` 或 `path_dir/fence.csv`，保持与 PNC 栈动态同源，无参数时优雅降级读取 `$MON/fence`。
  回归测试：`test_full.py` 98 项、`frontend_test.js` 127 项全数 PASS（0 failed）。
- **2026-09-21 D挡制动改为实测减速度/实际液压反馈协调，本机验证完成，待实车标定。**
  用户要求业务修改完全限定control，且不影响R。仅改control_comply.cpp/.h并新增forward_brake_control.inc；
  原电机D×13.5、+0.5、曲率/4秒预瞄/抗噪/横向保持，其他模块业务与参数不改。
  普通下降先观察电机0.6s，减速度不足才积分（默认门槛0.1m、上限0.2m）；首补刹3%，
  实际接入后每0.5s最多+1/-2个百分点、普通上限15%，不再用旧dec×15。
  允许相对本次最低规划目标≤0.1m/s的小涨，仍只由原始规划下降启动普通积分。
  按实测减速度提前撤刹；在途/接入/卸压禁止恢复驱动，规划复位不能清执行记忆。
  独立急停立即抢占；无响应/不释放/执行失效等故障锁100，恢复须手动D停稳并零反馈连续0.5s。
  新时效保护只作用于液压过程，普通直线不增加0.2s断流急刹；正常低速末停连续承接已有刹车。
  C++11节点桩编译/链接、13100项D、17611项曲率通过，4901标定点不变；224场景9136条R/N消息一致。
  新状态固定224 bytes，协调O(1)；现有bag不足以辨识可信制动逆模型或验收5m/s急停。
  下游canbus急停覆盖/刹车清转角及指令时效链保持，本轮不保证整车紧急制动距离或实际jerk。
  需同步三个业务文件，重编robot、重启control。参数/证据/恢复方式见
  [当前制动说明](src/pnc/tests/control/brake_feedback_20260921.md)，快照及命令见下方日志。
- **2026-09-21 canbus支持托盘反向端点标定，本机专项通过，待车端构建/重启。**
  pallet的min固定表示上端、max表示下端，允许数值反向；当前配置212/115，绝对间距97。
  两端均须为0～255整数、间距至少20；非法/缺失成对回退213/254，hook校验及配置178/252保持。
  到位仍要求与端点绝对差<5、连续11个T1回调命中；当前上端208～216→4，下端111～119→3。
  用户反馈实际122且palletStatus=0，此值距下端115为7，按现有条件未到位；
  122是否为实际最低端尚未确认，未据此改为122或放宽容差。0不代表已确认物理运动。
  C++11实际node源码桩编译、34组配置/启动用例、6422项状态断言通过；旧源码对照确认拒绝212/115。
  无高频路径改动；启动解析仍O(N)时间/O(1)额外空间。文件、命令及备份见下方日志。
- **2026-09-21 按用户要求更新 ultra_command 目标监控路径，本机回归通过。**
  将 `pudong_air/312_316_01`、`pudong_air/312_cargo_01`、`pudong_air/312_charge_01` 分别更新为
  `pudong_air/312_316`、`pudong_air/312_cargo`、`pudong_air/312_charge`；
  `_01_01` 初始化监控路径及起步判据保持。
  comply/node 注释、package.xml、README 与回归测试同步更新。
  35 项导航桩/普通模式/初始化监控/旧路径失效断言全数通过。需重编 ultra_command 并重启。
  快照备份：`ultra_command_before_20260921_180300.tar.gz`。
- **2026-09-21 HMI组3新增"自定义指令"卡片独立启停 ultra_command，本机验证+对抗复核完成。**
  `launch/control.launch` 不再 include ultra_command；卡片参与一键启动（非 optional，
  失败中止组4），nodes 型健康检查 `/ultra_command_node` 与 `ros_bridge._node_patterns()`
  同步（另有 test_config_sync 常驻守卫）；`start_l4.sh` 救急路径补独立终端行保持覆盖；
  stop_pat 兼顾 launch 与节点名防 SIGKILL 孤儿。pnc 卡标题节点数经复核为 6（按 node 标签
  实数；旧"6节点"在捆绑期实际 7 个，同为差一）。**部署必须连 control.launch 与 start_l4.sh
  一起同步车端，只拷 hmi/ 会与旧捆绑实例双起同名节点互杀。**
  已知边界：只启 pnc 不启本卡=矩形监控静默缺失（planning 缺省 0 无告警）；nodes 检查发现
  不了 respawn 快速崩溃循环（车辆侧 safe=1 停车兜底）。停止卡片=节点退出前写 `safe=1`
  （planning 停车属预期）。未部署车辆；详见下方日志与校验轮记录。
- **2026-09-21 按用户明确要求清理9月10日以前的无用备份，已删除11个文件，共3,216,679字节。**
  范围仅工作区根目录8个旧tar.gz、2个旧感知模块ZIP和1份旧workflow副本；不含9月10日当天。
  仍被canbus说明引用的09-05分析报告、9月10日及以后备份、现行源码/地图/参数/运行环境保持。
  本次清理为用户对旧备份的明确授权，其余保护规则继续有效；旧归档中的相关备份引用已失效。
  [删除清单与核对记录](docs/backup_cleanup_20260921.md)。
- **2026-09-21 D曲率预瞄由固定6m改为当前导航车速×4s，本机专项通过。**
  1/1.5/3/4.5/5m/s分别预瞄4/6/12/18/20m；无6m下限，零速仍取投影曲率并沿用历史。
  负速度按0m，非有限导航速度沿用安全停车；从导航参考点路径投影沿线起算，轨迹不足只到真实末点。
  仅动态选择已有缓存中的曲率点，不重算几何缓存；限速公式、前后各2m抗噪几何、30周期释放保持。
  延续上轮两版中值公式：
  参考90度右转上限1.4m/s，对应旧公式推算等效k=0.04、R=25m；k<=0.01完整保留，
  0.01～0.04沿用五次平滑下降倍率，k>=0.04使用0.28/sqrt(k)。已通过实车的直线/抗波动策略保持。
  保留导航车速+0.5、D×13.5/5%保底、横向/R及独立safety。
  新窗口真实直线88帧上限6.326–7.513m/s，输出与改前相同；5m/s给定专项稳定67%油门、0%刹车。
  扩大预瞄可能选中更远弯道，提前限速是本轮预期变化，不保证所有同一路径帧限速值不变。
  沿用上一轮积分介入后的普通补刹系数15（原50的30%），I+=dec*dt及规划下降门控保持；
  默认门槛0.1m，饱和上限2倍门槛、默认0.2m，积分不直接转换为刹车百分比。
  C++11节点构建、17611项曲率、16158项补刹、18组积分时序/饱和及190条R消息对照通过。
  同曲率4901点标定完全一致，96组动态窗口对照通过；3551帧回放几何/横向字段保持。
  业务修改control_comply.cpp，头文件仅注释；无新缓存/成员/分配，扫描点数随预瞄范围变化。
  重编robot并重启control生效，本轮待实车复测。[动态预瞄与验证](src/pnc/tests/control/curve_preview_4s_20260921.md)。
  旧主套件23项既有预期差异、原短路径越界和极低速保底边界保留说明，不扩改其他业务。
- **2026-09-21 删除D挡control起步上升斜坡，本机回归通过。**
  起步速度规划由planning负责，control限速后直接按D×15输出；移除斜坡函数、状态与参数读取。
  保留规划下降触发的刹车积分、曲率限速、原导航车速+0.5m/s指令上限、5%保底和全部安全停车。
  原复位入口仍清D曲率历史，非正给定仍归零；R行为保持。业务仅改control_comply.cpp/.h。
  C++11节点构建、247项控制、7473项积分请求、695项曲率及导航控制专项通过；详见下方日志。
  需重编robot并重启control，未部署或实车验证。旧launch_speed_slope参数不再生效。
- **2026-09-21 D挡积分补刹仅由规划目标下降触发，本机回归通过。**
  `desireSpeed`下降建立减速请求，同值目标保持请求；目标回升或导航车速达到规划目标后清除。
  仅实际超速、曲率/故障本地限速不能启动积分，状态切换/停车清历史，后续需新的规划下降。
  当时保留0.1m门槛、dec×50、D×15/5%保底；最新力度及标定以上方现行条目为准。
  R原行为与独立安全停车保持。该次业务仅改control_comply.cpp/.h；
  7473项触发专项、244项控制回归及导航/曲率/末端检查通过。需重编robot并重启control，未部署实车。
- **2026-09-21 导航速度切换复核：修复异常值输出遗漏，正常业务差分通过。**
  导航NaN原可导致D挡NaN转角且仍驱动，负无穷可进入最终规划目标；现控制行驶计算前停车，
  规划最终出口叠加安全零速，保持原终点/N挡等停车优先级，导航恢复不清除其他safety。
  同速度输入下186组6197条控制消息一致（仅排除预期改变的实际速度上报字段）；387组规划状态、
  635条路径消息、48条速度及参数事件完整一致。139项导航规划、210项异常/恢复专项通过。
  详见 [复核报告](src/pnc/tests/navigation_speed/review_20260921.md)；仍未部署或实车验证。
- **2026-09-21 CAN实际车速弃用，PNC/ultra_command统一导航速度，本机回归通过。**
  所有实测自车速度取 `/navigation_msg.gpsSpeed`；规划平滑/避障/起步确认、控制D/R反馈、
  状态心跳及控制转发已切换。CAN仍提供挡位/模式/转角/急停/挂钩与载荷；地图限速和目标速度保持。
  起步只以不同接收时刻的导航帧计票；控制积分/终点缓刹同时核对导航与CAN状态时效。
  HDMap没有CAN实测车速入口，几何样条导数与目标路径字段保持。保留用户修改的D×15及5/15保底。
  53个翻译单元/6个PNC节点桩构建、控制/规划/导航专项通过，详见下方日志和
  [验证说明](src/pnc/tests/navigation_speed/README.md)。需重编robot/ultra_command并重启，本轮未部署车辆。
- **2026-09-21 调试结束，planning临时速度打印已全部删除。** 移除本轮新增的50处编号
  std::cout，包含期望速度、GPS速度及车辆速度；8份规划分片与添加调试输出前逐字节一致。
  planning桩重编及1201项地图限速回归通过，运行日志不再出现编号输出。
  早先仅清理61处纯调试转储的范围保持；错误/停车原因、任务交互、ROS日志、主动诊断接口、
  求解器设置和原测试继续保留。[编号表](src/pnc/src/robot_path_plan/desire_speed_trace.md)已标记为历史记录。
  需重编robot并重启planning生效，本轮未部署车辆；控制模块另有外部修改，未被本次操作覆盖。
- **2026-09-20 D挡单次参考轨迹延长到最多60点，本机验证通过，待车端验证。** 保持原采样密度，
  路线足够时输出60个真实点；R及其他挡位保留原约30点/1.5倍前视，末端不外推、不加密凑数。
  业务仅改planning的`reference/path_generation.inc`；地图、停车限速、safety阈值和control代码未改。
  下游数据量会随轨迹增长；主机采样函数更快不能外推为整车CPU负荷降低。详见本轮日志。
- **2026-09-20 D挡曲率历史残留已修复，本机验证通过，待车端同步验证。** D使用独立30项循环历史，
  连续直线30次计算可淘汰旧曲率；任务/路线/路径编号、挡位/模式变化及原停车入口清D缓存。
  R继续原曲率函数、标定和控制流程。公式、取点范围、已回滚的70%/30%横向融合及其他safety保持。
  交付核对时另有并行D横向调参，17:53快照因std::abd拼写未通过编译；保留该改动，详见本轮日志。
- **2026-09-20 后续D挡横向调校已回滚。** 用户实车反馈随车速调权后效果更差，已恢复调权前
  的Stanley源码，D挡固定70% Stanley/30%纯追踪；回滚当时control目录12份源码/头文件与快照一致。
  后续D曲率修改见上条；横向回滚本机153项控制回归通过，车端需同步重编重启。
- **2026-09-20 实车验证状态：用户确认，截至本次反馈时，今日修改的部分均已实车测试验证通过。**
  后续工作以这次确认时的版本为已通过实车验证的基线；今日相关记录中的“尚未部署/待实车验证”
  是交付当时的历史状态，最新状态以本条及下方用户确认记录为准。
- 2026-09-20 `pnc/path/` 当前24个CSV、30992行统一为x/y/heading/speed四列；第五列变道
  标志及speed_limit.csv区域限速业务已删除。planning加载三列补10 m/s并保存原文件，多列
  保留前四列，已有四列限速不重写；非法几何或保存失败返回空路径、保留原文件。
  path_pudong051501.csv按用户确认保留删除；现有312_cargo_01_01.csv的113行0.5限速保留。
  最近点限速仍O(1)，连接插值保留限速。详见 [路径格式](src/pnc/path/README.md)及
  [现行验证](src/pnc/tests/planning/csv_four_column_verification_20260920.md)。
- 2026-09-20 planning 在 `/perception` 和 `/perception/planning` 消费处剔除 `type=2`
  左二目标；常规跟踪有效整帧清除同 ID 旧轨迹/确认，兼容链同步清风险历史，起步观察
  新帧重建缓存。其他车道及 safety 来源保持原规则；复用原遍历和 ID 表，无新增容器。
- 2026-09-20 性能恒定准则：用户反馈车载 CPU 接近饱和、内存尚有余量；每次改代码前
  重读 [风格记忆](docs/CODING_MEMORY.md) 中的性能准则，优先降低时间复杂度，允许有界
  空间换时间，保持业务/safety 语义和缓存失效时机。新增开销与未实测项必须如实说明。
- 本树是车载 `/home/nvidia/qingwei-L4-No2` 的重建副本，非 git。09-05 用户确认的实车版
  是早期基线；09-10 用户重新指定现树为 PNC 基底，09-11 又同步过车载回滚。此后持续修改，
  不能再把现在整树等同于其中任一旧快照，也不能用旧“已部署”结论覆盖近期改动。
- 2026-09-19 新增的起步、缓行、减速和制动调整**只作用于 D 挡**。R 挡保持当天首次
  修改前行为；09-18 已有的 R 挡末端转向修复仍是基线的一部分。
- 2026-09-20 闸机输入增加参考路径位置门控：两固定点到最近参考点<2m，且该点沿线在
  自车前方`0 < Δs < 8m`（当日后续由4m放宽）。沿用闸机原D/R覆盖范围，失去位置条件即清除闸机缓存；其他safety独立保留。
- 底层电机为速度环：当前D×13.5（保留09-21用户修改）、R×10（上限45%）。普通限速不得清除
  原 safety；`dec` 在补刹代码里是导航车速减限速目标，单位 m/s，不是物理减速度。
- 主链：FMS → task_plan → path_plan → control → can_comm → canbus；感知来自
  CenterPoint `/box`，转换后分 `/perception` 与 `/perception/planning`。后者为规划真实观测。
- PNC 自有 C/C++：C++11、4 空格、同行左大括号，include/既有名字不重排不重命名。
  canbus 按专用指南保持局部风格。`.inc` 只由既有 `.cpp` 包含，不独立加入 CMake。

## 2026-09-22：planning 转向灯重写复核（未发现错误；1 处回归 FAIL 与其无关）

用户自行把 `UpdateSoundLightCommand` 的 D 挡转向灯由旧"单点(第10点)偏角>6° 即亮"
重写为：18m 前瞻扫描 max 左/右偏角（度），左/右转意图=max≥35° 且末端≥20° 且 S 弯
反向抑制（对侧 <15°）；候选连续 3 周期(0.3s)防抖点亮；最小保持 1.5s（或 15 周期
steps 兜底）；左右偏角收敛 15° 内连续 5 周期(0.5s)熄灭。新增 6 个状态成员在
InitParameter/ResetParameter/resetTask 及非 D 分支全部复位。备份
`planning_turn_light_before_20260922_103000.tar.gz`。

复核结论（逐项核对+真实编译回归+归因实验）：**未发现修改错误**。角度单位一致
（PointDirectionToMe 返回度、正=左，3/4 灯值与 canbus 映射不变）；C++11 合法
（NSDMI/std::hypot，<cmath> 已含，6 节点链接通过）；新代码顺带修复了旧版
`min<int>(10, size()-1)` 在空路径下 size()-1 下溢访问越界的隐患（新版守卫
size()>=2）；防抖/保持/熄灭状态机逐周期推演无互锁漏洞；safety=1 末尾覆盖
LightCmd=8 的优先级保持；R 挡 5 与非 D 清态保持。每周期新增 O(前瞻点数)三角
计算（≤~90 点，10Hz）+固定 48 bytes 状态，符合 CPU 恒定准则。行为变化属设计
意图：6°→35° 只有真转角才亮、缓弯(20°~35°)不亮、S 弯两侧≥35° 完全不亮、
亮后至少保持 1.5s、按对齐收敛熄灭；`turning` 字符串下游显示语义随之变化。

**回归注意**：`verify_map_speed_limit.py` 当前 `terminal_control` 1 项 FAIL
（"actual planner arrival enters low-speed D terminal brake ramp"）——归因实验
证实把 3 个灯光文件还原为改前版后同样 FAIL，系并行 control 末停平顺修改
（10:55，见 control/D 沿程条目）与该 planning 侧旧断言的冲突，**与灯光改动
无关**；该断言期望需由 control 侧改动一并更新。灯光改动自身覆盖的规划域
7 套件（限速 1201/闸机 6581/末端 2612/感知 995/左二 615/起步 756 等）全过。
未实车验证灯实际闪烁节奏与 canbus 映射表现。

## 2026-09-22：按确认方案完成control/D沿程速度及制动平顺修复

- 最终审计发现planning的`path_plan_output.inc`/README及两份cargo CSV存在并发更新；本轮未写入，
  保留现状且未纳入control三文件部署包。完整哈希差异见`../control_d_smooth_20260922_103950/scope_audit.json`。
- 用户授权限定control的D挡，保留R逻辑。生产修改仅`control_comply.cpp`、`control_comply.h`、
  `forward_brake_control.inc`；增加两份平顺测试文件、更新两份液压测试预期及模块/测试说明。
- 实现与参数见[专项报告](src/pnc/tests/control/forward_speed_smooth_20260922.md)。修复预瞄随减速收缩释放、
  电机给定跳变、末停距离重算中断、固定禁油等待和手动N恢复边界。保持独立安全制动及原规划下降积分授权。
- 真实bag回放额外暴露“提前补刹时电机减速尚在建立”和“两个状态回调跨过控制周期”两个边界，
  已加入近期减速度校验、有效到位回调锁存和相应回归。无新共享消息/配置/其他模块业务变更。
- 验证命令：`tests/control/verify_forward_brake.py`（11288项及C++11节点构建）、
  `verify_brake_state_revision.py`（20场景10685项）、`verify_forward_speed_smooth.py`（11场景1840项）、
  `verify_non_d_brake_compat.py`（220场景8925消息一致）。完整命令、日志和哈希见专项报告及证据目录。
- 同输入原版/新版C++复算、4901点静态曲率对照、绑定单核交错5轮性能测量完成；新增缓存预热后零堆分配，
  规划路径回调有预计算成本，完整尾部/内存数字已披露。ROS桩/x86结果不能替代Orin、载荷及实际液压标定。
- 改前快照`../control_d_smooth_20260922_103950/before/`，归档`../control_d_smooth_20260922_103950_before.tar.gz`；
  完整证据、部署三文件及回滚基线均在同名证据目录。需要重编robot并重启control，未执行车端部署。

## 2026-09-22：pudong_air 限速列平滑（相邻差≤0.01 锥形包络，第三轮定稿）

用户三轮口径（最终）：把 `path/pudong_air/` 下限速列平滑——含用户限速行（第四列<10）
的文件**整列**平滑（默认 10 行参与，形成进出限速区完整坡）；全默认文件逐字节不动；
平滑值 ≤ 用户原值、下限 0.5；**相邻两点速度差 ≤ 0.01，无跳变**（第二轮 15 行移动
平均的 ~0.5/行坡被用户判定太陡，已恢复重做）。

最终算法：锥形包络（前向+后向两遍线性扫描 = min_j(orig[j]+0.01×|i-j|)）——
不高于原值的最大 0.01/行 Lipschitz 函数：平台保持原值，所有台阶与 10↔限速区
过渡均为 0.01/行 精确线性坡（点距约 0.22m → 坡率约 0.045 m/s/m；10→2.5 坡约
750 行 ≈165m）。cargo_01_01 入口坡起于约行 669，出口 0.5 起 0.01/行 升至文件尾
约 4.6（行数不足以回到 10，属约束下最长坡）；cargo_02 出口余量足回到 10。

数据：限速仅 2 文件（cargo_01_01 行1419-1873 阶梯降、cargo_02 行1663-2327 V 型）；
其余 13 轨迹全默认 10 不动；left1/left2/right 多边形排除；目标文件 CRLF 保留。

改动：2 文件共 3299 行（cargo_01_01 1406、cargo_02 1893）。验证：字节级断言全过
（全部 ≤原值且 ≥0.5；**整列相邻差实测最大 0.01**；前三列/行数/行尾不变；全默认
文件与备份逐字节一致）；planning CSV 回归 `verify_map_speed_limit.py` 12 组 PASS
无 FAIL。脚本 /tmp/smooth_pudong_air_speed.py（含干跑模式，STEP 可调）。
生效方式：重启 planning（或重新下发任务触发重载），无需重编。备份：工程同级
`pudong_air_speed_smooth_before_20260922_102302.tar.gz`（始终为最初原始版）。

## 2026-09-22：按用户确认修复围栏迁移业务冲突（生产交付）

生产文件19个，完整相对路径见 [部署清单](../pnc_fence_fix_delivery_20260922/部署文件清单.txt)。
修改覆盖 task_plan core/node及新增task_fence.inc/fence_geometry.h；planning任务/周期/最终输出/参考采样及新增fence_guard.inc；
新增msg/task_fence_guard.msg并注册CMake；control三文件仅清注释/include/旧告警初值；hmi/ros_bridge.py接入新鲜告警。
同步模块README、专项测试和本延续文档。当前几何缓存16文件/20万点，任务组合20万点上限，热循环无文件读取。
实际task1全程越界、task222缺路径会拒绝并取消后续块；其余44/50组原任务字段一致。修改前后control算法保持。

验证命令（实际输出位于 `/tmp/pnc_fence_fix_20260922_092231_76mnve0t/`，持久日志在交付目录）：
- `python3 src/pnc/tests/planning/verify_map_speed_limit.py --output <work>/planning`：53单元/6节点；1201地图、6581闸机、2612末停、995感知、615起步安全、756旧起步移除、113控制交接；CSV解析104/升级79及消毒器版本通过。
- `python3 src/pnc/tests/task_fence/verify.py --build <work>/planning/build --output <work>/fence_final`：59项联调、原六组专项、core C++11通过。
- `verify_forward_brake.py`：C++11 control节点及13462项；`verify_brake_state_revision.py`：20场景9764项；`verify_curve_history.py`：17609项。
- `verify_non_d_brake_compat.py --baseline-control <work>/before/robot_control --output <work>/non_d`：220场景8925条消息一致。
- `python3 -B hmi/tests/test_fence_alarm.py` 通过；`node hmi/tests/frontend_test.js`：72通过/0失败。
- 604个既有保护数据/接口文件哈希无变化。实际117边围栏10,000次主机位姿检查均值约0.60μs，不代表Orin整链路。

修改前备份：`../pnc_fence_fix_before_20260922_092231.tar.gz`。
交付：[生产包与说明](../pnc_fence_fix_delivery_20260922/部署说明.md)、[验证结论](../pnc_fence_fix_delivery_20260922/验证结论.md)、final_audit.json。
本轮未部署，车端须完整重编robot并重启task_plan/path_plan/control/HMI，不能混用新旧节点。
围栏边界停车/失联/挂钩例外和截停取消作业仍需实车复测；不声称全车/托盘扫掠或制动距离已经验收。

## 2026-09-22：围栏迁移（control→task_plan）对抗校验：迁移无回归可签收，围栏防护不可签收

用户要求确认自做的围栏迁移两点。多 agent 对抗校验（4 视角→3 验证者多数表决→完备性
批评，29 agent/1106 次工具调用）：24 条发现去重 19 条，前 8 条对抗验证 4 确认/4 推翻，
47 项核对通过。覆盖者产出最强差分证据：旧（07:27 含围栏）/新（08:21 移除后）control
源码在围栏静默场景同夹具输出**逐字节一致**（launch_speed 462+290、curve_history 17609、
non_d_brake 8925、forward_brake 13100）；旧/新 task_plan core 对 12 个生产任务
（栏内）位级一致、缺围栏文件时 fail-open 与旧行为一致。

**结论 1（control 清除干净且零业务影响）：成立。** 07:27→08:21 src diff 纯围栏删除
（5 文件）；当前源码围栏符号零残留；alarmcmd=0 与 path/fence.csv 保留；栏内行为等价有
逐字节差分。等价域为"围栏静默"——运行时兜底（车身前角 2.3m/0.73m 连续监控、人工推行/
GNSS 漂移/执行偏离开栏急刹）整体消失属迁移本意，但该场景清单迁移时未枚举；迁移窗口同批
删除了 5 个测试文件的围栏场景（07:27 备份不含 tests，无 before-run 记录）。仅存
README:366"围栏…一并复位"文档残留一句（3/3 票确认，本次已修：删"围栏/"并注明迁往
task_plan）。

**结论 2（task_plan 增加成功且零影响）：消息契约层面成立，围栏生效层面不成立。**
tp_before→当前纯增量 diff；task_plan_core 保持零 ROS 依赖、零磁盘写；117 顶点/AABB/
go_straight 截断至第 365 点均复现。但确认 2 条 major（3/3 票，真实代码+真实数据复现）：

1. **全栏外任务放行**（task_plan_core.cpp:821）：终点前所有点在栏外时仅 printf 警告、
   任务原样外发。**现网 param/task1.yaml（pudong/path_01，119 点全部在栏外）正命中该
   分支**——迁移前 control 的 20Hz FenceAlarm 会在车身出栏时停车，迁移后无任何拦截。
2. **首点出栏前缀放行**（:809）：路径首点在栏外、后续返栏时截断点取 last_in_idx
   （近末端），pathList 不变，车辆仍须驶过开头整段栏外轨迹；与"离开围栏前最后一个在
   内点"口径不符（该点对首点出栏路径不存在）。
3. [minor] LoadFenceFile 坏行静默跳过致围栏无声变形仍报"加载成功"（旧 control 遇
   坏行整文件拒载——健壮性口径回退）。

未判定但直接相关的遗留（11 条未过票数上限）：中段"出栏又返栏"截断取最后在内点放过
中段离开（major 级，:806）；**截断对 FMS/HMI 完全不可见**（1000 号远程指车实测 stop 被
改写而云端收"正常完成"回执，:815）；workflow.md task_plan 条目缺验证命令/文件清单/
备份引用且 test_task_plan_fence.cpp 未注册进 CMake/compile_all（手工编译命令未记录）；
"1000 次校验 0.13ms"未复现（O(N) 扫描，5290 点路径实测 62ms/千次）；monitor/fence/
新增的 fence_bak_apk.csv 使 monitor 按并集渲染出约 700m² "显示安全但业务会截停"区域
（fence.csv 已于 08:41 由用户同步为 path/fence.csv 同一文件，md5 一致，此前"两份围栏
不一致"发现随之消解）；运行期改 rosparam path_dir 后围栏不重载（:256）。

**待用户决策**：全栏外/首点出栏任务应拒发、告警还是放行（影响 task1.yaml 现网行为）；
截断是否需要跨模块可见信号（FMS 回执/HMI 显示）；中段出栏的正确截断口径。
证据：/tmp/wf_mig_result.json 与 workflow 转录
wf_2a074195-46d；对照备份 pnc_control_before_fence_removal_20260922_072700、
control_brake_state_before_20260922_082147、pnc_task_plan_before_fence_20260922_081000。

## 2026-09-22：monitor 电子围栏数据对齐与三态视觉语义升级

用户更新了 `monitor/fence/fence.csv` 并采纳此前关于消除视觉认知歧义的全部建议，已完成数据校验与业务代码实现。

### 1. 围栏数据一致性与业务轨迹校验
- **逐字节哈希校验**：
  比对 `monitor/fence/fence.csv` 与 `src/pnc/path/fence.csv`，二者二进制 SHA-256 均为 `c3dd8ffebbfbfcb5f78b1cfb6e5ba2451f2ce5f569b9aebbbff34fef94c6c06b`，顶点数均为 118 点（117 条闭合多边形边），逐行逐字节完全一致。
- **航站区现役作业轨迹实测**：
  使用更新后的围栏对三条机坪关键任务轨迹进行逐点射线法几何判定：
  - `pudong_air/312_cargo_01.csv`（2,332 点）：在栏率由 8.1% 恢复至 **100.0%**；
  - `pudong_air/312_charge_01.csv`（5,290 点）：在栏率由 41.5% 恢复至 **100.0%**；
  - `pudong_air/312_316_01.csv`（1,555 点）：在栏率由 43.1% 恢复至 **100.0%**；
  此前因 `monitor` 曾放入 29 点旧围栏导致正常自动行驶大面积误报红框的问题得到彻底根治。

### 2. 三态车框视觉语义改造（`static/index.html`）
- **痛点解决**：
  此前实现中，自车进入外扩 2.5m 斜线黄色警示带即变红车框，但在人机交互视觉上，黄色警示带代表“缓冲预警”，一旦直接变红会给操作员造成“已发生严重越界”的错误认知。
- **三态颜色映射**：
  - `fenceState(x, y)` 返回数值状态：
    - `0`（安全态 Safe）：定位中心落在任一围栏本体内，车体轮廓为黄色（`0xffff00` / `#ffff00`）；
    - `1`（预警态 Warning）：定位中心越出本体红线但落在外扩 2.5m 斜线缓冲带内，车体轮廓变为橙色（`0xff8c00` / `#ff8c00`）；
    - `2`（违规报警态 Violation）：定位中心完全越出外扩 2.5m 红线，车体轮廓变为红色（`0xff0000` / `#ff0000`）；
  - 保留 `fenceViolation(x, y)` 兼容旧调用接口；
  - 2D Canvas 降级渲染同步使用 `FENCE_VEH_HEX[fenceState(v.x, v.y)]` 保持全平台表现一致；
  - 渲染性能优化：内部状态追踪 `vehFenceState = -1`，仅在三态跨越翻转时触发材质颜色赋值，避免 60FPS 渲染循环中频繁触发 GC。

### 3. 后端参数动态联动改造（`ros_visualizer.py` & `monitor_config.py`）
- `ros_visualizer.py` 中 `fence_payload()` 扩展：
  优先尝试通过 `rospy.get_param` 获取 `/robot/fencefile`，或拼接 `path_dir/fence.csv`；若 ROS 参数不存在或参数指向的文件不可读，则安全回退至 `$MON/fence` 目录扫描，实现 monitor 与 PNC 核心规划栈电子围栏配置的动态同源联动。

### 4. 自动化测试套件与全量回归
- `monitor/tests/frontend_test.js`：
  升级 3D 与 2D 围栏变色断言，覆盖“安全区(黄) -> 缓冲警示带(橙) -> 完全越界(红) -> 返回安全区(黄)”完整生命周期状态机，全部 127 项断言通过（127 passing, 0 failed）；
- `monitor/tests/test_full.py`：
  98 项后端全栈断言全数通过（98 passing, 0 failed）。

## 2026-09-22：control/D制动状态修复与连续不足确认

用户在两份旧bag辨识/源码复核后确认修改建议并授权生产实施。未改变R控制计算及输出，不涉及planning/canbus业务。

业务修改三个文件：`src/pnc/src/robot_control/control_comply.cpp/.h`、`forward_brake_control.inc`。
新增`tests/control/brake_state_revision.cpp`、`verify_brake_state_revision.py`、`brake_state_revision_20260922.md`；
更新既有`forward_brake_feedback.cpp`中取消请求的过时期望，拆分未确认互锁与真实不足故障两种用例。
control与测试README、PNC近期说明和09-21旧文档入口同步。未顺手改旧横向/几何/终点、节点接线或参数。

关键修复：旧释放周期与新请求分开；自动非D只登记已发布的制动来源供D接管；观察期回跳保留下降请求重建时基；
普通连续不足以新估计确认；小请求未知、强请求失效、实际释放失败分别处理。独立审查另发现并修复迟到短响应
洗掉失败证据、反复强请求延迟超时、短断流把普通请求误归强，以及R小请求进入D后误归强四个边界。
没有将“零反馈”当作所有迟到请求都已完成，1.8s保护与既有手动D恢复条件保留。

验证命令（工程根）：

```bash
python3 -B src/pnc/tests/control/verify_brake_state_revision.py --output /tmp/control_brake_state_revision_final_20260922
python3 -B src/pnc/tests/control/verify_forward_brake.py --output /tmp/control_brake_state_20260922_082147/compat_forward_final
python3 -B src/pnc/tests/control/verify_curve_history.py --control-build /tmp/control_brake_state_20260922_082147/compat_forward_final --baseline-control /tmp/control_brake_state_20260922_082147/before/src/pnc/src/robot_control --output /tmp/control_brake_state_20260922_082147/compat_curve_final
python3 -B src/pnc/tests/control/verify_non_d_brake_compat.py --baseline-control /tmp/control_brake_state_20260922_082147/before/src/pnc/src/robot_control --output /tmp/control_brake_state_20260922_082147/compat_non_d_final
```

结果：真实C++11节点编译/链接通过；20个新公开入口场景9764断言、D反馈13462项；独立复核15场景36断言。
曲率17609项通过、4901点逐值一致、96动态窗口，直线噪声及5m/s指令保持。R/N完整对照220场景8925条消息
零差异（R4965、N3960，含48个D→非D场景）。最终源码哈希与构建/复核记录一致；VehicleControl、
曲率/预瞄/抗噪、横向、TerminalStopBrake函数文本与本轮改前完全一致。

复杂度：新增状态与每次协调均O(1)，主机ABI状态224→288字节、对象1496→1560字节。
五次交错微基准测D状态路径，P99增量0.006～0.018μs，分配数未增加；不含完整控制/几何/ROS调度，
不等于Orin实测。微基准为第二版D逻辑，最终版仅追加非D来源标签，所测D函数未再改动。

备份：[control_brake_state_before_20260922_082147.tar.gz](../control_brake_state_before_20260922_082147.tar.gz)。
持久证据：[control_brake_state_evidence_20260922_082147.tar.gz](../control_brake_state_evidence_20260922_082147.tar.gz)，
[最终审计与差异](../control_brake_state_evidence_20260922_082147/final_audit.json)。
当前完整行为与恢复操作见[说明](src/pnc/tests/control/brake_state_revision_20260922.md)。
未部署车辆，需同步三个业务文件、重编robot并重启control；3%有效性、15%能力、退出后剩余降速及5m/s紧急制动仍待实车标定/验收。

## 2026-09-22：电子围栏改版——警示带向内 2m 改向外 2.5m

用户确认两项口径（闯入判定基准=自车定位中心点；任一围栏内=安全），并要求把
"向内延伸 2 米"改为"向外延伸 2.5 米"：2.5=车头前伸 2.3+0.2 余量，使中心在栏内时
车头伸出的部分仍落在外扩红线之内——显示视觉上整车未超出电子围栏、不变红；
中心越出围栏本体（进入栏外斜线警示带或更远）才变红。

修改文件 5 份（无新增文件，无 C++/消息改动）：

- `monitor/ros_visualizer.py`：`FENCE_INSET_METERS=2.0` → `FENCE_OUTSET_METERS=2.5`；
  `inner_offset_polygon` 参数化为 `offset_polygon(pts, dist, inward)`（外扩=法线取反，
  miter 公式/限幅共用）；退化检测双向化 `_fence_offset_degenerate`（符号翻转 +
  方向化包含检查 + **偏移多边形自交/自搭接检测** `_segs_touch`——外扩窄凹槽形成指状
  自重叠，顶点包含检查对它不敏感，自搭接检测捕获；O(N²) 仅加载期一次）；
  payload 字段 inner/outer 改为 fence（本体=判定多边形）/outer（外扩=纯显示）。
- `monitor/static/index.html`：buildFences 画本体红线+外扩红线，警示带=外扩挖本体洞
  （Shape/Path2D 同构）；fenceViolation 判定改用 fence 本体多边形（不再依赖偏移结果，
  偏移退化只影响显示）；2D 同步。
- `monitor/monitor_config.py`：注释口径同步（外扩 2.5m、判定基准）。
- `monitor/tests/test_full.py`：t01 期望值改外扩几何（方环 15x15/三角手算值
  (17.5,-2.5),(17.5,16.04),(36.04,-2.5)）；t12 重写双向单位几何+退化矩阵
  （窄走廊内缩翻转/窄凹槽外扩自交/开口4m 外扩1.5m 不误伤/方环双向干净）+
  payload 用例（凸走廊 fence/outer 各 4 点、notch outer 停用本体照常、BOM 保持）。
- `monitor/tests/frontend_test.js`：夹具改 fence/outer 方形±20/±22.5；变色三态断言
  改"中心越出本体进入警示带(21,0)/完全越出(40,0)/回栏恢复"；警示带洞=本体断言。

关键发现：**真实 fence.csv 的 2m 内缩在西北角本来就会自相交**（1.47m 短边夹在长边
间，偏移边对穿越——昨日"顶点包含"检查看不到边穿越，对抗校验轮也未覆盖该半边）；
改为外扩 2.5m 后该角干净，且判定不再依赖偏移结果，结构上消除了这类风险。

验证（本机无 ROS）：

```bash
python3 tests/test_full.py      # 98 项(几何/退化矩阵/payload/BOM 共 12 项围栏断言)
node tests/frontend_test.js     # 126 项(判定三态/2D 降级同步改外扩口径)
```

外加服务端几何回归矩阵 9 组（方环/CW 三角双向精确、U 形窄凹槽 2.5 检出 1.5 不误伤、
3m 走廊内缩翻转/外扩干净、真实 fence 外扩干净）、真实 fence.csv 外扩 payload 冒烟
（fence/outer 各 29 点、坐标减 origin）。0 失败。
备份：工程同级 `monitor_fence_outset_before_20260922_080554.tar.gz`。
未实车/未在车载 Firefox 实测。

## 2026-09-22：control 模块清除电子围栏逻辑与全量回归

- **背景与需求**：
  用户要求：“清除掉control模块中关于电子围栏部分的代码，这一部分现在不需要放到control模块中了，注意清理的时候千万不要影响到任何其他的业务逻辑。”
- **清理范围与具体修改**：
  1. `src/pnc/src/robot_control/control_comply.h`：
     - 移除 `LoadPathFile(std::string tPath)`、`FenceAlarm()`、`local2global2(...)` 3 个成员函数声明；
     - 移除 `std::vector<XYZ_COOR_S> mFenceList;` 和 `int FenceWarning;` 2 个成员变量。
  2. `src/pnc/src/robot_control/control_comply.cpp`：
     - 移除 `LoadPathFile`、`pointInPolygon`、`local2global2`、`FenceAlarm` 4 个函数的完整实现（约120行）；
     - 移除 `VehicleControl()` 中的 `if(FenceWarning == 1)` 停车制动块（紧随其后的冲出跑道偏差 >5.5m 停车逻辑完整保留）；
     - 移除 `TerminalStopBrake()` 中 `normal_stop` 判据的 `FenceWarning == 0 &&` 条件。
  3. `src/pnc/src/robot_control/control_node.cpp`：
     - 移除节点启动时的 `controlComply.LoadPathFile("fence");` 调用；
     - 移除 20Hz 主循环中的 `controlComply.FenceAlarm();` 周期调用；
     - 保留启动时的 `ros::param::set("alarmcmd", 0);` 初始值，确保 HMI 轮询兼容。
  4. `src/pnc/src/robot_control/forward_brake_control.inc`：
     - 移除 `ForwardTerminalStopBrake()` 中 `normal` 判据的 `FenceWarning == 0 &&` 条件。
  5. 地图数据文件保护：
     - `src/pnc/path/fence.csv` 资产完整保留，严禁删除。
  6. 文档更新：
     - `src/pnc/src/robot_control/README.md`：移除围栏 CSV 读取与调用时序章节，更新数据流图，注明电子围栏已移出 control 模块；
     - `src/pnc/path/README.md`：更新 `fence.csv` 说明，注明不再由 control 节点读取或执行停车。
  7. 测试夹具同步更新：
     - `tests/control/launch_speed.cpp`、`tests/control/curve_history.cpp`、`tests/control/non_d_brake_compat.cpp`、`tests/control/verify.py`、`tests/control/verify_non_d_brake_compat.py` 移除对已删除围栏接口的调用与 `fence` 测试场景；
     - `tests/planning/terminal_control.cpp` 设置 `can.controlPanelState = 1` 并在停稳后推进模拟时钟，确保符合 D 挡末端反馈制动条件。
- **自动化测试与回归验证结论（全部 PASS）**：
  - `verify_forward_brake.py`：C++11 节点桩编译链接，13,100 项前向制动断言全数 PASS；
  - `verify_curve_history.py`：17,609 项 D 挡曲率历史与 4 秒动态预瞄断言全数 PASS；
  - `verify_non_d_brake_compat.py`：220 个测试场景、8,925 条完整非 D 消息与改前基线逐字段 100% 一致；
  - `verify_map_speed_limit.py`：53 个 PNC 编译单元、6 个节点链接、1,201 项限速、6,581 项闸机、2,612 项末停、995 项感知、615 项起步、756 项观察、113 项 terminal_control 规划与控制交接断言全数 PASS。
- **备份快照**：
  - `pnc_control_before_fence_removal_20260922_072700.tar.gz`

## 2026-09-22：monitor 电子围栏图层与自车闯入变色

用户要求（先复述确认，3 个默认取法经用户同意：闯入变色不受图层勾选影响、完全越出
外围栏同样红、图层默认勾选）＋修订：围栏数据放 `monitor/fence/`（用户已建，含
fence.csv 29 点），该目录全部 csv 按电子围栏加载，与普通轨迹地图 `map/` 分离避免混淆。

修改文件共 8 份（含 2 份新增 fixture，无 C++/消息改动）：

- 服务端 3 份：`monitor/ros_visualizer.py`（新增 `load_fence_polygon`/`inner_offset_polygon`
  与 `fence_payload()`：目录缺失/坏文件/顶点<3 跳过，坐标减地图 origin 缓存一次；
  miter 角平分线内缩 2m，有向面积定绕向，尖角限幅 4×d，凹多边形窄颈可能自交——仅
  显示与闯入判定用，不参与安全停车）；`monitor/monitor_config.py`（FENCE_PATH=$MON/fence、
  LAYERS.fence=True）；`monitor/monitor_server.py`（/api/map 附带 fences）。
- 前端 1 份：`monitor/static/index.html`（LAYER_NAMES+面板"电子围栏"项；3D：fenceGroup
  注册图层、外/内红线 Line、缓冲带 Shape 挖洞+32px 斜纹 CanvasTexture（2m 瓦片）、
  rotation.x=-90° 落地；2D：evenodd 环带 16px 斜纹 pattern+红线；MM.pointInPoly 射线法；
  tick/render2d 中自车越出内缩多边形（任一围栏内=安全，否则含完全越出=红）黄框变红，
  仅状态翻转时改材质色）。heading 列不参与几何。
- 测试 3 份：`tests/frontend_test.js`（桩补 Shape/ShapeGeometry/Vector2/Path/RepeatWrapping/
  DoubleSide/Path2D/createPattern/纹理 repeat；图层 10→11 项；新增 PIP/3D 几何/图层开关联动/
  闯入变色三态（带内红-越出红-回内黄）/2D 降级环带填充与变色断言）；`tests/test_full.py`
  （t01 增围栏 HTTP 断言：双围栏有序/坏行跳过/origin 偏移/方环内缩精确/CW 三角 miter；
  新增 t12 目录缺失/顶点不足/缓存/内缩单位几何）；`tests/ros_test_config.py`（FENCE_PATH
  指向 fixture）。
- fixture 2 份新增：`tests/fence_fixture/f1.csv`（方环含坏行/nan/空行）、`f2.csv`（CW 三角）。
- 文档 2 份：`monitor/README.md`（最近变化/语义对照/CPU 口径）、`workflow.md`（本节）。

验证（本机无 ROS，伪 ROS 全栈 + 无头前端）：

```bash
python3 tests/test_full.py      # 89 项(含围栏 HTTP/几何/边界 9 项新增)
node tests/frontend_test.js     # 126 项(含围栏 3D/2D/PIP/变色 23 项新增)
python3 tests/edge_test.py      # 28 项
node tests/frontend_dash_test.js # 51 项
python3 tests/chaos_test.py     # 11 项
```

全部 0 失败。真实 fence.csv（29 点）实测：内缩点到邻边距离≈2m（min 1.99，尖角/短边处
最大 2.77 属 miter 几何特性），全部内缩点在原多边形内，payload 坐标与地图同 origin。
性能：服务端加载/内缩各一次 O(N) 缓存；前端每帧 PIP O(N)（29 点≈60 次比较），斜纹纹理
全局一张；无新订阅/新拉取。边界：本机未接真 ROS master、未开浏览器实测视觉效果；
凹多边形内缩可能自交按原样画；围栏改动需重启 monitor（与地图一致）。
备份：工程同级 `monitor_fence_before_20260922_000046.tar.gz`（9 文件+哈希清单）。

## 2026-09-22：电子围栏对抗校验轮（4 确认/4 误报/53 项核对）与修正

用户要求校验上节改动。多 agent 对抗式校验（4 视角→3 验证者多数表决→完备性批评，
29 agent/643 次工具调用；1 个验证者因 API 限流失败，不影响表决）。10 条发现去重 9 条，
4 条确认、4 条被对抗推翻（0.1m 去重属已文档化约定且真实数据最小间距 1.39m 不触发；
3D 外推/2D 快照变色口径差异在匀速下数学上不构成锯齿；Path2D 缺失组合在真实浏览器
为空集；DoubleSide 经真 three 实测为承重正确选择非冗余）、1 条 info 未判定（inner 为空
数组时前端恒判闯入，当前后端契约不可达的防御路径）。执行覆盖：内缩几何性质 831 项
合成断言、真实服务冒烟、**开发机 Chromium 真浏览器+真实 three r147 实证建栏/闯入变色
三态/2D 降级**（上节"未浏览器实测"边界已被此证据超越）。

确认问题与本轮修正（备份：`monitor_fence_verify_before_20260922_075157.tar.gz`）：

1. **[已修] 围栏 csv BOM 首点丢失**：`load_fence_polygon` 原以 utf-8 打开，Excel
   "CSV UTF-8" 导出带 BOM 会使首行 x 解析失败而静默丢第一个顶点（多边形闭合边悄悄
   变形仍照常渲染）。改 `utf-8-sig`（无 BOM 文件行为不变），t12 补 BOM 回归。
   真实 fence.csv 首字节无 BOM 未受影响；`load_map_lines` 同款 open 为既有口径未动。
2. **[已修] 窄围栏内缩翻转误判安全**：宽度<2×内缩距（4m）的走廊/窄颈，miter 内缩
   整体翻转后"中间条带"会被误判为未闯入。新增 `_fence_inner_degenerate`（绕向符号
   + 内缩点越出外环双重检测，O(N²) 仅加载期一次），退化时停用 inner——该宽度下整栏
   几何上都在边界 2m 内，全部按缓冲带（红）恰是正确语义；前端对空 inner 已有守卫
   （跳过内线/斜纹，判定全红）。t12 补 3m 走廊用例。
3. **[已修] workflow 计数**：前端新增断言实为 23 项（原记 18 项低估），已更正。
4. **[仅记录] chaos_test 偶发 flake**："风暴后 master_ok 恢复"3 次实跑 1 败 2 过，
   该文件本轮零改动零围栏引用，属既有 master 重连时序抖动，不属本改动回归。

修正后回归：test_full **92/92**（+3：BOM 首点/窄走廊 inner 停用/BOM 围栏完整）、
frontend 126/126、0 失败。

待用户口径确认（当前实现自选，均已写入 README/基线）：

- **闯入判定基准点=自车定位中心点**：车头前伸 2.3m>2m 缓冲带，黄框视觉上跨线但
  中心未越内缩线时不变红；换车身外廓判定需另提（几何上需车辆轮廓四角 PIP）。
- **多围栏并集语义**："在任一围栏 inner 内=安全"，两个围栏之间的外部空地恒判闯入
  （转移全程红）。当前仅 1 个 fence.csv 不触发；多 csv 上线前建议确认。
- 未验证残留：/robot/mapfile 显式单文件模式与实车数据流仅有代码级证据；真 fence.csv
  内缩距统计为单方复测（inner_offset_polygon 已被 831 项性质测试钉死，风险低）；
  窄围栏翻转输入已在服务端拦截，但 3D earcut/2D evenodd 对自交多边形的渲染行为
  无用例（拦截后不可达）。

## 2026-09-21：仅control/D挡反馈制动实施

用户授权按已讨论方案实施，开始前已复述需求与详细计划；明确不能修改其他模块业务或R控制。
开工快照：`/home/mothotob/work/projects/claude-code/control_d_brake_before_20260921_233958.tar.gz`。
备份包含当时整个robot_control、control测试和workflow。工作区无git，本机无ROS1。

业务改动只有 `src/pnc/src/robot_control/control_comply.cpp/.h` 与新增
`forward_brake_control.inc`。新增D导航减速度估计、普通补刹协调、真实液压在途/反馈/卸压互锁、
低速末停连续建压及故障保护；最终D输出油刹互斥，所有早返回的制动都会记录执行记忆。
实际反馈为 `/can_msg.brakePercent` 非零，而不是control/can_comm请求；CAN速度仍不用。
独立安全制动不受观察期/积分/普通上限限制，D围栏70不得覆盖已有100。
普通首3%，接入后每0.5s加1/减2、上限15；默认积分饱和仍0.2m，仅planning原始desireSpeed下降触发。
当前4秒曲率预瞄、公式、抗噪及D/R横向源码保持。R/N全部控制字段和发布顺序保持。

实车数据复核否定当前采用单一延迟一阶电机逆模型或液压百分比→减速度表的可靠性；
因此用实测减速度闭环，所有新增舒适性/时延参数注明为工程初值。
制动退出守卫1.8s可能延长零电机给定、造成欠速；硬限速突降仍要求立即降低电机目标，
不能同时宣称任意目标突降下的整车jerk受限。bag最高3.1643m/s，不覆盖5m/s实际制动验收。
分析证据保存在工程外 `rosbag_analysis_20260921_154420_283/implementation_model_20260921/`。

新增测试 `forward_brake_feedback.cpp` / `verify_forward_brake.py`、
`non_d_brake_compat.cpp` / `verify_non_d_brake_compat.py`；更新curve_history独立夹具，
独立用例重建对象，不继承先前安全停车在途状态/回跳时钟。真实切换序列另有D与非D专项，
没有放松曲率数值断言。旧verify.py/brake_request/launch_speed保留历史补刹断言，
不能作为现行反馈控制全通过入口，也未宣称旧套件通过。

验证命令及结果：

```bash
python3 -B src/pnc/tests/control/verify_forward_brake.py --output /tmp/pnc_forward_brake_review
python3 -B src/pnc/tests/control/verify_curve_history.py --control-build /tmp/pnc_forward_brake_review --output /tmp/control_d_brake_20260921_233958/curve_final --baseline-control /tmp/control_d_brake_20260921_233958/before/src/pnc/src/robot_control --benchmark
python3 -B src/pnc/tests/control/verify_non_d_brake_compat.py --baseline-control /tmp/control_d_brake_20260921_233958/before/src/pnc/src/robot_control --output /tmp/control_d_brake_20260921_233958/non_d_compat --compare-after
```

真实control_node C++11桩编译/链接，13100项D输入/发布断言通过；17611项曲率/预瞄/噪声/5m/s检查，
4901个标定点和96组动态窗口对照通过；非D冻结基线224场景9136条消息、48组D切R/N完整一致。
D专项包括迟到液压、撤销后尚未接入、连续新反馈确认、目标小涨、复位不清执行状态、
手动指令不登记、回跳时钟重建、短暂断流不叠急刹、反馈有码而减速无效、15%台阶延迟1.7s不误故障。
节点对象1296→1520 bytes，新增固定224 bytes；新导航/CAN处理和周期协调O(1)，无新路径扫描或容器。
曲率微基准128/1024点、0/1.5/3/5m/s均零分配，1024点5m/s中位0.631→0.629us；
该基准不衡量新增液压协调或车端CPU，未以此宣称整车更快。

control README、测试README和[本轮说明](src/pnc/tests/control/brake_feedback_20260921.md)已同步。
本轮没有部署或实车验收。上线需三个业务文件一起同步、重编robot并重启control；
故障手动恢复具体要求为**手动D**停稳且连续新鲜零制动反馈0.5s，切R/N不清D故障。
canbus-vehicle-1/3的急停100→80覆盖、刹车清转角和can_comm旧指令重复问题在用户范围之外，未修改。

## 2026-09-21：canbus托盘反向标定212/115与反馈122未到位

用户给出 `pallet_position_min=212`、`pallet_position_max=115`，并反馈实际位置122、
`palletStatus=0`。旧LoadPositionConfig要求两对端点均数值递增，会拒绝212/115并整对回退
213/254；节点随后仍发布默认值，因此配置校验失败与rosparam键不存在是两件事。
改动前工程配置为212/231，间距19，同样未达到原有20的最小间距。

本轮修改：

- `src/canbus/src/canbus_node.cpp`：只改启动解析/校验。pallet的min/max分别保留上/下端语义，
  各自限定0～255，按绝对差≥20校验，不排序。反向允许0后，原atoi会将非法文本误当0；
  因此pallet键改为完整整数校验，支持空白与行尾#注释，拒绝无数字、溢出、尾随杂字等。
  缺失/坏值仍整对回退；重复键最后一条非法时也拒绝该对。hook解析、校验、内置默认均保持。
- `src/canbus/config.cfg`：pallet上端212、下端115；hook178/252保持，同步说明端点语义。
- 新增 `src/canbus/tests/verify_position_config.py`：从真实.msg生成字段桩，复制实际node源码
  逐字进行C++11编译；ROS和Comply用无I/O桩隔离，执行真实加载、main参数发布和T1回调。
- 同步 `src/canbus/README.md` 与本workflow。T1/T2、CAN协议、消息、挂脱钩联锁、PNC未改。

反馈122的结论：即使反向配置成功加载，`abs(122-115)=7` 也不满足 `<5`，连续等待仍为0。
现标定下上端区间208～216、下端111～119；连续11次T1命中才确认为4/3，离开立即清零。
已向用户询问122是否为实际最低端稳定反馈；尚无确认，保留115，不擅自扩大到位范围。

验证命令（工程根）：`python3 src/canbus/tests/verify_position_config.py`。
结果：C++11编译通过；34组配置/启动用例通过（含正常/反向/间距边界/越界/非法文本/缺键/
重复键/注释/默认回退/参数发布）；6422项T1断言通过（正反向全0～255位置扫描、11次计数、
离带重计、长时间保持、两端切换、122不误判、hook端点与中段堵转）。main桩确认T1=0.1s、
T2=0.05s、主循环100Hz。提取备份原node以同一脚本 `--source` 编译，首个212/115用例按预期
失败并回退213/254，证明专项覆盖原问题。测试不验证CAN编解码、ROS运行或机械端点。

性能：N为配置文本长度，启动解析前后均O(N)时间、O(1)辅助空间；新增严格解析仅启动运行，
周期回调逐字保持，没有新缓存或周期分配；未测车端CPU，不作性能提升结论。
修改前快照：工程同级 `canbus_pallet_reverse_before_20260921_193753.tar.gz`（node、config、
README、workflow）。部署需同步源码和配置，`catkin_make --pkg canbus -j4` 后重启已有canbus，
避免重复启动CAN栈；以 `rosparam get /canbus/palletposition` 核对加载值。未部署或实车验证。

## 2026-09-21：自定义指令卡片对抗校验轮（8 项确认/0 项误报）与修正

用户要求校验上一节改动的错误、逻辑问题及对 HMI 其他业务的影响。多 agent 对抗式校验
（4 视角审查→每条发现 3 视角反驳多数表决→完备性批评，29 agent/564 次工具调用）：
10 条发现去重 8 条全部确认、0 条被推翻；前端渲染/车辆面板/录制/标定卡/自启服务/订阅重建
等 53 项核对无波及。执行者另以 mock master 端到端驱动真实配置 nodes 健康检查、伪造进程
实测 pgrep 语义、组3 编排推演共 61 项断言通过。

确认问题与本轮修正（备份：工程同级 `hmi_ultra_card_verify_before_20260921_181716.tar.gz`）：

1. **[已修] pnc 卡标题"5节点"错误**：control.launch 5 个 include 实际含 6 个 `<node>`
   标签（robot_canbus.launch 含 perception_msg_convert+can_comm_node 两节点）；旧"6节点"
   在捆绑 ultra 时期实际 7 个，同为差一。标题改回"规划控制(6节点)"（这次与实数一致），
   hmi_config 内注明计数口径，4 处文档同步修正。
2. **[已修] stop_pat 只匹配 .launch，节点孤儿不可见**：roslaunch 被 SIGKILL/OOM 单击杀后
   ultra_command_node 孤儿（cmdline 不含 .launch）对停止残留检测/外部识别/CRASHED 重启
   清理三处均不可见，重启后新旧双节点竞态双写 `/ultra/status/safe`。stop_pat 扩为
   `ultra_command\.launch|ultra_command_nod[e]`（centerpoint 卡同款，[e] 防 shell 自匹配）。
3. **[已修] hmi/README 部署缺口**：通用指引"部署=拷 hmi/"对本变更是陷阱——车上旧
   control.launch 捆绑实例会与卡片双起同名节点互杀。README 补部署耦合警示（必须连
   launch/control.launch、start_l4.sh 一起同步）。
4. **[已修] 新可达状态未对账**：解绑后"只启 pnc、自定义指令卡从未启动"首次经 HMI 可达，
   此时参数不存在、planning 缺省 0 不停车，矩形监控静默缺失且无告警（与"停卡→safe=1→
   停车倒逼注意"不同）。README 与 ultra README 补边界说明：成套启动用一键启动。
5. **[新增防线] tests/test_config_sync.py**（4 项常驻断言）：配置 nodes pattern 集合 ==
   `_node_patterns()` 输出（漂移时组件 40s DEGRADED 中止一键启动且无日志指向根因）、
   组3 成员与顺序、ultra 卡无 optional/enabled+stop_pat 双覆盖、pnc 标题节点数 ==
   launch 递归展开实数。修正后回归：config_sync 4/4、centerpoint 3/3、录制 24/24、
   前端 72/72、全栈 80/80 全通过。
6. **[已修] 文档数字**：hmi/README 陈旧"66 项前端"改 72 项。

确认但未改的既有框架债（不属本轮引入，改动需另立任务）：

- **foreign 判定一次性**（process_manager detect_foreign 仅 HMI 启动时执行）：HMI 运行中
  由 start_l4.sh/手工拉起 ultra 再点一键启动仍会双实例互杀——对所有卡片同样成立，
  建议 start_component/_orchestrate_start 前置复查 detect_foreign。
- **nodes 健康可掩盖 respawn 快速崩溃循环**（5s 计数缓存+10s 降级门限 vs 1s respawn）：
  车辆侧 safe=1 停车兜底，README 已记边界。
- **_node_patterns 双处硬编码**：本次已同步且新测试守卫，根治方案为从 health_specs 派生
  （构造期一次，零轮询开销），留作后续独立小改。
- all_demo.launch 为指向外部工程的 XML 残片（全仓无消费者、本身解析失败），与本轮无关。
- 车载 git 副本（~/work/projects/github/qingwei-L4-pudong）未含本改动，两树正本/同步
  策略待用户定夺。

## 2026-09-21：ultra_command 目标监控路径更新

用户要求将 ultra_command 中的监控目标路径替换为：
- `pudong_air/312_316_01` 换成 `pudong_air/312_316`（触发 `ZONE_MODE_LEFT`，监测 `left1` + `left2`）
- `pudong_air/312_cargo_01` 换成 `pudong_air/312_cargo`（触发 `ZONE_MODE_LEFT`，监测 `left1` + `left2`）
- `pudong_air/312_charge_01` 换成 `pudong_air/312_charge`（触发 `ZONE_MODE_RIGHT`，监测 `right`）

### 修改文件清单（共7份，无新增依赖或数据结构）：
1. 业务逻辑（1份）：
   - `src/ultra_command/src/ultra_command_comply.cpp`：`SetTaskPlanPaths` 函数中精确匹配路径字符串替换，保持现行 `std::vector<std::string>` 遍历与首个命中即 `break` 逻辑，无新增容器与动态堆分配。
2. 接口与节点注释（2份）：
   - `src/ultra_command/src/ultra_command_comply.h`：更新文件头规格说明及 `SetTaskPlanPaths` 调用参数示例注释。
   - `src/ultra_command/src/ultra_command_node.cpp`：更新节点头部规格说明注释。
3. 模块元数据与说明文档（2份）：
   - `src/ultra_command/package.xml`：更新 `<description>` 字段中的目标路径说明。
   - `src/ultra_command/README.md`：在“最近核对与历史摘要”追加本轮路径更新记录；同步“功能（需求原文映射）”表格中的匹配路径；更新“编后核对”中的任务切换示例。
4. 测试夹具与断言（1份）：
   - `src/pnc/tests/navigation_speed/ultra.cpp`：新增针对 `pudong_air/312_316`、`pudong_air/312_cargo`（断言切换为 `ZONE_MODE_LEFT`）与 `pudong_air/312_charge`（断言切换为 `ZONE_MODE_RIGHT`）的测试断言；新增针对旧 `_01` 路径（`312_316_01`、`312_cargo_01`、`312_charge_01`）确认其不再激活监控（保持 `ZONE_MODE_NONE`）的反向测试断言。
5. 过程延续记录（1份）：
   - `workflow.md`：更新当前基线条目与本详细记录。

### 边界与保持项：
- **初始化监控保持原样**：`_01_01` 后缀变体（`pudong_air/312_316_01_01`、`pudong_air/312_cargo_01_01`、`pudong_air/312_charge_01_01`）及其基于 `/navigation_msg.gpsSpeed` 严格大于 0.5m/s 的单向起步闩锁逻辑完全保持原样。
- **地理围栏与算法保持原样**：`pudong_air/left1.csv`、`left2.csv`、`right.csv` 顶点数据、射线法判内算法（`IsPointInPolygon`）、1.0s 感知超时故障保护（`safe=1`）及普通模式三帧防抖确认机制完全保持原样。
- **性能与内存影响**：仅变更字符串字面量比较值，时间复杂度仍为 $O(N_{\text{paths}})$（典型 $N=1$），空间复杂度不变，零新增开销。

### 验证记录（本机无 ROS 环境，C++11 桩编译与断言）：
- 备份快照：`ultra_command_before_20260921_180300.tar.gz`。
- 编译与运行命令：
  ```bash
  g++ -std=c++11 -Wall -Wextra -I/tmp/nav_speed -I/tmp/nav_planning/build/stubs -Isrc/ultra_command/src \
      src/pnc/tests/navigation_speed/ultra.cpp src/ultra_command/src/ultra_command_comply.cpp -o /tmp/nav_speed/ultra
  /tmp/nav_speed/ultra /tmp/nav_speed/fixtures
  ```
- 结果输出：
  ```
  ====ultra_command==== 监控模式/任务切换: 0 -> 1 (task:0 path:pudong_air/312_316; 0=无 1=left 2=right 3=init-left 4=init-right)
  ====ultra_command==== 监控模式/任务切换: 0 -> 1 (task:0 path:pudong_air/312_cargo; 0=无 1=left 2=right 3=init-left 4=init-right)
  ====ultra_command==== 监控模式/任务切换: 0 -> 2 (task:0 path:pudong_air/312_charge; 0=无 1=left 2=right 3=init-left 4=init-right)
  PASS navigation ultra: 35 checks
  ```
  原有 29 项导航车速断言 + 新增 6 项目标模式匹配与旧路径排除断言全部通过。

### 车端部署注意：
需在车端执行 `catkin_make --pkg ultra_command -j4`，并在 HMI 组 3（规划控制组）重启“自定义指令”卡片（或重新运行 `./start_l4.sh`）生效。

## 2026-09-21：HMI“自定义指令”卡片独立启停 ultra_command

用户要求在 HMI“规划控制”组增加“自定义指令”标签页（形式同其他组件卡），用于启动
ultra_command 模块，并纳入“一键启动”。原 ultra_command 由 `launch/control.launch`
include 捆绑启动（与 pnc 同卡）；若直接加卡会与捆绑实例同名节点互杀（respawn=true 下
反复拉起），故先解绑再独立成卡。

修改文件共7份（无新增工程文件，无 C++/消息/CSV 改动）：

- 启动编排4份：`launch/control.launch`（移除 ultra_command include，留注释）；
  `start_l4.sh`（control 终端后补 `roslaunch ultra_command ultra_command.launch`
  独立终端行，救急路径覆盖保持）；`hmi/hmi_config.py`（组3 pnc 卡后新增
  ultra_command 卡：title“自定义指令”、非 optional、无 enabled 键即参与一键启动、
  健康=master 节点表计数 `/ultra_command_node` min 1、stop_pat 见下方校验轮修正；
  pnc 卡标题节点数口径修正见下方校验轮）；`hmi/ros_bridge.py`（`_node_patterns()` 增加
  `/ultra_command_node`，与配置 nodes 型规格同步）。
- 文档3份：`hmi/README.md`、`src/ultra_command/README.md`、`workflow.md`（本节）。

设计要点：ultra_command 不发话题，10Hz 写 rosparam `/ultra/status/safe`，健康检查
不能用话题频率，选 fms 同款 master 节点表计数（getSystemState 子串匹配，节点有
订阅必在表内）；launch 内 respawn=true 使进程崩溃由 roslaunch 1s 拉起。组内与 pnc
并行启动无碍：path_dir 参数缺失时节点每周期重试加载。planning 侧缺省 `ultra_safe=0`
（参数不存在不停车），与捆绑时期启动窗口行为一致。停止卡片时节点退出回调先写
`safe=1`，行驶中 planning 消费链将 desireSpeed 置 0——故障态停车属模块设计预期，
不是本轮引入的新语义。

验证（工程 hmi/ 目录，本机无 ROS）：

```bash
python3 -c "...load_config('hmi_config.py') 校验组3顺序/字段/占位符/nodes模式与 _node_patterns 同步..."
python3 tests/test_centerpoint_bundle.py && python3 -m unittest tests.test_record_rosbag
python3 tests/test_full.py        # 80项伪ROS全栈
node tests/frontend_test.js       # 72项前端无头
```

结果：control.launch XML 解析、start_l4.sh `bash -n` 通过；真实配置加载校验通过
（组3顺序 lidar_perception→pnc→ultra_command(自定义指令)→pnc_bags）；
centerpoint 3项、录制单测24项、全栈80项、前端72项全部通过，无新失败。
逐文件与改前备份 diff 仅含预期修改。车载生效：同步上述4份启动编排文件
（或整拷 hmi/ + launch/control.launch + start_l4.sh），无需重编任何节点。
备份：工程同级 `hmi_ultra_card_before_20260921_171300.tar.gz`；
改前哈希清单 `/tmp/hmi_ultra_card_20260921_171300_before.sha256`。
未部署车辆、未实车验证；卡片实际启停与一键启动链路待车端确认。

## 2026-09-21：清理9月10日以前的无用备份

用户要求识别并删除9月10日以前无用备份。按2026-09-10之前、不含当天处理，扫描工作区，
检查候选归档目录及精确文件名引用后，删除工作区根目录11个旧文件：8个tar.gz备份、
2个旧robot_perception_convert ZIP、1份workflow_full_backup_20260905.md；共3,216,679字节，约3.07 MiB。
相关引用仅在旧workflow副本及9月19日归档workflow中，无构建/运行/测试代码依赖。
9月5日baseline_sweep_reports分析报告仍被当前canbus README引用，保留；9月10日及以后备份也保留。

按明确名单逐项校验日期/类型/大小/SHA-256后unlink，未使用递归删除，也未另存一份旧备份抵消清理。
保留删除前哈希及清单于[本轮记录](docs/backup_cleanup_20260921.md)，详细元数据与删除结果在
/tmp/backup_cleanup_pre0910_20260921/。旧归档保持原文，其中已删除文件的引用不再代表备份可用。
现行代码、地图、参数、运行文件与新备份保持；另改本workflow并新增清单，本轮无需构建或重启。
用户对本次旧备份的直接删除要求优先于一般备份保护约定，不据此授权后续任意清理其他快照。

## 2026-09-21：曲率预瞄改为当前导航车速×4秒

用户明确要求control前6m曲率预瞄改为当前车速×4。业务修改control_comply.cpp：
preview_distance由6.0改为max(0,double(mVehicleSpeed))*4.0；速度来自/navigation_msg.gpsSpeed、单位m/s。
在原无效输入检查中补充速度有限性检查，并同步cpp/h预瞄注释。没有新增最小距离/人工最大距离；
1/3/5m/s分别4/12/20m，超过可用轨迹时仍止于真实末点。零速仍保留投影点曲率和原历史影响。
几何缓存独立于速度，因此导航速度改变后当周期调整扫描范围，无需新规划消息或几何重建。
曲率速度公式/1.4m/s参考标定、30周期历史释放、当前车速+0.5、补刹15及规划下降触发、横向/R保持。

沿用真实源码C++11节点桩编译/链接，曲率专项17611项、补刹16158项及18组积分时序/饱和通过；
R独立150条和D切R40条完整消息一致。同曲率4901点的上限与改前逐值相同。
新增动态窗口覆盖零速/负速/非有限值、亚米窗口、边界插值、8/20m边界、短路径、导航/CAN/规划速度不一致，
以及只更新导航后窗口扩大/缩小和30周期历史释放。96组跨版本输入对照，1.5m/s的12组与原6m完全一致。
原1cm扰动1800帧、44组左右转弯/掉头圆弧及直线3/4.5/5m/s继续通过，噪声容限未放宽。
旧6m空间用例显式设导航速度1.5m/s；不将原3m/s/6m假设继续当作新窗口的预期值。
3551帧实车记录回放，最近点、原横向曲率、路径点数保持；2503帧曲率限速因窗口/历史输入变化而改变。
88帧直线油门/刹车与改前相同，曲率上限6.326–7.513m/s；5m/s输出测试反馈是合成输入，未进行闭环实车测试。

```bash
python3 -B src/pnc/tests/control/verify.py --curve-preview --output /tmp/pnc_curve_preview_4s_20260921_161624/control --baseline-control /tmp/pnc_curve_preview_4s_20260921_161624/before/src/pnc/src/robot_control
python3 -B src/pnc/tests/control/verify_curve_history.py --control-build /tmp/pnc_curve_preview_4s_20260921_161624/control --output /tmp/control_preview_4s_benchmark --baseline-control /tmp/pnc_curve_preview_4s_20260921_161624/before/src/pnc/src/robot_control --benchmark
```

本轮已用首条构建产物直接运行第二条同等的80次基准测量，避免重复编译/回归；日志见benchmark.json与performance_result.json。
仍为O(P+30)扫描、O(N)几何重建；P是动态窗口内点数，最坏到N。没有新增缓存或周期内分配，实例1296 bytes。
相同1024点路径、5m/s下，主机5轮中位P50/P99由0.365/0.442增至1.024/1.155微秒，
最大值中位8.792→12.726微秒；这是6m扩为20m的必要扫描开销，不声称CPU零增量或车端时延保证。
测试修改curve_history.cpp、verify_curve_history.py，verify.py仅更新命令帮助；补刹夹具/算法未改。
同步PNC/控制/测试README、上一轮报告状态及workflow，新增[本轮报告](src/pnc/tests/control/curve_preview_4s_20260921.md)。
备份：工程同级pnc_curve_preview_4s_before_20260921_161624.tar.gz；
证据：/tmp/pnc_curve_preview_4s_20260921_161624/，持久包pnc_curve_preview_4s_evidence_20260921_161624.tar.gz。
planning点数/里程、地图/参数与其他业务未改。未部署车辆；同步源码重编robot并重启control，待实车复测。
旧主套件23项历史预期差异、短路径入口及极低速保底边界仍保留，不宣称旧全套通过。

## 2026-09-21：弯道限速取两版中值，参考1.4m/s，保留直线与曲率波动响应

后续用户要求改为当前车速×4秒预瞄，见上一节；本节公式表值继续有效，固定6m为当时记录。

用户反馈1.0m/s版对转弯/掉头限制偏低，要求介于该版与更早1.8m/s版之间，特别要求不影响
直线限速及规划曲率波动响应。业务仅改control_comply.cpp中turn_speed_ratio一行：
由1.0/1.8改为1.4/1.8；差异审计确认除此行及同行注释外，与改前文件逐字节一致。
现公式v_mid=v_base*(1-2/9*q)=(v_low+v_base)/2，q及所有阈值不变；
k<=0.01三版相同，k>=0.04为0.28/sqrt(k)，参考上限1.4m/s。
保留几何曲率估计、6m预瞄、30次历史、复位、导航+0.5及直线5m/s支持；
普通补刹仍规划desireSpeed下降触发、力度15、默认积分上限0.2m；横向、R及safety保持。

C++11控制节点桩编译/链接、17514项曲率和16158项补刹请求/力度通过，18组积分时序/饱和一致；
R独立150条和D切R40条完整消息一致。原1800帧1cm扰动、44组圆弧及直线5m/s测试继续通过，
噪声容限未放宽。三版真实源码4901点对照验证整段逐点中值，1001个直线保护区取值完全相同。
3551帧记录回放几何/历史字段三版一致，上限与两版均值最大差异2.385e-7m/s（float舍入）；
88帧匹配直线限速和油门/刹车完全相同，曲率上限6.326–8.180m/s。

```bash
python3 -B src/pnc/tests/control/verify.py --curve-preview --output /tmp/pnc_curve_turn_mid_20260921_155109/control --baseline-control /tmp/pnc_curve_turn_mid_20260921_155109/before/src/pnc/src/robot_control
python3 -B src/pnc/tests/control/verify_curve_history.py --control-build /tmp/pnc_curve_turn_mid_20260921_155109/control --output /tmp/pnc_curve_turn_mid_20260921_155109/final_curve --baseline-control /tmp/pnc_curve_turn_mid_20260921_155109/before/src/pnc/src/robot_control --upper-baseline-control /tmp/pnc_curve_turn_1m_20260921_150257/before/src/pnc/src/robot_control --benchmark
```

仅常量替换，无新增计算步骤/循环/分配/内存，周期仍O(P+30)、路径重建O(N)。
主机同输入5轮测量中位数：128点P50/P99为0.337/0.422→0.328/0.423微秒，1024点为
0.317/0.409→0.339/0.414微秒；最大值分别8.973→11.655和8.612→13.323微秒，有调度波动。
两版热态分配0次、实例1296 bytes、RSS18252 KiB，不将主机结果外推为车端时延保证。
测试修改curve_history.cpp、verify_curve_history.py，其他测试入口/补刹夹具未改；
同步PNC/控制/测试README、前次报告状态及workflow，新增[本轮报告](src/pnc/tests/control/curve_turn_midpoint_20260921.md)。
备份：工程同级pnc_curve_turn_mid_before_20260921_155109.tar.gz；
证据：/tmp/pnc_curve_turn_mid_20260921_155109/；持久包pnc_curve_turn_mid_evidence_20260921_155109.tar.gz。
本机未部署车辆；同步control_comply.cpp、重编robot并重启control后生效，折中标定待实车复测。
旧主套件23项历史预期差异、短路径入口及极低速保底边界均未扩改。

期间用户追问planning每次输出点数/米数：源码D挡最多60点，常规相邻采样至少0.3m、长度不固定；
现有88帧实车夹具每帧均60点，总折线路程26.273–26.633m。末端前方点不足时减少，
不足10点可补车后已有点供拟合；control随后重采样，6m限速窗口不等于整条planning轨迹长度。
本轮未改planning。

## 2026-09-21：参考右转1.8降至1.0m/s、保留直线5m/s、普通补刹系数降至30%

后续用户反馈本节1.0m/s版弯道偏慢，现行1.4m/s折中标定见上一节；下文保留当次交付记录。

用户明确确认前一版曲率抗噪修复已实车通过、直线符合要求；参考90度右转仍约1.8m/s，
要求1.0m/s并保持直线最大期望5m/s。随后反馈机械延迟下刹车过猛，要求减小补刹系数，
并追问积分饱和上限及其与刹车的关系。现有算法是积分门控+速度误差比例补刹；
按减轻力度的目标将50降至15，积分累计不乘0.3，避免另行延后介入。

业务仅改control_comply.cpp：保留现有基础曲线，乘以
`1-(4/9)*u^3*(10-15u+6u^2)`，`u=clamp((k-0.01)/0.03,0,1)`。
直线保护区逐值不变，k=0.04为1.0m/s，紧弯系数0.2；新倍率端点一、二阶导数为0，
全范围单调，原基础曲线与新过渡的一阶导数连续。0.04来自(0.36/1.8)^2的等效标定，
不将90度转角或参考实测速度当作直接测得的轨迹曲率。
原几何缓存、6m预瞄、30次释放、导航+0.5、D×13.5/5%保底、横向与R、独立safety保持。

普通补刹仍仅由规划desireSpeed下降建立请求，同值延续；到达规划目标、目标回升等原入口清除。
只改`dec*50`为`dec*15`，积分累计/门槛/时效/复位逻辑保持。积分默认超过0.1m接入、
0.2m封顶，饱和上限是运行门槛的2倍；单位m来自正速度误差的时间积分，不是刹车百分比或停车距离。
本机参数文件未设置该键，未连接车端读取运行覆盖值。力度按当前速度误差计算，
0.6m/s误差由30%改为9%；取整/保底/饱和和更强安全制动的优先级保持。

节点C++11桩编译/链接通过，17514项曲率/噪声/5m/s与16158项普通补刹请求/力度断言通过。
18组积分时序/饱和与改前一致；4901点曲线对照均不提高上限，直线区1001点逐值一致。
44组真实90度/180度、左右及11种半径圆弧符合表值（误差<1.5%）；
1cm相位扰动1800帧，直线5m/s指令稳定67%，急弯组波动<=约0.00225m/s，
新增过渡区组内总波动<2%、整数油门跨度<=1个百分点。R独立150条及D切R40条全消息不变。
3551帧记录重放中，匹配准确的88帧直线限速及输出与改前逐项相同，上限6.326–8.180m/s；
同组几何输入5m/s规划/反馈时稳定67%油门、0%刹车。该反馈是测试输入，非实车速度证据。

```bash
python3 -B src/pnc/tests/control/verify.py --curve-preview --output /tmp/pnc_curve_turn_1m_20260921_150257/control --baseline-control /tmp/pnc_curve_turn_1m_20260921_150257/before/src/pnc/src/robot_control
python3 -B src/pnc/tests/control/verify_curve_history.py --control-build /tmp/pnc_curve_turn_1m_20260921_150257/control --output /tmp/pnc_curve_turn_1m_20260921_150257/final_curve --baseline-control /tmp/pnc_curve_turn_1m_20260921_150257/before/src/pnc/src/robot_control --benchmark
```

本轮只新增常数算术，无新扫描、缓存、分配或类布局变化；总周期仍O(P+30)，路径重建仍O(N)。
主机同输入5轮对照：128点P50/P99为0.380/0.434→0.370/0.446微秒，1024点为
0.370/0.431→0.396/0.445微秒；最大值有调度波动，热态0次分配，实例仍1296 bytes。
不据此保证Orin实时性、机械响应或jerk。补充过渡区测试显示峰值曲率会使噪声曲线比理想圆弧
更慢，旧版也有偏置；不以修改旧急弯断言掩盖新增过渡区的这种输入差异。

测试修改curve_history.cpp、brake_request.cpp、verify.py、verify_curve_history.py；
同步PNC/控制/测试README、旧抗噪报告后续状态，新增[本轮验证报告](src/pnc/tests/control/curve_turn_1m_20260921.md)。
备份为工程同级pnc_curve_turn_before_20260921_150257.tar.gz；
证据/tmp/pnc_curve_turn_1m_20260921_150257/，持久包pnc_curve_turn_evidence_20260921_150257.tar.gz。
旧主套件历史标定/交通灯差异及原短路径/低速保底边界保留；仅同步本次普通刹车力度的旧断言。
从用户已验收上一版同步control_comply.cpp、重编robot、重启control。新调参未部署车辆，待实车复测。

## 2026-09-21：修复D挡曲率噪声导致的直线误限速与前后顿挫

后续用户已确认本节修复版通过实车、直线符合要求；其后转弯与力度重标定见上一节。
下文保留当次交付时的公式和验证状态。

用户实车确认错误算法直线约1.9m/s且车速忽快忽慢，随后要求验证4.5m/s规划给定，
并询问90度左右转弯及180度掉头上限。业务仅修改control_comply.cpp/.h：
新增CalcuForwardPathCurve，在有效路径更新/进入D时按实际沿线前后各2m计算三点圆曲率，
存入独立纵向缓存；原坐标、横向/R曲率与算法保持。缓存退出D清有效长度，重进D重建，
非法坐标使缓存无效且D给定零速。原6m选择窗口、0.6/0.36公式、30次历史释放、当前车速+0.5、
D×13.5/5%保底、规划下降积分及安全优先级保持。6m限定曲率点位置，几何支撑可能达到约8m。

3551帧固定实车输入回放中，使用匹配准确的90.0–94.4s共88帧作直线验收：
原曲率上限1.720–2.026m/s，修正为6.326–8.180m/s；原3m/s目标下油门23–27%、变化29次，
修正后始终40%。同一88帧几何设置规划和反馈4.5m/s后均为60%油门/0%刹车；
4.5m/s是测试输入，不是原记录或修正后车辆实测速度。当前车速<4.0时，+0.5仍限制给定。
24组90度/180度、左右方向、5/8/10/15/20/25m半径圆弧符合0.36*sqrt(R)表，误差<1.5%；
含1cm连续相位噪声的600帧弯道上限组内波动<0.06m/s、油门波动<=1个百分点。
真实入弯测试6m前已降上限、4m前达到圆弧标定；未用时间滤波推迟入弯降速。

新增fixtures/curve_noise_20260921.txt；更新curve_history.cpp、verify_curve_history.py及现行README，
新增[修复验证报告](src/pnc/tests/control/curve_noise_fix_20260921.md)。C++11节点编译/链接通过、
规划制动7473项、最终曲率/噪声/4.5m/s/转弯专项14787项通过；R150条及D切R40条完整消息一致。
旧主套件23项历史预期差异未改，本轮不宣称其全套通过；原短路径入口越界/极低速保底边界未扩改。

```bash
python3 -B src/pnc/tests/control/verify.py --curve-preview --output /tmp/pnc_curve_noise_fix_20260921_142711/control --baseline-control /tmp/pnc_curve_noise_fix_20260921_142711/before/src/pnc/src/robot_control
python3 -B src/pnc/tests/control/verify_curve_history.py --control-build /tmp/pnc_curve_noise_fix_20260921_142711/control --output /tmp/pnc_curve_noise_fix_20260921_142711/final_curve --baseline-control /tmp/pnc_curve_noise_fix_20260921_142711/before/src/pnc/src/robot_control --benchmark
```

路径重建新增O(N)，三插值游标均单向移动；缓存有效数据44N bytes、类增加48 bytes，
不累计多条路径，容量按最大已见规模复用。20Hz限速仍O(P+30)，不在周期内重复估计圆曲率。
主机128/1024点重建P50为1.381/14.016微秒，P99为1.399/16.205微秒；热态0次分配，
首次/容量增长会分配。周期限速P50约0.23微秒，与原版本接近；不是Orin负荷结论。
备份：工程同级pnc_curve_noise_before_20260921_142711.tar.gz；证据/tmp/pnc_curve_noise_fix_20260921_142711/，
持久证据包pnc_curve_noise_evidence_20260921_142711.tar.gz位于工程同级。
两份业务文件一起同步后完整重编robot并重启control。未部署车辆，修正版尚未实车验收。

## 2026-09-21：曲率限速独立复核，确认真实路径与完整入口覆盖缺口

用户要求检查拟合与覆盖。独立数学检查确认相同曲率下的插值、单调性和<=50m半径的20%比例正确；
但原平均曲率与新6m峰值不是同一输入，不能用0.36/0.3直接证明实际路径给定提高20%。
用已有rosbag解析数据链接真实控制源码回放，90.0–94.4s的88帧几何匹配准确，规划均为3m/s：
旧0.6曲率上限4.348–6.543m/s，新上限1.720–2.026m/s；全部低于旧0.3。
94.2987s预瞄窗口整体直线拟合残差最大仅9.04mm，短距离航向差却产生0.04334 1/m的曲率峰值。
回放使用记录反馈，不是新算法实车运行；确认过度限速反例，不推算车辆动力学效果。

另用真实SetPathPlanData入口检查30组短路径，14组在CalcuPathCurve访问size-13时越界，
旧0.6基线完全一致；前次短路径专项绕过了这一入口。极低曲率速度目标也可能被既有5%保底提高。
曲率只降电机给定、不独立触发液压积分补刹的原要求保持，6m不能据此当作完成减速保证。
既有12589项专项重跑仍通过，证明通过数量没有覆盖上述反例；旧主套件23项失败状态保持。

业务源码及现有测试均未改；新增[复核报告](src/pnc/tests/control/curve_preview_review_20260921.md)，
在PNC/控制/测试README及原报告更正验证状态。需先解决直线路径噪声，再对真实转弯/掉头作同轨迹对照，
并补完整短路径入口及最终输出优先级验收，不能把当前方案当作全部满足用户需求的版本。
备份：工程同级`pnc_curve_review_before_20260921_141406.tar.gz`；诊断命令及逐帧数据：
`/tmp/pnc_curve_review_20260921_141406/`，持久证据包为工程同级`pnc_curve_review_evidence_20260921_141406.tar.gz`。
本轮没有部署、新的车端验证或业务性能变更。

## 2026-09-21：D挡6m曲率预瞄、分段限速并恢复速度差上限

用户实车反馈统一0.6时直线稳定但左右转弯/掉头过快，确认弯道相对旧0.3提高约20%。
按此约束构造连续曲线：有效曲率<=0.005取0.6、>=0.02取0.36，中间三次smoothstep；
不是基于实车日志回归。ForwardCurveLimitSpeed复用当前最近点、投影相邻两段，从导航参考点
的投影向前沿路径累计6m，取最大绝对曲率，边界插值、短路径不补零；历史均值只延缓释放。
20%是同曲率稳态公式之比；更早取峰值可能比旧平均法更保守，不承诺实际车速增加20%。

按用户追加要求，在原位置恢复 `if(delta > 0.5) speed_cmd = speed_now + 0.5;`。
开工已是D×13.5及5/13.5，未改这两处标定；R旧曲率/横向/最终油门、规划下降积分、其他safety保持。
曲率单独降低给定仍不发起积分制动，6m不是实车制动完成保证；未修改planning或地图/参数/消息。

业务2份：control_comply.cpp/.h（头文件只改历史注释，未改变布局）。
测试3份：curve_history.cpp、verify_curve_history.py、verify.py，均位于src/pnc/tests/control；
后者新增明确的--curve-preview专项入口，原主套件断言保留。
文档：PNC/控制/控制测试README、本文及新增[曲率预瞄验证报告](src/pnc/tests/control/curve_preview_20260921.md)。
备份：工程同级 `pnc_curve_preview_before_20260921_135637.tar.gz`；证据 `/tmp/pnc_curve_preview_20260921_135637/`。

```bash
python3 -B src/pnc/tests/control/verify.py --curve-preview --output /tmp/pnc_curve_preview_20260921_135637/final --baseline-control /tmp/pnc_curve_preview_20260921_135637/before/src/pnc/src/robot_control
```

C++11节点编译/链接通过，规划制动请求7473项、曲率/边界/非均匀采样/真实左右圆弧/速度差12589项通过；
R独立150条及D切R40条完整消息与改前一致。旧主套件改前133项通过/23项失败，改后附带对照
224项通过/相同23项失败，主要为D×15和已不存在的交通灯停车期望；无新失败，未修改旧断言。
本轮专项入口不宣称旧全套通过。更多对照及耗时记录见报告。

以总路径N、6m窗口点数P计：新增几何预瞄O(P+30)，最坏O(N)，复用已有最近点，无额外全路径扫描；
原D函数固定30+30求和为O(1)。无新增堆分配、实例成员或高频日志，30项缓存按原入口清理。
主机5轮同输入交替测量中位P50约0.049→0.226μs、P99约0.055→0.250μs；单轮峰值波动详见报告。
这些不是Orin负荷或实车验收结论。同步业务两文件、重编robot并重启control后生效。

## 2026-09-21：删除D挡control起步上升斜坡

用户反馈planning起步加速与control缓起叠加，按要求删除D挡`SmoothLaunchSpeed()`及调用、
`ResetLaunchSpeed()`包装函数、两个斜坡状态成员和`/robot/control/launch_speed_slope`读取。
control现在从规划目标应用原速度差上限、曲率/故障限速与5%低速保底后，直接按D×15输出。
规划目标下降触发的刹车积分及门槛/比例、曲率公式、终点缓刹、各项独立安全停车和R行为保持。
删除包装函数时，将原复位入口改为直接清D曲率历史；原非正给定归零及清历史的处理也保留。
未修改planning、横向控制、导航速度来源、消息、参数文件、地图和用户标定。

修改文件共9个（无新增工程文件）：

- 业务2个：`src/pnc/src/robot_control/control_comply.cpp`、`control_comply.h`。
- 测试3个：`src/pnc/tests/control/launch_speed.cpp`、`curve_history.cpp`、`verify.py`。
  原斜坡断言更新为当周期执行规划目标、旧参数无效及原上限/保底；积分与安全断言继续保留。
  曲率测试当前分支改为直接清缓存；历史基准宏分支仍调用旧快照接口。
- 文档4个：`src/pnc/README.md`、`src/pnc/src/robot_control/README.md`、
  `src/pnc/tests/control/README.md`、`workflow.md`。

备份：工程同级`pnc_remove_launch_ramp_before_20260921_111448.tar.gz`；逐文件快照、
改前/改后哈希和验证证据在`/tmp/pnc_remove_launch_ramp_20260921_111448/`。
工程根执行的主要验证命令：

```bash
python3 -B src/pnc/tests/control/verify.py --output /tmp/pnc_remove_launch_ramp_20260921_111448/control --baseline-control /tmp/pnc_remove_launch_ramp_20260921_111448/before/src/pnc/src/robot_control
python3 -B src/pnc/tests/control/verify_curve_history.py --control-build /tmp/pnc_remove_launch_ramp_20260921_111448/control --output /tmp/pnc_remove_launch_ramp_20260921_111448/curve_history
```

结果：C++11控制节点编译/链接通过；控制247项（含R完整消息及稳定标定差分）、积分请求7473项、
曲率695项通过。另将现有`tests/navigation_speed/control.cpp`与本轮新控制对象、can_comm真实源码
重新链接，按既有导航专项的同一输入矩阵验证：18组216条CAN冲突消息对照、2项制动期望、
12项时效和210项异常/恢复检查通过；编译命令及结果在`navigation_control/compile_command.json`
和`navigation_control/result.json`。本轮未重复全量planning构建，planning源码未变。

以路径点数N为规模，删除的斜坡运算/状态均为O(1)，正常D驱动周期少一次ROS参数读取；
保留的非正给定判断与固定30项曲率复位为O(1)，未增加容器、循环、分配或日志。
原主控制/横向路径计算复杂度保持，未实测Orin时延或CPU负载；不将代码减少等同于整车性能结论。
本机无ROS1，以上为真实源码桩验证；需同步业务两文件、完整重编robot并重启control，未部署实车。

## 2026-09-21：普通积分补刹仅由规划减速触发

按用户要求，`src/pnc/src/robot_control/control_comply.cpp/.h`在规划回调中比较原始
`desireSpeed`：小于上一有效目标才建立减速请求，首条消息仅建立基准；同值保持请求，
连续下降保留积分，目标回升立即取消。导航车速达到原始规划目标或正误差消失即结束本次请求，
保持同值目标下的后续超速不能重新介入。曲率/传感器本地限速不能自行启动该积分。
没有增加速度差死区；原0.1m面积门槛、实际时间积分、dec×50/最小1%/上限100%和饱和规则保持。

任务/路线/路径编号、模式/挡位、CAN急停、停车及无效规划输入清除请求和比较历史，
避免跨上下文继承减速。普通误差清零保留目标比较基准，避免每帧清历史而漏掉真正的规划下降。
导航/CAN时效与已接入补刹的断流保持策略沿用；safety、GNSS、围栏、终点等独立制动优先级保持。
保留用户D×15和5/15目标保底，R×10/45%及横向算法、规划、消息、参数和业务CSV均未改。

新增`tests/control/brake_request.cpp`，更新`tests/control/launch_speed.cpp`与`verify.py`；
原积分用例补发真实规划下降，新增专项明确验证恒速/升速超速永不自行触发、达到目标后的反弹、
同周期回调和各类复位。`tests/navigation_speed/control.cpp`的积分时效夹具同步发真实下降。
更新PNC、控制与导航专项README，新增控制测试README，详见[控制回归](src/pnc/tests/control/README.md)。

备份：工程同级`pnc_brake_request_before_20260921_094612.tar.gz`；改前哈希、逐文件快照、
编译/对照日志及结果在`/tmp/pnc_brake_request_20260921_094612/`。复现命令（工程根）：

```bash
python3 -B src/pnc/tests/control/verify.py --output /tmp/pnc_brake_request_20260921_094612/control --baseline-control /tmp/pnc_brake_request_20260921_094612/before/src/pnc/src/robot_control
python3 -B src/pnc/tests/planning/verify_map_speed_limit.py --output /tmp/pnc_brake_request_20260921_094612/planning
python3 -B src/pnc/tests/navigation_speed/verify.py --planning-build /tmp/pnc_brake_request_20260921_094612/planning --output /tmp/pnc_brake_request_20260921_094612/navigation
python3 -B src/pnc/tests/control/verify_curve_history.py --control-build /tmp/pnc_brake_request_20260921_094612/control --output /tmp/pnc_brake_request_20260921_094612/curve_history
```

结果：触发专项7473、控制244（含改前R完整消息和标定对照）、曲率695项；C++11控制节点通过，
全量53个PNC翻译单元、6个节点编译链接通过。地图1201、闸机6581、末端2612、感知995、
起步左二615、旧起步移除756、规划/控制末端113；CSV读取102/升级79及各自ASan/UBSan通过。
导航规划139、时效12、异常恢复210、ultra29通过；18组216条CAN冲突完整消息及2项制动期望通过。
历史起步2m与当前1m的差异仍保留，仅运行左二专项，不宣称旧完整套件通过。
复用改前诊断的10组输入/2600条控制消息：原目标不变时0.01～0.6m/s超速导致的补刹均消失；
真实2→1.4m/s规划下降、实际2m/s的30%补刹仍在下降0.2s后介入。证据见`diagnosis_replay/result.json`。

新增状态仅两个bool，规划目标比较和控制门控为O(1)，无新增日志、容器或分配。
沿用任务回调已有路线列表比较，将比较条件扩展到已保存规划目标但曲率缓存尚未建立的阶段；
最坏仍为O(L)，L为路线名称列表及字符总长度，未增加另一轮比较。未实测Orin耗时/负载。
本机无ROS1，以上为真实源码桩验证；未部署或实车验收。需同步两份控制源码，重编robot并重启control。

## 2026-09-21：导航速度切换的独立复核与异常值修正

用户要求校验对其他业务的影响。开工核对上一轮交付1566份文件哈希一致；对照原始
`navigation_speed_before_20260921_081522.tar.gz`，重新编译改前控制及规划，用同输入逐字段比较。
正常控制126组3737条、R挡60组2460条消息一致，仅排除有意从默认0改为导航值的vehicleSpeed。
规划夹具的随机段此前仅设置导航值、CAN仍为0，先修正为两路同值再进行差分；387组状态、
635条路径消息、48条速度记录和参数事件共6054行逐字节一致。来源冲突专项仍用不同值，未被替代。

新增异常注入复现一类遗漏：导航NaN时控制发布15%驱动且转角NaN；规划在NaN/Inf下仍可能
发布非零目标，负无穷下目标为-Inf。业务仅再改 `robot_control/control_comply.cpp` 与
`robot_path_plan/path_plan_output.inc`：前者在原先行停车分支后、行驶计算前检查有限值，
异常时零驱动/转角/速度目标/期望加速度、100%制动并复位普通历史；后者最终发布叠加safety和零速。
原safety、终点/N挡80%保持、红灯、空路径处理顺序不变，不用异常保护覆盖这些已有分支。
正常输入无新增状态变化；异常恢复保持原斜坡和其他安全来源，不回退CAN、不伪造实际速度遥测。

测试更新 `tests/navigation_speed/{control.cpp,planning.cpp,verify.py}` 与
`tests/planning/regression.cpp`，补6组210项D/R NaN/±Inf及恢复断言、42项规划异常/恢复断言。
更新PNC/控制/规划/专项README，新增 [复核报告](src/pnc/tests/navigation_speed/review_20260921.md)。
备份为工程同级 `navigation_speed_review_20260921_083552_before_fix.tar.gz`；所有复现、
前后trace、差分JSON、构建及回归结果在 `/tmp/navigation_speed_review_20260921_083552/`。

最终重新执行（工程根）：

```bash
python3 -B src/pnc/tests/control/verify.py --output /tmp/navigation_speed_review_20260921_083552/control
python3 -B src/pnc/tests/planning/verify_map_speed_limit.py --output /tmp/navigation_speed_review_20260921_083552/planning
python3 -B src/pnc/tests/navigation_speed/verify.py --planning-build /tmp/navigation_speed_review_20260921_083552/planning --output /tmp/navigation_speed_review_20260921_083552/navigation
python3 -B src/pnc/tests/control/verify_curve_history.py --control-build /tmp/navigation_speed_review_20260921_083552/control --output /tmp/navigation_speed_review_20260921_083552/curve_history
```

53个翻译单元/6个PNC节点、C++11控制/ultra编译通过；控制153、曲率695；地图1201、
闸机6581、末端2612、感知995、起步左二615、旧起步移除756、规划/控制末端113；
CSV读取102/写回79及各自ASan/UBSan通过。导航规划139、时效12、异常恢复210、ultra29项通过，
18组216条CAN/导航冲突全消息对照继续通过。历史起步2m套件差异保留，未宣称全套通过。

CAN其余字段和消息定义未变；底盘不消费can_comm.vehicleSpeed，执行器编码未动；HDMap、
业务CSV、参数和库保持。新增两次O(1)有限值判断，无正常周期I/O/分配/额外遍历，未实测Orin。
导航两帧与CAN两帧耗时不同；导航/CAN≤0.2s检查只用于既定制动路径，不等于全局断流急停。
navigation缓存50Hz重发、原/localization超2s或status=2的GNSS故障链、ultra无独立速度老化保持。
本轮未车端构建、部署或实车验证；同步最新两份业务文件后重编robot并重启规划/控制生效。

## 2026-09-21：PNC/ultra_command 实际车速统一导航，核查 HDMap

用户确认 CAN 速度不准确，要求三个模块使用导航速度。业务仅切换实测自车速度：
规划 `path_plan_perception.inc`、`path_plan_output.inc`、`reference/path_generation.inc`、
`reference/collision_safety.inc`、`safety/perception_safety_adapter.inc` 改用 `gpsSpeed`；
`safety/startup_observation.h/.inc` 将 CAN 模式/挡位/载荷与导航速度更新分开。
`control_comply.cpp/.h` 在导航回调更新 `mVehicleSpeed`、`control_msg.vehicleSpeed` 及接收时间，
保留 CAN 非速度输入；`robot_task_plan/task_plan_node.cpp` 移除未使用的 CAN 速度复制。
`ultra_command_node.cpp` 改订阅 `/navigation_msg`，头文件同步来源注释，纯业务闩锁不改。
按既有命名约束保留 ultra 的 `CanMsgCallBack/can_msg_sub` 标识符，实际类型/话题均为导航。

`path_plan_status.curSpeed` 和下游心跳（×3.6 km/h）统一导航；can_comm 沿用原控制消息转发。
起步确认只允许两帧不同接收时刻的导航速度，绝对值严格>0.2m/s、间隔≤0.6s，仍需新鲜 CAN D挡状态。
控制积分和终点缓刹同时要求导航/CAN接收年龄≤0.2s；未介入积分遇断流清零，已介入补刹保持，
终点缓刹条件不成立返回原80%。检查的是回调时效，不能判断上游重发冻结测量。
ultra 仍严格>0.5m/s一次锁定，沿用无独立速度超时的缓存机制；不改变任务复位/感知/矩形规则。
HDMap源码/SDK接口核查无CAN实际速度；样条Speed为参数导数、velocity为路径目标字段，算法/库不改。
保留本轮开始前用户已改的 D×15 与 5/15 低速保底，R×10/45%及安全阈值保持；未改 canbus。

备份：工程同级 `navigation_speed_before_20260921_081522.tar.gz`，改前1560文件SHA-256及
逐文件快照在 `/tmp/navigation_speed_migration_20260921_081522/`。该目录同时保留测试日志、
结果JSON和最终差异。业务CSV、参数、消息定义和预编译库未写入；只同步相关模块说明。

复现命令（工程根；输出可改到新目录）：

```bash
python3 -B src/pnc/tests/control/verify.py --output /tmp/navigation_speed_migration_20260921_081522/control
python3 -B src/pnc/tests/planning/verify_map_speed_limit.py --output /tmp/navigation_speed_migration_20260921_081522/planning
python3 -B src/pnc/tests/control/verify_curve_history.py --control-build /tmp/navigation_speed_migration_20260921_081522/control --output /tmp/navigation_speed_migration_20260921_081522/curve_history
python3 -B src/pnc/tests/navigation_speed/verify.py --planning-build /tmp/navigation_speed_migration_20260921_081522/planning --output /tmp/navigation_speed_migration_20260921_081522/navigation
```

结果：53个PNC翻译单元、6个节点链接；C++11控制与ultra节点通过。控制153、曲率695；
地图限速1201、闸机6581、末端2612、感知995、起步左二615、旧起步移除756、规划/控制末端113项通过。
24个业务CSV/30992行读取检查102项、升级79项，普通和ASan/UBSan均通过。
新增97项导航规划断言；18组矛盾CAN/导航完整控制输出对照（216条消息）、2项独立制动期望；
12项断流/终点制动及29项ultra导航订阅/闩锁断言通过，NaN/Inf的CAN速度也不改变控制输出。
旧夹具同步补发导航以保持原场景；新专项分别调用真实入口，避免同值夹具掩盖来源错误。
曲率脚本原依赖已删除的生产打印，已改为测试副本计算并输出限速，未恢复生产调试输出。
历史起步2m套件与当前1m源码不一致，继续明确只运行其左二专项，未擅改半径或宣称全套通过。

性能：速度读取仍为O(1)，新增导航接收时间/起步速度时间两个double和固定条件判断；
没有新增周期遍历、容器、堆分配、文件I/O或参数请求。起步计票从CAN回调移到导航回调，
两路消息频率不同会改变“两帧”的实际耗时；阈值和0.6s上限保持。未实测Orin负载。
本机无ROS1，以上为真实源码桩验证；本轮未车端构建、部署或实车测试。
需同步本轮源码，完整重编robot和ultra_command并重启受影响节点后生效。

## 2026-09-21：调试完成，删除临时速度打印

按用户要求，仅删除此前新增在8份planning分片中的50处编号std::cout，包含期望/GPS/车辆速度。
逐文件核对与`/tmp/pnc_terminal_scope_20260921_063112/before_trace/`中的调试前源码完全一致；
没有修改条件、赋值、接口、发布顺序或原有效提示，减少这批打印的格式化和终端刷新。
同步PNC/规划README、将编号对照表标记为历史记录并更新本文，未改既有测试。
备份：工程同级`pnc_speed_trace_remove_before_20260921_074208.tar.gz`；证据：
`/tmp/pnc_speed_trace_remove_20260921_074208/`，保存12份修改前文件、差异、哈希及编译/链接命令。
重编path_plan_comply.cpp并链接既有map_speed_limit测试，1201项通过，运行编号打印为0。
回归命令：`/tmp/pnc_speed_trace_remove_20260921_074208/map_speed_limit /home/mothotob/work/projects/claude-code/qingwei-L4-No2-pudong/src/pnc /tmp/pnc_speed_trace_remove_20260921_074208/fixtures`。
开工检测到control_comply.cpp相对前次交付另有修改，已原样保留；本轮专项验证只覆盖planning。

## 2026-09-21：速度打印追加GPS和车辆速度

按用户要求扩展8份planning分片中的全部50处编号打印，在期望速度后依次追加
` << " " << mNavData.gpsSpeed << " " << mVehicleData.vehicleSpeed`，每行仍只用一次std::endl。
仅读取现有字段，没有改变速度计算、编号、条件或消息发布；同步编号说明。
备份：工程同级`pnc_speed_triplet_before_20260921_070019.tar.gz`；证据：
`/tmp/pnc_speed_triplet_20260921_070019/`，含修改前文件、编译/链接命令和结果。
重编path_plan_comply.cpp并链接既有map_speed_limit测试，1201项通过；2156行运行输出均含
三个用单空格分隔的速度值，原编号与desireSpeed和追加前逐行一致。新增两个字段的读取与
格式化开销未在车端测量，打印位置和刷新次数不变。
回归命令：`/tmp/pnc_speed_triplet_20260921_070019/map_speed_limit /home/mothotob/work/projects/claude-code/qingwei-L4-No2-pudong/src/pnc /tmp/pnc_speed_triplet_20260921_070019/fixtures`。

## 2026-09-21：速度打印增加空格分隔

按用户要求，8份planning分片的50处标签由`"number x"`改为`"number x "`，
例如最终发布值显示为`number 49 1.46`。编号和全部业务代码保持，更新编号说明。
备份：工程同级`pnc_speed_space_before_20260921_064825.tar.gz`；证据：
`/tmp/pnc_speed_space_20260921_064825/`，含10份修改前文件、编译/链接命令和结果。
使用既有ROS桩重编包含全部8份分片的path_plan_comply.cpp，复用已核对未变的对象，
链接并运行既有map_speed_limit测试：1201项通过，2156行运行打印均有单空格分隔。
回归命令：`/tmp/pnc_speed_space_20260921_064825/map_speed_limit /home/mothotob/work/projects/claude-code/qingwei-L4-No2-pudong/src/pnc /tmp/pnc_speed_space_20260921_064825/fixtures`。
逐文件反向去除这50个空格后与本次备份完全一致；没有新增流操作、循环或状态。

## 2026-09-21：收窄终端清理并按业务流程打印planning速度

用户澄清只清除不参与业务的调试输出，随后要求planning打印所有desireSpeed，并强调编号
必须按业务顺序。首次扩大到ROS日志、日志事件/状态及求解器输出设置的改动已撤回；
恢复148处原输出（含急停闩锁提示）、TaskPlan日志事件接口及发送顺序、安全日志限频状态、
求解器原verbose/printLevel和原控制测试。最终仅删除61处数值转储、分隔线及重复流程跟踪，
保留文件/参数错误、停车原因、任务输入/拒绝/切换/完成、业务配置和主动调用的诊断接口。

planning新增50条std::cout，覆盖mTaskPlanData.desireSpeed、mDesireSpeed、mReferPath.desireSpeed、
mPlanPath.desireSpeed及感知回调的局部desireSpeed。固定编号按初始化/输入→周期规划→参考路径
生成→兼容碰撞/专用感知限速→最终输出平滑/停车覆盖→发布和恢复的调用链编排，分支跳号。
普通周期从14开始，38为参考路径实际发布值，49为最终规划发布值，50为发布后恢复的内部值；
1～11的独立回调/初始化并非同一周期，12～13保留原作业调度失败分支。
每个编号、函数和位置见[对照表](src/pnc/src/robot_path_plan/desire_speed_trace.md)。

相对最初开工源码，最终修改21份自有C++源码/分片，仅增删打印语句，函数接口、成员、
计算式、条件、调用次序和发布字段保持。新增打印只读取现有标量，不重复求值业务函数。
44处有效速度显式赋值逐项核对均有打印；整包任务复制及输入/快照/发布额外6处观测。
没有新增测试或修改既有测试，业务提示原样保留；同步PNC/控制/规划README、新增编号表及本文。
消息、CMake、launch、路径CSV、YAML、第三方源码和库均未改动。

备份位于工程同级：首次原始基线`pnc_terminal_before_20260921_062232.tar.gz`；
范围纠正前`pnc_terminal_scope_before_20260921_063112.tar.gz`；新增编号打印前
`pnc_planning_speed_trace_before_20260921_063723.tar.gz`。证据目录：
`/tmp/pnc_terminal_scope_20260921_063112/`，保存删除/恢复清单、逐编号位置表、44处赋值覆盖、
源码逐字节差异审计、运行编号顺序及编译/测试结果。清除本轮新增cout后与收窄清理后的
8份规划分片逐字节一致；再还原61处删除打印后与最初业务源码逐字节一致。

验证命令（工程根，桩由真实.msg生成）：

```bash
python3 -B src/pnc/tests/planning/verify_map_speed_limit.py --output /tmp/pnc_terminal_scope_20260921_063112/planning
python3 -B src/pnc/tests/control/verify.py --baseline-control /tmp/pnc_remove_terminal_20260921_062232/before/src/pnc/src/robot_control --output /tmp/pnc_terminal_scope_20260921_063112/control
python3 -B src/pnc/src/robot_perception_convert/tests/verify_publish.py --output /tmp/pnc_terminal_scope_20260921_063112/perception
```

53个编译单元、6节点链接通过；地图1201、闸机6581、终点2612、感知995、起步左二/安全隔离615、
退出不重入756、终点控制113项通过。CSV解析普通/ASan+UBSan各102项（24文件30992行），
写回各79项通过。控制C++11回归244项通过，包含修改前消息/发布顺序对照；感知真实CMake目标
及SDK动态链接通过，发布专项普通/ASan+UBSan各4414项通过。旧起步2m夹具与现行1m差异未改，
使用已有定向入口，没有声称旧完整套件通过。从本轮地图与终点测试日志提取66个完整规划周期，
每个周期内编号均严格递增；覆盖9种执行序列，不把单独调用辅助函数的测试当作完整业务周期。

性能：没有新增遍历、缓存或业务计算，新增每个观测点为常数次标量读取和终端输出。
用户明确要求的std::endl逐行刷新会增加同步I/O开销；未测车端时延或CPU，不能声称零运行开销。
本机编译/回归不代替车端构建和实车验证，未连接或部署车辆。

## 2026-09-20：D挡单次参考轨迹延长到60点

用户要求单次轨迹由30点延长到60点；原实现按20个采样点里程的1.5倍截断，通常约30点，
并非固定30。沿用此前D挡修改边界：D改为最多60个真实采样点，保留原隔点筛选和0.3m间隔；
R及其他挡位完整保留原里程算法和末段插值。剩余路线不足时取真实终点，前方不足10点仍只
补入路线内已走过的点，整条路线不足规划要求时仍发布原零速空轨迹。不改任务终点/临停索引，
不将局部轨迹终点作为新的停车点；无新缓存、参数、消息字段或周期频率变动。

修改文件共4份：业务`src/pnc/src/robot_path_plan/reference/path_generation.inc`；
测试`src/pnc/tests/planning/terminal_stop.cpp`；说明`src/pnc/src/robot_path_plan/README.md`及本文。
车端运行仅需同步业务.inc，重编robot并重启planning。没有改control、CSV、配置或其他安全代码。
备份：工程同级`pnc_reference_60_before_20260920_183912.tar.gz`；证据：
`/tmp/pnc_reference_60_20260920_183912/`，含开工源码/哈希、编译命令、结果、性能与差异审计。

验证命令（工程根目录，ROS使用真实.msg生成的桩）：

```bash
python3 -B /tmp/pnc_reference_60_20260920_183912/verify_reference.py
python3 -B /tmp/pnc_reference_60_20260920_183912/probe_reference.py
```

40个planning/共享算法单元及修改前后规划节点编译链接通过（主机Boost依赖使用C++14，生产标准未变）。
地图限速1201、闸机6581、终点/轨迹2612、感知995、起步左二及安全隔离615、退出不重入756项通过。
新增断言验证D长路线固定60点、非均匀曲线保留实际采样、发布完整60点以及速度/真实终点距离保持。
既有末端、短路径、R插值、停车条件和其他safety覆盖继续通过；旧起步2m完整套件的既有差异未掩盖。
最后新增的两条完整发布断言另只重编/运行terminal_stop，结果已并入build/result.json。

采样函数原文配合实际XYZ_COOR_S，以C++11、-Wall/-Wextra/-Werror及ASan/UBSan各通过230977项：
遍历当前312地图全部起点及均匀/密集/非均匀曲线/重复/空/单点路线，核对60点上限、真实点序、
无外推、无历史残留，R/N/P逐点及航向与改前完全一致。当前CSV有2214个可取满60点的窗口，
原29～32点、中位30点；修改后均60点。其折线长度中位数13.359→27.255m，修改后约24.831～33.485m。
该统计直接使用本地CSV，未包含车辆接入段重采样，不是实车测量或固定27m门限。

`route_publication.cpp`使用真实规划函数/发布捕获，沿该CSV全部2332个起点分别检查旧链空输入、
新链有效空输入，并持续提供关闭闸机消息；修改前后各9331项检查通过。共4664帧的非几何完整
路径消息字段、规划状态、声光输出和参数事件完全一致（明确排除按需求改变的x/y数组）。
结果见route_publication_comparison.json；该对照不覆盖新增长轨迹区间内出现障碍物的所有场景。
参考轨迹变长可能使兼容碰撞或独立前扫描更早发现前方风险；阈值不变，专用感知仍使用其独立全局前视。

性能：D按点数终止，不再做原始段逐段里程hypot；三个局部数组各预留60项，避免扩容复制。
N为可用原始路线点数时最坏采样仍O(N)，极密/重复点仍可能扫到末端；没有新增预扫描或跨周期状态。
D输出数组容量合计720 bytes，方法返回后按原调用方式销毁；类布局不变，R算法/分配策略不变。
整条链的消息复制、闸机O(P)检查、兼容碰撞O(MP)及control重采样仍需处理更长轨迹，不能声称整车开销不变。

真实CSV相同起点序列、固定主机CPU、g++ C++11 -O2，预热1000次、每轮20000次、交替5轮；
以下为各轮统计值的中位数，仅测生产采样函数，含分配统计器，单位μs：

| 版本 | P50 | P99 | 单轮最大值 | 每次C++分配 | 每次申请字节 | 输出容量上限 |
|---|---|---|---|---|---|---|
| 改前 | 0.606 | 0.699 | 7.814 | 18 | 756 | 384 bytes |
| 改后 | 0.395 | 0.451 | 6.758 | 3 | 720 | 720 bytes |

基准进程峰值RSS中位数均13484 KiB（包含整个测试环境），未测Orin或整链CPU占用。
采样段提速不能抵消或代表下游增量，仍需车端核对实际负荷和循迹表现。本轮未连接或部署车辆。

## 2026-09-20：仅修复D挡曲率历史残留

用户先要求修复旧曲率残留，随后明确“只修改D档，不要动R档的逻辑”。新增D专用
`ForwardCurveLimitSpeed()`及实例内30项固定数组，每次计算循环覆盖一个最旧值；取消D原先
“曲率变化超过0.001才移动历史”的条件。取点仍是路径数组前30点，历史30项和当前最多30项
之和仍除以60，再使用原0.3/sqrt公式及0.001分母下限，不改预瞄/曲率生成或速度标定。
原 `CurveLimitSpeed()` 函数逐字节保留，仅在非D挡调用；R控制与最终输出保持原规则。

本机合成用例交替输入0.04/0.05曲率30次，再输入全零直线路径：旧函数历史首项被零替换后
余下曲率总和为1.31，算出约2.03m/s并冻结。**2.03是复现用例计算值，不是固定阈值或实车测量值。**
实际VehicleControl回放中旧版90次仍受该限制；新版第30次曲率上限恢复到全零曲率对应的300，
随后仍与原规划/其他速度约束取小，绝非要求车辆行驶300m/s。20Hz下30次约1.5秒；
实际加速仍受原0.15m/s²上升斜坡等规则约束。恒定弯道的历史也正常更新并按原公式收敛。

D历史在任务ID/类型/期望挡位、同ID任务pathList、有效路径Path_Id、挡位/自动状态变化时清空。
现有路径拒绝入口只附加D历史清理，原数据处置不变；原ResetLaunchSpeed停车/离开D入口同时清D历史。
同上下文重复消息不清空正常窗口，清理不会写safety/油门/刹车或改R历史；不同D实例不共享历史。
保留原任务变化对积分的重置条件，同ID路线变化仅清D曲率。

业务仅修改 `src/pnc/src/robot_control/control_comply.cpp/.h`；新增
`src/pnc/tests/control/curve_history.cpp`、`verify_curve_history.py`，更新控制README及本文。
本次补丁未修改planning、CSV/配置、stanley_controller目录源码或R横向源码。
由于类布局变化，车端同步两份业务文件后完整重编robot并重启control。

交付审计发现同一control_comply.cpp内 `VehicleStanleyControl()` 输出系数另有1.1→1.0差异，
文件mtime为17:43:04；核对本轮全部写入记录，该差异不在曲率修复补丁中，来源未确认，保留工作区当前1.0。
该入口仅D调用，70%/30%融合本体与R入口仍保持；不得把额外系数调整归因于本次曲率修复。

验证命令（工程根目录）：

```bash
python3 -B src/pnc/tests/control/verify.py --baseline-control /tmp/pnc_curve_history_20260920_173457/before/src/pnc/src/robot_control --output /tmp/pnc_curve_history_20260920_173457/control
python3 -B src/pnc/tests/control/verify_curve_history.py --control-build /tmp/pnc_curve_history_20260920_173457/control --baseline-control /tmp/pnc_curve_history_20260920_173457/before/src/pnc/src/robot_control --output /tmp/pnc_curve_history_20260920_173457/history --benchmark
```

发现上述系数差异后，按相同命令另在 `control_final/`、`history_final/` 重新编译验证当前源码
（第二条改用control_final作为构建输入，未重复性能测量）。原性能结果保留在history目录。

这次验证后，17:51:42工作区 `stanley_controller.cpp` 又保存了两条高于2m/s时调整k/phi_weight的语句，
也不在本次补丁内，未覆盖；异步询问用户是否正在调参。17:53单独编译该文件快照失败：
75/76行写成 `std::abd`，标准函数应为 `std::abs`；证据及源码快照在 `concurrent_stanley/`。
本轮244/695项通过不覆盖这份后续横向改动，不能将当前完整工作区标记为已编译可部署。

C++11控制节点编译/链接与244项既有回归通过，曲率专项695项通过，包含释放时限、恒定/微小曲率、
窗口边界、不同实例、任务/路线/路径/模式/挡位变化、重复消息及安全覆盖。
额外R直线/左右曲线、减速/safety切换150条完整输出一致，D后切R另40条完整输出一致。
编译仅保留已有告警；本机验证没有连接车辆，未将合成曲率用例写成现场减速根因或实车验收。

性能边界：D曲率入口由全路径按值复制的O(N)时间/临时空间改为O(1)，至多30+30项直接求和，
无周期堆分配；新固定状态增加256 bytes（本机ControlComply 1000→1256 bytes）。
不维护长期增量和，避免浮点漂移残留。D历史有效且任务原关键字段未变化时才比较pathList，
额外最坏O(L)，L为路径名总长度；任务回调原有复制保持。R曲率函数的复杂度不变。

相同输入/固定主机CPU、新旧交替5轮，每轮预热1000次、测10000次D曲率入口；表为各轮统计值
中位数，单位μs，改前→改后。计入C++分配统计器和原函数内日志格式化（输出到/dev/null）；
新D入口不再执行原函数内四条诊断打印，主控制curvelimitspeed/speed_cmd日志仍保留。

| 路径点数 | P50 | P99 | 单轮最大值 | 每次C++分配 |
|---|---|---|---|---|
| 128 | 0.531→0.048 | 0.693→0.055 | 11.308→0.119 | 8→0 |
| 1024 | 1.456→0.047 | 1.624→0.055 | 13.380→3.460 | 8→0 |

测试进程峰值RSS中位数均15768 KiB（含启动环境）；不把局部主机测量外推为整车CPU降幅。
开工备份：工程同级 `pnc_curve_history_before_20260920_173457.tar.gz`。
证据：`/tmp/pnc_curve_history_20260920_173457/`，含改前control、前后文件哈希、源码差异、
控制/专项回归日志及history目录的编译命令、回放和benchmark.json。

## 2026-09-20：按实车反馈回滚D挡横向融合调权

用户反馈“刚才control部分的改动效果更差了，回滚回去”。按调权前备份完整恢复
`src/pnc/src/robot_control/stanley_controller/stanley_controller.cpp`，包括原日志；
融合前恢复固定`k=0.3`，删除车速相关权重计算。R挡、原预瞄/滤波/限幅、纵向/制动/safety、
四列CSV读取及planning保持回滚前状态。删除本次调权新增的
`src/pnc/tests/control/lateral_blend.cpp`，更新控制README与本文，不回退其他已实车确认的改动。

验证：`python3 -B src/pnc/tests/control/verify.py --output /tmp/pnc_lateral_rollback_20260920_171948/control`
完成C++11控制节点桩编译/链接及153项回归。恢复文件与调权前归档逐字节一致，SHA-256为
`9f0c8af13d4cbb6f4f9795643c95a55583bd155278e916098eb5d581b73fce06`；
control目录12份源码/头文件均与调权前快照一致。全PNC自有文件及workflow回滚前后哈希核对，
范围仅本轮业务.cpp、删除的专项测试和两份文档；不把本机验证写成车端已完成回滚。

复杂度：路径点数N下，恢复前后完整横向算法均最坏O(N)，权重计算O(1)、辅助空间O(1)，
没有新增缓存或跨周期状态。复用已备份的性能驱动，仅运行其benchmark入口；相同输入、固定主机CPU、
每轮预热1000次/采样10000次、交替5轮，保留日志格式化但输出至/dev/null。
以下为5轮中位数，单位μs，回滚前调权版本→回滚后原版本：

| 路径点数/车速 | P50 | P99 | 单轮最大值 |
|---|---|---|---|
| 128 / 2 m/s | 1.256→1.265 | 1.488→1.501 | 7.509→8.548 |
| 128 / 3 m/s | 1.275→1.283 | 1.512→1.520 | 8.610→8.374 |
| 1024 / 2 m/s | 3.300→3.294 | 3.607→3.602 | 12.561→11.508 |
| 1024 / 3 m/s | 3.307→3.337 | 3.636→3.681 | 11.017→11.603 |

40次测量区间均无C++ new/new[]分配，对象24 bytes、测试进程峰值RSS中位数14916 KiB
（含启动环境）；不据这些主机数据宣称车端CPU降低或摇晃问题已解决。
恢复来源：工程同级`pnc_lateral_blend_before_20260920_165242.tar.gz`。
回滚前备份：工程同级`pnc_lateral_blend_rollback_before_20260920_171948.tar.gz`。
证据：`/tmp/pnc_lateral_rollback_20260920_171948/`，含源码、前后哈希清单、控制回归结果、
`verify_performance.py`、编译命令及`benchmark.json`/`benchmark_summary.json`。
车端运行文件只需同步上述Stanley `.cpp`，重编robot并重启control生效。

## 2026-09-20：仅D挡横向融合权重随实际车速小幅调整（已回滚）

本节保留当轮交付记录；后续实车反馈效果更差，当前实现以本节上方回滚记录为准。

用户反馈D挡空载直线、实际车速超过3 m/s时摇晃；本轮按明确要求只调整融合比例，
未将静态排查项作为已确认根因，也未顺手修改滤波、预瞄、转角限幅、通信周期或其他安全逻辑。
≤2 m/s维持Stanley70%/纯追踪30%；2～4 m/s线性插值，3 m/s为65%/35%，≥4 m/s封顶60%/40%。
每周期直接计算，降速无残留；新增分支限定Gear=4，R继续走原独立倒车控制。

业务仅修改 `src/pnc/src/robot_control/stanley_controller/stanley_controller.cpp` 的
`LatController::calculate()`：用实际速度计算k，移除后续固定0.3覆盖，并将原日志k改成真实权重/三位小数。
新增 `src/pnc/tests/control/lateral_blend.cpp`；更新控制README与本文，无消息/配置/地图改动。

验证：`python3 -B src/pnc/tests/control/verify.py --output /tmp/pnc_lateral_blend_20260920_165242/control`
完成C++11控制节点编译/链接及153项回归；实际算法专项2962项通过。改前横向源码重新链接后，
80个场景/2922条完整控制消息一致，其中60个R场景；专项另核对低速D、直接R入口、边界与无残留。
高频新增O(1)时间/空间，无新增容器或历史状态，完整算法仍最坏O(N)。主机同输入交替5轮
测量零C++堆分配、对象24 bytes；P50/P99/峰值与方法见下方历史证据目录。
当轮交付时未据主机回归或性能数据宣称车辆摇晃已解决；后续实车反馈及回滚见上条。

备份：工程同级 `pnc_lateral_blend_before_20260920_165242.tar.gz`。
证据：`/tmp/pnc_lateral_blend_20260920_165242/`，含 `lateral_compile_commands.json`、
`lateral.check.log`、`integration_comparison.json`、`benchmark.json` 和 `verify_comparison.py`。
车端只需同步本轮业务.cpp，重编robot并重启control；测试与文档不属于车载运行依赖。

## 2026-09-20：今日修改实车验证通过（本轮横向调校前，用户确认）

用户反馈：“目前为止今日修改的部分都已实车测试验证通过”。据此将截至该反馈时的
今日改动更新为实车验证通过，作为后续继续修改的基线。

对应今日交付范围：闸机参考路径位置门控及前方8m范围、闸机历史状态清理、两个感知
入口及起步观察的左二type=2过滤、地图第四列最高限速、第五列/旧区域限速业务删除，
以及planning读取旧地图时补speed=10并保存原文件。现有1m起步观察的保留情况已在反馈前核对。

本条验证结论来自用户实车反馈；此前各轮本机编译/回归记录继续作为各自的验证证据，
交付时的历史部署状态不覆盖本次确认。范围截至当前反馈，后续新增修改另行验证。
本次只更新workflow.md的现状、日志及待办状态；原文备份为工程同级
`workflow_before_vehicle_confirmation_20260920_162655.md`，文档差异检查与当前运行文件哈希核对用于确认记录范围。

## 2026-09-20：四列地图清理与planning自动写回

按用户要求删除地图第五列及PNC读入/传递/休眠变道标志链，删除speed_limit.csv和独立
5m圆形区域限速读取/缓存/应用。保留第四列限速、共享几何z_axis、独立lattice功能及其他安全逻辑。
后续追加需求：planning对三列补10、超过四列截到前四列并保存原CSV；有效四列不重写。
完整校验后流式写同目录临时文件，保留原文本/换行/权限，fsync后替换；并发修改、只读、
非法数据或写入失败保留原文件且不使用未保存路线。只在加载地图时执行，不增加周期I/O。

业务代码11份：新增`include/common/path_csv_upgrade.h`，修改`path_csv.h`、
`robot_path_plan/path_plan_comply.cpp/.h`、`path_plan_task.inc`、`path_plan_reference.inc`、
`path_plan_output.inc`、`path_plan_experimental.inc`、`reference/path_generation.inc`、
`task/global_path.inc`及`robot_control/control_comply.cpp`（路径均相对pnc相应目录）。
现存7份CSV去第五列，删除speed_limit.csv；用户确认保留path_pudong051501.csv的外部删除。
检测到312_cargo_01_01.csv并行修改，113行限速10→0.5及文本格式变化原样保留；所有地图
前三列数值与开工快照相同。task222.yaml仍有两处指向已删除地图，按确认不恢复地图、未擅改任务。
更新规划/控制/路径/测试README；新增写回测试/验证报告，旧区域测试与入口删除，终点控制覆盖迁入地图入口。

验证命令：`python3 -B src/pnc/tests/planning/verify_map_speed_limit.py --output <目录>`；
`python3 -B src/pnc/tests/control/verify.py --output <目录>`。
53编译单元/6节点通过，地图1201、闸机6581、终点2481、感知995、起步定向615、
退出不重入756、终点控制113；四列解析C++11/ASan/UBSan各102项（24文件30992行），
自动写回各79项，控制C++11回归153、实际围栏读取31032项。
前后1269条发布、48条速度、335份状态相同，参数事件仅去掉11次已删除区域配置的初始化读取。
旧起步2m完整套件差异仍保留；本机没有ROS1，不把桩测试当作实车结果。

周期删除区域O(K)扫描/缓存，地图读取仍O(1)，无新周期分配；正常文件加载一次stat+原O(B)
解析，仅需要转换时新增一次O(B)流式读写和临时磁盘空间。规划类2640→2544 bytes，
点布局仍36 bytes；热点前后同输入5轮尾延迟/分配/RSS见报告，未测车端CPU及冷启动耗时。
备份：工程同级`pnc_csv_four_before_20260920_145555.tar.gz`；证据及数据/文件审计：
`/tmp/pnc_csv_four_20260920_145555/`。全量入口结果在`verify_final/`；最终CSV/写回测试增量复验已合入结果。
部署包：工程同级`deployment_20260920_csv_four/`；包含本轮11份代码、上一轮3份配套代码及
全部24份现存地图，另列两份地图删除清单。同步后完整重编robot；携带上一轮view修改时一并重编view。
没有连接车辆或实际部署。详细边界、命令及耗时见
[本轮报告](src/pnc/tests/planning/csv_four_column_verification_20260920.md)。

## 2026-09-20：CSV地图最高限速（上一轮记录）

按用户要求迁移path下所有CSV，第4列默认10；未收到单位补充，已说明采用m/s（36km/h）。
26文件31,719行，原前三列/换行完整保留，8份原四列文件的旧标志移到第5列。
planning装载地图限速，当前最近点与任务/其他速度取小，最终发布再限制；接入段和
抽稀样条传递最低限速。新增每周期O(1)，每路径点增4 bytes，接入时O(N+M)，不另存历史。

改动业务文件：PNC新增`include/common/path_csv.h`，修改`struct_type.h`；规划
`path_plan_comply.cpp/.h`、`path_plan_task.inc`、`path_plan_reference.inc`、`path_plan_output.inc`、
`task/global_path.inc`、`task/operation_path.inc`；控制`control_comply.cpp`；另包simview的
`draw.cpp`，以及26份CSV。根目录审计实际读取节点，ultra已兼容、task_plan只读文件名。
同步模块/路径/测试文档；独立safety、消息、控制标定和原闸机规则未改。

持久验证命令：`python3 -B src/pnc/tests/planning/verify_map_speed_limit.py --output <目录>`。
最终全量53单元、6节点桩编译链接通过；地图932、区域284、闸机6581、终点2481、感知995、
起步定向615、退出不重入756；CSV C++11/ASan/UBSan各106项（全26文件）；控制读取31750项；
ultra 54000个区域包含结果相同，simview实际方法26文件显示相同（未编译完整RViz）。
1269条发布、48条速度和参数事件一致；335份内部状态中5份仅去掉旧EOF重复末点及相应索引。
起步旧2m完整套件差异仍保留。测试计数和性能原始数据见
[本轮报告](src/pnc/tests/planning/map_speed_verification_20260920.md)。

同输入200/20000点、5轮热点测量未新增周期堆分配；点布局32→36 bytes，完整基准进程
峰值RSS中位数16152→16296 KiB。主机统计不能替代车端负荷/实车制动验收。
备份：工程同级`pnc_map_speed_before_20260920_142627.tar.gz`；证据：
`/tmp/pnc_map_speed_20260920_142627/`，最终验证`final_verify/`。
部署准备包在工程同级`deployment_20260920_map_speed/`；因内部布局变化必须完整重编robot，
另重编view并重启，不可只同步CSV或混用旧库。本轮未部署车辆。

## 2026-09-20：闸机前方门限由4m放宽至8m

用户反馈D挡进货站未停，确认车端已同步且闸机输入正常、planning未置出safety；
本机两条实际进货路径在连续关闭输入下可复现4m版本触发，现场根因仍缺同刻参考/定位数据。
用户随后明确要求放宽到8m，本轮只将 `path_plan_output.inc` 的 `max_forward_distance`
从4.0改为8.0，保留严格2m横向、仅前方、D/R覆盖范围、原关闭状态清理及其他safety。
`/refer_path_msg`不含闸机覆盖，实际叠加发生在最终`/plan_path_msg`。

同步修改 `gantry_safety.cpp` 的边界/历史夹具、规划/闸机/测试README与本文，新增
[8m验证记录](src/pnc/tests/planning/gantry_8m_verification_20260920.md)。
新规划实现编译链接，闸机6,581项、C++11与ASan/UBSan几何各33,684项通过；
387组其他业务完整输出与改前一致，两条实际进货路径完整节点回调/任务/发布回放
前后各341项通过，每条路径在相同56帧接近序列中的触发帧由16扩大到36。
本轮只改常量，单次位置判定仍O(N)、额外空间O(1)，无新增遍历、容器或线程；
有效状态保留的区域扩大，不能由复杂度不变推断整段行程的实际CPU消耗相同。

备份：工程同级 `planning_gantry_8m_before_20260920_135011.tar.gz`。
验证命令：`python3 -B /tmp/pnc_gantry_8m_20260920_135011/verify_8m.py`，
证据在同目录；重用本轮左二过滤已编译且源码未变的公共算法/节点入口对象，重新编译
当前规划实现与测试。本机回放不代表现场问题已解决；8m版本尚未由本轮部署。

## 2026-09-20：左二车道输入过滤

按用户要求在 planning 的两个感知消费入口剔除左二 `type=2`，不附加速度/朝向条件；
常规避障与起步观察均生效。旧兼容链复用 ID 标记清除四帧历史，新跟踪在有效整帧提交后
删除旧几何与确认计数，起步观察在新有效帧清除旧框。整帧校验、时效、原急停解除迟滞、
其他安全来源和独立前后扫描规则保持；上游转换发布、control、消息、参数及起步半径未改。

业务仅修改 `path_plan_perception.inc`、`safety/perception_safety.inc`、
`safety/startup_observation.inc` 三个文件，均位于 `src/pnc/src/robot_path_plan/`。
更新四个感知/起步测试与规划、安全、测试 README；新增验证记录。本轮没有新业务文件。
入口复用原遍历，最坏复杂度仍为旧链 O(N+H)、跟踪 O(N+A)、起步输入 O(N)，无新增缓存。
N/H/A 分别是本帧目标数、四帧风险目标总数、活动轨迹数；新增条件仍有常数 CPU 成本。

验证：规划节点前后重新编译链接，387 组既有输出一致；感知核心 C++11 16,395 项、
起步定向核心 10 项及两套 ASan/UBSan 通过；感知集成 995、输入与安全隔离 615、
闸机 5,013、区域 284、起步后不重入 756、终点 2,483 项通过。旧 2m 起步完整套件差异保留。
300 目标、1,000 次预热后观测/评估，前后均 0 堆分配、缓存 4,789,248 bytes。
同输入耗时含无左二时的常数增长与有左二时的减少，未据主机数据宣称车端增量可忽略。

开工备份：工程同级 `planning_left_second_before_20260920_133352.tar.gz`。
主验证命令：`python3 -B src/pnc/tests/planning/verify.py --baseline <备份pnc> --output <build>`；
专项与性能命令：`python3 -B /tmp/pnc_left_second_20260920_133352/verify_remaining.py`。
证据目录 `/tmp/pnc_left_second_20260920_133352/`；完整结果、源码哈希和车端生效方式见
[左二过滤验证](src/pnc/tests/planning/left_second_lane_verification_20260920.md)。尚未部署车辆。

## 2026-09-20：性能恒定准则

将用户的 CPU 优先、有界空间换时间要求写入 `docs/CODING_MEMORY.md`，并在四份 Agent
规则入口明确要求每次开始代码修改任务前重读。按既有同步机制对齐 agy-cli、claude-code、
codex-code 的规则入口，并同步三份工程记忆；本次只修改规则与本文，不修改业务代码。

闸机新增几何判断的增量开销：关闭标志为真时，每次最终发布扫描 N 个当前参考点，
时间 O(N)、额外空间 O(1)；关闭标志为假时保持 O(1)。原闸机标志覆盖为 O(1)。
当前规划循环为 10Hz；尚未实测车端 CPU 增量，本次规则维护没有将该扫描改为缓存实现。

规则同步使用 agy-cli 工程的 `scripts/sync_agent_rules.py --source <本工程AGENTS.md>`；
核对规则正文、记忆内容一致性和两份闸机业务文件 SHA-256。原文备份为工程同级
`performance_policy_before_20260920_120824.tar.gz`，核对证据位于
`/tmp/performance_policy_20260920_120824/`。

## 2026-09-20：闸机误检过滤与历史清理

在`active && !gantry_open`基础上，加入进货站`(96.14,-405.63)`和出货站`(91.13,-393.80)`
两个地图点；任一点到当前`mReferPath`最近实际点的距离严格<2m，且该点沿参考线在自车
前方严格<4m，才在最终发布时叠加`safety=1`。自车以线段投影定位，距离按实际线段累计；
不使用车到闸机直线距离、固定点数或参考线首点近似，不触发车后点及2m/4m边界。

位置匹配每周期重算；离开范围、已驶过、参考线空/无效或任务复位清空参考线后清除
`mGantryStop`，重新进入等待新关闭消息。只清闸机来源；原规划/感知/起步/人工急停的
safety和速度保持，发布后仍恢复原内部值。仍未新增消息超时策略，检测器和control不改。

业务仅`src/pnc/src/robot_path_plan/path_plan_output.inc`、`path_plan_comply.h`两个文件；
测试调整`gantry_safety.cpp`、`perception_safety_integration.cpp`、`startup_observation_integration.cpp`、
`terminal_stop.cpp`。同步规划/检测/测试README与本文件，新增一份验证记录；无业务文件新增/删除。

验证：修改前后规划节点编译链接成功，387组未输入闸机的完整消息/状态/参数事件一致；
闸机5,013、感知998、区域284、起步后不重入756、终点2,483、起步安全交接及D/R隔离420项通过。
新增几何方法原文C++11与ASan/UBSan各26,964项通过，消息桩均来自真实`.msg`。
旧2m起步测试与现有1m实现的差异仍保留，本次定向验证没有冒充完整旧起步套件通过。
执行入口：`src/pnc/tests/planning/verify.py --baseline <备份pnc> --output <验证目录>`，
定向构建/回归命令`python3 -B /tmp/pnc_gantry_route_20260920_114056/run_targeted.py`；
日志、结果、哈希及具体复现方式见 [验证记录](src/pnc/tests/planning/gantry_route_verification_20260920.md)。

开工备份：工程同级`planning_gantry_route_before_20260920_114056.tar.gz`；本机证据目录
`/tmp/pnc_gantry_route_20260920_114056/`。未连接或部署车辆；车端同步两个业务文件、重编robot并重启规划生效。

## 2026-09-19：近期工作（详细）

### 文档与持久规则

按用户要求整理 README/workflow，增加根导航、模块用法、历史注意事项与易错点。
`AGENTS.md`、`CLAUDE.md` 将 `docs/CODING_MEMORY.md` 设为每次编码任务前必读；
风格记忆区分用户明确偏好、工程约定与可调整参数，不把旧缺陷固化为新代码规范。

原文及哈希归档于 `docs/history/2026-09-19-before-docs/`。本轮只修改文档；源码、测试、
配置、消息、地图、模型及启动脚本不改。新增规则覆盖本工程，不能宣称其他工程自动获得记忆。
文档整理时核实了 can_comm 周期、task5、初始化路径引用、感知空帧和 ultra 消费方等旧记录差异，
当前结论见下方“源码与旧记录差异”。

文档验收：32个工程README入口均有使用方法/注意事项，196个维护正文中的本地链接及
21处包名/launch引用核对通过；1,921个源码、测试、配置文件哈希未变，28份原文归档哈希一致。
第三方原文与安装副本按文档索引所列范围保留，本轮未执行车辆/算法运行测试。
整理期间四份规则入口出现同期同步更新，现为AGENTS/CLAUDE/CODEX/GEMINI；保留同步内容，
已核对四份正文一致且都指向风格记忆，未将外部同步文件或机制冒充本轮实现。

### D 挡本车道前方障碍物提前减速

当前新增普通限速的条件为：D 挡、本车道 `type=0`、物理框位于当前车头正前方并与前向
外廓横向相交；自车与目标的纵向接近速度严格大于 1.0m/s，或目标自身速度严格小于 0.5m/s。
距离是**车头保护外廓到障碍框近边的净距**。

- 制动距离之外额外预留 **3 秒**行驶距离。
- 净距 **6m** 时目标速度不超过 **0.45m/s**，严格低于 0.5；6→3m 连续收低。
- 净距 **3m 及以内**普通目标速度为零。
- 复用原唯一 `SmoothLimit()`，按实际周期约束减速度/jerk；近段另留收敛余量，重复
  时间戳不推进平滑。首见过近时硬限速优先，不能为平滑延后停车。
- 不改变原急停距离、确认票数、保持或解除判据。R、control 和起步半径本轮未改。

入口：`safety/perception_safety.inc::ForwardObstacleSpeedLimit()`；常量 `advance_time`、
`slow_distance`、`stop_distance`、`hard_slow_speed`。适配器以 `curGear == GEAR_D` 门控。
详细规则见 [感知安全 README](src/pnc/src/robot_path_plan/safety/README.md)。

该减速修改轮为业务 3 文件、测试 3 文件、文档 3 文件，无新增工程文件。核心 C++11 及 ASan/UBSan
各 16,364 项，最终发布 994 项通过；2,700 对相同输入的 safety 状态一致，576 组 R 完整输出
逐字节一致；300 目标、1,000 次预热后无堆分配。原始产物：
`/tmp/pnc_forward_slow_20260919_cskkj770/`。这些是本机验证；未部署车辆，实际车速与停止距离仍待验证。

### D 挡控制：起步、终点与积分补刹

09-19当日行为如下；09-21已增加规划减速请求门控并删除control起步斜坡，以本文件当前基线为准。

| 项目 | 09-19当日最终行为 | 入口 |
|---|---|---|
| 起步上升斜坡 | 默认 0.15m/s²，仅限制 D 给定上升，下降即时通过；保留油门标定 | `SmoothLaunchSpeed()`、`/robot/control/launch_speed_slope` |
| 普通终点 | 规划 a=0.2m/s²、T=4s、末段 d/8；保留 0.3m/s 蠕行及原 0.5m 到位判据 | `task/stop_speed.inc::LimitDrivingTaskSpeed()` |
| 终点缓刹 | 满足低速普通 D 循迹到位条件时，首帧5%、按100%/s建立至80%；停稳/异常走原保持 | `TerminalStopBrake()` |
| 减速积分 | 自动 D，任意正速度误差立即按实际时间积分；严格超过0.1m才接入普通 brake | `UpdateBrakeIntegral()`、`/robot/control/brake_integral_threshold` |
| 积分后补刹 | `min(100, dec*50)` 与原 brake 取大，最小整数1%，油门清零 | `control_comply.cpp` |
| safety | D 入口即时零油门/100%制动，不等待积分；R 保留原 safety 分支 | `VehicleControl()` |

速度差 0.5/0.05m/s 启动门限、直接 dec×30 补刹及旧三参数起步方案均已被替代。
控制细节及复现入口见 [控制 README](src/pnc/src/robot_control/README.md)。最新控制 C++11
节点编译/链接和244项检查记录在 `/tmp/pnc_brake50_20260919_cehrx1rr/`；D/R范围核对在
`/tmp/pnc_d_only_20260919_7aj03ejf/`。未据此宣称最新控制已部署或车辆响应达标。

### 起步观察与近距 safety

手动切自动后仅 D 挡使用 `/perception/planning` 的真实物理框观察车身四周。
**当前源码半径为1m**；历史需求及旧测试为2m，两者差异保留，不在后续减速任务中擅自修改。
有效空帧立即撤销本项停车；连续两帧 D 挡 CAN 实际车速绝对值严格>0.2m/s后退出，
同一自动会话停车/换任务不重入。起步未完成的 R→D 需等切回后的新感知。

原动态急停反应预留为2.1s，1m/s时计算距离约3.025m。本车道静态碰撞目标取至少3m；
正前方物理净距≤6m且满足紧迫碰撞的真实观测免三帧，6m不是无条件急停距离。
其他紧迫目标保留三帧确认；已急停时未确认的新ID风险也阻止释放。R后向扫描仍严格<2.5m。

专用感知 ACC 屏蔽修复只对 D 生效。用户已确认“切自动不放行”问题解决，但未提供现场
处理原因，不能归功于仅增加诊断的代码。旧2m起步断言仍失败；按当前1m临时夹具767项通过。
详见 [规划 README](src/pnc/src/robot_path_plan/README.md) 和 [测试 README](src/pnc/tests/planning/README.md)。

## 2026-09-18：路径与过滤（较详细）

- 前视路程扩大为原20点覆盖里程的1.5倍，截止真实路线末端；末端补路线内车后点供拟合，
  前馈从车辆投影里程选点。修复R短路径末端转向丢失、短全局路径沿用旧参考和稀疏末点不到位。
  多CSV累计里程连续化。终点曲线的09-18参数已被上面的09-19调柔值替代。
- 本车道外type=3/4过滤基础上，剔除左一type=1、速度≥0.2m/s且与行驶方向夹角>135°的
  明确对向目标；静止、同向慢车、横穿及不确定方向保留。当前有效状态需同时清理旧历史。
- HMI规控录制组加入 `/perception` 和 `/perception/planning`；两类话题同时保留便于定位。
- 证据：[末端复核](src/pnc/tests/planning/today_review_20260918.md)、
  [左一过滤](src/pnc/tests/planning/left_lane_motion_verification_20260918.md)。
  本机回归通过不代表后续版本已上车。

## 2026-09-17～09-14：近期摘要

| 日期 | 最终有效结论 | 详细入口 |
|---|---|---|
| 09-17 | `/perception/planning` 输出真实帧、物理角点和跟踪速度；规划采用缓存与连续碰撞，安全始终叠加 | [感知转换](src/pnc/src/robot_perception_convert/README.md)、[安全](src/pnc/src/robot_path_plan/safety/README.md) |
| 09-17 | 感知入口过滤type=3/4；细长框条件为dx>1且dy>1且长宽比>3。原通用三帧规则在09-19增加近距例外 | [规划](src/pnc/src/robot_path_plan/README.md) |
| 09-17 | 地图点5m圆内限速、重叠取小；闸机仅active且关闭时叠加safety；HMI捆绑3D感知与闸机启动 | [规划](src/pnc/src/robot_path_plan/README.md)、[HMI](hmi/README.md)、[闸机](src/gantry_detect/README.md) |
| 09-16 | 感知空间索引/匹配缓存和monitor文字批量绘制降低本机资源开销；Orin负载与控制延迟仍需实测 | [转换](src/pnc/src/robot_perception_convert/README.md)、[monitor](monitor/README.md) |
| 09-16 | log_online记录事件前后窗口并提供8083状态页；Timer参数改为rospy.Duration | [记录器](log_online/README.md) |
| 09-15 | HDMap车道分类接入；当前7条源地图生成12,537点，原图不变，SDK需区分主机架构 | [HDMap](src/hdmap/README.md) |
| 09-14 | PNC统一4空格和同行括号；格式化与业务修改分开验证；HDMap首期实现 | [PNC规范](src/pnc/README.md) |

## 2026-09-12及以前：历史摘要

| 时间 | 保留结论 |
|---|---|
| 09-12 | planning按task/reference/safety分职责、入口阶段化；保持同一翻译单元和原业务时序 |
| 09-10～09-11 | PNC基底重置及车载回滚；ultra_command加入指定路径区域与初始化监控；旧整理/修复不能假定仍在当前源码 |
| 09-08～09-09 | health_monitor旁路观测；calib/data_logger由用户移除；09-08～09-09的旧PNC修改被09-10基底声明取代 |
| 09-07 | EHB最终按用户整理的ehb-can.md解析；can_msg在T2 20Hz、ehb_msg在T1 10Hz；dashboard接入 |
| 09-05 | 用户确认当日实车基线运行正常；不是此后修改的实车验收结论 |
| 08-19～09-04 | 初期清理、编译修复、Web界面、协议与架构探索；保留归档，不恢复已下线功能 |

完整经过、旧快照名和每轮验证数见 [原workflow归档](docs/history/2026-09-19-before-docs/workflow.md)。
`PNC_ANALYSIS.md` 的旧行号和部分结论已失效，定位当前代码应按函数名搜索。

## 源码与旧记录差异（本次只核对文档，不修业务）

| 事项 | 2026-09-19现树核对结论 |
|---|---|
| can_comm周期 | `can_comm_node.cpp` 声明20Hz，但循环内没有`loop_rate.sleep()`；旧“09-10已修”记录不能作为当前结论 |
| task_plan latch | `task_plan_node.cpp` 的advertise当前无latch参数；晚启动/重启任务同步问题仍需单独处理 |
| 感知空帧/速度 | 当前转换器有效空观测会入窗，且有跟踪vx/vy；旧“零markers静默、vx/vy恒0”不再适用，断流仍不伪造空心跳 |
| ultra消费者 | planning兼容链及新前向适配器都读取`/ultra/status/safe`；旧“仓内零读者”已过时，R仍保持原消费范围 |
| 初始化任务路径 | 当前task5/task18等YAML已有`_01_01`引用，旧“尚未引用”失效；是否实际运行取决于任务下发 |
| task5任务数 | 当前`task_sum: 4`，旧3/4不一致记录不再是现行问题 |
| gantry目录 | 当前实体包位于`src/gantry_detect/`，不是根目录包加软链接；HMI已捆绑启动 |
| 多CSV里程/ACC | 多CSV连续化已在09-18修复，专用感知ACC门控已在09-19按D限定修复，不继续列为未修待办 |

## 待办与已知边界（合并后）

1. **已确认基线与横向调校回滚。** 2026-09-20用户确认截至首次反馈时的今日修改均已实车测试通过；
   后续D挡速度相关融合权重经实车反馈效果更差，已回滚至固定70%/30%，车端需同步重编重启。
   09-21用户已确认曲率抗噪修复通过实车、直线符合要求；后续1m/s版弯道被反馈偏慢，
   本轮取1.4m/s折中并沿用减轻后的补刹，待实车复测。
   其他日期专项记录及其量化边界按原日期保留。
2. **起步半径口径。** 当前1m、历史2m要求/测试的差异需后续明确并统一；不以改测试掩盖差异。
3. **外廓与制动标定。** 空车/挂托盘默认外廓尚未实测，当前无铰接托盘扫掠模型；最差载荷
   减速能力、参考点、静态目标/横穿/断流等需车端验证。
4. **gantry检测边界。** 09-17审查记录定位/点云老化、NaN、降采样后最小点数及漏检判开问题；
   当前启动和safety消费已接入，不等于检测问题已修复。按模块README回放并复核现场参数。
5. **控制与转发历史缺陷。** 当前can_comm循环缺sleep；control围栏文件判空、初始化字段、
   旧转向数学、模糊PID索引和油门/刹车同时非零的双发语义仍应分项复核。不要在文档任务中顺手修复。
6. **canbus边界。** 非法挡位字节初始化、N挡对急停制动的钳制、协议注释冲突及T1/T2相位见
   [模块README](src/canbus/README.md)。消息与灯光契约改前查两侧。
7. **任务/区域与输入。** task_plan非latch，ultra任务未知/重启窗口仍需验证；监控矩形与作业
   走廊重叠、感知排除区目录扫描和charge区域范围需按当前配置确认。旧空帧误报结论已被后续实现替代。
8. **部署工具。** rebuild_all缺少完整错误处理及目录锚定；HMI旧标定卡片无有效执行端；
   MQTT凭据/网络策略、navigation旧定位重发及其他历史问题保持记录，按授权的独立需求处理。
9. **其他模块验收。** HDMap在aarch64重编；monitor外参与Orin性能核对；log_online核对相机
   话题、事件前后窗、磁盘满降级和Timer修复。测试/工具目录不是车载运行必需项。

## 使用、验证与回滚

- 构建、启动、参数入口统一见 [根README](README.md) 和模块README，避免复制易过期命令。
- 本机使用真实`.msg`生成ROS桩；规划依赖的主机Boost可能需要C++14，生产C++11不变。
  用例要选择匹配基线；旧差分脚本不是所有后来业务修改都应逐字节相同。
- 近期PNC证据见 [测试说明](src/pnc/tests/planning/README.md)；归档里的`/tmp`产物可能失效，
  不把缺失旧产物写成可立即复现成功。持久脚本/报告优先。
- 非git改动前保存快照及文件哈希。旧快照主要在工程同级目录，名称见原文归档；本轮文档
  可从仓内原文快照逐文件恢复，不能用旧文档快照覆盖生产代码。

## 保护清单

| 内容 | 原因 |
|---|---|
| `src/pnc/path/`、`param/`、地图和外参 | 用户任务数据及标定，不因清理改写 |
| CenterPoint `build/`、`model/`、驱动可执行与库 | 当前启动依赖相对路径及目标架构产物 |
| `src/fms_agent/env/`、`data/tmp_path.txt` | 运行环境和运行时输入，不按无引用删除 |
| `src/ivlocmsg/`、跨包消息、第三方预编译库 | 编译/通信契约仍在使用 |
| 根启动/录制脚本、`launch/` | 运维入口；只在明确相关任务中修改 |
| 工程外快照、归档、已有版本记录 | 非git回滚证据 |

已下线的一键标定、相机伸缩杆、旧蜂鸣/CAN故障写入、旧固定区域起步等待等，不因整理
恢复。消息声明保留不代表功能仍在运行；反之不能仅凭旧“死通道”列表删除当前接口。
