# qingwei L4 Web HMI（8080）

车端 Web 人机界面：组件一键启停/健康/日志/车辆状态。纯 Python stdlib+rospy，零 pip，非 catkin 包，业务代码零改动。部署=拷 hmi/ 到工程根（与 start_l4.sh 同级）。

## 最近变化

09-21组3新增"自定义指令"卡片（ultra_command，监控矩形障碍物检测）：从 `launch/control.launch`
解绑为独立启停，参与一键启动；节点不发话题，健康=master 节点表计数 `/ultra_command_node`
（fms 同款），stop_pat 兼顾 launch 文件名与节点名（centerpoint 同款，防 SIGKILL 孤儿不可见）。
⚠**部署耦合：本变更必须连 `launch/control.launch`（已删 ultra include）和 `start_l4.sh`
（已补独立终端行）一起同步车端——只拷 hmi/ 的话，车上旧 control.launch 捆绑实例会与卡片
双起同名节点互杀（respawn 反复拉起）**。停止卡片时节点退出前写 `/ultra/status/safe=1`
（故障态），planning 消费链会把 desireSpeed 置 0——停车属预期行为。两个已知边界：只启 pnc
不启本卡时参数不存在、planning 缺省 0，矩形监控静默缺失且无告警（请用一键启动成套拉起）；
nodes 型健康发现不了 respawn 快速崩溃循环（车辆侧 safe=1 停车会倒逼排查）。
另：pnc 卡标题节点数经复核修正为 6——按 launch 递归展开的 node 标签实数（canbus launch
含 2 节点）；旧标题"6节点"在捆绑 ultra 时期实际拉起 7 个节点，同为差一错误。
09-18规控录制组已纳入 `/perception` 与 `/perception/planning`，便于对照历史显示与规划真实帧。
09-17“3D感知”卡片捆绑CenterPoint和gantry_detect，健康同时检查 `/box` 与 `/gantry_state`；
停卡应收掉两者。09-07以前的界面实现细节见历史归档，现行部署和停止契约仍以下文为准。

## 使用方法

```bash
scp -r hmi/ nvidia@<orin-ip>:/home/nvidia/qingwei-L4-No2/
bash hmi/hmi.sh                       # 0.0.0.0:8080；换端口 HMI_PORT=8090
# 打不开页面 → 查 Orin 防火墙 8080
```

### 开机自启（systemd，可选）

```bash
bash hmi/install_autostart.sh          # 安装+启动 qingwei-hmi；remove 卸载
```

- 自启只拉网页服务，组件仍需页面"一键启动"
- 安装脚本幂等写 `/etc/environment` 的 `MOZ_X11_EGL=1`（Firefox WebGL 兼容；装后需注销/重启才生效；卸载时保留）
- `systemctl stop/restart` 只停 HMI；unit 必须 `KillMode=process`（默认值会连杀组件，勿改）
- 日志 `journalctl -u qingwei-hmi -f`；组件日志 `hmi/logs/`
- 装自启后勿再手动 `hmi.sh`（8080 冲突）

### Firefox 3D(WebGL) 初始化失败排查（09-04 实车定稿）

monitor 3D 报 "error creating webgl context" 会自动降级 2D（不阻断）。排查序：

1. 变量是否进了进程：`sudo cat /proc/$(pgrep -f firefox|head -1)/environ | tr '\0' '\n' | grep -i egl`；无输出→启动命令加 `env MOZ_X11_EGL=1`
2. `env MOZ_X11_EGL=1 firefox http://localhost:8081` 手动验证；能出 3D=只是环境传递问题
3. **about:config 三开关（实车生效项），改完须重启 Firefox 而非刷新**：`webgl.disabled=false`、`webgl.force-enabled=true`、`webgl.disable-fail-if-major-performance-caveat=true`
4. 仍失败看 about:support 图形段"不可用"原因；snap 版换 deb（`snap list | grep firefox`）
5. `glxinfo -B` 的 renderer 应为 Tegra/NVIDIA

## 页面

- **一键启动**：按组顺序（核心→传感器→感知挂接→规控→业务辅助），组内并行、组间等健康；非可选组件失败中止后续；再点=补链
- **全部停止**：逆序+二次确认
- **组件卡**：状态灯（绿/黄=降级/红=故障/蓝=过渡/灰=停止）+健康详情（如 `/can_msg 49.8Hz`）+启停/重启/日志
- **车辆状态面板**：车速/挡位/模式/急停/任务/RTK/障碍距离/横向偏差/电量/挂接/传感器/CAN；字段 >5s 无数据灰划线（冻结非清零，用于判断裂点）
- ⚠ **脱挂钩一键标定卡片为孤儿**：canbus 标定 09-03 已整体下线，现触发只会 100s 超时后报失败；去留待决策，现状勿用

## 组件配置（hmi_config.py，纯数据+中文注释）

- netcheck 默认 disabled，启用改 `enabled: True`
- 感知/规控数据录制固定不参与一键启动，卡片手动
- 录制两组各读 `record_rostopic_list.md` 对应分组开关（见下节）
- monitor（8081）默认启用，卡片可启停；健康=存活即绿，ROS 状态看其页面右上
- 自定义指令（ultra_command）非可选、参与一键启动；nodes 型健康模式须同步 `ros_bridge.py` 的 `_node_patterns()`
- 新增组件照抄一条，`group` 决定启动顺序；健康检查四种：话题频率(默认)/采样型/master 探活/节点数(fms)
- simview 已移除（09-01）；start_l4.sh 仍会拉其终端，切回 HMI 前手动关

## 感知/规控数据录制（rosbag）

1. **选 topic**：改 `hmi/record_rostopic_list.md` 对应分组行尾 0→1（格式 `- /topic: 1`）；topic 须 `/` 开头绝对名、组内不重复；保存即生效无需重启；分组错误只影响本组
2. **启动**：卡片"启动"→校验→建目录→容量检查→时间戳命名→exec rosbag record；卡片转"运行"（健康=仅进程监控）
3. **停止**：SIGTERM 收尾 `.bag.active`——**必须先停再关机**，断电残留的 `.bag.active` 需人工恢复
4. **取文件**：`data/bags/perception/` 与 `data/bags/pnc/`，文件名=开始时间（毫秒防重名）

容量：每目录启动前检查，>2GiB 按修改时间删旧 `.bag`（`.bag.active`/他文件不动）；清后仍超则拒录；**录制中无上限**（长录控时长/只开必要 topic，点云优先 packets/低频 points）；旧 bag 会被自动清，需保留请及时转存。

FAQ：启动即故障=setup 没 source devel 或缺 rosbag｜分组全 0｜格式错｜清后仍超；运行无新文件=看 `hmi/logs/*_bags/`+磁盘；一键启动不含录制=预期。
（维护：逻辑在 record_rosbag.py，单测 tests/test_record_rosbag.py；目录/阈值/路径在脚本顶部。）

## 注意事项与容易疏忽的点

- 3D感知卡片现在有两个进程与两个健康话题，不能只看CenterPoint存活判断整卡正常。
- 规控录制要同时保留两个感知话题；组内开关独立，改完先核对所选组而非只搜同名topic。

1. 勿与 start_l4.sh 同用（抢 UDP/CAN/master）；start_l4.sh 仅救急，用前先"全部停止"
2. HMI 重启前先"全部停止"；不收养外来进程（橙色"外部"角标，须先停它才能再启）
3. 相机首次部署：rb_camera.sh 的 sudo 拷库需终端输一次密码（库持久保留）或配 NOPASSWD
4. hmi.sh 退出/SSH 断开不带走组件，需重开 HMI 执行停止
5. 无鉴权，严禁公网映射；"全部停止"≠硬件急停
6. 日志 `hmi/logs/<组件>/` 按启动分文件，轮转保留 3 份；磁盘 <1GB 告警、<500MB 清轮转

以下调试与测试命令在 `hmi/` 目录执行；工程启动命令则在工程根执行。

## 本机调试（无 ROS）

```bash
python3 hmi_server.py --config test_config.py --port 18080   # 假组件 sleepy/ticker/crasher
```

## 测试（开发机；车载不部署 tests/）

```bash
python3 tests/test_full.py                  # 80 项伪 ROS 全栈
node tests/frontend_test.js                 # 72 项前端无头
python3 -m unittest tests.test_record_rosbag
python3 tests/test_config_sync.py           # 配置↔代码同步守卫(nodes模式/组3/标题节点数)
python3 tests/chaos_test.py                 # 28 项混沌/浸泡
```

覆盖：频率健康/降级恢复/点云采样/master 重连/面板字段映射/HTTP 鲁棒/并发/坏配置/SIGKILL 升级/日志洪水/启停翻转无僵尸无泄漏。

## 文件

hmi.sh｜qingwei-hmi.service+install_autostart.sh｜hmi_config.py（组件配置）｜process_manager.py（状态机/停止链/轮转/编排）｜ros_bridge.py（rospy 桥）｜hmi_server.py（HTTP）｜static/index.html（前端）｜test_config.py｜tests/

## 围栏状态来源（2026-09-22）

车辆面板围栏告警改订阅 `/robot/task_plan/fence_guard` 的 `alarm`：0正常、1告警，-1未知。
超过1秒无消息也显示未知；旧 `/alarmcmd=0` 不能证明围栏模块存活。需要与新 robot 消息一起部署并重启 HMI。
独立验证：`python3 -B hmi/tests/test_fence_alarm.py`；前端验证：`node hmi/tests/frontend_test.js`。
