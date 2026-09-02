# qingwei L4 Web HMI

替代 `start_l4.sh` 的车端 Web 人机界面:一键启动/停止、组件健康状态、日志查看、
车辆实时状态面板。测试人员用浏览器操作,无需终端命令。

- **零第三方依赖**:仅 Python 标准库 + rospy(车载自带),不需要 pip 安装任何东西
- **不是 catkin 包**:不参与编译,部署 = 拷贝目录
- **不改动现有工程**:所有业务代码/脚本零改动,`start_l4.sh` 原样保留

## 部署(车载 Jetson Orin)

```bash
# 1. 把整个 hmi/ 目录拷到工程根目录(与 start_l4.sh 同级)
scp -r hmi/ nvidia@<orin-ip>:/home/nvidia/qingwei-L4-No2/

# 2. 车端启动
bash ~/qingwei-L4-No2/hmi/hmi.sh            # 默认 0.0.0.0:8080

# 3. 测试人员浏览器访问
http://<orin-ip>:8080
```

端口被占用时:`HMI_PORT=8090 bash hmi/hmi.sh`。
若平板/笔记本打不开页面,检查 Orin 防火墙是否放行 8080 端口。

### 开机自启动(可选,systemd)

```bash
bash hmi/install_autostart.sh           # 安装并启动服务 qingwei-hmi
bash hmi/install_autostart.sh remove    # 卸载自启,恢复手动方式
```

- 自启的**只有网页服务**,车辆组件不会被自动拉起,上电后仍需页面手动"一键启动"
- 安装脚本会幂等写入 `/etc/environment` 的 `MOZ_X11_EGL=1`,解决 Jetson/L4T
  上 Firefox 创建 WebGL context 失败的问题;首次安装后需注销重新登录或重启车机
  才能让桌面启动的 Firefox继承。卸载 HMI 自启时保留该车机级兼容配置
- `systemctl stop/restart qingwei-hmi` 只停 HMI 本身,组件不受影响
  (unit 用 `KillMode=process`,勿改为默认值——否则停服务会连带杀掉全部组件)
- HMI 服务日志:`journalctl -u qingwei-hmi -f`;组件日志仍在 `hmi/logs/`
- 装了自启就**不要再手动** `bash hmi/hmi.sh`(8080 冲突);换端口:
  `HMI_PORT=8090 bash hmi/install_autostart.sh` 或改 unit 后 daemon-reload

## 页面说明

- **一键启动**:按分组顺序拉起全部"已启用"组件(核心平台→传感器驱动→感知挂接→
  规划控制→业务辅助),组内并行,组间等健康检查通过;非可选组件失败会中止后续组
  并在横幅提示。再次点击等同于"补链"(已运行的跳过)
- **全部停止**:逆序停止,需在弹窗中二次确认
- **组件卡片**:状态灯(绿=运行 / 黄=降级 / 红=故障 / 蓝=启动或停止中 / 灰=已停止),
  健康详情(如 `/can_msg 49.8Hz`),可单独启动/停止/重启/看日志
- **车辆状态面板**:车速(里程表大数字)、挡位、自动/手动模式、急停、任务执行状态、
  RTK 定位、障碍物距离、横向偏差、电量、挂接状态、传感器健康、CAN/网络状态;
  字段超 5 秒无数据会变灰划线(数据冻结而非清零,便于判断是哪一环断了)
- **脱挂钩一键标定**:左侧"业务与辅助"分组里的"脱挂钩标定"组件卡(与外网监测
  同形态:状态点/当前行程/一键标定与日志按钮),点击"一键标定"触发 canbus 的
  hook/pallet 行程标定(`rosparam /canbus/calibration/hook`)。触发前检查
  自动驾驶模式 + N 档 + 车速≈0 + CAN 数据新鲜,不满足弹提醒
  "一键标定前请停车挂N档，切换到自动驾驶模式"(不触发);完成后弹
  "标定已完成，当前行程为销子：xxx(min)-xxx(max)  托盘：xxx(min)-xxx(max)",
  失败/超时弹原因。卡片常显当前行程(读 config.cfg)

## 组件配置

`hmi_config.py` 纯 Python 数据模块,每个组件定义命令/目录/健康检查/超时/停止方式,
全中文注释。调整示例:

- 启用 netcheck:把对应组件 `"enabled": False` 改为 `True`
- **行车记录**(`data_logger`)固定不参与一键启动，需要时从卡片手动启动
- **感知数据录制/规控数据录制**分别位于“感知与挂接”和“规划控制”分组，均固定
  不参与一键启动；两个组件各自读取 `record_rostopic_list.md` 对应分组的 0/1 开关
- **可视化(Web)**(monitor,默认启用):卡片可单独启停 monitor 服务,
  浏览器访问 `http://<车IP>:8081`;健康判定为存活即绿(Web 服务无
  ROS 发布话题),ROS/master 连接状态看 monitor 页面右上状态点
- 新增组件:照抄一条,`group` 决定启动顺序

(RViz 可视化组件 simview 已按需求移除(09-01)。注意:`start_l4.sh`
救急脚本仍会启动 simview 的旧终端,且 HMI 的"全部停止"不再管理该进程——
用 start_l4.sh 后切换回 HMI 前,请手动关闭 simview 的 gnome-terminal。)

健康检查四种形态:话题频率(默认)、采样型(点云,省 CPU)、master 探活(roscore)、
节点数统计(fms)。

## 感知/规控数据录制(rosbag)使用方法

页面提供两个按需手动启动的卡片:

- “感知与挂接”栏里的**感知数据录制**
- “规划控制”栏里的**规控数据录制**

两者都不参与一键启动，可以独立或同时录制。原“业务与辅助”栏里的“数据录制”组件
已删除。完整流程:

1. **选 topic**:编辑 `hmi/record_rostopic_list.md`。文件包含“感知数据录制”和
   “规控数据录制”两个顶级分组，每组都有完整 topic 清单；在目标分组里把要录制的
   topic 行尾开关从 `0` 改为 `1`。同一 topic 可在两个分组里分别设置，互不影响。
   保存后**无需重启 HMI**，每次手动启动录制都会重新读取。格式:

   ```
   - /can_msg: 1        # 录制
   - /rosout: 0         # 不录制
   ```

   规则:topic 必须是以 `/` 开头的绝对名;同一 topic 在同一个录制分组中只能出现
   一次;开关只认 `0`/`1`。启动时只校验所选分组，因此另一个分组临时写错不会阻断
   本次录制；所选分组缺失、重复、缺少 topic 前导 `/`、格式错误或一个都没开时
   拒绝启动。启动日志会以非阻断警告提示另一组错误/清单漂移，专项测试还会严格
   校验两个分组都存在、标题不重复且完整 topic 清单一致。
2. **启动**:点所需录制卡片上的**启动**。启动器(`record_rosbag.py`)选择对应分组，
   先校验 topic 和 rosbag 命令 → 建独立目录 → 检查容量 → 按当前时间戳命名 →
   `exec rosbag record`。配置或 ROS 环境有误时不会清理任何历史 bag。
   正常后卡片转"运行",健康行显示"仅进程监控"(录制进程无健康话题)。
3. **停止**:点卡片上的**停止**。rosbag 收到 SIGTERM 后会把进行中的
   `.bag.active` 收尾成完整 `.bag` 再退出——**务必先停止再关机/重启 HMI**,
   直接断电会留下未收尾的 `.bag.active`。启动器不会自动删除它，需由工程人员确认
   无进程占用后进行恢复或手动清理。
4. **取文件**:感知 bag 在 `data/bags/perception/`，规控 bag 在 `data/bags/pnc/`；
   文件名即录制开始时间(如 `20260901_153012_345.bag`,毫秒级防重名)。

**容量规则**:两个输出目录分别在每次启动前检查，单目录超过 **2 GiB** 时按修改
时间从旧到新删除已完成的 `.bag`，达到限制即停止；`.bag.active`、其它目录和其它
文件一律不自动删除。清理后仍超(例如 `.bag.active` 或非 bag 文件过大)则拒绝该项
录制并报错。独立目录也保证同时录制时互不误停、互不清理。
注意两点:
- 是**启动时**检查,录制过程中不设上限——长时间高码率 topic(如 4 路 32 线点云)
  可能写满磁盘,长录请控制时长或只开必要 topic(点云类建议优先用 packets/低频
  points,或参考文件里现成的开关组合);
- 自动清理仍会删除旧数据，需要长期保留的 bag 请及时转存。

**常见问题**:

| 现象 | 原因与处理 |
|---|---|
| 启动即"故障",卡片错误提示找不到 rosbag | 组件 `setup` 没 source devel 环境，或 ROS 未安装 |
| 启动即"故障",提示分组没有启用 topic | 对应分组的开关全部为 0——改 `record_rostopic_list.md` 后再启动 |
| 启动即"故障",提示缺少分组/格式错误/topic 重复 | 按提示修改当前录制分组；另一分组不会影响本次启动 |
| 启动即"故障",提示清理后仍超过限制 | 检查对应目录中的 `.bag.active` 或超量非 bag 文件，确认无进程占用后处理 |
| 卡片"运行"但没有新文件 | 查看该卡日志(`hmi/logs/perception_bags/` 或 `hmi/logs/pnc_bags/`)并确认磁盘未满 |
| 一键启动后录制或行车记录没有启动 | 这是预期行为；三项录制功能都必须从各自卡片手动启动 |

(维护者注:录制逻辑在 `hmi/record_rosbag.py`,单测 `tests/test_record_rosbag.py`;
输出根目录、2 GiB 单目录阈值、配置文件路径和两个录制类型都在该脚本顶部定义。)

## 已知事项与注意事项

1. **勿与 start_l4.sh 同时使用**:两套同时拉起会抢 UDP 端口/CAN/master,行为未定义。
   HMI 是日常入口;start_l4.sh 仅留作工程师救急(用前先在 HMI 里全部停止)
2. **HMI 重启前先"全部停止"**:HMI 不收养自己没启动的进程。若 HMI 重启时组件还在跑,
   卡片会标橙色"外部"角标并禁用启动/重启，必须先点该卡“停止”清理，或执行“全部
   停止”统一清理，之后才能重新启动，避免同一组件叠加运行
3. **相机首次部署**:rb_camera.sh 前段有 `sudo cp`/`modprobe`(拷编解码库到 /usr/lib)。
   HMI 无终端环境,sudo 无法交互输密码。库拷贝成功过一次后会持久保留;若相机启动日志
   出现 sudo 报错且无图像,请在车端终端手动跑一次 `./rb_camera.sh ros1_jpg`(输一次密码)
   或为 nvidia 用户配置这些命令的 NOPASSWD
4. **HMI 自身退出不带走组件**:关闭 hmi.sh 进程(或 SSH 断开)后组件继续运行,
   需重新打开 HMI 执行"全部停止"或在车端手动处理
5. **无鉴权**:仅限车载内网使用,**严禁**将端口映射到公网
6. **"全部停止"是软件停机**:不等同硬件急停。行驶中紧急情况请拍车载急停按钮
7. 日志在 `hmi/logs/<组件名>/` 下,按启动分文件,单文件超上限自动轮转(保留 3 份);
   磁盘剩余 <1GB 页面告警,<500MB 自动清理轮转副本

## 本机(无 ROS)调试

```bash
cd hmi
python3 hmi_server.py --config test_config.py --port 18080
# 浏览器打开 http://localhost:18080
# 三个假组件:sleepy(常驻)/ ticker(周期+文件心跳)/ crasher(启动即崩)
```

无 ROS 环境时车辆状态面板显示"无 ROS 环境",进程管理与日志功能正常——
便于在开发机上调试界面和编排逻辑。

## 文件结构

| 文件 | 职责 |
|---|---|
| `hmi.sh` | 启动包装(source devel → python3) |
| `qingwei-hmi.service` + `install_autostart.sh` | systemd 开机自启(安装/卸载) |
| `hmi_config.py` | 车载组件配置(分组/命令/健康检查) |
| `process_manager.py` | 进程状态机、停止链、日志轮转、分组编排 |
| `ros_bridge.py` | rospy 桥:健康频率统计、车辆状态聚合、master 探活 |
| `hmi_server.py` | HTTP 入口、系统资源监控、API 路由 |
| `static/index.html` | 前端单页(原生 JS) |
| `test_config.py` | 本机测试假组件 |
| `tests/` | 伪 ROS 全栈校验(见下) |

## 伪 ROS 全栈校验(开发机用)

`tests/` 下是一套模拟 ROS 环境:伪 rospy 模块、伪消息包、可编程话题泵
(控制文件调话题速率/字段/参数)、伪 ROS master(XML-RPC,pid 可变可停启)。
让 ros_bridge 的全部 ROS 侧代码在无 ROS 的开发机上可运行、可验证:

```bash
cd hmi && python3 tests/test_full.py                 # 80 项断言,约 2~3 分钟
node tests/frontend_test.js                          # 66 项断言(需 Node)
python3 -m unittest tests.test_record_rosbag         # rosbag 分组与启动器单元测试
```

覆盖:话题频率健康与降级恢复、点云采样探测、fms 节点数统计、master 重启
重连与失联告警、车辆面板全部字段映射(挡位/障碍物 100/200 编码/横向偏差
阈值/挂接文案/传感器位图/参数轮询)、HTTP 鲁棒性、并发压力、坏配置拒绝启动。

另两套(同为开发机用):

```bash
node tests/frontend_test.js      # 前端无头渲染:66 项(DOM 桩驱动真实页面脚本)
python3 tests/chaos_test.py      # 混沌/浸泡:28 项(敌对子进程/日志洪水/服务器暴毙/fd与RSS)
```

混沌套件覆盖:无视 SIGTERM 的 SIGKILL 升级、同组孙进程组杀完整性、
setsid 脱组残留检测(stop_failed)、日志洪水下轮转与并发读取、服务器 SIGKILL
后重启的 foreign 标记与清理、15 次快速启停翻转无僵尸无 fd 泄漏、
停止进行中操作 409、60s 浸泡(RSS/fd 零增长)、磁盘两级水位。
**车载不部署 tests/ 目录**(仅开发用)。
