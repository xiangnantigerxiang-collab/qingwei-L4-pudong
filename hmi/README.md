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

## 组件配置

`hmi_config.py` 纯 Python 数据模块,每个组件定义命令/目录/健康检查/超时/停止方式,
全中文注释。调整示例:

- 启用 simview/netcheck/bags:把对应组件 `"enabled": False` 改为 `True`
- simview 工作区路径不同:改其 `cwd` 与 `setup` 字段
- 新增组件:照抄一条,`group` 决定启动顺序

健康检查四种形态:话题频率(默认)、采样型(点云,省 CPU)、master 探活(roscore)、
节点数统计(fms)。

## 已知事项与注意事项

1. **勿与 start_l4.sh 同时使用**:两套同时拉起会抢 UDP 端口/CAN/master,行为未定义。
   HMI 是日常入口;start_l4.sh 仅留作工程师救急(用前先在 HMI 里全部停止)
2. **HMI 重启前先"全部停止"**:HMI 不收养自己没启动的进程。若 HMI 重启时组件还在跑,
   卡片会标橙色"外部"角标并提示;此时状态判断可能不准,建议全部停止后由 HMI 重新拉起
   ("全部停止"会先清理外部进程再逆序停止;对单个外部进程也可点其卡片上的"停止"清理)
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
cd hmi && python3 tests/test_full.py    # 47 项断言,约 2~3 分钟
```

覆盖:话题频率健康与降级恢复、点云采样探测、fms 节点数统计、master 重启
重连与失联告警、车辆面板全部字段映射(挡位/障碍物 100/200 编码/横向偏差
阈值/挂接文案/传感器位图/参数轮询)、HTTP 鲁棒性、并发压力、坏配置拒绝启动。

另两套(同为开发机用):

```bash
node tests/frontend_test.js      # 前端无头渲染:33 项(DOM 桩驱动真实页面脚本)
python3 tests/chaos_test.py      # 混沌/浸泡:28 项(敌对子进程/日志洪水/服务器暴毙/fd与RSS)
```

混沌套件覆盖:无视 SIGTERM 的 SIGKILL 升级、同组孙进程组杀完整性、
setsid 脱组残留检测(stop_failed)、日志洪水下轮转与并发读取、服务器 SIGKILL
后重启的 foreign 标记与清理、15 次快速启停翻转无僵尸无 fd 泄漏、
停止进行中操作 409、60s 浸泡(RSS/fd 零增长)、磁盘两级水位。
**车载不部署 tests/ 目录**(仅开发用)。
