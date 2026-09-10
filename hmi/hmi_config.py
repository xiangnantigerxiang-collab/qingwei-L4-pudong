# -*- coding: utf-8 -*-
"""
HMI 组件配置(车载正式配置)
================================

纯数据模块,被 hmi_server.py 加载。所有路径支持两个占位符:
  $ROOT   —— 工程根目录(hmi/ 的上一级,本机/车载自动适配)
  $VARDIR —— hmi/var/ 运行时目录(会被替换为绝对路径)

字段说明:
  name/title    组件标识(英文)/界面显示名(中文)
  group         启动分组,见 groups;组内并行、组间按健康门控顺序启动,停止逆序
  cmd           启动命令(list 形式,不走 shell;需要 shell 特性时显式写 ["bash","-c","..."])
  cwd           工作目录(相机/CenterPoint 有严格的目录依赖,不可改)
  setup         进 bash 前要 source 的文件列表(如 devel/setup.bash);为空则直接执行 cmd
  health        健康检查规格列表,四种形态:
                  {"topic": "/xxx", "min_hz": N}          连续话题,频率滑窗判定
                  {"topic": "/xxx", "min_hz": N, "sampled": True}  点云类,间歇采样判定(省 CPU)
                  {"type": "nodes", "pattern": "/cloud", "min": 5} master 节点表计数(fms)
                  {"type": "master"}                       roscore 专用
                  {"type": "file", "path": "...", "max_age": 秒}  文件 mtime 判定(仅测试用)
  start_timeout 启动后多久仍未达健康则降级 DEGRADED(秒)
  health_lost_s 健康连续丢失多久降级 DEGRADED(秒)
  max_log_mb    单个日志文件上限,超过轮转(保留 .1/.2/.3)
  stop_cmd      停止前的自定义清理命令(shell 执行,用于杀 root 子进程)
  stop_pat      进程特征串(pgrep -f 用):停止后残留检测 + HMI 启动时外部进程识别
  optional      一键启动中失败不阻断后续组
  enabled       默认 False 的组件不参与一键启动,界面标“手动启动”,可单独启动

启动分组依据(start_l4.sh 原始顺序 + 依赖修正):
  原脚本把 lidar_perception 放在第 1 位、auto_couple 在第 7 位,前者订阅后者发布的
  /back_pointcloud,靠 ROS 晚加入机制碰运气;本配置把 lidar_perception 挪到组 3
  (auto_couple 之后),其余顺序与原脚本一致。roscore 原本由第 1 个 roslaunch 隐式
  拉起(该 roslaunch 退出会连带杀掉 master),这里改为显式独立组件。
"""

CONFIG = {
    "groups": {
        0: "核心平台",
        1: "传感器驱动",
        2: "感知与挂接",
        3: "规划控制",
        4: "业务与辅助",
    },
    "defaults": {
        "start_timeout": 40,
        "health": [],       # 缺省无健康检查(新增组件漏写时不致 KeyError)
        "health_lost_s": 10,
        "max_log_mb": 50,
        "keep_logs": 3,
        "ready_s": 1.5,        # 无健康检查的组件,存活该时长后即视为 RUNNING
        "sample_period": 30,   # 采样型话题的重订阅周期(秒)
    },
    "components": [
        # ---------- 组 0:核心平台 ----------
        {
            "name": "roscore",
            "title": "ROS 核心",
            "group": 0,
            "cmd": ["roscore"],
            "cwd": "$ROOT",
            "health": [{"type": "master"}],
            "start_timeout": 25,
            "stop_pat": "rosmaster",
        },
        # ---------- 组 1:传感器驱动 ----------
        {
            "name": "rslidar",
            "title": "激光雷达×5",
            "group": 1,
            # 与 start_l4.sh 第 10 行一致(相对 ./launch/,工作目录必须是工程根)
            "cmd": ["roslaunch", "./launch/start.launch"],
            "cwd": "$ROOT",
            "setup": ["$ROOT/devel/setup.bash"],
            # 点云数据量大,用间歇采样判定存活(每 30s 订阅一次收首条即退订)
            "health": [
                {"topic": "/rslidar_points_mid", "min_hz": 5, "sampled": True},
                {"topic": "/rslidar_points_left", "min_hz": 5, "sampled": True},
                {"topic": "/rslidar_points_right", "min_hz": 5, "sampled": True},
                {"topic": "/rslidar_points_front", "min_hz": 5, "sampled": True},
            ],
            "stop_pat": "rslidar_sdk",
        },
        {
            "name": "ins",
            "title": "RTK/惯导定位",
            "group": 1,
            "cmd": ["roslaunch", "ins", "demo.launch"],
            "cwd": "$ROOT",
            "setup": ["$ROOT/devel/setup.bash"],
            "health": [{"topic": "/localization", "min_hz": 10}],
            "stop_pat": "ins demo.launch",
            # 该 launch 内节点 required=true:串口异常时 roslaunch 整体退出 → 立即 CRASHED
        },
        {
            "name": "canbus",
            "title": "CAN 总线",
            "group": 1,
            # canbus.sh 内部自带 sudo modprobe/can0 配置和 cd,原样调用
            "cmd": ["bash", "$ROOT/launch/canbus.sh"],
            "cwd": "$ROOT",
            "setup": ["$ROOT/devel/setup.bash"],
            "health": [
                # /can_msg 由 canbus T2(20Hz)发布(09-07 晚用户自 T1 移回,恢复基线编排);
                # min_hz=20 → 有效门槛 16Hz(0.8 容差)。发布频率若再调整必须同步本值
                # ——09-07 实车"CAN 总线启动超时"即阈值与实际频率漂移所致(workflow 09-07 日志)
                {"topic": "/can_msg", "min_hz": 20},
                {"topic": "/can_recv", "min_hz": 20},
            ],
            "start_timeout": 30,
            "stop_pat": "canbus.launch",
        },
        {
            "name": "camera",
            "title": "相机",
            "group": 1,
            # 脚本全部相对路径,cwd 不可改;不 source devel(直连 /opt/ros)
            "cmd": ["./rb_camera.sh", "ros1_jpg"],
            "cwd": "$ROOT/src/driver/cam_geac",
            "health": [{"topic": "/cam0/compressed", "min_hz": 5}],
            "health_lost_s": 15,
            "max_log_mb": 20,
            "stop_pat": "ros1_jpg_demo",
            # demo 二进制可能以 root 残留,组杀杀不掉时用 sudo pkill 兜底
            # [o] 写法避免 pkill 匹配到自身的 bash 包装
            "stop_cmd": "echo 'nvidia' | sudo -S pkill -f 'ros1_jpg_dem[o]' || true",
        },
        # ---------- 组 2:感知与挂接 ----------
        {
            "name": "centerpoint",
            "title": "3D 感知",
            "group": 2,
            # 裸可执行,不 source;cwd 必须是 build/(模型相对路径 ../model/)
            "cmd": ["./centerpoint_ros_node"],
            "cwd": "$ROOT/src/CUDA-CenterPoint/build",
            "health": [{"topic": "/box", "min_hz": 2}],
            "start_timeout": 120,   # TensorRT 模型加载慢
            "stop_pat": "centerpoint_ros_node",
        },
        {
            "name": "auto_couple",
            "title": "自动挂接+2D雷达",
            "group": 2,
            "cmd": ["roslaunch", "./launch/start_auto_couple.launch"],
            "cwd": "$ROOT",
            "setup": ["$ROOT/devel/setup.bash"],
            "health": [
                {"topic": "/hook_position", "min_hz": 1},
                {"topic": "/back_pointcloud", "min_hz": 2, "sampled": True},
            ],
            "stop_pat": "start_auto_couple.launch",
        },
        {
            "name": "perception_bags",
            "title": "感知数据录制",
            "group": 2,
            "optional": True,
            # 录制是按需人工操作，不参与一键启动。每次手动启动都会重新读取
            # record_rostopic_list.md 中“感知数据录制”分组的 0/1 开关。
            "enabled": False,
            "cmd": ["python3", "$ROOT/hmi/record_rosbag.py", "perception"],
            "cwd": "$ROOT",
            "setup": ["$ROOT/devel/setup.bash"],
            "health": [],
            "max_log_mb": 5,
            # 启动器参数覆盖 exec 前窗口；exec 后只匹配 rosbag record 的
            # 完整命令结构和本组件输出目录，避免误杀 scp/rsync/rosbag info。
            "stop_pat": (
                "record_rosbag[.]py perception( |$)|"
                "/rosbag record -O [^ ]*/data/bags/perception/"
            ),
        },
        # ---------- 组 3:规划控制 ----------
        {
            "name": "lidar_perception",
            "title": "2D 补盲感知",
            "group": 3,
            # 顺序修正:必须在 auto_couple(发布 /back_pointcloud)之后启动
            "cmd": ["roslaunch", "lidar_perception", "lidar_perception.launch"],
            "cwd": "$ROOT",
            "setup": ["$ROOT/devel/setup.bash"],
            "health": [{"topic": "/perception_back_bbox", "min_hz": 2}],
            "stop_pat": "lidar_perception.launch",
        },
        {
            "name": "pnc",
            "title": "规划控制(6节点)",
            "group": 3,
            "cmd": ["roslaunch", "./launch/control.launch"],
            "cwd": "$ROOT",
            "setup": ["$ROOT/devel/setup.bash"],
            "health": [
                {"topic": "/navigation_msg", "min_hz": 20},
                {"topic": "/control_msg", "min_hz": 8},
            ],
            "start_timeout": 60,
            "stop_pat": "launch/control.launch",
        },
        {
            "name": "pnc_bags",
            "title": "规控数据录制",
            "group": 3,
            "optional": True,
            # 录制是按需人工操作，不参与一键启动。每次手动启动都会重新读取
            # record_rostopic_list.md 中“规控数据录制”分组的 0/1 开关。
            "enabled": False,
            "cmd": ["python3", "$ROOT/hmi/record_rosbag.py", "pnc"],
            "cwd": "$ROOT",
            "setup": ["$ROOT/devel/setup.bash"],
            "health": [],
            "max_log_mb": 5,
            # 同上：同时匹配 Python 启动阶段和最终 rosbag record 进程。
            "stop_pat": (
                "record_rosbag[.]py pnc( |$)|"
                "/rosbag record -O [^ ]*/data/bags/pnc/"
            ),
        },
        # ---------- 组 4:业务与辅助 ----------
        {
            "name": "fms",
            "title": "云端网关",
            "group": 4,
            "cmd": ["./fms.sh"],
            "cwd": "$ROOT",
            # 双重环境:catkin + fms_agent venv(与 fms.sh 顺序一致)
            "setup": [
                "$ROOT/devel/setup.bash",
                "$ROOT/src/fms_agent/env/bin/activate",
            ],
            # fms_agent.launch 实际 6 个 /cloud 前缀节点,阈值 5 允许坏 1 个
            "health": [{"type": "nodes", "pattern": "/cloud", "min": 5}],
            "stop_pat": "fms_agent.launch",
        },
        {
            "name": "monitor",
            "title": "可视化(Web)",
            "group": 4,
            "optional": True,
            "enabled": True,
            # monitor Web 可视化(浏览器访问 http://<车IP>:8081, 与 hmi 8080 并存)
            # monitor.sh 自行 source devel 并自定位目录,无需 setup
            "cmd": ["bash", "$ROOT/monitor/monitor.sh"],
            "cwd": "$ROOT",
            "health": [],   # Web 服务无 ROS 发布话题:存活即绿;
                            # ROS/master 连接状态由页面右上状态点自示
            "start_timeout": 15,
            "stop_pat": "monitor_server",
        },
        {
            "name": "netcheck",
            "title": "外网监测",
            "group": 4,
            "optional": True,
            "enabled": False,
            # 原 start_l4.sh 中被注释;启用后车辆状态面板"网络"才有数据
            "cmd": ["bash", "$ROOT/launch/netcheck.sh"],
            "cwd": "$ROOT",
            "setup": ["$ROOT/devel/setup.bash"],
            "health": [],
            "stop_pat": "netcheck.sh",
        },
    ],
}
