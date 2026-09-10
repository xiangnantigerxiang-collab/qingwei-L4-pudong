# -*- coding: utf-8 -*-
"""
monitor 配置(纯数据模块,无逻辑)
================================

由 monitor_server.py 加载。改动后重启 monitor 生效。

顶层键:
  PORT            HTTP 端口(默认 8081,与 hmi 8080 并存)
  MAP_PATH        地图目录(其下全部 .csv 按文件名排序依次全部绘制,
                  x,y,heading 三列无表头);$MON 替换为本目录;
                  rosparam /robot/mapfile 指向单文件时只画该张
  SCAN_EXTRINSICS 每路 2D 激光的安装外参(车体坐标系,米/度)。
                  工程内没有任何 laser->车体 的静态 tf,rviz 里这两路
                  从未正确落位;默认 0 需实车标定一次(README 有步骤)
  CLOUD           3D 点云通道与降采样参数(CPU/带宽控制的核心旋钮)
  LAYERS          前端图层默认开关(默认态照抄 robot.rviz)
  TYPED_SUBS      类型化订阅表(话题, 模块, 类名, 处理函数名)
"""

CONFIG = {
    # HTTP 服务
    "PORT": 8081,

    # dashboard 仪表板独立端口(同进程第二服务,展示 /can_msg 与
    # /ehb_msg 全部字段,中文名称取自 ehb_msg.msg 注释);
    # 环境变量 DASHBOARD_PORT 优先;置 0 关闭
    "DASHBOARD_PORT": 8082,

    # 地图文件。rosparam /robot/mapfile 存在时优先(rospy 可用才读)。
    "MAP_PATH": "$MON/map",

    # 2D 补盲激光:话题名 -> 车体安装外参 {x, y, yaw_deg}
    # 语义:scan 点先在传感器系极坐标->笛卡尔,再平移 (x,y)、旋转 yaw_deg。
    # 车尾后向安装典型值: x=负(车尾方向), yaw_deg=180(朝后扫)。
    "SCAN_EXTRINSICS": {
        "/back_left_scan":  {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
        "/back_right_scan": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
    },
    # 可选第三路前向补盲(rviz 未显示过;不需要就把整行删掉)
    "SCAN_EXTRAS": {
        # "/front_scan": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
    },

    # 3D 感知点云(4 路 RoboSense,车体局部坐标,CenterPoint 的输入)
    "CLOUD": {
        "topics": [
            "/rslidar_points_left",
            "/rslidar_points_right",
            "/rslidar_points_mid",
            "/rslidar_points_front",
        ],
        "parse_interval": 1.0,   # 每 topic 解析节拍(秒)。越大越省 CPU
        "stride": 2,             # 字节级跨步预抽稀:每 N 点取 1(解包前)
        "voxel": 0.2,            # 体素网格抽稀边长(米)
        "max_points_per_lidar": 8000,   # 每路上限(体素后仍超则截断)
        "activity_timeout": 30,  # cloud.bin 无浏览器拉取超过该秒数->退订省 CPU
        "z_min": -1.5,           # 过滤:低于该高度(米)的点丢弃(地面反射)
        "z_max": 3.0,            # 过滤:高于该高度(米)的点丢弃
    },

    # 前端图层默认开关(照抄 robot.rviz 的 Displays Value)
    "LAYERS": {
        "vehicle": True,      # 车辆矩形+中心点+速度文字
        "lidar": True,        # 感知障碍物 CUBE
        "routing": True,      # 路由路径(黄)
        "planning": False,    # 规划路径(绿)
        "loadpos": True,      # 托盘位置点
        "stoppose": True,     # 停车点箭头
        "map": False,         # 地图三线
        "scan": True,         # 2D 补盲激光两路
        "cloud": True,        # 3D 感知点云
        "grid": False,        # 参考网格
    },
}
