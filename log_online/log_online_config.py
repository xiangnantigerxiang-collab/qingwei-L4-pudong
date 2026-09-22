# -*- coding: utf-8 -*-
"""log_online 配置。所有键带默认值，车载可直接改本文件。"""

VEHICLE_ID = "A03"

TOPIC_CAN = "/can_msg"
TOPIC_NAVIGATION = "/navigation_msg"
TOPIC_PERCEPTION = "/perception"
TOPIC_PLAN = "/plan_path_msg"
TOPIC_CONTROL = "/control_msg"
# cam_geac ros1_jpg 按 cam{n}/compressed 相对名发布;实车 rostopic list 核对,空串=视频状态禁用
TOPIC_CAMERA = "/cam0/compressed"

PARAM_SENSORSTATE = "/planning/sensorstate"
PARAM_NETCHECK = "/robot/planning/netcheck"
PARAM_ULTRA_SAFE = "/ultra/status/safe"
PARAM_LIGHT = "/canbus/light"
PARAM_HORN = "/canbus/horn"

SAMPLE_HZ = 10                 # 快照频率
RING_SECONDS = 120.0           # 环形缓冲时长(需求90s+30s余量)
PRE_WINDOW_S = 90.0            # 事件前窗口
POST_WINDOW_S = 90.0           # 事件后窗口
DEBOUNCE_TICKS = 5             # 触发去抖(0.5s@10Hz)
FLUSH_INTERVAL_S = 1.0         # 落盘 flush 周期
DISK_CAP_BYTES = 1073741824    # 事件数据磁盘上限(1GiB)
ROTATE_KEEP_RATIO = 0.9        # 轮转目标水位
MAX_OBJECTS = 64               # 单快照障碍物上限
STALE_AFTER_S = 2.0            # 链路 stale 阈值
ACCEL_EMA_S = 0.5              # 自算加速度 EMA 窗口
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8083               # 状态页(monitor 已占 8081/8082)
