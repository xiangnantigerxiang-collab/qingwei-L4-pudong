# -*- coding: utf-8 -*-
"""伪 ROS 测试配置(对标 hmi/tests/ros_test_config.py)。

与车载 monitor_config.py 的差异:
- SCAN_EXTRINSICS 给了已知外参(验证旋转变换):x=+1m, yaw=90°
- CLOUD stride=1 / voxel=0(关体素,便于确定性断言)/ parse_interval 0.3s 加速
- 其余话题与默认一致
"""

CONFIG = {
    "PORT": 18081,
    "MAP_PATH": "$MON/map/view.csv",
    "SCAN_EXTRINSICS": {
        "/back_left_scan":  {"x": 1.0, "y": 0.0, "yaw_deg": 90.0},
        "/back_right_scan": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
    },
    "SCAN_EXTRAS": {},
    "CLOUD": {
        "topics": ["/rslidar_points_mid"],
        "parse_interval": 0.3,
        "stride": 1,
        "voxel": 0.0,
        "max_points_per_lidar": 4,
        "activity_timeout": 30,
        "z_min": -1.5,
        "z_max": 3.0,
    },
    "LAYERS": {
        "vehicle": True, "lidar": True, "routing": True, "planning": False,
        "loadpos": True, "stoppose": True, "map": False, "scan": True,
        "cloud": True, "grid": False,
    },
}
