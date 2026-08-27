# -*- coding: utf-8 -*-
"""
伪 ROS 环境下的被测配置:小超时/小采样周期,覆盖全部四种健康检查形态。
组件进程本身只是 sleep 占位——真实数据由 mock_ros 的话题泵提供。
"""

CONFIG = {
    "groups": {0: "核心", 1: "被测"},
    "defaults": {
        "health": [],
        "start_timeout": 15,
        "health_lost_s": 3,
        "max_log_mb": 50,
        "keep_logs": 3,
        "ready_s": 1.0,
        "sample_period": 2,
    },
    "components": [
        {
            "name": "roscore",
            "title": "ROS 核心(master 健康型)",
            "group": 0,
            "cmd": ["sleep", "600"],
            "cwd": "$ROOT",
            "health": [{"type": "master"}],
            "health_lost_s": 4,
        },
        {
            "name": "pub",
            "title": "话题源(频率+采样+节点数)",
            "group": 1,
            "cmd": ["sleep", "600"],
            "cwd": "$ROOT",
            "health": [
                {"topic": "/can_msg", "min_hz": 20},
                {"topic": "/rslidar_points_mid", "min_hz": 5,
                 "sampled": True, "sample_period": 2},
                {"type": "nodes", "pattern": "/cloud", "min": 2},
            ],
        },
    ],
}
