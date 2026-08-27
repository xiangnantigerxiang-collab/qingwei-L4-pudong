# -*- coding: utf-8 -*-
"""
HMI 本机测试配置(无 ROS 环境端到端验证用)
==========================================

三个假组件覆盖状态机关键路径:
  sleepy   常驻进程,无健康检查 → 验证 STOPPED→STARTING→RUNNING 与 kill 后 CRASHED
  ticker   周期输出 + touch 心跳文件(file 型健康)→ 验证健康判定/DEGRADED/恢复/日志轮转
  crasher  启动即退(optional)→ 验证 CRASHED 字段、一键启动不被 optional 失败阻断

用法:python3 hmi_server.py --config test_config.py --port 18080
"""

CONFIG = {
    "groups": {0: "核心", 1: "假组件"},
    "defaults": {
        "start_timeout": 10,
        "health_lost_s": 4,
        "max_log_mb": 50,
        "keep_logs": 3,
        "ready_s": 1.5,
        "sample_period": 30,
    },
    "components": [
        {
            "name": "sleepy",
            "title": "常驻进程",
            "group": 0,
            "cmd": ["sleep", "3600"],
            "cwd": "$ROOT",
            "health": [],
            "start_timeout": 10,
        },
        {
            "name": "ticker",
            "title": "周期心跳",
            "group": 1,
            "cmd": ["bash", "-c",
                    "while true; do echo \"beat $(date +%H:%M:%S.%N | cut -c1-11)\"; "
                    "touch \"$HMI_VARDIR/ticker.beat\"; sleep 0.2; done"],
            "cwd": "$ROOT",
            "health": [{"type": "file", "path": "$VARDIR/ticker.beat", "max_age": 2.0}],
            "start_timeout": 8,
            "health_lost_s": 4,
            "max_log_bytes": 4000,   # 5 行/秒输出,约 10s 触发一次轮转,便于验证
        },
        {
            "name": "crasher",
            "title": "启动即崩",
            "group": 1,
            "optional": True,
            "cmd": ["bash", "-c", "echo boom; exit 1"],
            "cwd": "$ROOT",
            "health": [],
            "start_timeout": 5,
        },
    ],
}
