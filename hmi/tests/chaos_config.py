# -*- coding: utf-8 -*-
"""
混沌测试配置:一组"敌对"组件,专打进程管理的薄弱路径。
  gate     命令不存在 → Popen 直接失败 → CRASHED → 阻断一键启动(非 optional)
  stubborn 忽略 SIGTERM → 停止链必须升级到 SIGKILL
  breeder  同组孙进程 ×2 → killpg 必须全数收割
  daemon   setsid 逃离进程组 → 组杀无效 → 依赖 stop_pat 残留检测(stop_failed)
  grower   日志洪水(限速)→ 轮转 + 并发日志读取
  flapper  普通长驻进程 → 快速启停翻转 / foreign 检测与清理
"""

CONFIG = {
    "groups": {0: "门控", 1: "敌对子进程", 2: "负载"},
    "defaults": {
        "health": [],
        "start_timeout": 10,
        "health_lost_s": 3,
        "max_log_mb": 50,
        "keep_logs": 3,
        "ready_s": 1.5,
        "sample_period": 30,
    },
    "components": [
        {
            "name": "gate",
            "title": "坏命令(非可选)",
            "group": 0,
            "cmd": ["/nonexistent/prog"],
            "cwd": "$ROOT",
        },
        {
            "name": "stubborn",
            "title": "无视SIGTERM",
            "group": 1,
            "cmd": ["bash", "-c", "trap '' TERM; while true; do sleep 1; done"],
            "cwd": "$ROOT",
            "stop_pat": "trap '' TERM",
        },
        {
            "name": "breeder",
            "title": "同组孙进程",
            "group": 1,
            "cmd": ["bash", "-c", "sleep 310 & sleep 311 & wait"],
            "cwd": "$ROOT",
            "stop_pat": "sleep 31",
        },
        {
            "name": "daemon",
            "title": "逃离进程组",
            "group": 1,
            "cmd": ["bash", "-c",
                    "setsid sleep 600 >/dev/null 2>&1 & "
                    "while true; do sleep 1; done"],
            "cwd": "$ROOT",
            "stop_pat": "sleep 600",
        },
        {
            "name": "grower",
            "title": "日志洪水",
            "group": 2,
            "cmd": ["bash", "-c",
                    "while true; do head -c 65536 /dev/zero | tr '\\0' 'x'; "
                    "sleep 0.2; done"],
            "cwd": "$ROOT",
            "max_log_bytes": 200000,
            "stop_pat": "head -c 65536",
        },
        {
            "name": "flapper",
            "title": "翻转靶",
            "group": 2,
            "cmd": ["sleep", "900"],
            "cwd": "$ROOT",
            "stop_pat": "sleep 900",
        },
    ],
}
