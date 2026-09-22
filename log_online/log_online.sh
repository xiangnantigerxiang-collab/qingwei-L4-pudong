#!/bin/bash
# log_online 启动包装(对标 monitor/monitor.sh)
# 用法: bash log_online/log_online.sh   (默认 0.0.0.0:8083,环境变量 LOG_ONLINE_PORT 可改)
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$DIR")"

if [ -f "$ROOT/devel/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$ROOT/devel/setup.bash"
fi

cd "$DIR" || exit 1
exec python3 recorder.py
