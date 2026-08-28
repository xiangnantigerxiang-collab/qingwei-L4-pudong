#!/bin/bash
# monitor 启动包装脚本(对标 hmi/hmi.sh)
# 用法:bash monitor/monitor.sh    (默认监听 0.0.0.0:8081,环境变量 MONITOR_PORT 可改)
# 说明:devel/setup.bash 存在才 source(本机调试副本没有 devel,自动跳过)
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$DIR")"

if [ -f "$ROOT/devel/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$ROOT/devel/setup.bash"
fi

cd "$DIR" || exit 1
exec python3 monitor_server.py --config monitor_config.py --host 0.0.0.0 --port "${MONITOR_PORT:-8081}"
