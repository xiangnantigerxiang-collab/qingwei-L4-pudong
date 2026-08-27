#!/bin/bash
# HMI 启动包装脚本
# 用法:bash hmi/hmi.sh          (默认监听 0.0.0.0:8080,可用环境变量 HMI_PORT 改端口)
# 说明:devel/setup.bash 存在才 source(本机调试副本没有 devel,自动跳过)
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$DIR")"

if [ -f "$ROOT/devel/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$ROOT/devel/setup.bash"
fi

cd "$DIR" || exit 1
exec python3 hmi_server.py --config hmi_config.py --host 0.0.0.0 --port "${HMI_PORT:-8080}"
