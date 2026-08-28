#!/bin/bash
# HMI 开机自启动 安装/卸载脚本(车载运行)
# 用法:
#   bash hmi/install_autostart.sh            安装并启动 systemd 服务 qingwei-hmi
#   bash hmi/install_autostart.sh remove     停止服务并卸载自启
# 说明:仅网页服务自启,车辆组件不会被自动拉起,上电后仍需页面手动"一键启动"。
#       卸载后恢复手动方式:tmux 里 bash hmi/hmi.sh

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$DIR")"
UNIT_SRC="$DIR/qingwei-hmi.service"
UNIT_DST="/etc/systemd/system/qingwei-hmi.service"
SERVICE="qingwei-hmi"
PORT="${HMI_PORT:-8080}"
ENV_FILE="/etc/environment"
FIREFOX_EGL_ENV="MOZ_X11_EGL=1"

SUDO=""
if [ "$(id -u)" -ne 0 ]; then
    SUDO="sudo"
fi

# Jetson/L4T 上 Firefox 的默认 GLX 路径可能无法创建 WebGL context。
# 写入登录环境后,从桌面启动的 Firefox 会使用 EGL;幂等更新并清理重复项,
# 同时完整保留 /etc/environment 中的其他配置。
configure_firefox_egl() {
    local tmp_env
    tmp_env="$(mktemp)" || {
        echo "错误:无法创建 /etc/environment 临时文件" >&2
        return 1
    }

    if [ -f "$ENV_FILE" ]; then
        if ! $SUDO awk '
            BEGIN { written = 0 }
            /^[[:space:]]*(export[[:space:]]+)?MOZ_X11_EGL[[:space:]]*=/ {
                if (!written) {
                    print "MOZ_X11_EGL=1"
                    written = 1
                }
                next
            }
            { print }
            END {
                if (!written) {
                    print "MOZ_X11_EGL=1"
                }
            }
        ' "$ENV_FILE" > "$tmp_env"; then
            rm -f "$tmp_env"
            echo "错误:无法读取或处理 $ENV_FILE" >&2
            return 1
        fi
    else
        printf '%s\n' "$FIREFOX_EGL_ENV" > "$tmp_env"
    fi

    if [ -f "$ENV_FILE" ] && cmp -s "$tmp_env" "$ENV_FILE"; then
        rm -f "$tmp_env"
        echo "Firefox EGL 环境已配置:$FIREFOX_EGL_ENV"
        return 0
    fi
    if ! $SUDO install -m 0644 "$tmp_env" "$ENV_FILE"; then
        rm -f "$tmp_env"
        echo "错误:无法写入 $ENV_FILE" >&2
        return 1
    fi
    rm -f "$tmp_env"
    echo "已写入 $ENV_FILE:$FIREFOX_EGL_ENV"
}

# ---------- 卸载 ----------
if [ "$1" = "remove" ]; then
    echo "== 卸载 $SERVICE 自启 =="
    $SUDO systemctl disable --now "$SERVICE" 2>/dev/null
    $SUDO rm -f "$UNIT_DST"
    $SUDO systemctl daemon-reload
    $SUDO systemctl reset-failed "$SERVICE" 2>/dev/null
    echo "完成。组件进程(若有)未被触碰;HMI 需要时手动: bash hmi/hmi.sh"
    echo "说明:$ENV_FILE 中的 $FIREFOX_EGL_ENV 为车机 Firefox兼容配置,卸载 HMI 时保留"
    exit 0
fi

# ---------- 前置检查 ----------
if [ ! -f "$DIR/hmi_server.py" ]; then
    echo "错误: 未找到 hmi_server.py,请在工程 hmi/ 目录内运行本脚本" >&2
    exit 1
fi
if [ ! -f "$UNIT_SRC" ]; then
    echo "错误: 缺少 $UNIT_SRC" >&2
    exit 1
fi
if ! command -v systemctl >/dev/null 2>&1; then
    echo "错误: 系统无 systemctl,不支持 systemd 自启" >&2
    exit 1
fi

# ---------- 配置车端 Firefox WebGL/EGL ----------
if ! configure_firefox_egl; then
    exit 1
fi

# ---------- 生成 unit(按实际部署路径与当前用户适配模板) ----------
CUR_USER="$(id -un)"
TMP_UNIT="$(mktemp)"
sed -e "s|/home/nvidia/qingwei-L4-No2|$ROOT|g" \
    -e "s|Environment=HMI_PORT=8080|Environment=HMI_PORT=$PORT|" \
    -e "s|User=nvidia|User=$CUR_USER|" \
    "$UNIT_SRC" > "$TMP_UNIT"

# ---------- 处理已在跑的手动 HMI 实例(避免端口冲突) ----------
if pgrep -f hmi_server.py >/dev/null 2>&1; then
    echo "检测到已在运行的 HMI(tmux/手动),先停止以免 ${PORT} 端口冲突..."
    pkill -f hmi_server.py
    for _ in 1 2 3 4 5; do
        pgrep -f hmi_server.py >/dev/null 2>&1 || break
        sleep 1
    done
    if pgrep -f hmi_server.py >/dev/null 2>&1; then
        echo "错误: 旧 HMI 进程未能停止,请手动处理(pgrep -f hmi_server.py)后重试" >&2
        rm -f "$TMP_UNIT"
        exit 1
    fi
    echo "旧实例已停止(其拉起的组件若在运行不受影响)"
fi

# ---------- 安装并启动 ----------
$SUDO cp "$TMP_UNIT" "$UNIT_DST"
rm -f "$TMP_UNIT"
$SUDO systemctl daemon-reload
$SUDO systemctl enable --now "$SERVICE"

# ---------- 验证 ----------
sleep 2
if ! systemctl is-active --quiet "$SERVICE"; then
    echo "服务未正常运行,最近日志:" >&2
    journalctl -u "$SERVICE" -n 30 --no-pager >&2
    exit 1
fi
CODE="$(curl -s -o /dev/null -w '%{http_code}' "http://127.0.0.1:$PORT/" || echo 000)"
if [ "$CODE" = "200" ]; then
    echo "安装成功,HMI 已自启动: http://<本机IP>:$PORT (本机自检 HTTP $CODE)"
else
    echo "警告: 服务已启动但本机自检 HTTP $CODE(非 200)"
    echo "若 ${PORT} 被其他程序占用,改 hmi/qingwei-hmi.service 的 HMI_PORT 后:"
    echo "  sudo systemctl daemon-reload && sudo systemctl restart $SERVICE"
fi
echo ""
echo "常用命令:"
echo "  systemctl status $SERVICE          状态"
echo "  journalctl -u $SERVICE -f          HMI 服务日志(组件日志仍在 hmi/logs/)"
echo "  sudo systemctl restart $SERVICE    重启 HMI(组件不受影响)"
echo "  bash hmi/install_autostart.sh remove   卸载自启"
echo ""
echo "Firefox EGL 环境将在下次登录桌面后生效;首次安装请注销重新登录或重启车机"
