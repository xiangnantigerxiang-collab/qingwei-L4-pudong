#!/bin/bash
# HMI 车载一键诊断脚本 —— 8080 页面打不开排查
# 用法: 在工程根目录或 hmi/ 目录下执行:  bash hmi/diagnose_vehicle.sh
# 说明: 只读诊断(第7步会前台试启动服务5秒后自动退出,不影响现有进程);
#       若第2步发现已有 hmi_server 在跑,第7步会跳过避免端口冲突误报。
# 输出: 全部诊断结果 + 自动结论,请把完整输出发回给开发者。

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # hmi/ 目录
ROOT="$(dirname "$DIR")"

line() { echo; echo "===== $1 ====="; }

line "0. 系统信息"
uname -a
python3 --version 2>&1
which python3
grep PRETTY_NAME /etc/os-release 2>/dev/null
echo "磁盘余量: $(df -h / | tail -1 | awk '{print $4 " 可用"}')"

line "1. Python 语法兼容检查(车载 python3 编译全部 hmi 源码)"
FAIL=0
find "$DIR" -maxdepth 1 -name "*.py" | sort | while read -r f; do
    if ! python3 -m py_compile "$f" 2>&1; then
        echo "!! 编译失败: $f"
    fi
done
echo "(无『!! 编译失败』= 全部通过)"

line "2. hmi 进程状态"
pgrep -af "hmi_server.py" || echo ">> 未发现 hmi_server 进程(服务没在跑)"

line "3. 端口 8080 监听状态"
if command -v ss >/dev/null 2>&1; then
    ss -ltnp 2>/dev/null | grep ":8080" || echo ">> 8080 未监听"
else
    netstat -ltnp 2>/dev/null | grep ":8080" || echo ">> 8080 未监听(ss/netstat 均无结果)"
fi

line "4. 车端本机访问测试"
HTTP_CODE="$(curl -s -m 3 -o /dev/null -w '%{http_code}' http://127.0.0.1:8080/ 2>/dev/null)" || true
if [ "$HTTP_CODE" = "200" ]; then
    echo "HTTP 200 ← 本机访问正常"
else
    echo ">> 本机无法访问 8080 (http_code=${HTTP_CODE:-000}; 000=连接不上)"
fi

line "5. 网卡 IP(浏览器应访问其中与电脑同网段的地址)"
hostname -I 2>/dev/null || ip -4 addr show | grep inet

line "6. 防火墙状态"
ufw status 2>/dev/null || echo "ufw 不可用/未安装(默认无限制,一般可排除)"
iptables -S INPUT 2>/dev/null | head -5 || echo "(iptables 需 root,可 sudo 重跑本脚本确认)"

line "7. 前台试启动 5 秒抓启动输出"
if pgrep -f "hmi_server.py" >/dev/null; then
    echo ">> 已有 hmi_server 在跑,跳过试启动(避免端口冲突误报);如需重试请先停掉现有进程"
else
    echo "-- 启动输出如下(5秒后自动停止,『[HMI]』开头且含『可用』= 启动成功) --"
    cd "$DIR" || exit 1
    timeout 5 python3 hmi_server.py --config hmi_config.py --host 0.0.0.0 --port 8080 2>&1 | head -30
    echo "-- 试启动结束(退出码 $?) --"
fi

line "8. 部署完整性抽查"
for f in hmi_server.py hmi_config.py process_manager.py ros_bridge.py static/index.html; do
    [ -f "$DIR/$f" ] && echo "OK  $f" || echo "!!  缺失 $f"
done

# ---------- 自动结论 ----------
line "自动结论(按最可能根因排序,仅提示不绝对)"
if pgrep -f "hmi_server.py" >/dev/null && curl -sS -m 2 -o /dev/null http://127.0.0.1:8080/ 2>/dev/null; then
    echo "▲ 服务在跑且本机可访问 → 问题在『电脑→车端』网络:确认浏览器地址用的是第5步输出的同网段 IP、检查电脑与车是否同网段/有无 VPN 干扰、第6步防火墙"
elif ! pgrep -f "hmi_server.py" >/dev/null; then
    echo "▲ 服务根本没在跑 → 最常见原因:ssh 断开后前台服务被杀。请用 tmux 常驻启动:"
    echo "    tmux new -s hmi 'bash /home/nvidia/qingwei-L4-No2/hmi/hmi.sh'"
    echo "  然后看第7步试启动输出:有报错=按报错修;显示『[HMI]...可用』=启动本身没问题,用 tmux 常驻即可"
else
    echo "▲ 进程在但本机 curl 不通 → 进程可能僵死/卡在启动中:杀掉后用第7步方式前台重启观察输出"
fi
echo
echo ">>> 请把本脚本完整输出发回给开发者 <<<"
