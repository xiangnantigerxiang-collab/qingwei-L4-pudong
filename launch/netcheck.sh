#!/bin/bash
# 网络连通检测脚本

# 配置项
TARGET="223.5.5.5"   # 检测目标DNS
INTERVAL=2            # 检测间隔(秒)
LOG_FILE="./net_check.log"

# 日志头部
echo "==================== 网络检测启动 $(date '+%Y-%m-%d %H:%M:%S') ====================" >> $LOG_FILE
echo "开始持续监测网络，目标：$TARGET"

while true
do
    # ping 1次，超时1秒
    ping -c 1 -W 2 $TARGET > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] ✅ 网络正常"
        rosparam set /robot/planning/netcheck 0
    else
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] ❌ 网络断开"
        rosparam set /robot/planning/netcheck 1
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] 网络断开掉线" >> $LOG_FILE
    fi
    sleep $INTERVAL
done

