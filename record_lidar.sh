#!/bin/bash 
SAVE_DIR="lidar_record_$(date +%Y%m%d_%H%M%S)" 
mkdir -p "$SAVE_DIR" 
echo "=====================================" 
echo " 开始录制 4 个雷达话题（Ctrl+C 停止）" 
echo " 保存到：$SAVE_DIR" 
echo "=====================================" 
rosbag record -O "$SAVE_DIR/lidar.bag" /rslidar_points_mid /rslidar_points_left /rslidar_points_right /rslidar_points_front /perception
