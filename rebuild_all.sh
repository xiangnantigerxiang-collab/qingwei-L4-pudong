#!/bin/bash
  
# ---- 1. 清理工作空间产物 ----
sudo rm -rf ./build ./devel

# ---- 2. CUDA-CenterPoint 独立重建 ----
# 不加 -j：nvcc 单文件显存/内存峰值高,串行最稳（与原流程一致）
say "CenterPoint 独立 cmake 重建（串行,较慢属正常）"
( cd src/CUDA-CenterPoint
  rm -rf ./build
  mkdir ./build
  cd ./build
  cmake .. && make
) || die "CenterPoint 构建失败（旧可执行已清,修复后重跑本脚本）"
[ -f src/CUDA-CenterPoint/build/centerpoint_ros_node ] || die "centerpoint_ros_node 未产出"
say "centerpoint_ros_node OK"

# ---- 3. catkin 分级编译：关键包先编早暴露错误 ----
catkin_make --pkg rslidar_sdk -DENABLE_TRANSFORM=ON
catkin_make -j"$JOBS" --pkg ivlocmsg || die "ivlocmsg 编译失败"
catkin_make -j"$JOBS" --pkg auto_couple || die "auto_couple 编译失败"
catkin_make -j"$JOBS" --pkg lidar_perception || die "lidar_perception 编译失败"
catkin_make -j"$JOBS" --pkg robot  || die "robot(pnc) 编译失败"
catkin_make -j"$JOBS" --pkg canbus || die "canbus 编译失败"
# ⚠ --pkg 会把 CATKIN_WHITELIST_PACKAGES 写进 CMake 缓存,
#   之后裸跑 catkin_make 仍只编白名单包（fms/driver/simview 等全被跳过）
#   ——必须显式清空才是真"全量"
catkin_make -j"$JOBS" -DCATKIN_WHITELIST_PACKAGES="" \
    || die "全量编译失败（若为 OOM：JOBS=2 重跑）"
say "catkin 全量编译完成"
