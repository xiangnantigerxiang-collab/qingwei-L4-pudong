# GEAC相机：本工程使用入口

核对：2026-09-19。保留供应商demo/tools说明；本工程运行入口是 `rb_camera.sh ros1_jpg`。
早期部署经验包括系统库安装、工作目录和ROS环境，以下约束仍有效。

## 使用方法

```bash
cd /home/nvidia/qingwei-L4-No2
source devel/setup.bash
cd src/driver/cam_geac
bash rb_camera.sh ros1_jpg
```

需要roscore、正确相机配置及Orin对应系统库。脚本会按平台选择编码库、复制到系统目录并
初始化硬件，首次可能需要终端输入sudo密码；HMI已运行相机时不重复执行。
使用 `rostopic list` 核对实际 `cam*/compressed` 话题，再配置log_online等消费者。
[demo说明](demo/README.md)用于独立取图；[tools说明](tools/readme.txt)用于打包/系统服务。

## 注意事项与容易疏忽的点

- 所有相对cfg/lib/demo路径依赖cam_geac目录；不能在工程根直接调用脚本假定路径自动修正。
- demo可执行与库是车端运行依赖，不因缺源码或没在CMake中出现而删除。
- ROS1和ROS2模式不同，当前工程使用ros1_jpg；修改相机输出名需同步HMI/记录器实际消费者。
- 原供应商更换cfg示例含删除操作，先备份并确认目标配置，不能在工作树盲目执行清空命令。
- 记录器只记录视频链路状态，不会自动保存相机视频；Web页面正常也不证明压缩图像话题新鲜。
