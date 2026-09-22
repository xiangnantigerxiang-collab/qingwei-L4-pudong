GEAC tools 工程使用说明（核对：2026-09-19）

使用方法
本目录用于供应商打包/系统服务管理，整车ROS启动请看父目录README.md。
需要制作deb时进入本目录执行 bash pack.sh；先检查脚本的输入路径和目标平台。
服务安装完成后才使用下方systemctl示例；不要与HMI的ROS相机进程同时启动。

注意事项与容易疏忽的点
- 打包/切换系统服务模式不等于修改ROS发布模式；本工程常用rb_camera.sh ros1_jpg。
- 系统环境变量需要配套服务重启才生效；核对实际服务名和当前模式。
- 原有工具、库和可执行文件是部署依赖，本轮不改动或删除。

供应商原始说明：

1、使用方法： 执行./pack.sh 创建deb包
2、可以通过以下命令修改功能
sudo systemctl set-environment CAMERA_MODE=rtsp
sudo systemctl restart camera_demo_geac.service
3、可以通过
systemctl show --property=Environment 
systemctl  status camera_demo_geac.service
查看当前功能
