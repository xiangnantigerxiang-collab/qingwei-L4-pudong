1、使用方法： 执行./pack.sh 创建deb包
2、可以通过以下命令修改功能
sudo systemctl set-environment CAMERA_MODE=rtsp
sudo systemctl restart camera_demo_geac.service
3、可以通过
systemctl show --property=Environment 
systemctl  status camera_demo_geac.service
查看当前功能