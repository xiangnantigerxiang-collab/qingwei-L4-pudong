#!/bin/bash
gnome-terminal -t "driver_lidar" -x bash -c "cd /home/nvidia/qingwei-L4-No2; source devel/setup.bash;roslaunch lidar_perception lidar_perception.launch;exec bash"
sleep 1s
gnome-terminal -t "driver_cam" -x bash -c "cd /home/nvidia/qingwei-L4-No2/src/driver/cam_geac;./rb_camera.sh ros1_jpg;exec bash"
sleep 3s
gnome-terminal -t "driver" -x bash -c "cd /home/nvidia/qingwei-L4-No2; source devel/setup.bash;roslaunch ins demo.launch;exec bash"
sleep 2s
gnome-terminal -t "cannode" -x bash -c "cd /home/nvidia/qingwei-L4-No2; source devel/setup.bash;./launch/canbus.sh;exec bash"
sleep 2s
gnome-terminal -t "driver_lidar" -x bash -c "cd /home/nvidia/qingwei-L4-No2; source devel/setup.bash;roslaunch ./launch/start.launch;exec bash"
sleep 2s
gnome-terminal -t "perception" -x bash -c "cd /home/nvidia/qingwei-L4-No2/src/CUDA-CenterPoint/build; ./centerpoint_ros_node;exec bash"
sleep 2s
gnome-terminal -t "loc" -x bash -c "cd /home/nvidia/qingwei-L4-No2; source devel/setup.bash;roslaunch ./launch/start_auto_couple.launch;exec bash"
sleep 2s
gnome-terminal -t "control" -x bash -c "cd /home/nvidia/qingwei-L4-No2; source devel/setup.bash;roslaunch ./launch/control.launch;exec bash"
sleep 2s
gnome-terminal -t "fms" -x bash -c "cd /home/nvidia/qingwei-L4-No2; source devel/setup.bash;./fms.sh;exec bash"
sleep 2s
gnome-terminal -t "network" -x bash -c "cd /home/nvidia/qingwei-L4-No2; source devel/setup.bash;roslaunch data_logger data_logger.launch;exec bash"
sleep 1s
#gnome-terminal -t "network" -x bash -c "cd /home/nvidia/qingwei-L4-No2; source devel/setup.bash;./launch/netcheck.sh;exec bash"
#sleep 1s
gnome-terminal -t "simview" -x bash -c "cd /home/nvidia/zyd/0522; source devel/setup.bash;roslaunch src/simview/launch/simview.launch;exec bash"
sleep 2s

gnome-terminal -t "record_pnc" -x bash -c "cd /home/nvidia/qingwei-L4-No2/bags;./record_pnc.sh;exec bash"
sleep 1s
gnome-terminal -t "record_hook" -x bash -c "cd /home/nvidia/qingwei-L4-No2/bags;./record_hook_position.sh;exec bash"
sleep 1s
