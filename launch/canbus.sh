#!/bin/bash
echo 'nvidia' | sudo -S modprobe can
sudo modprobe can-raw
sudo modprobe can-bcm
sudo modprobe can-gw
sudo modprobe can_dev
sudo modprobe mttcan
sudo ifconfig can0 down
sudo ip link set can0 type can bitrate 250000
sudo ifconfig can0 up

cd /home/nvidia/qingwei-L4-No2/launch
roslaunch canbus.launch


