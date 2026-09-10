// health_monitor_node.cpp —— ROS 接线:初始化与主循环
#include "ros/ros.h"
#include "health_monitor.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "health_monitor_node");
    HealthMonitor monitor;
    monitor.Run();
    return 0;
}
