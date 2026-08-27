#include <signal.h>

#include <ros/ros.h>
#include "calib_logger.h"

void signalHandle(int signal);
int main(int argc, char *argv[])
{
    signal(SIGINT, signalHandle);
    signal(SIGTERM, signalHandle);

    ros::init(argc, argv, "calib_logger_node");
    CalibLogger calib_logger;
    calib_logger.run();
    return 0;
}

void signalHandle(int signal)
{
    std::cout << "消息切换任务终止" << std::endl;
    exit(0);
}