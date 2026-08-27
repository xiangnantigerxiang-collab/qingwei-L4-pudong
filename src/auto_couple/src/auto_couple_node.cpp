#include "auto_couple_core.h"
#include <vector>
#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <iostream>

int main(int argc, char **argv)
{
    std::string port;

    ros::init(argc, argv, "auto_couple");
    ros::NodeHandle nh("~");

    AutoCouple core(nh);

    return 0;
}
