#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <chrono>
#include <time.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

#include "calib_logger.h"

CalibLogger::CalibLogger()
{
    loadConfigParam();
    std::cout << interval_ << " " << frequency_ << std::endl;

    loc_pose_sub_ = nh_.subscribe<robot::navigation_msg>("navigation_msg", 1,
                                                         boost::bind(&CalibLogger::poseCallback, this, _1));
    vehicle_info_sub_ = nh_.subscribe<canbus::can_msg>("can_msg", 1,
                                                       boost::bind(&CalibLogger::vehicleInfoCallback, this, _1));

    control_cmd_sub_ = nh_.subscribe<canbus::can_comm_msg>("can_comm_msg", 1,
                                                           boost::bind(&CalibLogger::controlDebugCallback, this, _1));
    std::string log_path = ros::package::getPath("data_logger") + "/logs/";
    ros::param::get("log_path", log_path);
    csv_logger_ptr_ = std::make_unique<CSVLogger>(log_path);
}

CalibLogger::~CalibLogger()
{
    csv_logger_ptr_->stop();
}

void CalibLogger::loadConfigParam()
{
    std::string config_path = ros::package::getPath("data_logger") + "/config/log.yaml";

    YAML::Node root;
    try
    {
        root = YAML::LoadFile(config_path);
    }
    catch (YAML::ParserException &ex)
    {
        std::cout << "日志配置文件解析失败: " << ex.what() << std::endl;
    }
    catch (YAML::BadFile &ex)
    {
        std::cout << "日志配置文件解析失败 " << ex.what() << std::endl;
    }
    interval_ = root["interval"].as<int>();
    frequency_ = root["frequency"].as<int>();
}

void CalibLogger::run()
{
    ros::Rate rate(frequency_);
    this->init();
    while (ros::ok())
    {
        ros::spinOnce();
        if (update_flag_)
        {
            this->log();
            update_flag_ = false;
        }
        rate.sleep();
    }
}

void CalibLogger::init()
{
    std::string header = std::string("throttle,breke,speed, distance, acc");
    csv_logger_ptr_->start(header);
}

void CalibLogger::log()
{
    csv_logger_ptr_->log(throttle_, brake_, vehicle_speed_, distance_, 0.0);
}

// 定位模块回调函数
void CalibLogger::poseCallback(robot::navigation_msgConstPtr pose_msg)
{
    static double last_x =  pose_msg->xAxis;
    static double last_y =  pose_msg->yAxis;
    double current_x = pose_msg->xAxis;
    double current_y = pose_msg->yAxis;
    distance_ = hypot(current_x - last_x, current_y - last_y);
    update_flag_ = true;
}

// 控制模块回调函数
void CalibLogger::controlDebugCallback(canbus::can_comm_msgConstPtr msg)
{
    control_cmd_msg_ = *msg;
    throttle_ = control_cmd_msg_.throttlePercent;
    brake_ = control_cmd_msg_.brakePercent;
    update_flag_ = true;
}

// 底盘数据回调函数
void CalibLogger::vehicleInfoCallback(canbus::can_msgConstPtr v_info_msg)
{
    vehicle_info_msg_ = *v_info_msg;
    vehicle_speed_ = vehicle_info_msg_.vehicleSpeed;
    update_flag_ = true;
}