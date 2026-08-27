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

#include "data_logger.h"

DataLogger::DataLogger()
{
    loadConfigParam();
    std::cout << interval_ << " " << frequency_ << std::endl;

    loc_pose_sub_ = nh_.subscribe<robot::navigation_msg>("navigation_msg", 1,
                                                         boost::bind(&DataLogger::poseCallback, this, _1));
    vehicle_info_sub_ = nh_.subscribe<canbus::can_msg>("can_msg", 1,
                                                       boost::bind(&DataLogger::vehicleInfoCallback, this, _1));

    control_cmd_sub_ = nh_.subscribe<robot::control_msg>("control_msg", 1,
                                                         boost::bind(&DataLogger::controlDebugCallback, this, _1));
    task_sub_ = nh_.subscribe<robot::task_plan_msg>("task_plan_msg", 1,
                                                    boost::bind(&DataLogger::taskCallback, this, _1));
    path_status_sub_ = nh_.subscribe<robot::path_plan_status>("path_plan_status", 1,
                                                              boost::bind(&DataLogger::planningDebugCallback, this, _1));
    path_sub_ = nh_.subscribe<robot::path_plan_msg>("plan_path_msg", 1,
                                                    boost::bind(&DataLogger::planningCallback, this, _1));
    std::string log_path = ros::package::getPath("data_logger") + "/logs/";
    ros::param::get("log_path", log_path);
    csv_logger_ptr_ = std::make_unique<CSVLogger>(log_path);
}

DataLogger::~DataLogger()
{
    csv_logger_ptr_->stop();
}

void DataLogger::loadConfigParam()
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

void DataLogger::run()
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

void DataLogger::init()
{
    std::string header = std::string("auto, x,y,gps_speed,remain_s,ego2obj,task_execute, plan_speed,safety,desire_wheel_angle, desire_speed,desire_throttle,desire_brake,desire_gear,") +
                         std::string("wheel_angle,speed,throttle,brake,gear, bias_distance,bias_angle, sensorstate,software_state,network_state, total_mile");
    csv_logger_ptr_->start(header);
}

void DataLogger::log()
{
    int auto_state = int(vehicle_info_msg_.controlPanelState);
    double x = pose_msg_.xAxis;
    double y = pose_msg_.yAxis;
    double gps_speed = pose_msg_.gpsSpeed;
    double stop_x = path_status_msg_.stopX;
    double stop_y = path_status_msg_.stopY;
    // double x = path_status_msg_.curXAxis;
    // double y = path_status_msg_.curYAxis;
    double remain_s = path_status_msg_.distance2Stop;
    double ego2obj = path_status_msg_.distance2Object;
    int task_execute_flag = int(path_status_msg_.taskExecuStatus); // 0 no task 1 progress 2 finished
    double plan_speed = path_msg_.planspeed;
    int safety = int(path_msg_.safety);
    double desire_wheel_angle = control_cmd_msg_.wheelAngle;
    double desire_speed = control_cmd_msg_.desireSpeed;
    double desire_throttle = control_cmd_msg_.throttlePercent;
    double desire_brake = control_cmd_msg_.brakePercent;
    double desire_gear = task_plan_msg_.desireGear;
    double bias_distance = control_cmd_msg_.biaDistance;
    double bias_angle = control_cmd_msg_.biaAngle;

    double wheel_angle = vehicle_info_msg_.wheelAngle;
    double speed = vehicle_info_msg_.vehicleSpeed;
    int throttle = int(vehicle_info_msg_.throttlePercent);
    int brake = int(vehicle_info_msg_.brakePercent);
    int gear = int(vehicle_info_msg_.curGear);

    int sensorstate = 0;
    ros::param::get("/planning/sensorstate", sensorstate);
    int planning_state = 0;
    ros::param::get("planning/alive", planning_state);
    int network_status = 0;
    ros::param::get("/robot/planning/netcheck", network_status);

    csv_logger_ptr_->log(auto_state, x, y, gps_speed, remain_s, ego2obj, task_execute_flag, plan_speed, safety, desire_wheel_angle, desire_speed, desire_throttle, desire_brake, desire_gear,
                         wheel_angle, speed, throttle, brake, gear, bias_distance, bias_angle,
                         sensorstate, planning_state, network_status, total_mile_);
}

// 定位模块回调函数
void DataLogger::poseCallback(robot::navigation_msgConstPtr pose_msg)
{
    static auto last_pose = *pose_msg;
    double dist = sqrt(pow(pose_msg->xAxis - last_pose.xAxis, 2) + pow(pose_msg->yAxis - last_pose.yAxis, 2));
    if (dist > 0.1)
    {
        total_mile_ += dist;
        last_pose = *pose_msg;
    }
    pose_msg_ = *pose_msg;
    update_flag_ = true;
}

// 控制模块回调函数
void DataLogger::controlDebugCallback(robot::control_msgConstPtr msg)
{
    control_cmd_msg_ = *msg;
    update_flag_ = true;
}

void DataLogger::taskCallback(robot::task_plan_msgConstPtr task_msg)
{
    task_plan_msg_ = *task_msg;
    update_flag_ = true;
}

// 底盘数据回调函数
void DataLogger::vehicleInfoCallback(canbus::can_msgConstPtr v_info_msg)
{
    vehicle_info_msg_ = *v_info_msg;
    update_flag_ = true;
}

// 路径规划数据回调函数
void DataLogger::planningDebugCallback(robot::path_plan_statusConstPtr msg)
{
    path_status_msg_ = *msg;
    update_flag_ = true;
}

void DataLogger::planningCallback(robot::path_plan_msgConstPtr msg)
{
    path_msg_ = *msg;
    update_flag_ = true;
}

