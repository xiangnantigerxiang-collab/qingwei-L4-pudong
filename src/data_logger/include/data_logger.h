#ifndef _LOG_WRITER_H_
#define _LOG_WRITER_H_

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <fstream>

#include "canbus/can_comm_msg.h"
#include "canbus/can_msg.h"
#include "robot/control_msg.h"
#include "robot/navigation_msg.h"
#include "robot/path_plan_status.h"
#include "robot/path_plan_msg.h"
#include "robot/task_plan_msg.h"
#include "csv_logger.h"

class DataLogger
{
public:
    DataLogger();
    ~DataLogger();
    /**
     * @brief 循环接受话题消息，并记录日志
     *
     */
    void run();

    void init();

    void log();

private:

    /**
     * @brief 加载日志模块的配置文件，读入配置参数
     *
     */

    void loadConfigParam();

    // 定位模块回调函数
    void poseCallback(robot::navigation_msgConstPtr pose_msg);

    // 控制模块回调函数
    void controlDebugCallback(robot::control_msgConstPtr msg);

    // 任务回调
    void taskCallback(robot::task_plan_msgConstPtr task_msg);

    //底盘数据回调函数
    void vehicleInfoCallback(canbus::can_msgConstPtr v_info_msg);

    //路径规划数据回调函数
    void planningDebugCallback(robot::path_plan_statusConstPtr msg);

    void planningCallback(robot::path_plan_msgConstPtr msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber loc_pose_sub_;
    ros::Subscriber control_cmd_sub_;
    ros::Subscriber vehicle_info_sub_;
    ros::Subscriber path_status_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber task_sub_;
    robot::navigation_msg pose_msg_;
    robot::path_plan_status path_status_msg_;
    robot::path_plan_msg path_msg_;
    robot::control_msg control_cmd_msg_;
    canbus::can_msg vehicle_info_msg_;
    robot::task_plan_msg task_plan_msg_;

    std::unique_ptr<CSVLogger> csv_logger_ptr_;
    bool update_flag_ = false;
    double total_mile_ = 0.0;

    // 读取的yaml参数
    int interval_ = 5;
    int frequency_ = 10;
};

#endif
