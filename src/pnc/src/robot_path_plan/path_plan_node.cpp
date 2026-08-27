#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include "path_plan_comply.h"
#include "ivmsglocpos.h"
#include <visualization_msgs/MarkerArray.h>

bool Camera0 = true;
//bool Camera1 = true;
//bool Camera2 = true;
//bool Camera3 = true;
//bool Camera4 = true;
//bool Camera5 = true;
//bool Camera6 = true;
bool Camera7 = true;

PathPlanComply pathPlanComply;

void TaskPlanCallBack(const robot::task_plan_msg::ConstPtr &msg)
{
    pathPlanComply.SetTaskPlanData(*msg);
}

void CanMsgCallBack(const robot::can_msg::ConstPtr &msg)
{
    pathPlanComply.SetCanData(*msg);
}

void NavigationMsgCallBack(const robot::navigation_msg::ConstPtr &msg)
{
    pathPlanComply.SetNavigationData(*msg);
}

void PerceptionMsgCallBack(const robot::perception::ConstPtr &msg)
{
    pathPlanComply.sysTime.msgPerception = ros::Time::now().toSec();
    if (pathPlanComply.AccSwitch)
        return;
    pathPlanComply.SetPerceptionData(*msg);
    //pathPlanComply.SetPerceptionData2(*msg);
}

void FrontScanCallBack(const jsk_recognition_msgs::BoundingBoxArray &msg)
{
    if (pathPlanComply.AccSwitch)
        return;
    pathPlanComply.SetFrontScanData(msg);
}

void BackScanCallBack(const jsk_recognition_msgs::BoundingBoxArray &msg)
{
    pathPlanComply.SetBackScanData(msg);
}

void LoadPosCallBack(const robot::palletpos::ConstPtr &msg)
{
    pathPlanComply.SetLoadPosition(*msg);
}

void PalletCoorCallBack(const robot::hook_position::ConstPtr &msg)
{
    pathPlanComply.SetPalletCoorData(*msg);
}

void Rs232CallBack(const robot::rs232::ConstPtr &msg)
{
    pathPlanComply.SetButtonData(*msg);
}

void LocalizationCallBack(const ivlocmsg::ivmsglocpos::ConstPtr &msg)
{
    pathPlanComply.sysTime.msgNavigation = ros::Time::now().toSec();

    auto status = msg->localization_status;

    if (status == 2)
        pathPlanComply.sysTime.msgNavigation -= 100.0;
}

void Camera0CallBack(const std_msgs::Bool &msg)
{
    Camera0 = msg.data;
}
//
//void Camera1CallBack(const std_msgs::Bool &msg)
//{
//    Camera1 = msg.data;
//}
//
//void Camera2CallBack(const std_msgs::Bool &msg)
//{
//    Camera2 = msg.data;
//}
//
//void Camera3CallBack(const std_msgs::Bool &msg)
//{
//    Camera3 = msg.data;
//}
//
//void Camera4CallBack(const std_msgs::Bool &msg)
//{
//    Camera4 = msg.data;
//}
//
//void Camera5CallBack(const std_msgs::Bool &msg)
//{
//    Camera5 = msg.data;
//}
//
//void Camera6CallBack(const std_msgs::Bool &msg)
//{
//    Camera6 = msg.data;
//}

void Camera7CallBack(const std_msgs::Bool &msg)
{
    Camera7 = msg.data;
}

void CommandMsgCallBack(const robot::v2nCommandFeedback::ConstPtr &msg)
{
    int command_state = msg->commandState; // 0停车 1暂停 2继续
    pathPlanComply.SetCommandState(command_state);
    if (msg->commandState == 0) // 停车
    {
        pathPlanComply.resetTask();
    }
}

void AirPortMsgCallback(const robot::AirCraftParkingPortConstPtr &msg)
{
    pathPlanComply.updateAirPortInfo(*msg);
}

void T1Callback(const ros::TimerEvent &real)
{
    int sensorstate = 0;

    double tnow = pathPlanComply.sysTime.now;
    double tlidar = pathPlanComply.sysTime.msgPerception;
    double tgnss = pathPlanComply.sysTime.msgNavigation;
    double tcamera = pathPlanComply.sysTime.msgCamera;
    double dlidar = tnow - tlidar;
    double dgnss = tnow - tgnss;
    double dcamera = 0.0;
    // 任一摄像头无数据则判定异常
    if (!Camera0 || !Camera7)
        dcamera = 100.0;
    // 雷达异常
    if (dlidar > 3.0)
        sensorstate += 2;
    // 相机异常
    if (dcamera > 2.0)
        sensorstate += 4;
    // gnss异常
    if (dgnss > 2.0)
        sensorstate += 8;
    if (sensorstate > 0)
        sensorstate += 1;
    //test
    //sensorstate = 0;
    ros::param::set("/planning/sensorstate", sensorstate);
    ros::param::set("/planning/alive", 1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_plan_node");
    ros::NodeHandle nh;
    ros::Subscriber task_plan_sub = nh.subscribe(
        "task_plan_msg", 1, TaskPlanCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber can_msg_sub = nh.subscribe(
        "can_msg", 10, CanMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber navigation_sub = nh.subscribe(
        "navigation_msg", 10, NavigationMsgCallBack, ros::TransportHints().tcpNoDelay()); // 订阅定位数据
    ros::Subscriber sub = nh.subscribe(
        "localization", 10, LocalizationCallBack, ros::TransportHints().tcpNoDelay()); // 订阅定位状态
    ros::Subscriber perception_sub = nh.subscribe(
        "perception", 10, PerceptionMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber load_pos_sub = nh.subscribe(
        "/palletpos", 10, LoadPosCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber pallet_pos_sub = nh.subscribe(
        "/hook_position", 10, PalletCoorCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber front_scan_sub = nh.subscribe(
        "/scan_bbox_result", 10, FrontScanCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber back_scan_sub = nh.subscribe(
        "/perception_back_bbox", 10, BackScanCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber rs232_sub = nh.subscribe(
        "/robot/serial/rs232/", 10, Rs232CallBack, ros::TransportHints().tcpNoDelay()); // 急停按键
    // 相机
    ros::Subscriber camera0_sub = nh.subscribe(
        "/cam0/status", 1, Camera0CallBack, ros::TransportHints().tcpNoDelay());
//    ros::Subscriber camera1_sub = nh.subscribe(
//        "/cam1/status", 1, Camera1CallBack, ros::TransportHints().tcpNoDelay());
//    ros::Subscriber camera2_sub = nh.subscribe(
//        "/cam2/status", 1, Camera2CallBack, ros::TransportHints().tcpNoDelay());
//    ros::Subscriber camera3_sub = nh.subscribe(
//        "/cam3/status", 1, Camera3CallBack, ros::TransportHints().tcpNoDelay());
//    ros::Subscriber camera4_sub = nh.subscribe(
//        "/cam4/status", 1, Camera4CallBack, ros::TransportHints().tcpNoDelay());
//    ros::Subscriber camera5_sub = nh.subscribe(
//        "/cam5/status", 1, Camera5CallBack, ros::TransportHints().tcpNoDelay());
//    ros::Subscriber camera6_sub = nh.subscribe(
//        "/cam6/status", 1, Camera6CallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber camera7_sub = nh.subscribe(
        "/cam7/status", 1, Camera7CallBack, ros::TransportHints().tcpNoDelay());

    ros::Publisher refer_path_pub = nh.advertise<robot::path_plan_msg>(
        "refer_path_msg", 10);
    ros::Publisher plan_path_pub = nh.advertise<robot::path_plan_msg>(
        "plan_path_msg", 10);
    ros::Publisher plan_status_pub = nh.advertise<robot::path_plan_status>(
        "path_plan_status", 10);
    ros::Publisher sound_light_sub = nh.advertise<robot::sound_light_msg>(
        "sound_light_msg", 10);
    ros::Subscriber command_msg_sub = nh.subscribe("/cloud/msg/command_msg", 1,
                                                   CommandMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber airport_sub = nh.subscribe("/cloud/msg/airport_msg", 1,
                                               AirPortMsgCallback, ros::TransportHints().tcpNoDelay());
    ros::Publisher pub_test_trajs = nh.advertise<visualization_msgs::MarkerArray>("test_trajs", 10);
    ros::Publisher pub_obstacles = nh.advertise<visualization_msgs::MarkerArray>("planning/obstacles",1);

    ros::Rate loop_rate(10);
    ros::Timer T1 = nh.createTimer(ros::Duration(0.1), T1Callback);

    pathPlanComply.InitParameter();

    double msgPerception;
    double msgNavigation;
    double msgLidar;
    double msgCamera;

    while (ros::ok())
    {
        ros::spinOnce();

        pathPlanComply.sysTime.now = ros::Time::now().toSec();

        nh.getParam("/robot/lanechangecmd", pathPlanComply.LaneChangeRequset);
        nh.getParam("/robot/control/accswitch", pathPlanComply.AccSwitch);
        nh.getParam("/robot/planning/pallettype", pathPlanComply.PalletType);

        // if (pathPlanComply.workMode == MANUALCONTROLMODE) // 人工驾驶
        // {
        //     pathPlanComply.InitSafetyCheck = 0;
        //     // ROS_INFO("Planning : Manual Mode ... ");
        // }
        pathPlanComply.PathPlanProcess(); // 这里从task读取任务数据包括:挡位，轨迹，速度
        pathPlanComply.PublishReferPath(refer_path_pub);
        pathPlanComply.PublishPlanPath(plan_path_pub, sound_light_sub);
        pathPlanComply.PublishPathPlanStatus(plan_status_pub);
        // pathPlanComply.pubLatticeTrajs(pub_test_trajs);
        //pathPlanComply.pubObstacles(pub_obstacles);
 printf("=================perception time:%f=========\n", pathPlanComply.sysTime.now-pathPlanComply.sysTime.msgPerception);
        loop_rate.sleep();
    }

    return 0;
}

