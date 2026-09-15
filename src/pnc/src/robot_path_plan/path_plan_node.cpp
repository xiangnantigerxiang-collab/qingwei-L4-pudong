#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include "path_plan_comply.h"
#include "ivmsglocpos.h"
#include <visualization_msgs/MarkerArray.h>

bool Camera0 = true;
bool Camera7 = true;

PathPlanComply pathPlanComply;

// ROS 回调只更新 PathPlanComply 的输入快照；规划和发布统一在 10 Hz 主循环执行。
void TaskPlanCallBack(const robot::task_plan_msg::ConstPtr &msg) {
    pathPlanComply.SetTaskPlanData(*msg);
}

void CanMsgCallBack(const robot::can_msg::ConstPtr &msg) {
    pathPlanComply.SetCanData(*msg);
}

void NavigationMsgCallBack(const robot::navigation_msg::ConstPtr &msg) {
    pathPlanComply.SetNavigationData(*msg);
}

void PerceptionMsgCallBack(const robot::perception::ConstPtr &msg) {
    pathPlanComply.sysTime.msgPerception = ros::Time::now().toSec();
    if(pathPlanComply.AccSwitch) return;
    pathPlanComply.SetPerceptionData(*msg);
}

void FrontScanCallBack(const jsk_recognition_msgs::BoundingBoxArray &msg) {
    if(pathPlanComply.AccSwitch) return;
    pathPlanComply.SetFrontScanData(msg);
}

void BackScanCallBack(const jsk_recognition_msgs::BoundingBoxArray &msg) {
    pathPlanComply.SetBackScanData(msg);
}

void LoadPosCallBack(const robot::palletpos::ConstPtr &msg) {
    pathPlanComply.SetLoadPosition(*msg);
}

void PalletCoorCallBack(const robot::hook_position::ConstPtr &msg) {
    pathPlanComply.SetPalletCoorData(*msg);
}

void Rs232CallBack(const robot::rs232::ConstPtr &msg) {
    pathPlanComply.SetButtonData(*msg);
}

void LocalizationCallBack(const ivlocmsg::ivmsglocpos::ConstPtr &msg) {
    pathPlanComply.sysTime.msgNavigation = ros::Time::now().toSec();

    auto status = msg->localization_status;

    if(status == 2)
        pathPlanComply.sysTime.msgNavigation -= 100.0;
}

void Camera0CallBack(const std_msgs::Bool &msg) {
    Camera0 = msg.data;
}

void Camera7CallBack(const std_msgs::Bool &msg) {
    Camera7 = msg.data;
}

void CommandMsgCallBack(const robot::v2nCommandFeedback::ConstPtr &msg) {
    pathPlanComply.SetCommandData(*msg);
}

void AirPortMsgCallback(const robot::AirCraftParkingPortConstPtr &msg) {
    pathPlanComply.updateAirPortInfo(*msg);
}

void T1Callback(const ros::TimerEvent &real) {
    // 传感器状态按位累加：雷达=2、相机=4、GNSS=8；存在任一故障时再加 1，
    // 因此最低位可作为“有故障”的总标志。这里沿用既有参数协议。
    int sensorstate = 0;

    double tnow = pathPlanComply.sysTime.now;
    double tlidar = pathPlanComply.sysTime.msgPerception;
    double tgnss = pathPlanComply.sysTime.msgNavigation;
    double tcamera = pathPlanComply.sysTime.msgCamera;
    double dlidar = tnow - tlidar;
    double dgnss = tnow - tgnss;
    double dcamera = 0.0;
    // 任一摄像头无数据则判定异常
    if(!Camera0 || !Camera7) dcamera = 100.0;
    // 雷达异常
    if(dlidar > 3.0) sensorstate += 2;
    //相机异常
    if(dcamera > 2.0) sensorstate += 4;
    //gnss异常
    if(dgnss > 2.0) sensorstate += 8;

    if(sensorstate > 0) sensorstate += 1;

    ros::param::set("/planning/sensorstate", sensorstate);
    ros::param::set("/planning/alive", 1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_plan_node");
    ros::NodeHandle nh;
    ros::Subscriber task_plan_sub = nh.subscribe(
        "/task_plan_msg", 1, TaskPlanCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber can_msg_sub = nh.subscribe(
        "/can_msg", 10, CanMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber navigation_sub = nh.subscribe(
        "/navigation_msg", 10, NavigationMsgCallBack, ros::TransportHints().tcpNoDelay());  // 订阅定位数据
    ros::Subscriber sub = nh.subscribe(
        "/localization", 10, LocalizationCallBack, ros::TransportHints().tcpNoDelay());  // 订阅定位状态
    ros::Subscriber perception_sub = nh.subscribe(
        "/perception", 10, PerceptionMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber load_pos_sub = nh.subscribe(
        "/palletpos", 10, LoadPosCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber pallet_pos_sub = nh.subscribe(
        "/hook_position", 10, PalletCoorCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber front_scan_sub = nh.subscribe(
        "/scan_bbox_result", 10, FrontScanCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber back_scan_sub = nh.subscribe(
        "/perception_back_bbox", 10, BackScanCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber rs232_sub = nh.subscribe(
        "/robot/serial/rs232/", 10, Rs232CallBack, ros::TransportHints().tcpNoDelay());  // 急停按键
    ros::Subscriber camera0_sub = nh.subscribe(
        "/cam0/status", 1, Camera0CallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber camera7_sub = nh.subscribe(
        "/cam7/status", 1, Camera7CallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber command_msg_sub = nh.subscribe(
        "/cloud/msg/command_msg", 1, CommandMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber airport_sub = nh.subscribe(
        "/cloud/msg/airport_msg", 1, AirPortMsgCallback, ros::TransportHints().tcpNoDelay());

    ros::Publisher refer_path_pub = nh.advertise<robot::path_plan_msg>(
        "/refer_path_msg", 10);
    ros::Publisher plan_path_pub = nh.advertise<robot::path_plan_msg>(
        "/plan_path_msg", 10);
    ros::Publisher plan_status_pub = nh.advertise<robot::path_plan_status>(
        "/path_plan_status", 10);
    ros::Publisher sound_light_sub = nh.advertise<robot::sound_light_msg>(
        "/sound_light_msg", 10);

    ros::Rate loop_rate(10);
    ros::Timer T1 = nh.createTimer(ros::Duration(0.1), T1Callback);

    pathPlanComply.InitParameter();

    double msgPerception;
    double msgNavigation;
    double msgLidar;
    double msgCamera;

    while(ros::ok()) {
        ros::spinOnce();

        pathPlanComply.sysTime.now = ros::Time::now().toSec();

        nh.getParam("/robot/lanechangecmd", pathPlanComply.LaneChangeRequset);
        nh.getParam("/robot/control/accswitch", pathPlanComply.AccSwitch);
        nh.getParam("/robot/planning/pallettype", pathPlanComply.PalletType);
        // 挂钩/托盘位置判定线: canbus 启动时发布, 读失败保持当前值
        nh.getParam("/canbus/hookposition/min", pathPlanComply.HookPosMin);
        nh.getParam("/canbus/hookposition/max", pathPlanComply.HookPosMax);
        nh.getParam("/canbus/palletposition/min", pathPlanComply.PalletPosMin);
        nh.getParam("/canbus/palletposition/max", pathPlanComply.PalletPosMax);

        // 调用顺序属于现役数据流：先更新任务进度和速度上限，再生成参考路径，
        // 最后叠加人工/急停/断网/等待区等安全条件并发布最终路径与状态。
        pathPlanComply.PathPlanProcess();  // 这里从task读取任务数据包括:挡位，轨迹，速度
        pathPlanComply.PublishReferPath(refer_path_pub);
        pathPlanComply.PublishPlanPath(plan_path_pub, sound_light_sub);
        pathPlanComply.PublishPathPlanStatus(plan_status_pub);

        loop_rate.sleep();
    }

    return 0;
}
