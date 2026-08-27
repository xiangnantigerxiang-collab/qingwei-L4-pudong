#include <geometry_msgs/PoseStamped.h>
#include "control_comply.h"


ControlComply controlComply;

void NavigationMsgCallBack(const robot::navigation_msg::ConstPtr &msg)
{
 //  printf("NavigationMsgCallBack\n");
    controlComply.SetNavigationData(*msg);
}

void PathPlanMsgCallBack(const robot::path_plan_msg::ConstPtr &msg)
{
  // printf("PathPlanMsgCallBack\n");
    controlComply.SetPathPlanData(*msg);
}

void CanMsgCallBack(const robot::can_msg::ConstPtr &msg)
{
   //printf("CanMsgCallBack\n");
    controlComply.SetCanData(*msg);
}

void PalletCoorCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    //printf("PalletCoorCallback\n");
    robot::hook_position hook_pos_t;
    auto quat = msg->pose.orientation;
    float vehicle_head_reverse = 0;

    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double a = quat.w * quat.z + quat.x * quat.y;
    double b = quat.y * quat.y + quat.z * quat.z;

    hook_pos_t.center_point_x = x;
    hook_pos_t.center_point_y = y;
    hook_pos_t.center_distance = hypot(x, y);
    hook_pos_t.beta = atan2(2 * a, 1 - 2 * b) * 180.0 / M_PI + 180.0;

    vehicle_head_reverse = 90.0 - 1.0 * hook_pos_t.beta; // confirm? +/-

    if ((vehicle_head_reverse + 180.0) >= 360.0)
        hook_pos_t.beta -= 360.0;

    controlComply.SetPalletCoorData(hook_pos_t);
}

void TlStatusCoorCallback(const robot::TLStatus::ConstPtr &msg)
{
   // printf("TLStatusCoorCallback\n");
    controlComply.SetTlStatusData(*msg);
    ROS_INFO("/camera/tl_status:%d", static_cast<int>(msg->light_status));
}

void PerceptionMsgCallBack(const robot::perception &msg) // zhangyu 20220213
{
  // printf("PerceptionMsgCallBack\n");
    controlComply.LidarObject = msg;
}

void pathStatusCallback(const robot::path_plan_statusConstPtr &msg)
{
  // printf("pathStatusCallback\n");
    controlComply.SetPathStatusData(*msg);
}

void TaskMsgCallBack(const robot::task_plan_msg &msg)
{
    controlComply.setTaskPlanData(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    ros::Subscriber navigation_sub =
        nh.subscribe("navigation_msg", 1, NavigationMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber path_plan_sub =
        nh.subscribe("plan_path_msg", 1, PathPlanMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber path_status_sub = nh.subscribe("path_plan_status", 1, pathStatusCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber can_msg_sub = nh.subscribe("can_msg", 10, CanMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber hook_coor_sub =
        nh.subscribe("/baselink_in_trailer", 1, PalletCoorCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber tl_status_sub =
        nh.subscribe("/camera/tl_status", 1, TlStatusCoorCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber task_sub = nh.subscribe("task_plan_msg", 1, TaskMsgCallBack, ros::TransportHints().tcpNoDelay());

    ros::Subscriber perception_sub = nh.subscribe(
        "perception", 1, PerceptionMsgCallBack,
        ros::TransportHints().tcpNoDelay());

    ros::Publisher control_pub = nh.advertise<robot::control_msg>("control_msg", 10);

    ros::Publisher acc_pub = nh.advertise<robot::acc>("acc", 10);
    controlComply.LoadPathFile("fence");

    ros::Rate loop_rate(20);
    ros::param::set("alarmcmd", 0);
    while (ros::ok())
    {
        nh.getParam("/robot/speed", controlComply.speedCMD);
        nh.getParam("/robot/lanechangecmd", controlComply.LaneChangeCommand);
        nh.setParam("/sound/play", controlComply.SoundPlayCommand);

        ros::spinOnce();

        controlComply.VehicleControl();
        //printf("fence alara\n");
        controlComply.FenceAlarm(); //电子围栏检测，设置标志位
        //printf("publish message\n");
        controlComply.PublishMessage(control_pub);

        acc_pub.publish(controlComply.acc_msg);

        loop_rate.sleep();
    }
    return 0;
}
