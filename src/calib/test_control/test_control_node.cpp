#include "ros/ros.h"
#include "robot/navigation_msg.h"
#include "robot/path_plan_msg.h"
#include "robot/can_msg.h"
#include "robot/control_msg.h"
#include "lat_controller/lat_controller.h"
#include "lat_controller/math_utils.h"

robot::path_plan_msg path_plan_msg;
double vehicle_speed = 0.0;
double cur_steer_angle = 0.0;
Pose2d ego_pose;

double steer_angle = 0.0;

void NavigationMsgCallBack(const robot::navigation_msg::ConstPtr &msg)
{
    ego_pose.x = msg->xAxis;
    ego_pose.y = msg->yAxis;
    ego_pose.heading = math_utils::azimuthToYaw(msg->heading);
}

void PathPlanMsgCallBack(const robot::path_plan_msg::ConstPtr &msg)
{
    path_plan_msg = *msg;
}

void CanMsgCallBack(const robot::can_msg::ConstPtr &msg)
{
    vehicle_speed = msg->vehicleSpeed;
    cur_steer_angle = msg->wheelAngle / 22.0 / 180 * 3.1415926;
}

std::vector<Pose2d> toPath2d(const std::vector<float> &xlist, const std::vector<float> &ylist)
{
    if (xlist.empty() || ylist.empty())
    {
        printf("empty path\n");
        return {};
    }
    printf("xsize:%d, ysize:%d\n", xlist.size(), ylist.size());
    int size = std::min(xlist.size(), ylist.size());
    std::vector<Pose2d> new_path;
    for (int i = 0; i < size; i++)
    {
        printf("x: %f, y: %f\n", xlist[i], ylist[i]);
        new_path.emplace_back(xlist[i], ylist[i]);
    }
    math_utils::computePoseAttr(new_path);
    return new_path;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    ros::Subscriber navigation_sub =
        nh.subscribe("navigation_msg", 1, NavigationMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber path_plan_sub =
        nh.subscribe("plan_path_msg", 1, PathPlanMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber can_msg_sub = nh.subscribe("can_msg", 10, CanMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Publisher control_pub = nh.advertise<robot::control_msg>("control_msg", 10);

    ros::Rate loop_rate(20);
    LatController controller;
    controller.setParameters(1.6, 22, 8);

    while (ros::ok())
    {
        ros::spinOnce();
        robot::control_msg control_msg;
        {
            std::vector<Pose2d> path_2d = toPath2d(path_plan_msg.x, path_plan_msg.y);
            printf("path 2d size:%d\n", path_2d.size());
            float steering_angle = controller.calculate(path_2d, ego_pose, vehicle_speed, cur_steer_angle);
            control_msg.wheelAngle = -steering_angle * 180.0 / M_PI;
        }
        control_pub.publish(control_msg);
        loop_rate.sleep();
    }
    return 0;
}

