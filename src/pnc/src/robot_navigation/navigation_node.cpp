#include "ros/ros.h" 
#include <std_msgs/String.h>
#include "robot/navigation_msg.h"
#include "robot/hook_position.h"
#include "robot/task_plan_msg.h"
#include <iostream>
#include <fstream>
#include "common/struct_type.h"
#include "ivmsglocpos.h"

double latitude_temp;
double longitude_temp;
double altitude_temp;
double speed_temp;
double head_temp;
double utmx_temp;
double utmy_temp;
double utmz_temp;
double hook_xg;
double hook_yg;
double hook_heading;

double nav_head_temp;
double nav_utmx_temp;
double nav_utmy_temp;
double nav_utmz_temp;

int HookEnable = 0;
int PalletType = 0;

void ExtractGpsData(ros::Publisher ros_pub)
{
    if(head_temp < 0) head_temp += 360;
    if(head_temp > 360.0) head_temp -= 360;
    if(hook_heading < 0) hook_heading += 360;
    if(hook_heading > 360.0) hook_heading -= 360;

    robot::navigation_msg gps_data;

    gps_data.lat = latitude_temp ;
    gps_data.lon = longitude_temp ;
    gps_data.altitude = altitude_temp;
    gps_data.gpsSpeed = speed_temp;
    gps_data.rtkState = "no_fixed";
    gps_data.xAxis = utmx_temp;
    gps_data.yAxis = utmy_temp;
    gps_data.zAxis = utmz_temp;
    gps_data.heading = head_temp;
    gps_data.hook_xg = hook_xg;
    gps_data.hook_yg = hook_yg;
    gps_data.hook_heading = hook_heading;

    ros_pub.publish(gps_data);
}

void PalletCoorCallback(const robot::hook_position::ConstPtr& msg)
{
    double x = msg->center_point_x;
    double y = msg->center_point_y;
    double l = hypot(x, y);
    double h = msg->beta  * M_PI / 180.0;

    if(fabs(x) < 1e-3) x = 1e-3;
    if(fabs(y) < 1e-3) y = 1e-3;

    double theta = -atan2(y, x) - h;

    if(theta > 2 * M_PI) theta -= 2 * M_PI;
    if(theta < 0) theta += 2 * M_PI;

    if(HookEnable == 1) {
        latitude_temp = 0.0;
        longitude_temp = 0.0;
        altitude_temp = 0.0;
        head_temp = 90.0 - msg->beta;
        utmx_temp = l * cos(theta);
        utmy_temp = l * sin(theta);
        utmz_temp = 0.0;

	if(PalletType == 0) {
            head_temp = nav_head_temp;
            utmx_temp = nav_utmx_temp;
            utmy_temp = nav_utmy_temp;
            utmz_temp = nav_utmz_temp;
	}

        hook_xg = 0.0;
        hook_yg = 0.0;
        hook_heading = 90.0;
    }
}

void TaskPlanCallBack(const robot::task_plan_msg::ConstPtr& msg)
{
    if(msg->taskType == ADAPTIVEHOOK) HookEnable = 1;
    else HookEnable = 0;
}

void NavigationCallBack(const ivlocmsg::ivmsglocpos::ConstPtr& msg)
{
    speed_temp = msg->velocity;

    if(HookEnable == 0) {
        latitude_temp = msg->lat;
        longitude_temp = msg->lon;
        altitude_temp = msg->height;
        speed_temp = msg->velocity;
        head_temp = msg->heading;
        utmx_temp = msg->xg;
        utmy_temp = msg->yg;
        utmz_temp = msg->zg;
        hook_xg = msg->hook_xg;
        hook_yg = msg->hook_yg;
        hook_heading = msg->hook_heading;
    }

    if(HookEnable == 1) {
        nav_head_temp = msg->heading;
        nav_utmx_temp = msg->xg;
        nav_utmy_temp = msg->yg;
        nav_utmz_temp = msg->zg;
    }
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"serial_send_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe(
        "/localization", 1, NavigationCallBack,
        ros::TransportHints().tcpNoDelay());
    ros::Subscriber task_plan_sub = nh.subscribe(
        "/task_plan_msg", 1, TaskPlanCallBack,
        ros::TransportHints().tcpNoDelay());
    ros::Subscriber pallet_pos_sub = nh.subscribe(
        "/hook_position", 1, PalletCoorCallback,
        ros::TransportHints().tcpNoDelay());
    ros::Publisher read_pub = nh.advertise<robot::navigation_msg>(
        "/navigation_msg",10);

    ros::Rate loop_rate(50);

    while(ros::ok()) {
        ExtractGpsData(read_pub);

        nh.getParam("/robot/planning/pallettype", PalletType);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

