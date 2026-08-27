#include "perception_msg_convert.h"
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Matrix3x3.h>

robot::perception mPerception;
robot::navigation_msg mGPS;

ros::Subscriber gnss_sub;
ros::Subscriber box_sub;
ros::Publisher perception_pub;
ros::Publisher udp_pub;
ros::Publisher string_pub;

std::string StringToHex(const std::string &data)
{
    const std::string hex = "0123456789ABCDEF";
    std::stringstream ss;

    for (std::string::size_type i = 0; i < data.size(); ++i)
        ss << hex[(unsigned char)data[i] >> 4] << hex[(unsigned char)data[i] & 0xf];

    return ss.str();
}

void GNSSCallback(const ivlocmsg::ivmsglocpos &msg)
{
    mGPS.lat = msg.lat;
    mGPS.lon = msg.lon;
    mGPS.altitude = msg.height;
    mGPS.gpsSpeed = msg.velocity;
    mGPS.xAxis = msg.xg;
    mGPS.yAxis = msg.yg;
    mGPS.zAxis = msg.zg;
    mGPS.heading = msg.heading;
}

void BoxMsgCallBack(const visualization_msgs::MarkerArray &msg)
{
    mPerception.objs.clear();

    if (!msg.markers.size())
        return;

    double x = mGPS.xAxis;
    double y = mGPS.yAxis;
    double h = 90.0 - mGPS.heading;

    if (h > 360.0)
        h -= 360.0;
    if (h < 0.0)
        h += 360.0;

    double theta = h * M_PI / 180.0;

    for (auto i : msg.markers)
    {
        robot::object obj;
        double x_ = i.pose.position.x;
        double y_ = i.pose.position.y;
        double heading_to_base = tf::getYaw(i.pose.orientation);
        printf("i 朝向: %f \n", heading_to_base / M_PI * 180);
        obj.x = x + x_ * cos(theta) - y_ * sin(theta);
        obj.y = y + x_ * sin(theta) + y_ * cos(theta);
        obj.dx = i.scale.x;
        obj.dy = i.scale.y;

        obj.heading = fmod(heading_to_base / M_PI * 180.0 + mGPS.heading + 90, 360.0);
        if (obj.heading < 0)
        {
            obj.heading += 360.0;
        }

        mPerception.objs.push_back(obj);
    }

    perception_pub.publish(mPerception);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "perception_msg_convert");
    ros::NodeHandle nh;

    gnss_sub = nh.subscribe(
        "/localization", 1, GNSSCallback,
        ros::TransportHints().tcpNoDelay());
    box_sub = nh.subscribe(
        "/box", 1, BoxMsgCallBack,
        ros::TransportHints().tcpNoDelay());

    perception_pub = nh.advertise<robot::perception>(
        "/perception", 10);

    ros::Rate loop(100);

    while (ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
