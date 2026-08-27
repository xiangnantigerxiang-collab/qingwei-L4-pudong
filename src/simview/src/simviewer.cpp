#include "simviewer.h"
#include "view/path_plan_status.h"

ros::Publisher vehicle_pub;
ros::Publisher lidar_pub;
ros::Publisher map_pub;
ros::Publisher routing_pub;
ros::Publisher planning_pub;
ros::Publisher loadpos_pub;
ros::Publisher stoppose_pub;
ros::Publisher globalpath_sub;

tf::Transform odom_trans;

robot::SimViewer *viewer = new robot::SimViewer();
void PathStatusCallback(const view::path_plan_status &msg)
{
    double stop_x = msg.stopX;
    double stop_y = msg.stopY;
    geometry_msgs::PoseStamped stoppose;
    stoppose.header.stamp = ros::Time::now();
    stoppose.header.frame_id = "world";
    stoppose.pose.position.x = stop_x;
    stoppose.pose.position.y = stop_y;
    stoppose.pose.orientation.w = 1.0;
    stoppose_pub.publish(stoppose);
}
int main(int argc, char *argv[])
{
    //--ros init--
    ros::init(argc, argv, "simviewer");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    //--ros topic setup--
    ros::Subscriber path_status_sub = n.subscribe(
        "path_plan_status", 1, PathStatusCallback);
    ros::Subscriber gnss_sub = n.subscribe(
        "navigation_msg", 1, GNSSCallback);
    ros::Subscriber lidar_sub = n.subscribe(
        "/perception", 1, PerceptionCallback);
    ros::Subscriber planning_sub = n.subscribe(
        "/plan_path_msg", 1, PlanningCallback);
    ros::Subscriber routing_sub = n.subscribe(
        "/refer_path_msg", 1, RoutingCallback);
    ros::Subscriber loadpos_sub = n.subscribe(
        "/palletpos", 1, LoadposCallback);

    vehicle_pub = n.advertise<SHAPES>("/robot/simviewer/vehicle", 10);
    lidar_pub = n.advertise<SHAPES>("/robot/simviewer/lidar", 10);
    map_pub = n.advertise<SHAPES>("/robot/simviewer/map", 10);
    routing_pub = n.advertise<SHAPES>("/robot/simviewer/routing", 10);
    planning_pub = n.advertise<SHAPES>("/robot/simviewer/planning", 10);
    loadpos_pub = n.advertise<SHAPES>("/robot/simviewer/loadpos", 10);
    stoppose_pub = n.advertise<geometry_msgs::PoseStamped>("/robot/simviewer/stoppose", 10);
    globalpath_sub = n.advertise<SHAPES>("/robot/simviewer/globalpath", 10);

    //--world frame init--
    tf::TransformBroadcaster odom_broadcaster;
    odom_trans.setOrigin(tf::Vector3(0, 0, 0));
    odom_trans.setRotation(tf::Quaternion(0, 0, 0, 1.0));

    //--load map--
    ConvexPoints points;
    std::string mapfile;

    if (n.getParam("/robot/mapfile", mapfile))
    {
        points = viewer->LoadMap(mapfile);
    }

    //--set timer--
    ros::Timer timer1 = n.createTimer(ros::Duration(2.0), Timer1Callback); // 10Hz

    //--ROS thread running--
    while (ros::ok())
    {
        n.getParam("/robot/simviewer/mapswitch", viewer->sw.map);

        if (viewer->sw.map)
        {
            map_pub.publish(viewer->DrawMap(points, 1000, 10000));
            n.setParam("/robot/simviewer/mapswitch", 0);
        }

        odom_broadcaster.sendTransform(tf::StampedTransform(
            odom_trans, ros::Time::now(), "/robot", "/world"));

        ros::spinOnce();
        loop_rate.sleep();
    }

    //--Processor exit--
    if (ros::ok() == false)
    {
        delete viewer;
    }

    return 0;
}

