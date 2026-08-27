#include "simviewer.h"

void GNSSCallback(const view::navigation_msg &msg)
{
    viewer->gnss.x = msg.xAxis;
    viewer->gnss.y = msg.yAxis;
    viewer->gnss.z = msg.zAxis;
    viewer->gnss.yaw = msg.heading;
    viewer->gnss.speed = msg.gpsSpeed;
    viewer->gnss.yaw = 90.0 - msg.heading;
    viewer->hook_xg = msg.hook_xg - 2.8;
    viewer->hook_yg = msg.hook_yg;
    viewer->hook_heading = 90.0 - msg.hook_heading;

    if(viewer->gnss.yaw > 360.0) viewer->gnss.yaw -= 360.0;
    if(viewer->gnss.yaw <-360.0) viewer->gnss.yaw += 360.0;
    if(viewer->hook_heading > 360.0) viewer->hook_heading -= 360.0;
    if(viewer->hook_heading <-360.0) viewer->hook_heading += 360.0;

    viewer->hook_heading = viewer->hook_heading * M_PI / 180.0;

    //set world frame
    double x = 0;
    double y = 0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    tf::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);

    odom_trans.setOrigin(tf::Vector3(x, y, z)); 
    odom_trans.setRotation(quat);

    vehicle_pub.publish(viewer->DrawVehicle(0, 0.01));
}

void PerceptionCallback(const view::perception &msg)
{
    int size_a = msg.objs.size();
    double speed = 0.0;

    view::perception objs;

    for(auto i : msg.objs)  {
        bool l = 0;
        bool w = 0;

        //if(i.dx > 0.0 && i.dx < 5.0) l = 1;
        //if(i.dy > 1.0 && i.dy < 5.0) w = 1;

        //if(l == 1 && w == 1) {
            objs.objs.push_back(i);
	    speed = hypot(i.vx, i.vy);
        //}
    }

    int size_b = objs.objs.size(); 

    lidar_pub.publish(viewer->DrawLidarObjects(objs, 1000, 0.5));

    std::cout << " lidar objects = " << size_a
              << " valued = " << size_b
              << " speed = " << speed << std::endl;
}

void RoutingCallback(const view::path_plan_msg &msg)
{
    ConvexPoints points;

    if(msg.x.size() != msg.y.size()) {
        ROS_ERROR("Routing points size mismatch !!!"); return;}

    for(int i = 0; i < msg.x.size(); i++)  {
        ConvexPoint p;

	p.xg = msg.x[i];
	p.yg = msg.y[i];

	points.push_back(p);
    }

    routing_pub.publish(viewer->DrawPath(points, 100.0, 1.0, 1.0, 0.0));    
}

void PlanningCallback(const view::path_plan_msg &msg)
{
    ConvexPoints points;

    if(msg.x.size() != msg.y.size()) {
        ROS_ERROR("Planning points size mismatch !!!"); return;}

    for(int i = 0; i < msg.x.size(); i++)  {
        ConvexPoint p;

        p.xg = msg.x[i];
        p.yg = msg.y[i];

        points.push_back(p);
    }

    planning_pub.publish(viewer->DrawPath(points, 100.0, 0.0, 1.0, 0.0));
}

void LoadposCallback(const view::palletpos &msg)
{
    ConvexPoint p;

    p.xg = msg.xg;
    p.yg = msg.yg;

    loadpos_pub.publish(viewer->DrawPoint(
        p, 200, 100.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0));

}

void Timer1Callback(const ros::TimerEvent &real)
{
}

