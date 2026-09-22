#include "simviewer.h"
#include <sstream>
#include <cmath>

ConvexPoints robot::SimViewer::LoadMap(std::string file)
{
    double x_ = 0.0;
    double y_ = 0.0;

    ConvexPoints points;
    std::ifstream inFile(file);

    if(!inFile.is_open()) {
        ROS_ERROR("Map CSV open failed: %s", file.c_str());
        return points;
    }
    std::string line;
    std::size_t invalid_rows = 0;
    while(std::getline(inFile, line)) {
        if(line.compare(0, 3, "\xEF\xBB\xBF") == 0) line.erase(0, 3);
        const auto first = line.find_first_not_of(" \t\r");
        if(first == std::string::npos || line[first] == '#') continue;
        std::istringstream row(line);
        double values[5] = {};
        std::size_t columns = 0;
        bool valid = true;
        // 每次消费整行，兼容3列旧图、4列限速图及带第5列旧标志的图。
        for(;;) {
            if(columns == 5 || !(row >> values[columns]) || !std::isfinite(values[columns])) {
                valid = false;
                break;
            }
            ++columns;
            row >> std::ws;
            if(row.eof()) break;
            char comma = 0;
            if(!(row >> comma) || comma != ',') {
                valid = false;
                break;
            }
        }
        if(!valid || columns < 3 || (columns >= 4 && values[3] < 0)) {
            ++invalid_rows;
            continue;
        }
        ConvexPoint p;
        double z = 90.0 - values[2];

	if(z > 360.0) z -= 360.0;
	if(z <-360.0) z += 360.0;

        p.xg = values[0];
        p.yg = values[1];
        p.zg = z / 180.0 * M_PI;

        if(hypot(p.xg - x_, p.yg - y_) > 0.1)  {
            points.push_back(p);
            x_ = p.xg;
            y_ = p.yg;
        }
    }

    if(inFile.bad()) {
        ROS_ERROR("Map CSV read failed: %s", file.c_str());
        return ConvexPoints();
    }
    if(invalid_rows) ROS_WARN("Map CSV skipped %zu invalid rows: %s", invalid_rows, file.c_str());

    int size = points.size();

    ROS_INFO("Load map OK ! size = %d", size);

    return points;
}

SHAPES robot::SimViewer::DrawPoint(
           const ConvexPoint &point, int id, double t,
           double r, double g, double b,
           double x, double y, double z)
{
    geometry_msgs::Point node;
    SHAPE Point;
    SHAPES obj;

    Point.header.frame_id = "robot";
    Point.ns = "simviewer";
    Point.header.stamp = ros::Time::now();
    Point.action = SHAPE::ADD;
    Point.lifetime = ros::Duration(t);
    Point.pose.orientation.x = 0.0;
    Point.pose.orientation.y = 0.0;
    Point.pose.orientation.z = 0.0;
    Point.pose.orientation.w = 1.0;
    Point.type = SHAPE::POINTS;
    Point.id = id;
    Point.scale.x = x;
    Point.scale.y = y;
    Point.scale.z = z;
    Point.color.r = 1.0;
    Point.color.g = 0.0;
    Point.color.b = 0.0;
    Point.color.a = 1.0;
    Point.points.clear();
    Point.points.resize(1);
    Point.points[0].x = point.xg;
    Point.points[0].y = point.yg;
    Point.points[0].z = 1.0;

    obj.markers.push_back(Point);

    return obj;
}

SHAPES robot::SimViewer::DrawPath(
           const ConvexPoints &points, double t, 
           double r, double g, double b)
{
    geometry_msgs::Point node;
    SHAPE Line;
    SHAPES obj;

    Line.header.frame_id = "robot";
    Line.ns = "simviewer";
    Line.header.stamp = ros::Time::now();
    Line.action = SHAPE::ADD;
    Line.lifetime = ros::Duration(t);
    Line.pose.orientation.x = 0.0;
    Line.pose.orientation.y = 0.0;
    Line.pose.orientation.z = 0.0;
    Line.pose.orientation.w = 1.0;
    Line.type = SHAPE::LINE_STRIP;
    Line.scale.x = 0.05;
    Line.color.r = r;
    Line.color.g = g;
    Line.color.b = b;
    Line.color.a = 1.0;
    Line.id = 100;

    for(auto i : points)  {
        node.x = i.xg;
        node.y = i.yg;
        node.z = 0.0;
        Line.points.push_back(node);
    }

    obj.markers.push_back(Line);

    return obj;
}

SHAPES robot::SimViewer::DrawMap(const ConvexPoints &points, int id, double t)
{
    geometry_msgs::Point node;
    SHAPE Line;
    SHAPES obj;

    Line.header.frame_id = "robot";
    Line.ns = "simviewer";
    Line.header.stamp = ros::Time::now();
    Line.action = SHAPE::ADD;
    Line.lifetime = ros::Duration(t);
    Line.pose.orientation.x = 0.0;
    Line.pose.orientation.y = 0.0;
    Line.pose.orientation.z = 0.0;
    Line.pose.orientation.w = 1.0;
    Line.type = SHAPE::LINE_STRIP;
    Line.scale.x = 0.05;
    Line.color.r = 1.0;
    Line.color.g = 1.0;
    Line.color.b = 1.0;
    Line.color.a = 1.0;
    Line.id = id;

    for(auto i : points)  {
        node.x = i.xg;
        node.y = i.yg;
        node.z = 0.0;
        Line.points.push_back(node);
    }
    
    obj.markers.push_back(Line);

    Line.points.clear();
    Line.header.frame_id = "robot";
    Line.ns = "simviewer";
    Line.header.stamp = ros::Time::now();
    Line.action = SHAPE::ADD;
    Line.lifetime = ros::Duration(t);
    Line.pose.orientation.x = 0.0;
    Line.pose.orientation.y = 0.0;
    Line.pose.orientation.z = 0.0;
    Line.pose.orientation.w = 1.0;
    Line.type = SHAPE::LINE_STRIP;
    Line.scale.x = 0.05;
    Line.color.r = 1.0;
    Line.color.g = 0.0;
    Line.color.b = 1.0;
    Line.color.a = 1.0;
    Line.id = ++id;

    for(auto i : points)  {
        node.x = i.xg + 1.0 * sin(i.zg);
        node.y = i.yg - 1.0 * cos(i.zg);
        node.z = 0.0;
        Line.points.push_back(node);
    }

    obj.markers.push_back(Line);

    Line.points.clear();
    Line.header.frame_id = "robot";
    Line.ns = "simviewer";
    Line.header.stamp = ros::Time::now();
    Line.action = SHAPE::ADD;
    Line.lifetime = ros::Duration(t);
    Line.pose.orientation.x = 0.0;
    Line.pose.orientation.y = 0.0;
    Line.pose.orientation.z = 0.0;
    Line.pose.orientation.w = 1.0;
    Line.type = SHAPE::LINE_STRIP;
    Line.scale.x = 0.05;
    Line.color.r = 1.0;
    Line.color.g = 0.0;
    Line.color.b = 1.0;
    Line.color.a = 1.0;
    Line.id = ++id;

    for(auto i : points)  {
        node.x = i.xg - 1.0 * sin(i.zg);
        node.y = i.yg + 1.0 * cos(i.zg);
        node.z = 0.0;
        Line.points.push_back(node);
    }

    ROS_INFO("Draw map OK !");

    obj.markers.push_back(Line);

    return obj;
}

SHAPES robot::SimViewer::DrawVehicle(int id, double t)
{
    SHAPE draw_vehicle_centre;
    SHAPE draw_vehicle_shape;
    SHAPE draw_vehicle_wheellf;
    SHAPE draw_vehicle_wheelrf;
    SHAPE draw_vehicle_nearest_point;
    SHAPE draw_vehicle_referns_point;
    SHAPE draw_vehicle_text;

    unsigned int ID = id;
    double theta = gnss.yaw * M_PI / 180.0;
    double cur_x = gnss.x;
    double cur_y = gnss.y;
    double cur_z = 0.0;
    double cur_spd = gnss.speed;
    double steer = 0.0 * M_PI / 180.0;

    while(theta > M_PI)  theta -= 2.0 * M_PI;
    while(theta <= -M_PI)  theta += 2.0 * M_PI;
    while(hook_heading > M_PI)  hook_heading -= 2.0 * M_PI;
    while(hook_heading <= -M_PI)  hook_heading += 2.0 * M_PI;

    //STEP--0 : Init
    SHAPES marker_array;
    //STEP--1 : Draw vheicle centre 
    draw_vehicle_centre.header.frame_id = "robot";
    draw_vehicle_centre.ns = "simviewer"; 
    draw_vehicle_centre.header.stamp = ros::Time::now();
    draw_vehicle_centre.action = SHAPE::ADD;
    draw_vehicle_centre.pose.orientation.x = 0.0;
    draw_vehicle_centre.pose.orientation.y = 0.0;
    draw_vehicle_centre.pose.orientation.z = 0.0;
    draw_vehicle_centre.pose.orientation.w = 1.0;
    //shape of point
    draw_vehicle_centre.id = ID++;  
    draw_vehicle_centre.type = SHAPE::POINTS;
    // POINTS markers use x and y scale for width/height respectively
    draw_vehicle_centre.scale.x = 0.15;
    draw_vehicle_centre.scale.y = 0.15;
    // Points are red
    draw_vehicle_centre.color.r = 1.0;
    draw_vehicle_centre.color.g = 0.0;
    draw_vehicle_centre.color.b = 0.0;
    draw_vehicle_centre.color.a = 1.0; 
    draw_vehicle_centre.points.clear();
    draw_vehicle_centre.points.resize(1); 
    draw_vehicle_centre.points[0].x = cur_x;
    draw_vehicle_centre.points[0].y = cur_y;
    draw_vehicle_centre.points[0].z = 1.0;

    marker_array.markers.push_back(draw_vehicle_centre);

    //STEP--2 : Draw vheicle body & wheel
    draw_vehicle_shape.header.frame_id = "robot";
    draw_vehicle_shape.ns = "simviewer";
    draw_vehicle_shape.header.stamp = ros::Time::now();
    draw_vehicle_shape.action = SHAPE::ADD;
    draw_vehicle_shape.pose.orientation.x = 0.0;
    draw_vehicle_shape.pose.orientation.y = 0.0;
    draw_vehicle_shape.pose.orientation.z = 0.0;
    draw_vehicle_shape.pose.orientation.w = 1.0;
    //shape of line
    draw_vehicle_shape.id = ID++;
    draw_vehicle_shape.type = SHAPE::LINE_STRIP;
    //LINE_STRIP markers use x respectively
    draw_vehicle_shape.scale.x = 0.1;
    //LINE_STRIP is yellow
    draw_vehicle_shape.color.r = 1.0;
    draw_vehicle_shape.color.g = 1.0;
    draw_vehicle_shape.color.b = 0.0;
    draw_vehicle_shape.color.a = 1.0;
    //Draw shape of the vehicle 
    draw_vehicle_shape.points.clear();
    draw_vehicle_shape.points.resize(5);
    printf("x:%f y:%f\n", cur_x, cur_y);
    draw_vehicle_shape.points[0].x = 2.3*cos(theta) - 0.73*sin(theta) + cur_x;
    draw_vehicle_shape.points[0].y = 2.3*sin(theta) + 0.73*cos(theta) + cur_y;
    draw_vehicle_shape.points[0].z = 1.0;
    draw_vehicle_shape.points[1].x = 2.3*cos(theta) + 0.73*sin(theta) + cur_x;
    draw_vehicle_shape.points[1].y = 2.3*sin(theta) - 0.73*cos(theta) + cur_y;
    draw_vehicle_shape.points[1].z = 1.0;
    draw_vehicle_shape.points[2].x =-0.8*cos(theta) + 0.73*sin(theta) + cur_x;
    draw_vehicle_shape.points[2].y =-0.8*sin(theta) - 0.73*cos(theta) + cur_y;
    draw_vehicle_shape.points[2].z = 1.0;
    draw_vehicle_shape.points[3].x =-0.8*cos(theta) - 0.73*sin(theta) + cur_x;
    draw_vehicle_shape.points[3].y =-0.8*sin(theta) + 0.73*cos(theta) + cur_y;
    draw_vehicle_shape.points[3].z = 1.0;
    draw_vehicle_shape.points[4].x = 2.3*cos(theta) - 0.73*sin(theta) + cur_x;
    draw_vehicle_shape.points[4].y = 2.3*sin(theta) + 0.73*cos(theta) + cur_y;
    draw_vehicle_shape.points[4].z = 1.0;

    marker_array.markers.push_back(draw_vehicle_shape);

    //Draw pallet
    // draw_vehicle_shape.header.frame_id = "robot";
    // draw_vehicle_shape.ns = "simviewer";
    // draw_vehicle_shape.header.stamp = ros::Time::now();
    // draw_vehicle_shape.action = SHAPE::ADD;
    // draw_vehicle_shape.pose.orientation.x = 0.0;
    // draw_vehicle_shape.pose.orientation.y = 0.0;
    // draw_vehicle_shape.pose.orientation.z = 0.0;
    // draw_vehicle_shape.pose.orientation.w = 1.0;
    // //shape of line
    // draw_vehicle_shape.id = ID++;
    // draw_vehicle_shape.type = SHAPE::LINE_STRIP;
    // //LINE_STRIP markers use x respectively
    // draw_vehicle_shape.scale.x = 0.2;
    // //LINE_STRIP is yellow
    // draw_vehicle_shape.color.r = 1.0;
    // draw_vehicle_shape.color.g = 1.0;
    // draw_vehicle_shape.color.b = 0.0;
    // draw_vehicle_shape.color.a = 1.0;
    //Draw shape of the vehicle 
    /**
    draw_vehicle_shape.points.clear();
    draw_vehicle_shape.points.resize(5);
    draw_vehicle_shape.points[0].x = 1.3*cos(hook_heading) - 1.75*sin(hook_heading) + hook_xg;
    draw_vehicle_shape.points[0].y = 1.3*sin(hook_heading) + 1.75*cos(hook_heading) + hook_yg;
    draw_vehicle_shape.points[0].z = 1.0;
    draw_vehicle_shape.points[1].x = 1.3*cos(hook_heading) + 1.75*sin(hook_heading) + hook_xg;
    draw_vehicle_shape.points[1].y = 1.3*sin(hook_heading) - 1.75*cos(hook_heading) + hook_yg;
    draw_vehicle_shape.points[1].z = 1.0;
    draw_vehicle_shape.points[2].x =-1.3*cos(hook_heading) + 1.75*sin(hook_heading) + hook_xg;
    draw_vehicle_shape.points[2].y =-1.3*sin(hook_heading) - 1.75*cos(hook_heading) + hook_yg;
    draw_vehicle_shape.points[2].z = 1.0;
    draw_vehicle_shape.points[3].x =-1.3*cos(hook_heading) - 1.75*sin(hook_heading) + hook_xg;
    draw_vehicle_shape.points[3].y =-1.3*sin(hook_heading) + 1.75*cos(hook_heading) + hook_yg;
    draw_vehicle_shape.points[3].z = 1.0;
    draw_vehicle_shape.points[4].x = 1.3*cos(hook_heading) - 1.75*sin(hook_heading) + hook_xg;
    draw_vehicle_shape.points[4].y = 1.3*sin(hook_heading) + 1.75*cos(hook_heading) + hook_yg;
    draw_vehicle_shape.points[4].z = 1.0;

    marker_array.markers.push_back(draw_vehicle_shape);
    */

    //Draw left wheel
    // draw_vehicle_wheellf.header.frame_id = "robot";
    // draw_vehicle_wheellf.ns = "simviewer";
    // draw_vehicle_wheellf.header.stamp = ros::Time::now();
    // draw_vehicle_wheellf.action = SHAPE::ADD;
    // draw_vehicle_wheellf.pose.orientation.x = 0.0;
    // draw_vehicle_wheellf.pose.orientation.y = 0.0;
    // draw_vehicle_wheellf.pose.orientation.z = 0.0;
    // draw_vehicle_wheellf.pose.orientation.w = 1.0;
    // //shape of line
    // draw_vehicle_wheellf.id = ID++;
    // draw_vehicle_wheellf.type = SHAPE::LINE_STRIP;
    // //LINE_STRIP markers use x respectively
    // draw_vehicle_wheellf.scale.x = 0.5;
    // //LINE_STRIP is red
    // draw_vehicle_wheellf.color.r = 1.0;
    // draw_vehicle_wheellf.color.g = 0.0;
    // draw_vehicle_wheellf.color.b = 0.0;
    // draw_vehicle_wheellf.color.a = 1.0;
    // draw_vehicle_wheellf.points.clear();
    // draw_vehicle_wheellf.points.resize(2);
    // draw_vehicle_wheellf.points[0].x = 1.5*cos(theta) - 1.2*sin(theta) + cur_x;
    // draw_vehicle_wheellf.points[0].y = 1.5*sin(theta) + 1.2*cos(theta) + cur_y;
    // draw_vehicle_wheellf.points[0].z = 1.0;
    // draw_vehicle_wheellf.points[1].x = 1.5*cos(theta) - 1.2*sin(theta) + cur_x;
    // draw_vehicle_wheellf.points[1].y = 1.5*sin(theta) + 1.2*cos(theta) + cur_y;
    // draw_vehicle_wheellf.points[1].z = 1.0;

    // marker_array.markers.push_back(draw_vehicle_wheellf);

    // //Draw right wheel
    // draw_vehicle_wheelrf.header.frame_id = "robot";
    // draw_vehicle_wheelrf.ns = "simviewer";
    // draw_vehicle_wheelrf.header.stamp = ros::Time::now();
    // draw_vehicle_wheelrf.action = SHAPE::ADD;
    // draw_vehicle_wheelrf.pose.orientation.x = 0.0;
    // draw_vehicle_wheelrf.pose.orientation.y = 0.0;
    // draw_vehicle_wheelrf.pose.orientation.z = 0.0;
    // draw_vehicle_wheelrf.pose.orientation.w = 1.0;
    // //shape of line
    // draw_vehicle_wheelrf.id = ID++;
    // draw_vehicle_wheelrf.type = SHAPE::LINE_STRIP;
    // //LINE_STRIP markers use x respectively
    // draw_vehicle_wheelrf.scale.x = 0.5;
    // //LINE_STRIP is red
    // draw_vehicle_wheelrf.color.r = 1.0;
    // draw_vehicle_wheelrf.color.g = 0.0;
    // draw_vehicle_wheelrf.color.b = 0.0;
    // draw_vehicle_wheelrf.color.a = 1.0;
    // draw_vehicle_wheelrf.points.clear();
    // draw_vehicle_wheelrf.points.resize(2);
    // draw_vehicle_wheelrf.points[0].x = 1.5*cos(theta) + 1.2*sin(theta) + cur_x;
    // draw_vehicle_wheelrf.points[0].y = 1.5*sin(theta) - 1.2*cos(theta) + cur_y;
    // draw_vehicle_wheelrf.points[0].z = 1.0;
    // draw_vehicle_wheelrf.points[1].x = 1.5*cos(theta) + 1.2*sin(theta) + cur_x;
    // draw_vehicle_wheelrf.points[1].y = 1.5*sin(theta) - 1.2*cos(theta) + cur_y;
    // draw_vehicle_wheelrf.points[1].z = 1.0;

    // marker_array.markers.push_back(draw_vehicle_wheelrf);

    //STEP--3 : Draw vehicle kinematics info
    draw_vehicle_text.header.frame_id = "robot";
    draw_vehicle_text.ns = "simviewer";
    draw_vehicle_text.header.stamp = ros::Time::now();
    draw_vehicle_text.action = SHAPE::ADD;
    draw_vehicle_text.pose.orientation.x = 0.0;
    draw_vehicle_text.pose.orientation.y = 0.0;
    draw_vehicle_text.pose.orientation.z = 0.0;
    draw_vehicle_text.pose.orientation.w = 1.0;
    //Shape of text
    draw_vehicle_text.id = ID++;
    draw_vehicle_text.type = SHAPE::TEXT_VIEW_FACING;
    //Text markers use z respectively
    draw_vehicle_text.scale.z = 0.5;
    //Text is white
    draw_vehicle_text.color.r = 1.0;
    draw_vehicle_text.color.g = 1.0;
    draw_vehicle_text.color.b = 1.0;
    draw_vehicle_text.color.a = 1.0;
    //Text position
    draw_vehicle_text.pose.position.x = 2.0 + cur_x;
    draw_vehicle_text.pose.position.y = 2.0 + cur_y;
    draw_vehicle_text.pose.position.z = 1.0;

    //Text content
    std::stringstream value;
    std::string text = "";
    value.str("");
    value.setf(std::ios::fixed);value.width(4);value.setf(std::ios::right);value.precision(1);
    value << cur_spd;
    text += "cur_spd: " + value.str() + " m/s\n";
    value.str("");
    value.setf(std::ios::fixed);value.width(4);value.setf(std::ios::right);value.precision(1);
    value << cur_x;
    text += "cur_x: " + value.str() + " lat\n";
    value.str("");
    value.setf(std::ios::fixed);value.width(4);value.setf(std::ios::right);value.precision(1);
    value << cur_y;
    text += "cur_y: " + value.str() + " lon\n";
    value.str("");
    value.setf(std::ios::fixed);value.width(4);value.setf(std::ios::right);value.precision(1);
    value << (theta * 180.0 / M_PI);
    text += "cur_yaw: " + value.str() + " deg\n";

    draw_vehicle_text.text = text;

    marker_array.markers.push_back(draw_vehicle_text);

    ROS_INFO("Draw vehicle OK !");

    return marker_array;
}

SHAPES robot::SimViewer::DrawLidarObjects(
           const view::perception &objs, int id, double t)
{
    geometry_msgs::Point point;

    SHAPE object;
    SHAPES _objects;

    int size = objs.objs.size();

    for(int i = 0; i < size; i++)  {
        object.header.frame_id = "robot";
        object.ns = "simviewer";
        object.id = 8 * (i + id);
        object.type = SHAPE::CUBE;
        object.action = SHAPE::ADD;
        object.lifetime = ros::Duration(t);

        double theta = 90.0 - objs.objs[i].heading;

        if(theta > 360.0) theta -= 360.0;
        if(theta <-360.0) theta += 360.0;

        theta = theta * M_PI / 180.0;

        tf::Quaternion quat;
        quat.setRPY(0, 0, theta);

        quaternionTFToMsg(quat, object.pose.orientation);

        double xg = objs.objs[i].x;
        double yg = objs.objs[i].y;
        double length = objs.objs[i].dy;
	double width = objs.objs[i].dx;
	double height = objs.objs[i].height;

        object.color.r = 0.2;
        object.color.g = 0.6;
        object.color.b = 0.6;
        object.color.a = 1.0;
        object.pose.position.x = xg;
        object.pose.position.y = yg;
        object.pose.position.z = 0.0;
        object.scale.x = length;
        object.scale.y = width;
        object.scale.z = height;

        _objects.markers.push_back(object);
    }

    return _objects;
}


