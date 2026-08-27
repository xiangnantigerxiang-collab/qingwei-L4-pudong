#include "path_plan_comply.h"
#include "robot/can_msg.h"
#include <cmath>
#include <fstream>
#include <visualization_msgs/MarkerArray.h>
#include "lattice/math_utils.h"
#include "lattice/gis_utils.h"
#include <yaml-cpp/yaml.h>

using Dot = std::pair<double, double>;
const double EPS = 1e-9;

std::vector<Dot> getRectangleVertices(const std::vector<Dot> &centerLine, double width)
{
    if (centerLine.size() != 2)
    {
        throw invalid_argument("centerLine 必须包含两个点");
    }

    if (width <= 0)
    {
        throw invalid_argument("矩形宽度必须大于 0");
    }

    double x1 = centerLine[0].first;
    double y1 = centerLine[0].second;
    double x2 = centerLine[1].first;
    double y2 = centerLine[1].second;

    double vx = x2 - x1;
    double vy = y2 - y1;

    double len = sqrt(vx * vx + vy * vy);

    if (len == 0)
    {
        throw invalid_argument("输入的两个点不能重合");
    }

    double halfWidth = width / 2.0;

    // 垂直于中心线的单位向量
    double nx = -vy / len;
    double ny = vx / len;

    return {
        {x1 + nx * halfWidth, y1 + ny * halfWidth},
        {x2 + nx * halfWidth, y2 + ny * halfWidth},
        {x2 - nx * halfWidth, y2 - ny * halfWidth},
        {x1 - nx * halfWidth, y1 - ny * halfWidth}};
}

double cross(const Dot &A, const Dot &B, const Dot &P)
{
    double ABx = B.first - A.first;
    double ABy = B.second - A.second;
    double APx = P.first - A.first;
    double APy = P.second - A.second;

    return ABx * APy - ABy * APx;
}

// 判断点 P 是否在线段 AB 上
bool onSegment(const Dot &A, const Dot &B, const Dot &P)
{
    if (fabs(cross(A, B, P)) > EPS)
    {
        return false;
    }

    return P.first >= min(A.first, B.first) - EPS &&
           P.first <= max(A.first, B.first) + EPS &&
           P.second >= min(A.second, B.second) - EPS &&
           P.second <= max(A.second, B.second) + EPS;
}

// 将四个顶点按顺时针或逆时针排序
void sortRectangleVertices(std::vector<Dot> &rect)
{
    double cx = 0.0;
    double cy = 0.0;

    for (const auto &p : rect)
    {
        cx += p.first;
        cy += p.second;
    }

    cx /= rect.size();
    cy /= rect.size();

    sort(rect.begin(), rect.end(), [cx, cy](const Dot &a, const Dot &b)
         {
        double angleA = atan2(a.second - cy, a.first - cx);
        double angleB = atan2(b.second - cy, b.first - cx);
        return angleA < angleB; });
}

// 判断点是否在矩形内
// 返回值：0 外部，1 内部，2 边上
int pointInRectangle(std::vector<Dot> rect, const Dot &p)
{
    if (rect.size() != 4)
    {
        throw invalid_argument("rect 必须包含四个顶点");
    }

    sortRectangleVertices(rect);

    bool hasPositive = false;
    bool hasNegative = false;

    for (int i = 0; i < 4; ++i)
    {
        const Dot &A = rect[i];
        const Dot &B = rect[(i + 1) % 4];

        if (onSegment(A, B, p))
        {
            return 2;
        }

        double c = cross(A, B, p);

        if (c > EPS)
        {
            hasPositive = true;
        }
        else if (c < -EPS)
        {
            hasNegative = true;
        }

        if (hasPositive && hasNegative)
        {
            return 0;
        }
    }

    return 1;
}

void PathPlanComply::TrajectoryMove(double offset)
{
    mVirtualPath.clear();

    int points_num = mOriginPath.size();
    double dist_origin = 0.0;

    for (int i = 0; i < points_num - 1; i++)
    {
        XYZ_COOR_S point;

        double x0 = mOriginPath[i].x_axis;
        double y0 = mOriginPath[i].y_axis;
        double x1 = mOriginPath[i + 1].x_axis;
        double y1 = mOriginPath[i + 1].y_axis;
        double theta = GetLineDirection(x1, y1, x0, y0) * M_PI / 180.0;

        theta = theta + M_PI / 2.0;

        point.x_axis = (x0 + x1) / 2.0 + offset * cos(theta);
        point.y_axis = (y0 + y1) / 2.0 + offset * sin(theta);
        point.z_axis = 0.0;
        point.heading = theta * 180.0 / M_PI;
        point.velocity = 5;
        point.p2pDistance = 0.04;
        point.dist_origin = dist_origin + point.p2pDistance;

        mVirtualPath.push_back(point);
    }
}

void PathPlanComply::LaneChange(int quest)
{
    if (mLaneChangeSwitch == 0)
        return; // lane change disable

    if (quest == 1 && mLaneId == 0)
    { // left
        mDrivingPath = mVirtualPath;
        mLaneId = 1;
    }

    if (quest == 2 && mLaneId == 1)
    { // right
        mDrivingPath = mOriginPath;
        mLaneId = 0;
    }
}

PathPlanComply::PathPlanComply()
{
}
PathPlanComply::~PathPlanComply() {}

void PathPlanComply::InitParameter()
{
    mDesireSpeed = 0;
    mKeypoint = 0;
    InitSafetyCheck = 0;
    mDrivingPath.clear();
    mRcvGpsData = false;
    mVehicleData.curGear = GEAR_N;
    mTaskPlanData.workMode = MANUALCONTROLMODE;
    mTaskPlanData.desireSpeed = 0;
    mTaskPlanData.desireGear = GEAR_N;
    mPathPlanStatus.stopX = -1;
    mPathPlanStatus.stopY = -1;
    mPathPlanStatus.stopAngle = 0;
    mPathPlanStatus.taskExecuStatus = NOTASK;
    mHookPos.center_distance = 100;
    mHookPos.center_point_x = 0.0;
    mHookPos.center_point_y = 0.0;
    sysTime.init = ros::Time::now().toSec();
    this->loadAirCraftPorts();
}

void PathPlanComply::loadAirCraftPorts()
{
    std::string task_file = "";
    ros::param::get("task_file", task_file);
    task_file += "aircraft_parking_port.yaml";
    printf("aircraft_parking_port.yaml path :%s\n", task_file.c_str());
    YAML::Node root = YAML::LoadFile(task_file.c_str());
    go_task_id_ = root["go_task_id"].as<int>();
    back_task_id_ = root["back_task_id"].as<int>();
    printf("go_task_id:%d, back_task_id:%d\n",go_task_id_,back_task_id_);
    // 解析port info
    YAML::Node port_arr = root["aircraft_parkingPorts"];
    aircraft_parking_ports_.clear();
    for (std::size_t i = 0; i < port_arr.size(); ++i)
    {
        YAML::Node item = port_arr[i];
        int pid = item["id"].as<int>();

        // 读取子节点坐标
        YAML::Node go_pt = item["go_stop_point"];
        double gx = go_pt["x"].as<double>();
        double gy = go_pt["y"].as<double>();

        YAML::Node back_pt = item["back_stop_point"];
        double bx = back_pt["x"].as<double>();
        double by = back_pt["y"].as<double>();

        AirCraftParkingPort port;
        port.id = pid;
        port.parking = item["parking"].as<uint>();
        port.go_stop_point = Point(gx, gy);
        port.back_stop_point = Point(bx, by);
        aircraft_parking_ports_.push_back(port);
        printf("i: %d,id:%d.parking:%d, gx: %f, gy: %f, bx: %f, by: %f\n", i,pid,port.parking, gx, gy, bx, by);
    }
}

void PathPlanComply::ResetParameter()
{
    mDesireSpeed = 0;
    mKeypoint = 0;
    mDrivingPath.clear();
    mRcvGpsData = false;
    mTaskPlanData.desireSpeed = 0;
    mTaskPlanData.desireGear = GEAR_N;
    mPathPlanStatus.stopX = -1;
    mPathPlanStatus.stopY = -1;
    mPathPlanStatus.stopAngle = 0;
    mPathPlanStatus.taskExecuStatus = NOTASK;
}

bool PathPlanComply::JudgeTaskPlanMsgChanged(
    robot::task_plan_msg tLast, robot::task_plan_msg tNow)
{
    int path_num = tNow.pathList.size();
    printf("tlast type: %d, id:%d, now: type:%d, id:%d\n",
           int(tLast.taskType), tLast.task_id, int(tNow.taskType), tNow.task_id);
    if (tNow.taskType == NOTHING)
        return 0;
    // if (tNow.taskType == DOACTION)
    //     return 0;
    if (path_num == 0)
        return 0;
    if (tLast.pathList.size() != path_num)
        return 1;
    if (tLast.stopX != tNow.stopX)
        return 1;
    if (tLast.stopY != tNow.stopY)
        return 1;
    if (tLast.task_id != tNow.task_id)
        return 1;
    if (tLast.taskType != tNow.taskType)
        return 1;

    for (int i = 0; i < path_num; ++i)
    {
        if (0 != strcmp(tLast.pathList[i].c_str(), tNow.pathList[i].c_str()))
            return 1;
    }
    printf("return 0\n");
    return 0;
}

bool PathPlanComply::JudgeShiftParkingPoint(
    robot::task_plan_msg tLast, robot::task_plan_msg tNow)
{
    if (tNow.taskType == NOTHING)
        return 0;
    if (tNow.taskType == DOACTION)
        return 0;
    if (tNow.pathList.size() == 0)
        return 0;

    if (tNow.stopX == 0 && tNow.stopY == 0)
        return 0;
    if (tNow.stopX == -1 && tNow.stopY == -1)
        return 0;

    double dx = tLast.stopX - tNow.stopX;
    double dy = tLast.stopY - tNow.stopY;

    if (hypot(dx, dy) > 0.5)
        return 1;
    else
        return 0;
}

void PathPlanComply::calcuGlobalPath(robot::task_plan_msg task_plan_t)
{
    printf("=============calcuGlobalPath keyIndex and stopIndex==============\n");
    std::vector<XYZ_COOR_S> path_list;

    if (task_plan_t.taskType == NOTHING)
        return;
    if (task_plan_t.taskType == DOACTION)
        return;

    // 设置当前任务路径
    mDrivingPath.clear();
    for (auto pathName : task_plan_t.pathList)
    {
        path_list.clear();
        path_list = LoadPathFile(pathName);
        for (auto path_point : path_list)
        {
            // printf("Load path point: x=%.2f, y=%.2f, heading=%.2f\n", path_point.x_axis,
            //         path_point.y_axis, path_point.heading);
            mDrivingPath.push_back(path_point);
        }
    }
    if (mDrivingPath.size() < 10)
    {
        printf("路径太短，小于10个点\n");
        return;
    }
    printf("111111mKeyPoint:%d\n", mKeypoint);
    {
        // 程序启动时，查找起始点
        double dist_with_first = 0.0;
        double min_dist = 1e6;
        for (int i = 0; i < mDrivingPath.size() - 10; i++)
        {
            auto p = mDrivingPath[i];
            double dist = hypot(p.x_axis - mNavData.xAxis, p.y_axis - mNavData.yAxis);
            if (i == 0)
            {
                dist_with_first = dist;
            }
            if (dist < min_dist)
            {
                mKeypoint = i;
                min_dist = dist;
            }
        }
        printf("first_dist: %f\n", dist_with_first);
        printf("mKeyPoint:%d, size:%d, min_dist:%f\n", mKeypoint, mDrivingPath.size(), min_dist);
    }

    // connect curpose to driving path
    auto xyz = mDrivingPath.at(mKeypoint);
    double x0 = mNavData.xAxis;
    double y0 = mNavData.yAxis;
    double h0 = mNavData.heading;
    double x1 = xyz.x_axis;
    double y1 = xyz.y_axis;
    double h1 = xyz.heading;

    while (h0 > 360.0)
        h0 -= 360.0;
    while (h0 < 0.0)
        h0 += 360.0;
    while (h1 > 360.0)
        h1 -= 360.0;
    while (h1 < 0.0)
        h1 += 360.0;

    // 横向偏差
    float distance2loadpos = pubalgor.CalculatePoint2PointDistance_P(
        x0, y0, 0, x1, y1, 0);

    if (distance2loadpos > 1.0 && distance2loadpos < 2.5)
    {
        // generate connect path
        printf("Distance to path is %.2f, generating connect path...\n", distance2loadpos);
        XYZ_COOR_S xyz_temp;
        std::vector<XYZ_COOR_S> connect_path;
        xyz_temp.x_axis = x0;
        xyz_temp.y_axis = y0;
        xyz_temp.heading = h0;

        connect_path.emplace_back(xyz_temp);

        for (int i = 0; i < 10; ++i)
        {
            xyz_temp = pubalgor.CalculNextPointByAngle_P(
                connect_path.back().x_axis, connect_path.back().y_axis, h0, 0.2);

            connect_path.emplace_back(xyz_temp);
        }

        for (int i = mKeypoint + 5; i < mDrivingPath.size(); i++)
        {
            connect_path.emplace_back(mDrivingPath[i]);
        }

        // smooth
        auto bak = connect_path;
        connect_path.clear();
        for (int i = 0; i < bak.size() - 1; i += 6)
            connect_path.emplace_back(bak[i]);
        connect_path.emplace_back(bak.back());

        CSpline spline;
        spline.SplinePointSet(connect_path, mDrivingPath, 0.1);
        // 计算path
        calcuPath(mDrivingPath);
        mKeypoint = pubalgor.FindKeyPointByTargetPoint_P(
            mDrivingPath, mNavData.xAxis, mNavData.yAxis);
        printf("3333mKeyPoint:%d\n", mKeypoint);
    }
    // 计算停车点id
    float stop_x = task_plan_t.stopX;
    float stop_y = task_plan_t.stopY;
    {
        // 程序启动时，查找停车点
        double min_dist = 1e6;
        for (int i = 4; i < mDrivingPath.size(); i++)
        {
            auto p = mDrivingPath[i];
            double dist = hypot(p.x_axis - stop_x, p.y_axis - stop_y);
            if (dist < min_dist)
            {
                mStopIndex = i;
                min_dist = dist;
            }
        }
        printf("4444mStopIndex:%d, dist:%f\n", mStopIndex, min_dist);
    }
}

void PathPlanComply::calcuPath(std::vector<XYZ_COOR_S> &path)
{
    if (path.empty())
    {
        return;
    }
    path.front().dist_origin = 0.0;
    for (int i = 0; i < path.size(); i++)
    {
        if (i < path.size() - 1)
        {
            double dx = path.at(i + 1).x_axis - path.at(i).x_axis;
            double dy = path.at(i + 1).y_axis - path.at(i).y_axis;
            path.at(i).heading = fmod(90 - atan2(dy, dx) * 180.0 / M_PI + 360.0, 360.0);
            path.at(i + 1).dist_origin = path.at(i).dist_origin + hypot(dx, dy);
        }
        else
        {
            path.at(i).heading = path.at(i - 1).heading;
        }
    }
}

int PathPlanComply::findNearestIndexOnPath(std::vector<XYZ_COOR_S> &path, double x, double y)
{
    double min_distance = 1e6;
    int min_index = 0;
    for (int i = 10; i < path.size(); i++)
    {
        auto point = path.at(i);
        double dx = point.x_axis - x;
        double dy = point.y_axis - y;
        double distance = hypot(dx, dy);
        if (distance < min_distance)
        {
            min_distance = distance;
            min_index = i;
        }
    }
    return min_index;
}

void PathPlanComply::GenerateHookPath(robot::hook_position hook_pos)
{
    printf("生成hook path,hook_dist:%f pos: %f, %f\n",
           hook_pos.center_distance, hook_pos.center_point_x, hook_pos.center_point_y);
    std::vector<std::array<float, 3>> hook_path;
    XYZ_COOR_S xyz_temp;

    if (hook_pos.center_distance > 10.0)
        return;
    if (hook_pos.center_distance < 2.0)
        return;

    double x0 = mNavData.xAxis;
    double y0 = mNavData.yAxis;
    double h0 = mNavData.heading;
    double x1 = hook_pos.center_point_x;
    double y1 = hook_pos.center_point_y;
    double b1 = hook_pos.beta;
    double d1 = hook_pos.center_distance;

    mDrivingPath.clear();

    hook_path = adap_hook_c.GenerateAdaptiveHookPath(x0, y0, h0, x1, y1, b1, d1, PalletType);
    // printf("---------------\n");
    for (auto temp : hook_path)
    {
        xyz_temp.x_axis = temp[0];
        xyz_temp.y_axis = temp[1];
        xyz_temp.heading = temp[2];
        printf("hook path: %f %f %f\n", xyz_temp.x_axis, xyz_temp.y_axis, xyz_temp.heading);
        mDrivingPath.push_back(xyz_temp);
    }
    mStopIndex = mDrivingPath.size() - 1;
    // z终点沿伸1.25m，用于精准停靠
    for (int i = 1; i < 300; ++i)
    {
        x0 = mDrivingPath.back().x_axis;
        y0 = mDrivingPath.back().y_axis;
        h0 = mDrivingPath.back().heading;

        xyz_temp = pubalgor.CalculNextPointByAngle_P(x0, y0, h0, 0.05);

        mDrivingPath.emplace_back(xyz_temp);

        if (25 == i && hook_pos.center_distance > 2.0)
        {
            mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
            mPathPlanStatus.stopX = xyz_temp.x_axis;
            mPathPlanStatus.stopY = xyz_temp.y_axis;
            mPathPlanStatus.stopAngle = xyz_temp.heading;
        }
    }
    // 计算path
    calcuPath(mDrivingPath);
    for (auto &pt : mDrivingPath)
    {
        pt.heading += 180.0;
        if (pt.heading > 360.0)
            pt.heading -= 360.0;
    }

    mKeypoint = 0;
}

void PathPlanComply::GenerateParkPath(robot::palletpos park_pos, int withdraw)
{
    double x0 = mNavData.xAxis;
    double y0 = mNavData.yAxis;
    double h0 = mNavData.heading;
    double x1 = park_pos.xg;
    double y1 = park_pos.yg;
    double h1 = park_pos.heading;

    while (h0 > 360.0)
        h0 -= 360.0;
    while (h0 < 0.0)
        h0 += 360.0;
    while (h1 > 360.0)
        h1 -= 360.0;
    while (h1 < 0.0)
        h1 += 360.0;

    float distance2loadpos = pubalgor.CalculatePoint2PointDistance_P(
        x0, y0, 0, x1, y1, 0);

    if (distance2loadpos > 30.0)
    {
        ROS_INFO("Parking position is so far");
        return;
    }

    int cur_num = pubalgor.FindKeyPointByTargetPoint_P(mDrivingPath, x0, y0);
    int tar_num = pubalgor.FindKeyPointByTargetPoint_P(mDrivingPath, x1, y1);

    XYZ_COOR_S xyz_temp;
    std::vector<XYZ_COOR_S> park_path;

    for (int i = cur_num; i >= tar_num + withdraw; i--)
        park_path.emplace_back(mDrivingPath[i]);

    // generate connect path
    double insert_h = h1;
    std::vector<XYZ_COOR_S> connect_path;

    xyz_temp.x_axis = x1;
    xyz_temp.y_axis = y1;
    xyz_temp.heading = h1;

    connect_path.emplace_back(xyz_temp);

    for (int i = 0; i < 15; ++i)
    {
        xyz_temp = pubalgor.CalculNextPointByAngle_P(
            connect_path.back().x_axis, connect_path.back().y_axis, insert_h, 0.2);

        connect_path.emplace_back(xyz_temp);
    }

    for (int i = connect_path.size() - 1; i >= 0; i--)
    {
        park_path.emplace_back(connect_path[i]);
    }

    // sample
    auto bak = park_path;
    park_path.clear();
    for (int i = 0; i < bak.size() - 1; i += 6)
        park_path.emplace_back(bak[i]);
    park_path.emplace_back(bak.back());

    CSpline spline;
    spline.SplinePointSet(park_path, mDrivingPath, 0.1);

    // extend path
    h1 = h1 - 180.0;
    while (h1 > 360.0)
        h1 -= 360.0;
    while (h1 < 0.0)
        h1 += 360.0;

    mDrivingPath.back().heading = h1;
    for (int i = 1; i < 100; ++i)
    {
        xyz_temp = pubalgor.CalculNextPointByAngle_P(
            mDrivingPath.back().x_axis, mDrivingPath.back().y_axis,
            mDrivingPath.back().heading, 0.1);

        mDrivingPath.emplace_back(xyz_temp);
    }
    calcuPath(mDrivingPath);
    mStopIndex = findNearestIndexOnPath(mDrivingPath, x1, y1);
    mKeypoint = 0;
}

void PathPlanComply::GenerateLoadPath(robot::task_plan_msg task_plan_t)
{
    mLoadPos.xg = -33.96;
    mLoadPos.yg = -24.92;
    mLoadPos.heading = 275.7;

    double x0 = mNavData.xAxis;
    double y0 = mNavData.yAxis;
    double x1 = mLoadPos.xg;
    double y1 = mLoadPos.yg;

    float distance2loadpos = pubalgor.CalculatePoint2PointDistance_P(
        x0, y0, 0, x1, y1, 0);

    if (distance2loadpos > 30.0)
    {
        ROS_INFO("Load position is so far");
        return;
    }

    std::vector<XYZ_COOR_S> front_segment, mid_segment, rear_segment;
    XYZ_COOR_S xyz_temp, xyz_temp1, end_point;
    float angle_temp = mLoadPos.heading - 90.0;

    if (angle_temp < 0.0)
        angle_temp += 360.0;

    xyz_temp = pubalgor.CalculNextPointByAngle_P(
        mLoadPos.xg, mLoadPos.yg, angle_temp, 2.3);

    angle_temp -= 90.0;

    if (angle_temp < 0.0)
        angle_temp += 360.0;

    xyz_temp = pubalgor.CalculNextPointByAngle_P(
        xyz_temp.x_axis, xyz_temp.y_axis, angle_temp, 10.0);

    // extend path
    for (int i = 1; i < 100; ++i)
    {
        xyz_temp1 = pubalgor.CalculNextPointByAngle_P(
            xyz_temp.x_axis, xyz_temp.y_axis, mLoadPos.heading, 0.1);

        mid_segment.emplace_back(xyz_temp1);

        xyz_temp = xyz_temp1;

        if (90 == i)
        {
            mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
            mPathPlanStatus.stopX = xyz_temp1.x_axis;
            mPathPlanStatus.stopY = xyz_temp1.y_axis;
            mPathPlanStatus.stopAngle = xyz_temp1.heading;
        }
    }

    front_segment = pubalgor.GenerateBezierPath_P(
        mNavData.xAxis, mNavData.yAxis, mNavData.heading,
        mid_segment[0].x_axis, mid_segment[0].y_axis, mid_segment[0].heading);

    std::vector<XYZ_COOR_S> path_list;

    for (auto pathName : task_plan_t.pathList)
    {
        path_list = LoadPathFile(pathName);
        end_point = path_list.at(0);
    }

    rear_segment = pubalgor.GenerateBezierPath_P(
        mid_segment.back().x_axis, mid_segment.back().y_axis, mid_segment.back().heading,
        end_point.x_axis, end_point.y_axis, end_point.heading);

    mDrivingPath.clear();

    for (auto pt : front_segment)
        mDrivingPath.emplace_back(pt);
    for (auto pt : mid_segment)
        mDrivingPath.emplace_back(pt);
    for (auto pt : rear_segment)
        mDrivingPath.emplace_back(pt);
    for (auto pt : path_list)
        mDrivingPath.emplace_back(pt);

    // extend path
    for (int i = 1; i < 100; ++i)
    {
        xyz_temp = pubalgor.CalculNextPointByAngle_P(
            mDrivingPath.back().x_axis, mDrivingPath.back().y_axis, mDrivingPath.back().heading, 0.05);

        mDrivingPath.emplace_back(xyz_temp);
    }
    calcuPath(mDrivingPath);
    mStopIndex = findNearestIndexOnPath(mDrivingPath, x1, y1);
    mKeypoint = 0;
}

void PathPlanComply::UpdateStopPoint(robot::task_plan_msg task_plan_t)
{
    std::vector<XYZ_COOR_S> path_list;

    for (auto pathName : task_plan_t.pathList)
        path_list = LoadPathFile(pathName);

    size_t size = path_list.size();

    mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
    mPathPlanStatus.stopX = path_list.at(size - 1).x_axis;
    mPathPlanStatus.stopY = path_list.at(size - 1).y_axis;
    mPathPlanStatus.stopAngle = path_list.at(size - 1).heading;
}

void PathPlanComply::SetTaskPlanData(robot::task_plan_msg task_plan_t)
{
    printf("========================设置任务: %d, id:%d===========================\n",
           int(task_plan_t.taskType), int(task_plan_t.task_id));
    printf("current task execustatus: %d, workmode: %d, tasktype: %d\n",
           mPathPlanStatus.taskExecuStatus, workMode, mTaskPlanData.taskType);
    bool is_take_new_task = true;
    if (int(mPathPlanStatus.taskExecuStatus) == 1)
    {
        if (!is_take_new_task)
        {
            printf("任务正在执行中,请勿设置新任务\n");
            return;
        }
        else
        {
            printf("舍弃当前未完成的任务，执行新任务\n");
        }
    }
    this->loadAirCraftPorts(); //test
    // if (JudgeTaskPlanMsgChanged(mTaskPlanData, task_plan_t))
    {
        printf("任务数据有更新, 重置关键点\n");
        sysTime.taskStart = ros::Time::now().toSec();
        mKeypoint = 0;
        mStopIndex = 0;
        mTaskPlanData = task_plan_t;
        workMode = task_plan_t.workMode;
        mPathPlanStatus.stopX = task_plan_t.stopX;
        mPathPlanStatus.stopY = task_plan_t.stopY;
        mPathPlanStatus.stopAngle = task_plan_t.stopAngle;
        InitSafetyCheck = 0;
        mDrivingPath.clear();
        mReferPath.desireSpeed = 0.0;
        mReferPath.x.clear();
        mReferPath.y.clear();
        if (task_plan_t.taskType == NOTHING)
        {
            mPathPlanStatus.taskExecuStatus = TASKFINISHED;
        }
        else
        {
            mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
        }

        printf("stopx:%f, stopy:%f, desired_speed:%f\n",
               mPathPlanStatus.stopX, mPathPlanStatus.stopY, mTaskPlanData.desireSpeed);
        calcuGlobalPath(task_plan_t);
        if (task_plan_t.task_id == go_task_id_) // 99去程,100回程
        {
            updateAirPortStopIndex(mDrivingPath, true);
        }
        if (task_plan_t.task_id == back_task_id_) // 99去程,100回程
        {
            updateAirPortStopIndex(mDrivingPath, false);
        }
    }
    // else
    // {
    //     printf("任务未改变\n");
    // }
    // handleDrivingPath();
}

void PathPlanComply::resetTask()
{
    mPathPlanStatus.taskExecuStatus = NOTASK;
    mKeypoint = 0;
    mStopIndex = 0;
    InitSafetyCheck = 0;
    mDrivingPath.clear();
    mReferPath.desireSpeed = 0.0;
    mReferPath.planspeed = 0.0;
    mReferPath.x.clear();
    mReferPath.y.clear();
}

void PathPlanComply::handleDrivingPath()
{
    if (mTaskPlanData.taskType == ADAPTIVEUNLOADPATH)
    {
        if (mTaskPlanData.taskType != ADAPTIVEUNLOADPATH)
        {
            mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
            UpdateStopPoint(mTaskPlanData);
        }
    }

    if (mTaskPlanData.taskType == ADAPTIVELOADPATH)
    {
        if (mTaskPlanData.taskType != ADAPTIVELOADPATH)
        {
            mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
            GenerateLoadPath(mTaskPlanData);
            ROS_INFO("Generate Load Path %d Points", (int)mDrivingPath.size());
        }
    }

    if (mTaskPlanData.taskType == ADAPTIVEHOOK)
    {
        if (mTaskPlanData.taskType != ADAPTIVEHOOK)
            mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
        GenerateHookPath(mHookPos);
        ROS_INFO("Generate Hook Path %d Points", (int)mDrivingPath.size());
    }
    if (mTaskPlanData.taskType == DOACTION)
    {
        // no need path do nonthing
        // mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
    }

    if (mTaskPlanData.taskType == ADAPTIVEPARK)
    {
        if (mTaskPlanData.taskType != ADAPTIVEPARK)
        {
            mParkPos.xg = mTaskPlanData.stopX;
            mParkPos.yg = mTaskPlanData.stopY;
            mParkPos.heading = mTaskPlanData.stopAngle;
            mPathPlanStatus.stopX = mParkPos.xg;
            mPathPlanStatus.stopY = mParkPos.yg;
            mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;

            GenerateParkPath(mParkPos, 30);

            ROS_INFO("Generate Park Path %d Points", (int)mDrivingPath.size());
        }
    }

    // if (JudgeTaskPlanMsgChanged(mTaskPlanData, mTaskPlanData))
    // {
    //     mTaskPlanData = mTaskPlanData;
    //     calcuGlobalPath(mTaskPlanData);

    //     mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
    //     mPathPlanStatus.stopX = mTaskPlanData.stopX;
    //     mPathPlanStatus.stopY = mTaskPlanData.stopY;
    //     mPathPlanStatus.stopAngle = mTaskPlanData.stopAngle;
    //     return;
    // }
    // if (JudgeShiftParkingPoint(mTaskPlanData, mTaskPlanData))
    // {
    //     mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
    //     mPathPlanStatus.stopX = mTaskPlanData.stopX;
    //     mPathPlanStatus.stopY = mTaskPlanData.stopY;
    //     mPathPlanStatus.stopAngle = mTaskPlanData.stopAngle;
    // }
}

void PathPlanComply::updateAirPortStopIndex(std::vector<XYZ_COOR_S> &global_path, bool is_go_path)
{
    for (int i = 0; i < aircraft_parking_ports_.size(); i++)
    {
        auto &airport = aircraft_parking_ports_[i];
        Point stoppoint = is_go_path ? airport.go_stop_point : airport.back_stop_point;
        double min_distance = 1e9;
        airport.go_stop_index = 0;
        airport.back_stop_index = 0;
        for (int j = 0; j < global_path.size(); j++)
        {
            auto gpoint = global_path[j];
            double dist1 = pow((gpoint.x_axis - stoppoint.x), 2) + pow((gpoint.y_axis - stoppoint.y), 2);
            if (dist1 < min_distance)
            {
                min_distance = dist1;
                if (is_go_path)
                {
                    airport.go_stop_index = j;
                }
                else
                {
                    airport.back_stop_index = j;
                }
            }
        }
    }
}

void PathPlanComply::updateAirPortInfo(robot::AirCraftParkingPort airport)
{
    for (int i = 0; i < aircraft_parking_ports_.size(); i++)
    {
        auto &port = aircraft_parking_ports_[i];
        if (port.id == airport.port_id)
        {
            port.parking = airport.status;
            return;
        }
    }
    ROS_ERROR("not found the airport:%d\n", int(airport.port_id));
}

AirCraftParkingPort PathPlanComply::findNextAirCraftParkingPort()
{
    AirCraftParkingPort next_port = aircraft_parking_ports_.front();
    int nearest_stop_index = mDrivingPath.size() - 1;
    for (int i = 0; i < aircraft_parking_ports_.size(); i++)
    {
        auto port = aircraft_parking_ports_[i];
        printf("mkeypoint:%d, port id:%d, go_stop_index:%d, back_stop_index:%d, nearest_stop_index:%d\n", mKeypoint,
               port.id, port.go_stop_index, port.back_stop_index, nearest_stop_index);
        if (port.go_stop_index > mKeypoint ||
            abs(port.go_stop_index - mKeypoint) < 5)
        {
            if (port.go_stop_index < nearest_stop_index)
            {
                nearest_stop_index = port.go_stop_index;
                next_port = port;
            }
        }
        if (port.back_stop_index > mKeypoint)
        {
            if (port.back_stop_index < nearest_stop_index ||
                abs(port.back_stop_index - mKeypoint) < 5)
            {
                nearest_stop_index = port.back_stop_index;
                next_port = port;
            }
        }
    }
    return next_port;
}

void PathPlanComply::SetCanData(robot::can_msg can_msg_t)
{
    mVehicleData = can_msg_t;
    if (mVehicleData.emergencyStop)
        emergencyStop = 1;
    mPathPlanStatus.curSpeed = mVehicleData.vehicleSpeed;
}

void PathPlanComply::SetPerceptionData(robot::perception perception_t)
{
    sysTime.msgPerception = ros::Time::now().toSec();

    mPerception.objs.clear();

    for (auto i : perception_t.objs)
    {
        Point me;
        Point point;
        me.x = mNavData.xAxis;
        me.y = mNavData.yAxis;
        me.heading = mNavData.heading;
        point.x = i.x;
        point.y = i.y;

        double dh = PointDirectionToMe(point, me);
        double hCheck = 0;

        if (fabs(dh) > 90.0)
            hCheck = 1;
        if (hCheck == 0)
        {
            if (i.dx < 0.05 || i.dy < 0.05)
                continue;
            mPerception.objs.push_back(i);
        }
        // mPerception.objs.push_back(i);
    }

    for (auto i : mPerceptionFrontScan.objs)
        mPerception.objs.push_back(i);

    // collistion check precise
    Point VehicleInfo;
    VehicleInfo.x = mNavData.xAxis;
    VehicleInfo.y = mNavData.yAxis;
    VehicleInfo.heading = mNavData.heading;
    VehicleInfo.length = 3.6;
    VehicleInfo.width = 2.0;

    double dist = 10000.0; // 距离车辆最近障碍物距离
    double sdh = 100.0;    // 距离车辆最近障碍物朝向

    for (auto i : mPerception.objs)
    {
        Point point;
        point.x = i.x;
        point.y = i.y;

        double dertah = PointDirectionToMe(point, VehicleInfo);
        double dx = VehicleInfo.x - point.x;
        double dy = VehicleInfo.y - point.y;
        double ds = hypot(dx, dy) - VehicleInfo.length / 2.0;

        if (ds < dist)
        {
            dist = ds;
            sdh = fabs(dertah) * M_PI / 180.0;
        }
    }
    // 下面根据障碍物距离和障碍物朝向判断安全，下面的desiredSpeed没有用到
    double safety = 0;
    double desireSpeed = mNavData.gpsSpeed + 1.0;
    double width = dist * sin(sdh);

    if (dist < 20.0 && width < 1.5)
        desireSpeed = std::min<double>(desireSpeed, 4.0 * dist / 20.0);
    if (dist < 5.0 && width < 1.5)
        safety = 1;

    // high frequncy safaty check filter
    static int safety_check_counter = 0;

    if (safety == 1)
    {
        safety_check_counter += 1;

        if (safety_check_counter < 3)
            mReferPath.safety = 0;
        else
            safety_check_counter = 10;
    }
    else
        safety_check_counter = 0;

    if (safety == 1)
        desireSpeed = 0.0;

    // 这里的brake命令没有在代码中用到
    int BrakeCmd = (int)((mNavData.gpsSpeed - desireSpeed) * 30.0);

    if (BrakeCmd < 0)
        BrakeCmd = 0;
    if (desireSpeed == 0.0)
        BrakeCmd = 85;

    ros::param::set("/canbus/brake", BrakeCmd);
}

void PathPlanComply::SetFrontScanData(jsk_recognition_msgs::BoundingBoxArray msg)
{
    mPerceptionFrontScan.objs.clear();

    if (!msg.boxes.size())
        return;

    double x = mNavData.xAxis;
    double y = mNavData.yAxis;
    double h = 90.0 - mNavData.heading;

    if (h > 360.0)
        h -= 360.0;
    if (h < 0.0)
        h += 360.0;

    double theta = h * M_PI / 180.0;

    for (auto i : msg.boxes)
    {
        robot::object obj;
        obj.x = x - 1.5 + i.pose.position.x * cos(theta) - i.pose.position.y * sin(theta);
        obj.y = y + i.pose.position.x * sin(theta) + i.pose.position.y * cos(theta);
        obj.dx = 0.5;
        obj.dy = 0.5;
        mPerceptionFrontScan.objs.push_back(obj);
    }
}

void PathPlanComply::SetPerceptionData2(robot::perception perception_t)
{
    sysTime.msgPerception = ros::Time::now().toSec();
    // 侧向雷达
    std::vector<ObstaclePtr> new_obstacles;
    for (auto i : perception_t.objs)
    {
    if (fabs(i.dx) < 0.01 || fabs(i.dy) < 0.01)
            continue;
        // mPerception.objs.push_back(i);
        ObstaclePtr obstacle = std::make_shared<Obstacle>();
        obstacle->x = i.x;
        obstacle->y = i.y;
        obstacle->width = i.dx;
        obstacle->length = i.dy;
        obstacle->heading = gis_utils::azimuthToYaw(i.heading-90);
        new_obstacles.push_back(obstacle);
    }
    // 前向雷达
    for (auto i : mPerceptionFrontScan.objs)
    {
    if (fabs(i.dx) < 0.01 || fabs(i.dy) < 0.01)
            continue;
        // mPerception.objs.push_back(i);
        ObstaclePtr obstacle = std::make_shared<Obstacle>();
        obstacle->x = i.x;
        obstacle->y = i.y;
        obstacle->width = i.dx;
        obstacle->length = i.dy;
        obstacle->heading = gis_utils::azimuthToYaw(i.heading-90);
        new_obstacles.push_back(obstacle);
    }
    // 比较old and new obstacles
    // new find old
    for (int i = 0; i < new_obstacles.size(); i++)
    {
        auto o = new_obstacles[i];
        auto find_it = findObstacle(obstacles_, o->x, o->y);
        if (find_it)
        {
            find_it->history.erase(find_it->history.begin());
            find_it->x = o->x;
            find_it->y = o->y;
            find_it->length = o->length;
            find_it->width = o->width;
            find_it->heading = o->heading;
            find_it->history.push_back(1);
        }
        else
        {
            obstacles_.push_back(o);
        }
    }
    // old find new
    for (int i = 0; i < obstacles_.size(); i++)
    {
        auto o = obstacles_[i];
        auto find_it = findObstacle(new_obstacles, o->x, o->y);
        if (!find_it)
        {
            o->history.erase(o->history.begin());
            o->history.push_back(0);
        }
    }
    // 清理消失的障碍物
    for (auto it = obstacles_.begin(); it != obstacles_.end();)
    {
        auto o = *it;
        
         bool erase = o->historySum() < 1;
        if (erase)
        {
            it = obstacles_.erase(it);
        }
        else
        {
            it++;
        }
    }
    // 输出障碍物列表
    // for (int i = 0; i < obstacles_.size(); i++)
    // {
    //     auto o = obstacles_[i];
    //     printf("obstacle %d: (%.2f, %.2f), (%.2f, %.2f,%.2f), (%d,%d,%d,%d)\n", i, o->x, o->y, o->length, o->width, o->heading,
    //            o->history[0], o->history[1], o->history[2], o->history[3]);
    // }
}

void PathPlanComply::SetBackScanData(jsk_recognition_msgs::BoundingBoxArray msg)
{
    mPerceptionBackScan.objs.clear();

    if (!msg.boxes.size())
        return;

    double x = mNavData.xAxis;
    double y = mNavData.yAxis;
    double h = 90.0 - mNavData.heading;

    if (h > 360.0)
        h -= 360.0;
    if (h < 0.0)
        h += 360.0;

    double theta = h * M_PI / 180.0;

    for (auto i : msg.boxes)
    {
        robot::object obj;
        obj.x = x - 0.9 - i.pose.position.x * cos(theta) + i.pose.position.y * sin(theta);
        obj.y = y - i.pose.position.x * sin(theta) - i.pose.position.y * cos(theta);
        obj.dx = 0.5;
        obj.dy = 0.5;
        mPerceptionBackScan.objs.push_back(obj);
    }
}

void PathPlanComply::SetPalletCoorData(robot::hook_position hook_pos_t)
{
    mHookPos = hook_pos_t;
    // mHookPos.center_point_y += 0.1;
}

void PathPlanComply::SetLoadPosition(robot::palletpos load_pos_t)
{
    mLoadPos = load_pos_t;
}

void PathPlanComply::SetNavigationData(robot::navigation_msg navigation_msg_t)
{
    mNavData = navigation_msg_t;
    mPathPlanStatus.curXAxis = mNavData.xAxis;
    mPathPlanStatus.curYAxis = mNavData.yAxis;

    mPathPlanStatus.curHead = mNavData.heading;
    //mPathPlanStatus.curSpeed = mNavData.gpsSpeed;
    mPathPlanStatus.rtkState = mNavData.rtkState;
    // 计算车辆的包围盒 车长3.45m，后轴中心到前雷达:2.41   车宽1.47m,托盘宽3.4m
    //  double l = 4.82;
    //  double w = 3.4;
}

void PathPlanComply::SetButtonData(robot::rs232 button_data_t)
{
    if (mVehicleData.emergencyStop)
        return;
    if (button_data_t.data[3])
        emergencyStop = 0;
}

double PathPlanComply::Distance2PerceptionObjs(double x, double y)
{
    double dist = 10000.0;

    for (auto i : mPerception.objs)
    {
        double dx = fabs(x - i.x);
        double dy = fabs(y - i.y);
        double len = std::min<double>(i.dx / 2.0, i.dy / 2.0);
        double ds = hypot(dx, dy) - len - 0.5;
        double dl = dx - i.dx / 2.0;
        double dw = dy - i.dy / 2.0;

        if (dist > ds)
            dist = ds;
    }

    return dist;
}

double PathPlanComply::Distance2PerceptionObjs(GeoLine line)
{
    double dist = 10000.0;

    for (auto i : mPerception.objs)
    {
        GeoPoint point(i.x, i.y);
        double len = std::min<double>(fabs(i.dx / 2.0), fabs(i.dy / 2.0));
        double d = bg::distance(point, line) - len - 0.15;

        // double dx = fabs(x - i.x);
        // double dy = fabs(y - i.y);
        // double len = std::min<double>(i.dx / 2.0, i.dy / 2.0);
        // double ds = hypot(dx, dy) - len - 0.5;
        // double dl = dx - i.dx / 2.0;
        // double dw = dy - i.dy / 2.0;

        // if (dist > ds)
        //     dist = ds;
        if (d < dist)
        {
            dist = d;
        }
    }

    return dist;
}

void PathPlanComply::PathPlanProcess()
{
    printf("=====================================================\n");
    printf("任务状态: task execustatus: %d, tasktype: %d\n",
           mPathPlanStatus.taskExecuStatus, mTaskPlanData.taskType);
    // 1. 生成路径
    // printf("handle driving path\n");
    handleDrivingPath();
    // if (mDrivingPath.empty())
    // {
    //     printf("driving path is emppty\n");
    //     mKeypoint = 0;
    //     return;
    // }
    // 2.  更新最近点mKeyPoint索引

    mDesireSpeed = mTaskPlanData.desireSpeed;

    double x0 = mNavData.xAxis;
    double y0 = mNavData.yAxis;
    double h0 = mNavData.heading;
    double xt = mPathPlanStatus.stopX;
    double yt = mPathPlanStatus.stopY;
    double ht = mPathPlanStatus.stopAngle;
    if (!mDrivingPath.empty())
    {
        printf("update path info, path size: %d\n", mDrivingPath.size());
        UpdatePathInfo();
        if (mVehicleData.curGear == GEAR_D)
        {
            XYZ_COOR_S xyz = mDrivingPath.at(mKeypoint);
            x0 = xyz.x_axis;
            y0 = xyz.y_axis;
        }
    }
    // 根据与停止点的距离计算一个速度
    printf("计算limit速度, 更新完成状态\n");
    double speed = LimitSpeedByDistanceToStop(x0, y0, h0, xt, yt, ht);
    ////////////////计算一个限速/////////////////
    // 与任务要求速度进行比较
    printf("speed limit:%f, desire speed:%f\n", speed, mDesireSpeed);
    mDesireSpeed = std::min<double>(speed, mDesireSpeed);
    if (mVehicleData.curGear == GEAR_R) // 倒挡速度要求不大于0.5
        mDesireSpeed = std::min<double>(mDesireSpeed, 0.5);
    if (mVehicleData.curGear == GEAR_N)
    {
        printf("挡位: N, desire speed: 0\n");
        mDesireSpeed = 0.0;
    }

    // if (mPathPlanStatus.stopX == -1 && mPathPlanStatus.stopY == -1)
    // {
    //     printf("无停止点, desire speed: 0\n");
    //     mDesireSpeed = 0.0;
    // }

    if (mPathPlanStatus.taskExecuStatus == TASKFINISHED)
    {
        mDesireSpeed = 0.0;
        mKeypoint = 0;
        mStopIndex = 0;
        mTaskPlanData.pathList.clear();
        mTaskPlanData.taskType = NOTASK;
        // mPathPlanStatus.taskExecuStatus = NOTASK;
    }

    /////////////////////////////////////////////
    // 运行过程中挂钩异常，直接结束任务
    int hookstate = 0;
    ros::param::get("/canbus/hookstate", hookstate);
    // if (hookstate == 1 && mPathPlanStatus.curSpeed < 0.3)
    // {
    //     printf("hookstate == 1 and curspeed < 0.3 task finifshed\n");
    //     mDesireSpeed = 0.0;
    //     mPathPlanStatus.taskExecuStatus = TASKFINISHED;
    // }
}

void PathPlanComply::PublishPathPlanStatus(ros::Publisher &tPub)
{
    // if (mTaskPlanData.workMode != NOMALWORKINGMODE)
    //     mPathPlanStatus.taskExecuStatus = NOTASK;
    tPub.publish(mPathPlanStatus);
}

void PathPlanComply::PublishReferPath(ros::Publisher &tPub)
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> heading;
    std::vector<XYZ_COOR_S> extend_path;
    std::vector<XYZ_COOR_S> xyz_list;
    //////////////////////
     if (mTaskPlanData.task_id == 4) // 测试分段限速
    {
        mPerception.objs.clear();
        mPerceptionFrontScan.objs.clear();
        mPerceptionBackScan.objs.clear();
    }
    /////////////////////
    if (mPathPlanStatus.taskExecuStatus == TASKFINISHED)
    {
        printf("任务结束\n");
        mReferPath.x.clear();
        mReferPath.y.clear();
        mReferPath.desireSpeed = 0.0;
        mReferPath.planspeed = 0.0;
        return;
    }

    int size = mDrivingPath.size(); // 全局路径，长度固定
    if (size < 20)
        return;

    XYZ_COOR_S xyz_last = mDrivingPath.at(mKeypoint);
    x.push_back(xyz_last.x_axis);
    y.push_back(xyz_last.y_axis);
    heading.push_back(xyz_last.heading);

    int LaneChangeSwitch = xyz_last.z_axis;

    // load path section from refernce path
    for (int i = mKeypoint + 1; i < size - 2; i += 2)
    {
        XYZ_COOR_S xyz_temp = mDrivingPath.at(i);

        double dx = xyz_temp.x_axis - xyz_last.x_axis;
        double dy = xyz_temp.y_axis - xyz_last.y_axis;
        double dist = hypot(dx, dy);

        if (dist < 0.3)
            continue;

        x.push_back(xyz_temp.x_axis);
        y.push_back(xyz_temp.y_axis);
        heading.push_back(xyz_temp.heading);

        xyz_last = xyz_temp;

        if (x.size() >= 20)
            break;
    }

    // generate extended path
    // 这里添加延长路径是为了精准停靠
    if (x.size() < 20)
    {
        XYZ_COOR_S xyz_temp = mDrivingPath.at(size - 2);
        // printf("last point: x:%f, y:%f, heading:%f\n", xyz_temp.x_axis, xyz_temp.y_axis, xyz_temp.heading);
        float head_extend = xyz_temp.heading;
         printf("xyz heading:%f\n", head_extend);
        if (mVehicleData.curGear == GEAR_R)
        {
            head_extend += 180.0;

            if (head_extend > 360.0)
                head_extend -= 360.0;
            // if (head_extend > 0.0)
            //     head_extend += 360.0;
        }
        printf("head extend is:%f\n", head_extend);
        extend_path = pubalgor.GenerateExtendedPath(
            xyz_temp.x_axis, xyz_temp.y_axis, head_extend, 30);

        size = extend_path.size();

        for (int i = 0; i < size; i += 6)
        {
            xyz_temp = extend_path.at(i);
            x.push_back(xyz_temp.x_axis);
            y.push_back(xyz_temp.y_axis);
            heading.push_back(head_extend);

            if (x.size() >= 20)
                break;
        }
    }

    // gear R
    if (mVehicleData.curGear == GEAR_R)
    {
        mReferPath.x = x;
        mReferPath.y = y;
        mReferPath.Path_Id = 20;
        mReferPath.safety = 0;
        mReferPath.planspeed = 1.0;
        mReferPath.desireSpeed = mDesireSpeed;

        double backdist = 100000.0;
        for (auto i : mPerceptionBackScan.objs)
        {
            double backx = mNavData.xAxis;
            double backy = mNavData.yAxis;
            double d = hypot(i.x - backx, i.y - backy);

            if (d < backdist)
                backdist = d;
        }
printf("R挡检测: objs数量=%d, backdist=%.3f\n", (int)mPerceptionBackScan.objs.size(), backdist);

        if (backdist < 2.5)
        {
            printf("R挡, 障碍物距离<0.2, 停车，障碍物距离: %f\n", backdist);
            mReferPath.safety = 1;
            mReferPath.planspeed = 0;
            mReferPath.desireSpeed = 0;
            ROS_ERROR("STOP");
        }

        tPub.publish(mReferPath);
        return;
    }

    // gear D
    lidarobjs_global_.clear();
    lidarobjs_global_ = Obj_Projecte_Map(mPerception, lidarobjs_global_);

    // path plan with lattice planner
    // printf("----plan lattice start\n");
    // path_final = LatticePlan(x, y, heading);
    // printf("----lattice planner size:%d\n", path_final.size());
    // if (path_final.empty())
    // {
    //     // mReferPath.x.clear();
    //     // mReferPath.y.clear();
    //     mReferPath.safety = 1;
    //     mReferPath.planspeed = 0;
    //     mReferPath.desireSpeed = 0;
    //     return;
    // }
    // path_final.erase(path_final.begin());

    // check last trajectory collision
    int collision_check = 0;
    double x0 = 0.0;
    double y0 = 0.0;
    double h0 = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
    double h1 = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    double dh = 0.0;
    double dist = 0.0;
    double path_dist = 0.0;

    size = path_final_last.size();

    if (size > 20 && InitSafetyCheck == 1)
    {
        for (auto i : mPerception.objs)
        {
            path_dist = 100.0;
            // 计算障碍物与路径的最近距离
            for (auto j : path_final_last)
            {
                dx = i.x - j.x;
                dy = i.y - j.y;
                dist = hypot(dx, dy);

                if (dist < path_dist)
                    path_dist = dist;
            }

            dx = mNavData.xAxis - i.x;
            dy = mNavData.yAxis - i.y;
            dist = hypot(dx, dy);
            // 这里如果距离最近障碍物小于1m并且距离车base小于5m则认为有碰撞风险 车长3.45m，后轴中心到前雷达:2.41   车宽1.47m
            // if (path_dist < 1.0 && dist < 5.0)
            //     collision_check = 1;
            if (path_dist < 2.75)
                collision_check = 1;
        }

        dx = mNavData.xAxis - path_final_last[size - 1].x;
        dy = mNavData.yAxis - path_final_last[size - 1].y;
        dist = hypot(dx, dy); // 自车与终点的距离

        if (collision_check == 0 && dist > 10.0) // 如果上一帧路径无碰撞风险，且距离>10m则继续使用上一帧路径,并从当前位置裁剪
        {
            path_final = path_final_last;

            // get path cut from cur pose
            auto pathcut = path_final;
            path_dist = 10000.0;

            for (int i = 0; i < path_final.size(); i++)
            {
                dx = mNavData.xAxis - path_final[i].x;
                dy = mNavData.yAxis - path_final[i].y;
                dist = hypot(dx, dy);

                if (dist < path_dist)
                {
                    path_dist = dist;
                    size = i;
                }
            }

            pathcut.clear();
            for (int i = size; i < path_final.size(); i++)
            {
                pathcut.push_back(path_final[i]);
            }

            path_final = pathcut;
        }
    }

    path_final_last = path_final;

    // speed plan
    // 这里只进行了参数传递，没有进行计算
    //Speed_Planning speed_planning(&path_final, mPathPlanStatus.curSpeed, lidarobjs_global_);

    // float planspeed = speed_planning.SpeedCalculateInGuidePth(&path_final);
    double planspeed = 8.0;

    dist = Distance2PerceptionObjs(mNavData.xAxis, mNavData.yAxis);
    dist = std::max<double>(1.0, dist / 2.0);
    if (dist < 3 && mVehicleData.vehicleSpeed > 3.0)
    {
        planspeed = std::min<double>(planspeed, dist);
    }

    std::vector<float> x_final;
    std::vector<float> y_final;

    for (auto i : path_final)
    {
        x_final.push_back(i.x);
        y_final.push_back(i.y);
    }

    mReferPath.x = x_final;
    mReferPath.y = y_final;
    mReferPath.Path_Id = 5;
    mReferPath.safety = 0;
    mReferPath.planspeed = planspeed;
    mReferPath.desireSpeed = std::min<double>(planspeed, mDesireSpeed);
    printf("=======task_id:%d\n", mTaskPlanData.task_id);
    if(mTaskPlanData.task_id == 4) // 测试分段限速
    {
        //double cur_lane_speed = mReferPath.desireSpeed;
        //double next_lane_speed = std::max(1.0,  mReferPath.desireSpeed - 1.5);
        //printf("cur_lane_speed = %f, next_lane_speed = %f\n", cur_lane_speed, next_lane_speed);
        double limit_speed = getLaneLimitSpeedTest(3.0, 1.5);
        mReferPath.desireSpeed = std::min(double(mReferPath.desireSpeed), limit_speed);
    }
    // printf("ref path id:%d, lane change switch:%d\n", mReferPath.Path_Id, LaneChangeSwitch);
    // LaneChangeSwitch = 1; // 绕障
    if (mReferPath.Path_Id == 5 || LaneChangeSwitch == 0)
    {
        mReferPath.x = x;
        mReferPath.y = y;
    }

    if (mPerception.objs.size() == 0)
    {
        tPub.publish(mReferPath);
        return;
    }
    // 下面进行碰撞检测
    //  collistion check precise
    Point VehicleInfo;
    VehicleInfo.x = mNavData.xAxis;
    VehicleInfo.y = mNavData.yAxis;
    VehicleInfo.heading = mNavData.heading;
    VehicleInfo.length = 3.6;
    // VehicleInfo.width = 2.0; //3.4m
    VehicleInfo.width = 3.4; // 3.4m

    //////////////////////////////////////////////////////////
    CollisionCheck ccheck;

    auto risk_objs = mPerception.objs;
    risk_objs.clear();
    // 对于每个障碍物与路径的每个点进行碰撞检测，找到最近的距离，如果距离小于0.5m则认为有碰撞风险
    static std::vector<std::vector<robot::object>> history_risk_vec(4, std::vector<robot::object>());
    for (int i = 0; i < mPerception.objs.size(); i++) // 障碍物与路径最近距离计算
    {
        Point ObjInfo;
        ObjInfo.x = mPerception.objs[i].x;
        ObjInfo.y = mPerception.objs[i].y;
        ObjInfo.heading = mPerception.objs[i].heading;
        ObjInfo.length = mPerception.objs[i].dy;
        ObjInfo.width = mPerception.objs[i].dx;
        auto ObjPoints = ccheck.GetRect(ObjInfo);

        dist = 10000.0;
        // 当前障碍物与路径进行碰撞检测
        for (int j = 0; j < mReferPath.x.size(); j++)
        {
            VehicleInfo.x = mReferPath.x[j];
            VehicleInfo.y = mReferPath.y[j];

            auto VehiclePoints = ccheck.GetRect(VehicleInfo);
            auto res = VehiclePoints;

            double d = ccheck.CheckRelation(VehiclePoints, ObjPoints, res);

            if (d < dist)
                dist = d;
        }
        // 将距离小于0.5m的障碍物认为有碰撞风险，加入风险列表
        if (dist < 0.1)
        {
            risk_objs.push_back(mPerception.objs[i]);
            risk_objs.back().height = dist; // 记录距离
        }
        // printf("obj i:%d, dist_to_ref:%.2f\n", i, dist);
    }
    history_risk_vec.erase(history_risk_vec.begin());
    history_risk_vec.emplace_back(risk_objs);
    int risk_num = 0;
    for (int i = 0; i < history_risk_vec.size(); i++)
    {
        if (history_risk_vec[i].size() > 0)
            risk_num++;
    }
    printf("risk vector: %d, %d, %d, %d\n", (int)history_risk_vec[0].size(), (int)history_risk_vec[1].size(),
           (int)history_risk_vec[2].size(), (int)history_risk_vec[3].size());
    if (risk_num == 0)
    {
        mPathPlanStatus.distance2Object = 100 + mPerception.objs.size();
        tPub.publish(mReferPath);
        return;
    }
    else
    {
        for (int i = history_risk_vec.size() - 1; i >= 0; i--)
        {
            if (history_risk_vec[i].size() > 0)
            {
                risk_objs = history_risk_vec[i];
                break;
            }
        }
    }

    // 这里对车辆周围90度范围内的障碍物进行过滤，认为这些障碍物对车辆有碰撞风险
    auto risk_objs_filter = risk_objs;
    risk_objs_filter.clear();
    std::vector<std::vector<robot::object>> history_risk_vec_filter(4, std::vector<robot::object>());
    for (auto i : risk_objs)
    {
        Point point;
        VehicleInfo.x = mNavData.xAxis;
        VehicleInfo.y = mNavData.yAxis;
        point.x = i.x;
        point.y = i.y;

        dh = PointDirectionToMe(point, VehicleInfo);

        if (fabs(dh) < 90.0)
            risk_objs_filter.push_back(i);
    }
    history_risk_vec_filter.erase(history_risk_vec_filter.begin());
    history_risk_vec_filter.emplace_back(risk_objs_filter);
    ////////////////////////记录历史碰撞
    int risk_num_filter = 0;
    for (int i = 0; i < history_risk_vec_filter.size(); i++)
    {
        if (history_risk_vec_filter[i].size() > 0)
            risk_num_filter++;
    }

    if (risk_num_filter == 0)
    {
        mPathPlanStatus.distance2Object = 200 + risk_objs.size();
        tPub.publish(mReferPath);
        return;
    }
    else
    {
        for (int i = history_risk_vec_filter.size() - 1; i >= 0; i--)
        {
            if (history_risk_vec_filter[i].size() > 0)
            {
                risk_objs_filter = history_risk_vec_filter[i];
                break;
            }
        }
    }
    ///////////////////////////////////////
    ////////////////////////////////////////////////////////////
    dist = 10000.0;
    // 这里计算最近障碍物的距离和方向，进行速度调整和安全判断
    double base_to_front = 2.3; // 车辆后轴中心到前雷达的距离
    int j = 0;
    double dist_to_refline = 10000;
    for (auto i : risk_objs_filter) // 障碍物与自车最近距离计算
    {
        ////////////////////
        Point ObjInfo;
        ObjInfo.x = i.x;
        ObjInfo.y = i.y;
        ObjInfo.heading = i.heading;
        ObjInfo.length = i.dy;
        ObjInfo.width = i.dx;
        auto ObjPoints = ccheck.GetRect(ObjInfo);
        Point VehicleInfo;
        VehicleInfo.x = mNavData.xAxis;
        VehicleInfo.y = mNavData.yAxis;
        VehicleInfo.heading = mNavData.heading;
        VehicleInfo.length = 3.6;
        // VehicleInfo.width = 2.0;
        VehicleInfo.width = 3.4; // 按照托盘尺寸计算
        // 计算车辆中心点
        double yaw = gis_utils::azimuthToYaw(VehicleInfo.heading);
        VehicleInfo.x = VehicleInfo.x + (base_to_front - VehicleInfo.length / 2) * cos(yaw);
        VehicleInfo.y = VehicleInfo.y + (base_to_front - VehicleInfo.length / 2) * sin(yaw);
        /////////////////////

        auto VehiclePoints = ccheck.GetRect(VehicleInfo);
        auto res = VehiclePoints;

        double ds = ccheck.CheckRelation(VehiclePoints, ObjPoints, res);
        double dertah = PointDirectionToMe(ObjInfo, VehicleInfo);
        printf("i obj:%f, ds:%f, dh:%f\n", j++, ds, dh);
        if (ds < dist)
        {
            dist = ds;
            dh = dertah;
            dist_to_refline = i.height;
        }
    }
    printf("Collision dist = %f,ref_dist:%f, collision heading = %f, spd = %f\n",
           dist, dist_to_refline, dh, mReferPath.desireSpeed);
    double min_speed = 0.0;
    double spd = mReferPath.desireSpeed;
    
    if (dist_to_refline > 0.2) // 车道外，不会碰撞
    {
        min_speed = 0.5;
        // 根据与障碍物的距离计算速度
        mReferPath.desireSpeed = std::min<double>(spd, dist_to_refline) * 0.5 + mVehicleData.vehicleSpeed * 0.5;
    }
    else // 发生碰撞
    {
        // 根据与障碍物的距离计算速度
        if ((dist < 25.0) && (dist >= 10.0))
            mReferPath.desireSpeed = std::min<double>(spd, dist / 5.0) * 0.5 + mVehicleData.vehicleSpeed * 0.5;
        if (dist < 20.0)
            mReferPath.desireSpeed = std::min<double>(spd, dist / 6.0) * 0.5 + mVehicleData.vehicleSpeed * 0.5;
        if (dist < 15.0)
            mReferPath.desireSpeed = std::min<double>(spd, dist / 7.0) * 0.5 + mVehicleData.vehicleSpeed * 0.5;
        if (dist < 10.0)
            mReferPath.desireSpeed = std::min<double>(spd, dist / 8.0) * 0.5 + mVehicleData.vehicleSpeed * 0.5;
        if (dist < 6.5 && mVehicleData.vehicleSpeed > 1.5)
            mReferPath.safety = 1;
        if (dist < 4.5)
            mReferPath.safety = 1;
    }

    mPathPlanStatus.distance2Object = dist;

    // high frequncy safaty check filter
    static int safety_check_counter = 0;

    if (mReferPath.safety == 1)
    {
        safety_check_counter += 1;

        if (safety_check_counter < 3)
            mReferPath.safety = 0;
        else
            safety_check_counter = 10;
    }
    else
        safety_check_counter = 0;

    if (mReferPath.safety == 1)
    {
        mReferPath.desireSpeed = 0.0;
        // ROS_ERROR("Collision dist = %f, dh = %f", dist, dh);
    } // else ROS_INFO("Collision dist = %f, dh = %f", dist, dh);

    tPub.publish(mReferPath);
}

void PathPlanComply::PublishPlanPath(ros::Publisher &tPub1, ros::Publisher &tPub2)
{
    printf("desireSpeed in plan path begin: %f\n", mPlanPath.desireSpeed);
    // 启动时，检查四周是否有障碍物
    bool is_around_unsafe = checkAroundObstacle();
    bool is_T_unsafe = checkTJunctionObstacle();
    bool is_in_waiting = checkIsInWaiting();
    if (mVehicleData.controlPanelState == 0)
    {
        printf("==========人工驾驶中==========\n");
        mPlanPath.x.clear();
        mPlanPath.y.clear();
        mPlanPath.desireSpeed = 0.0;
        tPub1.publish(mPlanPath);
        InitSafetyCheck = 0;
        ros::param::set("/canbus/light", 0);
        // int around_horn_cmd = 0;
        //  ros::param::get("/canbus/light", around_horn_cmd);
        //  if (around_horn_cmd == 7)
        //  {
        //      ros::param::set("/canbus/light", 0);
        //  }
        return;
    }
    if (mReferPath.x.size() < 5)
    {
        ROS_ERROR("publish plan path size less 5.");
        mPlanPath.x.clear();
        mPlanPath.y.clear();
        mPlanPath.desireSpeed = 0.0;
        tPub1.publish(mPlanPath);
        InitSafetyCheck = 0;
        ros::param::set("/canbus/light", 0);
        return;
    }
    // 起步观察
    printf("IniSafetyCheck: %d\n", InitSafetyCheck);

    if (InitSafetyCheck == 0)
    {
        InitSafetyCheck = 1;
        mPlanPath.safety = 1;
        if (is_around_unsafe)
        {
            ROS_WARN("周围有障碍物.");
            // printf("四周不安全\n");
            InitSafetyCheck = 0;
        }
        // T路口观察
        if (is_T_unsafe)
        {
            ROS_WARN("T路口有障碍物");
            // printf("T路口不安全\n");
            InitSafetyCheck = 0;
        }
    }

    // 速度规划
    mPlanPath.x = mReferPath.x;
    mPlanPath.y = mReferPath.y;
    mPlanPath.Path_Id = mReferPath.Path_Id;
    mPlanPath.safety = mReferPath.safety;
    mPlanPath.desireSpeed = mReferPath.desireSpeed;
    mPlanPath.planspeed = mReferPath.planspeed;

    double spd = mVehicleData.vehicleSpeed;
    double acc = mPlanPath.desireSpeed - mVehicleData.vehicleSpeed;
    // 这里的代码是速度的加速跟踪
    if (acc > 0.5)
    // if (true)
    {
        if (mVehicleData.linkPallet && mVehicleData.epsERR1 < 190) // 挂托盘，平滑过渡
        {
            mPlanPath.desireSpeed = 0.1 * mPlanPath.desireSpeed + 0.9 * mVehicleData.vehicleSpeed;
        }
        else
        {
            mPlanPath.desireSpeed = 0.3 * mPlanPath.desireSpeed + 0.7 * mVehicleData.vehicleSpeed;
        }
    }
    // emergency pause and recovery
    if (emergencyStop == 1)
    {
        mPlanPath.safety = 1;
        mPlanPath.desireSpeed = 0.0;
    }
    printf("desireSpeed in plan path end: %f\n", mPlanPath.desireSpeed);
    // perception & navigation alive check
    // double pGap = sysTime.now - sysTime.msgPerception;
    // double nGap = sysTime.now - sysTime.msgNavigation;

    // if(pGap > 1.0) {
    //     mPlanPath.safety = 1;
    //     mPlanPath.desireSpeed = 0.0;
    //     ROS_WARN("Percetion node offline !");
    // }

    // if(nGap > 1.0) {
    //     mPlanPath.safety = 1;
    //     mPlanPath.desireSpeed = 0.0;
    //     ROS_WARN("Localization node offline !");
    // }

    // location status check
    // 定位异常，则立即停车
    // if (mNavData.localization_status != 0)
    // {
    //     mPlanPath.desireSpeed = 0.0;
    //     ROS_WARN("GPS signal is weak !");
    // }

    // network check
    int network_status = 0;
    ros::param::get("/robot/planning/netcheck", network_status);
    // 网络异常，则立即停车
    if (network_status != 0)
    {
        mPlanPath.desireSpeed = 0.0;
        ROS_WARN("Network Disconnected !");
    }

    // sound & light command publish
    int HornCmd = 0;
    static int HornFlag = 1;
    if (mPathPlanStatus.curSpeed > 0.1)
    {
        sysTime.vehicleStop = ros::Time::now().toSec();
    }
    else
        sysTime.vehicleRun = ros::Time::now().toSec();

    double stime = sysTime.now - sysTime.vehicleStop;
    double rtime = sysTime.now - sysTime.vehicleRun;
    double ttime = sysTime.now - sysTime.taskStart;
    // 鸣笛逻辑
    if (stime > 3.0) // 起步鸣笛
        HornFlag = 1;

    if (HornFlag == 1 && rtime < 1.0 && stime < 1.0)
        HornCmd = 1;
    if (ttime < 1.0 && stime < 1.0)
        HornCmd = 1;
    if (rtime > 1.0) // 停车>1s，不鸣笛
        HornFlag = 0;

    // 灯光逻辑
    int LightCmd = 0;
    if (mVehicleData.curGear == GEAR_D)
    {
        int n = std::min<int>(10, mReferPath.x.size() - 1);

        Point me;
        Point point;
        me.x = mNavData.xAxis;
        me.y = mNavData.yAxis;
        me.heading = mNavData.heading;
        point.x = mReferPath.x[n];
        point.y = mReferPath.y[n];

        mPathPlanStatus.turning = "none";

        double dh = PointDirectionToMe(point, me);
        if (dh > 6.0) // 目标点在右侧超过10度
        {
            LightCmd = 3; // 开启右转向灯
            mPathPlanStatus.turning = "left";
        }

        if (dh < -6.0) // 目标点在左侧超过10度
        {
            LightCmd = 4; // 开启左转向灯
            mPathPlanStatus.turning = "right";
        }
    }

    if (mVehicleData.curGear == GEAR_R)
    {
        LightCmd = 5; // 开启倒车灯
    }

    if (mPlanPath.safety == 1)
    {
        LightCmd = 8;
        // int sum = sumVec(history_unsafe);
    }
    if (InitSafetyCheck == 0)
    {
        LightCmd = 7; // 报警
        mPlanPath.desireSpeed = 0.0;
    }
    else
    {
        if (checkIsInWaiting() && is_T_unsafe)
        {
            printf("车辆在等待区, 检测周围障碍物\n");
            LightCmd = 7; // 报警
            mPlanPath.desireSpeed = 0.0;
        }
    }
    if (command_state == 1) // 暂停
    {
        mPlanPath.desireSpeed = 0.0;
    }
    printf("end speedlimit: %f\n", mPlanPath.desireSpeed);
    // param set & msg pub
    ros::param::set("/canbus/light", LightCmd); // 灯光和起步周边障碍物报警再canbus用到了
    ros::param::set("/canbus/horn", HornCmd);   // 鸣笛也没有用到
    tPub1.publish(mPlanPath);
    tPub2.publish(mSoundLightData); // 这个没有用到
}

void PathPlanComply::pubLatticeTrajs(ros::Publisher &tPub)
{
    // draw
    visualization_msgs::MarkerArray traj_cluster;
    for (int i = 0; i < lattice_trajs_.size(); i++)
    {
        auto traj = lattice_trajs_[i];
        visualization_msgs::Marker traj_marker;
        traj_marker.header.frame_id = "robot";
        traj_marker.header.stamp = ros::Time::now();
        traj_marker.ns = "lattice_traj";
        traj_marker.id = i;
        traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
        traj_marker.action = visualization_msgs::Marker::ADD;
        traj_marker.scale.x = 0.05;
        traj_marker.color.a = 1.0;
        traj_marker.color.r = 1.0 - 0.2 * i;
        traj_marker.color.g = 0.2 * i;
        traj_marker.color.b = 0.0;
        for (auto point : traj)
        {
            geometry_msgs::Point p;
            p.x = point.path_point.x();
            p.y = point.path_point.y();
            p.z = 0.0;
            traj_marker.points.push_back(p);
        }
        traj_cluster.markers.push_back(traj_marker);
    }
    tPub.publish(traj_cluster);
}

bool PathPlanComply::checkTJunctionObstacle()
{
    std::vector<Dot> vertexes;
    Dot vertex = {-4.83, 18.3};
    vertexes.push_back(vertex);

    vertex = {0.22, 20};
    vertexes.push_back(vertex);

    vertex = {12.3, -17.8};
    vertexes.push_back(vertex);

    vertex = {7.2, -19.5};
    vertexes.push_back(vertex);
    bool is_unsafe = false;
    for (auto i : mPerception.objs)
    {
        Dot objpoint = {i.x, i.y};
        if (pointInRectangle(vertexes, objpoint) != 0)
        {
            // ROS_WARN("20m Object detect in lane");
            is_unsafe = true;
            break;
        }
    }
    static std::vector<bool> unsafe_vec(4, true);
    unsafe_vec.erase(unsafe_vec.begin());
    unsafe_vec.push_back(is_unsafe);
    int sum = 0;
    printf("T unsafe: ");
    for (int i = 0; i < unsafe_vec.size(); i++)
    {
        printf("%d ", int(unsafe_vec[i]));
        sum += unsafe_vec[i];
    }
    printf(" %d\n", sum);
    if (sum > 0) // 连续4帧有一帧有障碍物，则认为有危险
    {
        return true;
    }
    return false;
}

bool PathPlanComply::checkAroundObstacle()
{
    // 自车位置
    double x = mNavData.xAxis;
    double y = mNavData.yAxis;
    double h = mNavData.heading;

    auto point0 = local2global2(x, y, h, 2.3, 0.73, 0);   // 左前角
    auto point1 = local2global2(x, y, h, 2.3, -0.73, 0);  // 右前角
    auto point2 = local2global2(x, y, h, -0.8, 0.73, 0);  // 左后角
    auto point3 = local2global2(x, y, h, -0.8, -0.73, 0); // 右后角
    GeoPoint point0_geo(point0.x, point0.y);
    GeoPoint point1_geo(point1.x, point1.y);
    GeoPoint point2_geo(point2.x, point2.y);
    GeoPoint point3_geo(point3.x, point3.y);
    GeoLine line0(point0_geo, point1_geo); // 前
    GeoLine line1(point1_geo, point3_geo); // 右
    GeoLine line2(point2_geo, point3_geo); // 后
    GeoLine line3(point0_geo, point2_geo); // 左

    double dist0 = Distance2PerceptionObjs(line0);
    double dist1 = Distance2PerceptionObjs(line1);
    double dist2 = Distance2PerceptionObjs(line2);
    double dist3 = Distance2PerceptionObjs(line3);

    double safe_dist = 1.7 - 0.75;
    if (int(mVehicleData.linkPallet) == 1) // 连接托盘
    {
        safe_dist = 2.2 - 0.75;
    }

    bool unsafe = (dist0 < safe_dist || dist1 < safe_dist || dist2 < safe_dist || dist3 < safe_dist);

    static std::vector<bool> history_unsafe(30, true);
    history_unsafe.erase(history_unsafe.begin());
    history_unsafe.push_back(unsafe);
    int sum = 0;
    printf("safe_dist:%.2f, 前侧 = %.2f, 右侧 = %.2f, 后侧 = %.2f, 左侧 = %.2f\n", safe_dist, dist0, dist1, dist2, dist3);
    printf("Around_unsafe: ");
    for (int i = 0; i < history_unsafe.size(); i++)
    {
        printf("%d ", int(history_unsafe[i]));
        if (history_unsafe[i])
            sum += 1;
    }
    printf(" %d\n", sum);
    // if(history_unsafe.back() == true)
    // {
    //     return true;
    // }
    return sum > 5;
}

bool PathPlanComply::checkIsInWaiting()
{
    GeoPolygon waiting_polygon;
    waiting_polygon.outer().push_back(GeoPoint(1.0, 4.0));
    waiting_polygon.outer().push_back(GeoPoint(4.1, -4.7));
    waiting_polygon.outer().push_back(GeoPoint(-0.4, -6.13));
    waiting_polygon.outer().push_back(GeoPoint(-3.37, 2.45));
    GeoPoint cur_point(mNavData.xAxis, mNavData.yAxis);
    return bg::within(cur_point, waiting_polygon);
}

float PathPlanComply::distan2Line(GeoPoint point, GeoLine line)
{
    double d = bg::distance(point, line);
    return d;
}

std::vector<XYZ_COOR_S> PathPlanComply::LoadPathFile(string tPath)
{
    std::vector<XYZ_COOR_S> vector_list;
    std::vector<XYZ_COOR_S> incsv;
    float distance_temp = 0;
    std::string file_dir = "";
    ros::param::get("path_dir", file_dir);
    XYZ_COOR_S intp;

    FILE *fp;
    std::string path_dir = file_dir + tPath + ".csv";
    ROS_INFO("path_dir:%s", path_dir.c_str());
    fp = fopen(path_dir.c_str(), "r");

    while (!feof(fp))
    {
        fscanf(fp, "%f,%f,%f,%f", &intp.x_axis, &intp.y_axis, &intp.heading, &intp.z_axis);
        incsv.push_back(intp);
    }
    if (incsv.empty())
    {
        printf("incsv is empty\n");
        return vector_list;
    }
    XYZ_COOR_S intp_last = incsv.at(0);
    for (auto i : incsv)
    {
        XYZ_COOR_S xyz_temp;
        xyz_temp.heading = i.heading;
        xyz_temp.x_axis = i.x_axis;
        xyz_temp.y_axis = i.y_axis;
        xyz_temp.z_axis = i.z_axis;
        xyz_temp.velocity = 5;
        double dx = xyz_temp.x_axis - intp_last.x_axis;
        double dy = xyz_temp.y_axis - intp_last.y_axis;
        xyz_temp.p2pDistance = hypot(dx, dy);
        distance_temp += xyz_temp.p2pDistance;
        xyz_temp.dist_origin = distance_temp;
        intp_last = xyz_temp;
        vector_list.push_back(xyz_temp);
    }

    return vector_list;
}

float PathPlanComply::CalcuParkPointTangentDistance(
    float tCurX, float tCurY, float tTargetX, float tTargetY, float tTargetAngle)
{
    float angle_temp = tTargetAngle + 90.0;
    XYZ_COOR_S xyz_temp;

    if (angle_temp >= 360)
        angle_temp -= 360;

    xyz_temp = pubalgor.CaculateCrossPoint_P(tTargetX, tTargetY, angle_temp, tCurX, tCurY, tTargetAngle);

    double dx = xyz_temp.x_axis - tCurX;
    double dy = xyz_temp.y_axis - tCurY;

    return hypot(dx, dy);
}

float PathPlanComply::LimitSpeedByDistanceToStop(
    float tCurX, float tCurY, float tCurAngle,
    float tTargetX, float tTargetY, float tTargetAngle)
{
    if (mTaskPlanData.taskType == DOACTION)
    {
        if (mTaskPlanData.hookCmd == HOOKOPERATION)
        {
            printf("正在hook中....\n");
            if (mVehicleData.epsERR1 < 185)
            {
                printf("hook到位, task finished\n");
                mPathPlanStatus.taskExecuStatus = TASKFINISHED;
            }
        }
        if (mTaskPlanData.hookCmd == DECOUPLING)
        {
            printf("正在decoupling中....\n");
            if (mVehicleData.epsERR1 > 245)
            {
                printf("hook open, task finished\n");
                mPathPlanStatus.taskExecuStatus = TASKFINISHED;
            }
        }
        if (mTaskPlanData.stopX == -1 && mTaskPlanData.stopY == -1)
        {
            printf("无停止点, task finished\n");
            mPathPlanStatus.taskExecuStatus = TASKFINISHED;
        }
        return 0.0;
    }
    else
    {
        double dx = tTargetX - tCurX;
        double dy = tTargetY - tCurY;
        // remain_distance_ = hypot(dx, dy);
        if (mStopIndex < 0 || mStopIndex >= mDrivingPath.size() || mKeypoint < 0 || mKeypoint >= mDrivingPath.size())
        {
            printf("Invalid stop index or keypoint index. stopIndex: %d, keypoint: %d\n", mStopIndex, mKeypoint);
            return 0.0;
        }
        
        // 计算剩余距离
        remain_distance_ = mDrivingPath.at(mStopIndex).dist_origin - mDrivingPath.at(mKeypoint).dist_origin;
        printf("remain_distance_: %f, key_index:%d, stop_index:%d\n", remain_distance_, mKeypoint, mStopIndex);
        mPathPlanStatus.distance2Stop = remain_distance_;
        if (mTaskPlanData.taskType == ADAPTIVEPARK)
        {
            printf("倒车入库中...\n");
            Point point;
            Point VehicleInfo;

            VehicleInfo.x = tCurX;
            VehicleInfo.y = tCurY;
            VehicleInfo.heading = tCurAngle;
            point.x = tTargetX;
            point.y = tTargetY;

            double dh = PointDirectionToMe(point, VehicleInfo);

            if (fabs(dh) < 45.0)
            {
                printf("停车点在车前45度内\n");
                remain_distance_ = 0.0;
            }

            if (remain_distance_ < 0.2)
            {
                remain_distance_ = 0.0;
                printf("倒车入库结束 task finished\n");
                mPathPlanStatus.taskExecuStatus = TASKFINISHED;
            }
            return std::min<double>(remain_distance_, 0.6);
        }
        if (mTaskPlanData.taskType == ADAPTIVEHOOK)
        {
            printf("挂钩循迹中...\n");
            remain_distance_ = mHookPos.center_distance;
            printf("hook, remain_distance_ %f\n", remain_distance_);
    
            static int backdist_flag = 0;

            if (remain_distance_ < 1.5)
            {
            // 仅当车辆在指定矩形区域内时才检测托盘挂钩
//                std::vector<Dot> hook_rect;
//                hook_rect.push_back({-22.1, -17.9});
//                hook_rect.push_back({-22.1, -19.957});
//                hook_rect.push_back({-20.1, -19.957});
//                hook_rect.push_back({-20.1, -17.9});
//                Dot cur_pos = {mNavData.xAxis, mNavData.yAxis};
//                bool in_rect = (pointInRectangle(hook_rect, cur_pos) != 0);
//                printf("x:%f, y:%f\n", mNavData.xAxis, mNavData.yAxis);
//                printf("*******in_rect %d\n", in_rect);
//                if (in_rect && mVehicleData.linkPallet) // 在矩形框内且检测到托盘，切换到动作任务
            
                if (mVehicleData.linkPallet) // 检测到托盘，切换到动作任务
                {
                    printf("link hook到位, task finished\n");
                    mPathPlanStatus.taskExecuStatus = TASKFINISHED;
                    ROS_INFO("[Plan][info:][link pallet]");
                    return 0;
                }
                
		if(PalletType == 0) {
		    if(remain_distance_ < 0.3) backdist_flag = 1;
		    if(remain_distance_ > 0.5 && backdist_flag == 1) return 0;
                }

                return std::max(remain_distance_ / 2.0, 0.1);//6.0
            }else backdist_flag = 0;

            return 0.5;
        }
    }
    int stop_index = mStopIndex;
        if (mTaskPlanData.task_id == go_task_id_ || mTaskPlanData.task_id == back_task_id_)
        {
            auto port = findNextAirCraftParkingPort();
            printf("next port is:%d, parking:%d\n", port.id, port.parking);
            if (int(port.parking) == 1) // 入位中
            {
                stop_index =  port.go_stop_index == 0 ? port.back_stop_index : port.go_stop_index;
            }
        }
        remain_distance_ = mDrivingPath.at(stop_index).dist_origin - mDrivingPath.at(mKeypoint).dist_origin;
        printf("<<<<dest stop index: %d, airport stop index: %d>>>>>\n", mStopIndex, stop_index);
    if (mVehicleData.curGear == GEAR_N)
    {
        printf("挡位:N, 停车\n");
        return 0;
    }
    
    if (mVehicleData.curGear == GEAR_D)
    {
        printf("挡位:D, 路径跟踪\n");
        
        
         if (remain_distance_ < 0.5) // 距离终点还剩0.5开始刹停
        {
            if (stop_index == mStopIndex)
            {
             printf("离终点小于0.5, task_finished\n");
                remain_distance_ = 0.0;
                mPathPlanStatus.taskExecuStatus = TASKFINISHED;
                return 0.0;
                
            }
            else
            {
                printf("达到停止线停车点\n");
                return 0.0;
            }
        }
        else
        {
            if (remain_distance_ > 10.0) // 距离大于6m,按6m/s返回(也就是说会大于20km/h)
                return remain_distance_;
            else
                return std::max<double>(remain_distance_ / 6.0, 0.3);
        }
    }
    else
    {
        printf("挡位:%d\n", mVehicleData.curGear);
    }
    printf("return 0\n");
    return 0.0;
}

void PathPlanComply::UpdatePathInfo()
{

    int keyPoint = mKeypoint;
    int size = mDrivingPath.size();

    XYZ_COOR_S xyz_temp;

    float min = 100.0;
    float distance_temp = 0.0;
    if (mDrivingPath.size() == 0)
        return;
    if (mKeypoint > (size - 1))
        return;
    // printf("------------keypoint:%d-----------\n", keyPoint);
    for (int k = keyPoint; k < keyPoint + 80; ++k)
    {
        if (k >= (size - 1))
            break;

        xyz_temp = mDrivingPath.at(k % size);

        double dx = xyz_temp.x_axis - mNavData.xAxis;
        double dy = xyz_temp.y_axis - mNavData.yAxis;

        distance_temp = hypot(dx, dy);
        if (min > distance_temp)
        {
            min = distance_temp;
            keyPoint = k;
        }
    }
    mKeypoint = keyPoint;
}

void PathPlanComply::pubObstacles(ros::Publisher &tPub)
{
    // draw
    visualization_msgs::MarkerArray marker_array;
    for(int i = 0; i < obstacles_.size(); i++)
    {
        visualization_msgs::Marker obstacle_marker;
        obstacle_marker.header.frame_id = "robot";
        obstacle_marker.header.stamp = ros::Time::now();
        obstacle_marker.ns = "obstacle_" + std::to_string(i);
        obstacle_marker.id = i;
        obstacle_marker.type = visualization_msgs::Marker::CUBE;
        obstacle_marker.action = visualization_msgs::Marker::ADD;
        obstacle_marker.pose.position.x = obstacles_[i]->x;
        obstacle_marker.pose.position.y = obstacles_[i]->y;
        obstacle_marker.pose.position.z = 0.1;
        obstacle_marker.pose.orientation = tf::createQuaternionMsgFromYaw(obstacles_[i]->heading);
        obstacle_marker.scale.x = obstacles_[i]->width;
        obstacle_marker.scale.y = obstacles_[i]->length;
        obstacle_marker.scale.z = 0.1;
        obstacle_marker.color.a = 0.5;
        obstacle_marker.color.r = 1.0;
        obstacle_marker.color.g = 0.0;
        obstacle_marker.color.b = 0.0;
        marker_array.markers.push_back(obstacle_marker);
    }
    tPub.publish(marker_array);
}

ObstaclePtr PathPlanComply::findObstacle(std::vector<ObstaclePtr> &obstacles, double x, double y)
{
    for (auto obstacle_ptr : obstacles)
    {
        double ox = obstacle_ptr->x;
        double oy = obstacle_ptr->y;
        double dist = hypot(ox - x, oy - y);
        if (dist < 0.5)
        {
            return obstacle_ptr;
        }
    }
    return nullptr;
}

double PathPlanComply::getLaneLimitSpeedTest(double acur_lane_speed, double anext_lane_speed)
{
    static double next_lane_speed = anext_lane_speed;
    static double current_lane_speed = acur_lane_speed;
    static double lane_limit_speed = current_lane_speed;
    double divid_point_x = 6.25;
    double divid_point_y = -65.5;

    double ego_x = mNavData.xAxis;
    double ego_y = mNavData.yAxis;
    double ego_heading = gis_utils::azimuthToYaw(mNavData.heading);

    double ego_array_x = cos(ego_heading);
    double ego_array_y = sin(ego_heading);

    double ego_to_divid_point_x = divid_point_x - ego_x;
    double ego_to_divid_point_y = divid_point_y - ego_y;

    double dot_product = ego_array_x * ego_to_divid_point_x + ego_array_y * ego_to_divid_point_y;
    if (dot_product > 0) // 分界点在前方
    {
        double distance = sqrt(pow(ego_x - divid_point_x, 2) + pow(ego_y - divid_point_y, 2));
        if (distance < 10)
        {
            //lane_limit_speed = next_lane_speed + (current_lane_speed - next_lane_speed) / 10 * distance;
            // lane_limit_speed = next_lane_speed + (current_lane_speed - next_lane_speed) / 10 * distance;
        }
        printf("distance: %f\n", distance);
    }else
    {
    lane_limit_speed = next_lane_speed;
    }
    printf("============lane limit speed: %f, dot:%f\n", lane_limit_speed, dot_product);
    return lane_limit_speed;
}

std::vector<OriginalInsData> PathPlanComply::LatticePlan(
    std::vector<float> x, std::vector<float> y, std::vector<float> heading)
{
    std::vector<OriginalInsData> smoothed_path;
    std::vector<OriginalInsData> origin_refer_path;
    // printf("---lattice plan xsize:%d, ysize:%d,headingsize:%d---\n", x.size(), y.size(), heading.size());
    for (int i = 0; i < x.size(); i++)
    {
        OriginalInsData temp;
        temp.x = x[i];
        temp.y = y[i];
        temp.heading = heading[i];
        origin_refer_path.push_back(temp);
    }
    ReferenceLine referenceLine;
    transData.createReferenceLine(referenceLine, origin_refer_path);
    ReferenceLine referenceLineResult;
    smoothed_path = referenceLineProvider.smoothReferenceLine(
        referenceLine, &referenceLineResult);

    int smooth_point_num = smoothed_path.size();

    for (int i = 1; i < smooth_point_num; i++)
    {
        double x0 = smoothed_path[i].x;
        double y0 = smoothed_path[i].y;
        double x1 = smoothed_path[i - 1].x;
        double y1 = smoothed_path[i - 1].y;

        smoothed_path[i].heading = GetLineDirection(x0, y0, x1, y1) * M_PI / 180.0;
    }
    smoothed_path[0].heading = smoothed_path[1].heading;
    ////////////////////////////////////////////////////////
    // return smoothed_path;
    ///////////////////后面的代码有bug//////////////////////////
    int vehicle_near_id = FindNearestPoint2VehicleID(
        mNavData.xAxis, mNavData.yAxis, smoothed_path);

    double match_heading = smoothed_path[vehicle_near_id].heading;
    double match_kappa = smoothed_path[vehicle_near_id].kappa;

    TrajectoryPoint init_plan_point;
    init_plan_point.path_point.setX(mNavData.xAxis);
    init_plan_point.path_point.setY(mNavData.yAxis);
    printf("x0:%f, y0:%f\n", mNavData.xAxis, mNavData.yAxis);

    init_plan_point.setV(30.0 / 3.6);
    init_plan_point.setA(0.0);
    init_plan_point.path_point.setTheta(match_heading);
    init_plan_point.path_point.setKappa(match_kappa);
    // printf("---[plan on reference line, smooth path size:%d\n", smoothed_path.size());
    std::tuple<std::vector<TrajectoryPoint>, std::vector<std::vector<TrajectoryPoint>>, bool, int> generate_path =
        latticeplanner.PlanOnReferenceLine(init_plan_point, smoothed_path, lidarobjs_global_, 1);
    std::vector<TrajectoryPoint> temp_global_path_final = std::get<0>(generate_path);
    {
        lattice_trajs_ = std::get<1>(generate_path);
    }
    path_id = std::get<3>(generate_path);
    path_all_collision = std::get<2>(generate_path);
    std::vector<OriginalInsData> final_path_out;

    for (auto trajectory_point : temp_global_path_final)
    {
        OriginalInsData point_temp;
        point_temp.x = trajectory_point.path_point.x();
        point_temp.y = trajectory_point.path_point.y();
        point_temp.kappa = trajectory_point.path_point.kappa();
        point_temp.heading = trajectory_point.path_point.theta();
        point_temp.Id_path = path_id;
        point_temp.safety = path_all_collision;
        final_path_out.push_back(point_temp);
    }
    // printf("-----dddddd size:%d\n", final_path_out.size());
    if (final_path_out.empty())
    {
        return final_path_out;
    }
    // 计算path的属性
    double s = 0.0;
    for (int i = 0; i < final_path_out.size() - 1; i++)
    {
        if (i == 0)
        {
            final_path_out[i].length = 0.0;
            continue;
        }
        // printf("i:%d, s:%f, size:%d\n", i, s, final_path_out.size());
        double dx = final_path_out[i + 1].x - final_path_out[i].x;
        double dy = final_path_out[i + 1].y - final_path_out[i].y;

        s += std::sqrt(dx * dx + dy * dy);

        final_path_out[i].length = s;
    }

    return final_path_out;
}

bool PathPlanComply::IsApproach(const double num1, const double num2, const double factor)
{
    return ((num2 - num1) <= factor && (num2 - num1) >= -factor);
}

double PathPlanComply::GetLineDirection(double xsecond, double ysecond, double xfirst, double yfirst)
{
    double alpha = 0.0;

    if ((IsApproach(xsecond, xfirst, MIN_)) && (ysecond > yfirst))
    {
        alpha = 90;
    }
    else if ((IsApproach(xsecond, xfirst, MIN_)) && (ysecond < yfirst))
    {
        alpha = -90;
    }
    else if ((IsApproach(xsecond, xfirst, MIN_)) && (IsApproach(ysecond, yfirst, MIN_)))
    {
        alpha = 0;
    }
    else
        alpha = 180 / M_PI * atan2((ysecond - yfirst), (xsecond - xfirst));

    return alpha;
}

int PathPlanComply::FindNearestPoint2VehicleID(double x, double y, std::vector<OriginalInsData> lpath)
{
    int nearest_id = 0;
    float mini_dis = 1000000.0;
    int path_num = lpath.size();

    if (path_num < 1)
        return 0;

    for (int i = 0; i < path_num; i++)
    {
        double dx = lpath[i].x - x;
        double dy = lpath[i].y - y;

        float temp_dis = dx * dx + dy * dy;

        if (temp_dis < mini_dis)
        {
            mini_dis = temp_dis;
            nearest_id = i;
        }
    }

    return nearest_id;
}

std::vector<sCellMsg> PathPlanComply::Obj_Projecte_Map(robot::perception local_object, std::vector<sCellMsg> ldobjs_global)
{
    int obj_nums = local_object.objs.size();
    int subcon_num = 0;

    if (obj_nums < 1)
        return ldobjs_global;

    for (int i = 0; i < obj_nums; i++)
    {
        sCellMsg center_point;
        center_point.xg = local_object.objs[i].x;
        center_point.yg = local_object.objs[i].y;
        center_point.heading = local_object.objs[i].heading;
        ldobjs_global.emplace_back(center_point);
    }

    return ldobjs_global;
}

LidarPos PathPlanComply::local2global2(double ox, double oy, double oheading,
                                       double lx, double ly, double lheading)
{
    double h = 90.0 - oheading;

    if (h > 360.0)
        h -= 360.0;
    if (h < 0.0)
        h += 360.0;

    double rad = h * M_PI / 180.0;
    double cosd = cos(rad);
    double sind = sin(rad);
    // rotate
    double dx = cosd * lx - sind * ly;
    double dy = sind * lx + cosd * ly;
    // translation
    LidarPos point;
    point.x = dx + ox;
    point.y = dy + oy;
    point.heading = lheading + oheading;

    return point;
}

// direction define : - castrain coordinator
double PathPlanComply::PointDirectionToMe(Point point, Point me)
{
    double x0 = me.x;
    double y0 = me.y;
    double h0 = 90.0 - me.heading;
    double x1 = point.x;
    double y1 = point.y;
    double h1 = GetLineDirection(x1, y1, x0, y0);

    while (h0 > 360)
        h0 -= 360.0;
    while (h0 < 0.0)
        h0 += 360.0;
    while (h1 > 360)
        h1 -= 360.0;
    while (h1 < 0.0)
        h1 += 360.0;

    double dh = h1 - h0;

    if (dh > 180.0)
        dh -= 360.0;
    if (dh < -180.0)
        dh += 360.0;

    return dh; // negtive = left, positive = right
}

