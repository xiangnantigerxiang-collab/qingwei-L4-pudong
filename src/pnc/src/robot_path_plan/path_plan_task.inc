// 本文件由 path_plan_comply.cpp 原位包含，负责“任务消息 -> 全局驾驶路径”。
// 不要单独编译；函数顺序沿用拆分前的单文件实现。

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
    // 跨帧历史窗口的初值与原函数内 static 声明保持一致
    history_risk_vec.assign(4, std::vector<robot::object>());
    unsafe_vec.assign(4, true);
    history_unsafe.assign(30, true);
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
    try
    {
        YAML::Node root = YAML::LoadFile(task_file.c_str());
        int new_go_id = root["go_task_id"].as<int>();
        int new_back_id = root["back_task_id"].as<int>();
        printf("go_task_id:%d, back_task_id:%d\n",new_go_id,new_back_id);
        // 解析port info(先组装局部表, 全部成功才提交)
        YAML::Node port_arr = root["aircraft_parkingPorts"];
        std::vector<AirCraftParkingPort> new_ports;
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
            new_ports.push_back(port);
            printf("i: %d,id:%d.parking:%d, gx: %f, gy: %f, bx: %f, by: %f\n", i,pid,port.parking, gx, gy, bx, by);
        }
        // 事务式提交: 中途任何字段异常都不会留下半新半旧的端口表
        go_task_id_ = new_go_id;
        back_task_id_ = new_back_id;
        aircraft_parking_ports_ = new_ports;
    }
    catch (const std::exception &e)
    {
        // 配置坏/缺不崩节点: 完整保留上一次端口表继续工作
        printf("aircraft_parking_port.yaml 加载失败, 保留上次端口表: %s\n",
               e.what());
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

    // 全局路径构建分四步：
    // 1. 按任务给出的顺序拼接 CSV；
    // 2. 用当前定位寻找起始关键点；
    // 3. 车辆距原路径 1~2.5 m 时生成连接段并重新采样；
    // 4. 在最终路径上定位任务停车点。后续速度规划依赖这两个索引。
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
    // task_plan_msg 是事件输入，不是每周期快照。每次收到消息都会重新加载
    // 机场配置和路径文件；必须先判断 task_id 是否切换，再覆盖旧任务数据。
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
        // 任务切换判定须在覆盖 mTaskPlanData 之前(块推进消息 task_id 不变)
        bool task_switched = (task_plan_t.task_id != mTaskPlanData.task_id);
        mTaskPlanData = task_plan_t;
        workMode = task_plan_t.workMode;
        mPathPlanStatus.stopX = task_plan_t.stopX;
        mPathPlanStatus.stopY = task_plan_t.stopY;
        mPathPlanStatus.stopAngle = task_plan_t.stopAngle;
        InitSafetyCheck = 0;
        // 仅真实任务切换时复位跨帧任务状态（块推进保持滚动窗口）：
        // 否则同任务每个块边界都会重播观察窗（约 2.5 s 禁行）并抹掉云端暂停。
        if (task_switched)
        {
            history_risk_vec.assign(4, std::vector<robot::object>());
            safety_check_counter_percep = 0;
            safety_check_counter_refer = 0;
            HornFlag = 1;
            // 播种全 true = 保守方向: 新任务起步再观察(约 2.5s @10Hz)
            unsafe_vec.assign(4, true);
            history_unsafe.assign(30, true);
            backdist_flag = 0;
            mPerception.objs.clear();
            mPerceptionFrontScan.objs.clear();
            mPerceptionBackScan.objs.clear();
            command_state = 2; // 上一任务的云端"暂停"不带入新任务
            if (emergencyStop == 1)
                printf("警告: 急停闩锁仍生效, 新任务将被压 0 速(需人工解除)\n");
            mReferPath.planspeed = 0.0;
            mReferPath.safety = 0;
        }
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
    // 作业型路径在 10 Hz 主循环中按任务类型补充生成。
    // 这里保留了既有的同值比较条件：ADAPTIVEHOOK 会持续重建路径，其余几个
    // 分支中的内层生成条件当前不可达。该现象属于业务缺陷，不在整理中改动。
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
    // YAML 只保存停车点坐标；任务接收后把每个停车点投影到本次全局路径索引，
    // 这样周期限速逻辑可直接用 dist_origin 计算到下一飞机位的剩余距离。
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
    if (aircraft_parking_ports_.empty())
    {
        // 空端口表(yaml 加载失败/数组为空): 停车点回退路径终点, 防 front() 越界
        printf("aircraft_parking_ports 为空, 停车点回退路径终点\n");
        AirCraftParkingPort fallback;
        fallback.go_stop_index = (int)mDrivingPath.size() - 1;
        fallback.back_stop_index = (int)mDrivingPath.size() - 1;
        return fallback;
    }
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
