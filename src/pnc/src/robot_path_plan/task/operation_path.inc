// 作业路径调度与挂钩、入库、装载路径生成。由 path_plan_task.inc 包含。

void PathPlanComply::handleDrivingPath() {
    // 作业型路径在 10 Hz 主循环中按任务类型补充生成。
    // 这里保留了既有的同值比较条件：ADAPTIVEHOOK 会持续重建路径，其余几个
    // 分支中的内层生成条件当前不可达。该现象属于业务缺陷，不在整理中改动。
    if(mTaskPlanData.taskType == ADAPTIVEUNLOADPATH) {
        if(mTaskPlanData.taskType != ADAPTIVEUNLOADPATH) {
            mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
            UpdateStopPoint(mTaskPlanData);
        }
    }

    if(mTaskPlanData.taskType == ADAPTIVELOADPATH) {
        if(mTaskPlanData.taskType != ADAPTIVELOADPATH) {
            mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
            GenerateLoadPath(mTaskPlanData);
            ROS_INFO("Generate Load Path %d Points", (int)mDrivingPath.size());
        }
    }

    if(mTaskPlanData.taskType == ADAPTIVEHOOK) {
        if(mTaskPlanData.taskType != ADAPTIVEHOOK)
            mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
        GenerateHookPath(mHookPos);
        ROS_INFO("Generate Hook Path %d Points", (int)mDrivingPath.size());
    }
    if(mTaskPlanData.taskType == DOACTION) {
        // no need path do nonthing
        // mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
    }

    if(mTaskPlanData.taskType == ADAPTIVEPARK) {
        if(mTaskPlanData.taskType != ADAPTIVEPARK) {
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

void PathPlanComply::GenerateHookPath(robot::hook_position hook_pos) {
    printf("生成hook path,hook_dist:%f pos: %f, %f\n",
           hook_pos.center_distance, hook_pos.center_point_x, hook_pos.center_point_y);
    std::vector<std::array<float, 3>> hook_path;
    XYZ_COOR_S xyz_temp;

    if(hook_pos.center_distance > 10.0)
        return;
    if(hook_pos.center_distance < 2.0)
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
    for(auto temp : hook_path) {
        xyz_temp.x_axis = temp[0];
        xyz_temp.y_axis = temp[1];
        xyz_temp.heading = temp[2];
        printf("hook path: %f %f %f\n", xyz_temp.x_axis, xyz_temp.y_axis, xyz_temp.heading);
        mDrivingPath.push_back(xyz_temp);
    }
    mStopIndex = mDrivingPath.size() - 1;
    // 从原路径终点继续补点；第 25 点（1.25 m）更新停车坐标，其余补点供跟踪使用。
    for(int i = 1; i < 300; ++i) {
        x0 = mDrivingPath.back().x_axis;
        y0 = mDrivingPath.back().y_axis;
        h0 = mDrivingPath.back().heading;

        xyz_temp = pubalgor.CalculNextPointByAngle_P(x0, y0, h0, 0.05);

        mDrivingPath.emplace_back(xyz_temp);

        if(25 == i && hook_pos.center_distance > 2.0) {
            mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
            mPathPlanStatus.stopX = xyz_temp.x_axis;
            mPathPlanStatus.stopY = xyz_temp.y_axis;
            mPathPlanStatus.stopAngle = xyz_temp.heading;
        }
    }
    // 计算path
    calcuPath(mDrivingPath);
    for(auto &pt : mDrivingPath) {
        pt.heading += 180.0;
        if(pt.heading > 360.0)
            pt.heading -= 360.0;
    }

    mKeypoint = 0;
}

void PathPlanComply::GenerateParkPath(robot::palletpos park_pos, int withdraw) {
    double x0 = mNavData.xAxis;
    double y0 = mNavData.yAxis;
    double h0 = mNavData.heading;
    double x1 = park_pos.xg;
    double y1 = park_pos.yg;
    double h1 = park_pos.heading;

    while(h0 > 360.0)
        h0 -= 360.0;
    while(h0 < 0.0)
        h0 += 360.0;
    while(h1 > 360.0)
        h1 -= 360.0;
    while(h1 < 0.0)
        h1 += 360.0;

    float distance2loadpos = pubalgor.CalculatePoint2PointDistance_P(
        x0, y0, 0, x1, y1, 0);

    if(distance2loadpos > 30.0) {
        ROS_INFO("Parking position is so far");
        return;
    }

    int cur_num = pubalgor.FindKeyPointByTargetPoint_P(mDrivingPath, x0, y0);
    int tar_num = pubalgor.FindKeyPointByTargetPoint_P(mDrivingPath, x1, y1);

    XYZ_COOR_S xyz_temp;
    std::vector<XYZ_COOR_S> park_path;

    for(int i = cur_num; i >= tar_num + withdraw; i--)
        park_path.emplace_back(mDrivingPath[i]);

    // generate connect path
    double insert_h = h1;
    std::vector<XYZ_COOR_S> connect_path;

    xyz_temp.x_axis = x1;
    xyz_temp.y_axis = y1;
    xyz_temp.heading = h1;

    connect_path.emplace_back(xyz_temp);

    for(int i = 0; i < 15; ++i) {
        xyz_temp = pubalgor.CalculNextPointByAngle_P(
            connect_path.back().x_axis, connect_path.back().y_axis, insert_h, 0.2);

        connect_path.emplace_back(xyz_temp);
    }

    for(int i = connect_path.size() - 1; i >= 0; i--) {
        park_path.emplace_back(connect_path[i]);
    }

    // sample
    auto bak = park_path;
    park_path.clear();
    for(int i = 0; i < bak.size() - 1; i += 6)
        park_path.emplace_back(bak[i]);
    park_path.emplace_back(bak.back());

    CSpline spline;
    spline.SplinePointSet(park_path, mDrivingPath, 0.1);

    // 入库轨迹末端反向延长，再用目标坐标重搜停止索引；车辆最近点从 0 重新推进。
    h1 = h1 - 180.0;
    while(h1 > 360.0)
        h1 -= 360.0;
    while(h1 < 0.0)
        h1 += 360.0;

    mDrivingPath.back().heading = h1;
    for(int i = 1; i < 100; ++i) {
        xyz_temp = pubalgor.CalculNextPointByAngle_P(
            mDrivingPath.back().x_axis, mDrivingPath.back().y_axis,
            mDrivingPath.back().heading, 0.1);

        mDrivingPath.emplace_back(xyz_temp);
    }
    calcuPath(mDrivingPath);
    mStopIndex = findNearestIndexOnPath(mDrivingPath, x1, y1);
    mKeypoint = 0;
}

void PathPlanComply::GenerateLoadPath(robot::task_plan_msg task_plan_t) {
    mLoadPos.xg = -33.96;
    mLoadPos.yg = -24.92;
    mLoadPos.heading = 275.7;

    double x0 = mNavData.xAxis;
    double y0 = mNavData.yAxis;
    double x1 = mLoadPos.xg;
    double y1 = mLoadPos.yg;

    float distance2loadpos = pubalgor.CalculatePoint2PointDistance_P(
        x0, y0, 0, x1, y1, 0);

    if(distance2loadpos > 30.0) {
        ROS_INFO("Load position is so far");
        return;
    }

    std::vector<XYZ_COOR_S> front_segment, mid_segment, rear_segment;
    XYZ_COOR_S xyz_temp, xyz_temp1, end_point;
    float angle_temp = mLoadPos.heading - 90.0;

    if(angle_temp < 0.0)
        angle_temp += 360.0;

    xyz_temp = pubalgor.CalculNextPointByAngle_P(
        mLoadPos.xg, mLoadPos.yg, angle_temp, 2.3);

    angle_temp -= 90.0;

    if(angle_temp < 0.0)
        angle_temp += 360.0;

    xyz_temp = pubalgor.CalculNextPointByAngle_P(
        xyz_temp.x_axis, xyz_temp.y_axis, angle_temp, 10.0);

    // extend path
    for(int i = 1; i < 100; ++i) {
        xyz_temp1 = pubalgor.CalculNextPointByAngle_P(
            xyz_temp.x_axis, xyz_temp.y_axis, mLoadPos.heading, 0.1);

        mid_segment.emplace_back(xyz_temp1);

        xyz_temp = xyz_temp1;

        if(90 == i) {
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

    for(auto pathName : task_plan_t.pathList) {
        path_list = LoadPathFile(pathName);
        end_point = path_list.at(0);
    }

    rear_segment = pubalgor.GenerateBezierPath_P(
        mid_segment.back().x_axis, mid_segment.back().y_axis, mid_segment.back().heading,
        end_point.x_axis, end_point.y_axis, end_point.heading);

    mDrivingPath.clear();

    for(auto pt : front_segment)
        mDrivingPath.emplace_back(pt);
    for(auto pt : mid_segment)
        mDrivingPath.emplace_back(pt);
    for(auto pt : rear_segment)
        mDrivingPath.emplace_back(pt);
    for(auto pt : path_list)
        mDrivingPath.emplace_back(pt);

    // extend path
    for(int i = 1; i < 100; ++i) {
        xyz_temp = pubalgor.CalculNextPointByAngle_P(
            mDrivingPath.back().x_axis, mDrivingPath.back().y_axis, mDrivingPath.back().heading, 0.05);

        mDrivingPath.emplace_back(xyz_temp);
    }
    calcuPath(mDrivingPath);
    mStopIndex = findNearestIndexOnPath(mDrivingPath, x1, y1);
    mKeypoint = 0;
}

void PathPlanComply::UpdateStopPoint(robot::task_plan_msg task_plan_t) {
    std::vector<XYZ_COOR_S> path_list;

    for(auto pathName : task_plan_t.pathList)
        path_list = LoadPathFile(pathName);

    size_t size = path_list.size();

    mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
    mPathPlanStatus.stopX = path_list.at(size - 1).x_axis;
    mPathPlanStatus.stopY = path_list.at(size - 1).y_axis;
    mPathPlanStatus.stopAngle = path_list.at(size - 1).heading;
}
