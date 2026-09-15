// CSV 路径装载、连接段生成与全局路径索引。由 path_plan_task.inc 包含。

void PathPlanComply::calcuGlobalPath(robot::task_plan_msg task_plan_t) {
    // 空任务/动作任务不生成行驶路径；其余任务依次装载、定位起点、接入路径、定位终点。
    printf("=============calcuGlobalPath keyIndex and stopIndex==============\n");
    if(task_plan_t.taskType == NOTHING)
        return;
    if(task_plan_t.taskType == DOACTION)
        return;

    LoadTaskPaths(task_plan_t);
    if(mDrivingPath.size() < 10) {
        printf("路径太短，小于10个点\n");
        return;
    }

    FindTaskStartPoint();
    ConnectVehicleToPath();
    FindTaskStopPoint(task_plan_t);
}

void PathPlanComply::LoadTaskPaths(const robot::task_plan_msg &task_plan_t) {
    std::vector<XYZ_COOR_S> path_list;

    // 按 pathList 原顺序拼接，每段保留 CSV 装载时的里程值；此处不重新计算整条里程。
    mDrivingPath.clear();
    for(auto pathName : task_plan_t.pathList) {
        path_list.clear();
        path_list = LoadPathFile(pathName);
        for(auto path_point : path_list) {
            // printf("Load path point: x=%.2f, y=%.2f, heading=%.2f\n", path_point.x_axis,
            //         path_point.y_axis, path_point.heading);
            mDrivingPath.push_back(path_point);
        }
    }
}

std::vector<XYZ_COOR_S> PathPlanComply::LoadPathFile(string tPath) {
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
    if(fp == NULL) {
        printf("路径文件打开失败: %s\n", path_dir.c_str());
        return vector_list;
    }

    while(!feof(fp)) {
        fscanf(fp, "%f,%f,%f,%f", &intp.x_axis, &intp.y_axis, &intp.heading, &intp.z_axis);
        incsv.push_back(intp);
    }
    fclose(fp);
    if(incsv.empty()) {
        printf("incsv is empty\n");
        return vector_list;
    }
    XYZ_COOR_S intp_last = incsv.at(0);
    for(auto i : incsv) {
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

void PathPlanComply::FindTaskStartPoint() {
    // 起始搜索排除最后 10 点，与周期推进时的前向 80 点窗口是两套规则。
    printf("111111mKeyPoint:%d\n", mKeypoint);
    {
        // 程序启动时，查找起始点
        double dist_with_first = 0.0;
        double min_dist = 1e6;
        for(int i = 0; i < mDrivingPath.size() - 10; i++) {
            auto p = mDrivingPath[i];
            double dist = hypot(p.x_axis - mNavData.xAxis, p.y_axis - mNavData.yAxis);
            if(i == 0) {
                dist_with_first = dist;
            }
            if(dist < min_dist) {
                mKeypoint = i;
                min_dist = dist;
            }
        }
        printf("first_dist: %f\n", dist_with_first);
        printf("mKeyPoint:%d, size:%d, min_dist:%f\n", mKeypoint, mDrivingPath.size(), min_dist);
    }
}

void PathPlanComply::ConnectVehicleToPath() {
    // 距原路径严格在 1~2.5 m 之间才补连接段；过近或过远均沿用原路径。
    auto xyz = mDrivingPath.at(mKeypoint);
    double x0 = mNavData.xAxis;
    double y0 = mNavData.yAxis;
    double h0 = mNavData.heading;
    double x1 = xyz.x_axis;
    double y1 = xyz.y_axis;
    double h1 = xyz.heading;

    while(h0 > 360.0)
        h0 -= 360.0;
    while(h0 < 0.0)
        h0 += 360.0;
    while(h1 > 360.0)
        h1 -= 360.0;
    while(h1 < 0.0)
        h1 += 360.0;

    // 横向偏差
    float distance2loadpos = pubalgor.CalculatePoint2PointDistance_P(
        x0, y0, 0, x1, y1, 0);

    if(distance2loadpos > 1.0 && distance2loadpos < 2.5) {
        // generate connect path
        printf("Distance to path is %.2f, generating connect path...\n", distance2loadpos);
        XYZ_COOR_S xyz_temp;
        std::vector<XYZ_COOR_S> connect_path;
        xyz_temp.x_axis = x0;
        xyz_temp.y_axis = y0;
        xyz_temp.heading = h0;

        connect_path.emplace_back(xyz_temp);

        for(int i = 0; i < 10; ++i) {
            xyz_temp = pubalgor.CalculNextPointByAngle_P(
                connect_path.back().x_axis, connect_path.back().y_axis, h0, 0.2);

            connect_path.emplace_back(xyz_temp);
        }

        for(int i = mKeypoint + 5; i < mDrivingPath.size(); i++) {
            connect_path.emplace_back(mDrivingPath[i]);
        }

        // smooth
        auto bak = connect_path;
        connect_path.clear();
        for(int i = 0; i < bak.size() - 1; i += 6)
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
}

void PathPlanComply::FindTaskStopPoint(const robot::task_plan_msg &task_plan_t) {
    // 从第 4 点开始找最近停车点，必须在连接段生成后执行，保证索引指向最终路径。
    float stop_x = task_plan_t.stopX;
    float stop_y = task_plan_t.stopY;
    {
        // 程序启动时，查找停车点
        double min_dist = 1e6;
        for(int i = 4; i < mDrivingPath.size(); i++) {
            auto p = mDrivingPath[i];
            double dist = hypot(p.x_axis - stop_x, p.y_axis - stop_y);
            if(dist < min_dist) {
                mStopIndex = i;
                min_dist = dist;
            }
        }
        printf("4444mStopIndex:%d, dist:%f\n", mStopIndex, min_dist);
    }
}

void PathPlanComply::calcuPath(std::vector<XYZ_COOR_S> &path) {
    if(path.empty()) {
        return;
    }
    path.front().dist_origin = 0.0;
    for(int i = 0; i < path.size(); i++) {
        if(i < path.size() - 1) {
            double dx = path.at(i + 1).x_axis - path.at(i).x_axis;
            double dy = path.at(i + 1).y_axis - path.at(i).y_axis;
            path.at(i).heading = fmod(90 - atan2(dy, dx) * 180.0 / M_PI + 360.0, 360.0);
            path.at(i + 1).dist_origin = path.at(i).dist_origin + hypot(dx, dy);
        } else {
            path.at(i).heading = path.at(i - 1).heading;
        }
    }
}

int PathPlanComply::findNearestIndexOnPath(std::vector<XYZ_COOR_S> &path, double x, double y) {
    double min_distance = 1e6;
    int min_index = 0;
    for(int i = 10; i < path.size(); i++) {
        auto point = path.at(i);
        double dx = point.x_axis - x;
        double dy = point.y_axis - y;
        double distance = hypot(dx, dy);
        if(distance < min_distance) {
            min_distance = distance;
            min_index = i;
        }
    }
    return min_index;
}

void PathPlanComply::UpdatePathInfo() {
    // 仅向前寻找最近点，避免进度回退。循环上界使用持续更新的 keyPoint + 80，
    // 因此搜索范围会随候选最近点推进，并非固定在初始关键点后的 80 点内。
    int keyPoint = mKeypoint;
    int size = mDrivingPath.size();

    XYZ_COOR_S xyz_temp;

    float min = 100.0;
    float distance_temp = 0.0;
    if(mDrivingPath.size() == 0)
        return;
    if(mKeypoint > (size - 1))
        return;
    // printf("------------keypoint:%d-----------\n", keyPoint);
    for(int k = keyPoint; k < keyPoint + 80; ++k) {
        if(k >= (size - 1))
            break;

        xyz_temp = mDrivingPath.at(k % size);

        double dx = xyz_temp.x_axis - mNavData.xAxis;
        double dy = xyz_temp.y_axis - mNavData.yAxis;

        distance_temp = hypot(dx, dy);
        if(min > distance_temp) {
            min = distance_temp;
            keyPoint = k;
        }
    }
    mKeypoint = keyPoint;
}
