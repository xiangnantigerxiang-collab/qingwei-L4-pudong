// CSV 路径装载、连接段生成与全局路径索引。由 path_plan_task.inc 包含。

double PathPlanComply::GetMapSpeedLimit() const {
    if(mKeypoint < 0 || static_cast<size_t>(mKeypoint) >= mDrivingPath.size())
        return std::numeric_limits<double>::infinity();
    // 复用本周期已有最近点索引；不重新读文件或扫描路径。0限速也必须生效。
    const double limit = mDrivingPath[mKeypoint].map_speed_limit;
    return std::isnan(limit) || limit < 0.0 ? 0.0 : limit;
}

void PathPlanComply::calcuGlobalPath(robot::task_plan_msg task_plan_t) {
    // 空任务/动作任务不生成行驶路径；其余任务依次装载、定位起点、接入路径、定位终点。
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

    // 按 pathList 原顺序拼接并累计整条路线里程，避免跨 CSV 时距离重置导致提前判到位。
    mDrivingPath.clear();
    for(auto pathName : task_plan_t.pathList) {
        path_list.clear();
        path_list = LoadPathFile(pathName);
        if(path_list.empty()) {
            // 一段读取失败时不能拼接剩余文件，避免跨越缺失路线或沿用旧路径。
            mDrivingPath.clear();
            return;
        }
        for(auto path_point : path_list) {
            // printf("Load path point: x=%.2f, y=%.2f, heading=%.2f\n", path_point.x_axis,
            //         path_point.y_axis, path_point.heading);
            path_point.p2pDistance = 0.0;
            path_point.dist_origin = 0.0;
            if(!mDrivingPath.empty()) {
                const auto &previous = mDrivingPath.back();
                path_point.p2pDistance = hypot(path_point.x_axis - previous.x_axis,
                                             path_point.y_axis - previous.y_axis);
                path_point.dist_origin = previous.dist_origin + path_point.p2pDistance;
            }
            mDrivingPath.push_back(path_point);
        }
    }
}

std::vector<XYZ_COOR_S> PathPlanComply::LoadPathFile(string tPath) {
    std::vector<XYZ_COOR_S> vector_list;
    float distance_temp = 0;
    std::string file_dir = "";
    ros::param::get("path_dir", file_dir);
    if(!file_dir.empty() && file_dir.back() != '/') file_dir += '/';
    std::string path_dir = file_dir + tPath + ".csv";
    ROS_INFO("path_dir:%s", path_dir.c_str());
    struct stat source_info;
    if(::stat(path_dir.c_str(), &source_info) != 0) {
        ROS_ERROR("Path CSV stat failed: %s", path_dir.c_str());
        return vector_list;
    }
    std::ifstream input(path_dir.c_str());
    if(!input.is_open()) {
        printf("路径文件打开失败: %s\n", path_dir.c_str());
        return vector_list;
    }
    std::string line;
    std::size_t line_number = 0;
    bool needs_upgrade = false;
    while(std::getline(input, line)) {
        ++line_number;
        path_csv::ROW_S row;
        bool changed = false;
        const auto status = path_csv::NormalizeMapRow(line, row, changed);
        if(status == path_csv::SKIP_ROW) continue;
        if(status == path_csv::INVALID_ROW) {
            ROS_ERROR("Invalid path CSV row: %s:%zu", path_dir.c_str(), line_number);
            return std::vector<XYZ_COOR_S>();
        }
        needs_upgrade = needs_upgrade || changed;
        XYZ_COOR_S xyz_temp;
        xyz_temp.heading = row.values[2];
        xyz_temp.x_axis = row.values[0];
        xyz_temp.y_axis = row.values[1];
        xyz_temp.velocity = 5;
        xyz_temp.map_speed_limit = row.values[3];
        if(!vector_list.empty()) {
            const auto &previous = vector_list.back();
            xyz_temp.p2pDistance = hypot(xyz_temp.x_axis - previous.x_axis,
                                        xyz_temp.y_axis - previous.y_axis);
        }
        distance_temp += xyz_temp.p2pDistance;
        xyz_temp.dist_origin = distance_temp;
        vector_list.push_back(xyz_temp);
    }
    if(input.bad()) {
        ROS_ERROR("Path CSV read failed: %s", path_dir.c_str());
        return std::vector<XYZ_COOR_S>();
    }
    if(needs_upgrade) {
        std::string error;
        if(!path_csv::SaveFourColumnMap(path_dir, source_info, error)) {
            ROS_ERROR("Path CSV upgrade failed: %s: %s", path_dir.c_str(), error.c_str());
            return std::vector<XYZ_COOR_S>();
        }
        ROS_INFO("Path CSV saved as x/y/heading/speed: %s", path_dir.c_str());
    }
    return vector_list;
}

void PathPlanComply::FindTaskStartPoint() {
    // 起始搜索排除最后 10 点，与周期推进时的前向 80 点窗口是两套规则。
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
        // 连接段会越过原路径后续5个点，保留这一小段的最低限速。
        float connection_limit = xyz.map_speed_limit;
        const int connection_end = std::min<int>(mKeypoint + 5, mDrivingPath.size() - 1);
        for(int i = mKeypoint + 1; i <= connection_end; ++i)
            connection_limit = std::min(connection_limit, mDrivingPath[i].map_speed_limit);
        XYZ_COOR_S xyz_temp;
        std::vector<XYZ_COOR_S> connect_path;
        xyz_temp.x_axis = x0;
        xyz_temp.y_axis = y0;
        xyz_temp.heading = h0;
        xyz_temp.map_speed_limit = connection_limit;

        connect_path.emplace_back(xyz_temp);

        for(int i = 0; i < 10; ++i) {
            xyz_temp = pubalgor.CalculNextPointByAngle_P(
                connect_path.back().x_axis, connect_path.back().y_axis, h0, 0.2);
            xyz_temp.map_speed_limit = connection_limit;

            connect_path.emplace_back(xyz_temp);
        }

        for(int i = mKeypoint + 5; i < mDrivingPath.size(); i++) {
            connect_path.emplace_back(mDrivingPath[i]);
        }

        // smooth
        auto bak = connect_path;
        connect_path.clear();
        for(int i = 0; i < bak.size() - 1; i += 6) {
            auto point = bak[i];
            // 抽稀不能丢失短限速区：当前采样段保存被跨过各点的最低限速。
            const int end = std::min<int>(i + 6, bak.size() - 1);
            for(int j = i + 1; j <= end; ++j)
                point.map_speed_limit = std::min(point.map_speed_limit, bak[j].map_speed_limit);
            connect_path.emplace_back(point);
        }
        connect_path.emplace_back(bak.back());

        // 与SplinePointSet的弧长参数一致；仅接入路径时构建，不在10Hz循环重新匹配。
        float source_s = 0.0f;
        connect_path.front().dist_origin = 0.0f;
        for(size_t i = 1; i < connect_path.size(); ++i) {
            source_s += hypot(connect_path[i].x_axis - connect_path[i - 1].x_axis,
                              connect_path[i].y_axis - connect_path[i - 1].y_axis);
            connect_path[i].dist_origin = source_s;
        }
        CSpline spline;
        spline.SplinePointSet(connect_path, mDrivingPath, 0.1);
        size_t speed_index = 0;
        for(size_t i = 0; i < mDrivingPath.size(); ++i) {
            const float sample_s = i + 1 == mDrivingPath.size() ? source_s : i * 0.1f;
            while(speed_index + 1 < connect_path.size() &&
                  connect_path[speed_index + 1].dist_origin <= sample_s) ++speed_index;
            mDrivingPath[i].map_speed_limit = connect_path[speed_index].map_speed_limit;
        }
        // 计算path
        calcuPath(mDrivingPath);
        mKeypoint = pubalgor.FindKeyPointByTargetPoint_P(
            mDrivingPath, mNavData.xAxis, mNavData.yAxis);
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
    if(mKeypoint < 0 || mKeypoint > (size - 1))
        return;
    // printf("------------keypoint:%d-----------\n", keyPoint);
    for(int k = keyPoint; k < keyPoint + 80; ++k) {
        // 必须检查真实末点；末段距离 >= 0.5 m 时，跳过末点会永远无法判定到位。
        if(k >= size)
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
