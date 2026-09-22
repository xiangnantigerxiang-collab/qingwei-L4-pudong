// 休眠规划入口及调试输出。保留原有实现和调用门控；不可直接用于实车。

std::vector<OriginalInsData> PathPlanComply::LatticePlan(
    std::vector<float> x, std::vector<float> y, std::vector<float> heading) {
    std::vector<OriginalInsData> smoothed_path;
    std::vector<OriginalInsData> origin_refer_path;
    // printf("---lattice plan xsize:%d, ysize:%d,headingsize:%d---\n", x.size(), y.size(), heading.size());
    for(int i = 0; i < x.size(); i++) {
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

    for(int i = 1; i < smooth_point_num; i++) {
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

    for(auto trajectory_point : temp_global_path_final) {
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
    if(final_path_out.empty()) {
        return final_path_out;
    }
    // 计算path的属性
    double s = 0.0;
    for(int i = 0; i < final_path_out.size() - 1; i++) {
        if(i == 0) {
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

bool PathPlanComply::JudgeTaskPlanMsgChanged(
    robot::task_plan_msg tLast, robot::task_plan_msg tNow) {
    int path_num = tNow.pathList.size();
    if(tNow.taskType == NOTHING)
        return 0;
    // if (tNow.taskType == DOACTION)
    //     return 0;
    if(path_num == 0)
        return 0;
    if(tLast.pathList.size() != path_num)
        return 1;
    if(tLast.stopX != tNow.stopX)
        return 1;
    if(tLast.stopY != tNow.stopY)
        return 1;
    if(tLast.task_id != tNow.task_id)
        return 1;
    if(tLast.taskType != tNow.taskType)
        return 1;

    for(int i = 0; i < path_num; ++i) {
        if(0 != strcmp(tLast.pathList[i].c_str(), tNow.pathList[i].c_str()))
            return 1;
    }
    return 0;
}

bool PathPlanComply::JudgeShiftParkingPoint(
    robot::task_plan_msg tLast, robot::task_plan_msg tNow) {
    if(tNow.taskType == NOTHING)
        return 0;
    if(tNow.taskType == DOACTION)
        return 0;
    if(tNow.pathList.size() == 0)
        return 0;

    if(tNow.stopX == 0 && tNow.stopY == 0)
        return 0;
    if(tNow.stopX == -1 && tNow.stopY == -1)
        return 0;

    double dx = tLast.stopX - tNow.stopX;
    double dy = tLast.stopY - tNow.stopY;

    if(hypot(dx, dy) > 0.5)
        return 1;
    else
        return 0;
}

double PathPlanComply::getLaneLimitSpeedTest(double acur_lane_speed, double anext_lane_speed) {
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
    if(dot_product > 0) {  // 分界点在前方
        double distance = sqrt(pow(ego_x - divid_point_x, 2) + pow(ego_y - divid_point_y, 2));
        if(distance < 10) {
            //lane_limit_speed = next_lane_speed + (current_lane_speed - next_lane_speed) / 10 * distance;
            // lane_limit_speed = next_lane_speed + (current_lane_speed - next_lane_speed) / 10 * distance;
        }
    } else {
        lane_limit_speed = next_lane_speed;
    }
    return lane_limit_speed;
}
