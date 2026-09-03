// 本文件由 path_plan_comply.cpp 原位包含。
// LatticePlan 入口当前在 PublishReferPath 中被注释，内部也保留“后面的代码有 bug”
// 标记，不能仅取消调用注释就用于实车。文件尾的坐标/方向辅助函数仍被现役
// 周边障碍物和碰撞判断调用，因此本文件不能从编译链删除。

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
