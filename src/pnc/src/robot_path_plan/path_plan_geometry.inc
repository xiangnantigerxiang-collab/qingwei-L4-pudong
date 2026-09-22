// 矩形构造与路径方向/距离辅助。仅由 path_plan_comply.cpp 包含。

using Dot = std::pair<double, double>;

std::vector<Dot> getRectangleVertices(const std::vector<Dot> &centerLine, double width) {
    if(centerLine.size() != 2) {
        throw invalid_argument("centerLine 必须包含两个点");
    }

    if(width <= 0) {
        throw invalid_argument("矩形宽度必须大于 0");
    }

    double x1 = centerLine[0].first;
    double y1 = centerLine[0].second;
    double x2 = centerLine[1].first;
    double y2 = centerLine[1].second;

    double vx = x2 - x1;
    double vy = y2 - y1;

    double len = sqrt(vx * vx + vy * vy);

    if(len == 0) {
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

bool PathPlanComply::IsApproach(const double num1, const double num2, const double factor) {
    return ((num2 - num1) <= factor && (num2 - num1) >= -factor);
}

double PathPlanComply::GetLineDirection(double xsecond, double ysecond, double xfirst, double yfirst) {
    double alpha = 0.0;

    if((IsApproach(xsecond, xfirst, MIN_)) && (ysecond > yfirst)) {
        alpha = 90;
    } else if((IsApproach(xsecond, xfirst, MIN_)) && (ysecond < yfirst)) {
        alpha = -90;
    } else if((IsApproach(xsecond, xfirst, MIN_)) && (IsApproach(ysecond, yfirst, MIN_))) {
        alpha = 0;
    } else
        alpha = 180 / M_PI * atan2((ysecond - yfirst), (xsecond - xfirst));

    return alpha;
}

int PathPlanComply::FindNearestPoint2VehicleID(double x, double y, std::vector<OriginalInsData> lpath) {
    int nearest_id = 0;
    float mini_dis = 1000000.0;
    int path_num = lpath.size();

    if(path_num < 1)
        return 0;

    for(int i = 0; i < path_num; i++) {
        double dx = lpath[i].x - x;
        double dy = lpath[i].y - y;

        float temp_dis = dx * dx + dy * dy;

        if(temp_dis < mini_dis) {
            mini_dis = temp_dis;
            nearest_id = i;
        }
    }

    return nearest_id;
}

double PathPlanComply::PointDirectionToMe(Point point, Point me) {
    double x0 = me.x;
    double y0 = me.y;
    double h0 = 90.0 - me.heading;
    double x1 = point.x;
    double y1 = point.y;
    double h1 = GetLineDirection(x1, y1, x0, y0);

    while(h0 > 360)
        h0 -= 360.0;
    while(h0 < 0.0)
        h0 += 360.0;
    while(h1 > 360)
        h1 -= 360.0;
    while(h1 < 0.0)
        h1 += 360.0;

    double dh = h1 - h0;

    if(dh > 180.0)
        dh -= 360.0;
    if(dh < -180.0)
        dh += 360.0;

    return dh;  // negtive = left, positive = right
}

float PathPlanComply::distan2Line(GeoPoint point, GeoLine line) {
    double d = bg::distance(point, line);
    return d;
}

float PathPlanComply::CalcuParkPointTangentDistance(
    float tCurX, float tCurY, float tTargetX, float tTargetY, float tTargetAngle) {
    float angle_temp = tTargetAngle + 90.0;
    XYZ_COOR_S xyz_temp;

    if(angle_temp >= 360)
        angle_temp -= 360;

    xyz_temp = pubalgor.CaculateCrossPoint_P(tTargetX, tTargetY, angle_temp, tCurX, tCurY, tTargetAngle);

    double dx = xyz_temp.x_axis - tCurX;
    double dy = xyz_temp.y_axis - tCurY;

    return hypot(dx, dy);
}
