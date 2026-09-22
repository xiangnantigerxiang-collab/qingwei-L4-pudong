// R 挡横向几何控制 GeometricConstrol 实现：
// 局部坐标转换、预瞄点查找、转弯半径与转向角计算。

#include "lateral_reverse_control.h"
#include <algorithm>

GeometricConstrol::GeometricConstrol() {
}

float GeometricConstrol::LateralControlTrack1(
    vector<XYZ_COOR_S> tPathList, XYZ_COOR_S tPosition, const int tKeyPoint,
    float tSpeed, uint8_t tGear) {
    float preview_dis = 2.0;  // 预瞄距离设置的是固定的2m
    int nearest_id = 0;
    int preview_p_index = 10;
    double turning_radius = max_turning_R_;
    double abs_exp_steering = 0.0;
    float steering_ang = 0.0;
    double startpoint_id = 0;
    double wheel_base = 1.6;
    int path_size = tPathList.size();

    startpoint_id = FindNearestPoint2VehicleID(tPathList, tPosition);

    size_t valid_size = tPathList.size() - startpoint_id;
    // 末端不再有规划生成的路线外延长点，少于 10 个前方点也需继续跟踪真实末点。
    if(valid_size < 2) {
        return 0.0;
    }

    XYZ_COOR_S temp{};
    vector<XYZ_COOR_S> localPath;
    // 转换为局部坐标（车体）
    for(int i = startpoint_id; i < path_size; i++) {
        XYZ_COOR_S temp_point =
            global2local(tPosition.x_axis, tPosition.y_axis, tPosition.heading,
                         tPathList[i].x_axis, tPathList[i].y_axis,
                         tPathList[i].heading, temp.x_axis, temp.y_axis,
                         temp.heading);

        localPath.emplace_back(temp_point);
    }
    tPosition.x_axis = 0.0;
    tPosition.y_axis = 0.0;
    nearest_id = FindNearestPoint2VehicleID(localPath, tPosition);

    preview_dis = GetPreviewDistance(tSpeed, tGear);  // speed: m/s
    preview_p_index = FindPreviewPointOnPath(localPath, preview_dis, nearest_id);

    if(hypot(localPath[preview_p_index].x_axis, localPath[preview_p_index].y_axis) < 1e-6)
        return 0.0;

    turning_radius = GetTurningRadiusByPosAndHeading(localPath, preview_p_index);

    if(localPath[preview_p_index].y_axis < 0.0)  // 车前为X，车right为Y+.
        abs_exp_steering = GetDesiredSteeringAng(-wheel_base, turning_radius);
    else
        abs_exp_steering = GetDesiredSteeringAng(wheel_base, turning_radius);

    steering_ang = abs_exp_steering;

    return steering_ang;
}

unsigned int GeometricConstrol::FindNearestPoint2VehicleID(
    vector<XYZ_COOR_S> lpath, XYZ_COOR_S tPosition) {
    unsigned int nearest_id = 0;
    float mini_dis = 1000000.0;
    unsigned int path_num = lpath.size();

    if(path_num < 1) return 0;

    for(unsigned int i = 0; i < path_num; i++) {
        float temp_dis = (lpath[i].x_axis - tPosition.x_axis) *
                             (lpath[i].x_axis - tPosition.x_axis) +
                         (lpath[i].y_axis - tPosition.y_axis) *
                             (lpath[i].y_axis - tPosition.y_axis);

        if(temp_dis < mini_dis) {
            mini_dis = temp_dis;
            nearest_id = i;
        }
    }
    return nearest_id;
}

float GeometricConstrol::GetPreviewDistance(float speed, uint8_t tGear) {
    float d = 0.0;
    float min_preview_d = 3.0;

    if(speed <= 8.33333333) d = 1.5 * speed + min_preview_d;

    d = d > min_preview_d ? d : min_preview_d;

    if(tGear == 3) d = 3.0;

    return d;
}

unsigned int GeometricConstrol::FindPreviewPointOnPath(
    vector<XYZ_COOR_S> lpath, float d, int nearest_id) {
    unsigned int index = 0;
    unsigned int path_num = lpath.size();
    float preview_dis = d;
    double s_point = 0;

    if(path_num == 0) return 0;
    nearest_id = std::max(0, std::min(nearest_id, static_cast<int>(path_num) - 1));

    lpath[0].p2pDistance = 0;

    for(int i = 1; i < path_num; i++) {
        double dx = lpath[i].x_axis - lpath[i - 1].x_axis;
        double dy = lpath[i].y_axis - lpath[i - 1].y_axis;
        s_point += std::sqrt(dx * dx + dy * dy);

        lpath[i].p2pDistance = s_point;
    }
    for(index = nearest_id + 1; index < path_num; index++) {
        if((lpath[index].p2pDistance - lpath[nearest_id].p2pDistance) >=
           preview_dis) {
            break;
        }
    }

    // 剩余路径不足前视距离时选真实末点；不再减 4/5，避免短路径无符号下溢。
    if(index >= path_num) index = path_num - 1;

    nearest_p_.x_axis = lpath[nearest_id].x_axis;
    nearest_p_.y_axis = lpath[nearest_id].y_axis;
    nearest_p_.heading = lpath[nearest_id].heading;
    nearest_p_.p2pDistance = lpath[nearest_id].p2pDistance;

    return index;
}

double GeometricConstrol::GetTurningRadiusByPosAndHeading(
    std::vector<XYZ_COOR_S> lpath, int prev_id) {
    double c_c_x = 0.0;
    double c_c_y1 = 0.0;
    double R2 = max_turning_R_;
    double R = max_turning_R_;

    double min_steer_R = 2.0;

    XYZ_COOR_S prev_p;

    if(prev_id > lpath.size()) prev_id = lpath.size() - 4;

    prev_p.x_axis = lpath[prev_id].x_axis;
    prev_p.y_axis = lpath[prev_id].y_axis;
    prev_p.p2pDistance = lpath[prev_id].p2pDistance;
    prev_p.heading = lpath[prev_id].heading;

    // 目标在纵向轴上应直行；虚构 5 cm 横向偏差会在真实短末端放大成急转向。
    if(fabs(prev_p.y_axis) < 1e-6) return max_turning_R_;
    // 计算预瞄点的航向
    double desired_r = CalculateLineDirection(prev_p);

    if(fabs(desired_r) < very_less_dir_) return max_turning_R_;

    c_c_x = 0.0;
    c_c_y1 = (double)(((0.0 - prev_p.x_axis) * (0.0 - prev_p.x_axis) +
                       prev_p.y_axis * prev_p.y_axis - 0.0 * 0.0) /
                      (2.0 * prev_p.y_axis - 2.0 * 0.0));
    R2 = GetLength(c_c_x, c_c_y1, prev_p.x_axis, prev_p.y_axis);
    R = R2;

    double dis_err = GetLength(nearest_p_.x_axis, nearest_p_.y_axis, 0.0, 0.0);
    double cof = 10;
    double delta_r = cof * dis_err;

    if(delta_r > 10) delta_r = 10;

    if(R < 100.0) {
        if(prev_p.y_axis > 0.0) {
            if(nearest_p_.y_axis > 0.0)
                R = R - delta_r;
            else if(nearest_p_.y_axis < 0.0)
                R = R + delta_r;
        } else if(prev_p.y_axis < 0.0) {
            if(nearest_p_.y_axis > 0.0)
                R = R + delta_r;
            else if(nearest_p_.y_axis < 0.0)
                R = R - delta_r;
        }
    }

    if(R < min_steer_R)
        R = min_steer_R;
    else if(R > max_turning_R_)
        R = max_turning_R_;

    return R;
}

double GeometricConstrol::CalculateLineDirection(XYZ_COOR_S p2) {
    return atan2((double)p2.y_axis, (double)p2.x_axis) * 180.0 / M_PI;  // degree.
}

double GeometricConstrol::GetDesiredSteeringAng(double L, double r) {
    double steering_ang = 0.0;
    double max_steering = 40;
    double min_steering = -40;
    if(r == max_turning_R_)
        steering_ang = 0.0;
    else
        steering_ang = (atan2(L, r) * 180.0 / M_PI);

    if(steering_ang > max_steering)
        steering_ang = max_steering;
    else if(steering_ang < min_steering)
        steering_ang = min_steering;
    return steering_ang;  // Steering angle, not front wheel angle.
}

double GeometricConstrol::GetLength(double x1, double y1, double x2,
                                    double y2) {
    return std::hypot(x1 - x2, y1 - y2);
}

XYZ_COOR_S GeometricConstrol::global2local(double ox, double oy,
                                           double oheading, double gx, double gy, double gheading, double lx,
                                           double ly, double lheading) {
    XYZ_COOR_S temp{};
    double dx = gx - ox;
    double dy = gy - oy;
    oheading = 360 - oheading;
    double rad = oheading * M_PI / 180.0;
    double cosd = cos(rad);
    double sind = sin(rad);
    // rotate
    lx = cosd * dx + sind * dy;
    ly = -sind * dx + cosd * dy;
    lheading = gheading - oheading;  // degree
    temp.x_axis = ly;
    temp.y_axis = lx;
    temp.heading = lheading;
    return temp;
}
