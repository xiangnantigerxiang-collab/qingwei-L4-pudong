#ifndef ADAPTIVEHOOK_H
#define ADAPTIVEHOOK_H
#include <iostream>
#include <vector>
#include <array>
#include <math.h>

namespace adaptive {

class AdaptiveHook
{
private:
    const float lidar_to_wheel_dis_c;
public:
    AdaptiveHook() :lidar_to_wheel_dis_c(0.49) {}
    ~AdaptiveHook() {}

    std::vector<std::array<float,3>> GenerateAdaptiveHookPath(
        const float cur_x, const float cur_y,const float cur_head,
        const float lidar_x, const float lidar_y, const float vehicle_angle, const float distance, const int pallettype);

private:
    float Trans2PItoPI(const float angle);
    float TransPIto2PI(const float angle);

    std::array<float,3> CalculNextPointByAngle(
        const float coor_x, const float coor_y, const float heading, const float distance);

    float CalculatePoint2PointDistance(
        float x1_coor, float y1_coor, float z1_coor,
        float x2_coor,float y2_coor,float z2_coor);

    std::vector<std::array<float,3>> GenerateBezierPath(
        const float start_x, const float start_y, const float start_angle,
        const float stop_x, const float stop_y, const float stop_angle);
};

}

#endif
