#ifndef GIS_UTILS_H
#define GIS_UTILS_H

#include <cmath>

#define BASE_GAUSS_X 456817.65
#define BASE_GAUSS_Y 4132154.06
namespace gis_utils
{
    /**
     * 方位角(0,360)(与正北夹角，顺时针为正)转航向角(-pi,pi)(与正东夹角，逆时针为正)
     */
    double azimuthToYaw(const double &azimuth)
    {
        // 转为与正东夹角，逆时针为正
        double angle = 90 - azimuth;

        if (angle < -180)
        {
            angle += 360;
        }
        // 转为弧度
        double rad_angle = angle / 180 * M_PI;
        return rad_angle;
    }

    /**
     * 航向角转方位角
     */
    double yawToAzimuth(const double &yaw)
    {
        double deg_angle = yaw / M_PI * 180;
        deg_angle = 90 - deg_angle;
        if (deg_angle < 0)
        {
            deg_angle = 360 + deg_angle;
        }
        return deg_angle;
    }

    /**
     * 将弧度角度转为(-pi, pi]
     */
    double mod2Pi(const double &rad_angle)
    {
        double D_M_PI = 2 * M_PI;
        double angle = fmod(rad_angle, D_M_PI); // 将角度控制在（0，2*pi）
        if (angle > M_PI)
        {
            angle -= D_M_PI;
        }
        else if (angle < M_PI)
        {
            angle += D_M_PI;
        }
        return angle;
    }
}
#endif