#ifndef HDMAP_PNC_ADAPTER_H
#define HDMAP_PNC_ADAPTER_H

#include "hdmap/hdmap_types.h"
#include <cmath>
#include <limits>

namespace hdmap
{
// 模板参数在 PNC 侧传 XYZ_COOR_S，SDK 本身不包含 ROS 或 PNC 私有头文件。
// 逐字段赋值避免假设两个结构体内存布局一致，也避免把曲率写进 z_axis。
template <typename T>
STATUS_S ConvertToPncPath(const MapPointList& tInput, std::vector<T>& tOutput)
{
    std::vector<T> points;
    points.reserve(tInput.size());
    const double float_limit = std::numeric_limits<float>::max();
    for (std::size_t i = 0; i < tInput.size(); ++i)
    {
        const MAP_POINT_S& point = tInput[i];
        const double values[] = {point.x_axis, point.y_axis, point.heading,
            point.curvature, point.dist_origin, point.p2pDistance};
        for (std::size_t j = 0; j < sizeof(values) / sizeof(values[0]); ++j)
        {
            if (!std::isfinite(values[j]) || std::abs(values[j]) > float_limit)
            {
                return STATUS_S(INVALID_DATA, "地图点无法转换为 PNC float 字段");
            }
        }
        if (point.heading < 0.0 || point.heading >= 360.0 || point.curvature < 0.0 ||
            point.dist_origin < 0.0 || point.p2pDistance < 0.0)
        {
            return STATUS_S(INVALID_DATA, "地图点不符合 PNC 航向或距离约定");
        }
        T xyz_temp;
        xyz_temp.x_axis = static_cast<float>(point.x_axis);
        xyz_temp.y_axis = static_cast<float>(point.y_axis);
        xyz_temp.z_axis = 0.0f;
        xyz_temp.heading = static_cast<float>(point.heading);
        // double 接近 360 时转 float 可能舍入成 360，仍保持 [0, 360)。
        if (xyz_temp.heading >= 360.0f) xyz_temp.heading = 0.0f;
        xyz_temp.p2pDistance = static_cast<float>(point.p2pDistance);
        xyz_temp.curvature = static_cast<float>(point.curvature);
        xyz_temp.velocity = 0.0f; // 速度由 PNC 任务层赋值，地图不决定车速。
        xyz_temp.dist_origin = static_cast<float>(point.dist_origin);
        points.push_back(xyz_temp);
    }
    tOutput.swap(points);
    return STATUS_S();
}
}

#endif
