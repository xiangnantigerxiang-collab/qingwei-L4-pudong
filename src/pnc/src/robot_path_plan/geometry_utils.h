#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include <math.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/segment.hpp>

namespace bg = boost::geometry;
using GeoPoint = bg::model::d2::point_xy<double>;
using GeoSegment = bg::model::segment<GeoPoint>;
using GeoPolygon = bg::model::polygon<GeoPoint>;
using GeoBox = bg::model::box<GeoPoint>;

namespace geometry_utils
{
    inline GeoPolygon createOBB(double cx, double cy, double length, double width, double heading)
    {
        GeoPolygon poly;
        double half_l = length / 2.0;
        double half_w = width / 2.0;
        double cos_h = std::cos(heading);
        double sin_h = std::sin(heading);

        // 四个角点（局部坐标）
        std::vector<GeoPoint> local_corners = {
            GeoPoint(-half_l, -half_w), // 左下
            GeoPoint(half_l, -half_w),  // 右下
            GeoPoint(half_l, half_w),   // 右上
            GeoPoint(-half_l, half_w)   // 左上
        };

        // 变换到世界坐标
        for (const auto &corner : local_corners)
        {
            double px = cx + corner.x() * cos_h - corner.y() * sin_h;
            double py = cy + corner.x() * sin_h + corner.y() * cos_h;
            // printf("px:%f, py:%f\n", px, py);
            poly.outer().emplace_back(px, py);
        }
        poly.outer().push_back(poly.outer().front()); // 闭合
        bg::correct(poly);                            // 确保方向正确
        return poly;
    }

    inline GeoBox createAABB(const GeoPolygon &obb)
    {
        GeoBox aabb;
        bg::envelope(obb, aabb);
        return aabb;
    }
    inline double distanceOBB(const GeoPolygon &poly1, const GeoPolygon &poly2)
    {
        // 1. 检查是否相交
        if (bg::intersects(poly1, poly2))
        {
            return 0.0;
        }
        double min_dist = std::numeric_limits<double>::max();

        // 2. 提取所有边（线段）
        auto get_segments = [](const GeoPolygon &poly) -> std::vector<GeoSegment>
        {
            std::vector<GeoSegment> segs;
            const auto &ring = poly.outer();
            for (size_t i = 0; i < ring.size() - 1; ++i)
            {
                segs.emplace_back(ring[i], ring[i + 1]);
            }
            return segs;
        };

        // 3. 提取所有顶点
        auto get_points = [](const GeoPolygon &poly) -> std::vector<GeoPoint>
        {
            return poly.outer();
        };

        auto segs1 = get_segments(poly1);
        auto segs2 = get_segments(poly2);
        auto pts1 = get_points(poly1);
        auto pts2 = get_points(poly2);

        // 4. 计算点到线段的距离（16 种组合）
        for (const auto &pt : pts1)
        {
            for (const auto &seg : segs2)
            {
                double dist = bg::distance(pt, seg);
                min_dist = std::min(min_dist, dist);
            }
        }
        for (const auto &pt : pts2)
        {
            for (const auto &seg : segs1)
            {
                double dist = bg::distance(pt, seg);
                min_dist = std::min(min_dist, dist);
            }
        }

        return min_dist;
    }

    inline double distanceAABB(const GeoBox &box1, const GeoBox &box2)
    {
        double min_dist = bg::distance(box1, box2);
        return min_dist;
    }
}

#endif