
#ifndef PLANNING_MAP_POINT_H
#define PLANNING_MAP_POINT_H

#include "common/surface/vec2d.h"

using namespace math;

namespace had_map
{
    class MapPoint : public Vec2d
    {
    public:
        MapPoint() = default;

        MapPoint(const Vec2d& point, const double heading)
                :Vec2d(point.x(), point.y()),m_heading(heading){}

        MapPoint(const Vec2d& point, const double heading, const int type)
                :Vec2d(point.x(), point.y()),m_heading(heading),m_type(type){}

        double heading() const {return m_heading;}
        void setHeading(const double heading) { m_heading = heading;}
        int type() const {return m_type;}

    private:
        double m_heading = 0.0;    //radian
        int m_type;
    };

}

#endif //PLANNING_MAP_POINT_H
