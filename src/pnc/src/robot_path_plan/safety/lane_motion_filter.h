#ifndef LANE_MOTION_FILTER_H
#define LANE_MOTION_FILTER_H

#include <cmath>
#include "robot/object.h"

namespace planning_perception {
    enum MOTION_DIRECTION_E { UNDETERMINED = 0,
                              SAME_DIRECTION,
                              ONCOMING };

    // 一帧只计算一次行驶方向；vx/vy 是地图绝对速度，不能减去本车速度。
    class LaneMotionFilter {
    public:
        LaneMotionFilter() = default;
        explicit LaneMotionFilter(double tTravelHeading) {
            if(!std::isfinite(tTravelHeading)) return;
            const double angle = std::remainder(tTravelHeading, 360.0) * 3.14159265358979323846 / 180.0;
            mUx = std::sin(angle);
            mUy = std::cos(angle);
            mValid = true;
        }

        MOTION_DIRECTION_E Classify(const robot::object& tObject) const {
            if(!mValid || !std::isfinite(tObject.vx) || !std::isfinite(tObject.vy)) return UNDETERMINED;
            const double vx = tObject.vx, vy = tObject.vy;
            const double squared_speed = vx * vx + vy * vy;
            // 0.2 m/s 与上游跟踪器的静止阈值一致；异常速度留给原输入校验。
            if(squared_speed < 0.2 * 0.2 || squared_speed > 35.0 * 35.0) return UNDETERMINED;
            const double along = vx * mUx + vy * mUy;
            const double across = -vx * mUy + vy * mUx;
            // 同/对向各取 45 度锥内，横穿及斜穿方向不剔除。
            if(std::abs(along) <= std::abs(across)) return UNDETERMINED;
            return along > 0 ? SAME_DIRECTION : ONCOMING;
        }

        bool Ignore(const robot::object& tObject) const {
            return tObject.type == 1 && Classify(tObject) == ONCOMING;
        }

    private:
        double mUx = 0, mUy = 0;
        bool mValid = false;
    };
}

#endif  // LANE_MOTION_FILTER_H
