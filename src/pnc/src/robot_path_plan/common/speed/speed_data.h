
#pragma once

#include <string>
#include <vector>

#include "common/pnc_point/speed_point.h"

namespace planning {
    class SpeedData : std::vector<SpeedPoint> {
    public:
        SpeedData() = default;

        virtual ~SpeedData() = default;

        explicit SpeedData(std::vector<SpeedPoint> speed_points);

        void appendSpeedPoint(const double s, const double time, const double v,
                              const double a, const double da);

        bool evaluateByTime(const double time,
                            SpeedPoint *const speedPoint) const;

        double totalTime() const;
    };

}  // namespace planning
