

#pragma once

#include <vector>

#include "common/pnc_point/path_point.h"

namespace planning
{
    class DiscretizedPath : public std::vector<PathPoint>
    {
    public:
        DiscretizedPath() = default;

        explicit DiscretizedPath(const std::vector<PathPoint> &path_points);

        double length() const;

        PathPoint evaluate(const double path_s) const;

        PathPoint evaluateReverse(const double path_s) const;

    protected:
        std::vector<PathPoint>::const_iterator QueryLowerBound(
                const double path_s) const;
        std::vector<PathPoint>::const_iterator QueryUpperBound(
                const double path_s) const;
    };

}  // namespace planning

