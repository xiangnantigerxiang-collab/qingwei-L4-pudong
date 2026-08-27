
#include <algorithm>
#include "common/path/discretized_path.h"
#include "common/line/linear_interpolation.h"

namespace planning
{
    using planning::PathPoint;

    DiscretizedPath::DiscretizedPath(const std::vector<PathPoint> &path_points)
            : std::vector<PathPoint>(path_points) {}

    double DiscretizedPath::length() const
    {
        if (empty())
        {
            return 0.0;
        }

        return back().s() - front().s();
    }

    PathPoint DiscretizedPath::evaluate(const double path_s) const
    {
        //CHECK(!empty());
        auto it_lower = QueryLowerBound(path_s);

        if (it_lower == begin())
        {
            return front();
        }

        if (it_lower == end())
        {
            return back();
        }

        return math::InterpolateUsingLinearApproximation(*(it_lower - 1),
                                                                 *it_lower, path_s);
    }

    std::vector<PathPoint>::const_iterator DiscretizedPath::QueryLowerBound(
            const double path_s) const
    {
        auto func = [](const PathPoint & tp, const double path_s)
        {
            return tp.s() < path_s;
        };
        return std::lower_bound(begin(), end(), path_s, func);
    }

    PathPoint DiscretizedPath::evaluateReverse(const double path_s) const
    {
        //CHECK(!empty());
        auto it_upper = QueryUpperBound(path_s);

        if (it_upper == begin())
        {
            return front();
        }

        if (it_upper == end())
        {
            return back();
        }

        return math::InterpolateUsingLinearApproximation(*(it_upper - 1),
                                                                 *it_upper, path_s);
    }

    std::vector<PathPoint>::const_iterator DiscretizedPath::QueryUpperBound(
            const double path_s) const
    {
        auto func = [](const double path_s, const PathPoint & tp)
        {
            return tp.s() < path_s;
        };
        return std::upper_bound(begin(), end(), path_s, func);
    }
}