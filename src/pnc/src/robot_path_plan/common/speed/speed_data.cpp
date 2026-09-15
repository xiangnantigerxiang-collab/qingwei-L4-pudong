
#include "common/speed/speed_data.h"

#include <algorithm>
#include <utility>
#include <vector>
#include "common/line/linear_interpolation.h"

namespace planning {
    SpeedData::SpeedData(std::vector<SpeedPoint> speed_points)
        : std::vector<SpeedPoint>(std::move(speed_points)) {
        std::sort(begin(), end(), [](const SpeedPoint &p1, const SpeedPoint &p2) { return p1.t() < p2.t(); });
    }

    void SpeedData::appendSpeedPoint(
        const double s, const double time,
        const double v, const double a, const double da) {
        SpeedPoint point(s, time, v, a, da);
        push_back(point);
    }

    bool SpeedData::evaluateByTime(
        const double t, SpeedPoint *const speedPoint) const {
        if(size() < 2) return false;

        if(!(front().t() < t + 1.0e-6 && t - 1.0e-6 < back().t()))
            return false;

        auto comp = [](const SpeedPoint &sp, const double t) { return sp.t() < t; };

        auto it_lower = std::lower_bound(begin(), end(), t, comp);

        if(it_lower == end())
            *speedPoint = back();
        else if(it_lower == begin())
            *speedPoint = front();
        else {
            const auto &p0 = *(it_lower - 1);
            const auto &p1 = *it_lower;
            double t0 = p0.t();
            double t1 = p1.t();

            SpeedPoint res;
            res.setT(t);

            double s = math::lerp(p0.s(), t0, p1.s(), t1, t);
            res.setS(s);

            if(p0.v() && p1.v()) {
                double v = math::lerp(p0.v(), t0, p1.v(), t1, t);
                res.setV(v);
            }

            if(p0.a() && p1.a()) {
                double a = math::lerp(p0.a(), t0, p1.a(), t1, t);
                res.setA(a);
            }

            if(p0.da() && p1.da()) {
                double da = math::lerp(p0.da(), t0, p1.da(), t1, t);
                res.setDa(da);
            }

            *speedPoint = res;
        }

        return true;
    }

    double SpeedData::totalTime() const {
        if(empty()) return 0.0;

        return back().t() - front().t();
    }

}  // namespace planning
