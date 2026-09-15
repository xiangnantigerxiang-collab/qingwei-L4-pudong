

#ifndef PLANNING_FRENET_FRAME_PATH_H
#define PLANNING_FRENET_FRAME_PATH_H

#include <vector>
#include <cstdint>
#include <common/path/sl_boundary.h>
#include "common/pnc_point/frenet_frame_point.h"

namespace planning {

    class FrenetFramePath {
    public:
        FrenetFramePath() = default;
        explicit FrenetFramePath(
            const std::vector<FrenetFramePoint> &sl_points);
        virtual ~FrenetFramePath() = default;

        void set_points(const std::vector<FrenetFramePoint> &points);
        const std::vector<FrenetFramePoint> &points() const;
        std::uint32_t NumOfPoints() const;
        double Length() const;
        const FrenetFramePoint PointAt(const std::uint32_t index) const;
        FrenetFramePoint EvaluateByS(const double s) const;

        /**
     * @brief Get the FrenetFramePoint that is within SLBoundary, or the one with
     * smallest l() in SLBoundary's s range [start_s(), end_s()]
     */
        FrenetFramePoint GetNearestPoint(const SLBoundary &sl) const;

        virtual void Clear();

    private:
        static bool LowerBoundComparator(const FrenetFramePoint &p,
                                         const double s) {
            return p.s() < s;
        }
        static bool UpperBoundComparator(const double s,
                                         const FrenetFramePoint &p) {
            return s < p.s();
        }

        std::vector<FrenetFramePoint> points_;
    };

}  // namespace planning

#endif  //PLANNING_FRENET_FRAME_PATH_H
