
#ifndef PLANNING_REFERENCE_LINE_PROVIDER_H
#define PLANNING_REFERENCE_LINE_PROVIDER_H

#include <list>
#include "common/struct_type.h"
#include "qp_spline_corridor_reference_line_generation.h"

namespace planning
{
class ReferenceLineProvide
{
    public:
        ReferenceLineProvide() = default;
        ~ReferenceLineProvide() = default;

        std::vector<OriginalInsData> smoothReferenceLine(
                const ReferenceLine &raw_reference_line,
                ReferenceLine *reference_line);

    private:
        AnchorPoint GetAnchorPoint(
                const ReferenceLine& reference_line,
                double s,
                const std::vector<double> &accumulateS) const;

        void GetAnchorPoints(
                const ReferenceLine &reference_line,
                std::vector<AnchorPoint> *anchor_points);
    };
}

#endif //PLANNING_REFERENCE_LINE_PROVIDER_H
