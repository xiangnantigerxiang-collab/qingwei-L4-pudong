
#include "common/math/math_utils.h"
#include "common/config/config.h"
#include "reference_line/reference_line_provider.h"
#include "common/curve1d/cubic_spline.h"

namespace planning
{
    AnchorPoint ReferenceLineProvide::GetAnchorPoint(
            const planning::ReferenceLine &reference_line, double s,
            const std::vector<double> &accumulateS) const
    {
        double longitudinalBound = FLAGS_longitudinal_boundary_bound;
        double lateralBound = FLAGS_lateral_boundary_bound;

        AnchorPoint anchor_point;
        anchor_point.longitudinal_bound = longitudinalBound;
        anchor_point.lateral_bound = lateralBound;
        anchor_point.abs_s = s;
        ReferencePoint ref_point = reference_line.getReferencePoint(s);

        anchor_point.pointInfo = ref_point.pointInfo();
        anchor_point.xds = ref_point.xds();
        anchor_point.yds = ref_point.yds();
        anchor_point.xseconds = ref_point.xsenconds();
        anchor_point.yseconds = ref_point.ysenconds();

        return anchor_point;
    }

    void ReferenceLineProvide::GetAnchorPoints(
            const ReferenceLine &reference_line,
            std::vector<AnchorPoint> *anchor_points)
    {
        const std::vector<ReferencePoint> &referencePoints = reference_line.referencePoints();
        const std::vector<double> &accumulateS = reference_line.accumulateS();

        const double interval = FLAGS_max_point_interval;
        int num_of_anchor = std::max(2, static_cast<int>(reference_line.length() / interval + 0.1));
        std::vector<double> anchor_s;
        math::uniform_slice(0.0,reference_line.length(),num_of_anchor - 1,&anchor_s);
        for (const double s : anchor_s) {
            anchor_points->emplace_back(GetAnchorPoint(reference_line, s, accumulateS));
            if(std::isnan(anchor_points->back().pointInfo.x()) ||
               std::isnan(anchor_points->back().pointInfo.y()))
            {
                anchor_points->pop_back();
            }
        }
    }

    std::vector<OriginalInsData> ReferenceLineProvide::smoothReferenceLine(const ReferenceLine &raw_reference_line, ReferenceLine *reference_line)
    {
        std::vector<AnchorPoint> anchor_points;

        GetAnchorPoints(raw_reference_line, &anchor_points);

        QpSplineReferenceLineSmooth smoother_;
        smoother_.setAnchorPoints(anchor_points);

        auto point_ = smoother_.smooth(raw_reference_line,reference_line);
        return point_;
    }
}
