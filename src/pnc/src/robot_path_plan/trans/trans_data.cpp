#include <cstring>
#include <iostream>

#include "trans_data.h"
#include "proj4/proj_api.h"
#include "common/pnc_point/map_point.h"
#include "common/surface/vec2d.h"
#include "common/math/math_utils.h"
#include "reference_line/reference_point.h"

namespace planning {

    bool TransData::createReferenceLine(
        planning::ReferenceLine &referenceLine,
        std::vector<OriginalInsData> &originpoint) {
        std::vector<GaussData> raw_reference_line;

        ImportData(raw_reference_line, originpoint);

        double laneWidth = 3.5;

        std::vector<ReferencePoint> referencePoints;
        std::vector<double> accumulateS;

        for(const GaussData &referencePoint : raw_reference_line) {
            had_map::MapPoint mapPoint(
                math::Vec2d{referencePoint.x, referencePoint.y},
                degreeToRadian(referencePoint.heading),
                referencePoint.type);

            ReferencePoint point(mapPoint, laneWidth / 2.0, laneWidth / 2.0);

            accumulateS.emplace_back(referencePoint.s);

            referencePoints.emplace_back(point);
        }

        ReferenceLine line(referencePoints, accumulateS);
        referenceLine = line;

        return true;
    }

    bool TransData::ImportData(
        std::vector<GaussData> &raw_reference_line,
        std::vector<OriginalInsData> &originpoint) {
        uint16_t iNum = originpoint.size();
        GaussData InputData;

        double sumS = 0.0;

        for(uint16_t i = 0; i < iNum; ++i) {
            InputData.x = originpoint[i].x;
            InputData.y = originpoint[i].y;
            InputData.heading = originpoint[i].heading;
            InputData.type = 1;
            InputData.z = 0.000;
            InputData.s = 0.000;

            if(i > 0) {
                sumS += sqrt((InputData.x - raw_reference_line[i - 1].x) *
                                 (InputData.x - raw_reference_line[i - 1].x) +
                             (InputData.y - raw_reference_line[i - 1].y) *
                                 (InputData.y - raw_reference_line[i - 1].y));

                InputData.s = sumS;
            }

            raw_reference_line.emplace_back(InputData);
        }

        return true;
    }

}  //namespace planning
