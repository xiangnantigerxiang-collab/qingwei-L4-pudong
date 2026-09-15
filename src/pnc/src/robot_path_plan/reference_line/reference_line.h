#ifndef _PLANNING_REFERENCE_LINE_H_
#define _PLANNING_REFERENCE_LINE_H_

#include <vector>

#include "reference_point.h"
#include "common/path/sl_boundary.h"
#include "common/surface/box2d.h"
#include "common/pnc_point/sl_point.h"

namespace planning {
    class ReferenceLine {
    public:
        ReferenceLine();

        ReferenceLine(const std::vector<planning::ReferencePoint> &reference_points,
                      const std::vector<double> &accumulate_s);

        const std::vector<ReferencePoint> &referencePoints() const;

        std::vector<ReferencePoint> &mutableReferencePoints();

        const std::vector<LinkLaneSegment> &linkLaneSegments() const;

        const std::vector<double> &accumulateS() const;

        const ReferencePoint &getNearstPoint(const double s) const;

        ReferencePoint getReferencePoint(const double s) const;

        ReferencePoint getReferencePoint2(const double s) const;

        ReferencePoint interpolateLinearApproximation(const ReferencePoint &p0,
                                                      const double s0,
                                                      const ReferencePoint &p1,
                                                      const double s1,
                                                      const double s) const;

        int getNearstPointLocation(const double s) const;

        bool getNearstPoint(const had_map::MapPoint &point, int &location) const;

        bool getNearstPoint(const ReferencePoint &point, int &location, const double &minDis, bool printDis = false) const;

        bool getNearstPoint(const math::Vec2d &point, int &location, double &minDis) const;

        bool getNearstPoint(const math::Vec2d &point, int &location, const double startS, const double endS, const double &minDis) const;

        bool getNearstPoint(const had_map::MapPoint &point, ReferencePoint &reference_point) const;

        bool getLeftAndRightWidth(const had_map::MapPoint point, double &left_width, double &right_width) const;

        bool getLeftAndRightWidth(const double s, double &left_width, double &right_width) const;

        const LinkLaneSegment &getLinkLaneByS(const double s) const;

        bool getSpeed(const had_map::MapPoint &point, double &maxSpeed, double &minSpeed) const;

        void removePointFromIndex(const int &start, const int &end);

        void removePointFromS(const double &start_s, const double &end_s);

        void connectOtherReference(ReferenceLine *ref_line, int &start);

        bool Shrink(const math::Vec2d &point, SLPoint &slPoint,
                    const double look_backward, const double look_forward);

        bool SLToXY(const SLPoint &sl_point,
                    math::Vec2d *const xy_point) const;

        bool XYToSL(const math::Vec2d &point, SLPoint &slPoint) const;

        bool XYToSL(const had_map::MapPoint &point, SLPoint &slPoint) const;

        bool XYToSL(const math::Vec2d &vec2d, const double startS, const double endS, SLPoint &slPoint) const;

        bool getSLBoundary(const math::Box2d &box,
                           SLBoundary *const sl_boundary) const;

        bool isBlockRoad(const math::Box2d &box, double gap) const;

        double distanceToBox(const math::Box2d &box2d) const;

        bool IsOnRoad(const SLBoundary &sl_boundary) const;

        bool IsOnRoad(const SLPoint &sl_point) const;

        bool IsOnRoad(const math::Vec2d &point) const;

        void swapPoint(const ReferencePoint &point, uint32_t index);

        void testPrintS() const;

        const double length() const;

        const double spacingDis() const;

        void setSpacingDis(const double &dis);

        bool addOnePointInBack(ReferencePoint &referencePoint);

        bool addOnePointInBegin(ReferencePoint &referencePoint);

        void setSommthFlag(bool smooth_flag);

        bool smoothFlag();

        void setLaneId(const int &laneID);

        int getLaneId() const;

        bool getApproximateSLBoundary(const math::Box2d &box, const double startS, const double endS, SLBoundary *const sl_boundary) const;

        bool includeLinkLane(const LinkLaneSegment &linkLaneSegment) const;

        void reCalculateSegments();

        void reCalculateS();

        void reCalculateK();

        //相同link不同lane返回0；相同link相同lane返回2，不同link，不同lane返回1
        int judgeRoadIncludeLane1(const LinkLaneSegment &linkLaneSegment) const;

        //包含返回true，不包含返回false
        bool judgeRoadIncludeLane2(const LinkLaneSegment &linkLaneSegment) const;

        void clear();

        std::vector<ReferencePoint> reference_points_;
        std::vector<double> accumulate_s_;

    private:
        void RemoveDuplicates();

        double distanceTwoPoints(ReferencePoint &point1, ReferencePoint &point2) const;

        std::vector<LinkLaneSegment> m_linkLaneSegments;

        double spacing_dis_;

        bool smooth_flag_;

        int m_laneId;
    };
}

#endif  //_PLANNING_REFERENCE_LINE_H_
