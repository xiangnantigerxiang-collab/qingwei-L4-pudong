
#ifndef PLANNING_REFERENCE_POINT_H
#define PLANNING_REFERENCE_POINT_H

#include "common/pnc_point/map_point.h"

namespace planning
{
    // 当前引导线左边marking类型 */
    // 0 Not investigated 10 Single Dashed
    // 12 Short Thick Dashed
    // 13 Double Dashed
    // 20 Single Solid
    // 21 Double Solid
    // 30 Left Solid/Right Dashed
    // 31 Left Dashed/Right Solid
    // 32 Turn variable lane marking
    // 33 Single thick solid
    // 99 Virtual Marking
    struct LinkLaneSegment
    {
        long long absLinkId;
        int laneId;
        int leftMarkingType;
        int rightMarkingType;

        LinkLaneSegment()
        {
            absLinkId = -1;
            laneId = -1;
            leftMarkingType = 20;
            rightMarkingType = 20;
        }

        LinkLaneSegment(const long long linkId,
                        const int lane)
        {
            absLinkId =linkId;
            laneId = lane;
        }

        LinkLaneSegment(const long long linkId,
                        const int lane,const int leftType,
                        const int rightType)
        {
            absLinkId =linkId;
            laneId = lane;
            leftMarkingType = leftType;
            rightMarkingType = rightType;
        }

        void print() const
        {
            printf("linkId = %lld, laneId = %d\r\n",absLinkId, laneId);
        }

        bool operator==(const LinkLaneSegment &other) const
        {
            return (absLinkId == other.absLinkId && laneId == other.laneId);
        }

        bool operator!=(const LinkLaneSegment &other) const
        {
            return !(absLinkId == other.absLinkId && laneId == other.laneId);
        }
    };

    class ReferencePoint
    {
    public:
        ReferencePoint() = default;

        ReferencePoint(const had_map::MapPoint &point_info, const double &left_width,
                       const double &rightWidth, const double &maxSpeed,
                       const double &minSpeed,const long long linkId, const int laneId,
                       const int leftMarkingType, const  int rightMarkingType);

//        ReferencePoint(const had_map::MapPoint &point_info, const double kappa,
//                      const double dkappa);

        ReferencePoint(const had_map::MapPoint &point_info, const double &kappa,
                       const double &dkappa, const double &xds,
                       const double &yds, const double &xsenconds,
                       const double &ysenconds, const double &left_width,
                       const double &right_width);

        ReferencePoint(const had_map::MapPoint &point_info, const double &kappa,
                       const double &dkappa, const double &xds,
                       const double &yds, const double &xsenconds,
                       const double &ysenconds, const double &left_width,
                       const double &right_width,const double &limit_max_speed,
                       const double &limit_min_speed,const LinkLaneSegment &linkLaneSegment1);

        ReferencePoint(const had_map::MapPoint &point_info,const double &left_width,
                       const double &right_width);

        double xds()const;

        double yds()const;

        double xsenconds()const;

        double ysenconds()const;

        void setKappa(const double k);

        double kappa()const;

        void setDkappa(const double dkappa);

        double dkappa()const;

        const had_map::MapPoint &pointInfo() const;

        had_map::MapPoint &mutablePointInfo();

        double leftWidth()const;

        double rightWidth()const;

        double limitMaxSpeed()const;

        double limitMinSpeed()const;

        const LinkLaneSegment &linkLaneSegment() const;

    private:
        had_map::MapPoint m_point;

        double m_kappa = 0;

        double m_dkappa = 0;

        double m_xds = 0;

        double m_yds = 0;

        double m_xseconds = 0;

        double m_yseconds = 0;

        double m_leftWidth = 0;           //dis from point to left boundary

        double m_rightWidth = 0;          //dis from point to right boundary

        double m_limitMaxSpeed = 120.0;

        double m_limitMinSpeed = 0.0;

        LinkLaneSegment m_linkLaneSegment;
    };
} //namespace planning

#endif //PLANNING_REFERENCE_POINT_H
