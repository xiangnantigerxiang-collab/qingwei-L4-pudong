
#include "reference_line/reference_point.h"

namespace planning {
    ReferencePoint::ReferencePoint(const had_map::MapPoint &point_info, const double &left_width,
                                   const double &rightWidth, const double &maxSpeed,
                                   const double &minSpeed, const long long linkId, const int laneId,
                                   const int leftMarkingType, const int rightMarkingType)
        : m_point(point_info), m_leftWidth(left_width), m_rightWidth(rightWidth), m_limitMaxSpeed(maxSpeed), m_limitMinSpeed(minSpeed), m_linkLaneSegment({linkId, laneId, leftMarkingType, rightMarkingType}) {
    }

    ReferencePoint::ReferencePoint(const had_map::MapPoint &point_info, const double &kappa,
                                   const double &dkappa, const double &xds,
                                   const double &yds, const double &xsenconds,
                                   const double &ysenconds, const double &left_width,
                                   const double &right_width)
        : m_point(point_info), m_kappa(kappa), m_dkappa(dkappa), m_xds(xds), m_yds(yds), m_xseconds(xsenconds), m_yseconds(ysenconds), m_leftWidth(left_width), m_rightWidth(right_width) {
    }

    ReferencePoint::ReferencePoint(const had_map::MapPoint &point_info, const double &kappa,
                                   const double &dkappa, const double &xds,
                                   const double &yds, const double &xsenconds,
                                   const double &ysenconds, const double &left_width,
                                   const double &right_width, const double &limit_max_speed,
                                   const double &limit_min_speed, const LinkLaneSegment &linkLaneSegment1)
        : m_point(point_info), m_kappa(kappa), m_dkappa(dkappa), m_xds(xds), m_yds(yds), m_xseconds(xsenconds), m_yseconds(ysenconds), m_leftWidth(left_width), m_rightWidth(right_width), m_limitMaxSpeed(limit_max_speed), m_limitMinSpeed(limit_min_speed), m_linkLaneSegment(linkLaneSegment1) {
    }

    ReferencePoint::ReferencePoint(const had_map::MapPoint &point_info, const double &left_width,
                                   const double &right_width)
        : m_point(point_info),
          m_leftWidth(left_width),
          m_rightWidth(right_width) {
    }

    double ReferencePoint::xds() const {
        return m_xds;
    }

    double ReferencePoint::yds() const {
        return m_yds;
    }

    double ReferencePoint::xsenconds() const {
        return m_xseconds;
    }

    double ReferencePoint::ysenconds() const {
        return m_yseconds;
    }

    void ReferencePoint::setKappa(const double k) {
        m_kappa = k;
    }

    double ReferencePoint::kappa() const {
        return m_kappa;
    }

    void ReferencePoint::setDkappa(const double dkappa) {
        m_dkappa = dkappa;
    }

    double ReferencePoint::dkappa() const {
        return m_dkappa;
    }

    const had_map::MapPoint &ReferencePoint::pointInfo() const {
        return m_point;
    }

    had_map::MapPoint &ReferencePoint::mutablePointInfo() {
        return m_point;
    }

    double ReferencePoint::leftWidth() const {
        return m_leftWidth;
    }

    double ReferencePoint::rightWidth() const {
        return m_rightWidth;
    }

    double ReferencePoint::limitMaxSpeed() const {
        return m_limitMaxSpeed;
    }

    double ReferencePoint::limitMinSpeed() const {
        return m_limitMinSpeed;
    }

    const LinkLaneSegment &ReferencePoint::linkLaneSegment() const {
        return m_linkLaneSegment;
    }

}  //namespace planning