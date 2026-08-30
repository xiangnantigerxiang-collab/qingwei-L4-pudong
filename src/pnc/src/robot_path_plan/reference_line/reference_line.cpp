
#include <algorithm>
#include <iostream>
#include <common/line/linear_interpolation.h>

#include "common/math/math_utils.h"
#include "common/config/reference_line_config/smooth_reference_line_config.h"
#include "common/config/planner_config/planner_param.h"
#include "common/surface/box2d.h"
#include "reference_line/reference_line.h"

namespace planning
{
    namespace {
        constexpr double headingDiffCoefficient = 3.0;
        const double MAX_point_dis = 5.0;
    }
    ReferenceLine::ReferenceLine()
    {
        smooth_flag_ = false;
        spacing_dis_ = -1.0;
    }

    ReferenceLine::ReferenceLine(const std::vector<planning::ReferencePoint>& reference_points,
                                 const std::vector<double>& accumulate_s)
                                :reference_points_(reference_points),accumulate_s_(accumulate_s){}

    const std::vector<ReferencePoint> &ReferenceLine::referencePoints() const
    {
        return reference_points_;
    }

    std::vector<ReferencePoint> &ReferenceLine::mutableReferencePoints()
    {
        return reference_points_;
    }

    const std::vector<LinkLaneSegment> &ReferenceLine::linkLaneSegments() const
    {
        return m_linkLaneSegments;
    }

    const std::vector<double> &ReferenceLine::accumulateS() const
    {
        return accumulate_s_;
    }

    const ReferencePoint &ReferenceLine::getNearstPoint(const double s) const
    {
        const auto& accumulated_s = accumulate_s_;
        if (s < accumulated_s.front() - 1e-2) {
            std::cout << "warning: The requested s " << s << " < 0";
            return reference_points_.front();
        }
        if (s > accumulated_s.back() + 1e-2) {
            return reference_points_.back();
        }
        auto it_lower =
                std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);
        if (it_lower == accumulated_s.begin()) {
            return reference_points_.front();
        } 
        else if (it_lower == accumulated_s.end())
        {
            return reference_points_.back();
        }

        else {
            auto index = std::distance(accumulated_s.begin(), it_lower);
            if (std::fabs(accumulated_s[index - 1] - s) <
                std::fabs(accumulated_s[index] - s)) {
                return reference_points_[index - 1];
            } else {
                return reference_points_[index];
            }
        }
    }

    ReferencePoint ReferenceLine::interpolateLinearApproximation(const ReferencePoint &p0,
                                                  const double s0,
                                                  const ReferencePoint &p1,
                                                  const double s1,
                                                  const double s)const
    {
        double weight = (s - s0) / (s1 - s0);
        double x = (1 - weight) * p0.pointInfo().x() + weight * p1.pointInfo().x();
        double y = (1 - weight) * p0.pointInfo().y() + weight * p1.pointInfo().y();
        double theta = math::slerp(p0.pointInfo().heading(), s0, p1.pointInfo().heading(), s1, s);
        had_map::MapPoint point;
        point.set_x(x);
        point.set_y(y);
        point.setHeading(theta);
        double kappa = (1 - weight) * p0.kappa() + weight * p1.kappa();
        double dkappa = (1 - weight) * p0.dkappa() + weight * p1.dkappa();
        ReferencePoint path_point;
        path_point.mutablePointInfo() = point;
        path_point.setKappa(kappa);
        path_point.setDkappa(dkappa);
        return path_point;
    }

    ReferencePoint ReferenceLine::getReferencePoint(const double s) const
    {
        if (s < accumulate_s_.front() - 1e-2) {
            return reference_points_.front();
        }
        if (s > accumulate_s_.back() + 1e-2) {
            return reference_points_.back();
        }

        uint32_t index = getNearstPointLocation(s);
        if(index == reference_points_.size() - 1)
        {
            index = reference_points_.size() - 2;
        }
        uint32_t next_index = index + 1;

        const auto& p0 = reference_points_[index];
        const auto& p1 = reference_points_[next_index];

        const double s0 = accumulate_s_[index];
        const double s1 = accumulate_s_[next_index];

        return interpolateLinearApproximation(p0, s0, p1, s1, s);
    }

    ReferencePoint ReferenceLine::getReferencePoint2(const double s) const
    {
        if (s < accumulate_s_.front() - 1e-2) {
            return reference_points_.front();
        }
        if (s > accumulate_s_.back() + 1e-2) {
            return reference_points_.back();
        }

        uint32_t index = getNearstPointLocation(s);
        if(std::fabs(s - accumulate_s_[index]) < 0.1)
        {
            return reference_points_[index];
        }
        if(index == reference_points_.size() - 1)
        {
            index = reference_points_.size() - 2;
        }
        uint32_t next_index = index + 1;

        const auto& p0 = reference_points_[index];
        const auto& p1 = reference_points_[next_index];

        const double s0 = accumulate_s_[index];
        const double s1 = accumulate_s_[next_index];

        return interpolateLinearApproximation(p0, s0, p1, s1, s);
    }

    int ReferenceLine::getNearstPointLocation(const double s) const
    {
        const auto& accumulated_s = accumulate_s_;
        if (s < accumulated_s.front() - 1e-2) {
            std::cout << "warning: The requested s " << s << " < 0";
            return 0;
        }
        if (s > accumulated_s.back() + 1e-2) {
            std::cout << "warning: The requested s " << s << " > reference line length "
                 << accumulated_s.back();
            return reference_points_.size() - 1;
        }
        auto it_lower =
                std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);
        if (it_lower == accumulated_s.begin()) {
            return 0;
        } else {
            auto index = std::distance(accumulated_s.begin(), it_lower);
            if (std::fabs(accumulated_s[index - 1] - s) <
                std::fabs(accumulated_s[index] - s)) {
                return index - 1;
            } else {
                return index;
            }
        }
    }

    bool ReferenceLine::getNearstPoint(const had_map::MapPoint &point,ReferencePoint &reference_point) const
    {
        double min_distance = 3.0;
        bool find_point = false;
        for (auto &ref_point : reference_points_)
        {
            const double temp_dis = std::hypot((ref_point.pointInfo().x() - point.x()),(ref_point.pointInfo().y() - point.y()))
                                    + fabs(ref_point.pointInfo().heading() - point.heading()) * headingDiffCoefficient;
            if(temp_dis < min_distance)
            {
                min_distance = temp_dis;
                reference_point = ref_point;
                find_point = true;
            }
        }
        return find_point;
    }

    bool ReferenceLine::getNearstPoint(const had_map::MapPoint &point,int& location) const
    {
        double min_distance = 2;
        bool find_point = false;
        int i= 0;
        for (auto &ref_point : reference_points_)
        {
            const double temp_dis = std::hypot((ref_point.pointInfo().x() - point.x()),(ref_point.pointInfo().y() - point.y()));
            if(find_point && temp_dis > min_distance)
            {
                return find_point;
            }
            if(temp_dis < min_distance)
            {
                min_distance = temp_dis;
                location = i;
                find_point = true;
            }
            i++;
        }
        return find_point;
    }

    bool ReferenceLine::getNearstPoint(const ReferencePoint &point,int& location,
                                       const double &minDis, bool printDis) const
    {
        double min_distance = minDis + FLAGS_lateral_boundary_bound;
        bool find_point = false;
        int i= -1;
        for (const auto &ref_point : reference_points_)
        {
            i++;
            if(printDis)
            {
                printf("ref_point ");
                ref_point.linkLaneSegment().print();
                printf("point ");
                point.linkLaneSegment().print();
            }
            if(ref_point.linkLaneSegment() != point.linkLaneSegment())
            {
                continue;
            }
            const double temp_dis = std::hypot((ref_point.pointInfo().x() - point.pointInfo().x()),(ref_point.pointInfo().y() - point.pointInfo().y()));
            if(printDis)
            {
                printf("minDis = %f, tempdis = %f\r\n",min_distance, temp_dis);
            }
            if(find_point && temp_dis > min_distance)
            {
                return find_point;
            }
            if(temp_dis < min_distance)
            {
                min_distance = temp_dis;
                location = i;
                find_point = true;
            }
        }
        return find_point;
    }

    bool ReferenceLine::getNearstPoint(const math::Vec2d &point,int& location, double &minDis) const
    {
        double min_distance = minDis;
        bool find_point = false;
        int i= 0;
        for (auto &ref_point : reference_points_)
        {
            const double temp_dis = std::hypot((ref_point.pointInfo().x() - point.x()),(ref_point.pointInfo().y() - point.y()));
            if(temp_dis < min_distance)
            {
                min_distance = temp_dis;
                location = i;
                find_point = true;
            }
            i++;
        }
        minDis = min_distance;
        return find_point;
    }

    bool ReferenceLine::getNearstPoint(const math::Vec2d &point,int& location,const double startS, const double endS, const double &minDis) const
    {
        double min_distance = minDis;
        bool find_point = false;
        int i= 0;
        auto it_lower = std::lower_bound(accumulate_s_.begin(), accumulate_s_.end(),
                                         startS);
        int start_index = std::distance(accumulate_s_.begin(), it_lower);

        auto it_upper = std::lower_bound(accumulate_s_.begin(), accumulate_s_.end(),
                                         endS);
        int end_index = std::distance(accumulate_s_.begin(), it_lower);
        for (int i = start_index;i <= end_index;i++)
        {
            auto &ref_point = reference_points_[i];
            const double temp_dis = std::hypot((ref_point.pointInfo().x() - point.x()),(ref_point.pointInfo().y() - point.y()));
            if(temp_dis < min_distance)
            {
                min_distance = temp_dis;
                location = i;
                find_point = true;
            }
            i++;
        }
        return find_point;
    }

    bool ReferenceLine::getLeftAndRightWidth(const had_map::MapPoint point,double &left_width, double &right_width) const
    {
        ReferencePoint nearst_point;
        if(getNearstPoint(point, nearst_point))
        {
            left_width = nearst_point.leftWidth();
            right_width = nearst_point.rightWidth();
            had_map::MapPoint relativePoint;
            math::gussPointToFLU(nearst_point.pointInfo(), point, relativePoint);
            double dis = relativePoint.y();
            left_width -= dis;
            right_width += dis;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool ReferenceLine::getLeftAndRightWidth(const double s,double &left_width, double &right_width) const
    {
        ReferencePoint point = getNearstPoint(s);
        left_width = point.leftWidth();
        right_width = point.rightWidth();
        return true;
    }

    const LinkLaneSegment &ReferenceLine::getLinkLaneByS(const double s) const
    {
        const auto &point = getNearstPoint(s);
        return point.linkLaneSegment();
    }

    bool ReferenceLine::getSpeed(const had_map::MapPoint &point,double &maxSpeed, double &minSpeed) const
    {
        ReferencePoint nearst_point;
        if(getNearstPoint(point, nearst_point))
        {
            maxSpeed = nearst_point.limitMaxSpeed();
            minSpeed = nearst_point.limitMinSpeed();
            return true;
        }
        else
        {
            return false;
        }
    }

    void ReferenceLine::removePointFromIndex(const int& start,const int& end)
    {
        if(end < 0 || start >= (int)reference_points_.size())
        {
            return;
        }
        else if(start >= end)
        {
            return;
        }
        else
        {
            reference_points_.erase(reference_points_.begin() + start,reference_points_.begin() + end);
            accumulate_s_.erase(accumulate_s_.begin() + start,accumulate_s_.begin() + end);
            reCalculateS();
            reCalculateSegments();
        }
    }

    void ReferenceLine::testPrintS()const
    {
        for (auto s : accumulate_s_)
        {
            printf("s = %f\r\n",s);
        }
    }

    void ReferenceLine::removePointFromS(const double& start_s,const double& end_s)
    {
        if(start_s >= end_s)
        {
            return;
        }
        else
        {
            int start_point = getNearstPointLocation(start_s);
            int end_point = getNearstPointLocation(end_s);
            printf("size = %d\r\n", static_cast<int>(reference_points_.size()));
            removePointFromIndex(start_point,end_point + 1);
        }
    }

    void ReferenceLine::swapPoint(const ReferencePoint& point, uint32_t index)
    {
        if(index >= (reference_points_.size()) || index < 0) return;
        reference_points_[index] = point;
    }

    void ReferenceLine::RemoveDuplicates()
    {
        for (uint32_t i = 1; i < reference_points_.size(); ++i)
        {
            if(distanceTwoPoints(reference_points_[i],reference_points_[i - 1]) < 0.05)
            {
                reference_points_.erase(reference_points_.begin()+i);
                i--;
            }
        }

        reCalculateS();
        reCalculateSegments();
    }

    void ReferenceLine::reCalculateS()
    {
        accumulate_s_.clear();
        double s = 0;
        for (uint32_t i = 0; i < reference_points_.size(); ++i)
        {
            if(i != 0)
            {
                s += distanceTwoPoints(reference_points_[i],reference_points_[i-1]);
            }
            accumulate_s_.emplace_back(s);
        }
    }

    void ReferenceLine::reCalculateK()
    {
        if(reference_points_.empty()) return;

        for(size_t i = 0;i < reference_points_.size() - 1; i++) {
            reference_points_[i].setKappa(
                math::calculateKByTwoPoint(reference_points_[i].pointInfo(),
                                           reference_points_[i+1].pointInfo()));
        }

        if(reference_points_.size() > 1) {
            reference_points_[reference_points_.size() - 1].setKappa(
                reference_points_[reference_points_.size() - 2].kappa());
        }else {
            reference_points_[reference_points_.size() - 1].setKappa(0.0);
        }

        for(size_t i = 0;i < reference_points_.size() - 1; i++) {
            double dkappa = (reference_points_[i + 1].kappa() - reference_points_[i].kappa())
                            /(accumulate_s_[i+1] - accumulate_s_[i]);
            reference_points_[i].setDkappa(dkappa);
        }
    }

    void ReferenceLine::reCalculateSegments()
    {
        m_linkLaneSegments.clear();

        for (uint32_t i = 0; i < reference_points_.size(); ++i) {
            auto findIter = find(m_linkLaneSegments.begin(),m_linkLaneSegments.end(), reference_points_[i].linkLaneSegment());

            if(findIter == m_linkLaneSegments.end()){
                m_linkLaneSegments.emplace_back(reference_points_[i].linkLaneSegment());
            }
        }
    }

    //todo:应该从当前车辆位置出发判断是否相同轨迹
    int ReferenceLine::judgeRoadIncludeLane1(const LinkLaneSegment &linkLaneSegment) const
    {
        for(const auto &linkLane : m_linkLaneSegments) {
            if(linkLane.absLinkId == linkLaneSegment.absLinkId) {
                if(linkLane.laneId != linkLaneSegment.laneId) return 0;
                else return 2;
            }
        }

        return 1;
    }

    bool ReferenceLine::judgeRoadIncludeLane2(const LinkLaneSegment &linkLaneSegment) const
    {
        for(const auto &linkLane : m_linkLaneSegments) {
            if(linkLane == linkLaneSegment) return true;
        }

        return false;
    }

    void ReferenceLine::clear()
    {
        reference_points_.clear();
        m_linkLaneSegments.clear();
        accumulate_s_.clear();
        smooth_flag_ = false;
    }

    void ReferenceLine::connectOtherReference(ReferenceLine* ref_line,int& start)
    {
        if((int)ref_line->referencePoints().size() > start + 1) {
            reference_points_.insert(reference_points_.begin(),
                ref_line->referencePoints().begin() + start,
                ref_line->referencePoints().end());
        }

        //remove repeat points
        RemoveDuplicates();
    }


    bool ReferenceLine::Shrink(const math::Vec2d &point, SLPoint &slPoint,
                               const double look_backward,const double look_forward)
    {
        if(!XYToSL(point,slPoint)) {
            printf("WARNING : car is no on this reference line\r\n");
            return false;
        }

        size_t start_index = 0;

        if(slPoint.s() > look_backward) {
            auto it_lower = std::lower_bound(
                accumulate_s_.begin(), accumulate_s_.end(), slPoint.s() - look_backward);

            start_index = std::distance(accumulate_s_.begin(), it_lower);
        }

        size_t end_index = reference_points_.size();
        if(slPoint.s() + look_forward < length()) {
            auto start_it = accumulate_s_.begin();
            std::advance(start_it, start_index);
            auto it_higher =
                    std::upper_bound(start_it, accumulate_s_.end(), slPoint.s() + look_forward);
            end_index = std::distance(accumulate_s_.begin(), it_higher);
        }

        reference_points_.erase(reference_points_.begin() + end_index,
                                reference_points_.end());
        reference_points_.erase(reference_points_.begin(),
                                reference_points_.begin() + start_index);

        reCalculateS();
        reCalculateSegments();

        if (reference_points_.size() < 2) {
            std::cout << "WARNING : Too few reference points after shrinking.";
            return false;
        }

        return true;
    }

    bool ReferenceLine::SLToXY(const SLPoint& sl_point,
             math::Vec2d* const xy_point) const
    {
        if(xy_point == NULL) return false;
        if(reference_points_.size() < 2) return false;

        const auto matched_point = getNearstPoint(sl_point.s());
        xy_point->set_x(matched_point.pointInfo().x() - std::cos(matched_point.pointInfo().heading()) * sl_point.l());
        xy_point->set_y(matched_point.pointInfo().y() + std::sin(matched_point.pointInfo().heading()) * sl_point.l());

        return true;
    }

    bool ReferenceLine::XYToSL(const math::Vec2d &vec2d, SLPoint &slPoint)const
    {      
        int index;
        double minDis = std::numeric_limits<double>::max();
        if(!getNearstPoint(vec2d,index,minDis)) return false;
        had_map::MapPoint transformPoint;
        had_map::MapPoint goalPoint;
        transformPoint.set_x(vec2d.x());
        transformPoint.set_y(vec2d.y());
        transformPoint.setHeading(0.0);
        math::gussPointToFLU(reference_points_[index].pointInfo(), transformPoint, goalPoint);
        slPoint.setS(accumulate_s_[index] + goalPoint.x());
        slPoint.setL(goalPoint.y());
        return true;
    }

    bool ReferenceLine::XYToSL(const had_map::MapPoint &point, SLPoint &slPoint)const
    {
        int index;
        double minDis = std::numeric_limits<double>::max();

        int i= 0;
        bool findPointFlag = false;
        double coe = 3.0;
        for (auto &ref_point : reference_points_)
        {
            const double temp_dis = std::hypot((ref_point.pointInfo().x() - point.x()),(ref_point.pointInfo().y() - point.y()))
                                    + coe * fabs(point.heading() - ref_point.pointInfo().heading());
            if(temp_dis < minDis)
            {
                minDis = temp_dis;
                index = i;
                findPointFlag = true;
            }
            i++;
        }
        if(!findPointFlag)
        {
            return false;
        }
        had_map::MapPoint transformPoint;
        had_map::MapPoint goalPoint;
        transformPoint.set_x(point.x());
        transformPoint.set_y(point.y());
        transformPoint.setHeading(point.heading());
        math::gussPointToFLU(reference_points_[index].pointInfo(), transformPoint, goalPoint);
        slPoint.setS(accumulate_s_[index] + goalPoint.x());
        slPoint.setL(goalPoint.y());
        return true;
    }

    bool ReferenceLine::XYToSL(const math::Vec2d &vec2d,const double startS,const double endS, SLPoint &slPoint)const
    {
        int index;
        double maxDis = 100;

        if(!getNearstPoint(vec2d,index,startS,endS,maxDis)) return false;

        had_map::MapPoint transformPoint;
        had_map::MapPoint goalPoint;
        transformPoint.set_x(vec2d.x());
        transformPoint.set_y(vec2d.y());
        transformPoint.setHeading(0.0);
        math::gussPointToFLU(reference_points_[index].pointInfo(), transformPoint, goalPoint);

        double radian = reference_points_[index].pointInfo().heading();

        slPoint.setS(accumulate_s_[index] + goalPoint.x());
        slPoint.setL(goalPoint.y());

        return true;
    }

    bool ReferenceLine::getSLBoundary(const math::Box2d& box,
                                          SLBoundary* const sl_boundary) const
    {
        double start_s(std::numeric_limits<double>::max());
        double end_s(std::numeric_limits<double>::lowest());
        double start_l(std::numeric_limits<double>::max());
        double end_l(std::numeric_limits<double>::lowest());

        std::vector<math::Vec2d> corners;
        box.GetAllCorners(&corners);

        for (const auto& point : corners) {
            SLPoint sl_point;

            if(!XYToSL(point, sl_point)) return false;

            start_s = std::fmin(start_s, sl_point.s());
            end_s = std::fmax(end_s, sl_point.s());
            start_l = std::fmin(start_l, sl_point.l());
            end_l = std::fmax(end_l, sl_point.l());
        }

        sl_boundary->setStartS(start_s);
        sl_boundary->setEndS(end_s);
        sl_boundary->setStartL(start_l);
        sl_boundary->setEndL(end_l);

        return true;
    }

    bool ReferenceLine::getApproximateSLBoundary(const math::Box2d& box,const double startS,const double endS,SLBoundary* const sl_boundary)const
    {
        double start_s(std::numeric_limits<double>::max());
        double end_s(std::numeric_limits<double>::lowest());
        double start_l(std::numeric_limits<double>::max());
        double end_l(std::numeric_limits<double>::lowest());

        std::vector<math::Vec2d> corners;
        box.GetAllCorners(&corners);

        for(const auto& point : corners) {
            SLPoint sl_point;

            if(!XYToSL(point,startS,endS,sl_point)) return false;
 
            start_s = std::fmin(start_s, sl_point.s());
            end_s = std::fmax(end_s, sl_point.s());
            start_l = std::fmin(start_l, sl_point.l());
            end_l = std::fmax(end_l, sl_point.l());
        }

        sl_boundary->setStartS(start_s);
        sl_boundary->setEndS(end_s);
        sl_boundary->setStartL(start_l);
        sl_boundary->setEndL(end_l);

        return true;
    }

    bool ReferenceLine::includeLinkLane(const LinkLaneSegment &linkLaneSegment) const
    {
        for(const auto &linkLane : m_linkLaneSegments) {
            if(linkLane == linkLaneSegment) return true;
        }

        return false;
    }

    bool ReferenceLine::isBlockRoad(const math::Box2d &box, double gap)const
    {
        if(distanceToBox(box) < gap + kMathEpsilon) return true;
        
        return false;
    }

    double ReferenceLine::distanceToBox(const math::Box2d &box2d)const
    {
        double minDis = std::numeric_limits<double>::max();
        const auto &corners = box2d.GetAllCorners();

        for(const auto &corner : corners) {
            SLPoint slPoint;
            XYToSL(corner,slPoint);

            if(fabs(slPoint.l()) < minDis) minDis = fabs(slPoint.l());
        }

        return minDis;
    }

    bool ReferenceLine::IsOnRoad(const SLBoundary& sl_boundary) const {
        if(sl_boundary.endS() < 0 || sl_boundary.startS() > length()) return false;
       
        double middle_s = (sl_boundary.startS() + sl_boundary.endS()) / 2.0;
        auto it_lower = std::lower_bound(accumulate_s_.begin(), accumulate_s_.end(), middle_s);
        int location = std::distance(accumulate_s_.begin(),it_lower);
        double laneLeftWidth = reference_points_[location].leftWidth();
        double laneRightWidth = reference_points_[location].rightWidth();

        //todo:一个角在此区域及认为在道路线上,待后续查看
        return !(sl_boundary.startL() > laneLeftWidth ||
                 sl_boundary.endL() < -laneRightWidth);
    }

    bool ReferenceLine::IsOnRoad(const SLPoint& sl_point) const {
        if(sl_point.s() <= 0 || sl_point.s() > length()) return false;
       
        double left_width = 0.0;
        double right_width = 0.0;

        if(!getLeftAndRightWidth(sl_point.s(), left_width, right_width)) return false;

        return (sl_point.l() < left_width && sl_point.l() > -right_width);
    }

    bool ReferenceLine::IsOnRoad(const math::Vec2d& point) const
    {
        SLPoint slPoint;

        if(!XYToSL(point,slPoint)) return false;
        
        return IsOnRoad(slPoint);
    }

    double ReferenceLine::distanceTwoPoints(ReferencePoint& point1, ReferencePoint& point2) const
    {
        return std::hypot((point1.pointInfo().x() - point2.pointInfo().x()),(point1.pointInfo().y() - point2.pointInfo().y()));
    }

    const double ReferenceLine::length() const
    {
        return accumulate_s_.back();
    }

    bool ReferenceLine::addOnePointInBack(ReferencePoint &referencePoint)
    {
        double dis;

        if(reference_points_.empty()) dis = 0;
        else {
            dis = distanceTwoPoints(referencePoint,reference_points_.back());

            if(dis > MAX_point_dis) {
                printf("error! front maps points dis more than 5 meter!!!!\r\n");
                return false;
            }
        }

        if(dis < MINDISTANCEFROMTWOPOINTS && !reference_points_.empty()) return false;

        if(dis > spacing_dis_) spacing_dis_ = dis;

        reference_points_.emplace_back(std::move(referencePoint));

        return true;
    }

    bool ReferenceLine::addOnePointInBegin(ReferencePoint &referencePoint)
    {
        double dis;
        if(reference_points_.empty()) dis = 0;
        else {
            dis = distanceTwoPoints(referencePoint,reference_points_.front());

            if(dis > MAX_point_dis) {
                printf("error! back maps points dis more than 5 meter!!!!\r\n");
                return false;
            }
        }

        if(dis < MINDISTANCEFROMTWOPOINTS && !reference_points_.empty()) return false;

        reference_points_.insert(reference_points_.begin(),referencePoint);

        if(dis > spacing_dis_) spacing_dis_ = dis;

        return true;
    }

    void ReferenceLine::setSommthFlag(bool smooth_flag)
    {
        smooth_flag_ = smooth_flag;
    }

    bool ReferenceLine::smoothFlag()
    {
        return smooth_flag_;
    }

    const double ReferenceLine::spacingDis() const
    {
        return spacing_dis_;
    }

    void ReferenceLine::setSpacingDis(const double &dis)
    {
        spacing_dis_ = dis;
    }

    void ReferenceLine::setLaneId(const int &laneID)
    {
        m_laneId = laneID;
    }

    int ReferenceLine::getLaneId()const
    {
        return m_laneId;
    }
}
