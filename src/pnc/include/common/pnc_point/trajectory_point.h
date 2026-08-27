
#ifndef PLANNING_TRAJECTORY_POINT_H
#define PLANNING_TRAJECTORY_POINT_H

#include "path_point.h"

using namespace planning;

    class TrajectoryPoint
    {
    public:
        TrajectoryPoint() = default;

        const PathPoint &pathPoint()const{
            return path_point;
        }

        PathPoint *mutablePathPoint()
        {
            return &path_point;
        }

        double v()const{
            return m_v;
        }

        double a()const{
            return m_a;
        }
        double cost()const{
            return m_cost;
        }
        bool safeproperty()const{
            return m_safeproperty;
        }

        double relativeTime()const{
            return m_relative_time;
        }

        void setPathPoint(const PathPoint &pathPoint1){
            path_point = pathPoint1;
        }

        void setV(const double speed)
        {
            m_v = speed;
        }

        void setA(const double a)
        {
            m_a = a;
        }
        
        void setCost(const double cost)
        {
            m_cost = cost;
        }
        
        void setSafeProperty(const bool safeproperty)
        {
            m_safeproperty = safeproperty;
        }

        void setRelativeTime(const double relativeTime)
        {
            m_relative_time = relativeTime;
        }

        void setLatUse(const bool latFlag)
        {
            m_latUse = latFlag;
        }

        bool latUse() const
        {
            return m_latUse;
        }
     
        PathPoint path_point;
    private:
        // linear velocity
        double m_v;  // in [m/s]
        // linear acceleration
        double m_a;
        // relative time from beginning of the trajectory
        double m_relative_time;

        double m_cost;

        bool m_safeproperty;

        bool m_latUse = false;
    };


#endif //PLANNING_TRAJECTORY_POINT_H
