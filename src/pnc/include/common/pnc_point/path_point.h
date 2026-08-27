
#ifndef PLANNING_PATH_POINT_H
#define PLANNING_PATH_POINT_H

namespace planning
{
    class PathPoint
    {
    public:
        PathPoint() = default;

        PathPoint(double x, double y , double z,
                  double theta, double kappa,
                  double dkappa, double ddkappa):
                    m_x(x), m_y(y),m_z(z),
                    m_theta(theta),m_kappa(kappa),
                    m_dkappa(dkappa), m_ddkappa(ddkappa){}

        double x()const
        {
            return m_x;
        }

        double y()const
        {
            return m_y;
        }

        double z()const
        {
            return m_z;
        }

        double theta()const
        {
            return m_theta;
        }

        double kappa()const
        {
            return m_kappa;
        }

        double s()const
        {
            return m_s;
        }

        double dkappa()const
        {
            return m_dkappa;
        }

        double ddkappa()const
        {
            return m_ddkappa;
        }

        int laneId()const
        {
            return m_lane_id;
        }

        void setX(const double x)
        {
            m_x = x;
        }

        void setY(const double y)
        {
            m_y = y;
        }
        void setZ(const double z)
        {
            m_z = z;
        }
        void setTheta(const double theta)
        {
            m_theta = theta;
        }
        void setKappa(const double kappa)
        {
            m_kappa = kappa;
        }
        void setS(const double s1)
        {
            m_s = s1;
        }
        void setDkappa(const double dkappa)
        {
            m_dkappa = dkappa;
        }
        void setDdkappa(const double ddkappa)
        {
            m_ddkappa = ddkappa;
        }
        void setLaneId(const int laneId)
        {
            m_lane_id = laneId;
        }

        void CopyFrom(const PathPoint &point)
        {
            m_x = point.x();
            m_y = point.y();
            m_z = point.z();
            m_theta = point.theta();
            m_kappa = point.kappa();
            m_s = point.s();
            m_dkappa = point.dkappa();
            m_ddkappa = point.ddkappa();
            m_lane_id = point.laneId();
        }

    private:
         double m_x = 0.0;
         double m_y = 0.0;
         double m_z = 0.0;

        // direction on the x-y plane
         double m_theta = 0.0;
        // curvature on the x-y planning
         double m_kappa = 0.0;
        // accumulated distance from beginning of the path
         double m_s = 0.0;

        // derivative of kappa w.r.t s.
         double m_dkappa = 0.0;
        // derivative of derivative of kappa w.r.t s.
         double m_ddkappa = 0.0;
        // The lane ID where the path point is on
         int m_lane_id = 0.0;
    };
}//end namespace planning
#endif //PLANNING_PATH_POINT_H
