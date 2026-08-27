
#ifndef PLANNING_REFERENCE_POINT_H
#define PLANNING_REFERENCE_POINT_H

class ReferencePoint {
    public:
        ReferencePoint() = default;

        ReferencePoint(double x, double y,
                  double heading, double kappa,
                  double dkappa double s):
                    m_x(x), m_y(y),
                    m_theta(theta),m_kappa(kappa),
                    m_dkappa(dkappa),m_s(s){}

        double x()const
        {
            return m_x;
        }

        double y()const
        {
            return m_y;
        }

        double heading()const
        {
            return m_heading;
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

        void set_x(const double x)
        {
            m_x = x;
        }

        void set_y(const double y)
        {
            m_y = y;
        }
    
        void set_heading(const double heading)
        {
            m_heading = heading;
        }
        void set_kappa(const double kappa)
        {
            m_kappa = kappa;
        }
        
        void set_dkappa(const double dkappa)
        {
            m_dkappa = dkappa;
        }
        void set_s(const double s1)
        {
            m_s = s1;
        }
        
    private:
         double m_x = 0.0;
         double m_y = 0.0;
        // direction on the x-y plane
         double m_heading = 0.0;
        // curvature on the x-y planning
         double m_kappa = 0.0;
        // accumulated distance from beginning of the path
         double m_s = 0.0;

        // derivative of kappa w.r.t s.
         double m_dkappa = 0.0;
    };
#endif //PLANNING_REFERENCE_POINT_H