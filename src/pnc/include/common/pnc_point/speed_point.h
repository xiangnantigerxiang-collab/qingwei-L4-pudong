
#ifndef PLANNING_SPEED_POINT_H
#define PLANNING_SPEED_POINT_H

namespace planning {

    class SpeedPoint {
    public:
        SpeedPoint() = default;
        ~SpeedPoint() = default;

        SpeedPoint(double s, double t, double v, double a, double da) {
            m_s = s;
            m_t = t;
            m_v = v;
            m_a = a;
            m_da = da;
        };

        inline double s() const {
            return m_s;
        }

        inline double t() const {
            return m_t;
        }

        inline double v() const {
            return m_v;
        }

        inline double a() const {
            return m_a;
        }

        inline double da() const {
            return m_da;
        }

        void setS(double s) {
            m_s = s;
        }

        void setT(double t) {
            m_t = t;
        }

        void setV(double v) {
            m_v = v;
        }

        void setA(double a) {
            m_a = a;
        }

        void setDa(double da) {
            m_da = da;
        }

    private:
        double m_s;
        double m_t;
        double m_v;
        double m_a;
        double m_da;
    };

}  // namespace planning

#endif  //PLANNING_SPEED_POINT_H
