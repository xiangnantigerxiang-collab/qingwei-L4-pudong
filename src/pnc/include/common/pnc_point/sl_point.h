
#ifndef PLANNING_SL_POINT_H
#define PLANNING_SL_POINT_H

namespace math
{
    class SLPoint
    {
    public:
        SLPoint() = default;
        SLPoint(const double s1, const double l1):
                m_s(s1),m_l(l1){}
        double s() const
        {
            return m_s;
        }

        double l() const
        {
            return m_l;
        }

        void setS(const double s1)
        {
            m_s = s1;
        }

        void setL(const double l1)
        {
            m_l = l1;
        }

    private:
        double m_s;
        double m_l;
    };
}

#endif //PLANNING_SL_POINT_H
