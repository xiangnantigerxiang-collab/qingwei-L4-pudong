

#ifndef PLANNING_FRENET_FRAME_POINT_H
#define PLANNING_FRENET_FRAME_POINT_H

namespace planning
{
    class FrenetFramePoint
    {
    public:
        double s()const {
            return m_s;
        }

        double l()const{
            return m_l;
        }

        double dl()const{
            return m_dl;
        }

        double ddl()const{
            return m_ddl;
        }

        void setS(double s)
        {
            m_s = s;
        }

        void setL(double l)
        {
            m_l = l;
        }

        void setDl(double dl)
        {
            m_dl = dl;
        }

        void setDdl(double ddl)
        {
            m_ddl = ddl;
        }

    private:
        double m_s = -1;
        double m_l = 0;
        double m_dl = 0;
        double m_ddl = 0;
    };
}

#endif //PLANNING_FRENET_FRAME_POINT_H
