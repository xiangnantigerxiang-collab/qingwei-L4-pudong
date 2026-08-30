
#ifndef PLANNING_SLBOUNDARY_H
#define PLANNING_SLBOUNDARY_H

namespace planning
{
    class SLBoundary
    {
    public:
        double startS()const;
        double endS()const;
        double startL()const;
        double endL()const;

        void setStartS(const double start_s);
        void setStartL(const double start_l);
        void setEndS(const double end_s);
        void setEndL(const double end_l);

        void print() const;

    private:
        double m_start_s;
        double m_end_s;
        double m_start_l;
        double m_end_l;
    };
}//end namespace planning

#endif //PLANNING_SLBOUNDARY_H
