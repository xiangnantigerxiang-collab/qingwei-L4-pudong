
#include <cstdio>
#include "common/path/sl_boundary.h"

namespace planning {
    double SLBoundary::startS() const {
        return m_start_s;
    }

    double SLBoundary::endS() const {
        return m_end_s;
    }

    double SLBoundary::startL() const {
        return m_start_l;
    }

    double SLBoundary::endL() const {
        return m_end_l;
    }

    void SLBoundary::setStartS(const double start_s) {
        m_start_s = start_s;
    }

    void SLBoundary::setStartL(const double start_l) {
        m_start_l = start_l;
    }

    void SLBoundary::setEndS(const double end_s) {
        m_end_s = end_s;
    }

    void SLBoundary::setEndL(const double end_l) {
        m_end_l = end_l;
    }

    void SLBoundary::print() const {
        printf("start_S = %f\r\n", m_start_s);
        printf("end_s = %f\r\n", m_end_s);
        printf("start_l = %f\r\n", m_start_l);
        printf("end_l = %f\r\n", m_end_l);
    }
}  //end namespace planning