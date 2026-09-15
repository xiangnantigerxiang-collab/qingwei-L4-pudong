#ifndef SPLINE_H
#define SPLINE_H

#include <vector>
#include <math.h>
#include "../pubalgor/pubalgor.h"

class CSpline {
public:
    CSpline(void) {};
    ~CSpline(void) {};

    void SplinePointSet(std::vector<XYZ_COOR_S> tSrcCoorList,
                        std::vector<XYZ_COOR_S> &tDesCoorList,
                        float tRes);

    void SplinePointSet(std::vector<XYZ_COOR_S> tSrcCoorList,
                        std::vector<XYZ_COOR_S> &tDesCoorList, float tRes,
                        std::vector<XYZ_COOR_S> tSrcList,
                        std::vector<XYZ_COOR_S> &tDesList);

private:
    void Splint(float *xa, float *ya, float *m, int n, float &x, float &y);
    void Spline1(float *xa, float *ya, int n, float *&m, float bound1, float bound2);
    void Spline2(float *xa, float *ya, int n, float *&m, float bound1 = 0, float bound2 = 0);
};

#endif
