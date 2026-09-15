#ifndef HDMAP_CUBIC_SPLINE_H
#define HDMAP_CUBIC_SPLINE_H

#include "hdmap/hdmap_types.h"

namespace hdmap
{
namespace detail
{
class CubicSpline
{
public:
    STATUS_S Fit(const MapPointList& tPoints, DIRECTION_E tDirection);
    STATUS_S EvaluateByLength(double tLength, MAP_POINT_S& tPoint) const;
    double GetLength() const;

private:
    struct Polynomial
    {
        double a, b, c, d;
        double Value(double tU) const;
        double First(double tU) const;
        double Second(double tU) const;
    };
    struct Segment
    {
        Polynomial x, y;
        double span;
        double length;
        double Speed(double tU) const;
    };
    static bool BuildAxis(const std::vector<double>& tValues,
                          const std::vector<double>& tSpans, double tStart,
                          double tEnd, std::vector<Polynomial>& tPolynomials);
    static bool Integrate(const Segment& tSegment, double tFrom, double tTo,
                          double tTolerance, double& tLength);
    static bool RefineIntegral(const Segment& tSegment, double tFrom, double tTo,
                               double tLeft, double tMiddle, double tRight,
                               double tWhole, double tTolerance, int tDepth,
                               double& tLength);
    static bool HasRegularTangent(const Segment& tSegment);
    std::vector<Segment> mSegments;
    std::vector<double> mLengths;
    DIRECTION_E mDirection = DIRECTION_FORWARD;
};
}
}

#endif
