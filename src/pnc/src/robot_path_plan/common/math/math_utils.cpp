
#include "common/math/math_utils.h"

#include <cmath>
#include <utility>
#include <common/config/config.h>

namespace math {

    double Sqr(const double x) { return x * x; }

    double CrossProd(const Vec2d& start_point, const Vec2d& end_point_1,
                     const Vec2d& end_point_2) {
        return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
    }

    double InnerProd(const Vec2d& start_point, const Vec2d& end_point_1,
                     const Vec2d& end_point_2) {
        return (end_point_1 - start_point).InnerProd(end_point_2 - start_point);
    }

    double CrossProd(const double x0, const double y0, const double x1,
                     const double y1) {
        return x0 * y1 - x1 * y0;
    }

    double InnerProd(const double x0, const double y0, const double x1,
                     const double y1) {
        return x0 * x1 + y0 * y1;
    }

    double degreeToRadian(const double angle){
        return (angle * M_PI / 180.0);
    }

    double radianToDegree(const double radian){
        return (radian * 180.0 / M_PI);
    }

    Vec2d translationPoint(const Vec2d point, const double theta,
                           const double length)
    {
        Vec2d newPoint;
        newPoint.set_x(point.x() + length * sin(theta));
        newPoint.set_y(point.y() + length * cos(theta));
        return newPoint;
    }

    double WrapAngle(const double angle) {
        const double new_angle = std::fmod(angle, M_PI * 2.0);
        return new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle;
    }

    double NormalizeAngle(const double angle) {
        double a = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (a < 0.0) {
            a += (2.0 * M_PI);
        }
        return a - M_PI;
    }

    double AngleDiff(const double from, const double to) {
        return NormalizeAngle(to - from);
    }

    double getAngle(const double x, const double y)
    {
        double radian = -atan2(y,x) + M_PI / 2.0;
        return WrapAngle(radian);
    }

    int RandomInt(const int s, const int t, unsigned int rand_seed) {
        if (s >= t) {
            return s;
        }
        return s + rand_r(&rand_seed) % (t - s + 1);
    }

    double RandomDouble(const double s, const double t, unsigned int rand_seed) {
        return s + (t - s) / 16383.0 * (rand_r(&rand_seed) & 16383);
    }

    double accBySpeedOfChange(const double startSpeed,
                              const double endSpeed,
                              const double distance)
    {
        if(distance < 1e-6)
        {
            if(endSpeed - startSpeed > 0)
            {
                return std::numeric_limits<double>::max();
            }
            else
            {
                return std::numeric_limits<double>::lowest();
            }
        }
        return (endSpeed * endSpeed - startSpeed * startSpeed) / (2.0 * distance);
    }

    void gussPointToFLU(const had_map::MapPoint &originPoint,
                        const had_map::MapPoint &goalPoint,
                        had_map::MapPoint &relativePoint)
    {
        math::Vec2d point = goalPoint - originPoint;
        double x;
        double y;
        RotateAxis(-originPoint.heading(),point.x(),point.y(),
                    &x,&y);

        //RFU to FLU
        relativePoint.set_x(y);
        relativePoint.set_y(-x);
        relativePoint.setHeading(NormalizeAngle(goalPoint.heading()
                                                - originPoint.heading()
                                                + M_PI / 2.0));
    }

    void gussPointToRFU(const had_map::MapPoint &originPoint,
                        const had_map::MapPoint &goalPoint,
                        had_map::MapPoint &relativePoint)
    {
        math::Vec2d point = goalPoint - originPoint;
        double x;
        double y;
        RotateAxis(-originPoint.heading(),point.x(),point.y(),
                   &x,&y);

        relativePoint.set_x(x);
        relativePoint.set_y(y);
        relativePoint.setHeading(NormalizeAngle(goalPoint.heading()
                                                - originPoint.heading()));
    }

    void gussPointToRFU2(const had_map::MapPoint &originPoint,
                         const math::Vec2d &goalPoint,
                         math::Vec2d &relativePoint)
    {
        math::Vec2d point = goalPoint - originPoint;
        double x;
        double y;
        RotateAxis(-originPoint.heading(),point.x(),point.y(),
                   &x,&y);

        relativePoint.set_x(x);
        relativePoint.set_y(y);
    }

    void RFUPointToGuss(const had_map::MapPoint &originPoint,
                        const had_map::MapPoint &goalPoint,
                        had_map::MapPoint &relativePoint)
    {
        double x;
        double y;
        RotateAxis(originPoint.heading(),goalPoint.x(),goalPoint.y(),
                   &x,&y);
        relativePoint.set_x(x + originPoint.x());
        relativePoint.set_y(y + originPoint.y());
        relativePoint.setHeading(WrapAngle(goalPoint.heading() + originPoint.heading()));
    }

    void FLUPointToGuss(const had_map::MapPoint &originPoint,
                        const had_map::MapPoint &goalPoint,
                        had_map::MapPoint &relativePoint)
    {
        math::Vec2d rfuPoint;
        rfuPoint.set_x(-goalPoint.y());
        rfuPoint.set_y(goalPoint.x());
        double x;
        double y;
        RotateAxis(originPoint.heading(),rfuPoint.x(),rfuPoint.y(),
                   &x,&y);
        relativePoint.set_x(x + originPoint.x());
        relativePoint.set_y(y + originPoint.y());
        relativePoint.setHeading(WrapAngle(goalPoint.heading() - M_PI_2 + originPoint.heading()));
    }

// Gaussian
    double Gaussian(const double u, const double std, const double x) {
        return (1.0 / std::sqrt(2 * M_PI * std * std)) *
               std::exp(-(x - u) * (x - u) / (2 * std * std));
    }

// Sigmoid
    double Sigmoid(const double x) { return 1.0 / (1.0 + std::exp(-x)); }

    double Sigmoid2(const double x) { return 1.0 / (0.0 + std::exp(-x)); }

    void RotateAxis(const double theta, const double x0, const double y0,
                    double* x1, double* y1) {
//        CHECK_NOTNULL(x1);
//        CHECK_NOTNULL(y1);

        const double cos_theta = std::cos(theta);
        const double sin_theta = std::sin(theta);
        *x1 = x0 * cos_theta + y0 * sin_theta;
        *y1 = -x0 * sin_theta + y0 * cos_theta;
    }

    double calculateKByTwoPoint(const had_map::MapPoint &p1, const had_map::MapPoint &p2)
    {
        double dis = std::hypot((p1.x() - p2.x()), (p1.y() - p2.y()));
        double heading = NormalizeAngle(p1.heading() - p2.heading());
        if(fabs(heading) < 0.00001 || dis < 0.001)
        {
            return 0;
        }
        double a = heading / 2;
        double k = 2 * std::sin(a) / dis;
        return k;
    }
}  // namespace math