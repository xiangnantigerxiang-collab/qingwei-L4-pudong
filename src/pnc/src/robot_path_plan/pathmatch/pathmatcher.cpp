#include <algorithm>
#include <iostream>
#include <cmath>
#include "pathmatcher.h"

double NormalizeAngle(const double angle) {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);

    if(a < 0.0) a += (2.0 * M_PI);

    return a - M_PI;
}

double slerp(const double a0, const double t0,
             const double a1, const double t1, const double t) {
    if(std::abs(t1 - t0) <= 0.0001) return NormalizeAngle(a0);

    const double a0_n = NormalizeAngle(a0);
    const double a1_n = NormalizeAngle(a1);

    double d = a1_n - a0_n;

    if(d > M_PI) {
        d = d - 2 * M_PI;
    } else if(d < -M_PI) {
        d = d + 2 * M_PI;
    }

    const double r = (t - t0) / (t1 - t0);
    const double a = a0_n + d * r;

    return NormalizeAngle(a);
}

PathPoint PathMatcher::MatchToPath(const std::vector<PathPoint>& reference_line,
                                   const double x, const double y) {
    auto func_distance_square = [](const PathPoint& point, const double x,
                                   const double y) {
        double dx = point.x() - x;
        double dy = point.y() - y;
        return dx * dx + dy * dy;
    };

    double distance_min = func_distance_square(reference_line.front(), x, y);
    std::size_t index_min = 0;

    for(std::size_t i = 1; i < reference_line.size(); ++i) {
        double distance_temp = func_distance_square(reference_line[i], x, y);
        if(distance_temp < distance_min) {
            distance_min = distance_temp;
            index_min = i;
        }
    }

    std::size_t index_start = (index_min == 0) ? index_min : index_min - 1;
    std::size_t index_end =
        (index_min + 1 == reference_line.size()) ? index_min : index_min + 1;

    if(index_start == index_end) {
        return reference_line[index_start];
    }

    return FindProjectionPoint(
        reference_line[index_start], reference_line[index_end], x, y);
}

PathPoint PathMatcher::MatchToPath(const std::vector<PathPoint>& reference_line,
                                   const double s) {
    auto comp = [](const PathPoint& point, const double s) {
        return point.s() < s;
    };

    auto it_lower =
        std::lower_bound(reference_line.begin(), reference_line.end(), s, comp);

    if(it_lower == reference_line.begin()) {
        return reference_line.front();
    } else if(it_lower == reference_line.end()) {
        return reference_line.back();
    }

    return InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, s);
}

PathPoint PathMatcher::FindProjectionPoint(const PathPoint& p0,
                                           const PathPoint& p1, const double x,
                                           const double y) {
    double v0x = x - p0.x();
    double v0y = y - p0.y();

    double v1x = p1.x() - p0.x();
    double v1y = p1.y() - p0.y();

    double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
    double dot = v0x * v1x + v0y * v1y;

    double delta_s = dot / v1_norm;
    return InterpolateUsingLinearApproximation(p0, p1, p0.s() + delta_s);
}

PathPoint PathMatcher::InterpolateUsingLinearApproximation(const PathPoint& p0,
                                                           const PathPoint& p1,
                                                           const double s) {
    double s0 = p0.s();
    double s1 = p1.s();

    PathPoint path_point;
    double weight = (s - s0) / (s1 - s0);
    double x = (1 - weight) * p0.x() + weight * p1.x();
    double y = (1 - weight) * p0.y() + weight * p1.y();
    double theta = (p0.theta() + p1.theta()) / 2;
    double kappa = (1 - weight) * p0.kappa() + weight * p1.kappa();
    double dkappa = (1 - weight) * p0.dkappa() + weight * p1.dkappa();
    double ddkappa = (1 - weight) * p0.ddkappa() + weight * p1.ddkappa();

    path_point.setX(x);
    path_point.setY(y);
    path_point.setTheta(theta);
    path_point.setKappa(kappa);
    path_point.setDkappa(dkappa);
    path_point.setDdkappa(ddkappa);
    path_point.setS(s);

    return path_point;
}
