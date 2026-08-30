
#include <cmath>
#include "common/polynomial_xd/curve_math.h"
namespace planning {

// kappa = (dx * d2y - dy * d2x) / [(dx * dx + dy * dy)^(3/2)]
    double CurveMath::ComputeCurvature_(const double dx, const double d2x,
                                       const double dy, const double d2y) {
        const double a = dx * d2y - dy * d2x;
        constexpr double kOrder = 1.5;
        const double b = std::pow(dx * dx + dy * dy, kOrder);
        return a / b;
    }

    double CurveMath::ComputeCurvature_(const double dy, const double d2y)
    {
        return d2y / (std::pow((1 + dy * dy), 1.5));
    }

    double CurveMath::ComputeCurvatureDerivative_(const double dx, const double d2x,
                                                 const double d3x, const double dy,
                                                 const double d2y,
                                                 const double d3y) {
        const double a = dx * d2y - dy * d2x;
        const double b = dx * d3y - dy * d3x;
        const double c = dx * d2x + dy * d2y;
        const double d = dx * dx + dy * dy;

        return (b * d - 3.0 * a * c) / std::pow(d, 3.0);
    }

}  // namespace planning