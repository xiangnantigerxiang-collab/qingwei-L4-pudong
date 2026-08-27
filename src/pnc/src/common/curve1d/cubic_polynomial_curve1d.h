#ifndef PLANNING_CUBIC_POLYNOMIAL_CURVE1D_H
#define PLANNING_CUBIC_POLYNOMIAL_CURVE1D_H

#include <array>
#include <string>

#include "polynomial_curve1d.h"

namespace planning {

class CubicPolynomialCurve1d : public PolynomialCurve1d {
public:
    CubicPolynomialCurve1d() = default;
    virtual ~CubicPolynomialCurve1d() = default;

    CubicPolynomialCurve1d(const std::array<double, 3>& start, const double end,
                           const double param);

    CubicPolynomialCurve1d(const std::array<double, 2>& start, const double end,
                           const double param);

    CubicPolynomialCurve1d(const double x0, const double dx0, const double ddx0,
                           const double x1, const double param);

    CubicPolynomialCurve1d(const CubicPolynomialCurve1d& other);

    double Evaluate(const std::uint32_t order, const double p) const override;

    double ParamLength() const { return param_; }
    // std::string ToString() const override;

private:
    void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                             const double x1, const double param);
    std::array<double, 4> coef_ = {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, 3> start_condition_ = {{0.0, 0.0, 0.0}};
    double end_condition_ = 0.0;
};

}  // namespace planning

#endif //PLANNING_CUBIC_POLYNOMIAL_CURVE1D_H
