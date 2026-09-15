
#ifndef PLANNING_SPLINE_2D_CONSTRAINT_H
#define PLANNING_SPLINE_2D_CONSTRAINT_H

#include <vector>
#include <cstdint>

#include "affine_constraint.h"
#include "common/surface/vec2d.h"

namespace planning {
    class Spline2dConstraint {
    public:
        Spline2dConstraint() = default;
        Spline2dConstraint(const std::vector<double>& t_knots, const uint32_t order);

        bool Add2dBoundary(
            const std::vector<double>& t_coord, const std::vector<double>& angle,
            const std::vector<math::Vec2d>& ref_point,
            const std::vector<double>& longitudinal_bound,
            const std::vector<double>& lateral_bound);

        bool AddPointAngleConstraint(const double t,
                                     const double angle);

        bool AddPointSecondDerivativeSmoothConstraint(const double& t,
                                                      const double& xds,
                                                      const double& yds,
                                                      const double& xseconds,
                                                      const double& yseconds);

        bool AddSecondDerivativeSmoothConstraint();

        const AffineConstraint& inequality_constraint() const;
        const AffineConstraint& equality_constraint() const;

    private:
        double SignDistance(const math::Vec2d& xy_point,
                            const double& angle) const;

        uint32_t FindIndex(const double t) const;

        std::vector<double> AffineCoef(const double angle,
                                       const double t) const;

        std::vector<double> PolyCoef(const double t) const;

        std::vector<double> AffineDerivativeCoef(
            const double angle, const double t) const;

        std::vector<double> AffineDerivativeCoef(const double t) const;

        std::vector<double> AffineSecondDerivativeCoef(const double t) const;

        bool AddInequalityConstraint(const Eigen::MatrixXd& constraint_matrix,
                                     const Eigen::MatrixXd& constraint_boundary);

        bool AddEqualityConstraint(const Eigen::MatrixXd& constraint_matrix,
                                   const Eigen::MatrixXd& constraint_boundary);

        std::vector<double> DerivativeCoef(const double t) const;

        std::vector<double> SecondDerivativeCoef(const double t) const;

        std::vector<double> ThirdDerivativeCoef(const double t) const;

    private:
        AffineConstraint inequality_constraint_;
        AffineConstraint equality_constraint_;
        std::vector<double> t_knots_;
        uint32_t spline_order_;
        uint32_t total_param_;
    };
}

#endif  //PLANNING_SPLINE_2D_CONSTRAINT_H
