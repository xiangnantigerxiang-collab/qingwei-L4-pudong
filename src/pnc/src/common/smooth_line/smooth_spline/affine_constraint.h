

#ifndef PLANNING_AFFINE_CONSTRAINT_H
#define PLANNING_AFFINE_CONSTRAINT_H

#include "Eigen/Dense"
#include <iostream>

namespace planning
{
    class AffineConstraint
    {
    public:
        AffineConstraint() = default;
        explicit AffineConstraint(const bool is_equality);
        explicit AffineConstraint(const Eigen::MatrixXd& constraint_matrix,
                                  const Eigen::MatrixXd& constraint_boundary,
                                  const bool is_equality);

        void SetIsEquality(const double is_equality);

        const Eigen::MatrixXd& constraint_matrix() const;
        const Eigen::MatrixXd& constraint_boundary() const;
        bool AddConstraint(const Eigen::MatrixXd& constraint_matrix,
                           const Eigen::MatrixXd& constraint_boundary);

    private:
        Eigen::MatrixXd constraint_matrix_;
        Eigen::MatrixXd constraint_boundary_;
        bool is_equality_ = true;
    };
}

#endif //PLANNING_AFFINE_CONSTRAINT_H
