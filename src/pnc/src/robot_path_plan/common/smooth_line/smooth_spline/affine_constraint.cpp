
#include "common/smooth_line/smooth_spline/affine_constraint.h"
namespace planning {

    AffineConstraint::AffineConstraint(const bool is_equality)
            : is_equality_(is_equality) {}

    AffineConstraint::AffineConstraint(const Eigen::MatrixXd& constraint_matrix,
                                       const Eigen::MatrixXd& constraint_boundary,
                                       const bool is_equality)
            : constraint_matrix_(constraint_matrix),
              constraint_boundary_(constraint_boundary),
              is_equality_(is_equality) {
        if(constraint_matrix_.cols() != constraint_boundary_.cols())
        {
            std::cout << "Error! Fail to add constraint because constraint matrix rows != constraint boundary rows."<< std::endl;
        }
    }

    void AffineConstraint::SetIsEquality(const double is_equality) {
        is_equality_ = is_equality;
    }

    const Eigen::MatrixXd& AffineConstraint::constraint_matrix() const {
        return constraint_matrix_;
    }

    const Eigen::MatrixXd& AffineConstraint::constraint_boundary() const {
        return constraint_boundary_;
    }

    bool AffineConstraint::AddConstraint(
            const Eigen::MatrixXd& constraint_matrix,
            const Eigen::MatrixXd& constraint_boundary) {
        if (constraint_matrix.rows() != constraint_boundary.rows()) {
            std::cout << "Fail to add constraint because constraint matrix rows != "
                    "constraint boundary rows.";
            std::cout << "constraint matrix rows = " << constraint_matrix.rows();
            std::cout << "constraint boundary rows = " << constraint_boundary.rows();
            return false;
        }

        if (constraint_matrix_.rows() == 0) {
            constraint_matrix_ = constraint_matrix;
            constraint_boundary_ = constraint_boundary;
            return true;
        }
        if (constraint_matrix_.cols() != constraint_matrix.cols()) {
            std::cout     << "constraint_matrix_ cols and constraint_matrix cols do not match.";
            std::cout << "constraint_matrix_.cols() = " << constraint_matrix_.cols();
            std::cout << "constraint_matrix.cols() = " << constraint_matrix.cols();
            return false;
        }
        if (constraint_boundary.cols() != 1) {
            std::cout << "constraint_boundary.cols() should be 1.";
            return false;
        }

        Eigen::MatrixXd n_matrix(constraint_matrix_.rows() + constraint_matrix.rows(),
                                 constraint_matrix_.cols());
        Eigen::MatrixXd n_boundary(
                constraint_boundary_.rows() + constraint_boundary.rows(), 1);

        n_matrix << constraint_matrix_, constraint_matrix;
        n_boundary << constraint_boundary_, constraint_boundary;
        constraint_matrix_ = n_matrix;
        constraint_boundary_ = n_boundary;
        return true;
    }

}  // namespace planning
