
#ifndef PLANNING_SPLINE_2D_SOLVER_H
#define PLANNING_SPLINE_2D_SOLVER_H

#include <cstdint>
#include <vector>
#include <qpOASES.hpp>

#include "spline_2d_kernel.h"
#include "spline_2d_constraint.h"
#include "spline2d.h"
#include "Eigen/Dense"
#include "memory"

namespace planning {
    class Spline2dSolver {
    public:
        Spline2dSolver(const std::vector<double>& t_knots, const uint32_t order);
        // customize setup
        Spline2dConstraint* mutable_constraint();
        Spline2dKernel* mutable_kernel();
        Spline2d* mutable_spline();

        const Spline2d& spline() const;

        void Reset(const std::vector<double>& t_knots, const uint32_t order);

        bool Solve();

    private:
        Spline2d spline_;
        Spline2dKernel kernel_;
        Spline2dConstraint constraint_;
        std::unique_ptr<::qpOASES::SQProblem> sqp_solver_;

        int last_num_constraint_ = 0;
        int last_num_param_ = 0;
        bool last_problem_success_ = false;
    };

}

#endif  //PLANNING_SPLINE_2D_SOLVER_H
