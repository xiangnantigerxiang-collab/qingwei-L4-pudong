
#ifndef PLANNING_SPLINE_1D_GENERATOR_H
#define PLANNING_SPLINE_1D_GENERATOR_H

#include <cstdint>
#include <vector>
#include <memory>

#include "Eigen/Dense"
#include <qpOASES.hpp>
#include "spline_1d.h"
#include "spline_1d_constraint.h"
#include "spline_1d_kernel.h"

namespace planning{
    class Spline1dGenerator{

    public:
        Spline1dGenerator(const std::vector<double>& x_knots, const uint32_t order);

        void Reset(const std::vector<double>& x_knots, const uint32_t order);

        // add constraint through pss_constraint
        Spline1dConstraint* mutable_spline_constraint();

        // add kernel through pss_kernel
        Spline1dKernel* mutable_spline_kernel();

        // solve
        bool Solve();

        // output
        const Spline1d& spline() const;

    private:
        Spline1d spline_;
        Spline1dConstraint spline_constraint_;
        Spline1dKernel spline_kernel_;

        std::unique_ptr<::qpOASES::SQProblem> sqp_solver_;

        int last_num_constraint_ = 0;
        int last_num_param_ = 0;
        bool last_problem_success_ = false;
    };
}

#endif //PLANNING_SPLINE_1D_GENERATOR_H
