
#include "common/config/reference_line_config/smooth_reference_line_config.h"
#include "common/smooth_line/smooth_spline/spline_2d_solver.h"
#include "Eigen/Core"
#include "Eigen/LU"

namespace planning
{
    constexpr double kRoadBound = 1e10;

    Spline2dSolver::Spline2dSolver(const std::vector<double>& t_knots,
                                   const uint32_t order)
            : spline_(t_knots, order),
              kernel_(t_knots, order),
              constraint_(t_knots, order) {}

    Spline2dConstraint* Spline2dSolver::mutable_constraint() {
        return &constraint_;
    }

    void Spline2dSolver::Reset(const std::vector<double>& t_knots,
                               const uint32_t order) {
        spline_ = Spline2d(t_knots, order);
        kernel_ = Spline2dKernel(t_knots, order);
        constraint_ = Spline2dConstraint(t_knots, order);
    }

    Spline2dKernel* Spline2dSolver::mutable_kernel() { return &kernel_; }

    Spline2d* Spline2dSolver::mutable_spline() { return &spline_; }

    const Spline2d& Spline2dSolver::spline() const { return spline_; }

    bool Spline2dSolver::Solve() {
        const Eigen::MatrixXd& kernel_matrix = kernel_.kernel_matrix();
        const Eigen::MatrixXd& offset = kernel_.offset();
        const Eigen::MatrixXd& inequality_constraint_matrix =
                constraint_.inequality_constraint().constraint_matrix();
        const Eigen::MatrixXd& inequality_constraint_boundary =
                constraint_.inequality_constraint().constraint_boundary();
        const Eigen::MatrixXd& equality_constraint_matrix =
                constraint_.equality_constraint().constraint_matrix();
        const Eigen::MatrixXd& equality_constraint_boundary =
                constraint_.equality_constraint().constraint_boundary();

        if (kernel_matrix.rows() != kernel_matrix.cols()) {
            return false;
        }

        int num_param = kernel_matrix.rows();
        int num_constraint =
                equality_constraint_matrix.rows() + inequality_constraint_matrix.rows();

        bool use_hotstart =
                last_problem_success_ &&
                (FLAGS_enable_sqp_solver && sqp_solver_ != nullptr &&
                 num_param == last_num_param_ && num_constraint == last_num_constraint_);
        if (!use_hotstart) {
            sqp_solver_.reset(new ::qpOASES::SQProblem(num_param, num_constraint,
                                                       ::qpOASES::HST_UNKNOWN));
            ::qpOASES::Options my_options;
            my_options.enableCholeskyRefactorisation = 10;
            my_options.epsNum = FLAGS_smooth_line_default_active_set_eps_num;
            my_options.epsDen = FLAGS_smooth_line_default_active_set_eps_den;
            my_options.epsIterRef = FLAGS_smooth_line_default_active_set_eps_iter_ref;
            sqp_solver_->setOptions(my_options);
            if (!FLAGS_default_enable_active_set_debug_info) {
                sqp_solver_->setPrintLevel(qpOASES::PL_NONE);
            }
        }
        // definition of qpOASESproblem
        const int kNumOfMatrixElements = kernel_matrix.rows() * kernel_matrix.cols();
        double h_matrix[kNumOfMatrixElements];  // NOLINT
        memset(h_matrix, 0, sizeof h_matrix);

        const int kNumOfOffsetRows = offset.rows();
        double g_matrix[kNumOfOffsetRows];  // NOLINT
        memset(g_matrix, 0, sizeof g_matrix);

        int index = 0;

        for (int r = 0; r < kernel_matrix.rows(); ++r) {
            g_matrix[r] = offset(r, 0);
            for (int c = 0; c < kernel_matrix.cols(); ++c) {
                h_matrix[index++] = kernel_matrix(r, c);
            }
        }
        // search space lower bound and uppper bound
        double lower_bound[num_param];  // NOLINT
        double upper_bound[num_param];  // NOLINT
        memset(lower_bound, 0, sizeof lower_bound);
        memset(upper_bound, 0, sizeof upper_bound);

        const double l_lower_bound_ = -kRoadBound;
        const double l_upper_bound_ = kRoadBound;

        for (int i = 0; i < num_param; ++i) {
            lower_bound[i] = l_lower_bound_;
            upper_bound[i] = l_upper_bound_;
        }

        // constraint matrix construction
        double affine_constraint_matrix[num_param * num_constraint];  // NOLINT
        memset(affine_constraint_matrix, 0, sizeof affine_constraint_matrix);

        double constraint_lower_bound[num_constraint];  // NOLINT
        double constraint_upper_bound[num_constraint];  // NOLINT
        memset(constraint_lower_bound, 0, sizeof constraint_lower_bound);
        memset(constraint_upper_bound, 0, sizeof constraint_upper_bound);

        index = 0;
        for (int r = 0; r < equality_constraint_matrix.rows(); ++r) {
            constraint_lower_bound[r] = equality_constraint_boundary(r, 0);
            constraint_upper_bound[r] = equality_constraint_boundary(r, 0);

            for (int c = 0; c < num_param; ++c) {
                affine_constraint_matrix[index++] = equality_constraint_matrix(r, c);
            }
        }

        const double constraint_upper_bound_ = kRoadBound;
        for (int r = 0; r < inequality_constraint_matrix.rows(); ++r) {
            constraint_lower_bound[r + equality_constraint_boundary.rows()] =
                    inequality_constraint_boundary(r, 0);
            constraint_upper_bound[r + equality_constraint_boundary.rows()] =
                    constraint_upper_bound_;

            for (int c = 0; c < num_param; ++c) {
                affine_constraint_matrix[index++] = inequality_constraint_matrix(r, c);
            }
        }

        // initialize problem
        int max_iter = std::max(FLAGS_default_qp_iteration_num, num_constraint);

        ::qpOASES::returnValue ret;
//        const double start_timestamp = Clock::NowInSeconds();
        if (use_hotstart) {
            ret = sqp_solver_->hotstart(
                    h_matrix, g_matrix, affine_constraint_matrix, lower_bound, upper_bound,
                    constraint_lower_bound, constraint_upper_bound, max_iter);
            if (ret != qpOASES::SUCCESSFUL_RETURN) {
                ret = sqp_solver_->init(h_matrix, g_matrix, affine_constraint_matrix,
                                        lower_bound, upper_bound, constraint_lower_bound,
                                        constraint_upper_bound, max_iter);
            }
        } else {
            ret = sqp_solver_->init(h_matrix, g_matrix, affine_constraint_matrix,
                                    lower_bound, upper_bound, constraint_lower_bound,
                                    constraint_upper_bound, max_iter);
        }
//        const double end_timestamp = Clock::NowInSeconds();

        if (ret != qpOASES::SUCCESSFUL_RETURN) {
            if (ret == qpOASES::RET_MAX_NWSR_REACHED) {
            } else {
            }
            last_problem_success_ = false;
            return false;
        }

        last_problem_success_ = true;
        double result[num_param];  // NOLINT
        memset(result, 0, sizeof result);

        sqp_solver_->getPrimalSolution(result);

        Eigen::MatrixXd solved_params = Eigen::MatrixXd::Zero(num_param, 1);
        for (int i = 0; i < num_param; ++i) {
            solved_params(i, 0) = result[i];
        }

        last_num_param_ = num_param;
        last_num_constraint_ = num_constraint;

        return spline_.set_splines(solved_params, spline_.spline_order());
    }
}