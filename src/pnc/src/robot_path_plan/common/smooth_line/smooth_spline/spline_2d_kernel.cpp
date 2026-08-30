
#include "common/smooth_line/smooth_spline/spline_2d_kernel.h"

namespace planning
{
    Spline2dKernel::Spline2dKernel(const std::vector<double> &t_knots, const uint32_t spline_order)
            : t_knots_(t_knots), spline_order_(spline_order) {
        total_params_ =
                (t_knots_.size() > 1 ? 2 * (t_knots_.size() - 1) * (1 + spline_order_)
                                     : 0);
        kernel_matrix_ = Eigen::MatrixXd::Zero(total_params_, total_params_);
        offset_ = Eigen::MatrixXd::Zero(total_params_, 1);
    }

    const Eigen::MatrixXd Spline2dKernel::kernel_matrix() const {
        Eigen::MatrixXd h = kernel_matrix_;
        h *= 2;
        return h;
    }

    const Eigen::MatrixXd Spline2dKernel::offset() const { return offset_; }

    void Spline2dKernel::AddNthDerivativeKernelMatrix(const uint32_t n,
                                                      const double weight) {
        for (uint32_t i = 0; i + 1 < t_knots_.size(); ++i) {
            const uint32_t num_params = spline_order_ + 1;
            Eigen::MatrixXd cur_kernel =
                    (SplineSegKernel::instance()->NthDerivativeKernel(
                            n, num_params, t_knots_[i + 1] - t_knots_[i]));
            cur_kernel *= weight;
            kernel_matrix_.block(2 * i * num_params, 2 * i * num_params, num_params,
                                 num_params) += cur_kernel;
            kernel_matrix_.block((2 * i + 1) * num_params, (2 * i + 1) * num_params,
                                 num_params, num_params) += cur_kernel;
        }
    }

    void Spline2dKernel::AddRegularization(const double regularization_param) {
        Eigen::MatrixXd id_matrix =
                Eigen::MatrixXd::Identity(kernel_matrix_.rows(), kernel_matrix_.cols());
        id_matrix *= regularization_param;
        kernel_matrix_ += id_matrix;
    }

    void Spline2dKernel::AddDerivativeKernelMatrix(const double weight) {
        AddNthDerivativeKernelMatrix(1, weight);
    }

    void Spline2dKernel::AddSecondOrderDerivativeMatrix(const double weight) {
        AddNthDerivativeKernelMatrix(2, weight);
    }

    void Spline2dKernel::AddThirdOrderDerivativeMatrix(const double weight) {
        AddNthDerivativeKernelMatrix(3, weight);
    }


    uint32_t Spline2dKernel::find_index(const double t) const {
        auto upper_bound = std::upper_bound(t_knots_.begin() + 1, t_knots_.end(), t);
        return std::min(static_cast<uint32_t>(t_knots_.size() - 1),
                        static_cast<uint32_t>(upper_bound - t_knots_.begin())) -
               1;
    }
}
