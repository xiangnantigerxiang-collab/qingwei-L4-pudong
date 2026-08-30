
#include "common/smooth_line/smooth_spline/spline_1d_kernel.h"

namespace planning{
    Spline1dKernel::Spline1dKernel(const Spline1d& spline1d)
            : Spline1dKernel(spline1d.x_knots(), spline1d.spline_order()) {}

    Spline1dKernel::Spline1dKernel(const std::vector<double>& x_knots,
                                   const uint32_t spline_order)
            : x_knots_(x_knots), spline_order_(spline_order) {
        total_params_ =
                (x_knots.size() > 1 ? (x_knots.size() - 1) * (1 + spline_order_) : 0);
        kernel_matrix_ = Eigen::MatrixXd::Zero(total_params_, total_params_);
        offset_ = Eigen::MatrixXd::Zero(total_params_, 1);
    }

    void Spline1dKernel::AddRegularization(const double regularized_param) {
        Eigen::MatrixXd id_matrix =
                Eigen::MatrixXd::Identity(kernel_matrix_.rows(), kernel_matrix_.cols());
        id_matrix *=  regularized_param * 2;
        kernel_matrix_ += id_matrix;
    }

    bool Spline1dKernel::AddKernel(const Eigen::MatrixXd& kernel,
                                   const Eigen::MatrixXd& offset,
                                   const double weight) {
        if (kernel.rows() != kernel.cols() ||
            kernel.rows() != kernel_matrix_.rows() || offset.cols() != 1 ||
            offset.rows() != offset_.rows()) {
            return false;
        }
        kernel_matrix_ += kernel;
        kernel_matrix_ *= weight;
        offset_ += offset;
        offset_ *= weight;
        return true;
    }

    bool Spline1dKernel::AddKernel(const Eigen::MatrixXd& kernel,
                                   const double weight) {
        Eigen::MatrixXd offset = Eigen::MatrixXd::Zero(kernel.rows(), 1);
        return AddKernel(kernel, offset, weight);
    }

    Eigen::MatrixXd* Spline1dKernel::mutable_kernel_matrix() {
        return &kernel_matrix_;
    }

    Eigen::MatrixXd* Spline1dKernel::mutable_offset() { return &offset_; }

    const Eigen::MatrixXd& Spline1dKernel::kernel_matrix() const {
        return kernel_matrix_;
    }

    const Eigen::MatrixXd& Spline1dKernel::offset() const { return offset_; }

// build-in kernel methods
    void Spline1dKernel::AddNthDerivativekernelMatrix(const uint32_t n,
                                                      const double weight) {
        const uint32_t num_params = spline_order_ + 1;
        for (uint32_t i = 0; i + 1 < x_knots_.size(); ++i) {
            auto nth_derivative_kernel = SplineSegKernel::instance()->NthDerivativeKernel(
                    n, num_params, x_knots_[i + 1] - x_knots_[i]);
            Eigen::MatrixXd cur_kernel = 2 * nth_derivative_kernel;
            cur_kernel *= weight;
            kernel_matrix_.block(i * num_params, i * num_params, num_params,
                                 num_params) += cur_kernel;
        }
    }

    void Spline1dKernel::AddDerivativeKernelMatrix(const double weight) {
        AddNthDerivativekernelMatrix(1, weight);
    }

    void Spline1dKernel::AddSecondOrderDerivativeMatrix(const double weight) {
        AddNthDerivativekernelMatrix(2, weight);
    }

    void Spline1dKernel::AddThirdOrderDerivativeMatrix(const double weight) {
        AddNthDerivativekernelMatrix(3, weight);
    }

    void Spline1dKernel::AddNthDerivativekernelMatrixForSplineK(
            const uint32_t n, const uint32_t k, const double weight) {
        if (k + 1 >= x_knots_.size()) {
            printf("Cannot add NthDerivativeKernel for spline K because k is out of "
                           "range. k = %d\r\n",k);
            return;
        }
        const uint32_t num_params = spline_order_ + 1;
        auto nth_derivative_kernel = SplineSegKernel::instance()->NthDerivativeKernel(
                n, num_params, x_knots_[k + 1] - x_knots_[k]);
        Eigen::MatrixXd cur_kernel = 2 * nth_derivative_kernel;
        cur_kernel *= weight;
        kernel_matrix_.block(k * num_params, k * num_params, num_params,
                             num_params) += cur_kernel;
    }

    void Spline1dKernel::AddDerivativeKernelMatrixForSplineK(const uint32_t k,
                                                             const double weight) {
        AddNthDerivativekernelMatrixForSplineK(1, k, weight);
    }

    void Spline1dKernel::AddSecondOrderDerivativeMatrixForSplineK(
            const uint32_t k, const double weight) {
        AddNthDerivativekernelMatrixForSplineK(2, k, weight);
    }

    void Spline1dKernel::AddThirdOrderDerivativeMatrixForSplineK(
            const uint32_t k, const double weight) {
        AddNthDerivativekernelMatrixForSplineK(3, k, weight);
    }

    bool Spline1dKernel::AddReferenceLineKernelMatrix(
            const std::vector<double>& x_coord, const std::vector<double>& ref_x,
            const double weight) {
        if (ref_x.size() != x_coord.size()) {
            printf("取消巡航阶段的基准线\r\n");
            return false;
        }

        const uint32_t num_params = spline_order_ + 1;
        for (uint32_t i = 0; i < x_coord.size(); ++i) {
            uint32_t cur_index = FindIndex(x_coord[i]);
            double cur_rel_x = x_coord[i] - x_knots_[cur_index];
            // update offset
            double offset_coef = -2.0 * ref_x[i] * weight;
            for (uint32_t j = 0; j < num_params; ++j) {
                offset_(j + cur_index * num_params, 0) += offset_coef;
                offset_coef *= cur_rel_x;
            }
            // update kernel matrix
            Eigen::MatrixXd ref_kernel(num_params, num_params);

            double cur_x = 1.0;
            std::vector<double> power_x;
            for (uint32_t n = 0; n + 1 < 2 * num_params; ++n) {
                power_x.emplace_back(cur_x);
                cur_x *= cur_rel_x;
            }

            for (uint32_t r = 0; r < num_params; ++r) {
                for (uint32_t c = 0; c < num_params; ++c) {
                    ref_kernel(r, c) = 2.0 * power_x[r + c];
                }
            }

            kernel_matrix_.block(cur_index * num_params, cur_index * num_params,
                                 num_params, num_params) += weight * ref_kernel;
        }
        return true;
    }

    bool Spline1dKernel::addSecDerivativeReferenceLineKernelMatrix(const std::vector<double>& x_coord,
                                                                   const std::vector<double>& ref_x,
                                                                   const double weight)
    {
        if (ref_x.size() != x_coord.size()) {
            printf("取消二阶的基准线过程\r\n");
            return false;
        }

        const uint32_t num_params = spline_order_ + 1;
        for (uint32_t i = 0; i < x_coord.size(); ++i) {
            uint32_t cur_index = FindIndex(x_coord[i]);
            double cur_rel_x = x_coord[i] - x_knots_[cur_index];
            // update offset
            double offset_coef = -2.0 * ref_x[i] * weight;
            for (uint32_t j = 0; j < num_params; ++j) {
                if(j < 2)
                {
                    offset_(j + cur_index * num_params, 0) += 0;
                }
                else
                {
                    offset_(j + cur_index * num_params, 0) += j * (j - 1) * offset_coef;
                    offset_coef *= cur_rel_x;
                }
//                printf("j = %d,offset = %f\r\n", j + cur_index * num_params,  offset_(j + cur_index * num_params, 0));
            }
            // update kernel matrix
            Eigen::MatrixXd ref_kernel(num_params, num_params);

            double cur_x = 1.0;
            std::vector<double> power_x;
            for (uint32_t n = 0; n + 1 < 2 * num_params; ++n) {
                power_x.emplace_back(cur_x);
                cur_x *= cur_rel_x;
            }

            for (uint32_t r = 0; r < num_params; ++r) {
                for (uint32_t c = 0; c < num_params; ++c) {
                    if(r < 2 || c < 2)
                    {
                        ref_kernel(r, c) = 0;
                    }
                    else
                    {
                        ref_kernel(r, c) = 2.0 * r * (r - 1)
                                           * c * (c - 1)
                                           * power_x[r + c - 4];
                    }
                }
            }

            kernel_matrix_.block(cur_index * num_params, cur_index * num_params,
                                 num_params, num_params) += weight * ref_kernel;
        }
        return true;
    }

    uint32_t Spline1dKernel::FindIndex(const double x) const {
        auto upper_bound = std::upper_bound(x_knots_.begin() + 1, x_knots_.end(), x);
        return std::min(static_cast<uint32_t>(x_knots_.size() - 1),
                        static_cast<uint32_t>(upper_bound - x_knots_.begin())) -
               1;
    }

    void Spline1dKernel::AddDistanceOffset(const double weight) {
        for (uint32_t i = 1; i < x_knots_.size(); ++i) {
            const double cur_x = x_knots_[i] - x_knots_[i - 1];
            double pw_x = 2.0 * weight;
            for (uint32_t j = 0; j < spline_order_ + 1; ++j) {
                offset_((i - 1) * (spline_order_ + 1) + j, 0) -= pw_x;
                pw_x *= cur_x;
            }
        }
    }
}
