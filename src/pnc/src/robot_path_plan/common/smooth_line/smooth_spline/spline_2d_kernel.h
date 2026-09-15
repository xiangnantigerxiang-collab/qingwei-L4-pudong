
#ifndef PLANNING_SPLINE_2D_KERNEL_H
#define PLANNING_SPLINE_2D_KERNEL_H

#include "Eigen/Dense"
#include "spline_seg_kernel.h"
#include <vector>

namespace planning {
    class Spline2dKernel {
    public:
        Spline2dKernel(const std::vector<double>& t_knots,
                       const uint32_t spline_order);

        void AddDerivativeKernelMatrix(const double weight);
        void AddSecondOrderDerivativeMatrix(const double weight);
        void AddThirdOrderDerivativeMatrix(const double weight);

        void AddRegularization(const double regularization_param);

        const Eigen::MatrixXd kernel_matrix() const;

        const Eigen::MatrixXd offset() const;

    private:
        void AddNthDerivativeKernelMatrix(const uint32_t n, const double weight);
        uint32_t find_index(const double t) const;

    private:
        Eigen::MatrixXd kernel_matrix_;
        Eigen::MatrixXd offset_;
        std::vector<double> t_knots_;
        uint32_t spline_order_;
        uint32_t total_params_;
    };
}

#endif  //PLANNING_SPLINE_2D_KERNEL_H
