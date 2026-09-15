
#ifndef PLANNING_SPLINE_SEG_KERNEL_H
#define PLANNING_SPLINE_SEG_KERNEL_H

#include "Eigen/Dense"
#include <vector>

namespace planning {
    class SplineSegKernel {
    public:
        static SplineSegKernel* instance();
        void destroy();

        Eigen::MatrixXd NthDerivativeKernel(const uint32_t n,
                                            const uint32_t num_params,
                                            const double accumulated_x);

    private:
        SplineSegKernel();
        Eigen::MatrixXd DerivativeKernel(const uint32_t num_of_params,
                                         const double accumulated_x);
        Eigen::MatrixXd SecondOrderDerivativeKernel(const uint32_t num_of_params,
                                                    const double accumulated_x);
        Eigen::MatrixXd ThirdOrderDerivativeKernel(const uint32_t num_of_params,
                                                   const double accumulated_x);

        void IntegratedTermMatrix(const uint32_t num_of_params, const double x,
                                  const std::string& type,
                                  Eigen::MatrixXd* term_matrix) const;

        void CalculateFx(const uint32_t num_of_params);
        void CalculateDerivative(const uint32_t num_of_params);
        void CalculateSecondOrderDerivative(const uint32_t num_of_params);
        void CalculateThirdOrderDerivative(const uint32_t num_of_params);

    private:
        static SplineSegKernel* splineSegKernel_;

        const uint32_t reserved_order_ = 5;
        Eigen::MatrixXd kernel_fx_;
        Eigen::MatrixXd kernel_derivative_;
        Eigen::MatrixXd kernel_second_order_derivative_;
        Eigen::MatrixXd kernel_third_order_derivative_;
    };
}

#endif  //PLANNING_SPLINE_SEG_KERNEL_H
