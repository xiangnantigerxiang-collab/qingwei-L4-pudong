
#ifndef QP_SPLINE_SMOOTHING_QP_SPLINE_CORRIDOR_H
#define QP_SPLINE_SMOOTHING_QP_SPLINE_CORRIDOR_H

#include "osqp/osqp.h"
#include "Eigen/Dense"
#include <vector>
#include <iostream>

namespace planning {
    class SplineCommon {
    public:
        SplineCommon();

        int prod(const int m, const int n);

        void computeTvec(const double t, const double order, const double r, Eigen::MatrixXd& tvec);

        void computeP(const int n, const int r, const double t1, const double t2, Eigen::MatrixXd& Q);

        void linspace(const double t1, const double t2, const int n, std::vector<double>& vec);

        bool optimizeWithOsqp(
            const size_t kernel_dim, const size_t num_affine_constraint,
            std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
            std::vector<c_int>* P_indptr, std::vector<c_float>* A_data,
            std::vector<c_int>* A_indices, std::vector<c_int>* A_indptr,
            std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds,
            std::vector<c_float>* q, std::vector<c_float>* primal_warm_start,
            OSQPData* data, OSQPWorkspace** work, OSQPSettings* settings);

        double PositionX(const double* paras, const double t);

        double DerivativeX(const double* paras, const double t);

        double SecondDerivativeX(const double* paras, const double t);

        double ThirdDerivativeX(const double* paras, const double t);

        double ComputeCurvature(const double dx, const double d2x,
                                const double dy, const double d2y);

        double ComputeCurvatureDerivative(const double dx, const double d2x,
                                          const double d3x, const double dy,
                                          const double d2y,
                                          const double d3y);

        double ComputeS(const double ts, const double te, const double* para_x, const double* para_y);

    private:
    };
}
#endif  //QP_SPLINE_SMOOTHING_QP_SPLINE_CORRIDOR_H
