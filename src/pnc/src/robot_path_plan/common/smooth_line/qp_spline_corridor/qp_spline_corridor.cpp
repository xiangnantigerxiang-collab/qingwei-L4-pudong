
#include "common/smooth_line/qp_spline_corridor/qp_spline_corridor.h"

namespace planning {
    SplineCommon::SplineCommon() {
    }

    int SplineCommon::prod(const int m, const int n) {
        int res = 1;
        for(int i = m; i <= n; i++)
            res = res * i;
        return res;
    }

    void SplineCommon::computeTvec(const double t, const double order, const double r, Eigen::MatrixXd& tvec) {
        tvec = Eigen::MatrixXd::Zero(1, order + 1);
        for(int i = r; i <= order; i++) {
            tvec(0, i) = prod(i - r + 1, i) * pow(t, i - r);
        }
    }

    void SplineCommon::computeP(const int n, const int r, const double t1, const double t2, Eigen::MatrixXd& Q) {
        Eigen::MatrixXd T = Eigen::MatrixXd::Zero((n - r) * 2 + 1, 1);
        for(int i = 0; i < (n - r) * 2 + 1; i++)
            T(i, 0) = pow(t2, i + 1) - pow(t1, i + 1);
        for(int i = r; i < n + 1; i++) {
            for(int j = i; j < n + 1; j++) {
                double k1 = i - r;
                double k2 = j - r;
                double k = k1 + k2 + 1;
                Q(i, j) = prod(k1 + 1, k1 + r) * prod(k2 + 1, k2 + r) / k * T(k - 1, 0);
                Q(j, i) = Q(i, j);
            }
        }
    }

    void SplineCommon::linspace(const double t1, const double t2, const int n, std::vector<double>& vec) {
        double d = (t2 - t1) / (n - 1);
        for(int i = 0; i < n; i++)
            vec.emplace_back(t1 + i * d);
    }

    bool SplineCommon::optimizeWithOsqp(
        const size_t kernel_dim, const size_t num_affine_constraint,
        std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
        std::vector<c_int>* P_indptr, std::vector<c_float>* A_data,
        std::vector<c_int>* A_indices, std::vector<c_int>* A_indptr,
        std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds,
        std::vector<c_float>* q, std::vector<c_float>* primal_warm_start,
        OSQPData* data, OSQPWorkspace** work, OSQPSettings* settings) {
        data->n = kernel_dim;
        data->m = num_affine_constraint;
        data->P = csc_matrix(data->n, data->n, P_data->size(), P_data->data(),
                             P_indices->data(), P_indptr->data());
        data->q = q->data();
        data->A = csc_matrix(data->m, data->n, A_data->size(), A_data->data(),
                             A_indices->data(), A_indptr->data());
        data->l = lower_bounds->data();
        data->u = upper_bounds->data();

        *work = osqp_setup(data, settings);
        //    osqp_warm_start_x(*work, primal_warm_start->data());

        // Solve Problem
        osqp_solve(*work);

        auto status = (*work)->info->status_val;

        if(status < 0) {
            // std::cout << "failed optimization status:\t" << (*work)->info->status << std::endl;
            return false;
        }

        if(status != 1 && status != 2) {
            // std::cout << "failed optimization status:\t" << (*work)->info->status << std::endl;
            return false;
        }

        return true;
    }
    double SplineCommon::PositionX(const double* paras, const double t) {
        return paras[0] + paras[1] * t + paras[2] * pow(t, 2) + paras[3] * pow(t, 3) + paras[4] * pow(t, 4) + paras[5] * pow(t, 5);
    }

    double SplineCommon::DerivativeX(const double* paras, const double t) {
        return paras[1] + 2 * paras[2] * pow(t, 1) + 3 * paras[3] * pow(t, 2) + 4 * paras[4] * pow(t, 3) + 5 * paras[5] * pow(t, 4);
    }

    double SplineCommon::SecondDerivativeX(const double* paras, const double t) {
        return 2 * paras[2] + 6 * paras[3] * pow(t, 1) + 12 * paras[4] * pow(t, 2) + 20 * paras[5] * pow(t, 3);
    }
    double SplineCommon::ThirdDerivativeX(const double* paras, const double t) {
        return 6 * paras[3] + 24 * paras[4] * pow(t, 1) + 60 * paras[5] * pow(t, 2);
    }

    double SplineCommon::ComputeCurvature(const double dx, const double d2x,
                                          const double dy, const double d2y) {
        const double a = dx * d2y - dy * d2x;
        constexpr double kOrder = 1.5;
        const double b = std::pow(dx * dx + dy * dy, kOrder);
        return a / b;
    }

    double SplineCommon::ComputeCurvatureDerivative(const double dx, const double d2x,
                                                    const double d3x, const double dy,
                                                    const double d2y,
                                                    const double d3y) {
        const double a = dx * d2y - dy * d2x;
        const double b = dx * d3y - dy * d3x;
        const double c = dx * d2x + dy * d2y;
        const double d = dx * dx + dy * dy;

        return (b * d - 3.0 * a * c) / std::pow(d, 3.0);
    }

    double SplineCommon::ComputeS(const double ts, const double te, const double* para_x, const double* para_y) {
        double interval = 0.01;
        double sum = 0;
        for(double t = ts; t < te; t += interval) {
            sum += interval * sqrt(pow(DerivativeX(para_x, t), 2) + pow(DerivativeX(para_y, t), 2));
            //            std::cout<<ts<<" "<<te<<" "<<t<<" "<<sum<<std::endl;
        }
        return sum;
    }

}
