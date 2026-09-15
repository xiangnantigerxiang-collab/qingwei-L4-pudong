#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <functional>

#include "trajectory_cost.h"
#include "common/math/math_utils.h"

TrajectoryCost::TrajectoryCost(const DpPolyPathConfig &config)
    : config_(config) {
}

double TrajectoryCost::CalculatePathCost(const SDPoint &cur_point) {
    static double path_cost = 0.0;

    std::function<float(const float)> quasi_softmax = [this](const float x) {
        const float l0 = this->config_.getDpPolyPathConfig().pathLCostParamL0;
        const float b = this->config_.getDpPolyPathConfig().pathLCostParamB;
        const float k = this->config_.getDpPolyPathConfig().pathLCostParamK;
        return (b + std::exp(-k * (x - l0))) / (1.0 + std::exp(-k * (x - l0)));
    };

    const float l = cur_point.d;

    double l_cost = l * l * config_.getDpPolyPathConfig().pathLCost * 200.0 / quasi_softmax(std::fabs(l));  //zhangyu 20220419

    const float dl = std::fabs(cur_point.d_prime);
    double dl_cost = dl * dl * config_.getDpPolyPathConfig().pathDlCost;

    const float ddl = std::fabs(cur_point.d_pprime);
    double ddl_cost = ddl * ddl * calculateDlWeight(cur_point.s_dot);

    path_cost = l_cost + dl_cost + ddl_cost;

    return path_cost;
}

double TrajectoryCost::CentripetalAccelerationCost(
    const SDPoint &cur_point, const std::vector<PathPoint> &reference_line) {
    // Assumes the vehicle is not obviously deviate from the reference line.
    double centripetal_acc_sum = 0.0;
    double centripetal_acc_sqr_sum = 0.0;
    double s = cur_point.s;
    double v = cur_point.s_dot;

    PathPoint matched_ref_point;
    static int nearest_id = 0;

    for(unsigned int i = 0; i < reference_line.size() - 1; i++) {
        if(reference_line[i].s() < s && reference_line[i + 1].s() >= s) nearest_id = i;
    }

    if(nearest_id >= reference_line.size() - 1) nearest_id = reference_line.size() - 1;
    matched_ref_point = reference_line[nearest_id];

    double kappa = matched_ref_point.kappa();
    double centripetal_acc = v * v * fabs(kappa);
    centripetal_acc_sum = std::fabs(centripetal_acc);
    centripetal_acc_sqr_sum = centripetal_acc * centripetal_acc;

    return centripetal_acc_sqr_sum / centripetal_acc_sum;
}

double TrajectoryCost::LatComfortCost(const SDPoint &cur_point) {
    double dpprime = std::fabs(cur_point.d_pprime) * pow(cur_point.s_dot, 2);
    double dprime = std::fabs(cur_point.d_prime) * std::fabs(cur_point.s_ddot);
    double cost = dpprime + dprime;

    double max_cost = 30 * std::max(max_cost, std::fabs(cost));

    return max_cost;
}

double TrajectoryCost::calculateDlWeight(const float speed) {
    static double m_dlWeight = 0.0;

    if(speed < 3.0)
        m_dlWeight = 675;
    else
        m_dlWeight = 75 * (speed * speed - 1);

    return m_dlWeight;
}

double TrajectoryCost::PathChangeCost(const int pathnumber_last, const int pathnumber_current) {
    double path_cost = std::fabs(pathnumber_last - pathnumber_current) * 5.0e5;
    return path_cost;
}

double TrajectoryCost::Calculate(const SDPoint &cur_point, const std::vector<PathPoint> &reference_line) {
    static double total_cost = 0.0;

    double path_cost = CalculatePathCost(cur_point);
    double centripetal_cost = CentripetalAccelerationCost(cur_point, reference_line);
    double latcomfort_cost = LatComfortCost(cur_point);
    total_cost = path_cost + centripetal_cost + latcomfort_cost;

    return total_cost;
}
