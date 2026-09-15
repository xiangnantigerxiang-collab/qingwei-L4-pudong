#ifndef PLANNING_TRAJECTORY_COST_H
#define PLANNING_TRAJECTORY_COST_H

#include <vector>
#include "common/struct_type.h"
#include "common/pnc_point/path_point.h"
#include "common/config/dp_poly_path_config.h"
#include "common/config/vehicle_param_config/vehicle_param_config.h"

using namespace planning;

class TrajectoryCost {
public:
    TrajectoryCost() = default;
    explicit TrajectoryCost(const DpPolyPathConfig &config);
    double Calculate(const SDPoint &cur_point, const std::vector<PathPoint> &reference_line);
    double PathChangeCost(const int pathnumber_last, const int pathnumber_current);

private:
    double CalculatePathCost(const SDPoint &cur_point);
    double CentripetalAccelerationCost(const SDPoint &cur_point, const std::vector<PathPoint> &reference_line);
    double LatComfortCost(const SDPoint &cur_point);
    double calculateDlWeight(const float speed);
    const DpPolyPathConfig config_;
};

// }  // namespace planning

#endif  //PLANNING_TRAJECTORY_COST_H
