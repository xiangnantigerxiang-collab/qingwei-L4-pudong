#pragma once

#include <vector>
#include <algorithm>
#include <memory>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <utility>
#include <cmath>
#include <list>
#include "common/curve1d/curve1d.h"
#include "common/struct_type.h"
#include "common/math/cartesian_frenet_conversion.h"
#include "common/pnc_point/trajectory_point.h"
#include "common/pnc_point/path_point.h"
#include "../pathmatch/pathmatcher.h"
#include "common/config/planning_gflag/planning_gflag.h"
#include "trajectory_cost.h"
#include "common/config/dp_poly_path_config.h"
#include "common/config/vehicle_param_config/vehicle_param_config.h"

using namespace math;

class TrajectoryCombiner {
public:
    std::vector<std::vector<TrajectoryPoint>> Combine(
        const std::vector<PathPoint>& reference_line,
        const std::vector<std::shared_ptr<Curve1d>>& lon_trajectory_total,
        const std::vector<std::shared_ptr<Curve1d>>& lat_trajectory_total);

    struct DPRoadGraphNode {
        SDPoint sd_point;
        double total_cost;
    };

    double UpdateNode(TrajectoryCost* trajectory_cost,
                      DPRoadGraphNode* cur_node,
                      const std::vector<PathPoint>& reference_line);

    std::vector<std::vector<TrajectoryPoint>> FrenetPathToCartesianPath(
        const std::vector<std::vector<DPRoadGraphNode>>& graph_nodes,
        const std::vector<PathPoint>& reference_line);

    CartesianFrenetConverter* cartesianfrenetconverter;
    PathMatcher* pathmatcher;
};
