#pragma once

#include <iostream>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include "common/pnc_point/trajectory_point.h"
#include "common/pnc_point/path_point.h"
#include "trajectory1d_generator.h"
#include "trajectory_combiner.h"
#include "trajectory_cost.h"
#include "common/math/cartesian_frenet_conversion.h"
#include "../pathmatch/pathmatcher.h"
#include "common/struct_type.h"
#include "common/config/vehicle_param_config/vehicle_param_config.h" 
#include "../collisioncheck/collision_detection.h" 
#include "../pathdecision/path_decision.h"

class LatticePlanner 
{ 
public:
    std::tuple<std::vector<TrajectoryPoint>,std::vector<std::vector<TrajectoryPoint>>,bool,int> PlanOnReferenceLine(
        const TrajectoryPoint& planning_init_point,
        std::vector<OriginalInsData>& reference_line,
        std::vector<sCellMsg> lidarobjs_global,
        int vehiclestatus);

    std::vector<std::vector<TrajectoryPoint>> combined_trajectory;
    std::vector<std::vector<TrajectoryPoint>> checked_trajectory;

private:
    void ComputeInitFrenetState(const PathPoint& matched_point,
                                const TrajectoryPoint& cartesian_state,
                                std::array<double, 3>* ptr_s,
                                std::array<double, 3>* ptr_d);

    std::tuple<std::vector<TrajectoryPoint>,std::vector<std::vector<TrajectoryPoint>>,bool,int> GetMinCostPath(
        const std::vector<std::vector<TrajectoryPoint>> &trajectory,
        std::tuple<bool,bool,int> &path_status,
        int vehiclestatus);

    int CalculateMinNumber(std::vector<std::vector<TrajectoryPoint>> trajectory,
                           std::tuple<bool,bool,int> &path_status,
                           int idex_last,bool changepath);
  
    double speed_limit = 30.0/3.6;
    CartesianFrenetConverter *cartesianfrenetconverter;
    PathMatcher *pathmatcher;
    TrajectoryCombiner *trajectorycombiner;
    VehicleParamConfig vehicle_param;
    std::unique_ptr<TrajectoryCost> trajectorycost; 
};

