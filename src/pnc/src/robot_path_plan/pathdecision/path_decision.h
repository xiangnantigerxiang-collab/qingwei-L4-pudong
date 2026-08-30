#pragma once

#include <iostream>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <tuple>
#include "common/pnc_point/trajectory_point.h"

class PathDecision
{
public:
    PathDecision(const std::vector<std::vector<TrajectoryPoint>> &trajectory);
    ~PathDecision() = default;

    int GetMaxcostPath();
    std::tuple<bool,bool,int> PathSideJudge(double maxnumber);

private:
    std::vector<std::vector<TrajectoryPoint>> path_decision_trajectory_;
    bool left_side_pass = false;
    bool right_side_pass = false;
    int pathcost_max_idex = 0;
};

