#include "path_decision.h"

PathDecision::PathDecision(
    const std::vector<std::vector<TrajectoryPoint>> &trajectory)
    : path_decision_trajectory_(trajectory) {
}

int PathDecision::GetMaxcostPath() {
    double path_cost_min = 0.0;

    for(int i = 0; i < path_decision_trajectory_.size(); ++i) {
        if(!path_decision_trajectory_[i].back().safeproperty()) continue;

        double path_cost_ = path_decision_trajectory_[i].back().cost();

        if(path_cost_ > path_cost_min) {
            path_cost_min = path_cost_;
            pathcost_max_idex = i;
        }
    }

    return pathcost_max_idex;
}

std::tuple<bool, bool, int> PathDecision::PathSideJudge(double maxnumber) {
    int number_ = (path_decision_trajectory_.size() - 1) / 2;
    int case_number = 0;

    switch(case_number = maxnumber >= number_ ? 1 : 2) {
        case 1:
            left_side_pass = true;
            break;
        case 2:
            right_side_pass = true;
            break;
        default:
            break;
    }

    return std::forward_as_tuple(left_side_pass, right_side_pass, maxnumber);
}
