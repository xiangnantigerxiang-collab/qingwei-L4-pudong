#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <string>
#include "common/config/avoid_plan.h"

using namespace planning;

// Input: planning objective, vehicle kinematic/dynamic constraints,
// Output: sampled ending 1 dimensional states with corresponding time duration.
class EndConditionSampler {
public:
    EndConditionSampler(
        const std::array<double, 3>& init_s, const std::array<double, 3>& init_d
        /* std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
      std::shared_ptr<PredictionQuerier> ptr_prediction_querier*/
    );

    virtual ~EndConditionSampler() = default;

    std::vector<std::pair<std::array<double, 3>, double>> SampleLatEndConditions()
        const;

    std::vector<std::pair<std::array<double, 3>, double>>
    SampleLonEndConditionsForCruising(const double ref_cruise_speed) const;

private:
    std::array<double, 3> init_s_;
    std::array<double, 3> init_d_;
};
