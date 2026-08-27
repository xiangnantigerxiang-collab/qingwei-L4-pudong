#include "end_condition_sampler.h"
#include <algorithm>

using State = std::array<double, 3>;
using Condition = std::pair<State, double>;

EndConditionSampler::EndConditionSampler(
    const State& init_s, const State& init_d)
    : init_s_(init_s), init_d_(init_d)
{
    //nop
}

std::vector<Condition> EndConditionSampler::SampleLatEndConditions() const 
{
    std::vector<Condition> end_d_conditions;

    std::array<double, 11> end_d_candidates = {};
    std::array<double, 1> end_s_candidates = {};
 
    end_d_candidates = {-3.0, -2.5, -2.0, -1.5, -1.0, 0.0, 1.0, 1.5, 2.0, 2.5, 3.0};
    end_s_candidates = {4.0};

    for(const auto& s : end_s_candidates) {
        for(const auto& d : end_d_candidates) {
            State end_d_state = {d, 0.0, 0.0};
            end_d_conditions.emplace_back(end_d_state, s);
        }
    }

    return end_d_conditions;
}

std::vector<Condition> EndConditionSampler::SampleLonEndConditionsForCruising(
    const double ref_cruise_speed) const 
{   
    std::vector<Condition> end_s_conditions;
    std::array<double, 1> speed_samples = {ref_cruise_speed};
    std::array<double, 1> time_samples = {2.0};

    for(const auto& time : time_samples) {
        for(const auto& speed : speed_samples) {
            State end_speed_state = {0.0, speed, 0.0};
            end_s_conditions.emplace_back(end_speed_state, time);
        }
    }

    return end_s_conditions;
}
