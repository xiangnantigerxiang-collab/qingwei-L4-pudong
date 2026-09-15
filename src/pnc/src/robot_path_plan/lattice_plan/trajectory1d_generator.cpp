#include "trajectory1d_generator.h"

// A common function for trajectory bundles generation with
// a given initial state and  end conditions.
typedef std::array<double, 3> State;
typedef std::pair<State, double> Condition;
typedef std::vector<std::shared_ptr<Curve1d>> Trajectory1DBundle;

Trajectory1dGenerator::Trajectory1dGenerator(
    const State& lon_init_state, const State& lat_init_state
    /* std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
    std::shared_ptr<PredictionQuerier> ptr_prediction_querier*/
    )
    : init_lon_state_(lon_init_state),
      init_lat_state_(lat_init_state),
      end_condition_sampler_(lon_init_state, lat_init_state) {
}

/* ptr_path_time_graph_(ptr_path_time_graph)*/

void Trajectory1dGenerator::GenerateTrajectoryBundles(
    const double target_speed,
    Trajectory1DBundle* ptr_lon_trajectory_bundle,
    Trajectory1DBundle* ptr_lat_trajectory_bundle) {
    GenerateSpeedProfilesForCruising(target_speed, ptr_lon_trajectory_bundle);
    GenerateLateralTrajectoryBundle(ptr_lat_trajectory_bundle);
}

void Trajectory1dGenerator::GenerateSpeedProfilesForCruising(
    const double target_speed,
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
    auto end_conditions =
        end_condition_sampler_.SampleLonEndConditionsForCruising(target_speed);

    if(end_conditions.empty()) return;

    // For the cruising case, We use the "QuarticPolynomialCurve1d" class (not the
    // "QuinticPolynomialCurve1d" class) to generate curves. Therefore, we can't
    // invoke the common function to generate trajectory bundles.
    GenerateTrajectory1DBundle<4>(init_lon_state_, end_conditions,
                                  ptr_lon_trajectory_bundle);
}

void Trajectory1dGenerator::GenerateLateralTrajectoryBundle(
    Trajectory1DBundle* ptr_lat_trajectory_bundle) const {
    auto end_conditions = end_condition_sampler_.SampleLatEndConditions();

    // Use the common function to generate trajectory bundles.
    GenerateTrajectory1DBundle<5>(init_lat_state_, end_conditions,
                                  ptr_lat_trajectory_bundle);
}
