
#ifndef PLANNING_PLANNING_GFLAG_H
#define PLANNING_PLANNING_GFLAG_H
namespace planning {

    const bool FLAGS_enable_nudge_slowdown = true;

    const double FLAGS_planning_upper_speed_limit = 33.33;

    const bool FLAGS_enable_side_vehicle_st_boundary = false;

    const double FLAGS_min_stop_distance_obstacle = 4.0;

    const double FLAGS_stop_line_stop_diatance = 1.0;

    const double FLAGS_follow_min_time_sec = 0.1;

    const double FLAGS_follow_time_buffer = 2.5;

    const double FLAGS_follow_min_distance = 3.0;

    const double FLAGS_yield_distance = 3.0;

    const double FLAGS_yield_distance_pedestrian_bycicle = 5.0;

#define FLAGS_numerical_epsilon 1e-6
#define FLAGS_trajectory_time_length 15
#define FLAGS_trajectory_time_resolution 0.05
#define FLAGS_trajectory_number 11;

#define FLAGS_enable_sqp_solver true
#define FLAGS_default_active_set_eps_num (-1e-7)
#define FLAGS_default_active_set_eps_den 1e-7
#define FLAGS_default_active_set_eps_iter_ref 1e-7
#define FLAGS_default_enable_active_set_debug_info false

#define FLAGS_trajectory_time_min_interval 0.02
#define FLAGS_trajectory_time_max_interval 0.1
#define FLAGS_trajectory_time_high_density_period 1.0
#define FLAGS_enable_follow_accel_constraint true

#define FLAGS_change_lane_speed_relax_percentage 0.05
#define FLAGS_virtual_stop_wall_length 0.1
#define FLAGS_max_stop_speed 0.2
}
#endif  //PLANNING_PLANNING_GFLAG_H
