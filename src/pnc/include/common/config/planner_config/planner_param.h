
#ifndef PLANNING_PLANNER_PARAM_H
#define PLANNING_PLANNER_PARAM_H

namespace planning {
    const double FLAGS_look_forward_time_sec = 12.0;
    const double FLAGS_look_forward_short_distance = 40;
    const double FLAGS_car_distance_from_lane = 10;
    const double FLAGS_st_max_t = 100;
    const double FLAGS_st_max_s = 300;
    const double FLAGS_max_stop_distance_obstacle = 8.0;
    //    const double FLAGS_min_stop_distance_obstacle = 6;
    const double FLAGS_search_back_length = 30.0;

}  //end namespace planning

#endif  //PLANNING_PLANNER_PARAM_H
