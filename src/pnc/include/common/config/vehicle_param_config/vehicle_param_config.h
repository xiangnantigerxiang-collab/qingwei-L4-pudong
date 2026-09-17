#ifndef PLANNING_VEHICLE_PARAM_CONFIG_H
#define PLANNING_VEHICLE_PARAM_CONFIG_H

// namespace planning
// {
//use FLU
class VehicleParamConfig {
public:
    double car_length = 3.09;
    double car_width = 1.56;
    double wheel_base = 1.6;
    double rear_axis_to_front = 2.3;
    double rear_axis_to_rear = 0.7;
    double front_edge_to_center = 1.5;
    double back_edge_to_center = 1.5;
    double mini_r = 4.0;
    double gear_ratio = 17;
    double max_steering = 510.0;
    double min_steering = -510.0;
    double steering_offset = 0.0;

    double max_acceleration = 2.0;
    double max_deceleration = -5.0;
    double emergency_brake = -7.0;
};

// 长城车辆参数
// class VehicleParamConfig
// {
// public:

//     double car_length = 3.495;
//     double car_width = 1.660;
//     double wheel_base = 2.475;
//     double rear_axis_to_front = 3.045;
//     double rear_axis_to_rear  = 0.45;
//     double front_edge_to_center = 1.805;
//     double back_edge_to_center = 1.69;
//     double mini_r = 4.1;
//     double gear_ratio = 16.5;
//     double max_steering = 540.0;
//     double min_steering = -540.0;
//     double steering_offset = -4.0;
//     double max_acceleration = 2.0;
//     double max_deceleration = -5.0;
//     double emergency_brake = -7.0;
// };

#endif  //PLANNING_VEHICLE_PARAM_CONFIG_H
