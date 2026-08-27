
#ifndef PLANNING_SMOOTH_REFERENCE_LINE_CONFIG_H
#define PLANNING_SMOOTH_REFERENCE_LINE_CONFIG_H

#include <stdint-gcc.h>
#include <math.h>
namespace planning
{
    #define FLAGS_longitudinal_boundary_bound 1

    #define FLAGS_lateral_boundary_bound 0.1

    #define FLAGS_max_point_interval 2.0

    #define FLAGS_max_spline_length 25.0

    #define FLAGS_smooth_max_spline 8

    #define FLAGS_SPLINE_ORDER 6.0

    #define FLAGS_second_derivative_weight 1000.0
    #define FLAGS_third_derivative_weight 500.0
    #define FLAGS_regularization_weight 1e-5

    #define RESOLUTION 0.1

    #define MINDISTANCEFROMTWOPOINTS 0.05

    #define FLAGS_smooth_line_default_active_set_eps_num (-1e-6)
    #define FLAGS_smooth_line_default_active_set_eps_iter_ref 1e-6
    #define FLAGS_smooth_line_default_active_set_eps_den 1e-6
    #define FLAGS_enable_sqp_solver true
    #define FLAGS_default_qp_iteration_num 100000
    #define FLAGS_default_enable_active_set_debug_info false

    #define FLAGS_default_osqp_iteration_num 20000
    #define FLAGS_default_soqp_solver_eps_abs 1e-8
    #define FLAGS_default_soqp_solver_eps_rel 1e-8
    #define FLAGS_default_soqp_solver_verbose false

    const double totalTime = 10;
    const int splineOrder = 5;
    const double corridor = 0.5;
    const double plotCorridor = 2;
    const double lambda  = 100;
    const double spacingDis = 0.1;
}
#endif //PLANNING_SMOOTH_REFERENCE_LINE_H