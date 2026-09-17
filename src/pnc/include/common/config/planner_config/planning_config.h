
#include "dp_poly_path_config.h"
#include "dp_st_speed_config.h"
#include "navi_obstacle_decider_config.h"
#include "navi_path_decider_config.h"
#include "navi_speed_decider_config.h"
#include "poly_st_speed_config.h"
#include "qp_spline_path_config.h"
#include "qp_st_speed_config.h"
#include <vector>

#ifndef PLANNING_PLANNING_CONFIG_H
#define PLANNING_PLANNING_CONFIG_H

namespace planning {
    class PlanningConfig {
    public:
        enum TaskType {
            DP_POLY_PATH_OPTIMIZER = 0,
            DP_ST_SPEED_OPTIMIZER,
            QP_SPLINE_PATH_OPTIMIZER,
            QP_SPLINE_ST_SPEED_OPTIMIZER,
            PATH_DECIDER,
            SPEED_DECIDER,
            POLY_ST_SPEED_OPTIMIZER,
            NAVI_PATH_DECIDER,
            NAVI_SPEED_DECIDER,
            NAVI_OBSTACLE_DECIDER
        };

        struct ScenarioConfig {
            enum ScenarioType {
                LANE_FOLLOW = 0,  // default scenario
                CHANGE_LANE,
                SIDE_PASS,  // go around an object when it blocks the road
                APPROACH,   // approach to an intersection
                INTERSECTION_STOP_SIGN_FOUR_WAY,
                INTERSECTION_STOP_SIGN_ONE_OR_TWO_WAY,
                INTERSECTION_TRAFFIC_LIGHT_LEFT_TURN,
                INTERSECTION_TRAFFIC_LIGHT_RIGHT_TURN,
                INTERSECTION_TRAFFIC_LIGHT_GO_THROUGH
            };
            ScenarioType scenario_type;

            struct ScenarioTaskConfig {
                std::vector<TaskType> tasks;
                DpPolyPathConfig dp_poly_path_config;
                DpStSpeedConfig dp_st_speed_config;
                QpSplinePathConfig qp_spline_path_config;
                QpStSpeedConfig qp_st_speed_config;
                PolyStSpeedConfig poly_st_speed_config;
                //todo:pathDeciderConfig存疑
                double path_decider_config;

                ScenarioTaskConfig() {
                    tasks.emplace_back(DP_POLY_PATH_OPTIMIZER);
                    tasks.emplace_back(PATH_DECIDER);
                    tasks.emplace_back(QP_SPLINE_PATH_OPTIMIZER);
                    tasks.emplace_back(DP_ST_SPEED_OPTIMIZER);
                    tasks.emplace_back(SPEED_DECIDER),
                        tasks.emplace_back(QP_SPLINE_ST_SPEED_OPTIMIZER);
                    tasks.emplace_back(POLY_ST_SPEED_OPTIMIZER);
                }
            };
            vector<ScenarioTaskConfig> scenario_task_config;
        };

        struct PlannerOnRoadConfig {
            vector<ScenarioConfig::ScenarioType> scenario_type;
        };

        struct PlannerNaviConfig {
            vector<TaskType> task;
            NaviPathDeciderConfig navi_path_decider_config;
            NaviSpeedDeciderConfig navi_speed_decider_config;
            NaviObstacleDeciderConfig navi_obstacle_decider_config;
        };

        enum PlannerType {
            RTK = 0,
            ONROAD,
            OPENSPACE,
            NAVI
        };

        struct StandardPlanningConfig {
            vector<PlannerType> planner_type;
            PlannerOnRoadConfig planner_navi_config;
        };

        struct NavigationPlanningConfig {
            vector<PlannerType> planner_type;
            PlannerNaviConfig planner_navi_config;
        };

        struct planningConfig {
            PlannerType planner_type;
            PlannerNaviConfig planner_navi_config;

            PlannerType rtk_planning_config;
            StandardPlanningConfig standard_planning_config;
            NavigationPlanningConfig navi_planning_config;
        };

        PlanningConfig() = default;

        explicit PlanningConfig(const planningConfig &planning_config)
            : m_planningConfig(planning_config) {
        }

        ~PlanningConfig() = default;

        const planningConfig &getPlaningConfig() const {
            return m_planningConfig;
        }

    private:
        planningConfig m_planningConfig;
    };
}

#endif  //PLANNING_PLANNING_CONFIG_H
