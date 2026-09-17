

#ifndef PLANNING_DP_POLY_PATH_CONFIG_H
#define PLANNING_DP_POLY_PATH_CONFIG_H

#include <zconf.h>

class DpPolyPathConfig {
public:
    struct wayPointSamplerConfig {
        u_int32_t sampleMaxPointsNumEachLevel;
        u_int32_t sampleMinPointsNumEachLevel;
        float minLevelInterval;
        float stepLengthMax;
        float stepLengthMIn;
        float lateralSampleOffset;
        float lateralAdjustCoeff;
        float sidePassDistance;
        float planningMaxTime;
        float setMinSpeed;
        wayPointSamplerConfig()
            : sampleMaxPointsNumEachLevel(7),
              sampleMinPointsNumEachLevel(3),
              minLevelInterval(0.2),
              stepLengthMax(100.0),
              stepLengthMIn(10.0),
              lateralSampleOffset(0.5),
              lateralAdjustCoeff(0.5),
              sidePassDistance(2.8),
              planningMaxTime(10.0),
              setMinSpeed(3.0) {
        }
    };

    struct SpeedAndKappaConfig {
        float speed_centric_acceleration_limit;
        float min_reduce_acc;
        float reduce_acc_cost;

        SpeedAndKappaConfig()
            : speed_centric_acceleration_limit(0.8),
              min_reduce_acc(-0.3),
              reduce_acc_cost(2e6) {
        }
    };

    struct CollisionConfig {
        float obstacleIgnoreDistance;
        float obstacleCollisionDistance;
        float obstacleRiskDistance;
        float obstacleRiskDistance2;
        float obstacleCollisionCost;
        float obstacleCollisionCost_by_distance;
        float obstacleNoCollision;
        float obstacleMaxSpeedCollisionCost;
        float obstacleMaxSpeedRiskCollisionCost;
        float obstacleMaxSpeedRiskCollisionCost2;
        CollisionConfig()
            : obstacleIgnoreDistance(20.0),
              obstacleCollisionDistance(0.2),
              obstacleRiskDistance(0.7),
              obstacleRiskDistance2(0.5),
              obstacleCollisionCost(1e7),
              obstacleCollisionCost_by_distance(2e4),
              obstacleNoCollision(0.0),
              obstacleMaxSpeedCollisionCost(0.0),
              obstacleMaxSpeedRiskCollisionCost(8000.0f),
              obstacleMaxSpeedRiskCollisionCost2(8000.0f) {
        }
    };

    struct PathDecisionConfig {
        float haslap_solid_line_cost;

        PathDecisionConfig()
            : haslap_solid_line_cost(1e8) {
        }
    };

    struct Config {
        wayPointSamplerConfig waypoint_sampler_config;
        SpeedAndKappaConfig speedAndKappaConfig;
        PathDecisionConfig pathDecisionConfig;
        CollisionConfig collisionConfig;

        // Trajectory Cost Config
        float evalTimeInterval;
        float pathMinResolution;
        float pathMaxResolution;
        float pathMinResolutionDistance;
        float pathLCost;
        float pathDlCost;
        float pathDdlCost;
        float pathLCostParamL0;
        float pathLCostParamB;
        float pathLCostParamK;
        float pathOutGoalLCost;
        float pathOutLaneCost;
        float pathEndLCost;
        float pathChangeCost;
        Config()
            : evalTimeInterval(0.5),
              pathMinResolution(0.1),
              pathMaxResolution(1.0),
              pathMinResolutionDistance(10.0),
              pathLCost(6.5),
              pathDlCost(3e3),
              pathDdlCost(5e1),
              pathLCostParamL0(1.50),
              pathLCostParamB(0.40),
              pathLCostParamK(1.5),
              pathOutGoalLCost(5e4),
              pathOutLaneCost(1e8),
              pathEndLCost(1.0e4),
              pathChangeCost(1.0e5) {
        }
    };

    DpPolyPathConfig() = default;

    explicit DpPolyPathConfig(const Config &dp_poly_path_config)
        : m_DpPolyPathConfig(dp_poly_path_config) {
    }

    ~DpPolyPathConfig() = default;

    const Config &getDpPolyPathConfig() const {
        return m_DpPolyPathConfig;
    }

    const CollisionConfig &getCollisionConfig() const {
        return m_CollisionConfig;
    }

private:
    Config m_DpPolyPathConfig;
    CollisionConfig m_CollisionConfig;
};

#endif  //PLANNING_DP_POLY_PATH_CONFIG_H
