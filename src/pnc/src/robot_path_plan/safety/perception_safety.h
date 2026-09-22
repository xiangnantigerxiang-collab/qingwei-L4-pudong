#ifndef PERCEPTION_SAFETY_H
#define PERCEPTION_SAFETY_H

#include <vector>
#include <cstdint>
#include "robot/perception.h"
#include "lane_motion_filter.h"

namespace planning_perception {
    struct CONFIG_S {
        // 未取得实测外廓前，两种工况均保留原 3.4 m 保护宽度。
        double empty_width = 3.4;
        double loaded_width = 3.4;
        double empty_length = 3.6;
        double loaded_length = 3.6;
        double empty_front = 2.3;
        double loaded_front = 2.3;
        double lateral_margin = 0.15;
        double stop_margin = 1.0;
        double emergency_margin = 0.3;
        double reaction_time = 2.1;
        double static_obstacle_extra_time = 2.0;
        double static_stop_min_distance = 3.0;
        double front_no_confirmation_distance = 6.0;
        double comfort_deceleration = 0.8;
        double emergency_deceleration = 0.8;
        double jerk = 0.8;
        double release_acceleration = 0.3;
        double confirmation_time = 0.2;
        double confirmation_gap = 0.3;
        double coast_time = 0.5;
        double input_timeout = 0.6;
        double clear_time = 0.3;
        double prediction_time = 6.0;
        double prediction_margin_rate = 0.05;
        int confirmation_hits = 3;
        double low_score = 0.35;
    };

    struct PATH_POINT_S {
        double x, y;
        PATH_POINT_S(double tX = 0, double tY = 0)
            : x(tX), y(tY) {
        }
    };

    struct EGO_S {
        double x = 0, y = 0, heading = 0, speed = 0;
        bool loaded = false;
    };

    enum REASON_E { CLEAR = 0,
                    CONFIRMING,
                    SLOWING,
                    EMERGENCY,
                    HOLDING,
                    STALE_INPUT,
                    INVALID_INPUT };

    struct RESULT_S {
        double speed_limit = 0;
        double distance = 10000;
        double ttc = 10000;
        double emergency_distance = 0;
        int object_id = 0;
        int emergency_hits = 0;
        bool immediate_confirmation = false;
        REASON_E reason = CLEAR;
        bool emergency = false;
        std::size_t candidates = 0, axis_tests = 0;
    };

    // Observe 累计真实观测；Evaluate 每个新观测帧只累计一次紧急风险，重复调用不能补票。
    // 固定 ID 表在堆上一次分配，热循环仅遍历活跃 ID，几何在输入时缓存。
    class PerceptionSafety {
    public:
        PerceptionSafety();
        bool Configure(const CONFIG_S& tConfig);
        const CONFIG_S& GetConfig() const {
            return mConfig;
        }
        bool HasInput() const {
            return mStarted;
        }
        bool Observe(const robot::perception& tMessage, double tNow,
                     const LaneMotionFilter& tLaneFilter = LaneMotionFilter());
        RESULT_S Evaluate(const std::vector<PATH_POINT_S>& tPath, const EGO_S& tEgo,
                          double tTargetSpeed, double tNow, bool tForward = false);
        double Lookahead(double tSpeed) const;
        std::size_t WorkingBytes() const;
        void ResetMotion();

    private:
        struct FORWARD_LIMIT_S {
            double approach_speed = 0, hard_speed = 0;
        };
        struct BOX_S {
            double x = 0, y = 0, ux = 1, uy = 0;
            double hx = 0, hy = 0, ex = 0, ey = 0;
        };
        struct TRACK_S {
            BOX_S box;
            double vx = 0, vy = 0, first_seen = 0, last_seen = 0;
            double score = 0;
            std::uint64_t frame = 0;
            int hits = 0;
            bool active = false, confirmed = false, own_lane = false;
            std::uint64_t emergency_frame = 0;
            int emergency_hits = 0;  // 同一目标的连续真实紧急观测，不能用规划循环补票。
        };
        struct OBSERVATION_S {
            BOX_S box;
            double vx = 0, vy = 0, score = 0;
            int id = 0;
            bool own_lane = false;
        };
        struct SEGMENT_S {
            BOX_S box;
            double dx = 0, dy = 0, length = 0, s = 0, time = 0, duration = 0;
            double rotation_margin = 0;
            double min_x = 0, max_x = 0, min_y = 0, max_y = 0;
        };
        CONFIG_S mConfig;
        std::vector<TRACK_S> mTracks;
        std::vector<int> mActiveIds;
        std::vector<std::uint32_t> mValidationMarks;
        std::vector<std::uint32_t> mOutsideMarks;
        std::vector<OBSERVATION_S> mObservations;
        std::vector<SEGMENT_S> mSegments;
        std::uint32_t mValidationGeneration = 0;
        std::uint64_t mFrame = 0, mClearFrame = 0;
        double mFrameTime = -1, mLastEvaluation = -1, mClearSince = -1;
        double mSpeed = 0, mAcceleration = 0;
        double mPathMinX = 0, mPathMaxX = 0, mPathMinY = 0, mPathMaxY = 0;
        bool mStarted = false, mSpeedValid = false, mEmergencyLatched = false;

        static bool ReadBox(const robot::object& tObject, BOX_S& tBox);
        static bool Sweep(const SEGMENT_S& tSegment, const BOX_S& tBox, double tVx, double tVy,
                          double tMargin, double tMaxFraction, double& tEnter, std::size_t& tAxisTests);
        void ClearTracks();
        bool ConfirmEmergency(TRACK_S& tTrack, bool tRisk);
        bool IsNearForwardObstacle(const BOX_S& tBox, std::size_t& tAxisTests) const;
        FORWARD_LIMIT_S ForwardObstacleSpeedLimit(const EGO_S& tEgo, double tTargetSpeed, double tNow) const;
        void Prune(double tNow, bool tNewFrame);
        bool BuildSegments(const std::vector<PATH_POINT_S>& tPath, const EGO_S& tEgo);
        double SmoothLimit(double tLimit, double tTarget, double tMeasured, double tDt, bool tRisk);
    };
}

#endif  // PERCEPTION_SAFETY_H
