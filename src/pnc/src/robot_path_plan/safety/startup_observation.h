#ifndef STARTUP_OBSERVATION_H
#define STARTUP_OBSERVATION_H

#include "perception_safety.h"

namespace planning_perception {
    // 手动切自动后仅 D 挡检查完整真实感知；起步完成后锁定退出，任务切换/再次停车不复位。
    class StartupObservation {
    public:
        struct STATUS_S {
            const char* reason = "inactive";
            bool stop = false;
            int object_id = 0;
            std::size_t objects = 0;
            double perception_age = -1, pose_age = -1, can_age = -1;
        };
        StartupObservation();
        void SetVehicle(bool tAutomatic, bool tLoaded, double tNow, bool tForward = true);
        void SetSpeed(double tSpeed, double tNow);
        void SetPose(double tX, double tY, double tHeading, double tNow);
        void Observe(const robot::perception& tMessage, double tNow, double tTimeout);
        bool MustStop(const CONFIG_S& tConfig, double tNow) const;
        STATUS_S CheckStatus(const CONFIG_S& tConfig, double tNow) const;

    private:
        struct BOX_S {
            double x, y, ux, uy, hx, hy;
            int id;
        };
        static bool ReadBox(const robot::object& tObject, BOX_S& tBox);
        static bool WithinTwoMeters(const BOX_S& tVehicle, const BOX_S& tObject);
        bool mAutomatic = false, mObserving = false, mLoaded = false;
        bool mForward = true;
        bool mPoseValid = false, mFrameValid = false, mSpeedValid = false;
        int mMovingSamples = 0;
        double mArmedTime = -1, mCanTime = -1, mPoseTime = -1, mFrameTime = -1;
        double mSpeedTime = -1;
        double mX = 0, mY = 0, mHeading = 0;
        const char* mInvalidFrameReason = "waiting_perception_planning";
        int mInvalidObjectId = 0;
        std::vector<BOX_S> mBoxes;
    };
}

#endif  // STARTUP_OBSERVATION_H
