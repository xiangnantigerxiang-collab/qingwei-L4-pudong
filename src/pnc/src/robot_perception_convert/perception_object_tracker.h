#ifndef PERCEPTION_OBJECT_TRACKER_H
#define PERCEPTION_OBJECT_TRACKER_H

#include <vector>
#include "robot/perception.h"

// 地图坐标系下的匀速 Kalman 跟踪器；节点在工作副本上更新，整帧成功后再提交。
// Update 只写 id/vx/vy/heading，不移动发布框，不改 type/confidence/polygons。
class PerceptionObjectTracker {
public:
    explicit PerceptionObjectTracker(int tNextId = 1);
    void Clear();
    void KeepIdSequence(const PerceptionObjectTracker& tPrevious);
    void Maintain(double tNow);
    // 替换同时间戳时，调用方先回到上一检查点；旧帧仅提供出生 ID 提示，不参与估速。
    void Update(robot::perception& tPerception, double tTimestamp, const robot::perception* tReplacedFrame = nullptr);

private:
    struct TRACK_S {
        int id;
        double x, y, vx, vy;
        double p00, p01, p11;  // 两个坐标轴使用相同的 [位置, 速度] 协方差。
        double dx, dy, heading;
        double first_seen, last_seen, state_time;
        double observed_x, observed_y;
        std::size_t observations;
    };

    std::vector<TRACK_S> mTracks;
    int mNextId;
    double mLastFrameTime;

    void Predict(TRACK_S& tTrack, double tTimestamp) const;
    void Correct(TRACK_S& tTrack, const robot::object& tObject, double tTimestamp) const;
    void WriteMotion(TRACK_S& tTrack, robot::object& tObject) const;
};

#endif  // PERCEPTION_OBJECT_TRACKER_H
