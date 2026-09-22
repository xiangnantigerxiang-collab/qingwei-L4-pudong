// 由 path_plan_comply.cpp 包含；除共用的左二过滤外，独立于常规避障过滤、确认帧和 ACC 开关。
#include "startup_observation.h"
#include <algorithm>
#include <cmath>

namespace planning_perception {
    StartupObservation::StartupObservation() {
        mBoxes.reserve(512);
    }

    void StartupObservation::SetVehicle(bool tAutomatic, bool tLoaded, double tNow, bool tForward) {
        // 未完成 D 挡起步时切回 D，重新等待当前挡位的新感知；已完成后不因换挡重启观察。
        if(tAutomatic && (!mAutomatic || (mObserving && tForward && !mForward))) {
            mObserving = true;
            mMovingSamples = 0;
            mSpeedValid = false;
            mArmedTime = tNow;
            mFrameTime = -1;
            mFrameValid = false;
            mInvalidFrameReason = "waiting_perception_planning";
            mInvalidObjectId = 0;
            mBoxes.clear();
        } else if(!tAutomatic) {
            mObserving = false;
            mMovingSamples = 0;
        }
        mAutomatic = tAutomatic;
        mForward = tForward;
        mLoaded = tLoaded;
        if(!mForward) mMovingSamples = 0;
        mCanTime = tNow;
    }

    void StartupObservation::SetSpeed(double tSpeed, double tNow) {
        mSpeedValid = std::isfinite(tSpeed) && std::isfinite(tNow);
        if(mObserving) {
            // 导航车速为 float；仅不同时间的导航帧计票，CAN 仍负责挡位/模式时效。
            if(mForward && mSpeedValid && mCanTime >= 0 && tNow >= mCanTime && tNow - mCanTime <= 0.6 &&
               std::abs(tSpeed) > static_cast<double>(0.2f)) {
                if(mMovingSamples > 0 && tNow > mSpeedTime && tNow - mSpeedTime <= 0.6) {
                    mObserving = false;
                    mBoxes.clear();
                } else {
                    mMovingSamples = 1;
                }
            } else {
                mMovingSamples = 0;
            }
        }
        mSpeedTime = tNow;
    }

    void StartupObservation::SetPose(double tX, double tY, double tHeading, double tNow) {
        // 保存原始地图定位，避免作业路径临时改写 mNavData 后与地图障碍框不在同一坐标系。
        mX = tX;
        mY = tY;
        mHeading = tHeading;
        mPoseTime = tNow;
        mPoseValid = std::isfinite(tX) && std::isfinite(tY) && std::isfinite(tHeading) &&
                     std::isfinite(tNow) && std::abs(tX) <= 1e8 && std::abs(tY) <= 1e8;
    }

    bool StartupObservation::ReadBox(const robot::object& tObject, BOX_S& tBox) {
        // 只使用真实物理四角；运动 heading、车道、速度、置信度不代表周边是否有障碍。
        if(tObject.polygons.size() != 4 || !std::isfinite(tObject.dx) || !std::isfinite(tObject.dy)) return false;
        const auto& p = tObject.polygons;
        for(std::size_t i = 0; i < 4; ++i) {
            if(!std::isfinite(p[i].x) || !std::isfinite(p[i].y) ||
               std::abs(p[i].x) > 1e8 || std::abs(p[i].y) > 1e8) return false;
        }
        const double ax = p[1].x - p[0].x, ay = p[1].y - p[0].y;
        const double bx = p[3].x - p[0].x, by = p[3].y - p[0].y;
        const double a = std::hypot(ax, ay), b = std::hypot(bx, by);
        if(a < 0.001 || b < 0.001 || a > 50 || b > 50 ||
           std::abs(a - tObject.dx) > 0.01 || std::abs(b - tObject.dy) > 0.01 ||
           std::abs(ax * bx + ay * by) > 0.01 * a * b || ax * by - ay * bx <= 0 ||
           std::hypot(p[2].x - p[0].x - ax - bx, p[2].y - p[0].y - ay - by) > 0.02) return false;
        tBox = {(p[0].x + p[2].x) * 0.5, (p[0].y + p[2].y) * 0.5,
                ax / a, ay / a, a * 0.5, b * 0.5, tObject.id};
        return true;
    }

    void StartupObservation::Observe(const robot::perception& tMessage, double tNow, double tTimeout) {
        if(!mObserving || !mForward) return;
        const double stamp = tMessage.header.stamp.toSec();
        if(!std::isfinite(tNow) || !std::isfinite(stamp) || stamp < 0 || stamp > tNow + 0.01) {
            mFrameValid = false;
            mInvalidFrameReason = "invalid_perception_time";
            mInvalidObjectId = 0;
            return;
        }
        // 切换前、重复或乱序帧不能放行，也不能刷新时效。时钟回退不能把旧帧视为新数据。
        if(stamp < mArmedTime || stamp <= mFrameTime || tNow - stamp > tTimeout) return;
        mFrameTime = stamp;
        mFrameValid = false;
        mInvalidFrameReason = "invalid_perception_box";
        mInvalidObjectId = 0;
        mBoxes.clear();
        for(const auto& object : tMessage.objs) {
            // /perception/planning 的左二目标不进入起步缓存；其他车道保留原周边观察规则。
            if(object.type == 2) continue;
            BOX_S box;
            if(!ReadBox(object, box)) {
                mInvalidObjectId = object.id;
                return;
            }
            mBoxes.push_back(box);
        }
        // 空帧同样必须经过时间检查；坏帧不能以部分有效目标或空列表放行。
        mFrameValid = true;
    }

    bool StartupObservation::WithinTwoMeters(const BOX_S& tVehicle, const BOX_S& tObject) {
        const double dx = tObject.x - tVehicle.x, dy = tObject.y - tVehicle.y;
        const double axes[4][2] = {{tVehicle.ux, tVehicle.uy}, {-tVehicle.uy, tVehicle.ux},
                                   {tObject.ux, tObject.uy}, {-tObject.uy, tObject.ux}};
        bool overlap = true;
        for(int i = 0; i < 4; ++i) {
            const double ux = axes[i][0], uy = axes[i][1];
            const double r1 = tVehicle.hx * std::abs(ux * tVehicle.ux + uy * tVehicle.uy) +
                              tVehicle.hy * std::abs(-ux * tVehicle.uy + uy * tVehicle.ux);
            const double r2 = tObject.hx * std::abs(ux * tObject.ux + uy * tObject.uy) +
                              tObject.hy * std::abs(-ux * tObject.uy + uy * tObject.ux);
            const double gap = std::abs(dx * ux + dy * uy) - r1 - r2;
            if(gap > 1.0 + 1e-8) return false;
            if(gap > 0) overlap = false;
        }
        if(overlap) return true;
        // 两矩形不相交时，最近距离在某一角点到另一矩形边界处取得。
        // 使用欧氏距离，避免仅给长宽各加 2 m 导致斜角处超出 2 m 的目标也被误拦。
        for(int side = 0; side < 2; ++side) {
            const BOX_S& from = side == 0 ? tVehicle : tObject;
            const BOX_S& to = side == 0 ? tObject : tVehicle;
            for(int sx = -1; sx <= 1; sx += 2) {
                for(int sy = -1; sy <= 1; sy += 2) {
                    const double x = from.x + sx * from.hx * from.ux - sy * from.hy * from.uy - to.x;
                    const double y = from.y + sx * from.hx * from.uy + sy * from.hy * from.ux - to.y;
                    const double ex = std::max(0.0, std::abs(x * to.ux + y * to.uy) - to.hx);
                    const double ey = std::max(0.0, std::abs(-x * to.uy + y * to.ux) - to.hy);
                    if(ex * ex + ey * ey <= 1.0 + 1e-8) return true;
                }
            }
        }
        return false;
    }

    bool StartupObservation::MustStop(const CONFIG_S& tConfig, double tNow) const {
        return CheckStatus(tConfig, tNow).stop;
    }

    StartupObservation::STATUS_S StartupObservation::CheckStatus(const CONFIG_S& tConfig, double tNow) const {
        STATUS_S status;
        if(!mObserving || !mForward) return status;
        status.perception_age = mFrameTime < 0 ? -1 : tNow - mFrameTime;
        status.pose_age = mPoseTime < 0 ? -1 : tNow - mPoseTime;
        status.can_age = mCanTime < 0 ? -1 : tNow - mCanTime;
        status.objects = mBoxes.size();
        status.stop = true;
        // 将原来的合并失效条件分开报告，判定方向不变；未收到消息的年龄用 -1 表示。
        if(!mFrameValid) {
            status.reason = mInvalidFrameReason;
            status.object_id = mInvalidObjectId;
        } else if(!mPoseValid) {
            status.reason = "invalid_or_missing_pose";
        } else if(!mSpeedValid) {
            status.reason = "invalid_navigation_speed";
        } else if(!std::isfinite(tNow) || tNow < mArmedTime || tNow < mCanTime ||
                  tNow < mPoseTime || tNow < mSpeedTime || tNow + 0.01 < mFrameTime) {
            status.reason = "clock_mismatch";
        } else if(tNow - mFrameTime > tConfig.input_timeout) {
            status.reason = "perception_timeout";
        } else if(tNow - mPoseTime > tConfig.input_timeout) {
            status.reason = "pose_timeout";
        } else if(tNow - mCanTime > tConfig.input_timeout) {
            status.reason = "can_timeout";
        } else if(tNow - mSpeedTime > tConfig.input_timeout) {
            status.reason = "navigation_speed_timeout";
        } else {
            status.stop = false;
            status.reason = "clear";
        }
        if(status.stop) return status;
        const double length = mLoaded ? tConfig.loaded_length : tConfig.empty_length;
        const double width = mLoaded ? tConfig.loaded_width : tConfig.empty_width;
        const double front = mLoaded ? tConfig.loaded_front : tConfig.empty_front;
        const double yaw = (90.0 - mHeading) * M_PI / 180.0;
        const double ux = std::cos(yaw), uy = std::sin(yaw);
        const double shift = front - length * 0.5;
        const BOX_S vehicle = {mX + shift * ux, mY + shift * uy, ux, uy, length * 0.5, width * 0.5, 0};
        for(const auto& box : mBoxes) {
            if(WithinTwoMeters(vehicle, box)) {
                status.stop = true;
                status.reason = "nearby_obstacle";
                status.object_id = box.id;
                break;
            }
        }
        return status;
    }
}
