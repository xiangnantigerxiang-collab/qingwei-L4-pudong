// 由 path_plan_comply.cpp 包含。固定容量状态 + 连续分离轴检测，不做逐时刻密集采样。
#include "perception_safety.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace planning_perception {
    PerceptionSafety::PerceptionSafety()
        : mTracks(32768), mValidationMarks(32768, 0), mOutsideMarks(32768, 0) {
        mActiveIds.reserve(512);
        mObservations.reserve(512);
        mSegments.reserve(128);
    }

    bool PerceptionSafety::Configure(const CONFIG_S& tConfig) {
        const double positive[] = {tConfig.empty_width, tConfig.loaded_width, tConfig.empty_length,
                                   tConfig.loaded_length, tConfig.empty_front, tConfig.loaded_front, tConfig.stop_margin,
                                   tConfig.emergency_margin, tConfig.reaction_time, tConfig.comfort_deceleration,
                                   tConfig.emergency_deceleration, tConfig.jerk, tConfig.release_acceleration,
                                   tConfig.confirmation_time, tConfig.confirmation_gap, tConfig.coast_time,
                                   tConfig.input_timeout, tConfig.clear_time, tConfig.prediction_time};
        for(std::size_t i = 0; i < sizeof(positive) / sizeof(positive[0]); ++i) {
            if(!std::isfinite(positive[i]) || positive[i] <= 0) return false;
        }
        if(!std::isfinite(tConfig.lateral_margin) || tConfig.lateral_margin < 0 ||
           !std::isfinite(tConfig.static_obstacle_extra_time) || tConfig.static_obstacle_extra_time < 0 ||
           tConfig.static_obstacle_extra_time > 5 ||
           !std::isfinite(tConfig.static_stop_min_distance) || tConfig.static_stop_min_distance < 0 ||
           tConfig.static_stop_min_distance > 10 ||
           !std::isfinite(tConfig.front_no_confirmation_distance) || tConfig.front_no_confirmation_distance < 0 ||
           tConfig.front_no_confirmation_distance > 20 ||
           !std::isfinite(tConfig.prediction_margin_rate) || tConfig.prediction_margin_rate < 0 ||
           !std::isfinite(tConfig.low_score) || tConfig.low_score < 0 || tConfig.low_score > 1 ||
           tConfig.empty_front > tConfig.empty_length || tConfig.loaded_front > tConfig.loaded_length ||
           tConfig.emergency_deceleration < tConfig.comfort_deceleration ||
           tConfig.emergency_margin > tConfig.stop_margin ||
           tConfig.confirmation_hits < 2 || tConfig.confirmation_hits > 20 ||
           tConfig.confirmation_gap > tConfig.input_timeout || tConfig.coast_time > tConfig.input_timeout ||
           tConfig.prediction_time > 10 || tConfig.lateral_margin > 2 ||
           tConfig.prediction_margin_rate > 1 || tConfig.comfort_deceleration > 5 ||
           tConfig.emergency_deceleration > 10 || tConfig.empty_length > 30 || tConfig.loaded_length > 30 ||
           tConfig.empty_width > 10 || tConfig.loaded_width > 10 ||
           tConfig.empty_width < 0.2 || tConfig.loaded_width < 0.2 ||
           tConfig.empty_length < 0.5 || tConfig.loaded_length < 0.5 ||
           tConfig.comfort_deceleration < 0.05 || tConfig.jerk < 0.05 || tConfig.jerk > 10 ||
           tConfig.release_acceleration < 0.05 || tConfig.release_acceleration > 3 ||
           tConfig.reaction_time < 0.05 || tConfig.reaction_time > 5 ||
           tConfig.confirmation_time < 0.05 || tConfig.confirmation_time > 2 ||
           tConfig.confirmation_gap < 0.05 || tConfig.coast_time < 0.05 ||
           tConfig.input_timeout > 5 || tConfig.clear_time > 5 ||
           tConfig.prediction_time < 1 || tConfig.stop_margin > 10) return false;
        mConfig = tConfig;
        ResetMotion();
        return true;
    }

    bool PerceptionSafety::ReadBox(const robot::object& tObject, BOX_S& tBox) {
        if(tObject.polygons.size() != 4 || !std::isfinite(tObject.vx) || !std::isfinite(tObject.vy) ||
           std::hypot(tObject.vx, tObject.vy) > 35) return false;
        for(std::size_t i = 0; i < 4; ++i) {
            if(!std::isfinite(tObject.polygons[i].x) || !std::isfinite(tObject.polygons[i].y) ||
               std::abs(tObject.polygons[i].x) > 1e8 || std::abs(tObject.polygons[i].y) > 1e8) return false;
        }
        const auto& p = tObject.polygons;
        const double ax = p[1].x - p[0].x, ay = p[1].y - p[0].y;
        const double bx = p[3].x - p[0].x, by = p[3].y - p[0].y;
        const double a = std::hypot(ax, ay), b = std::hypot(bx, by);
        if(a < 0.001 || b < 0.001 || a > 50 || b > 50 ||
           !std::isfinite(tObject.dx) || !std::isfinite(tObject.dy) ||
           std::abs(a - tObject.dx) > 0.01 || std::abs(b - tObject.dy) > 0.01 ||
           std::abs(ax * bx + ay * by) > 0.01 * a * b || ax * by - ay * bx <= 0 ||
           std::hypot(p[2].x - p[0].x - ax - bx, p[2].y - p[0].y - ay - by) > 0.02) return false;
        tBox.x = (p[0].x + p[2].x) * 0.5;
        tBox.y = (p[0].y + p[2].y) * 0.5;
        tBox.ux = ax / a;
        tBox.uy = ay / a;
        tBox.hx = a * 0.5;
        tBox.hy = b * 0.5;
        tBox.ex = std::abs(tBox.ux) * tBox.hx + std::abs(tBox.uy) * tBox.hy;
        tBox.ey = std::abs(tBox.uy) * tBox.hx + std::abs(tBox.ux) * tBox.hy;
        return true;
    }

    void PerceptionSafety::ClearTracks() {
        for(std::size_t i = 0; i < mActiveIds.size(); ++i) mTracks[mActiveIds[i]] = TRACK_S();
        mActiveIds.clear();
        mFrameTime = -1;
        ResetMotion();
    }

    void PerceptionSafety::ResetMotion() {
        mSpeedValid = false;
        mAcceleration = 0;
        mLastEvaluation = -1;
    }

    void PerceptionSafety::Prune(double tNow, bool tNewFrame) {
        for(std::size_t i = 0; i < mActiveIds.size();) {
            TRACK_S& track = mTracks[mActiveIds[i]];
            const bool missed = tNewFrame && track.frame != mFrame;
            const bool outside = tNewFrame && mOutsideMarks[mActiveIds[i]] == mValidationGeneration;
            if(outside || tNow - track.last_seen > mConfig.coast_time || (missed && !track.confirmed)) {
                track = TRACK_S();
                mActiveIds[i] = mActiveIds.back();
                mActiveIds.pop_back();
            } else {
                ++i;
            }
        }
    }

    bool PerceptionSafety::Observe(const robot::perception& tMessage, double tNow,
                                  const LaneMotionFilter& tLaneFilter) {
        const double stamp = tMessage.header.stamp.toSec();
        if(!std::isfinite(tNow) || !std::isfinite(stamp) || stamp < 0 || stamp > tNow + 0.01 ||
           tNow - stamp > mConfig.input_timeout) return false;
        if(mFrameTime > tNow + 0.01) ClearTracks();
        if(stamp <= mFrameTime) return false;  // 重复/乱序消息不能刷新时效或累计确认。
        if(++mValidationGeneration == 0) {
            std::fill(mValidationMarks.begin(), mValidationMarks.end(), 0);
            std::fill(mOutsideMarks.begin(), mOutsideMarks.end(), 0);
            ++mValidationGeneration;
        }
        mObservations.clear();
        // 完整校验后才提交；坏 ID/角点不能造成“半帧更新、另一半被认为消失”。
        for(std::size_t i = 0; i < tMessage.objs.size(); ++i) {
            const robot::object& object = tMessage.objs[i];
            // 左二/车道外/左一对向目标忽略；整帧有效后删除该 ID 旧框，不能继续漏检外推。
            if(object.type == 2 || object.type == 3 || object.type == 4 || tLaneFilter.Ignore(object)) {
                if(object.id > 0) {
                    if(mValidationMarks[object.id] == mValidationGeneration) return false;
                    mOutsideMarks[object.id] = mValidationGeneration;
                }
                continue;
            }
            // 与 /perception 入口一致，防止被剔除的细长框通过真实观测通道进入避障。
            if(object.dx > 1.0 && object.dy > 1.0 &&
               (object.dx > 3.0 * object.dy || object.dy > 3.0 * object.dx)) continue;
            // 与原发布尺寸范围及 planning 的退化框过滤一致；正常小框不应让整帧超时。
            if(std::isfinite(object.dx) && std::isfinite(object.dy) &&
               (object.dx < 0.05 || object.dy < 0.05 || object.dx > 4.0 || object.dy > 15.0)) continue;
            OBSERVATION_S observed;
            observed.id = object.id;
            if(observed.id <= 0 || mValidationMarks[observed.id] == mValidationGeneration ||
               mOutsideMarks[observed.id] == mValidationGeneration ||
               !ReadBox(object, observed.box)) return false;
            mValidationMarks[observed.id] = mValidationGeneration;
            observed.vx = object.vx;
            observed.vy = object.vy;
            observed.score = std::isfinite(object.confidence) ? object.confidence : 0;
            observed.own_lane = object.type == 0;
            mObservations.push_back(observed);
        }
        ++mFrame;
        for(std::size_t i = 0; i < mObservations.size(); ++i) {
            const OBSERVATION_S& observed = mObservations[i];
            TRACK_S& track = mTracks[observed.id];
            const double gap = stamp - track.last_seen;
            const double jump_x = observed.box.x - track.box.x;
            const double jump_y = observed.box.y - track.box.y;
            const double jump_limit = 35.0 * std::max(0.0, gap) + 0.5;
            if(!track.active) {
                mActiveIds.push_back(observed.id);
                track.active = true;
                track.first_seen = stamp;
            } else if(gap > mConfig.coast_time || jump_x * jump_x + jump_y * jump_y > jump_limit * jump_limit) {
                track.hits = 0;
                track.emergency_hits = 0;
                track.confirmed = false;
                track.first_seen = stamp;
            }
            if(track.frame + 1 != mFrame || gap > mConfig.confirmation_gap) {
                track.hits = 0;
                track.emergency_hits = 0;
                track.first_seen = stamp;
            }
            track.hits = std::min(track.hits + 1, 100);
            track.box = observed.box;
            track.vx = observed.vx;
            track.vy = observed.vy;
            track.score = observed.score;
            track.own_lane = observed.own_lane;
            track.last_seen = stamp;
            track.frame = mFrame;
            const bool low_score = track.score > 0 && track.score < mConfig.low_score;
            const int hits = mConfig.confirmation_hits + (low_score ? 2 : 0);
            const double span = mConfig.confirmation_time * (low_score ? 2 : 1);
            if(track.hits >= hits && stamp - track.first_seen + 1e-9 >= span) track.confirmed = true;
        }
        mFrameTime = stamp;
        mStarted = true;
        Prune(stamp, true);
        return true;
    }

    double PerceptionSafety::Lookahead(double tSpeed) const {
        const double speed = std::max(0.0, tSpeed);
        double extra_time = 0;
        double minimum_distance = 0;
        for(std::size_t i = 0; i < mActiveIds.size(); ++i) {
            const TRACK_S& track = mTracks[mActiveIds[i]];
            if(track.own_lane && track.vx == 0 && track.vy == 0) {
                extra_time = mConfig.static_obstacle_extra_time;
                minimum_distance = mConfig.static_stop_min_distance;
                break;
            }
        }
        // 检查路径也覆盖提前减速距离，不能限速公式已提前而前视范围仍然过短。
        const double reaction = mConfig.reaction_time + mConfig.comfort_deceleration / mConfig.jerk + extra_time;
        return std::min(60.0, std::max(std::max(8.0, minimum_distance + 4.0),
                                     speed * reaction + speed * speed / (2 * mConfig.comfort_deceleration) + mConfig.stop_margin + 4.0));
    }

    PerceptionSafety::FORWARD_LIMIT_S PerceptionSafety::ForwardObstacleSpeedLimit(
        const EGO_S& tEgo, double tTargetSpeed, double tNow) const {
        FORWARD_LIMIT_S limit;
        limit.approach_speed = limit.hard_speed = std::max(0.0, tTargetSpeed);
        // 无效/失效输入仍由原 Evaluate 的 safety 逻辑处理，本项不产生新 safety。
        if(!mStarted || !std::isfinite(tNow) || tNow < mFrameTime || tNow - mFrameTime > mConfig.input_timeout ||
           !std::isfinite(tEgo.x) || !std::isfinite(tEgo.y) || !std::isfinite(tEgo.heading) ||
           !std::isfinite(tEgo.speed) || tEgo.speed < 0 || tEgo.speed >= 20 ||
           !std::isfinite(tTargetSpeed) || tTargetSpeed <= 0 || tTargetSpeed > 20) return limit;

        const double advance_time = 3.0;   // s，在制动距离之外额外预留 3 秒行驶距离。
        const double slow_distance = 6.0; // m，车头外廓到障碍物近边的前向净距。
        const double stop_distance = 3.0; // m，仅限制目标速度为零，不新增或清除 safety。
        const double slow_speed = 0.4;    // m/s，低速段参考速度，另留时间平滑的收敛余量。
        const double hard_slow_speed = 0.45; // m/s，6 m 处不可超过的上限，严格小于 0.5 m/s。
        const double speed = std::max(tEgo.speed, tTargetSpeed);
        const double deceleration = mConfig.comfort_deceleration;
        const double near_deceleration = slow_speed * slow_speed / (2 * (slow_distance - stop_distance));
        const double near_av = near_deceleration * advance_time;
        const double near_entry_speed = std::sqrt(near_av * near_av + slow_speed * slow_speed) - near_av;
        const double lookahead = slow_distance + speed * advance_time +
                                 std::max(0.0, speed * speed - slow_speed * slow_speed) / (2 * deceleration);
        const double length = tEgo.loaded ? mConfig.loaded_length : mConfig.empty_length;
        const double width = tEgo.loaded ? mConfig.loaded_width : mConfig.empty_width;
        const double front = tEgo.loaded ? mConfig.loaded_front : mConfig.empty_front;
        const double heading = (90 - std::fmod(tEgo.heading, 360.0)) * 3.14159265358979323846 / 180;
        SEGMENT_S forward;
        BOX_S& vehicle = forward.box;
        vehicle.ux = std::cos(heading);
        vehicle.uy = std::sin(heading);
        vehicle.hx = length * 0.5;
        vehicle.hy = width * 0.5;
        vehicle.x = tEgo.x + (front - vehicle.hx) * vehicle.ux;
        vehicle.y = tEgo.y + (front - vehicle.hx) * vehicle.uy;
        forward.dx = lookahead * vehicle.ux;
        forward.dy = lookahead * vehicle.uy;

        for(std::size_t i = 0; i < mActiveIds.size(); ++i) {
            const TRACK_S& track = mTracks[mActiveIds[i]];
            const double age = tNow - track.last_seen;
            if(!track.active || !track.own_lane || age < 0 || age > mConfig.coast_time) continue;
            const double object_speed = std::hypot(track.vx, track.vy);
            const double closing_speed = tEgo.speed - track.vx * vehicle.ux - track.vy * vehicle.uy;
            if(!(closing_speed > 1.0 || object_speed < 0.5)) continue;

            BOX_S box = track.box;
            box.x += track.vx * age;
            box.y += track.vy * age;
            const double along = (box.x - vehicle.x) * vehicle.ux + (box.y - vehicle.y) * vehicle.uy;
            const double extent = std::abs(box.ux * vehicle.ux + box.uy * vehicle.uy) * box.hx +
                                  std::abs(-box.uy * vehicle.ux + box.ux * vehicle.uy) * box.hy;
            if(along + extent < vehicle.hx - 1e-8) continue;
            // 真实外廓沿当前前进方向扫掠，要求横向相交；侧后方目标不能仅凭 type=0 限速。
            // 不复用动态预测相交距离，也不扩展 Evaluate 的路径，保持原 safety 判据不变。
            double enter = 0;
            std::size_t axis_tests = 0;
            if(!Sweep(forward, box, 0, 0, 1e-8, 1, enter, axis_tests)) continue;
            const double distance = enter * lookahead;
            double obstacle_limit = 0;
            double hard_limit = 0;
            if(distance > slow_distance) {
                // d-6 = v*3 + (v²-0.4²)/(2a)，反解提前减速目标，交给时间域 a/jerk 平滑。
                const double av = deceleration * advance_time;
                obstacle_limit = std::max(near_entry_speed, std::sqrt(av * av + slow_speed * slow_speed +
                                                    2 * deceleration * (distance - slow_distance)) - av);
                hard_limit = std::sqrt(hard_slow_speed * hard_slow_speed + 2 * deceleration * (distance - slow_distance));
            } else if(distance > stop_distance) {
                // 6→3 m 的硬上限按 v² 递减；减速目标另预留时间，防止平滑滞后撞上硬上限。
                const double ratio = std::sqrt((distance - stop_distance) / (slow_distance - stop_distance));
                obstacle_limit = std::sqrt(near_av * near_av + 2 * near_deceleration * (distance - stop_distance)) - near_av;
                hard_limit = hard_slow_speed * ratio;
            }
            limit.approach_speed = std::min(limit.approach_speed, obstacle_limit);
            limit.hard_speed = std::min(limit.hard_speed, hard_limit);
            if(limit.hard_speed == 0) break;
        }
        return limit;
    }

    bool PerceptionSafety::BuildSegments(const std::vector<PATH_POINT_S>& tPath, const EGO_S& tEgo) {
        mSegments.clear();
        if(tPath.size() < 2) return false;
        const double length = tEgo.loaded ? mConfig.loaded_length : mConfig.empty_length;
        const double width = tEgo.loaded ? mConfig.loaded_width : mConfig.empty_width;
        const double front = tEgo.loaded ? mConfig.loaded_front : mConfig.empty_front;
        const double offset = front - length * 0.5;
        const double radius = std::hypot(length, width) * 0.5 + std::abs(offset);
        const double speed = std::max(0.25, tEgo.speed);
        const double heading = (90 - std::fmod(tEgo.heading, 360.0)) * 3.14159265358979323846 / 180;
        double last_ux = std::cos(heading), last_uy = std::sin(heading), distance = 0;
        mPathMinX = mPathMinY = std::numeric_limits<double>::infinity();
        mPathMaxX = mPathMaxY = -std::numeric_limits<double>::infinity();
        for(std::size_t i = 1; i < tPath.size() && mSegments.size() < 128; ++i) {
            SEGMENT_S segment;
            segment.dx = tPath[i].x - tPath[i - 1].x;
            segment.dy = tPath[i].y - tPath[i - 1].y;
            segment.length = std::hypot(segment.dx, segment.dy);
            if(!std::isfinite(segment.length) || segment.length > 10 ||
               !std::isfinite(tPath[i - 1].x) || !std::isfinite(tPath[i - 1].y)) return false;
            if(segment.length < 0.01) continue;
            BOX_S& box = segment.box;
            // 起点到参考线的短连接可能横向偏斜；车身此时仍沿真实车头方向，
            // 不能把定位横误差当作车辆瞬间转过几十度。
            box.ux = mSegments.empty() ? last_ux : segment.dx / segment.length;
            box.uy = mSegments.empty() ? last_uy : segment.dy / segment.length;
            box.hx = length * 0.5;
            box.hy = width * 0.5;
            box.x = tPath[i - 1].x + offset * box.ux;
            box.y = tPath[i - 1].y + offset * box.uy;
            box.ex = std::abs(box.ux) * box.hx + std::abs(box.uy) * box.hy;
            box.ey = std::abs(box.uy) * box.hx + std::abs(box.ux) * box.hy;
            // 对相邻切向变化给旋转扫掠留包络，不漏掉分段接头处的车角。
            const double dot = std::max(-1.0, std::min(1.0, box.ux * last_ux + box.uy * last_uy));
            segment.rotation_margin = radius * std::sqrt(std::max(0.0, 2 * (1 - dot)));
            const double margin = segment.rotation_margin + mConfig.lateral_margin +
                                  mConfig.prediction_margin_rate * mConfig.prediction_time + mConfig.coast_time * 0.2;
            segment.min_x = std::min(box.x, box.x + segment.dx) - box.ex - margin;
            segment.max_x = std::max(box.x, box.x + segment.dx) + box.ex + margin;
            segment.min_y = std::min(box.y, box.y + segment.dy) - box.ey - margin;
            segment.max_y = std::max(box.y, box.y + segment.dy) + box.ey + margin;
            mPathMinX = std::min(mPathMinX, segment.min_x);
            mPathMaxX = std::max(mPathMaxX, segment.max_x);
            mPathMinY = std::min(mPathMinY, segment.min_y);
            mPathMaxY = std::max(mPathMaxY, segment.max_y);
            segment.s = distance;
            segment.time = distance / speed;
            segment.duration = segment.length / speed;
            mSegments.push_back(segment);
            distance += segment.length;
            last_ux = box.ux;
            last_uy = box.uy;
        }
        return !mSegments.empty();
    }

    bool PerceptionSafety::Sweep(const SEGMENT_S& tSegment, const BOX_S& tBox, double tVx, double tVy,
                                 double tMargin, double tMaxFraction, double& tEnter, std::size_t& tAxisTests) {
        const BOX_S& ego = tSegment.box;
        const double sx = tBox.x + tVx * tSegment.time - ego.x;
        const double sy = tBox.y + tVy * tSegment.time - ego.y;
        const double dx = tVx * tSegment.duration - tSegment.dx;
        const double dy = tVy * tSegment.duration - tSegment.dy;
        const double axes_x[] = {ego.ux, -ego.uy, tBox.ux, -tBox.uy};
        const double axes_y[] = {ego.uy, ego.ux, tBox.uy, tBox.ux};
        double enter = 0, leave = tMaxFraction;
        for(int i = 0; i < 4; ++i) {
            ++tAxisTests;
            const double ax = axes_x[i], ay = axes_y[i];
            const double radius = std::abs(ax * ego.ux + ay * ego.uy) * ego.hx +
                                  std::abs(-ax * ego.uy + ay * ego.ux) * ego.hy +
                                  std::abs(ax * tBox.ux + ay * tBox.uy) * tBox.hx +
                                  std::abs(-ax * tBox.uy + ay * tBox.ux) * tBox.hy + tMargin;
            const double separation = ax * sx + ay * sy, motion = ax * dx + ay * dy;
            if(std::abs(motion) < 1e-10) {
                if(std::abs(separation) > radius) return false;
                continue;
            }
            double a = (-radius - separation) / motion, b = (radius - separation) / motion;
            if(a > b) std::swap(a, b);
            enter = std::max(enter, a);
            leave = std::min(leave, b);
            if(enter > leave) return false;
        }
        tEnter = enter;
        return true;
    }

    double PerceptionSafety::SmoothLimit(double tLimit, double tTarget, double tMeasured, double tDt, bool tRisk) {
        if(!mSpeedValid) {
            if(!tRisk) return tTarget;
            mSpeed = std::min(tTarget, tMeasured);
            mAcceleration = 0;
            mSpeedValid = true;
        }
        const double target = std::min(tTarget, tLimit);
        // 接近目标时提前收敛减速度，避免“跨过目标后把加速度突然清零”。
        // 时间常数 >= a_max/j，使一阶收敛段的加速度变化也落在 jerk 约束内。
        const double settling_time = std::max(tDt,
                                              std::max(mConfig.comfort_deceleration, mConfig.release_acceleration) / mConfig.jerk);
        const double desired_acc = std::max(-mConfig.comfort_deceleration,
                                            std::min(mConfig.release_acceleration, (target - mSpeed) / settling_time));
        mAcceleration += std::max(-mConfig.jerk * tDt, std::min(mConfig.jerk * tDt, desired_acc - mAcceleration));
        double next = std::max(0.0, mSpeed + mAcceleration * tDt);
        if(target == 0 && next <= 0.5 * mConfig.jerk * tDt * tDt &&
           mAcceleration >= -mConfig.jerk * tDt) {
            next = 0;
            mAcceleration = 0;
        }
        // 只能降低已有业务速度；任务停车/其他限速不能被舒适性斜坡抬高。
        mSpeed = std::min(next, tTarget);
        if(!tRisk && mSpeed >= tTarget - 1e-6) mSpeedValid = false;
        return mSpeed;
    }

    bool PerceptionSafety::ConfirmEmergency(TRACK_S& tTrack, bool tRisk) {
        if(tTrack.frame != mFrame) {
            tTrack.emergency_hits = 0;  // 漏检外推不能延续紧急确认。
            return false;
        }
        if(tTrack.emergency_frame != mFrame) {
            if(tTrack.emergency_frame + 1 != mFrame) tTrack.emergency_hits = 0;
            tTrack.emergency_hits = tRisk ? std::min(tTrack.emergency_hits + 1, 3) : 0;
            tTrack.emergency_frame = mFrame;
        } else if(!tRisk) {
            tTrack.emergency_hits = 0;
        }
        return tRisk && tTrack.emergency_hits >= 3;
    }

    bool PerceptionSafety::IsNearForwardObstacle(const BOX_S& tBox, std::size_t& tAxisTests) const {
        if(mSegments.empty() || mConfig.front_no_confirmation_distance <= 0) return false;
        SEGMENT_S forward = mSegments.front();
        const BOX_S& vehicle = forward.box;
        const double along = (tBox.x - vehicle.x) * vehicle.ux + (tBox.y - vehicle.y) * vehicle.uy;
        const double extent = std::abs(tBox.ux * vehicle.ux + tBox.uy * vehicle.uy) * tBox.hx +
                              std::abs(-tBox.uy * vehicle.ux + tBox.ux * vehicle.uy) * tBox.hy;
        if(along + extent < vehicle.hx - 1e-8) return false;
        // 用当前物理框与车头朝向判断 6 m 范围；不能把动态预测的未来相交距离当成当前距离。
        // 零速度扫掠同时要求横向真实重叠，避免仅凭 type=0 将侧后方目标纳入免确认范围。
        forward.dx = vehicle.ux * mConfig.front_no_confirmation_distance;
        forward.dy = vehicle.uy * mConfig.front_no_confirmation_distance;
        double enter = 0;
        return Sweep(forward, tBox, 0, 0, 1e-8, 1, enter, tAxisTests);
    }

    RESULT_S PerceptionSafety::Evaluate(const std::vector<PATH_POINT_S>& tPath, const EGO_S& tEgo,
                                        double tTargetSpeed, double tNow, bool tForward) {
        RESULT_S result;
        result.speed_limit = std::max(0.0, tTargetSpeed);
        const bool valid_ego = std::isfinite(tEgo.x) && std::isfinite(tEgo.y) && std::isfinite(tEgo.heading) &&
                               std::isfinite(tEgo.speed) && tEgo.speed >= 0 && tEgo.speed < 20 &&
                               std::isfinite(tTargetSpeed) && tTargetSpeed >= 0 && tTargetSpeed <= 20;
        const bool stale = !std::isfinite(tNow) || !mStarted || tNow < mFrameTime ||
                           (mLastEvaluation >= 0 && tNow < mLastEvaluation) ||
                           tNow - mFrameTime > mConfig.input_timeout;
        if(stale || !valid_ego || !BuildSegments(tPath, tEgo)) {
            result.reason = stale ? STALE_INPUT : INVALID_INPUT;
            result.emergency = true;
            result.speed_limit = 0;
            mEmergencyLatched = true;
            mClearSince = -1;
            ResetMotion();
            return result;
        }
        // D 挡按实际时间推进；同一时刻重复计算不应继续减速，周期中断仍沿用原 0.2 s 上限。
        const double dt = mLastEvaluation < 0 ? 0.1 : std::max(tForward ? 0.0 : 0.001, std::min(0.2, tNow - mLastEvaluation));
        if(mLastEvaluation >= 0 && tNow - mLastEvaluation > 0.5) mSpeedValid = false;
        mLastEvaluation = tNow;
        Prune(tNow, false);
        double limit = result.speed_limit;
        const double emergency_distance = tEgo.speed * mConfig.reaction_time +
                                                  tEgo.speed * tEgo.speed / (2 * mConfig.emergency_deceleration) + mConfig.emergency_margin;
        const double comfort_delay = mConfig.reaction_time + mConfig.comfort_deceleration / mConfig.jerk;
        bool risk = false, emergency = false, pending_emergency = false;
        for(std::size_t i = 0; i < mActiveIds.size(); ++i) {
            const int id = mActiveIds[i];
            TRACK_S& track = mTracks[id];
            const double age = std::max(0.0, tNow - track.last_seen);
            BOX_S box = track.box;
            box.x += track.vx * age;
            box.y += track.vy * age;
            const double future_x = box.x + track.vx * mConfig.prediction_time;
            const double future_y = box.y + track.vy * mConfig.prediction_time;
            if(std::max(box.x, future_x) + box.ex < mPathMinX || std::min(box.x, future_x) - box.ex > mPathMaxX ||
               std::max(box.y, future_y) + box.ey < mPathMinY || std::min(box.y, future_y) - box.ey > mPathMaxY) {
                ConfirmEmergency(track, false);
                continue;
            }
            ++result.candidates;
            // 上游已经对小速度归零；非零分量都使用同一个有界时间域，避免阈值舍入不一致。
            const bool moving = track.vx != 0 || track.vy != 0;
            const double stop_distance = track.own_lane && !moving
                                             ? std::max(emergency_distance, mConfig.static_stop_min_distance)
                                             : emergency_distance;
            bool soft_risk = false, hard_risk = false;
            double distance = 10000, ttc = 10000;
            for(std::size_t j = 0; j < mSegments.size(); ++j) {
                const SEGMENT_S& segment = mSegments[j];
                if(soft_risk && segment.s > stop_distance) break;
                if(moving && segment.time > mConfig.prediction_time) break;
                const double fraction = moving ? std::min(1.0, (mConfig.prediction_time - segment.time) / segment.duration) : 1.0;
                const double end_time = segment.time + segment.duration * fraction;
                const double sx = box.x + track.vx * segment.time, sy = box.y + track.vy * segment.time;
                const double ex = box.x + track.vx * end_time, ey = box.y + track.vy * end_time;
                if(std::max(sx, ex) + box.ex < segment.min_x || std::min(sx, ex) - box.ex > segment.max_x ||
                   std::max(sy, ey) + box.ey < segment.min_y || std::min(sy, ey) - box.ey > segment.max_y) continue;
                double enter = 0;
                const double margin = segment.rotation_margin + mConfig.lateral_margin + age * 0.2 +
                                      (moving ? mConfig.prediction_margin_rate * std::min(end_time, mConfig.prediction_time) : 0.0);
                if(!soft_risk && Sweep(segment, box, track.vx, track.vy, margin, fraction, enter, result.axis_tests)) {
                    soft_risk = true;
                    distance = segment.s + enter * segment.length;
                    ttc = segment.time + enter * segment.duration;
                }
                // 普通相交和紧急相交可能在相邻段；不能在首个软余量相交后漏查下一段。
                double hard_enter = 0;
                if(soft_risk && distance <= stop_distance &&
                   Sweep(segment, box, track.vx, track.vy, segment.rotation_margin + 0.05,
                         fraction, hard_enter, result.axis_tests) &&
                   segment.s + hard_enter * segment.length <= stop_distance) hard_risk = true;
                if(hard_risk || (soft_risk && distance > stop_distance)) break;
            }
            const bool immediate_confirmation = hard_risk && track.own_lane && track.frame == mFrame &&
                                                IsNearForwardObstacle(box, result.axis_tests);
            const bool confirmed_emergency = ConfirmEmergency(track, hard_risk) || immediate_confirmation;
            if(!soft_risk) continue;
            risk = true;
            if(distance < result.distance) {
                result.distance = distance;
                result.ttc = ttc;
                result.object_id = id;
                result.emergency_distance = stop_distance;
                result.emergency_hits = track.emergency_hits;
                result.immediate_confirmation = immediate_confirmation;
                result.reason = (hard_risk && !confirmed_emergency) || !track.confirmed ? CONFIRMING : SLOWING;
            }
            if(hard_risk) {
                emergency = emergency || confirmed_emergency;
                pending_emergency = pending_emergency || !confirmed_emergency;
            } else {
                // 上游已将低速噪声归零；静态沿用 vx/vy 均为零的现有判据。
                // 额外预留只作用于本车道的普通减速，不扩大急停距离或解除保持范围。
                const double extra_time = track.own_lane && !moving ? mConfig.static_obstacle_extra_time : 0.0;
                const double av = mConfig.comfort_deceleration * (comfort_delay + extra_time);
                const double usable = std::max(0.0, distance - mConfig.stop_margin);
                const double safe_speed = std::max(0.0, std::sqrt(av * av + 2 * mConfig.comfort_deceleration * usable) - av);
                limit = std::min(limit, safe_speed);
                if(!track.confirmed) limit = std::min(limit, tEgo.speed);
            }
        }
        if(emergency) mEmergencyLatched = true;
        // 仍有紧迫风险（包括新 ID 等待确认）时保持已触发的急停。
        // 风险退到紧急范围及停车余量外后，继续由舒适限速管理；
        // 不能因另一远处目标仍在路径上而永久保持此前的紧急制动。
        if(emergency || pending_emergency || (risk && result.distance <= mConfig.stop_margin + mConfig.emergency_margin)) {
            mClearSince = -1;
        } else if(mClearSince < 0) {
            mClearSince = tNow;
            mClearFrame = mFrame;
        } else if(tNow - mClearSince >= mConfig.clear_time && mFrame >= mClearFrame + 2) {
            mEmergencyLatched = false;
        }
        if(mEmergencyLatched) {
            result.reason = emergency ? EMERGENCY : HOLDING;
            result.emergency = true;
            result.speed_limit = 0;
            mSpeed = 0;
            mAcceleration = 0;
            mSpeedValid = true;
        } else {
            FORWARD_LIMIT_S forward;
            forward.approach_speed = forward.hard_speed = std::max(0.0, tTargetSpeed);
            if(tForward) forward = ForwardObstacleSpeedLimit(tEgo, tTargetSpeed, tNow);
            const bool forward_slowing = forward.approach_speed < tTargetSpeed;
            // 新目标仍走唯一的时间域减速度/jerk 状态，避免叠加两个平滑器或逐帧硬切速度。
            result.speed_limit = SmoothLimit(std::min(limit, forward.approach_speed), std::max(0.0, tTargetSpeed),
                                             tEgo.speed, dt, risk || forward_slowing);
            if(result.speed_limit > forward.hard_speed) {
                // 首次发现过近或跟踪落后时，6 m 低速/3 m 零速上限优先，不能用平滑延后停车。
                // 只压目标速度，不写 emergency、确认票数或 safety 解除状态。
                result.speed_limit = forward.hard_speed;
                mSpeed = result.speed_limit;
                mAcceleration = 0;
            }
            if(forward_slowing && result.reason == CLEAR) result.reason = SLOWING;
            if(pending_emergency) {
                // 前两帧立即限制为当前实际车速的 60%；其他更低速度优先，不新增 safety。
                result.speed_limit = std::min(result.speed_limit, tEgo.speed * 0.6);
                if(result.speed_limit < mSpeed) {
                    mSpeed = result.speed_limit;
                    mAcceleration = 0;
                }
            }
        }
        return result;
    }

    std::size_t PerceptionSafety::WorkingBytes() const {
        return mTracks.capacity() * sizeof(TRACK_S) +
               (mValidationMarks.capacity() + mOutsideMarks.capacity()) * sizeof(std::uint32_t) +
               mActiveIds.capacity() * sizeof(int) + mObservations.capacity() * sizeof(OBSERVATION_S) +
               mSegments.capacity() * sizeof(SEGMENT_S);
    }
}
