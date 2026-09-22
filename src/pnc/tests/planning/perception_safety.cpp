// 直接编译生产算法；ROS 只提供由真实 .msg 生成的字段/时间类型。
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include "robot/perception.h"
#define private public
#include "../../src/robot_path_plan/safety/perception_safety.h"
#undef private
#include "../../src/robot_path_plan/safety/perception_safety.inc"

using namespace planning_perception;
namespace {
    int checks = 0;
    void Check(bool condition, const char* name) {
        ++checks;
        if(!condition) {
            std::cerr << "FAIL: " << name << '\n';
            std::exit(1);
        }
    }
    robot::object Object(int id, double x, double y, double length = 1.2, double width = 1,
                         double physical_heading = 90, double vx = 0, double vy = 0) {
        robot::object object;
        object.id = id;
        object.x = x;
        object.y = y;
        object.dx = width;
        object.dy = length;
        object.vx = vx;
        object.vy = vy;
        object.confidence = 0.8;
        object.heading = 270;  // 故意与物理框不同；几何只能来自 polygons。
        const double yaw = (180 - physical_heading) * 3.14159265358979323846 / 180;
        const double lx[] = {-width / 2, width / 2, width / 2, -width / 2};
        const double ly[] = {-length / 2, -length / 2, length / 2, length / 2};
        object.polygons.resize(4);
        for(int i = 0; i < 4; ++i) {
            object.polygons[i].x = x + lx[i] * std::cos(yaw) - ly[i] * std::sin(yaw);
            object.polygons[i].y = y + lx[i] * std::sin(yaw) + ly[i] * std::cos(yaw);
        }
        return object;
    }
    robot::perception Frame(double stamp, std::vector<robot::object> objects) {
        robot::perception frame;
        frame.header.stamp = ros::Time(stamp);
        frame.objs = std::move(objects);
        return frame;
    }
    EGO_S Ego(double speed = 0.8) {
        EGO_S ego;
        ego.x = ego.y = 100;
        ego.heading = 90;
        ego.speed = speed;
        return ego;
    }
    std::vector<PATH_POINT_S> Path() {
        std::vector<PATH_POINT_S> path;
        for(int i = 0; i <= 35; ++i) path.emplace_back(100 + i, 100);
        return path;
    }
    void Confirm(PerceptionSafety& safety, const robot::object& object) {
        Check(safety.Observe(Frame(100, {object}), 100), "first real frame accepted");
        Check(safety.Observe(Frame(100.1, {object}), 100.1), "second real frame accepted");
        Check(safety.Observe(Frame(100.2, {object}), 100.2), "third real frame accepted");
    }
    void TestObservationAspectFilter() {
        struct CASE_S {
            float dx, dy;
            bool keep;
        };
        const CASE_S cases[] = {
            {2, 7, false}, {2, 6, true}, {1, 6, true}, {0.8f, 6, true}, {4, 0.8f, true}, {2, 2, true}, {2, std::nextafter(6.0f, 7.0f), false}, {2, std::nextafter(6.0f, 5.0f), true}, {std::nextafter(1.0f, 2.0f), 6, false}};
        for(const auto& item : cases) {
            PerceptionSafety safety;
            const auto object = Object(7, 102, 100, item.dy, item.dx);
            Check(safety.Observe(Frame(100, {object}), 100), "filtered observation is a valid frame, not sensor failure");
            Check(safety.mTracks[7].active == item.keep, "tracked input obeys strict aspect and size boundaries");
            auto result = safety.Evaluate(Path(), Ego(), 3, 100);
            Check(result.emergency == item.keep && result.speed_limit == (item.keep ? 0 : 3.0),
                  "retained front obstacles stop on the first valid observation");
            for(int tick = 1; tick <= 2; ++tick) {
                const double now = 100 + tick * 0.1;
                safety.Observe(Frame(now, {object}), now);
                result = safety.Evaluate(Path(), Ego(), 3, now);
            }
            Check(result.emergency == item.keep, "only retained shapes can confirm an emergency");
        }
        PerceptionSafety history;
        Confirm(history, Object(7, 108, 100));
        const auto elongated = Object(7, 108, 100, 7, 2);
        Check(history.Observe(Frame(100.3, {elongated, Object(8, 110, 100)}), 100.3),
              "mixed frame keeps normal observations");
        Check(history.mTracks[8].active && history.mTracks[7].last_seen == 100.2 &&
                  history.mTracks[7].box.hy < 1,
              "filtered shape never replaces or refreshes a previously confirmed normal observation");
        Check(history.Observe(Frame(100.8, {elongated}), 100.8) && !history.mTracks[7].active,
              "previous normal observation expires under unchanged coast policy despite recurring filtered shapes");
    }
    void TestObservationConfirmation() {
        PerceptionSafety safety;
        auto frame = Frame(100, {Object(7, 108, 100)});
        Check(safety.Observe(frame, 100), "new frame");
        for(int tick = 0; tick < 4; ++tick) {
            const auto result = safety.Evaluate(Path(), Ego(), 3, 100 + tick * 0.1);
            Check(!result.emergency && result.reason == CONFIRMING, "old sample cannot become confirmed or emergency");
            Check(!safety.Observe(frame, 100 + tick * 0.1), "duplicate does not refresh frame");
        }
        Check(safety.Observe(Frame(100.4, {}), 100.4), "empty raw frame");
        Check(safety.mActiveIds.empty(), "one-frame ghost removed on explicit miss");
        Check(!safety.Evaluate(Path(), Ego(), 3, 100.4).emergency, "ghost never latched emergency");
        Check(safety.Observe(Frame(100.5, {Object(7, 108, 100)}), 100.5), "reappearance");
        Check(!safety.mTracks[7].confirmed && safety.mTracks[7].hits == 1, "miss breaks consecutive observations");
        Check(safety.Observe(Frame(100.6, {Object(8, 108, 100)}), 100.6), "different ID");
        Check(safety.mTracks[8].hits == 1 && !safety.mTracks[7].active, "different IDs cannot share votes");

        PerceptionSafety low;
        auto object = Object(17, 108, 100);
        object.confidence = 0.1;
        Confirm(low, object);
        Check(!low.mTracks[17].confirmed, "low detection score requires more evidence");
        low.Observe(Frame(100.3, {object}), 100.3);
        low.Observe(Frame(100.4, {object}), 100.4);
        Check(low.mTracks[17].confirmed, "persistent low-score object remains eligible");
    }
    void TestGeometryAndMotion() {
        PerceptionSafety outside;
        auto object = Object(7, 105, 97.5, 4, 1);
        object.type = 1;
        Confirm(outside, object);
        auto result = outside.Evaluate(Path(), Ego(), 3, 100.2);
        Check(result.reason == CLEAR && !result.emergency && result.speed_limit == 3,
              "true 0.3 m geometry gap stays clear despite wrong motion heading");

        PerceptionSafety passing;
        object = Object(7, 105, 98, 1.2, 1, 90, 6, 0);
        object.type = 1;  // 保留几何测试，不能因左二入口过滤而绕过碰撞计算。
        Confirm(passing, object);
        result = passing.Evaluate(Path(), Ego(), 3, 100.2);
        Check(result.reason == CLEAR && !result.emergency, "side overtaker moving away has no time-space collision");

        PerceptionSafety cut_in;
        object = Object(7, 112, 97, 1.2, 1, 90, 0, 0.8);
        object.type = 1;
        Confirm(cut_in, object);
        result = cut_in.Evaluate(Path(), Ego(2), 3, 100.2);
        Check(result.object_id == 7 && result.reason == SLOWING && !result.emergency,
              "left-lane cut-in predicts collision and slows in advance");

        PerceptionSafety static_block;
        object = Object(7, 111, 100);
        object.type = 0;
        Confirm(static_block, object);
        result = static_block.Evaluate(Path(), Ego(2), 3, 100.2);
        Check(!result.emergency && result.speed_limit < 2 && result.speed_limit > 1.9,
              "confirmed static obstacle begins gentle deceleration instead of old 6.5 m full brake");

        PerceptionSafety urgent;
        for(int tick = 0; tick < 3; ++tick) {
            const double now = 100 + tick * 0.1;
            urgent.Observe(Frame(now, {Object(7, 102.8, 100)}), now);
            result = urgent.Evaluate(Path(), Ego(), 3, now);
            Check(result.emergency, "own-lane frontal imminent collision bypasses confirmation");
        }
        Check(result.speed_limit == 0, "confirmed emergency stops");
        urgent.Observe(Frame(100.3, {}), 100.3);
        Check(urgent.Evaluate(Path(), Ego(0), 3, 100.3).emergency, "emergency release retains coast and hysteresis");
        urgent.Observe(Frame(100.8, {}), 100.8);
        urgent.Evaluate(Path(), Ego(0), 3, 100.8);
        Check(urgent.Evaluate(Path(), Ego(0), 3, 101.2).emergency, "time alone cannot release emergency");
        urgent.Observe(Frame(101.2, {}), 101.2);
        urgent.Evaluate(Path(), Ego(0), 3, 101.2);
        urgent.Observe(Frame(101.3, {}), 101.3);
        Check(!urgent.Evaluate(Path(), Ego(0), 3, 101.3).emergency, "fresh clear frames release emergency");
    }

    void TestLanePolicy() {
        for(int type = 0; type <= 4; ++type) {
            PerceptionSafety safety;
            auto object = Object(7, 102.8, 100);
            object.type = type;
            for(int tick = 0; tick < 3; ++tick) {
                const double now = 100 + tick * 0.1;
                Check(safety.Observe(Frame(now, {object}), now), "all lane types form valid observations");
                const auto result = safety.Evaluate(Path(), Ego(), 3, now);
                if(type >= 2) {
                    Check(result.reason == CLEAR && !result.emergency && result.speed_limit == 3 && result.candidates == 0,
                          "left-second and outside-lane physical overlap is ignored without geometry work");
                } else {
                    Check(result.emergency == (type == 0 || tick == 2), "only own-lane frontal risk bypasses confirmation");
                    Check(std::abs(result.speed_limit - (type == 0 || tick == 2 ? 0 : 0.48)) < 1e-9,
                          "other lanes retain pending speed cap and three-frame confirmation");
                }
            }
        }
        PerceptionSafety safety;
        Confirm(safety, Object(7, 105, 100));
        auto outside = Object(7, 102.8, 100);
        outside.type = 4;
        auto bad = Object(8, 102.8, 100);
        bad.polygons.clear();
        Check(!safety.Observe(Frame(100.3, {outside, bad}), 100.3) && safety.mTracks[7].active,
              "bad frame cannot partially remove old tracks through outside classification");
        Check(safety.Observe(Frame(100.3, {outside}), 100.3) && !safety.mTracks[7].active,
              "valid outside classification immediately removes confirmed cached geometry");
        Check(safety.Evaluate(Path(), Ego(), 3, 100.3).reason == CLEAR, "outside ID cannot coast as its previous inside box");
        outside.id = -1;
        outside.polygons.clear();
        outside.vx = std::numeric_limits<double>::quiet_NaN();
        Check(safety.Observe(Frame(100.4, {outside}), 100.4), "ignored outside geometry is not a bad primary frame");
        for(int tick = 0; tick < 2; ++tick) {
            const double now = 100.5 + tick * 0.1;
            safety.Observe(Frame(now, {Object(7, 102.8, 100)}), now);
            Check(safety.Evaluate(Path(), Ego(), 3, now).emergency, "fresh own-lane frontal reentry also bypasses confirmation");
        }
    }

    void TestLaneMotionDirection() {
        auto object = Object(7, 103, 100);
        object.type = 1;
        struct DIRECTION_S {
            double offset;
            MOTION_DIRECTION_E expected;
        };
        const DIRECTION_S directions[] = {
            {0, SAME_DIRECTION}, {44, SAME_DIRECTION}, {46, UNDETERMINED}, {90, UNDETERMINED},
            {134, UNDETERMINED}, {136, ONCOMING}, {180, ONCOMING}, {224, ONCOMING},
            {226, UNDETERMINED}, {270, UNDETERMINED}, {314, UNDETERMINED}, {316, SAME_DIRECTION}};
        for(double heading : {-720.0, -90.0, 0.0, 30.0, 90.0, 180.0, 270.0, 359.0, 360.0, 450.0}) {
            const LaneMotionFilter filter(heading);
            for(const auto& direction : directions) {
                const double angle = (heading + direction.offset) * 3.14159265358979323846 / 180;
                object.vx = 2 * std::sin(angle);
                object.vy = 2 * std::cos(angle);
                object.heading = heading;  // 旧 heading 与速度矛盾时也只读真实速度。
                Check(filter.Classify(object) == direction.expected, "absolute velocity direction is rotation and wrap invariant");
                Check(filter.Ignore(object) == (direction.expected == ONCOMING), "only definite oncoming motion is ignored");
            }
        }
        const LaneMotionFilter north(0);
        object.vx = 0;
        for(float speed : {0.0f, 0.19f, std::nextafter(0.2f, 0.0f), 0.2f, 1.0f, 35.0f, 36.0f}) {
            object.vy = -speed;
            Check(north.Ignore(object) == (speed >= 0.2f && speed <= 35), "dynamic threshold and invalid speed guard");
        }
        object.vx = 1;
        object.vy = -1;
        Check(north.Classify(object) == UNDETERMINED, "exact diagonal boundary is retained");
        object.vy = -std::nextafter(1.0f, 2.0f);
        Check(north.Ignore(object), "just inside oncoming cone is ignored");
        object.vy = -std::nextafter(1.0f, 0.0f);
        Check(!north.Ignore(object), "just outside oncoming cone is retained");
        object.vx = 0;
        object.vy = -1;
        Check(!LaneMotionFilter().Ignore(object), "no direction cannot exclude an obstacle");
        const double invalid = std::numeric_limits<double>::quiet_NaN();
        Check(!LaneMotionFilter(invalid).Ignore(object), "invalid ego heading cannot exclude an obstacle");
        Check(!LaneMotionFilter(std::numeric_limits<double>::infinity()).Ignore(object), "infinite ego heading is unknown");
        for(int type = 0; type <= 5; ++type) {
            object.type = type;
            Check(north.Ignore(object) == (type == 1), "direction policy applies only to left first lane");
        }
        object.type = 1;
        object.vx = invalid;
        Check(!north.Ignore(object), "nonfinite velocity is retained for normal validation");
        object.vx = 0;
        object.vy = std::numeric_limits<float>::infinity();
        Check(!north.Ignore(object), "infinite target velocity is not treated as oncoming");
    }

    void TestLeftLaneObservationPolicy() {
        const LaneMotionFilter filter(90);
        struct CASE_S {
            int type;
            double vx, vy;
            bool ignored;
        };
        const CASE_S cases[] = {
            {0, -2, 0, false}, {1, -2, 0, true}, {2, -2, 0, true}, {1, 0.5, 0, false},
            {1, 0, 0, false}, {1, -0.19, 0, false}, {1, 0, 2, false}, {1, -1, 2, false}, {1, -2, 1, true}};
        for(const auto& item : cases) {
            PerceptionSafety safety;
            auto object = Object(7, 102.8, 100, 1.2, 1, 90, item.vx, item.vy);
            object.type = item.type;
            for(int tick = 0; tick < 3; ++tick) {
                const double now = 100 + tick * 0.1;
                Check(safety.Observe(Frame(now, {object}), now, filter), "direction-filtered frames remain valid input");
                const auto result = safety.Evaluate(Path(), Ego(), 3, now);
                Check(safety.mTracks[7].active != item.ignored, "left-second or left-first oncoming target is removed from tracks");
                if(item.ignored) {
                    Check(result.reason == CLEAR && result.candidates == 0 && result.speed_limit == 3 && !result.emergency,
                          "oncoming left target cannot cause either slowing or emergency even when boxes overlap");
                } else {
                    Check(result.emergency == (item.type == 0 || tick == 2), "own-lane front stops immediately while other lane hazards still confirm");
                }
            }
        }
        auto previous = Object(7, 102.8, 100);
        previous.type = 1;
        auto oncoming = previous;
        oncoming.vx = -2;
        PerceptionSafety history;
        Confirm(history, previous);
        auto bad = Object(8, 102.8, 100);
        bad.polygons.clear();
        Check(!history.Observe(Frame(100.3, {oncoming, bad}), 100.3, filter) && history.mTracks[7].active,
              "bad frame cannot partially discard a previously confirmed track");
        Check(!history.Observe(Frame(100.1, {oncoming}), 100.3, filter) && history.mTracks[7].active,
              "out-of-order oncoming evidence cannot discard current tracks");
        for(const auto& objects : {std::vector<robot::object>{oncoming, previous}, std::vector<robot::object>{previous, oncoming}}) {
            Check(!history.Observe(Frame(100.3, objects), 100.3, filter) && history.mTracks[7].active,
                  "conflicting duplicate IDs cannot silently remove a real obstacle in either order");
        }
        Check(history.Observe(Frame(100.3, {oncoming}), 100.3, filter) && !history.mTracks[7].active,
              "accepted oncoming classification removes old confirmed track immediately without coasting");
        Check(history.mActiveIds.empty() && history.mTracks[7].emergency_hits == 0 && history.HasInput(),
              "ignored target leaves no cached geometry or votes and preserves stream activation");
        const auto clear = history.Evaluate(Path(), Ego(), 3, 100.3);
        Check(clear.reason == CLEAR && clear.speed_limit == 3, "old confirmed geometry cannot slow the route after removal");
        Check(history.Observe(Frame(100.8, {oncoming}), 100.8, filter) && history.mFrameTime == 100.8,
              "all-filtered frame refreshes observation heartbeat");
        Check(history.Evaluate(Path(), Ego(), 3, 100.8).reason == CLEAR, "all-filtered active stream does not time out");
        for(int tick = 0; tick < 3; ++tick) {
            const double now = 100.9 + tick * 0.1;
            history.Observe(Frame(now, {previous}), now, filter);
            Check(history.Evaluate(Path(), Ego(), 3, now).emergency == (tick == 2),
                  "ID returning as a stationary obstacle needs fresh emergency evidence");
        }
        // 已有急停仍按清场迟滞解除，不能借剔除一个目标抹掉其他目标风险。
        const auto other = Object(8, 102.8, 100);
        for(int tick = 0; tick < 5; ++tick) {
            const double now = 101.2 + tick * 0.1;
            history.Observe(Frame(now, {oncoming, other}), now, filter);
            Check(history.Evaluate(Path(), Ego(), 3, now).emergency, "removing left oncoming target never releases another urgent hazard");
        }
        RESULT_S released;
        for(int tick = 0; tick < 10; ++tick) {
            const double now = 101.7 + tick * 0.1;
            history.Observe(Frame(now, {oncoming}), now, filter);
            released = history.Evaluate(Path(), Ego(0), 3, now);
        }
        Check(!released.emergency, "fresh filtered frames eventually release old emergency by existing clear hysteresis");

        PerceptionSafety invalid;
        oncoming.vx = std::numeric_limits<float>::quiet_NaN();
        Check(!invalid.Observe(Frame(100, {oncoming}), 100, filter), "invalid left-lane velocity still fails observation validation");
        oncoming.vx = -36;
        Check(!invalid.Observe(Frame(100, {oncoming}), 100, filter), "out-of-range oncoming speed still fails observation validation");
    }

    void TestLeftSecondLaneHistory() {
        for(double vx : {-2.0, 0.0, 2.0}) {
            for(double vy : {-2.0, 0.0, 2.0}) {
                PerceptionSafety safety;
                auto object = Object(7, 102.8, 100, 1.2, 1, 90, vx, vy);
                object.type = 2;
                Check(safety.Observe(Frame(100, {object}), 100), "left-second frame is valid without a known travel direction");
                const auto result = safety.Evaluate(Path(), Ego(), 3, 100, true);
                Check(!safety.mTracks[7].active && result.candidates == 0 && result.axis_tests == 0 &&
                          !result.emergency && result.speed_limit == 3,
                      "stationary, crossing, same-way and oncoming left-second targets never reach collision work");
            }
        }

        PerceptionSafety safety;
        auto inside = Object(7, 102.8, 100);
        inside.type = 1;
        Confirm(safety, inside);
        auto removed = inside;
        removed.type = 2;
        auto bad = Object(8, 102.8, 100);
        bad.polygons.clear();
        Check(!safety.Observe(Frame(100.3, {removed, bad}), 100.3) && safety.mTracks[7].confirmed,
              "invalid retained object prevents partial deletion of left-second history");
        Check(!safety.Observe(Frame(100.2, {removed}), 100.3) && safety.mTracks[7].confirmed,
              "duplicate timestamp cannot delete an old track");
        for(const auto& objects : {std::vector<robot::object>{removed, inside}, std::vector<robot::object>{inside, removed}}) {
            Check(!safety.Observe(Frame(100.3, objects), 100.3) && safety.mTracks[7].confirmed,
                  "conflicting IDs preserve the original track regardless of input order");
        }
        Check(safety.Observe(Frame(100.3, {removed}), 100.3) && !safety.mTracks[7].active &&
                  safety.mTracks[7].hits == 0 && safety.mTracks[7].emergency_hits == 0,
              "valid left-second classification removes old geometry and confirmation votes immediately");
        Check(safety.Evaluate(Path(), Ego(), 3, 100.3).reason == CLEAR,
              "removed track cannot coast as its old left-first geometry");
        removed.polygons.clear();
        removed.vx = std::numeric_limits<float>::quiet_NaN();
        Check(safety.Observe(Frame(100.4, {removed}), 100.4), "excluded geometry does not invalidate the retained frame");
        Check(safety.mFrameTime == 100.4 && safety.Evaluate(Path(), Ego(), 3, 100.4).reason == CLEAR,
              "fully excluded valid frame refreshes stream health");

        for(int tick = 0; tick < 3; ++tick) {
            const double now = 100.5 + tick * 0.1;
            safety.Observe(Frame(now, {inside}), now);
            Check(safety.Evaluate(Path(), Ego(), 3, now).emergency == (tick == 2),
                  "left-first reentry requires fresh confirmation instead of cached votes");
        }
        safety.Observe(Frame(100.8, {removed}), 100.8);
        Check(!safety.mTracks[7].active && safety.Evaluate(Path(), Ego(0), 3, 100.8).emergency,
              "track removal preserves the existing global emergency release hysteresis");
        auto other = Object(8, 102.8, 100);
        safety.Observe(Frame(100.9, {removed, other}), 100.9);
        const auto blocked = safety.Evaluate(Path(), Ego(), 3, 100.9);
        Check(blocked.emergency && safety.mTracks[8].active && !safety.mTracks[7].active,
              "left-second filtering cannot remove another ID's safety source");
    }

    void TestEmergencyEvidence() {
        auto near = Object(7, 103.29, 100);
        near.type = 1;  // 非本车道目标保留三帧策略，继续验证旧时间戳/漏检/低分数规则。
        PerceptionSafety ghost;
        ghost.Observe(Frame(100, {near}), 100);
        Check(!ghost.Evaluate(Path(), Ego(), 3, 100).emergency, "single urgent observation is not an emergency");
        ghost.Observe(Frame(100.1, {}), 100.1);
        const auto clear = ghost.Evaluate(Path(), Ego(), 3, 100.1);
        Check(!clear.emergency && clear.reason == CLEAR && clear.speed_limit > 0.48,
              "single urgent ghost disappears without emergency latch and speed recovers gradually");
        PerceptionSafety skipped;
        skipped.Observe(Frame(100, {near}), 100);
        skipped.Evaluate(Path(), Ego(), 3, 100);
        skipped.Observe(Frame(100.1, {near}), 100.1);
        skipped.Observe(Frame(100.2, {near}), 100.2);
        Check(!skipped.Evaluate(Path(), Ego(), 3, 100.2).emergency,
              "unevaluated intermediate observations cannot supply emergency risk votes");
        skipped.Observe(Frame(100.3, {near}), 100.3);
        Check(!skipped.Evaluate(Path(), Ego(), 3, 100.3).emergency,
              "skipped frame breaks the previous run of risk evidence");
        skipped.Observe(Frame(100.4, {near}), 100.4);
        Check(skipped.Evaluate(Path(), Ego(), 3, 100.4).emergency, "three evaluated consecutive risk frames confirm");
        for(double spacing : {0.1, 0.3, 1.0}) {
            PerceptionSafety safety;
            std::vector<PATH_POINT_S> path;
            for(int i = 0; i < 100; ++i) path.emplace_back(100 + i * spacing, 100);
            for(int tick = 0; tick < 3; ++tick) {
                const double now = 100 + tick * 0.1;
                safety.Observe(Frame(now, {near}), now);
                auto result = safety.Evaluate(path, Ego(), 3, now);
                Check(result.emergency == (tick == 2), "next-segment physical contact confirms independently of path spacing");
                Check(std::abs(result.speed_limit - (tick == 2 ? 0 : 0.48)) < 1e-9,
                      "next-segment contact receives the same pending speed and confirmed stop");
                for(int repeat = 0; repeat < 5; ++repeat) {
                    Check(!safety.Observe(Frame(now, {near}), now), "duplicate urgent observation rejected");
                    result = safety.Evaluate(path, Ego(), 3, now + repeat * 0.001);
                    Check(result.emergency == (tick == 2), "repeated planning cannot manufacture emergency votes");
                }
            }
        }
        PerceptionSafety established;
        Confirm(established, Object(7, 107, 100));
        established.Observe(Frame(100.3, {near}), 100.3);
        Check(!established.Evaluate(Path(), Ego(), 3, 100.3).emergency,
              "previously confirmed track still needs three urgent frames");
        established.Observe(Frame(100.4, {Object(7, 107, 100)}), 100.4);
        established.Evaluate(Path(), Ego(), 3, 100.4);
        for(int tick = 0; tick < 2; ++tick) {
            const double now = 100.5 + tick * 0.1;
            established.Observe(Frame(now, {near}), now);
            Check(!established.Evaluate(Path(), Ego(), 3, now).emergency, "nonurgent observation breaks consecutive risk");
        }
        established.Observe(Frame(100.7, {}), 100.7);
        Check(!established.Evaluate(Path(), Ego(), 3, 100.7).emergency, "coasting cannot supply the third emergency vote");
        established.Observe(Frame(100.8, {near}), 100.8);
        Check(!established.Evaluate(Path(), Ego(), 3, 100.8).emergency, "explicit miss breaks risk evidence for established ID");

        PerceptionSafety gap;
        for(double now : {100.0, 100.1, 100.5, 100.6}) {
            gap.Observe(Frame(now, {near}), now);
            Check(!gap.Evaluate(Path(), Ego(), 3, now).emergency, "observation gap resets urgent confirmation");
        }
        gap.Observe(Frame(100.7, {near}), 100.7);
        Check(gap.Evaluate(Path(), Ego(), 3, 100.7).emergency, "three timely frames after gap do confirm");

        PerceptionSafety different;
        for(int tick = 0; tick < 8; ++tick) {
            const double now = 100 + tick * 0.1;
            auto object = Object(7 + tick % 2, 102.8, 100);
            object.type = 1;
            different.Observe(Frame(now, {object}), now);
            Check(!different.Evaluate(Path(), Ego(), 3, now).emergency, "different IDs never pool emergency votes");
        }
        for(double target : {0.0, 0.2, 3.0}) {
            PerceptionSafety lower;
            lower.Observe(Frame(100, {near}), 100);
            auto result = lower.Evaluate(Path(), Ego(), target, 100);
            Check(!result.emergency && std::abs(result.speed_limit - std::min(target, 0.48)) < 1e-9,
                  "pending rule preserves other lower and zero target speeds");
        }
        PerceptionSafety low;
        auto low_object = near;
        low_object.confidence = 0.1;
        for(int tick = 0; tick < 3; ++tick) {
            const double now = 100 + tick * 0.1;
            low.Observe(Frame(now, {low_object}), now);
            Check(low.Evaluate(Path(), Ego(), 3, now).emergency == (tick == 2),
                  "emergency confirmation is three real risk frames even at low score");
        }
    }
    void TestFreshnessAndValidation() {
        PerceptionSafety safety;
        auto object = Object(32767, 110, 100, 1, 1, 90, 1, 0);
        Confirm(safety, object);
        safety.Observe(Frame(100.3, {}), 100.3);
        Check(safety.mTracks[32767].active, "confirmed target can coast through short occlusion");
        safety.Evaluate(Path(), Ego(), 3, 100.4);
        Check(safety.mTracks[32767].box.x == 110 && safety.mTracks[32767].last_seen == 100.2,
              "prediction does not mutate measurement or refresh age");
        auto bad = object;
        bad.polygons[0].x = std::numeric_limits<double>::quiet_NaN();
        Check(!safety.Observe(Frame(100.4, {Object(7, 108, 100), bad}), 100.4), "invalid frame rejected atomically");
        Check(!safety.mTracks[7].active && safety.mFrameTime == 100.3, "no half-frame mutation");
        Check(!safety.Observe(Frame(100.4, {object, object}), 100.4), "duplicate IDs rejected");
        Check(!safety.Observe(Frame(100.2, {}), 100.4), "out-of-order empty cannot erase obstacles");
        Check(!safety.Observe(Frame(200, {}), 100.4), "future stamp rejected");
        auto result = safety.Evaluate(Path(), Ego(), 3, 101.0);
        Check(result.emergency && result.reason == STALE_INPUT, "new stream loss stops after activation");
        Check(safety.Observe(Frame(50, {}), 50), "clock rollback allows a new epoch");
        Check(safety.mActiveIds.empty(), "old epoch tracks cleared");

        PerceptionSafety shapes;
        auto thin = Object(1, 105, 100, 1, 0.02);
        Check(shapes.Observe(Frame(100, {thin}), 100), "legacy-ignored thin box is a valid empty observation");
        Check(shapes.mActiveIds.empty(), "same degenerate-size policy");
        auto invalid = Object(1, 105, 100);
        invalid.polygons.pop_back();
        Check(!shapes.Observe(Frame(100.1, {invalid}), 100.1), "missing physical geometry cannot use motion heading");
    }
    void TestWidthsAndSpeedLimits() {
        PerceptionSafety safety;
        CONFIG_S config;
        config.empty_width = 1.5;  // 仅为测试几何；生产配置保留原保护宽度。
        Check(safety.Configure(config), "valid footprint configuration");
        Confirm(safety, Object(7, 106, 98.2));
        auto ego = Ego();
        Check(safety.Evaluate(Path(), ego, 3, 100.2).reason == CLEAR, "empty footprint clears side obstacle");
        ego.loaded = true;
        Check(safety.Evaluate(Path(), ego, 3, 100.2).reason != CLEAR, "loaded footprint includes pallet width");
        config.empty_width = -1;
        Check(!safety.Configure(config), "invalid size rejected instead of silently shrinking footprint");

        PerceptionSafety slow;
        double previous = 2.0, acceleration = 0;
        for(int tick = 0; tick < 15; ++tick) {
            const double now = 100 + tick * 0.1;
            slow.Observe(Frame(now, {Object(7, 111, 100)}), now);
            const auto result = slow.Evaluate(Path(), Ego(previous), 3, now);
            const double next_acc = (result.speed_limit - previous) / 0.1;
            if(next_acc < -0.80001 || std::abs(next_acc - acceleration) > 0.08001) {
                std::cerr << "jerk tick=" << tick << " previous=" << previous << " next=" << result.speed_limit
                          << " old_acc=" << acceleration << " new_acc=" << next_acc << '\n';
            }
            Check(!result.emergency && result.speed_limit <= previous + 1e-9, "ordinary stop decelerates without safety");
            Check(next_acc >= -0.80001 && std::abs(next_acc - acceleration) <= 0.08001,
                  "deceleration and jerk bounded in gradual slowdown");
            previous = result.speed_limit;
            acceleration = next_acc;
        }
        const auto stopped = slow.Evaluate(Path(), Ego(previous), 0, 101.41);
        Check(stopped.speed_limit == 0, "other business zero-speed command always wins");
        Check(slow.WorkingBytes() < 8 * 1024 * 1024, "bounded cache below 8 MiB");
    }

    void TestEarlyStaticBraking() {
        const CONFIG_S config;
        for(double speed : {1.0, 2.0, 15.0 / 3.6}) {
            // 场景距车头分别为 6.5、13.5、33 m；均在旧普通减速距离之外。
            const double gap = speed == 1 ? 6.5 : (speed == 2 ? 13.5 : 33.0);
            for(int type = 0; type <= 4; ++type) {
                for(double velocity : {0.0, 0.001, 0.5}) {
                    PerceptionSafety early, baseline;
                    CONFIG_S without_extra = config;
                    without_extra.static_obstacle_extra_time = 0;
                    Check(baseline.Configure(without_extra), "zero extra time disables only static anticipation");
                    auto object = Object(7, 100 + config.empty_front + gap + 0.25, 100, 0.5, 0.5, 90, velocity);
                    object.type = type;
                    Confirm(early, object);
                    Confirm(baseline, object);
                    const auto result = early.Evaluate(Path(), Ego(speed), speed, 100.2);
                    const auto original = baseline.Evaluate(Path(), Ego(speed), speed, 100.2);
                    Check(!result.emergency && !original.emergency, "distant obstacle stays in ordinary slowdown");
                    if(type == 0 && velocity == 0) {
                        Check(result.speed_limit < speed && original.speed_limit == speed,
                              "stationary own-lane obstacle slows earlier at low and road speed");
                        Check(early.Lookahead(speed) > baseline.Lookahead(speed) && early.Lookahead(speed) > gap,
                              "lookahead covers additional static braking reserve");
                    } else {
                        Check(result.speed_limit == original.speed_limit && result.reason == original.reason &&
                                  early.Lookahead(speed) == baseline.Lookahead(speed),
                              "extra time leaves adjacent, excluded and moving targets unchanged");
                    }
                }
            }
        }
        for(double gap : {2.8, 3.3}) {
            PerceptionSafety early, previous;
            CONFIG_S old = config;
            old.reaction_time = 0.4;
            old.static_obstacle_extra_time = 0;
            old.static_stop_min_distance = 0;
            old.front_no_confirmation_distance = 0;
            Check(previous.Configure(old), "previous emergency calibration is a valid comparison");
            const auto object = Object(7, 100 + config.empty_front + gap + 0.25, 100, 0.5, 0.5);
            for(int tick = 0; tick < 3; ++tick) {
                const double now = 100 + tick * 0.1;
                early.Observe(Frame(now, {object}), now);
                previous.Observe(Frame(now, {object}), now);
                const auto result = early.Evaluate(Path(), Ego(1), 1, now);
                const auto original = previous.Evaluate(Path(), Ego(1), 1, now);
                Check(result.emergency == (gap == 2.8) && !original.emergency,
                      "own-lane front obstacle inside three metres stops without confirmation delay");
                if(gap == 2.8) {
                    Check(result.speed_limit == 0, "near own-lane front obstacle publishes zero speed immediately");
                }
            }
            RESULT_S cleared;
            for(int tick = 0; tick < 15; ++tick) {
                const double now = 100.3 + tick * 0.1;
                early.Observe(Frame(now, {}), now);
                cleared = early.Evaluate(Path(), Ego(0), 1, now);
            }
            Check(!cleared.emergency && cleared.speed_limit > 0, "clear input releases early stop with existing hysteresis");
        }
        PerceptionSafety transition;
        auto object = Object(7, 100 + config.empty_front + 6.5 + 0.25, 100, 0.5, 0.5);
        Confirm(transition, object);
        auto result = transition.Evaluate(Path(), Ego(1), 1, 100.2);
        Check(!result.emergency && result.speed_limit < 1, "static transition starts with ordinary slowdown");
        const double extended = transition.Lookahead(1);
        object.type = 1;
        transition.Observe(Frame(100.3, {object}), 100.3);
        Check(transition.Lookahead(1) < extended, "same ID leaving own lane updates the cached classification");
        object.type = 0;
        object.vy = 0.5;
        transition.Observe(Frame(100.4, {object}), 100.4);
        Check(transition.Lookahead(1) < extended, "same ID starting to move drops extra static lookahead");
        object.vy = 0;
        transition.Observe(Frame(100.5, {object}), 100.5);
        Check(transition.Lookahead(1) == extended, "same ID stopping in own lane restores anticipation");
        Check(transition.Evaluate(Path(), Ego(1), 0, 100.5).speed_limit == 0,
              "early slowdown never overrides another business zero speed");

        PerceptionSafety side;
        object = Object(7, 100 + config.empty_front + 6.5 + 0.25, 104, 0.5, 0.5);
        Confirm(side, object);
        result = side.Evaluate(Path(), Ego(1), 1, 100.2);
        Check(result.reason == CLEAR && result.speed_limit == 1,
              "own-lane label alone cannot slow an object outside the swept footprint");
        for(double invalid : {-0.1, 5.1, std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::infinity()}) {
            CONFIG_S bad = config;
            bad.static_obstacle_extra_time = invalid;
            Check(!side.Configure(bad) && side.GetConfig().static_obstacle_extra_time == 2,
                  "invalid anticipation is rejected without changing active configuration");
        }
    }

    void TestNearForwardStop() {
        const CONFIG_S config;
        // 同一静态目标，车速不断下降：旧动态距离会跌破剩余距离并清零急停票数。
        for(int mode = 0; mode < 3; ++mode) {
            PerceptionSafety safety;
            CONFIG_S settings = config;
            if(mode > 0) settings.front_no_confirmation_distance = 0;
            if(mode == 2) settings.static_stop_min_distance = 0;
            Check(safety.Configure(settings), "near-stop options configure independently");
            const auto object = Object(7, 100 + config.empty_front + 3 + 0.25, 100, 0.5, 0.5);
            double x = 100;
            for(int tick = 0; tick < 75; ++tick) {
                const double now = 100 + tick * 0.1;
                const double speed = tick >= 72 ? 0 : std::max(0.2, 1 - tick * 0.06);
                auto ego = Ego(speed);
                ego.x = x;
                auto path = Path();
                for(auto& point : path) point.x += x - 100;
                safety.Observe(Frame(now, {object}), now);
                const auto result = safety.Evaluate(path, ego, 1, now);
                Check(result.emergency == (mode == 0 || (mode == 1 && tick >= 2)),
                      "stop floor prevents disappearing emergency evidence during deceleration");
                if(mode < 2) {
                    Check(result.emergency_distance >= 3, "static own-lane stopping threshold never falls below three metres");
                    if(result.emergency) Check(result.speed_limit == 0, "near obstacle holds zero speed after stopping");
                }
                x += speed * 0.1;
            }
        }
        for(double gap : {1.0, 3.0, 3.01, 3.06, 6.0, 6.01}) {
            for(double speed : {0.0, 0.2, 1.0, 2.0}) {
                for(int type = 0; type <= 2; ++type) {
                    PerceptionSafety safety;
                    auto object = Object(7, 100 + config.empty_front + gap + 0.25, 100, 0.5, 0.5);
                    object.type = type;
                    const double dynamic_distance = speed * 2.1 + speed * speed / 1.6 + 0.3;
                    const double threshold = type == 0 ? std::max(3.0, dynamic_distance) : dynamic_distance;
                    for(int tick = 0; tick < 3; ++tick) {
                        const double now = 100 + tick * 0.1;
                        safety.Observe(Frame(now, {object}), now);
                        const auto result = safety.Evaluate(Path(), Ego(speed), 2, now);
                        // 原紧急几何保留 5 cm 包络；6 m 免确认范围只看当前物理框。
                        const bool imminent = type != 2 && gap <= threshold + 0.05;
                        const bool immediate = type == 0 && gap <= 6 && imminent;
                        if(result.emergency != (imminent && (immediate || tick == 2))) {
                            std::cerr << "near stop: gap=" << gap << " speed=" << speed << " type=" << type
                                      << " tick=" << tick << " safety=" << result.emergency
                                      << " threshold=" << result.emergency_distance << '\n';
                        }
                        Check(result.emergency == (imminent && (immediate || tick == 2)),
                              "six-metre bypass and three-metre floor preserve lane and speed boundaries");
                        Check(result.immediate_confirmation == immediate,
                              "diagnostic distinguishes immediate confirmation from normal three-frame evidence");
                    }
                }
            }
        }
        for(double gap : {5.9, 8.0}) {
            PerceptionSafety safety;
            const auto object = Object(7, 100 + config.empty_front + gap + 0.25, 100, 0.5, 0.5, 90, -3);
            safety.Observe(Frame(100, {object}), 100);
            const auto result = safety.Evaluate(Path(), Ego(2), 2, 100);
            Check(result.distance < 6 && result.emergency == (gap < 6),
                  "moving collision prediction cannot turn a currently distant target into a near target");
        }
        for(double heading : {0.0, 45.0, 90.0, 180.0, 270.0, 360.0}) {
            PerceptionSafety safety;
            const double angle = (90 - heading) * 3.14159265358979323846 / 180;
            auto ego = Ego(2);
            ego.heading = heading;
            auto path = Path();
            for(auto& point : path) {
                const double along = point.x - 100;
                point.x = 100 + along * std::cos(angle);
                point.y = 100 + along * std::sin(angle);
            }
            Check(safety.BuildSegments(path, ego), "oriented near-stop path builds");
            const double along[] = {8.55, 8.56, -2, 2};
            const double across[] = {0, 0, 0, 3};
            for(int i = 0; i < 4; ++i) {
                const auto object = Object(7, 100 + along[i] * std::cos(angle) - across[i] * std::sin(angle),
                                           100 + along[i] * std::sin(angle) + across[i] * std::cos(angle),
                                           0.5, 0.5, heading);
                PerceptionSafety::BOX_S box;
                Check(safety.ReadBox(object, box), "oriented obstacle footprint accepted");
                std::size_t axes = 0;
                Check(safety.IsNearForwardObstacle(box, axes) == (i == 0),
                      "near range measures physical front gap and rejects side/rear at any heading");
            }
        }
        PerceptionSafety coast;
        Confirm(coast, Object(7, 110, 100, 0.5, 0.5));
        coast.Evaluate(Path(), Ego(2), 2, 100.2);
        coast.Observe(Frame(100.3, {}), 100.3);
        auto ego = Ego(2);
        ego.x += 4;
        auto path = Path();
        for(auto& point : path) point.x += 4;
        const auto missed = coast.Evaluate(path, ego, 2, 100.3);
        Check(!missed.emergency && !missed.immediate_confirmation,
              "extrapolated missed target cannot initiate immediate emergency confirmation");
        PerceptionSafety pending;
        pending.Observe(Frame(100, {Object(7, 104, 100)}), 100);
        Check(pending.Evaluate(Path(), Ego(2), 2, 100).emergency, "initial near obstacle latches stop");
        for(int tick = 1; tick < 15; ++tick) {
            const double now = 100 + tick * 0.1;
            pending.Observe(Frame(now, {Object(10 + tick, 109, 100, 0.5, 0.5)}), now);
            Check(pending.Evaluate(Path(), Ego(2), 2, now).emergency,
                  "new pending imminent risks cannot release an already latched stop");
        }
        for(int parameter = 0; parameter < 2; ++parameter) {
            for(double value : {-0.1, 21.0, std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity()}) {
                PerceptionSafety safety;
                CONFIG_S bad = config;
                (parameter == 0 ? bad.static_stop_min_distance : bad.front_no_confirmation_distance) = value;
                Check(!safety.Configure(bad), "invalid near-stop parameters rejected");
            }
        }
    }

    // 独立静态 oracle：顶点在凸多边形内 + 边相交，不复用生产分离轴公式。
    struct XY {
        double x, y;
    };
    double Cross(XY a, XY b, XY c) {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }
    std::vector<XY> Corners(const PerceptionSafety::BOX_S& box, double x, double y) {
        std::vector<XY> p;
        const double sx[] = {-1, 1, 1, -1}, sy[] = {-1, -1, 1, 1};
        for(int i = 0; i < 4; ++i) p.push_back({x + sx[i] * box.hx * box.ux - sy[i] * box.hy * box.uy,
                                                y + sx[i] * box.hx * box.uy + sy[i] * box.hy * box.ux});
        return p;
    }
    bool Inside(const std::vector<XY>& p, XY q) {
        for(int i = 0; i < 4; ++i)
            if(Cross(p[i], p[(i + 1) % 4], q) < -1e-8) return false;
        return true;
    }
    bool Intersects(const std::vector<XY>& a, const std::vector<XY>& b) {
        for(int i = 0; i < 4; ++i)
            if(Inside(a, b[i]) || Inside(b, a[i])) return true;
        for(int i = 0; i < 4; ++i) {
            for(int j = 0; j < 4; ++j) {
                if(Cross(a[i], a[(i + 1) % 4], b[j]) * Cross(a[i], a[(i + 1) % 4], b[(j + 1) % 4]) < 0 &&
                   Cross(b[j], b[(j + 1) % 4], a[i]) * Cross(b[j], b[(j + 1) % 4], a[(i + 1) % 4]) < 0) return true;
            }
        }
        return false;
    }
    void TestForwardSpeedProfile() {
        for(bool loaded : {false, true}) {
            for(double heading : {0.0, 37.0, 90.0, 180.0, 270.0}) {
                for(double gap : {2.9, 3.0, 3.01, 4.5, 5.99, 6.0, 6.01, 9.0}) {
                    PerceptionSafety safety;
                    CONFIG_S config;
                    config.loaded_front = 5;
                    config.loaded_length = 8;
                    config.loaded_width = 4;
                    Check(safety.Configure(config), "forward profile uses calibrated loaded footprint");
                    auto ego = Ego(1);
                    ego.heading = heading;
                    ego.loaded = loaded;
                    const double yaw = (90 - heading) * 3.14159265358979323846 / 180;
                    const double center = (loaded ? 5 : 2.3) + gap + 0.6;
                    auto object = Object(7, ego.x + center * std::cos(yaw), ego.y + center * std::sin(yaw), 1.2, 1, heading);
                    safety.Observe(Frame(100, {object}), 100);
                    const auto cap = safety.ForwardObstacleSpeedLimit(ego, 1, 100);
                    Check(cap.approach_speed <= cap.hard_speed && cap.hard_speed <= 1,
                          "approach target cannot exceed physical envelope or existing target");
                    if(gap <= 3) {
                        Check(cap.hard_speed <= 1e-5, "three-metre front-body clearance requires zero target");
                    } else if(gap <= 6) {
                        Check(std::abs(cap.hard_speed - 0.45 * std::sqrt((gap - 3) / 3)) < 1e-5,
                              "six-to-three metres uses finite-time squared-speed stopping profile");
                        Check(cap.hard_speed < 0.5, "six metres and closer remain strictly below 0.5m/s");
                    } else {
                        Check(cap.approach_speed < 1, "approach already lowers target beyond six metres");
                    }
                }
            }
        }
        struct SPEED_CASE_S { double ego, vx, vy; bool slow; };
        const SPEED_CASE_S cases[] = {{1, 0, 0, true}, {0, 0.4, 0, true}, {1, 0.499, 0, true},
                                      {1, 0.5, 0, false}, {1.5, 0.5, 0, false}, {1.501, 0.5, 0, true},
                                      {2, 1, 0, false}, {2, 0.999, 0, true}, {1, -0.8, 0, true},
                                      {1, 3, 0, false}, {0.8, 0, 0.6, false}};
        for(const auto& item : cases) {
            for(int lane = 0; lane < 5; ++lane) {
                PerceptionSafety safety;
                auto object = Object(7, 108.9, 100, 1.2, 1, 90, item.vx, item.vy);
                object.type = lane;
                safety.Observe(Frame(100, {object}), 100);
                const auto cap = safety.ForwardObstacleSpeedLimit(Ego(item.ego), 3, 100);
                Check((cap.hard_speed < 0.5) == (lane == 0 && item.slow),
                      "only own lane: closing speed strictly above 1 OR object speed strictly below 0.5");
            }
        }
        for(double speed : {0.8, 1.0, 2.0, 3.0, 4.167}) {
            const double start = 6 + speed * 3 + (speed * speed - 0.4 * 0.4) / (2 * 0.8);
            for(double offset : {-0.01, 0.01}) {
                PerceptionSafety safety;
                safety.Observe(Frame(100, {Object(7, 102.9 + start + offset, 100)}), 100);
                const auto cap = safety.ForwardObstacleSpeedLimit(Ego(speed), speed, 100);
                Check((cap.approach_speed < speed) == (offset < 0),
                      "deceleration begins three seconds of travel before braking distance to six metres");
            }
        }
        for(const auto& position : std::vector<PATH_POINT_S>{{95, 100}, {108.9, 103}, {100, 100}}) {
            PerceptionSafety safety;
            safety.Observe(Frame(100, {Object(7, position.x, position.y)}), 100);
            Check(safety.ForwardObstacleSpeedLimit(Ego(1), 1, 100).hard_speed == 1,
                  "side and rear boxes are not frontal speed constraints merely because type is zero");
        }
        PerceptionSafety coasting;
        Confirm(coasting, Object(7, 108.9, 100));
        coasting.Observe(Frame(100.3, {}), 100.3);
        Check(coasting.ForwardObstacleSpeedLimit(Ego(1), 1, 100.3).hard_speed < 0.5,
              "confirmed missed obstacle retains existing short coast protection");
        coasting.Observe(Frame(100.8, {}), 100.8);
        Check(coasting.ForwardObstacleSpeedLimit(Ego(1), 1, 100.8).hard_speed == 1,
              "expired track cannot retain the new speed envelope");
    }

    void TestForwardSafetyUnchanged() {
        for(int lane = 0; lane < 5; ++lane) {
            for(double vx : {0.0, 0.4, 0.5, 1.0, -1.0}) {
                for(double speed : {0.4, 1.0, 2.0}) {
                    PerceptionSafety baseline, forward;
                    for(int tick = 0; tick < 36; ++tick) {
                        const double now = 100 + tick * 0.1;
                        const double gap = tick < 10 ? 12 - tick : 2.8;
                        auto object = Object(tick < 12 ? 7 : 8, 102.9 + gap, 100, 1.2, 1, 90, vx);
                        object.type = lane;
                        const auto input = Frame(now, tick < 16 || tick >= 30 ? std::vector<robot::object>{object} : std::vector<robot::object>{});
                        baseline.Observe(input, now);
                        forward.Observe(input, now);
                        const auto old = baseline.Evaluate(Path(), Ego(speed), 3, now);
                        const auto current = forward.Evaluate(Path(), Ego(speed), 3, now, true);
                        Check(old.emergency == current.emergency && old.emergency_distance == current.emergency_distance &&
                              old.emergency_hits == current.emergency_hits && old.immediate_confirmation == current.immediate_confirmation &&
                              old.distance == current.distance && old.ttc == current.ttc &&
                              baseline.mEmergencyLatched == forward.mEmergencyLatched && baseline.mClearSince == forward.mClearSince &&
                              baseline.mClearFrame == forward.mClearFrame,
                              "same perception and CAN preserve emergency decisions, confirmation and release timing");
                    }
                }
            }
        }
    }

    void TestForwardTimeTrajectory() {
        for(double cruise : {1.0, 2.0, 4.167}) {
            for(bool irregular : {false, true}) {
                PerceptionSafety safety;
                auto ego = Ego(cruise);
                double gap = 45, time = 100, previous = cruise, previous_acceleration = 0;
                bool stopped = false, seen_six = false;
                const double periods[] = {0.05, 0.1, 0.15};
                for(int tick = 0; tick < 1500; ++tick) {
                    const double dt = irregular ? periods[tick % 3] : 0.1;
                    time += dt;
                    safety.Observe(Frame(time, {Object(7, 102.9 + gap, 100)}), time);
                    const auto current = safety.Evaluate(Path(), ego, cruise, time, true);
                    const double acceleration = (current.speed_limit - previous) / dt;
                    if(!current.emergency) {
                        if(acceleration < -0.80001 || std::abs(acceleration - previous_acceleration) > 0.8 * dt + 1e-5) {
                            std::cerr << "forward time: cruise=" << cruise << " gap=" << gap << " dt=" << dt
                                      << " previous=" << previous << " speed=" << current.speed_limit << " a=" << acceleration
                                      << " previous_a=" << previous_acceleration << '\n';
                        }
                        Check(acceleration >= -0.80001 && acceleration <= 0.30001 &&
                              std::abs(acceleration - previous_acceleration) <= 0.8 * dt + 1e-5,
                              "early deceleration respects elapsed-time acceleration and jerk bounds");
                    }
                    const auto duplicate = safety.Evaluate(Path(), ego, cruise, time, true);
                    Check(duplicate.speed_limit == current.speed_limit && duplicate.emergency == current.emergency,
                          "same planning timestamp cannot advance the temporal speed trajectory");
                    if(gap <= 6) {
                        seen_six = true;
                        Check(current.speed_limit <= 0.450001 && ego.speed < 0.5,
                              "with ideal target following the vehicle is below 0.5 before reaching six metres");
                    }
                    if(current.emergency) {
                        Check(current.speed_limit == 0 && gap >= 3 - 1e-5, "existing safety can stop before the three-metre boundary");
                        stopped = true;
                        break;
                    }
                    gap -= current.speed_limit * dt;
                    ego.speed = current.speed_limit;
                    previous = current.speed_limit;
                    previous_acceleration = acceleration;
                }
                Check(seen_six && stopped, "time trajectory covers approach, low speed and stop without stalling above six metres");
            }
        }
        PerceptionSafety late;
        late.Observe(Frame(100, {Object(7, 108.9, 100, 1.2, 1, 90, 0.4)}), 100);
        auto current = late.Evaluate(Path(), Ego(1), 1, 100, true);
        Check(current.speed_limit < 0.5, "late first detection cannot use smoothing to exceed six-metre cap");
        late.Observe(Frame(100.1, {Object(7, 105.9, 100, 1.2, 1, 90, 0.4)}), 100.1);
        current = late.Evaluate(Path(), Ego(0.3), 1, 100.1, true);
        Check(current.speed_limit == 0 && !current.emergency,
              "three-metre ordinary zero target does not invent safety for a slow moving obstacle");
    }

    void TestContinuousGeometryOracle() {
        std::mt19937 random(92751);
        std::uniform_real_distribution<double> position(-8, 8), angle(-3.14, 3.14), size(0.1, 3);
        int collisions = 0;
        for(int sample = 0; sample < 2000; ++sample) {
            PerceptionSafety::SEGMENT_S segment;
            PerceptionSafety::BOX_S obstacle;
            const double a = angle(random), b = angle(random);
            segment.box.ux = std::cos(a);
            segment.box.uy = std::sin(a);
            segment.box.hx = size(random);
            segment.box.hy = size(random);
            segment.dx = position(random);
            segment.dy = position(random);
            segment.duration = 0.5;
            obstacle.x = position(random);
            obstacle.y = position(random);
            obstacle.ux = std::cos(b);
            obstacle.uy = std::sin(b);
            obstacle.hx = size(random);
            obstacle.hy = size(random);
            const double vx = position(random) * 2, vy = position(random) * 2;
            double enter = 0;
            std::size_t axes = 0;
            const bool hit = PerceptionSafety::Sweep(segment, obstacle, vx, vy, 0, 1, enter, axes);
            Check(axes <= 4, "four cached axes bound each continuous check");
            bool sampled = false;
            for(int i = 0; i <= 200; ++i) {
                const double t = i / 200.0;
                sampled = sampled || Intersects(Corners(segment.box, segment.dx * t, segment.dy * t),
                                                Corners(obstacle, obstacle.x + vx * segment.duration * t, obstacle.y + vy * segment.duration * t));
            }
            Check(!sampled || hit, "continuous sweep never misses independent sampled collision");
            if(hit) {
                ++collisions;
                Check(enter >= 0 && enter <= 1 && Intersects(Corners(segment.box, segment.dx * enter, segment.dy * enter), Corners(obstacle, obstacle.x + vx * segment.duration * enter, obstacle.y + vy * segment.duration * enter)),
                      "reported continuous contact independently intersects");
            }
        }
        Check(collisions > 100, "oracle exercises nontrivial collisions");
    }
}

int main() {
    TestObservationAspectFilter();
    TestObservationConfirmation();
    TestGeometryAndMotion();
    TestLanePolicy();
    TestLaneMotionDirection();
    TestLeftLaneObservationPolicy();
    TestLeftSecondLaneHistory();
    TestEmergencyEvidence();
    TestFreshnessAndValidation();
    TestWidthsAndSpeedLimits();
    TestEarlyStaticBraking();
    TestNearForwardStop();
    TestForwardSpeedProfile();
    TestForwardSafetyUnchanged();
    TestForwardTimeTrajectory();
    TestContinuousGeometryOracle();
    std::cout << "PASS perception safety: " << checks << " checks\n";
}
