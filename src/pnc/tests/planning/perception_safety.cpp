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
            Check(!result.emergency && std::abs(result.speed_limit - (item.keep ? 0.48 : 3.0)) < 1e-9,
                  "retained near obstacles get a 60 percent limit before emergency confirmation");
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
        auto frame = Frame(100, {Object(7, 105, 100)});
        Check(safety.Observe(frame, 100), "new frame");
        for(int tick = 0; tick < 4; ++tick) {
            const auto result = safety.Evaluate(Path(), Ego(), 3, 100 + tick * 0.1);
            Check(!result.emergency && result.reason == CONFIRMING, "old sample cannot become confirmed or emergency");
            Check(!safety.Observe(frame, 100 + tick * 0.1), "duplicate does not refresh frame");
        }
        Check(safety.Observe(Frame(100.4, {}), 100.4), "empty raw frame");
        Check(safety.mActiveIds.empty(), "one-frame ghost removed on explicit miss");
        Check(!safety.Evaluate(Path(), Ego(), 3, 100.4).emergency, "ghost never latched emergency");
        Check(safety.Observe(Frame(100.5, {Object(7, 105, 100)}), 100.5), "reappearance");
        Check(!safety.mTracks[7].confirmed && safety.mTracks[7].hits == 1, "miss breaks consecutive observations");
        Check(safety.Observe(Frame(100.6, {Object(8, 105, 100)}), 100.6), "different ID");
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
        object.type = 2;
        Confirm(passing, object);
        result = passing.Evaluate(Path(), Ego(), 3, 100.2);
        Check(result.reason == CLEAR && !result.emergency, "side overtaker moving away has no time-space collision");

        PerceptionSafety cut_in;
        object = Object(7, 108, 97, 1.2, 1, 90, 0, 0.8);
        object.type = 1;
        Confirm(cut_in, object);
        result = cut_in.Evaluate(Path(), Ego(2), 3, 100.2);
        Check(result.object_id == 7 && result.reason == SLOWING && !result.emergency,
              "left-lane cut-in predicts collision and slows in advance");

        PerceptionSafety static_block;
        object = Object(7, 108, 100);
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
            Check(result.emergency == (tick == 2), "imminent collision requires three real risk frames");
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
                if(type >= 3) {
                    Check(result.reason == CLEAR && !result.emergency && result.speed_limit == 3 && result.candidates == 0,
                          "outside-lane physical overlap is ignored without geometry work");
                } else {
                    Check(result.emergency == (tick == 2), "current and both left lanes keep emergency eligibility");
                    Check(std::abs(result.speed_limit - (tick == 2 ? 0 : 0.48)) < 1e-9,
                          "first two risk frames cap at measured speed times 0.6");
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
            Check(!safety.Evaluate(Path(), Ego(), 3, now).emergency, "lane reentry starts emergency confirmation afresh");
        }
    }

    void TestEmergencyEvidence() {
        const auto near = Object(7, 103.29, 100);
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
        Confirm(established, Object(7, 105, 100));
        established.Observe(Frame(100.3, {near}), 100.3);
        Check(!established.Evaluate(Path(), Ego(), 3, 100.3).emergency,
              "previously confirmed track still needs three urgent frames");
        established.Observe(Frame(100.4, {Object(7, 105, 100)}), 100.4);
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
            different.Observe(Frame(now, {Object(7 + tick % 2, 102.8, 100)}), now);
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
            slow.Observe(Frame(now, {Object(7, 108, 100)}), now);
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
    TestEmergencyEvidence();
    TestFreshnessAndValidation();
    TestWidthsAndSpeedLimits();
    TestContinuousGeometryOracle();
    std::cout << "PASS perception safety: " << checks << " checks\n";
}
