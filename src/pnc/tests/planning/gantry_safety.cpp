// 验证实际发布消息，只允许闸机条件改变 safety；原有状态、速度、声光及参数事件均保持。
#include "test_access.h"
#include "navigation_feedback.h"
#include "test_trace.h"
#include "gantry_detect/gantry_state.h"
#include <cstdlib>
#include <limits>
#include <new>

extern PathPlanComply pathPlanComply;
void GantryStateCallBack(const gantry_detect::gantry_state::ConstPtr &msg);

int checks = 0;

void Check(bool passed, const std::string &name) {
    if(!passed) {
        std::cerr << "FAIL: " << name << '\n';
        std::exit(1);
    }
    checks++;
}

template<class T>
std::string Trace(const T &value) {
    std::ostringstream stream;
    stream.precision(17);
    TraceValue(stream, value);
    return stream.str();
}

struct Publication {
    robot::path_plan_msg path;
    std::string sound;
    std::string state;
    std::vector<std::string> events;
    int path_count = 0;
    int sound_count = 0;
};

struct Fixture {
    PathPlanComply *plan;

    Fixture(const std::string &pnc, const std::string &scenario) {
        // 与生产全局实例保持相同的零初始化，避免旧的未初始化成员干扰测试。
        void *storage = ::operator new(sizeof(PathPlanComply));
        std::memset(storage, 0, sizeof(PathPlanComply));
        plan = new(storage) PathPlanComply;
        ros::param::values().clear();
        ros::param::set("task_file", pnc + "/param/");
        ros::param::set("path_dir", pnc + "/path/");
        ros::testTime() = 100.0;
        plan->InitParameter();
        plan->sysTime.now = 100.0;
        plan->sysTime.taskStart = 90.0;
        plan->sysTime.vehicleStop = 90.0;
        plan->sysTime.vehicleRun = 90.0;
        plan->mNavData.xAxis = 100.0;
        plan->mNavData.yAxis = 100.0;
        plan->mNavData.heading = 90.0;
        robot::can_msg can;
        can.controlPanelState = scenario.find("manual") == 0 ? 0 : 1;
        can.curGear = scenario == "reverse" ? GEAR_R : GEAR_D;
        can.vehicleSpeed = 0.8;
        // 本夹具检查常规行驶阶段；真实导航连续两帧确认已经起步。
        ros::testTime() = 99.9;
        SetVehicleFeedback(*plan, can);
        ros::testTime() = 100.0;
        SetVehicleFeedback(*plan, can);
        plan->mReferPath.x = {100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111};
        plan->mReferPath.y.assign(plan->mReferPath.x.size(), 100.0);
        plan->mReferPath.desireSpeed = 3.0;
        plan->mReferPath.planspeed = 2.7;
        plan->mReferPath.Path_Id = scenario == "reverse" ? 5 : 0;
        plan->mReferPath.safety = scenario == "reference_unsafe";
        plan->mPlanPath.taskType = TRACKPATH;
        plan->mPlanPath.enableBypass = 1;
        plan->mPlanPath.bypassProcessing = 2;
        plan->mPlanPath.safety = scenario.find("retain_unsafe") != std::string::npos;
        plan->emergencyStop = scenario == "emergency";
        plan->SetCommandState(scenario == "pause" ? 1 : 2);
        ros::param::set("/robot/planning/netcheck", scenario == "network" ? 1 : 0);
        if(scenario.find("short") == 0) {
            plan->mReferPath.x.resize(4);
            plan->mReferPath.y.resize(4);
        }
        if(scenario == "empty") {
            plan->mReferPath.x.clear();
            plan->mReferPath.y.clear();
        }
    }

    ~Fixture() {
        plan->~PathPlanComply();
        ::operator delete(plan);
    }

    Publication Publish() {
        Publication result;
        ros::Publisher path_pub;
        ros::Publisher sound_pub;
        path_pub.name = "plan";
        sound_pub.name = "sound";
        path_pub.capture = [&](const std::type_info &, const void *msg) {
            result.path = *static_cast<const robot::path_plan_msg *>(msg);
            result.path_count++;
        };
        sound_pub.capture = [&](const std::type_info &, const void *msg) {
            result.sound = Trace(*static_cast<const robot::sound_light_msg *>(msg));
            result.sound_count++;
        };
        ros::param::events().clear();
        plan->PublishPlanPath(path_pub, sound_pub);
        result.events = ros::param::events();
        result.state = Trace(plan->mPlanPath) + Trace(plan->mReferPath) + Trace(plan->mPathPlanStatus)
                     + Trace(plan->HornFlag)
                     + Trace(plan->sysTime.vehicleStop) + Trace(plan->sysTime.vehicleRun);
        return result;
    }
};

void SetReferenceNearGantry(PathPlanComply &plan) {
    // 保留正常/短/空路径各自点数；闸机位于自车沿线前方 2 m。
    plan.mNavData.xAxis = 96.14f - 2.0f;
    plan.mNavData.yAxis = -405.63f;
    for(size_t i = 0; i < plan.mReferPath.x.size(); ++i) {
        plan.mReferPath.x[i] = 96.14f - 2.0f + float(i);
        plan.mReferPath.y[i] = -405.63f;
    }
}

void Compare(Publication expected, const Publication &actual, bool closed, const std::string &name) {
    expected.path.safety = expected.path.safety || closed;
    Check(actual.path_count == 1, name + ": one path per tick");
    Check(Trace(actual.path) == Trace(expected.path), name + ": only published safety may change");
    Check(actual.sound_count == expected.sound_count && actual.sound == expected.sound,
          name + ": sound/light publication unchanged");
    Check(actual.events == expected.events, name + ": parameter/publication order unchanged");
    Check(actual.state == expected.state, name + ": original planner state unchanged");
}

void TestSafetyHandover(const std::string &pnc) {
    // 用独立的预期值检查安全原因交接，避免对照实例也误清零时两边同时出错而漏检。
    struct Frame {
        const char *name;
        bool active;
        bool open;
        bool other_safety;
        bool emergency;
        bool expected;
    };
    const Frame frames[] = {
        {"gantry only", true, false, false, false, true},
        {"other cause appears while closed", true, false, true, false, true},
        {"opening preserves other cause", true, true, true, false, true},
        {"inactive closed preserves other cause", false, false, true, false, true},
        {"inactive open preserves other cause", false, true, true, false, true},
        {"other cause clears while inactive", false, false, false, false, false},
        {"emergency appears while closed", true, false, false, true, true},
        {"inactive preserves emergency", false, false, false, true, true},
        {"opening preserves emergency", true, true, false, true, true},
        {"emergency clears while open", true, true, false, false, false},
        {"gantry closes again", true, false, false, false, true},
        {"only gantry cause is released", false, false, false, false, false},
    };
    for(const std::string &scenario : {"forward", "reverse", "manual", "short", "empty"}) {
        Fixture reference(pnc, scenario);
        Fixture subject(pnc, scenario);
        SetReferenceNearGantry(*reference.plan);
        SetReferenceNearGantry(*subject.plan);
        for(const auto &frame : frames) {
            for(PathPlanComply *plan : {reference.plan, subject.plan}) {
                plan->mReferPath.safety = frame.other_safety;
                // 手动/退化分支保留已有内部 safety；模拟原有安全来源在进入输出前的结果。
                plan->mPlanPath.safety = frame.other_safety || frame.emergency;
                plan->emergencyStop = frame.emergency;
            }
            const std::string name = scenario + ": " + frame.name;
            const std::string plan_before = Trace(subject.plan->mPlanPath);
            const std::string refer_before = Trace(subject.plan->mReferPath);
            subject.plan->SetGantryState(frame.active, frame.open);
            Check(Trace(subject.plan->mPlanPath) == plan_before, name + ": input does not overwrite plan");
            Check(Trace(subject.plan->mReferPath) == refer_before, name + ": input does not overwrite reference");
            Publication expected = reference.Publish();
            Publication actual = subject.Publish();
            const bool gantry_stop = frame.active && !frame.open && scenario != "empty";
            const bool expected_safety = scenario == "empty" ? frame.other_safety || frame.emergency : frame.expected;
            Check(actual.path.safety == expected_safety, name + ": independent published safety expectation");
            Check(subject.plan->mPlanPath.safety == (frame.other_safety || frame.emergency),
                  name + ": original safety retained after publication");
            Compare(expected, actual, gantry_stop, name);
        }
    }
}

void TestGantryRouteGeometry(const std::string &pnc) {
    const float gates[][2] = {{96.14f, -405.63f}, {91.13f, -393.80f}};
    for(const auto &gate : gates) {
        for(int direction : {-1, 1}) {
            for(const std::string &gear : {"forward", "reverse"}) {
                // 直线的独立期望：真实点间距为 1 m，正负方向都按路径点序行驶。
                for(double ahead : {-3.0, -0.001, 0.0, 0.001, 3.999, 4.0, 4.001, 5.0,
                                     7.999, 8.0, 8.001, 9.0}) {
                    for(float lateral : {-2.001f, -2.0f, -1.999f, 0.0f, 1.999f, 2.0f, 2.001f}) {
                        Fixture fixture(pnc, gear);
                        auto &plan = *fixture.plan;
                        for(size_t i = 0; i < plan.mReferPath.x.size(); ++i) {
                            plan.mReferPath.x[i] = gate[0] + direction * (float(i) - 6.0f);
                            plan.mReferPath.y[i] = gate[1] + lateral;
                        }
                        plan.mNavData.xAxis = gate[0] - direction * ahead;
                        plan.mNavData.yAxis = gate[1] + lateral;
                        const bool expected = std::abs(lateral) < 2.0f && ahead > 0.0 && ahead < 8.0;
                        Check(plan.IsGantryAheadOnReference() == expected,
                              "two gates, both directions/gears, strict 2 m and forward 8 m boundaries");
                        const auto clear = fixture.Publish();
                        plan.SetGantryState(true, false);
                        Compare(clear, fixture.Publish(), expected, "geometry only gates published safety");
                        Check(plan.mGantryStop == expected, "ineligible route clears cached closed input");
                    }
                }
            }
        }
    }

    Fixture fixture(pnc, "forward");
    auto &plan = *fixture.plan;
    SetReferenceNearGantry(plan);
    const float gate_x = 96.14f, gate_y = -405.63f;
    plan.mReferPath.x = {gate_x - 16, gate_x - 2, gate_x, gate_x + 1};
    plan.mReferPath.y.assign(4, gate_y);
    plan.mNavData.yAxis = gate_y;
    plan.mNavData.xAxis = gate_x - 8.5;
    Check(!plan.IsGantryAheadOnReference(), "ego projection avoids a false stop from nearest-vertex rounding");
    plan.mNavData.xAxis = gate_x - 7.5;
    Check(plan.IsGantryAheadOnReference(), "nonuniform spacing uses projected ego and actual segment lengths");
    plan.mReferPath.x.insert(plan.mReferPath.x.begin(), gate_x - 50);
    plan.mReferPath.y.insert(plan.mReferPath.y.begin(), gate_y);
    Check(plan.IsGantryAheadOnReference(), "points already behind the ego do not inflate forward distance");

    plan.mReferPath.x = {gate_x - 3, gate_x + 3, gate_x + 4};
    plan.mReferPath.y.assign(3, gate_y);
    plan.mNavData.xAxis = gate_x - 2;
    Check(!plan.IsGantryAheadOnReference(), "gate must be near an actual reference point, not only a sparse segment");

    plan.mReferPath.x = {gate_x - 1, gate_x - 1, gate_x, gate_x, gate_x + 1};
    plan.mReferPath.y = {gate_y, gate_y + 6, gate_y + 6, gate_y, gate_y};
    plan.mNavData.xAxis = gate_x - 1;
    Check(!plan.IsGantryAheadOnReference(), "U-shaped path uses 13 m arc length instead of 1 m straight distance");
    plan.mNavData.xAxis = gate_x;
    plan.mNavData.yAxis = gate_y + 3;
    Check(plan.IsGantryAheadOnReference(), "ego projects onto the current descending path segment");

    SetReferenceNearGantry(plan);
    plan.mReferPath.x.insert(plan.mReferPath.x.begin() + 1, plan.mReferPath.x.front());
    plan.mReferPath.y.insert(plan.mReferPath.y.begin() + 1, plan.mReferPath.y.front());
    Check(plan.IsGantryAheadOnReference(), "duplicate points do not add arc length or divide by zero");
    for(int invalid = 0; invalid < 6; ++invalid) {
        Fixture bad(pnc, "forward");
        SetReferenceNearGantry(*bad.plan);
        const float nan = std::numeric_limits<float>::quiet_NaN();
        if(invalid == 0) bad.plan->mNavData.xAxis = nan;
        if(invalid == 1) bad.plan->mReferPath.x[4] = nan;
        if(invalid == 2) bad.plan->mReferPath.y[4] = std::numeric_limits<float>::infinity();
        if(invalid == 3) bad.plan->mReferPath.y.pop_back();
        if(invalid == 4) {
            bad.plan->mReferPath.x.resize(1);
            bad.plan->mReferPath.y.resize(1);
        }
        if(invalid == 5) {
            bad.plan->mReferPath.x.assign(12, gate_x);
            bad.plan->mReferPath.y.assign(12, gate_y);
        }
        Check(!bad.plan->IsGantryAheadOnReference(), "invalid or degenerate geometry cannot enable a gantry override");
        bad.plan->SetGantryState(true, false);
        bad.plan->mPlanPath.safety = true;
        ros::Publisher publisher;
        bad.plan->PublishFinalPlanPath(publisher);
        Check(!bad.plan->mGantryStop && bad.plan->mPlanPath.safety,
              "invalid geometry clears only gantry history and preserves prior safety");
    }
}

void TestGantryHistory(const std::string &pnc) {
    for(const std::string &departure : {"passed", "far", "replacement_route", "empty"}) {
        Fixture fixture(pnc, "forward");
        auto &plan = *fixture.plan;
        SetReferenceNearGantry(plan);
        plan.SetGantryState(true, false);
        Check(fixture.Publish().path.safety, "fresh closed state blocks within the route trigger");
        if(departure == "passed") plan.mNavData.xAxis = 96.14f + 0.1;
        if(departure == "far") plan.mNavData.xAxis = 96.14f - 9.0;
        if(departure == "replacement_route") {
            for(auto &y : plan.mReferPath.y) y += 10.0f;
            plan.mNavData.yAxis += 10.0;
        }
        if(departure == "empty") {
            plan.mReferPath.x.clear();
            plan.mReferPath.y.clear();
        }
        Check(!fixture.Publish().path.safety && !plan.mGantryStop,
              departure + ": route exit clears both the override and old closed input");
        plan.mReferPath.x.resize(12);
        plan.mReferPath.y.resize(12);
        SetReferenceNearGantry(plan);
        Check(!fixture.Publish().path.safety, departure + ": reentry cannot reuse an old closed message");
        plan.SetGantryState(true, false);
        Check(fixture.Publish().path.safety, "new message can trigger again after reentry");
        plan.SetGantryState(true, true);
        Check(!fixture.Publish().path.safety, "opening clears the gantry source");
        plan.SetGantryState(true, false);
        Check(fixture.Publish().path.safety, "closing re-enables an eligible gantry");
        plan.SetGantryState(false, false);
        Check(!fixture.Publish().path.safety, "inactive input clears the gantry source");

        plan.mReferPath.safety = true;
        plan.SetGantryState(true, false);
        fixture.Publish();
        plan.mNavData.xAxis = 96.14f + 1.0;
        Check(fixture.Publish().path.safety && !plan.mGantryStop,
              "leaving gantry scope never clears independent reference safety");
        plan.mReferPath.safety = false;
        plan.emergencyStop = 1;
        plan.SetGantryState(true, false);
        const auto emergency = fixture.Publish();
        Check(emergency.path.safety && emergency.path.desireSpeed == 0 && !plan.mGantryStop,
              "rejecting a distant gantry never clears emergency safety or zero speed");
        plan.emergencyStop = 0;
        Check(!fixture.Publish().path.safety, "cleared independent sources leave no gantry publication residue");
    }
}

int main(int argc, char **argv) {
    Check(argc == 2, "pnc directory supplied");
    PathPlanComply default_plan;
    Check(!default_plan.mGantryStop, "no message defaults to inactive");
    for(bool active : {false, true}) {
        for(bool open : {false, true}) {
            gantry_detect::gantry_state *data = new gantry_detect::gantry_state;
            data->active = active;
            data->gantry_open = open;
            gantry_detect::gantry_state::ConstPtr msg(data);
            GantryStateCallBack(msg);
            Check(pathPlanComply.mGantryStop == (active && !open), "actual node callback mapping");
        }
    }

    for(const std::string &scenario : {"forward", "reverse", "manual", "short", "empty", "startup",
                                       "reference_unsafe", "emergency", "network", "pause",
                                       "manual_retain_unsafe", "short_retain_unsafe"}) {
        Fixture reference(argv[1], scenario);
        Fixture subject(argv[1], scenario);
        SetReferenceNearGantry(*reference.plan);
        SetReferenceNearGantry(*subject.plan);
        Compare(reference.Publish(), subject.Publish(), false, scenario + ": no message");
        // 连续收消息/无新消息/开闭反复，覆盖退化输出在解除后不残留 safety。
        const bool inputs[][2] = {{false, false}, {false, true}, {true, true}, {true, false},
                                  {true, true}, {true, false}, {false, false}, {true, false},
                                  {false, true}};
        for(const auto &input : inputs) {
            subject.plan->SetGantryState(input[0], input[1]);
            for(int tick = 0; tick < 2; tick++) {
                Compare(reference.Publish(), subject.Publish(), input[0] && !input[1] && scenario != "empty", scenario);
            }
        }
        subject.plan->SetGantryState(true, false);
        reference.plan->ResetTaskHistory();
        subject.plan->ResetTaskHistory();
        Compare(reference.Publish(), subject.Publish(), scenario != "empty", scenario + ": task switch");
        reference.plan->resetTask();
        subject.plan->resetTask();
        Compare(reference.Publish(), subject.Publish(), false, scenario + ": task reset clears the reference");
        Check(!subject.plan->mGantryStop, "task reset clears obsolete gantry input with the empty reference");
    }
    TestSafetyHandover(argv[1]);
    TestGantryRouteGeometry(argv[1]);
    TestGantryHistory(argv[1]);
    std::cout << "PASS: " << checks << " gantry checks\n";
    return 0;
}
