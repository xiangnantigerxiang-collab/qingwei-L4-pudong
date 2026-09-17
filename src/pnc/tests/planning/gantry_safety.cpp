// 验证实际发布消息，只允许闸机条件改变 safety；原有状态、速度、声光及参数事件均保持。
#include "test_access.h"
#include "test_trace.h"
#include "gantry_detect/gantry_state.h"
#include <cstdlib>
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
        plan->InitSafetyCheck = scenario == "startup" ? 0 : 1;
        plan->mNavData.xAxis = 100.0;
        plan->mNavData.yAxis = 100.0;
        plan->mNavData.heading = 90.0;
        robot::can_msg can;
        can.controlPanelState = scenario.find("manual") == 0 ? 0 : 1;
        can.curGear = scenario == "reverse" ? GEAR_R : GEAR_D;
        can.vehicleSpeed = 0.8;
        plan->SetCanData(can);
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
                     + Trace(plan->InitSafetyCheck) + Trace(plan->HornFlag)
                     + Trace(plan->history_unsafe) + Trace(plan->unsafe_vec)
                     + Trace(plan->sysTime.vehicleStop) + Trace(plan->sysTime.vehicleRun);
        return result;
    }
};

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
            Check(actual.path.safety == frame.expected, name + ": independent published safety expectation");
            Check(subject.plan->mPlanPath.safety == (frame.other_safety || frame.emergency),
                  name + ": original safety retained after publication");
            Compare(expected, actual, frame.active && !frame.open, name);
        }
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
        Compare(reference.Publish(), subject.Publish(), false, scenario + ": no message");
        // 连续收消息/无新消息/开闭反复，覆盖退化输出在解除后不残留 safety。
        const bool inputs[][2] = {{false, false}, {false, true}, {true, true}, {true, false},
                                  {true, true}, {true, false}, {false, false}, {true, false},
                                  {false, true}};
        for(const auto &input : inputs) {
            subject.plan->SetGantryState(input[0], input[1]);
            for(int tick = 0; tick < 2; tick++) {
                Compare(reference.Publish(), subject.Publish(), input[0] && !input[1], scenario);
            }
        }
        subject.plan->SetGantryState(true, false);
        reference.plan->ResetTaskHistory();
        subject.plan->ResetTaskHistory();
        Compare(reference.Publish(), subject.Publish(), true, scenario + ": task switch");
        reference.plan->resetTask();
        subject.plan->resetTask();
        Compare(reference.Publish(), subject.Publish(), true, scenario + ": task reset");
    }
    TestSafetyHandover(argv[1]);
    std::cout << "PASS: " << checks << " gantry checks\n";
    return 0;
}
