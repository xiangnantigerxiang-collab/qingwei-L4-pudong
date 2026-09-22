// 从真实 CAN/定位/感知入口到最终发布验证起步检查，复用常规避障及消息捕获夹具。
#define main ExistingGantryTestsMain
#include "gantry_safety.cpp"
#undef main

void CanMsgCallBack(const robot::can_msg::ConstPtr& msg);
void NavigationMsgCallBack(const robot::navigation_msg::ConstPtr& msg);
void PlanningPerceptionCallBack(const robot::perception::ConstPtr& msg);
void PerceptionMsgCallBack(const robot::perception::ConstPtr& msg);

robot::object ObservedObject(int id, double x, double y) {
    robot::object object;
    object.id = id;
    object.x = x;
    object.y = y;
    object.dx = 1;
    object.dy = 1.2;
    object.confidence = 0.8;
    object.polygons.resize(4);
    const double dx[] = {0.6, 0.6, -0.6, -0.6};
    const double dy[] = {-0.5, 0.5, 0.5, -0.5};
    for(int i = 0; i < 4; ++i) {
        object.polygons[i].x = x + dx[i];
        object.polygons[i].y = y + dy[i];
    }
    return object;
}

robot::perception Observations(double time, const std::vector<robot::object>& objects = {}) {
    robot::perception message;
    message.header.stamp = ros::Time(time);
    message.objs = objects;
    return message;
}

void ArmStartup(Fixture& fixture, double time = 100) {
    auto can = fixture.plan->mVehicleData;
    can.controlPanelState = 0;
    can.vehicleSpeed = 0;
    ros::testTime() = time - 0.1;
    SetVehicleFeedback(*fixture.plan, can);
    ros::testTime() = time;
    can.controlPanelState = 1;
    SetVehicleFeedback(*fixture.plan, can);
    fixture.plan->SetNavigationData(fixture.plan->mNavData);
}

void StartupFrame(Fixture& fixture, double time, const std::vector<robot::object>& objects = {}) {
    ros::testTime() = time;
    fixture.plan->SetStartupPerceptionData(Observations(time, objects));
}

void TestStartupOutputs(const std::string& pnc) {
    for(const std::string& gear : {"forward"}) {
        for(int side = 0; side < 4; ++side) {
            Fixture fixture(pnc, gear);
            ArmStartup(fixture);
            auto output = fixture.Publish();
            Check(output.path.safety && output.path.desireSpeed == 0, "switching auto waits for fresh observation");
            StartupFrame(fixture, 100);
            const auto clear = fixture.Publish();
            Check(!clear.path.safety && clear.path.desireSpeed > 0, "clear surroundings allow startup immediately");
            // 真实框半长/半宽为 0.5/0.6；四个中心的最近边距均恰为 2 m。
            const double x[] = {104.9, 96.1, 100, 100};
            const double y[] = {100, 100, 104.2, 95.8};
            auto object = ObservedObject(8, x[side], y[side]);
            object.type = 4;
            ros::testTime() = 100.1;
            const auto before = fixture.Publish();
            StartupFrame(fixture, 100.1, {object});
            fixture.plan->SetPlanningPerceptionData(Observations(100.1, {object}));
            const auto blocked = fixture.Publish();
            Check(blocked.path.safety && blocked.path.desireSpeed == 0, "all four body sides block D startup");
            auto expected = clear.path;
            expected.safety = true;
            expected.desireSpeed = 0;
            Check(Trace(blocked.path) == Trace(expected), "startup changes only final safety and speed");
            Check(blocked.state == before.state, "startup override does not contaminate persistent plan state");
            Check(blocked.path_count == 1 && blocked.sound == before.sound && blocked.events == before.events,
                  "startup retains publication count, sound and parameter order");
            StartupFrame(fixture, 100.2);
            Check(Trace(fixture.Publish().path) == Trace(clear.path), "new clear frame removes only startup override");
        }
    }
}

void TestStartupLifecycle(const std::string& pnc) {
    Fixture fixture(pnc, "forward");
    ArmStartup(fixture);
    StartupFrame(fixture, 100, {ObservedObject(7, 100, 103)});
    auto can = fixture.plan->mVehicleData;
    can.vehicleSpeed = 0.3f;
    ros::testTime() = 100.1;
    SetVehicleFeedback(*fixture.plan, can);
    for(int i = 0; i < 5; ++i) Check(fixture.Publish().path.safety, "publication is not a fresh moving navigation frame");
    can.vehicleSpeed = 0.2f;
    ros::testTime() = 100.15;
    SetVehicleFeedback(*fixture.plan, can);
    can.vehicleSpeed = 0.3f;
    ros::testTime() = 100.2;
    SetVehicleFeedback(*fixture.plan, can);
    Check(fixture.Publish().path.safety, "threshold sample breaks consecutive motion in real Comply");
    ros::testTime() = 100.25;
    SetVehicleFeedback(*fixture.plan, can);
    Check(!fixture.Publish().path.safety, "second moving navigation frame exits startup");
    can.vehicleSpeed = 0;
    ros::testTime() = 110;
    SetVehicleFeedback(*fixture.plan, can);
    StartupFrame(fixture, 110, {ObservedObject(7, 100, 100)});
    fixture.plan->ResetTaskHistory();
    Check(!fixture.Publish().path.safety, "task change and automatic stop do not rearm observation");
    ArmStartup(fixture, 111);
    Check(fixture.Publish().path.safety, "next manual-to-auto transition rearms");
    StartupFrame(fixture, 111);
    Check(!fixture.Publish().path.safety, "new auto session can clear normally");
    can.controlPanelState = 0;
    SetVehicleFeedback(*fixture.plan, can);
    Check(!fixture.Publish().path.safety, "manual does not retain a startup-only safety latch");
}

void TestOtherSafetySources(const std::string& pnc) {
    for(const std::string& cause : {"forward", "reverse", "reference_unsafe", "emergency", "network", "pause", "short", "empty"}) {
        for(bool gantry : {false, true}) {
            Fixture fixture(pnc, cause);
            ArmStartup(fixture);
            fixture.plan->SetGantryState(gantry, false);
            StartupFrame(fixture, 100);
            const auto clear = fixture.Publish();
            StartupFrame(fixture, 100.1, {ObservedObject(9, 100, 100)});
            const auto blocked = fixture.Publish();
            if(cause == "reverse") {
                Check(Trace(clear.path) == Trace(blocked.path), "R retains original safety sources without startup override");
            } else {
                Check(blocked.path.safety && blocked.path.desireSpeed == 0, cause + ": independent startup stop wins");
            }
            Check(blocked.path_count == 1, cause + ": normal and early-return paths publish once");
            StartupFrame(fixture, 100.2);
            Check(Trace(fixture.Publish().path) == Trace(clear.path), cause + ": clearing startup preserves all other sources");
            if(cause == "reference_unsafe" || cause == "emergency") {
                Check(fixture.Publish().path.safety, cause + ": independent safety not cleared by empty surroundings");
            } else {
                Check(!fixture.Publish().path.safety, cause + ": distant gantry adds no stop after startup clears");
            }
        }
    }

    Fixture near(pnc, "forward");
    SetReferenceNearGantry(*near.plan);
    ArmStartup(near);
    near.plan->SetGantryState(true, false);
    StartupFrame(near, 100);
    Check(near.Publish().path.safety, "eligible gantry remains after startup gets a clear frame");
    StartupFrame(near, 100.1, {ObservedObject(91, near.plan->mNavData.xAxis, near.plan->mNavData.yAxis)});
    near.plan->SetGantryState(true, true);
    auto blocked = near.Publish().path;
    Check(blocked.safety && blocked.desireSpeed == 0, "opening gantry does not clear a real startup obstacle");
    near.plan->SetGantryState(true, false);
    near.plan->mNavData.xAxis = 96.14f + 0.5;
    blocked = near.Publish().path;
    Check(blocked.safety && blocked.desireSpeed == 0 && !near.plan->mGantryStop,
          "passed gantry history clears without overriding independent startup safety");
    StartupFrame(near, 100.2);
    Check(!near.Publish().path.safety, "clearing startup after leaving gantry range leaves no stale safety");
}

void TestRawNavigationAndMissingInputs(const std::string& pnc) {
    Fixture fixture(pnc, "forward");
    ArmStartup(fixture);
    StartupFrame(fixture, 100, {ObservedObject(7, 100, 103)});
    fixture.plan->mNavData.xAxis = 0;
    fixture.plan->mNavData.yAxis = 0;
    Check(fixture.Publish().path.safety, "local operation pose cannot move the guard away from map obstacles");
    StartupFrame(fixture, 100.1);
    Check(!fixture.Publish().path.safety, "raw map pose remains valid for clear frame");
    ros::testTime() = 100.71;
    Check(fixture.Publish().path.safety && fixture.Publish().path.desireSpeed == 0,
          "missing fresh inputs cannot be interpreted as clear surroundings");
}

void TestRealNodeCallbacks() {
    // 全局实例走实际 node 回调，证明 ACC 不会屏蔽起步观察和行驶安全输入。
    ros::testTime() = 200;
    boost::shared_ptr<robot::can_msg> can(new robot::can_msg);
    can->curGear = GEAR_D;
    can->controlPanelState = 0;
    CanMsgCallBack(can);
    can->controlPanelState = 1;
    CanMsgCallBack(can);
    boost::shared_ptr<robot::navigation_msg> nav(new robot::navigation_msg);
    nav->heading = 90;
    NavigationMsgCallBack(nav);
    pathPlanComply.AccSwitch = 1;
    for(int type = 0; type <= 4; ++type) {
        ros::testTime() = 200 + 0.01 * type;
        auto object = ObservedObject(type + 1, 0, -3);
        object.type = type;
        object.vx = -2;
        boost::shared_ptr<robot::perception> message(new robot::perception(Observations(ros::testTime(), {object})));
        PlanningPerceptionCallBack(message);
        Check(pathPlanComply.mStartupObservation.MustStop(pathPlanComply.mPerceptionSafety.GetConfig(), ros::testTime()) == (type != 2),
              "actual callback excludes left-second while retaining rear/right/oncoming targets even with ACC");
        Check(pathPlanComply.mPerceptionSafety.HasInput(), "ACC never blocks the regular safety observation stream");
    }
    ros::testTime() = 200.1;
    boost::shared_ptr<robot::perception> empty(new robot::perception(Observations(200.1)));
    PlanningPerceptionCallBack(empty);
    Check(!pathPlanComply.mStartupObservation.MustStop(pathPlanComply.mPerceptionSafety.GetConfig(), 200.1),
          "real callback forwards clear frames to startup regardless of ACC switch");
    pathPlanComply.AccSwitch = 0;
}

void TestLeftSecondLaneInputs(const std::string& pnc) {
    for(const std::string& cause : {"forward", "reverse", "reference_unsafe", "emergency", "network", "pause", "short"}) {
        Fixture reference(pnc, cause), subject(pnc, cause);
        ArmStartup(reference);
        ArmStartup(subject);
        auto object = ObservedObject(7, 103, 100);
        object.type = 2;
        for(int tick = 0; tick < 3; ++tick) {
            const double now = 100 + tick * 0.1;
            StartupFrame(reference, now);
            StartupFrame(subject, now, {object});
            const auto message = Observations(now, {object});
            const auto original = Trace(message);
            reference.plan->SetPlanningPerceptionData(Observations(now));
            subject.plan->SetPlanningPerceptionData(message);
            reference.plan->SetPerceptionData(Observations(now));
            subject.plan->SetPerceptionData(message);
            Check(subject.plan->mPerception.objs.empty(), "legacy snapshot excludes left-second on every frame");
            Check(Trace(message) == original, "both consumers leave the upstream message unchanged");
            Compare(reference.Publish(), subject.Publish(), false, cause + ": left-second equals empty input");
        }
        if(cause != "reverse") {
            object.type = 0;
            StartupFrame(subject, 100.3, {object});
            Check(subject.Publish().path.safety, "retained nearby target still blocks startup");
            object.type = 2;
            StartupFrame(subject, 100.4, {object});
            StartupFrame(reference, 100.4);
            Compare(reference.Publish(), subject.Publish(), false, cause + ": startup reclassification clears only its own cache");
        }
    }

    // 真正的 /perception 回调，验证左二剔除与独立前扫描保留。
    ros::testTime() = 300;
    robot::navigation_msg nav;
    nav.xAxis = nav.yAxis = 100;
    nav.heading = 90;
    pathPlanComply.SetNavigationData(nav);
    pathPlanComply.AccSwitch = 0;
    auto removed = ObservedObject(71, 103, 100);
    removed.type = 2;
    auto retained = ObservedObject(72, 104, 100);
    pathPlanComply.mPerceptionFrontScan.objs = {retained};
    robot::perception::ConstPtr message(new robot::perception(Observations(300, {removed})));
    PerceptionMsgCallBack(message);
    Check(pathPlanComply.mPerception.objs.size() == 1 && pathPlanComply.mPerception.objs[0].id == 72,
          "actual legacy callback excludes left-second while retaining independent front scan");
    pathPlanComply.mPerceptionFrontScan.objs.clear();
}

void TestForwardOnly(const std::string& pnc) {
    // 非 D 挡的正常/退化输出均不得受新增观察干扰，其他原有停车来源继续生效。
    for(const std::string& cause : {"reverse", "short", "empty", "reference_unsafe", "emergency", "network", "pause"}) {
        for(bool gantry : {false, true}) {
            Fixture fixture(pnc, cause);
            Fixture reference(pnc, cause);
            for(Fixture* target : {&fixture, &reference}) {
                auto can = target->plan->mVehicleData;
                can.curGear = GEAR_R;
                SetVehicleFeedback(*target->plan, can);
                ArmStartup(*target);
                target->plan->SetGantryState(gantry, false);
            }
            for(int frame = 0; frame < 4; ++frame) {
                auto message = Observations(100 + 0.01 * frame, {ObservedObject(71, 100, 100)});
                if(frame == 1) message.objs[0].polygons.clear();
                if(frame == 2) message.header.stamp = ros::Time(1);
                if(frame == 3) message.objs.clear();
                ros::testTime() = 100 + 0.01 * frame;
                fixture.plan->SetStartupPerceptionData(message);
                Check(!fixture.plan->mStartupObservation.MustStop(fixture.plan->mPerceptionSafety.GetConfig(), ros::testTime()),
                      "R ignores startup obstacles and invalid or missing inputs");
                const auto expected = reference.Publish();
                Compare(expected, fixture.Publish(), false, cause + ": R publication unaffected by startup observation");
            }
        }
    }

    Fixture fixture(pnc, "forward");
    ArmStartup(fixture);
    auto can = fixture.plan->mVehicleData;
    can.vehicleSpeed = 0.3;
    ros::testTime() = 100.05;
    SetVehicleFeedback(*fixture.plan, can);
    Check(fixture.Publish().path.safety, "first D moving sample still waits");
    can.curGear = GEAR_R;
    for(int i = 0; i < 3; ++i) {
        ros::testTime() = 100.1 + 0.05 * i;
        SetVehicleFeedback(*fixture.plan, can);
        StartupFrame(fixture, ros::testTime(), {ObservedObject(72, 100, 100)});
        Check(!fixture.Publish().path.safety, "D to R immediately removes only startup override");
    }
    can.curGear = GEAR_D;
    can.vehicleSpeed = 0;
    ros::testTime() = 100.25;
    SetVehicleFeedback(*fixture.plan, can);
    Check(fixture.Publish().path.safety, "R motion cannot count as successful D startup");
    fixture.plan->SetStartupPerceptionData(Observations(100.2));
    Check(fixture.Publish().path.safety, "frame from before returning to D cannot grant clearance");
    StartupFrame(fixture, 100.25);
    Check(!fixture.Publish().path.safety, "fresh clear D frame permits startup");
    StartupFrame(fixture, 100.3, {ObservedObject(72, 100, 100)});
    can.vehicleSpeed = 0.3;
    SetVehicleFeedback(*fixture.plan, can);
    Check(fixture.Publish().path.safety, "D confirmation restarts after gear interruption");
    ros::testTime() = 100.35;
    SetVehicleFeedback(*fixture.plan, can);
    Check(!fixture.Publish().path.safety, "two D moving samples complete startup");
    for(int gear : {GEAR_R, GEAR_N, GEAR_D}) {
        can.curGear = gear;
        can.vehicleSpeed = 0;
        ros::testTime() += 0.1;
        SetVehicleFeedback(*fixture.plan, can);
        Check(!fixture.plan->mStartupObservation.MustStop(fixture.plan->mPerceptionSafety.GetConfig(), ros::testTime()),
              "completed D startup is not rearmed by gear changes in same automatic session");
    }

    // 真实回调：R 挡 ACC 开关继续屏蔽常规输入，D 挡保留今天修复的持续输入。
    pathPlanComply.mPerceptionSafety = planning_perception::PerceptionSafety();
    pathPlanComply.mStartupObservation = planning_perception::StartupObservation();
    boost::shared_ptr<robot::can_msg> input(new robot::can_msg);
    input->curGear = GEAR_R;
    input->controlPanelState = 1;
    ros::testTime() = 300;
    CanMsgCallBack(input);
    pathPlanComply.AccSwitch = 1;
    boost::shared_ptr<robot::perception> message(new robot::perception(Observations(300)));
    PlanningPerceptionCallBack(message);
    Check(!pathPlanComply.mPerceptionSafety.HasInput(), "R callback preserves original ACC input gate");
    Check(!pathPlanComply.mStartupObservation.MustStop(pathPlanComply.mPerceptionSafety.GetConfig(), 300),
          "R first automatic frame does not arm an effective startup stop");
    input->curGear = GEAR_D;
    CanMsgCallBack(input);
    PlanningPerceptionCallBack(message);
    Check(pathPlanComply.mPerceptionSafety.HasInput(), "D callback still accepts regular input with ACC enabled");
    pathPlanComply.AccSwitch = 0;
}

void TestHandoverToRegularSafety(const std::string& pnc) {
    Fixture fixture(pnc, "forward");
    ArmStartup(fixture);
    StartupFrame(fixture, 100);
    auto can = fixture.plan->mVehicleData;
    can.vehicleSpeed = 0.3f;
    ros::testTime() = 100.05;
    SetVehicleFeedback(*fixture.plan, can);
    ros::testTime() = 100.1;
    SetVehicleFeedback(*fixture.plan, can);
    for(int i = 0; i < 3; ++i) {
        const double time = 100.2 + i * 0.1;
        ros::testTime() = time;
        const auto message = Observations(time, {ObservedObject(15, 102, 100)});
        fixture.plan->SetStartupPerceptionData(message);
        fixture.plan->SetPlanningPerceptionData(message);
        fixture.plan->sysTime.now = time;
        fixture.plan->mReferPath.safety = false;
        fixture.plan->mReferPath.desireSpeed = 3;
        fixture.plan->CheckForwardReferenceSafety();
        Check(fixture.Publish().path.safety, "regular own-lane near emergency applies immediately after startup");
    }
    Check(fixture.Publish().path.desireSpeed == 0, "regular emergency still stops vehicle after handover");
}

void TestStationaryObstacleClears(const std::string& pnc) {
    Fixture fixture(pnc, "forward");
    ArmStartup(fixture);
    auto can = fixture.plan->mVehicleData;
    can.vehicleSpeed = 0;
    auto object = ObservedObject(31, 100, 103);
    object.type = 4;  // 只由完整周边观察触发，避免将常规避障的解除迟滞混入本项。
    for(int frame = 0; frame < 100; ++frame) {
        const double now = 100 + frame * 0.1;
        const bool occupied = frame < 3;
        ros::testTime() = now;
        fixture.plan->sysTime.now = now;
        SetVehicleFeedback(*fixture.plan, can);
        fixture.plan->SetNavigationData(fixture.plan->mNavData);
        const auto message = Observations(now, occupied ? std::vector<robot::object>{object} : std::vector<robot::object>{});
        fixture.plan->SetStartupPerceptionData(message);
        fixture.plan->SetPlanningPerceptionData(message);
        fixture.plan->mReferPath.safety = false;
        fixture.plan->mReferPath.desireSpeed = 3;
        fixture.plan->CheckForwardReferenceSafety();
        const auto output = fixture.Publish();
        Check(output.path.safety == occupied, "stationary guard releases as soon as valid empty frame arrives");
        Check(occupied ? output.path.desireSpeed == 0 : output.path.desireSpeed > 0,
              "vehicle need not exceed startup speed to receive clear permission");
        const auto status = fixture.plan->mStartupObservation.CheckStatus(fixture.plan->mPerceptionSafety.GetConfig(), now);
        Check(std::strcmp(status.reason, occupied ? "nearby_obstacle" : "clear") == 0,
              "still-observing diagnostics distinguish obstruction from clearance throughout stationary sequence");
    }
}

int main(int argc, char** argv) {
    Check(argc == 2 || argc == 3, "pnc directory supplied");
    if(argc == 3 && std::string(argv[2]) == "--left-second-only") {
        TestLeftSecondLaneInputs(argv[1]);
        TestOtherSafetySources(argv[1]);
        TestForwardOnly(argv[1]);
        TestRealNodeCallbacks();
        std::cout << "PASS left-second input and safety isolation: " << checks << " checks\n";
        return 0;
    }
    if(argc == 3 && std::string(argv[2]) == "--gear-only") {
        TestForwardOnly(argv[1]);
        std::cout << "PASS D-only startup integration: " << checks << " checks\n";
        return 0;
    }
    TestForwardOnly(argv[1]);
    TestLeftSecondLaneInputs(argv[1]);
    TestStartupOutputs(argv[1]);
    TestStartupLifecycle(argv[1]);
    TestOtherSafetySources(argv[1]);
    TestRawNavigationAndMissingInputs(argv[1]);
    TestRealNodeCallbacks();
    TestHandoverToRegularSafety(argv[1]);
    TestStationaryObstacleClears(argv[1]);
    std::cout << "PASS startup observation integration: " << checks << " checks\n";
}
