// 独立发送互相矛盾的 CAN/导航输入，验证真实规划入口及起步观察。
#define main ExistingGantryTestsMain
#include "../planning/gantry_safety.cpp"
#undef main

void NavigationSpeed(PathPlanComply& plan, double speed) {
    auto nav = plan.mNavData;
    nav.gpsSpeed = speed;
    plan.SetNavigationData(nav);
}

void Arm(PathPlanComply& plan) {
    auto can = plan.mVehicleData;
    can.vehicleSpeed = 99;
    can.controlPanelState = 0;
    plan.SetCanData(can);
    can.controlPanelState = 1;
    can.curGear = GEAR_D;
    plan.SetCanData(can);
}

bool Observing(PathPlanComply& plan) {
    return plan.mStartupObservation.MustStop(plan.mPerceptionSafety.GetConfig(), ros::testTime());
}

int main(int argc, char** argv) {
    if(argc != 2) return 2;
    const double speeds[] = {-100, 0, 0.2, 1, 99, std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity()};
    for(double speed : speeds) {
        Fixture fixture(argv[1], "forward");
        auto& plan = *fixture.plan;
        auto can = plan.mVehicleData;
        can.vehicleSpeed = speed;
        NavigationSpeed(plan, 1.25);
        plan.SetCanData(can);
        Check(plan.mPathPlanStatus.curSpeed == 1.25, "CAN cannot overwrite reported navigation speed");
        plan.mPlanPath.desireSpeed = 3;
        plan.SmoothPlanSpeed();
        Check(std::abs(plan.mPlanPath.desireSpeed - 1.775) < 1e-6, "unloaded smoothing uses navigation");
        can.linkPallet = true;
        can.hookStatus = 0;
        plan.SetCanData(can);
        plan.mPlanPath.desireSpeed = 3;
        plan.SmoothPlanSpeed();
        Check(std::abs(plan.mPlanPath.desireSpeed - 1.425) < 1e-6, "CAN hook/load state still selects smoothing weight");
        NavigationSpeed(plan, 0);
        Check(plan.mPathPlanStatus.curSpeed == 0, "navigation updates reported speed after CAN");
        plan.mPlanPath.desireSpeed = 3;
        plan.SmoothPlanSpeed();
        Check(std::abs(plan.mPlanPath.desireSpeed - 0.3) < 1e-6, "navigation zero replaces preceding moving speed");
    }

    for(double nav_speed : {0.0, 1.0, 3.0}) {
        for(double can_speed : speeds) {
            Fixture fixture(argv[1], "forward");
            auto& plan = *fixture.plan;
            NavigationSpeed(plan, nav_speed);
            auto can = plan.mVehicleData;
            can.vehicleSpeed = can_speed;
            plan.SetCanData(can);
            robot::perception message;
            message.header.stamp = ros::Time(100);
            robot::object object;
            object.id = 7;
            object.x = 108.9;
            object.y = 100;
            object.dx = 1;
            object.dy = 1.2;
            object.confidence = 1;
            object.polygons.resize(4);
            const double dx[] = {0.6, 0.6, -0.6, -0.6};
            const double dy[] = {-0.5, 0.5, 0.5, -0.5};
            for(int i = 0; i < 4; ++i) {
                object.polygons[i].x = object.x + dx[i];
                object.polygons[i].y = object.y + dy[i];
            }
            message.objs.push_back(object);
            plan.SetPlanningPerceptionData(message);
            plan.CheckTrackedReferenceSafety();
            const auto& config = plan.mPerceptionSafety.GetConfig();
            const double expected = std::max(config.static_stop_min_distance,
                nav_speed * config.reaction_time + nav_speed * nav_speed / (2 * config.emergency_deceleration) + config.emergency_margin);
            Check(plan.mPerceptionSafetyResult.object_id == 7, "actual perception obstacle reaches speed-based judgment");
            Check(std::abs(plan.mPerceptionSafetyResult.emergency_distance - expected) < 1e-6,
                  "perception emergency distance uses navigation despite conflicting CAN");
        }
    }

    for(const std::string& scenario : {"forward", "reverse", "manual", "short", "empty", "reference_unsafe", "emergency"}) {
        for(double invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity()}) {
            Fixture fixture(argv[1], scenario);
            const auto expected = fixture.Publish().path;
            NavigationSpeed(*fixture.plan, invalid);
            const auto stopped = fixture.Publish();
            Check(stopped.path_count == 1 && stopped.path.safety && stopped.path.desireSpeed == 0,
                  "invalid navigation stops final planning output including early returns and reverse");
            NavigationSpeed(*fixture.plan, 0.8);
            Check(Trace(fixture.Publish().path) == Trace(expected),
                  "navigation recovery preserves original speed, safety, path and task fields");
        }
    }

    Fixture fixture(argv[1], "forward");
    auto& plan = *fixture.plan;
    Arm(plan);
    NavigationSpeed(plan, 0);
    Check(Observing(plan), "new automatic session waits for observation");
    for(int i = 1; i <= 5; ++i) {
        ros::testTime() = 100 + 0.02 * i;
        auto can = plan.mVehicleData;
        can.vehicleSpeed = 99;
        plan.SetCanData(can);
        Check(Observing(plan), "moving CAN frames cannot finish startup");
    }
    NavigationSpeed(plan, 0.3);
    Check(Observing(plan), "one moving navigation sample cannot finish startup");
    NavigationSpeed(plan, 0.3);
    Check(Observing(plan), "duplicate-time navigation sample cannot finish startup");
    ros::testTime() += 0.02;
    plan.SetCanData(plan.mVehicleData);
    Check(Observing(plan), "CAN cannot count as second navigation sample");
    NavigationSpeed(plan, 0.3);
    Check(!Observing(plan), "second fresh navigation sample finishes startup");
    NavigationSpeed(plan, 0);
    Check(!Observing(plan), "stopping again does not rearm startup");

    for(double interruption : {0.2, std::numeric_limits<double>::quiet_NaN()}) {
        Arm(plan);
        NavigationSpeed(plan, 0.3);
        ros::testTime() += 0.02;
        NavigationSpeed(plan, interruption);
        ros::testTime() += 0.02;
        NavigationSpeed(plan, 0.3);
        Check(Observing(plan), "threshold/invalid navigation interrupts consecutive motion");
        ros::testTime() += 0.02;
        NavigationSpeed(plan, 0.3);
        Check(!Observing(plan), "two new valid navigation samples recover motion confirmation");
    }
    Arm(plan);
    ros::testTime() += 0.7;
    NavigationSpeed(plan, 1);
    ros::testTime() += 0.02;
    NavigationSpeed(plan, 1);
    Check(Observing(plan), "fresh navigation cannot use stale CAN mode/gear");
    plan.SetCanData(plan.mVehicleData);
    NavigationSpeed(plan, 1);
    Check(Observing(plan), "fresh CAN restarts navigation confirmation");
    ros::testTime() += 0.02;
    NavigationSpeed(plan, 1);
    Check(!Observing(plan), "fresh CAN plus two navigation samples confirms motion");

    Arm(plan);
    NavigationSpeed(plan, 0);
    ros::testTime() += 0.7;
    plan.SetCanData(plan.mVehicleData);
    Check(Observing(plan), "fresh CAN cannot hide navigation timeout");
    auto can = plan.mVehicleData;
    can.curGear = GEAR_R;
    plan.SetCanData(can);
    NavigationSpeed(plan, 1);
    ros::testTime() += 0.02;
    NavigationSpeed(plan, 1);
    can.curGear = GEAR_D;
    plan.SetCanData(can);
    NavigationSpeed(plan, 0);
    Check(Observing(plan), "reverse motion does not finish D startup observation");
    std::cout << "PASS navigation planning: " << checks << " checks\n";
}
