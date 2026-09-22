// 验证已起步后不会重入周边观察，旧 T 区等待仍不生效；其他停车、声光和轨迹复用保持。
#define main ExistingGantryTestsMain
#include "gantry_safety.cpp"
#undef main
#include <cmath>

robot::object ObservationObject(double x, double y) {
    robot::object object;
    object.x = x;
    object.y = y;
    object.dx = object.dy = 0.5;
    return object;
}

void TestNoObservationHold(const std::string &pnc) {
    for(bool loaded : {false, true}) {
        for(const std::string &scene : {"clear", "around", "junction"}) {
            Fixture fixture(pnc, "startup");
            auto &plan = *fixture.plan;
            auto can = plan.mVehicleData;
            can.vehicleSpeed = 0;
            can.linkPallet = loaded;
            can.hookStatus = ACTUATOR_UP_END;
            SetVehicleFeedback(plan, can);
            if(scene == "around") {
                plan.mPerception.objs = {ObservationObject(102.4, 100), ObservationObject(100, 100.85),
                                        ObservationObject(100, 99.15), ObservationObject(99.1, 100)};
            } else if(scene == "junction") {
                plan.mNavData.xAxis = plan.mNavData.yAxis = 0;
                plan.mPerception.objs = {ObservationObject(5, 0)};
            }
            for(int tick = 0; tick < 40; ++tick) {
                ros::testTime() += 0.1;
                plan.sysTime.now = ros::testTime();
                const auto result = fixture.Publish();
                Check(std::abs(result.path.desireSpeed - 0.9) < 1e-6,
                      scene + ": first and later outputs retain normal acceleration smoothing");
                Check(!result.path.safety && ros::param::values()["/canbus/light"] != "7",
                      scene + ": final output adds no area safety or observation alarm");
                Check(result.path.x == plan.mReferPath.x && result.path.y == plan.mReferPath.y,
                      scene + ": observation removal never changes the reference coordinates");
            }
            plan.ResetTaskHistory();
            Check(std::abs(fixture.Publish().path.desireSpeed - 0.9) < 1e-6,
                  "new-task history reset cannot reintroduce an observation delay");
        }
    }
}

void TestOtherStopsAndSound(const std::string &pnc) {
    for(const std::string &cause : {"emergency", "network", "pause", "manual", "short", "empty", "zero"}) {
        Fixture fixture(pnc, cause);
        if(cause == "zero") fixture.plan->mReferPath.desireSpeed = 0;
        const auto result = fixture.Publish();
        Check(result.path.desireSpeed == 0, cause + ": independent stop still publishes zero speed");
        Check(result.path_count == 1, cause + ": exactly one final path is published");
    }
    Fixture fixture(pnc, "forward");
    fixture.plan->sysTime.vehicleStop = 100;
    fixture.plan->sysTime.vehicleRun = 100;
    fixture.plan->sysTime.taskStart = 100;
    fixture.Publish();
    Check(ros::param::values()["/canbus/horn"] == "1", "startup horn remains independent of removed observation");
    fixture.plan->mVehicleData.linkPallet = true;
    fixture.plan->mVehicleData.hookStatus = ACTUATOR_DOWN_END;
    Check(std::abs(fixture.Publish().path.desireSpeed - 1.02) < 1e-6,
          "loaded acceleration smoothing with hook not raised is unchanged");

    fixture.plan->history_risk_vec[0].push_back(ObservationObject(103, 100));
    fixture.plan->safety_check_counter_percep = 2;
    fixture.plan->safety_check_counter_refer = 2;
    fixture.plan->emergencyStop = 1;
    fixture.plan->SetGantryState(true, false);
    fixture.plan->SetCommandState(1);
    fixture.plan->ResetTaskHistory();
    Check(fixture.plan->emergencyStop == 1 && fixture.plan->mGantryStop,
          "task switch retains emergency and gantry inputs");
    Check(fixture.plan->command_state == 2 && fixture.plan->history_risk_vec[0].empty() &&
              fixture.plan->safety_check_counter_percep == 0 && fixture.plan->safety_check_counter_refer == 0,
          "task switch still resets cloud pause and legacy collision history");
    const auto stopped = fixture.Publish();
    Check(stopped.path.desireSpeed == 0 && stopped.path.safety,
          "task switch cannot release an independent stop");
}

void TestTrajectoryReuse(const std::string &pnc) {
    struct CASE_S {
        int points;
        double x;
        bool collision;
        bool reuse;
    };
    const CASE_S cases[] = {{40, 100, false, true}, {20, 100, false, false},
                            {40, 100, true, false}, {40, 124, false, false}, {40, 123.9, false, true}};
    for(const auto &item : cases) {
        Fixture fixture(pnc, "forward");
        auto &plan = *fixture.plan;
        plan.mNavData.xAxis = item.x;
        OriginalInsData current{};
        current.x = 501;
        current.y = 502;
        plan.path_final = {current};
        for(int i = 0; i < item.points; ++i) {
            OriginalInsData point{};
            point.x = 95 + i;
            point.y = 100;
            plan.path_final_last.push_back(point);
        }
        if(item.collision) plan.mPerception.objs = {ObservationObject(105, 100)};
        plan.ReusePreviousTrajectory();
        Check((plan.path_final.front().x != 501) == item.reuse,
              "trajectory reuse keeps length, collision and endpoint-distance conditions");
        Check(plan.path_final_last.size() == plan.path_final.size() &&
                  plan.path_final_last.front().x == plan.path_final.front().x,
              "trajectory history still updates when reused or rejected");
    }
}

int main(int argc, char **argv) {
    Check(argc == 2, "pnc directory supplied");
    TestNoObservationHold(argv[1]);
    TestOtherStopsAndSound(argv[1]);
    TestTrajectoryReuse(argv[1]);
    std::cout << "PASS startup removal: " << checks << " checks\n";
    return 0;
}
