// 复用既有实际发布捕获夹具，新增链必须服从原有独立安全来源。
#define main ExistingGantryTestsMain
#include "gantry_safety.cpp"
#undef main
#include <yaml-cpp/yaml.h>

void PlanningPerceptionCallBack(const robot::perception::ConstPtr& msg);

robot::object ObservedObject(int id, double x, double y, double vx = 0, double vy = 0) {
    robot::object object;
    object.id = id;
    object.x = x;
    object.y = y;
    object.dx = 1;
    object.dy = 1.2;
    object.vx = vx;
    object.vy = vy;
    object.heading = 0;
    object.type = 0;
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

robot::perception Observations(double time, std::vector<robot::object> objects = {}) {
    robot::perception result;
    result.header.stamp = ros::Time(time);
    result.objs = std::move(objects);
    return result;
}

void Tick(Fixture& fixture, double time, bool prior_safety = false, double speed = 3) {
    ros::testTime() = time;
    fixture.plan->sysTime.now = time;
    fixture.plan->mReferPath.safety = prior_safety;
    fixture.plan->mReferPath.desireSpeed = speed;
    fixture.plan->CheckForwardReferenceSafety();
}

void SetReferenceMapLimit(PathPlanComply &plan, float limit) {
    plan.mDrivingPath.resize(plan.mReferPath.x.size());
    plan.mKeypoint = 0;
    for(size_t i = 0; i < plan.mDrivingPath.size(); ++i) {
        plan.mDrivingPath[i].x_axis = plan.mReferPath.x[i];
        plan.mDrivingPath[i].y_axis = plan.mReferPath.y[i];
        plan.mDrivingPath[i].heading = 90;
        plan.mDrivingPath[i].map_speed_limit = limit;
    }
}

void TestEarlyStaticPublishing(const std::string& pnc) {
    const auto parameters = YAML::LoadFile(pnc + "/param/perception_safety.yaml")["/robot/planning/perception_safety"];
    for(int scene = 0; scene < 7; ++scene) {
        Fixture fixture(pnc, "forward");
        const std::string prefix = "/robot/planning/perception_safety/";
        for(auto entry : parameters) {
            const auto key = entry.first.as<std::string>();
            if(key == "confirmation_hits") {
                ros::param::set(prefix + key, entry.second.as<int>());
            } else {
                ros::param::set(prefix + key, entry.second.as<double>());
            }
        }
        Check(fixture.plan->ConfigurePerceptionSafety(), "actual launch YAML configures early braking");
        Check(fixture.plan->mPerceptionSafety.GetConfig().static_stop_min_distance == 3 &&
                  fixture.plan->mPerceptionSafety.GetConfig().front_no_confirmation_distance == 6,
              "actual launch YAML loads the three-metre floor and six-metre immediate range");
        fixture.plan->mNavData.gpsSpeed = 1;
        const double gap = scene == 0 || scene == 5 ? 2.8 : 6.5;
        auto object = ObservedObject(7, 100 + 2.3 + gap + 0.25, scene == 6 ? 104 : 100);
        object.dx = object.dy = 0.5;
        for(auto& point : object.polygons) {
            point.x = object.x + (point.x > object.x ? 0.25 : -0.25);
            point.y = object.y + (point.y > object.y ? 0.25 : -0.25);
        }
        object.vx = scene == 2 ? 0.5 : 0;
        object.type = scene >= 3 && scene <= 5 ? scene - 2 : 0;
        for(int tick = 0; tick < 3; ++tick) {
            const double now = 100 + tick * 0.1;
            ros::testTime() = now;
            fixture.plan->SetPlanningPerceptionData(Observations(now, {object}));
            Tick(fixture, now, false, 1);
            const auto output = fixture.Publish().path;
            Check(output.safety == (scene == 0), "published own-lane near emergency bypasses confirmation");
            if(scene == 0) {
                Check(output.desireSpeed == 0,
                      "near three-metre obstacle stops on first publication");
            } else if(scene == 1) {
                Check(output.desireSpeed < 1 && output.desireSpeed > 0,
                      "own-lane static obstacle six metres ahead slows final publication without safety");
            } else {
                Check(output.desireSpeed == 1,
                      "static anticipation leaves adjacent, moving, excluded and off-path publications unchanged");
            }
        }
        if(scene == 1) {
            ros::param::set(prefix + "static_obstacle_extra_time", 0.0);
            Check(fixture.plan->ConfigurePerceptionSafety(), "adapter accepts zero static anticipation");
            Tick(fixture, 100.2, false, 1);
            Check(fixture.Publish().path.desireSpeed < 1 && !fixture.Publish().path.safety,
                  "new D forward speed profile remains active independently of legacy static extra time");
            ros::param::set(prefix + "static_obstacle_extra_time", -1.0);
            Check(!fixture.plan->ConfigurePerceptionSafety(), "adapter rejects negative static anticipation");
        }
        Tick(fixture, 100.2, true, 0);
        const auto stopped = fixture.Publish().path;
        Check(stopped.safety && stopped.desireSpeed == 0, "early braking never clears independent safety or zero speed");
    }
}

void TestForwardSpeedPublishing(const std::string& pnc) {
    for(const std::string& gear : {"forward", "reverse"}) {
        Fixture fixture(pnc, gear);
        fixture.plan->mNavData.gpsSpeed = 1;
        fixture.plan->SetPlanningPerceptionData(Observations(100, {ObservedObject(7, 108.9, 100, 0.4)}));
        Tick(fixture, 100, false, 1);
        const auto output = fixture.Publish().path;
        Check(!output.safety, "six-metre ordinary speed restriction does not invent safety");
        Check(gear == "forward" ? output.desireSpeed < 0.5 : output.desireSpeed == 1,
              "D publishes six-metre low-speed envelope; R keeps original speed");
        Tick(fixture, 100, false, 0.2);
        Check(fixture.Publish().path.desireSpeed <= 0.2f, "new profile cannot raise an existing lower target");
        fixture.plan->mNavData.gpsSpeed = 0.3;
        ros::testTime() = 100.1;
        fixture.plan->SetPlanningPerceptionData(Observations(100.1, {ObservedObject(7, 105.9, 100, 0.4)}));
        Tick(fixture, 100.1, false, 1);
        const auto stopped = fixture.Publish().path;
        Check(!stopped.safety, "slow target moving away does not acquire a new emergency condition at three metres");
        if(gear == "forward") Check(stopped.desireSpeed == 0, "D three-metre zero target reaches final publication");
    }
    for(const std::string& cause : {"reference_unsafe", "emergency", "network", "pause"}) {
        Fixture fixture(pnc, cause);
        fixture.plan->SetPlanningPerceptionData(Observations(100, {ObservedObject(7, 108.9, 100, 0.4)}));
        Tick(fixture, 100, cause == "reference_unsafe", 1);
        const auto output = fixture.Publish().path;
        Check(output.desireSpeed == 0, "forward profile preserves other sources of zero speed");
        Check(output.safety == (cause == "reference_unsafe" || cause == "emergency"),
              "forward profile preserves each original safety bit semantics");
    }
    Fixture fixture(pnc, "forward");
    double gap = 15, speed = 1, previous_speed = 1, previous_acceleration = 0;
    bool reached_six = false, stopped = false;
    for(int tick = 0; tick < 600; ++tick) {
        const double now = 100 + tick * 0.1;
        ros::testTime() = now;
        auto can = fixture.plan->mVehicleData;
        can.vehicleSpeed = speed;
        SetVehicleFeedback(*fixture.plan, can);
        fixture.plan->SetPlanningPerceptionData(Observations(now, {ObservedObject(7, 102.9 + gap, 100)}));
        Tick(fixture, now, false, 1);
        const auto output = fixture.Publish().path;
        const double acceleration = (output.desireSpeed - previous_speed) / 0.1;
        if(!output.safety) {
            Check(acceleration >= -0.80001 && acceleration <= 0.30001 &&
                  std::abs(acceleration - previous_acceleration) <= 0.08001,
                  "published speed trajectory preserves deceleration and jerk through ordinary slowing");
        }
        if(gap <= 6) {
            reached_six = true;
            Check(speed < 0.5 && output.desireSpeed < 0.5,
                  "final publication and ideal following are below 0.5 at six metres");
        }
        if(output.safety) {
            Check(output.desireSpeed == 0 && gap >= 3 - 1e-5, "original safety still stops before three metres");
            stopped = true;
            break;
        }
        speed = output.desireSpeed;
        gap -= speed * 0.1;
        previous_speed = output.desireSpeed;
        previous_acceleration = acceleration;
    }
    Check(reached_six && stopped, "published trajectory exercises early slowing, six metres and original safety stop");
}

void TestLowSpeedStopPublishing(const std::string& pnc) {
    for(int mode = 0; mode < 3; ++mode) {
        Fixture fixture(pnc, "forward");
        const std::string prefix = "/robot/planning/perception_safety/";
        ros::param::set(prefix + "front_no_confirmation_distance", mode == 0 ? 6.0 : 0.0);
        ros::param::set(prefix + "static_stop_min_distance", mode == 2 ? 0.0 : 3.0);
        Check(fixture.plan->ConfigurePerceptionSafety(), "adapter accepts independently disabled near-stop options");
        auto object = ObservedObject(7, 105.35, 100);
        object.dx = object.dy = 0.5;
        for(auto& point : object.polygons) {
            point.x = object.x + (point.x > object.x ? 0.25 : -0.25);
            point.y = object.y + (point.y > object.y ? 0.25 : -0.25);
        }
        const double speed[] = {1, 0.94, 0.2, 0.2, 0, 0};
        for(int tick = 0; tick < 6; ++tick) {
            const double now = 100 + tick * 0.1;
            fixture.plan->mNavData.gpsSpeed = speed[tick];
            ros::testTime() = now;
            fixture.plan->SetPlanningPerceptionData(Observations(now, {object}));
            Tick(fixture, now, false, 1);
            const auto output = fixture.Publish().path;
            const bool stop = mode == 0 || (mode == 1 && tick >= 2);
            Check(output.safety == stop, "actual publication preserves static stop as CAN speed falls to zero");
            if(stop) Check(output.desireSpeed == 0, "published stopped target cannot creep toward nearby static box");
        }
    }
    for(double gap : {6.0, 6.01}) {
        Fixture fixture(pnc, "forward");
        fixture.plan->mNavData.gpsSpeed = 2;
        // ObservedObject 的纵向半长为 0.6 m。
        auto object = ObservedObject(7, 100 + 2.3 + gap + 0.6, 100);
        for(int tick = 0; tick < 3; ++tick) {
            const double now = 100 + tick * 0.1;
            ros::testTime() = now;
            fixture.plan->SetPlanningPerceptionData(Observations(now, {object}));
            Tick(fixture, now, false, 2);
            Check(fixture.Publish().path.safety == (gap == 6 || tick == 2),
                  "actual publication preserves immediate six-metre boundary and outside confirmation");
        }
    }
}

void TestSmallStationaryObstacle(const std::string& pnc) {
    for(int type = 0; type <= 4; ++type) {
        Fixture fixture(pnc, "forward");
        fixture.plan->mNavData.gpsSpeed = 1;
        auto object = ObservedObject(7, 102.65, 100);
        object.dx = object.dy = 0.5;
        object.type = type;
        for(auto& point : object.polygons) {
            point.x = object.x + (point.x > object.x ? 0.25 : -0.25);
            point.y = object.y + (point.y > object.y ? 0.25 : -0.25);
        }
        for(int tick = 0; tick < 3; ++tick) {
            const double time = 100 + tick * 0.1;
            ros::testTime() = time;
            fixture.plan->SetPlanningPerceptionData(Observations(time, {object}));
            Tick(fixture, time, false, 1);
            const auto output = fixture.Publish().path;
            if(type <= 1) {
                Check(output.safety == (type == 0 || tick == 2), "half-meter own-lane front box stops immediately");
                Check(type != 0 && tick < 2 ? std::abs(output.desireSpeed - 0.6) < 1e-6 : output.desireSpeed == 0,
                      "half-meter front box reaches published slowdown and stop commands");
            } else {
                Check(!output.safety && output.desireSpeed == 1,
                      "left-second and outside-lane half-meter boxes are excluded");
            }
        }
    }
}

void TestAccKeepsSafetyInput(const std::string& pnc) {
    Fixture fixture(pnc, "forward");
    pathPlanComply.InitParameter();
    pathPlanComply.mReferPath = fixture.plan->mReferPath;
    pathPlanComply.mPlanPath = fixture.plan->mPlanPath;
    pathPlanComply.sysTime = fixture.plan->sysTime;
    pathPlanComply.SetNavigationData(fixture.plan->mNavData);
    pathPlanComply.SetCommandState(2);
    auto can = fixture.plan->mVehicleData;
    can.vehicleSpeed = 1;
    ros::testTime() = 99.9;
    SetVehicleFeedback(pathPlanComply, can);
    ros::testTime() = 100;
    SetVehicleFeedback(pathPlanComply, can);
    auto object = ObservedObject(7, 102.65, 100);
    object.dx = object.dy = 0.5;
    for(auto& point : object.polygons) {
        point.x = object.x + (point.x > object.x ? 0.25 : -0.25);
        point.y = object.y + (point.y > object.y ? 0.25 : -0.25);
    }
    robot::path_plan_msg output;
    ros::Publisher path, sound;
    path.capture = [&](const std::type_info&, const void* raw) {
        output = *static_cast<const robot::path_plan_msg*>(raw);
    };
    // 已确认目标保留 0.5s 漏检外推，之后还需 0.3s 清除迟滞。
    for(int tick = 0; tick < 14; ++tick) {
        const double time = 100 + tick * 0.1;
        ros::testTime() = time;
        pathPlanComply.sysTime.now = time;
        pathPlanComply.AccSwitch = tick == 1 ? 0 : 1;
        robot::perception::ConstPtr message(new robot::perception(
            Observations(time, tick < 3 ? std::vector<robot::object>{object} : std::vector<robot::object>{})));
        PlanningPerceptionCallBack(message);
        Check(pathPlanComply.mPerceptionSafety.HasInput(), "ACC must not disable dedicated planning observation input");
        pathPlanComply.mReferPath.safety = false;
        pathPlanComply.mReferPath.desireSpeed = 1;
        pathPlanComply.CheckForwardReferenceSafety();
        pathPlanComply.PublishPlanPath(path, sound);
        if(tick < 3) {
            Check(output.safety && output.desireSpeed == 0, "ACC on/off preserves immediate own-lane obstacle stop");
        }
    }
    Check(!output.safety && output.desireSpeed > 0,
          "ACC on accepts fresh empty frames and does not manufacture a perception timeout");
    pathPlanComply.AccSwitch = 0;
}

void TestPerceptionAspectFilter(const std::string& pnc) {
    Fixture fixture(pnc, "forward");
    struct CASE_S {
        float dx, dy;
        bool keep;
    };
    const CASE_S cases[] = {
        {2, 7, false}, {7, 2, false}, {2, 6, true}, {6, 2, true}, {1, 6, true}, {6, 1, true}, {0.8f, 6, true}, {6, 0.8f, true}, {2, 2, true}, {2, std::nextafter(6.0f, 7.0f), false}, {std::nextafter(6.0f, 7.0f), 2, false}, {2, std::nextafter(6.0f, 5.0f), true}};
    robot::perception input = Observations(100);
    std::vector<robot::object> expected;
    for(std::size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        robot::object object = ObservedObject(i + 1, 106, 100);
        object.dx = cases[i].dx;
        object.dy = cases[i].dy;
        input.objs.push_back(object);
        if(cases[i].keep) expected.push_back(object);
    }
    const std::string original = Trace(input);
    for(int active = 0; active < 2; ++active) {
        if(active) fixture.plan->SetPlanningPerceptionData(Observations(100));
        fixture.plan->SetPerceptionData(input);
        Check(fixture.plan->mPerception.objs.size() == expected.size(),
              "perception ratio filter obeys both size gates and strict ratio boundaries in either mode");
        for(std::size_t i = 0; i < expected.size(); ++i) {
            Check(Trace(fixture.plan->mPerception.objs[i]) == Trace(expected[i]),
                  "retained perception objects preserve all fields and original order");
        }
    }
    Check(Trace(input) == original, "ratio filtering never mutates received perception message");
    robot::object elongated = input.objs.front();
    elongated.id = 99;
    fixture.plan->mPerceptionFrontScan.objs.push_back(elongated);
    fixture.plan->SetPerceptionData(Observations(100, {input.objs[0], input.objs[1]}));
    Check(fixture.plan->mPerception.objs.size() == 1 && fixture.plan->mPerception.objs.front().id == 99,
          "all filtered perception objects are removed while the independent front scanner is preserved");
    Check(fixture.plan->sysTime.msgPerception == 100, "filtered frame still refreshes original input heartbeat");
}

void TestLaneTypeFiltering(const std::string& pnc) {
    for(int active = 0; active < 2; ++active) {
        Fixture fixture(pnc, "forward");
        if(active) fixture.plan->SetPlanningPerceptionData(Observations(100));
        auto message = Observations(100);
        for(int type = 0; type <= 4; ++type) {
            auto object = ObservedObject(10 + type, 103, 100);
            object.type = type;
            message.objs.push_back(object);
        }
        const auto original = Trace(message);
        fixture.plan->SetPerceptionData(message);
        Check(fixture.plan->mPerception.objs.size() == 2, "both planning modes discard left-second and outside-lane objects at input");
        for(int i = 0; i < 2; ++i) {
            Check(Trace(fixture.plan->mPerception.objs[i]) == Trace(message.objs[i]),
                  "current lane and left-first lane retain their original fields");
        }
        Check(Trace(message) == original, "lane filtering leaves upstream message untouched");
    }
    for(int excluded_type : {2, 3, 4}) {
        Fixture fixture(pnc, "forward");
        auto inside = ObservedObject(7, 103, 100);
        fixture.plan->history_risk_vec[0].push_back(inside);
        fixture.plan->safety_check_counter_refer = 2;
        auto outside = inside;
        outside.type = excluded_type;
        fixture.plan->SetPerceptionData(Observations(100, {outside}));
        Check(fixture.plan->history_risk_vec[0].empty() && fixture.plan->safety_check_counter_refer == 0,
              "outside classification removes old compatibility risk and orphaned stop votes");
        auto retained = ObservedObject(8, 105, 100);
        retained.type = 1;
        fixture.plan->history_risk_vec[0] = {inside, retained};
        fixture.plan->SetPerceptionData(Observations(100.1, {outside, retained}));
        Check(fixture.plan->history_risk_vec[0].size() == 1 && fixture.plan->history_risk_vec[0][0].id == 8,
              "removing outside ID never removes another lane's risk history");
    }
    for(int outside = 0; outside < 2; ++outside) {
        Fixture fixture(pnc, "forward");
        fixture.plan->mNavData.xAxis = fixture.plan->mNavData.yAxis = 0;
        auto around = ObservedObject(7, 1, 0), junction = ObservedObject(8, 4, 0);
        around.type = outside ? 3 : 1;
        junction.type = outside ? 4 : 2;
        fixture.plan->SetPerceptionData(Observations(100, {around, junction}));
        for(int i = 0; i < 6; ++i) {
            const auto result = fixture.Publish();
            Check(result.path.desireSpeed > 0 && !result.path.safety,
                  "legacy around/T-zone inputs cannot add a stop during final publication");
        }
    }
}

void TestLeftLaneDirectionFiltering(const std::string& pnc) {
    for(int active = 0; active < 2; ++active) {
        Fixture fixture(pnc, "forward");
        fixture.plan->SetNavigationData(fixture.plan->mNavData);
        if(active) fixture.plan->SetPlanningPerceptionData(Observations(100));
        auto message = Observations(100.1);
        struct CASE_S {
            int type;
            double vx, vy;
            bool keep;
        };
        const CASE_S cases[] = {
            {1, -2, 0, false}, {1, -2, 1, false}, {1, 0.5, 0, true}, {1, 0, 0, true},
            {1, -0.19, 0, true}, {1, 0, 2, true}, {1, -1, 2, true}, {0, -2, 0, true}, {2, -2, 0, false}};
        std::vector<robot::object> expected;
        for(std::size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
            auto object = ObservedObject(10 + i, 103, 100, cases[i].vx, cases[i].vy);
            object.type = cases[i].type;
            object.heading = 270;  // 同向慢车也带反向旧 heading，防止误用此字段。
            message.objs.push_back(object);
            if(cases[i].keep) expected.push_back(object);
        }
        const auto original = Trace(message);
        ros::testTime() = 100.1;
        fixture.plan->SetPerceptionData(message);
        Check(fixture.plan->mPerception.objs.size() == expected.size(), "both planning modes apply the left-lane direction rule to legacy snapshot");
        for(std::size_t i = 0; i < expected.size(); ++i) {
            Check(Trace(fixture.plan->mPerception.objs[i]) == Trace(expected[i]), "retained lane and motion fields remain unchanged and ordered");
        }
        if(active) {
            fixture.plan->SetPlanningPerceptionData(message);
            Tick(fixture, 100.1);
            Check(fixture.plan->mPerceptionSafetyResult.candidates == expected.size(),
                  "raw observation adapter shares the exact legacy direction policy");
        }
        Check(Trace(message) == original, "direction filtering does not mutate the upstream message");
        auto scanner = message.objs[0];
        scanner.id = 99;
        fixture.plan->mPerceptionFrontScan.objs = {scanner};
        fixture.plan->history_risk_vec[0] = {scanner};
        fixture.plan->safety_check_counter_refer = 2;
        fixture.plan->SetPerceptionData(Observations(100.1, {message.objs[0]}));
        Check(fixture.plan->mPerception.objs.size() == 1 && fixture.plan->mPerception.objs[0].id == 99,
              "independent front scan is appended without the lane-direction filter");
        Check(fixture.plan->history_risk_vec[0].size() == 1 && fixture.plan->safety_check_counter_refer == 2,
              "discarding another ID does not erase independent scanner history or stop votes");
    }
    {
        Fixture fixture(pnc, "forward");
        fixture.plan->SetNavigationData(fixture.plan->mNavData);
        auto previous = ObservedObject(7, 103, 100);
        auto oncoming = previous;
        oncoming.type = 1;
        oncoming.vx = -2;
        for(auto& frame : fixture.plan->history_risk_vec) frame.push_back(previous);
        fixture.plan->safety_check_counter_refer = 2;
        fixture.plan->SetPerceptionData(Observations(100, {oncoming}));
        for(const auto& frame : fixture.plan->history_risk_vec) {
            Check(frame.empty(), "oncoming ID removes all compatibility history including older lane or stationary classifications");
        }
        Check(fixture.plan->safety_check_counter_refer == 0, "orphaned legacy emergency votes are reset");
        auto other = ObservedObject(8, 103, 100);
        fixture.plan->history_risk_vec[0] = {previous, other};
        fixture.plan->safety_check_counter_refer = 2;
        fixture.plan->SetPerceptionData(Observations(100.1, {oncoming, other}));
        Check(fixture.plan->history_risk_vec[0].size() == 1 && fixture.plan->history_risk_vec[0][0].id == 8 &&
                  fixture.plan->safety_check_counter_refer == 2,
              "current-lane obstacle history and votes survive adjacent oncoming removal");
    }
    {
        Fixture fixture(pnc, "forward");
        auto object = ObservedObject(7, 103, 100, -2, 0);
        object.type = 1;
        fixture.plan->mRcvGpsData = false;
        fixture.plan->SetPerceptionData(Observations(100, {object}));
        Check(fixture.plan->mPerception.objs.size() == 1, "navigation must be received before direction filtering");
        fixture.plan->SetNavigationData(fixture.plan->mNavData);
        for(int gear : {GEAR_N, GEAR_D, GEAR_R}) {
            auto can = fixture.plan->mVehicleData;
            can.curGear = gear;
            can.vehicleSpeed = 0;
            SetVehicleFeedback(*fixture.plan, can);
            fixture.plan->SetPerceptionData(Observations(100, {object}));
            Check(fixture.plan->mPerception.objs.empty() == (gear == GEAR_D), "gear determines travel direction even with zero measured ego speed");
        }
        object.vx = 2;
        fixture.plan->SetPerceptionData(Observations(100, {object}));
        Check(fixture.plan->mPerception.objs.empty(), "reverse travel uses heading plus 180 degrees");
        auto navigation = fixture.plan->mNavData;
        navigation.heading = std::numeric_limits<double>::quiet_NaN();
        fixture.plan->SetNavigationData(navigation);
        Check(!fixture.plan->BuildPerceptionLaneMotionFilter().Ignore(object), "invalid navigation heading disables exclusion");
    }
    for(int active = 0; active < 2; ++active) {
        Fixture fixture(pnc, "forward");
        Fixture clear(pnc, "forward");
        fixture.plan->SetNavigationData(fixture.plan->mNavData);
        clear.plan->SetNavigationData(clear.plan->mNavData);
        auto object = ObservedObject(7, 102.8, 100, -2, 0);
        object.type = 1;
        for(int tick = 0; tick < 8; ++tick) {
            const double now = 100 + tick * 0.1;
            ros::testTime() = now;
            auto message = Observations(now, {object});
            fixture.plan->SetPerceptionData(message);
            clear.plan->SetPerceptionData(Observations(now));
            if(active) {
                fixture.plan->SetPlanningPerceptionData(message);
                clear.plan->SetPlanningPerceptionData(Observations(now));
            }
            Tick(fixture, now);
            Tick(clear, now);
            const auto output = fixture.Publish().path;
            Check(!output.safety && output.desireSpeed > 0 && Trace(output) == Trace(clear.Publish().path),
                  "left-lane oncoming target cannot brake actual publication in either mode");
            Check(fixture.plan->sysTime.msgPerception == now, "discarding all objects still refreshes legacy heartbeat");
        }
        for(const std::string& cause : {"prior", "emergency", "pause", "network", "ultra", "gantry", "limit"}) {
            for(auto plan : {fixture.plan, clear.plan}) {
                plan->emergencyStop = cause == "emergency";
                plan->SetCommandState(cause == "pause" ? 1 : 2);
                plan->SetGantryState(cause == "gantry", false);
                SetReferenceMapLimit(*plan, cause == "limit" ? 0.2f : std::numeric_limits<float>::infinity());
            }
            ros::param::set("/robot/planning/netcheck", cause == "network" ? 1 : 0);
            ros::param::set("/ultra/status/safe", cause == "ultra" ? 1 : 0);
            Tick(fixture, 100.7, cause == "prior");
            Tick(clear, 100.7, cause == "prior");
            const auto output = fixture.Publish().path;
            Check(Trace(output) == Trace(clear.Publish().path), "oncoming exclusion keeps every output field for independent " + cause);
            if(cause == "prior" || cause == "emergency") {
                Check(output.safety, "oncoming exclusion preserves " + cause + " safety");
            }
            if(cause == "gantry") Check(!output.safety, "gantry message away from either station cannot add safety");
            if(cause != "gantry" && (active || (cause != "prior" && cause != "ultra"))) {
                Check(std::abs(output.desireSpeed - (cause == "limit" ? 0.2 : 0)) < 1e-6,
                      "oncoming exclusion preserves speed override from " + cause);
            }
        }
    }
}

void TestConfirmedEmergencyPublishing(const std::string& pnc) {
    for(double spacing : {0.1, 1.0}) {
        for(int type = 0; type <= 4; ++type) {
            Fixture fixture(pnc, "forward");
            fixture.plan->mDrivingPath.clear();
            for(int i = 0; i < 150; ++i) {
                XYZ_COOR_S point = {};
                point.x_axis = 100 + i * spacing;
                point.y_axis = 100;
                point.heading = 90;
                fixture.plan->mDrivingPath.push_back(point);
            }
            fixture.plan->mDesireSpeed = 3;
            auto object = ObservedObject(7, 103.29, 100);
            object.type = type;
            ros::Publisher reference;
            for(int tick = 0; tick < 3; ++tick) {
                const double now = 100 + tick * 0.1;
                ros::testTime() = now;
                fixture.plan->SetPlanningPerceptionData(Observations(now, {object}));
                fixture.plan->PublishReferPath(reference);
                auto result = fixture.Publish().path;
                Check(result.Path_Id == 5, "normal forward path reaches actual final publication");
                Check(result.safety == (type == 0 || (type == 1 && tick == 2)), "own-lane front risk is immediate; left-first lane still confirms");
                if(type < 2) {
                    Check(std::abs(result.desireSpeed - (type == 0 || tick == 2 ? 0 : 0.48)) < 1e-6,
                          "final published speed is 60 percent before confirmation and zero afterwards");
                } else {
                    Check(result.desireSpeed > 0.8, "outside target never limits an otherwise clear route");
                }
                for(int repeat = 0; repeat < 3; ++repeat) {
                    fixture.plan->PublishReferPath(reference);
                    Check(fixture.Publish().path.safety == (type == 0 || (type == 1 && tick == 2)),
                          "repeated full planning cycles cannot confirm a single sensor frame");
                }
            }
        }
    }
    for(const std::string& cause : {"prior", "emergency", "pause", "network", "ultra", "gantry", "limit"}) {
        Fixture fixture(pnc, "forward");
        auto pending = ObservedObject(7, 102.8, 100);
        pending.type = 1;
        fixture.plan->SetPlanningPerceptionData(Observations(100, {pending}));
        if(cause == "emergency") fixture.plan->emergencyStop = 1;
        if(cause == "pause") fixture.plan->SetCommandState(1);
        if(cause == "network") ros::param::set("/robot/planning/netcheck", 1);
        if(cause == "ultra") ros::param::set("/ultra/status/safe", 1);
        if(cause == "gantry") fixture.plan->SetGantryState(true, false);
        if(cause == "limit") SetReferenceMapLimit(*fixture.plan, 0.2f);
        Tick(fixture, 100, cause == "prior");
        const auto result = fixture.Publish().path;
        if(cause == "prior" || cause == "emergency") {
            Check(result.safety, "pending obstacle cannot clear " + cause + " safety");
        }
        if(cause == "gantry") Check(!result.safety, "distant gantry cannot stop a pending-obstacle scene");
        if(cause != "gantry") {
            Check(std::abs(result.desireSpeed - (cause == "limit" ? 0.2 : 0)) < 1e-6,
                  "pending obstacle cannot raise lower or zero speed from " + cause);
        }
    }
}

void TestGantryRouteSafetyHandover(const std::string& pnc) {
    Fixture fixture(pnc, "forward");
    auto& plan = *fixture.plan;
    SetReferenceNearGantry(plan);
    plan.SetNavigationData(plan.mNavData);
    plan.SetPlanningPerceptionData(Observations(100));
    plan.SetGantryState(true, false);
    Tick(fixture, 100);
    Check(fixture.Publish().path.safety, "eligible closed gantry still stops with clear tracked perception");

    // 在真实安全适配器中建立障碍物急停，再解除/越过闸机，急停必须继续保留。
    auto object = ObservedObject(71, plan.mNavData.xAxis + 3.0, plan.mNavData.yAxis);
    for(int frame = 1; frame <= 3; ++frame) {
        const double now = 100 + frame * 0.1;
        ros::testTime() = now;
        plan.SetPlanningPerceptionData(Observations(now, {object}));
        Tick(fixture, now);
    }
    Check(plan.mReferPath.safety, "real perception safety is established independently of gantry input");
    plan.SetGantryState(true, true);
    auto output = fixture.Publish().path;
    Check(output.safety && output.desireSpeed == 0, "opening the gantry preserves confirmed perception stop");
    plan.SetGantryState(true, false);
    plan.mNavData.xAxis = 96.14f + 0.5;
    output = fixture.Publish().path;
    Check(output.safety && output.desireSpeed == 0 && !plan.mGantryStop,
          "clearing a passed gantry cache preserves confirmed perception safety and zero speed");
}

int main(int argc, char** argv) {
    Check(argc == 2, "pnc directory supplied");
    TestEarlyStaticPublishing(argv[1]);
    TestForwardSpeedPublishing(argv[1]);
    TestLowSpeedStopPublishing(argv[1]);
    TestAccKeepsSafetyInput(argv[1]);
    TestSmallStationaryObstacle(argv[1]);
    TestPerceptionAspectFilter(argv[1]);
    TestLaneTypeFiltering(argv[1]);
    TestLeftLaneDirectionFiltering(argv[1]);
    TestConfirmedEmergencyPublishing(argv[1]);
    TestGantryRouteSafetyHandover(argv[1]);
    {
        robot::perception::ConstPtr message(new robot::perception(Observations(100)));
        PlanningPerceptionCallBack(message);
        Check(pathPlanComply.mPerceptionSafety.HasInput(), "real node callback accepts observation topic");
    }
    {
        Fixture fixture(argv[1], "forward");
        Check(fixture.plan->ConfigurePerceptionSafety(), "load startup defaults");
        ros::param::set("/robot/planning/perception_safety/empty_width", -1);
        Check(!fixture.plan->ConfigurePerceptionSafety(), "reject invalid footprint startup configuration");
    }
    {
        Fixture fixture(argv[1], "forward");
        // 旧 /perception 仍有历史鬼影，新真实帧为空，不能继续累计旧风险。
        auto legacy = Observations(100, {ObservedObject(7, 103, 100)});
        legacy.objs.front().polygons.clear();
        fixture.plan->SetPerceptionData(legacy);
        fixture.plan->SetPlanningPerceptionData(Observations(100));
        for(int tick = 0; tick < 4; ++tick) {
            const double time = 100 + tick * 0.1;
            ros::testTime() = time;
            fixture.plan->SetPlanningPerceptionData(Observations(time));
            Tick(fixture, time);
            Check(!fixture.Publish().path.safety, "legacy historical object cannot trigger new perception safety");
        }
        Tick(fixture, 100.3, true);
        Check(fixture.plan->mReferPath.safety && fixture.plan->mReferPath.desireSpeed == 0,
              "clear perception never clears an existing reference safety");
        for(bool active : {false, true}) {
            for(bool open : {false, true}) {
                Tick(fixture, 100.3, true);
                fixture.plan->SetGantryState(active, open);
                Check(fixture.Publish().path.safety, "all gantry combinations preserve other safety with new perception");
                Tick(fixture, 100.3);
                Check(!fixture.Publish().path.safety, "gantry at another location cannot override clear new perception");
            }
        }
        fixture.plan->SetGantryState(false, false);
        fixture.plan->emergencyStop = 1;
        Tick(fixture, 100.3);
        auto publication = fixture.Publish();
        Check(publication.path.safety && publication.path.desireSpeed == 0, "emergency stop wins after new perception");
        fixture.plan->emergencyStop = 0;
        fixture.plan->SetCommandState(1);
        Tick(fixture, 100.3);
        Check(fixture.Publish().path.desireSpeed == 0, "cloud pause cannot be lifted by smoothing");
        fixture.plan->SetCommandState(2);
        ros::param::set("/robot/planning/netcheck", 1);
        Tick(fixture, 100.3);
        Check(fixture.Publish().path.desireSpeed == 0, "network stop cannot be lifted by smoothing");
        ros::param::set("/robot/planning/netcheck", 0);
        ros::param::set("/ultra/status/safe", 1);
        Tick(fixture, 100.3);
        Check(fixture.Publish().path.desireSpeed == 0, "ultra zero speed preserved even when tracked corridor is clear");
        ros::param::set("/ultra/status/safe", 0);
        Tick(fixture, 101.0);
        publication = fixture.Publish();
        Check(publication.path.safety && publication.path.desireSpeed == 0, "primary stream timeout does not fall back to stale geometry");
    }
    {
        Fixture fixture(argv[1], "forward");
        fixture.plan->SetPlanningPerceptionData(Observations(100));
        jsk_recognition_msgs::BoundingBoxArray scan;
        scan.boxes.resize(1);
        scan.boxes.front().pose.position.x = 6;
        fixture.plan->SetFrontScanData(scan);
        const auto before = Trace(fixture.plan->mPerceptionFrontScan);
        const auto other = Trace(fixture.plan->mPerception);
        for(int tick = 0; tick < 3; ++tick) Tick(fixture, 100 + tick * 0.1);
        Check(fixture.plan->mReferPath.safety && fixture.Publish().path.desireSpeed == 0,
              "untracked front scanner retains independent original stop rule");
        Check(Trace(fixture.plan->mPerceptionFrontScan) == before && Trace(fixture.plan->mPerception) == other,
              "O(1) temporary source swap restores both snapshots");
        fixture.plan->safety_check_counter_refer = 0;
        Tick(fixture, 100.2, true, 0);
        Check(fixture.plan->mReferPath.safety && fixture.plan->mReferPath.desireSpeed == 0,
              "front scanner debounce and speed blend cannot clear another safety or lift task zero");
    }
    {
        Fixture fixture(argv[1], "forward");
        fixture.plan->mNavData.gpsSpeed = 2;
        for(int tick = 0; tick < 3; ++tick) {
            const double time = 100 + tick * 0.1;
            ros::testTime() = time;
            fixture.plan->SetPlanningPerceptionData(Observations(time, {ObservedObject(7, 111, 100)}));
            Tick(fixture, time);
            Check(!fixture.plan->mReferPath.safety && fixture.plan->mReferPath.desireSpeed < 2,
                  "ordinary forward obstacle has gentle limit without emergency safety");
        }
        Tick(fixture, 100.2, false, 0);
        Check(fixture.Publish().path.desireSpeed == 0, "task zero-speed cannot be raised by perception state");
    }
    {
        Fixture fixture(argv[1], "reverse");
        fixture.plan->SetPlanningPerceptionData(Observations(100, {ObservedObject(7, 100, 100)}));
        fixture.plan->UpdateReverseReferencePath(fixture.plan->mReferPath.x, fixture.plan->mReferPath.y);
        Check(!fixture.plan->mReferPath.safety, "forward observation engine does not change reverse branch");
    }
    {
        Fixture fixture(argv[1], "forward");
        fixture.plan->mNavData.gpsSpeed = 3.5;
        fixture.plan->mDesireSpeed = 3.5;
        auto side = Observations(100, {ObservedObject(7, 102, 97)});
        fixture.plan->SetPerceptionData(side);
        fixture.plan->SetPlanningPerceptionData(side);
        std::vector<float> x = fixture.plan->mReferPath.x, y = fixture.plan->mReferPath.y;
        std::vector<float> heading(x.size(), 90);
        fixture.plan->UpdateForwardReferencePath(x, y, heading);
        fixture.plan->CheckForwardReferenceSafety();
        Check(!fixture.plan->mReferPath.safety && fixture.plan->mReferPath.desireSpeed == 3.5,
              "old radial proximity heuristic cannot bypass new corridor logic at higher speed");
    }
    std::cout << "PASS perception integration: " << checks << " checks\n";
}
