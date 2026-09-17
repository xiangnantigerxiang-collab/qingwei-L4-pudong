// 复用既有实际发布捕获夹具，新增链必须服从原有独立安全来源。
#define main ExistingGantryTestsMain
#include "gantry_safety.cpp"
#undef main

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
        Check(fixture.plan->mPerception.objs.size() == 3, "both planning modes discard outside-lane objects at input");
        for(int i = 0; i < 3; ++i) {
            Check(Trace(fixture.plan->mPerception.objs[i]) == Trace(message.objs[i]),
                  "current lane and both left lanes retain their original fields");
        }
        Check(Trace(message) == original, "lane filtering leaves upstream message untouched");
    }
    {
        Fixture fixture(pnc, "forward");
        auto inside = ObservedObject(7, 103, 100);
        fixture.plan->history_risk_vec[0].push_back(inside);
        fixture.plan->safety_check_counter_refer = 2;
        auto outside = inside;
        outside.type = 4;
        fixture.plan->SetPerceptionData(Observations(100, {outside}));
        Check(fixture.plan->history_risk_vec[0].empty() && fixture.plan->safety_check_counter_refer == 0,
              "outside classification removes old compatibility risk and orphaned stop votes");
        auto retained = ObservedObject(8, 105, 100);
        retained.type = 2;
        fixture.plan->history_risk_vec[0] = {inside, retained};
        fixture.plan->SetPerceptionData(Observations(100.1, {outside, retained}));
        Check(fixture.plan->history_risk_vec[0].size() == 1 && fixture.plan->history_risk_vec[0][0].id == 8,
              "removing outside ID never removes another lane's risk history");
    }
    for(int outside = 0; outside < 2; ++outside) {
        Fixture fixture(pnc, "forward");
        fixture.plan->mNavData.xAxis = fixture.plan->mNavData.yAxis = 0;
        fixture.plan->history_unsafe.assign(30, false);
        fixture.plan->unsafe_vec.assign(4, false);
        auto around = ObservedObject(7, 1, 0), junction = ObservedObject(8, 4, 0);
        around.type = outside ? 3 : 1;
        junction.type = outside ? 4 : 2;
        fixture.plan->SetPerceptionData(Observations(100, {around, junction}));
        bool unsafe = false;
        for(int i = 0; i < 6; ++i) unsafe = fixture.plan->checkAroundObstacle();
        Check(unsafe == !outside, "startup observes left lanes and ignores both outside types");
        Check(fixture.plan->checkTJunctionObstacle() == !outside,
              "T junction observes left lanes and ignores both outside types");
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
                Check(result.safety == (type < 3 && tick == 2), "only three fresh inside/left-lane risk frames publish safety");
                if(type < 3) {
                    Check(std::abs(result.desireSpeed - (tick == 2 ? 0 : 0.48)) < 1e-6,
                          "final published speed is 60 percent before confirmation and zero afterwards");
                } else {
                    Check(result.desireSpeed > 0.8, "outside target never limits an otherwise clear route");
                }
                for(int repeat = 0; repeat < 3; ++repeat) {
                    fixture.plan->PublishReferPath(reference);
                    Check(fixture.Publish().path.safety == (type < 3 && tick == 2),
                          "repeated full planning cycles cannot confirm a single sensor frame");
                }
            }
        }
    }
    for(const std::string& cause : {"prior", "emergency", "pause", "network", "ultra", "gantry", "limit"}) {
        Fixture fixture(pnc, "forward");
        fixture.plan->SetPlanningPerceptionData(Observations(100, {ObservedObject(7, 102.8, 100)}));
        if(cause == "emergency") fixture.plan->emergencyStop = 1;
        if(cause == "pause") fixture.plan->SetCommandState(1);
        if(cause == "network") ros::param::set("/robot/planning/netcheck", 1);
        if(cause == "ultra") ros::param::set("/ultra/status/safe", 1);
        if(cause == "gantry") fixture.plan->SetGantryState(true, false);
        if(cause == "limit") fixture.plan->mPointSpeedLimits.push_back({100, 100, 0.2});
        Tick(fixture, 100, cause == "prior");
        const auto result = fixture.Publish().path;
        if(cause == "prior" || cause == "emergency" || cause == "gantry") {
            Check(result.safety, "pending obstacle cannot clear " + cause + " safety");
        }
        if(cause != "gantry") {
            Check(std::abs(result.desireSpeed - (cause == "limit" ? 0.2 : 0)) < 1e-6,
                  "pending obstacle cannot raise lower or zero speed from " + cause);
        }
    }
}

int main(int argc, char** argv) {
    Check(argc == 2, "pnc directory supplied");
    TestPerceptionAspectFilter(argv[1]);
    TestLaneTypeFiltering(argv[1]);
    TestConfirmedEmergencyPublishing(argv[1]);
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
                Check(fixture.Publish().path.safety == (active && !open), "gantry independently overrides clear new perception");
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
        fixture.plan->mVehicleData.vehicleSpeed = 2;
        for(int tick = 0; tick < 3; ++tick) {
            const double time = 100 + tick * 0.1;
            ros::testTime() = time;
            fixture.plan->SetPlanningPerceptionData(Observations(time, {ObservedObject(7, 108, 100)}));
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
        fixture.plan->mVehicleData.vehicleSpeed = 3.5;
        fixture.plan->mDesireSpeed = 3.5;
        auto side = Observations(100, {ObservedObject(7, 102, 97)});
        fixture.plan->SetPerceptionData(side);
        fixture.plan->SetPlanningPerceptionData(side);
        std::vector<float> x = fixture.plan->mReferPath.x, y = fixture.plan->mReferPath.y;
        std::vector<float> heading(x.size(), 90);
        fixture.plan->UpdateForwardReferencePath(x, y, heading, 0);
        fixture.plan->CheckForwardReferenceSafety();
        Check(!fixture.plan->mReferPath.safety && fixture.plan->mReferPath.desireSpeed == 3.5,
              "old radial proximity heuristic cannot bypass new corridor logic at higher speed");
    }
    std::cout << "PASS perception integration: " << checks << " checks\n";
}
