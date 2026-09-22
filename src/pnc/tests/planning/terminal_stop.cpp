// 真实规划入口验证提前减速、末端到位、D 挡 60 点/R 挡原前视距离和 pathList 边界。
#define main ExistingGantryTestsMain
#include "gantry_safety.cpp"
#undef main
#include <cmath>
#include <fstream>

void SetDrivingRoute(PathPlanComply &plan, int count = 400, bool reverse = false) {
    plan.mDrivingPath.clear();
    for(int i = 0; i < count; ++i) {
        XYZ_COOR_S point{};
        point.x_axis = 100.0 + (reverse ? -1.0 : 1.0) * i * 0.25;
        point.y_axis = 100.0;
        point.heading = 90.0;
        point.dist_origin = i * 0.25;
        point.p2pDistance = i == 0 ? 0.0 : 0.25;
        plan.mDrivingPath.push_back(point);
    }
    plan.mKeypoint = 0;
    plan.mStopIndex = count - 1;
    plan.mTaskPlanData.task_id = 12;
    plan.go_task_id_ = 99;
    plan.back_task_id_ = 100;
    plan.mTaskPlanData.taskType = TRACKPATH;
    plan.mTaskPlanData.desireSpeed = 15.0 / 3.6;
    plan.mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
    plan.mPathPlanStatus.stopX = plan.mDrivingPath.back().x_axis;
    plan.mPathPlanStatus.stopY = 100.0;
    plan.mVehicleData.curGear = reverse ? GEAR_R : GEAR_D;
    plan.mTaskPlanData.desireGear = plan.mVehicleData.curGear;
    plan.mNavData.gpsSpeed = plan.mTaskPlanData.desireSpeed;
    plan.mDesireSpeed = plan.mTaskPlanData.desireSpeed;
}

double StopSpeedAt(PathPlanComply &plan, double distance) {
    plan.mKeypoint = 0;
    plan.mDrivingPath[plan.mStopIndex].dist_origin = distance;
    plan.mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
    return std::min<double>(plan.LimitSpeedByDistanceToStop(100, 100, 90, 200, 100, 90),
                            plan.mTaskPlanData.desireSpeed);
}

void TestStopProfile(const std::string &pnc) {
    Fixture fixture(pnc, "forward");
    auto &plan = *fixture.plan;
    SetDrivingRoute(plan);
    const double cruise = plan.mTaskPlanData.desireSpeed;
    Check(std::abs(StopSpeedAt(plan, 80) - cruise) < 1e-6, "far from endpoint retains task speed");
    Check(StopSpeedAt(plan, 61) == cruise && StopSpeedAt(plan, 60) < cruise,
          "15 km/h deceleration starts around 61 m");
    Check(StopSpeedAt(plan, 25) < cruise - 0.8, "deceleration already active well before old 10 m boundary");
    const double before = StopSpeedAt(plan, 10.001);
    const double after = StopSpeedAt(plan, 9.999);
    Check(before >= after && before - after < 0.001, "no speed-command cliff at 10 m");
    Check(StopSpeedAt(plan, 2) < 0.5, "final approach is below 0.5 m/s");
    for(double distance = 0.5; distance <= 10.0; distance += 0.25) {
        Check(StopSpeedAt(plan, distance) <= std::max(distance / 6.0, 0.3) + 1e-6,
              "final approach never exceeds old close-range speed limit");
    }
    plan.mTaskPlanData.desireSpeed = 1.0;
    Check(StopSpeedAt(plan, 6.4) < 1.0, "low-speed task also decelerates earlier than old 6 m onset");
    plan.mTaskPlanData.desireSpeed = cruise;
    Check(std::abs(StopSpeedAt(plan, 0.5) - 0.3) < 1e-6 &&
              plan.mPathPlanStatus.taskExecuStatus == TASKINPROGRESS,
          "creep reaches existing strict 0.5 m completion boundary");
    Check(StopSpeedAt(plan, 0.49) == 0 && plan.mPathPlanStatus.taskExecuStatus == TASKFINISHED,
          "endpoint arrival still completes task and commands zero");
    Check(StopSpeedAt(plan, -0.1) == 0, "overshot endpoint cannot command reverse or acceleration");
    plan.mTaskPlanData.desireSpeed = 0.2;
    Check(std::abs(StopSpeedAt(plan, 5) - 0.2) < 1e-6, "creep minimum cannot raise lower task speed");
    plan.mTaskPlanData.desireSpeed = cruise;

    for(bool loaded : {false, true}) {
        plan.mVehicleData.linkPallet = loaded;
        double distance = 80.0;
        double previous = cruise;
        int tick = 0;
        // 用 10 Hz 理想速度跟随积分检查整段指令；这里只验证规划，不模拟车辆动力学。
        for(; distance >= 0.5 && tick < 2000; ++tick) {
            const double speed = StopSpeedAt(plan, distance);
            Check(std::isfinite(speed) && speed > 0 && speed <= previous + 1e-6,
                  "approach speed remains finite, positive and monotonic");
            Check((previous - speed) / 0.1 <= 0.201, "approach command deceleration bounded by 0.2 m/s2");
            previous = speed;
            distance -= speed * 0.1;
        }
        Check(tick < 2000 && previous <= 0.301 && StopSpeedAt(plan, distance) == 0,
              "both load states reach endpoint at creep speed and finish");
    }

    SetDrivingRoute(plan);
    plan.go_task_id_ = 12;
    AirCraftParkingPort port{};
    port.id = 1;
    port.parking = 1;
    port.go_stop_index = 80;
    plan.aircraft_parking_ports_ = {port};
    Check(plan.LimitDrivingTaskSpeed() < 3.0, "occupied airport stop line receives early slowdown");
    plan.mKeypoint = 79;
    Check(plan.LimitDrivingTaskSpeed() == 0 && plan.mPathPlanStatus.taskExecuStatus == TASKINPROGRESS,
          "temporary stop does not finish whole task");
    plan.aircraft_parking_ports_[0].parking = 0;
    Check(plan.LimitDrivingTaskSpeed() > 3.0, "airport clearance releases temporary stop");
    plan.mVehicleData.curGear = GEAR_N;
    Check(plan.LimitDrivingTaskSpeed() == 0, "neutral gear remains stopped");
    plan.mStopIndex = -1;
    Check(plan.LimitSpeedByDistanceToStop(0, 0, 0, 0, 0, 0) == 0, "invalid stop index remains stopped");
}

void TestReferenceHorizon(const std::string &pnc) {
    for(bool reverse : {false, true}) {
        Fixture fixture(pnc, reverse ? "reverse" : "forward");
        auto &plan = *fixture.plan;
        SetDrivingRoute(plan, 400, reverse);
        for(int key : {0, 100, 350, 390, 397, 399}) {
            plan.mKeypoint = key;
            std::vector<float> x, y, heading;
            plan.BuildReferencePathPoints(x, y, heading);
            // 0.25 m 等距路线：D 的 60 点为 0,3,5,...,119；R 保留原里程截断。
            const double expected = std::min(reverse ? 9.75 * 1.5 : 29.75, (399 - key) * 0.25);
            const double direction = reverse ? -1 : 1;
            const double ahead = direction * (x.back() - plan.mDrivingPath[key].x_axis);
            Check(std::abs(ahead - expected) < 1e-5, "D extends to 60 samples while R retains its original horizon");
            if(!reverse) {
                Check(x.size() <= 60, "D reference never exceeds the point budget");
                if(key <= 100) Check(x.size() == 60, "long D route publishes exactly 60 points without densifying");
            }
            Check(x.size() >= 10 && x.size() == y.size() && x.size() == heading.size(),
                  "terminal reference retains enough actual route points for control");
            for(std::size_t i = 0; i < x.size(); ++i) {
                const double distance = direction * (x[i] - 100.0);
                Check(distance >= 0 && distance <= 399 * 0.25 && y[i] == 100 && heading[i] == 90,
                      "all coordinates and reverse headings stay on original route");
                if(i > 0) Check(direction * (x[i] - x[i - 1]) > 0, "points remain in route order");
            }
        }
        plan.mKeypoint = -1;
        std::vector<float> x, y, heading;
        plan.BuildReferencePathPoints(x, y, heading);
        Check(x.empty() && y.empty(), "invalid nearest point does not index route");
        if(!reverse) {
            plan.mKeypoint = 0;
            plan.PathPlanProcess();
            ros::Publisher reference_pub;
            plan.PublishReferPath(reference_pub);
            const auto output = fixture.Publish().path;
            Check(plan.mReferPath.x.size() == 60 && output.x == plan.mReferPath.x &&
                      output.y == plan.mReferPath.y,
                  "all 60 D points reach both reference and final published messages");
            Check(!output.safety && output.desireSpeed == plan.mTaskPlanData.desireSpeed &&
                      std::abs(plan.mPathPlanStatus.distance2Stop - 99.75) < 1e-5,
                  "longer local reference preserves task speed, safety and distance to actual stop");
        }
    }

    Fixture fixture(pnc, "forward");
    auto &plan = *fixture.plan;
    SetDrivingRoute(plan);
    std::vector<double> arc(plan.mDrivingPath.size(), 0.0);
    double angle = 0.0;
    for(std::size_t i = 0; i < plan.mDrivingPath.size(); ++i) {
        angle += 0.007 + (i % 5) * 0.003;
        plan.mDrivingPath[i].x_axis = 50 * std::cos(angle);
        plan.mDrivingPath[i].y_axis = 50 * std::sin(angle);
        if(i > 0) arc[i] = arc[i - 1] + hypot(plan.mDrivingPath[i].x_axis - plan.mDrivingPath[i - 1].x_axis,
                                             plan.mDrivingPath[i].y_axis - plan.mDrivingPath[i - 1].y_axis);
    }
    std::vector<float> x, y, heading;
    plan.BuildReferencePathPoints(x, y, heading);
    Check(x.size() == 60 && x.back() == plan.mDrivingPath[117].x_axis &&
              y.back() == plan.mDrivingPath[117].y_axis,
          "nonuniform curved D route retains real samples and stops at the sixtieth point");
    // 非 D 挡保留原里程计算及线段内插值，不能因 D 点数增加改变倒车轨迹。
    plan.mVehicleData.curGear = GEAR_R;
    x.clear();
    y.clear();
    heading.clear();
    plan.BuildReferencePathPoints(x, y, heading);
    const double target = arc[37] * 1.5;  // 每段大于 0.3 m，原第 20 个采样点为索引 37。
    std::size_t end = 1;
    while(arc[end] < target) ++end;
    const double from_start = hypot(x.back() - plan.mDrivingPath[end - 1].x_axis,
                                    y.back() - plan.mDrivingPath[end - 1].y_axis);
    const double to_end = hypot(x.back() - plan.mDrivingPath[end].x_axis,
                                y.back() - plan.mDrivingPath[end].y_axis);
    Check(std::abs(arc[end - 1] + from_start - target) < 1e-4,
          "R curved nonuniform route still scales actual arc distance");
    Check(std::abs(from_start + to_end - (arc[end] - arc[end - 1])) < 1e-4,
          "interpolated endpoint lies on existing route segment");
}

void TestFinalPublication(const std::string &pnc) {
    for(const std::string &cause : {"forward", "emergency", "network", "pause", "gantry", "gantry_far", "map_limit"}) {
        Fixture fixture(pnc, cause);
        auto &plan = *fixture.plan;
        SetDrivingRoute(plan);
        if(cause == "gantry") {
            // 真实生成的末端参考线也需满足新位置条件；终点落在进货站闸机点。
            const double offset = 96.14f - plan.mDrivingPath.back().x_axis;
            for(auto &point : plan.mDrivingPath) {
                point.x_axis += offset;
                point.y_axis = -405.63f;
            }
            plan.mPathPlanStatus.stopX = plan.mDrivingPath.back().x_axis;
            plan.mPathPlanStatus.stopY = -405.63f;
            plan.mNavData.yAxis = -405.63f;
        }
        plan.mNavData.xAxis = plan.mDrivingPath[396].x_axis;
        plan.mNavData.gpsSpeed = 0.3;
        if(cause == "gantry" || cause == "gantry_far") plan.SetGantryState(true, false);
        if(cause == "map_limit")
            for(auto &point : plan.mDrivingPath) point.map_speed_limit = 0.1f;
        plan.PathPlanProcess();
        ros::Publisher refer;
        plan.PublishReferPath(refer);
        const auto result = fixture.Publish();
        Check(result.path.x.size() >= 10 && result.path.x.back() <= plan.mDrivingPath.back().x_axis,
              cause + ": final approach publishes an in-route trackable path");
        if(cause == "emergency" || cause == "network" || cause == "pause") {
            Check(result.path.desireSpeed == 0, cause + ": independent stop is immediate");
        } else {
            Check(result.path.desireSpeed > 0 && result.path.desireSpeed <= 0.301,
                  cause + ": short forward remainder does not force premature braking");
        }
        if(cause == "emergency" || cause == "gantry")
            Check(result.path.safety, cause + ": safety flag remains asserted");
        if(cause == "gantry_far")
            Check(!result.path.safety && !plan.mGantryStop, "unrelated terminal route clears a remote gantry input");
        if(cause == "map_limit")
            Check(result.path.desireSpeed <= 0.101, "lower map limit wins over endpoint profile");
    }

    Fixture fixture(pnc, "forward");
    auto &plan = *fixture.plan;
    SetDrivingRoute(plan);
    plan.mStopIndex = 40;  // 终点还有 10 m，当前车速显著高于终点限速。
    robot::object obstacle;
    obstacle.x = 109;
    obstacle.y = 100;
    obstacle.dx = obstacle.dy = 0.6;
    obstacle.heading = 90;
    plan.mPerception.objs = {obstacle};
    plan.PathPlanProcess();
    const double stop_limit = plan.mDesireSpeed;
    ros::Publisher refer;
    plan.PublishReferPath(refer);
    Check(plan.mReferPath.desireSpeed <= stop_limit && fixture.Publish().path.desireSpeed <= stop_limit,
          "legacy collision speed blending cannot undo endpoint slowdown");

    plan.mNavData.xAxis = plan.mDrivingPath[40].x_axis;
    plan.PathPlanProcess();
    plan.PublishReferPath(refer);
    const auto arrived = fixture.Publish();
    Check(plan.mPathPlanStatus.taskExecuStatus == TASKFINISHED && arrived.path.desireSpeed == 0,
          "full planning/publication chain stops and finishes at endpoint");
    for(int points : {0, 4, 5, 9}) {
        plan.mReferPath.x.assign(points, 100);
        plan.mReferPath.y.assign(points, 100);
        plan.mReferPath.desireSpeed = 2.0;
        const auto stopped = fixture.Publish();
        Check(stopped.path.x.empty() && stopped.path.desireSpeed == 0,
              "path rejected by controller cannot be published with nonzero speed");
    }
}

void TestTerminalBoundaryReview(const std::string &pnc) {
    for(int count : {0, 1, 9, 10, 15, 19}) {
        Fixture fixture(pnc, "forward");
        auto &plan = *fixture.plan;
        SetDrivingRoute(plan);
        ros::Publisher reference;
        plan.PathPlanProcess();
        plan.PublishReferPath(reference);
        Check(fixture.Publish().path.desireSpeed > 0, "previous task has a live moving reference");
        plan.mDrivingPath.resize(count);
        plan.mStopIndex = count - 1;
        for(int tick = 0; tick < 3; ++tick) {
            plan.PathPlanProcess();
            plan.PublishReferPath(reference);
            const auto output = fixture.Publish().path;
            Check(output.x.empty() && output.y.empty() && output.desireSpeed == 0,
                  "empty or undersized replacement route cannot republish old moving trajectory");
            Check(plan.mReferPath.x.empty() && plan.mReferPath.desireSpeed == 0 && plan.mReferPath.planspeed == 0,
                  "invalid route clears the cached reference as well as the final publication");
        }
        SetDrivingRoute(plan);
        plan.PathPlanProcess();
        plan.PublishReferPath(reference);
        Check(fixture.Publish().path.desireSpeed > 0, "valid replacement route resumes after short-route stop");
    }
    for(double spacing : {0.25, 0.5, 1.0, 2.0}) {
        Fixture fixture(pnc, "forward");
        auto &plan = *fixture.plan;
        SetDrivingRoute(plan, 60);
        for(int i = 0; i < 60; ++i) {
            plan.mDrivingPath[i].x_axis = 100 + i * spacing;
            plan.mDrivingPath[i].dist_origin = i * spacing;
        }
        plan.mKeypoint = 58;
        plan.mNavData.xAxis = plan.mDrivingPath.back().x_axis;
        plan.mPathPlanStatus.stopX = plan.mNavData.xAxis;
        plan.mNavData.gpsSpeed = 0.3;
        plan.PathPlanProcess();
        ros::Publisher reference;
        plan.PublishReferPath(reference);
        const auto output = fixture.Publish().path;
        Check(plan.mPathPlanStatus.taskExecuStatus == TASKFINISHED && plan.mPathPlanStatus.distance2Stop == 0,
              "real endpoint is matched even without a duplicate CSV tail and with sparse point spacing");
        Check(output.x.empty() && output.desireSpeed == 0, "arrival at sparse endpoint never continues creeping");
    }
}

void TestJoinedPathDistance(const std::string &pnc, const std::string &fixtures) {
    for(int part = 0; part < 2; ++part) {
        std::ofstream csv(fixtures + "/terminal_" + std::to_string(part) + ".csv");
        for(int i = 0; i <= 80; ++i) csv << part * 20 + i * 0.25 << ",100,90,10\n";
    }
    Fixture fixture(pnc, "forward");
    auto &plan = *fixture.plan;
    SetDrivingRoute(plan);
    ros::param::set("path_dir", fixtures + "/");
    robot::task_plan_msg task;
    task.pathList = {"terminal_0", "terminal_1"};
    plan.LoadTaskPaths(task);
    for(std::size_t i = 0; i < plan.mDrivingPath.size(); ++i) {
        Check(std::abs(plan.mDrivingPath[i].dist_origin - plan.mDrivingPath[i].x_axis) < 1e-5,
              "joined route uses cumulative distance across CSV boundary and duplicate endpoints");
        if(plan.mDrivingPath[i].x_axis == 5) plan.mKeypoint = i;
    }
    plan.mStopIndex = plan.mDrivingPath.size() - 1;
    const double speed = plan.LimitSpeedByDistanceToStop(5, 100, 90, 40, 100, 90);
    Check(std::abs(plan.mPathPlanStatus.distance2Stop - 35) < 1e-5 && speed > 2.9 && speed < 3.1,
          "remaining distance includes following pathList segment");
}

void WriteTerminalControlInputs(const std::string &pnc, const std::string &fixtures) {
    // 两节点各自定义了不同布局的 Pose2d，分别运行真实规划/控制可执行，按消息字段交接。
    std::ofstream output(fixtures + "/terminal_control_inputs.txt");
    output.precision(17);
    for(bool reverse : {false, true}) {
        for(int key : {360, 380, 392, 396, 397}) {
            Fixture fixture(pnc, reverse ? "reverse" : "forward");
            auto &plan = *fixture.plan;
            SetDrivingRoute(plan, 400, reverse);
            if(reverse) plan.mTaskPlanData.taskType = ADAPTIVEPARK;
            plan.mNavData.xAxis = plan.mDrivingPath[key].x_axis;
            plan.mNavData.yAxis = 100.05;
            plan.mNavData.gpsSpeed = 0.3;
            plan.mNavData.gpsSpeed = 0.3;
            plan.PathPlanProcess();
            ros::Publisher refer;
            plan.PublishReferPath(refer);
            const auto message = fixture.Publish().path;
            output << +plan.mVehicleData.curGear << ' ' << plan.mNavData.gpsSpeed << ' '
                   << plan.mNavData.xAxis << ' ' << plan.mNavData.yAxis << ' ' << plan.mNavData.heading << ' '
                   << +plan.mPathPlanStatus.taskExecuStatus << ' ' << message.desireSpeed << ' '
                   << +plan.mTaskPlanData.taskType << ' ' << +plan.mTaskPlanData.desireGear << ' '
                   << plan.mPathPlanStatus.distance2Stop << ' '
                   << +message.Path_Id << ' ' << message.safety << ' ' << message.x.size() << '\n';
            for(std::size_t i = 0; i < message.x.size(); ++i)
                output << message.x[i] << ' ' << message.y[i] << '\n';
        }
    }
    // 末端弯道必须保留倒车转向；只验证非 NaN 会漏掉少于 10 个前方点时突然回正。
    for(int side : {-1, 1}) {
        for(int key : {188, 190, 194}) {
            Fixture fixture(pnc, "reverse");
            auto &plan = *fixture.plan;
            SetDrivingRoute(plan, 201, true);
            plan.mTaskPlanData.taskType = ADAPTIVEPARK;
            for(int i = 0; i < 201; ++i) {
                const double theta = i * 0.05 / 10;
                auto &point = plan.mDrivingPath[i];
                point.x_axis = 100 - 10 * std::sin(theta);
                point.y_axis = 100 + side * 10 * (1 - std::cos(theta));
                point.heading = 90 + side * theta * 180 / M_PI;
                point.dist_origin = i * 0.05;
            }
            plan.mKeypoint = key;
            plan.mNavData.xAxis = plan.mDrivingPath[key].x_axis;
            plan.mNavData.yAxis = plan.mDrivingPath[key].y_axis;
            plan.mNavData.heading = plan.mDrivingPath[key].heading;
            plan.mNavData.gpsSpeed = 0.3;
            plan.mNavData.gpsSpeed = 0.3;
            plan.mPathPlanStatus.stopX = plan.mDrivingPath.back().x_axis;
            plan.mPathPlanStatus.stopY = plan.mDrivingPath.back().y_axis;
            plan.PathPlanProcess();
            ros::Publisher reference;
            plan.PublishReferPath(reference);
            const auto message = fixture.Publish().path;
            output << +plan.mVehicleData.curGear << ' ' << plan.mNavData.gpsSpeed << ' '
                   << plan.mNavData.xAxis << ' ' << plan.mNavData.yAxis << ' ' << plan.mNavData.heading << ' '
                   << +plan.mPathPlanStatus.taskExecuStatus << ' ' << message.desireSpeed << ' '
                   << +plan.mTaskPlanData.taskType << ' ' << +plan.mTaskPlanData.desireGear << ' '
                   << plan.mPathPlanStatus.distance2Stop << ' '
                   << +message.Path_Id << ' ' << message.safety << ' ' << message.x.size() << '\n';
            for(std::size_t i = 0; i < message.x.size(); ++i)
                output << message.x[i] << ' ' << message.y[i] << '\n';
        }
    }
    Fixture fixture(pnc, "forward");
    auto& plan = *fixture.plan;
    SetDrivingRoute(plan);
    plan.mNavData.xAxis = plan.mDrivingPath.back().x_axis;
    plan.mNavData.gpsSpeed = plan.mNavData.gpsSpeed = 0.3;
    const auto task = plan.mTaskPlanData;  // 控制订阅上游任务，规划内部到位清理不会改写该消息。
    plan.PathPlanProcess();
    ros::Publisher reference;
    plan.PublishReferPath(reference);
    const auto message = fixture.Publish().path;
    Check(message.x.empty() && message.desireSpeed == 0 &&
          plan.mPathPlanStatus.taskExecuStatus == TASKFINISHED,
          "ordinary endpoint produces the real finished-task zero-speed message");
    output << +plan.mVehicleData.curGear << ' ' << plan.mNavData.gpsSpeed << ' '
           << plan.mNavData.xAxis << ' ' << plan.mNavData.yAxis << ' ' << plan.mNavData.heading << ' '
           << +plan.mPathPlanStatus.taskExecuStatus << ' ' << message.desireSpeed << ' '
           << +task.taskType << ' ' << +task.desireGear << ' '
           << plan.mPathPlanStatus.distance2Stop << ' '
           << +message.Path_Id << ' ' << message.safety << ' ' << message.x.size() << '\n';
}

int main(int argc, char **argv) {
    Check(argc == 3, "pnc and temporary fixture directories supplied");
    TestStopProfile(argv[1]);
    TestReferenceHorizon(argv[1]);
    TestFinalPublication(argv[1]);
    TestTerminalBoundaryReview(argv[1]);
    TestJoinedPathDistance(argv[1], argv[2]);
    WriteTerminalControlInputs(argv[1], argv[2]);
    std::cout << "PASS terminal stop: " << checks << " checks\n";
    return 0;
}
