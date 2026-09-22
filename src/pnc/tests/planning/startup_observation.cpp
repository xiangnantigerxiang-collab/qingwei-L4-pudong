// 独立 C++11 核心测试：模式生命周期、实测车速、时效及完整车身的 2 m 欧氏距离。
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <cstring>
#include "../../src/robot_path_plan/safety/startup_observation.inc"

using planning_perception::CONFIG_S;
using planning_perception::StartupObservation;
int checks = 0;

void Check(bool condition, const char* message) {
    if(!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
    ++checks;
}

robot::object Box(double x, double y, double length = 1, double width = 1, double yaw = 0) {
    robot::object object;
    object.x = x;
    object.y = y;
    object.dx = length;
    object.dy = width;
    object.polygons.resize(4);
    const double u[] = {-0.5, 0.5, 0.5, -0.5}, v[] = {-0.5, -0.5, 0.5, 0.5};
    for(int i = 0; i < 4; ++i) {
        object.polygons[i].x = x + u[i] * length * std::cos(yaw) - v[i] * width * std::sin(yaw);
        object.polygons[i].y = y + u[i] * length * std::sin(yaw) + v[i] * width * std::cos(yaw);
    }
    return object;
}

robot::perception Frame(double time, const std::vector<robot::object>& objects = {}) {
    robot::perception frame;
    frame.header.stamp = ros::Time(time);
    frame.objs = objects;
    return frame;
}

CONFIG_S Config() {
    CONFIG_S config;
    config.empty_length = 4;
    config.empty_width = 2;
    config.empty_front = 3;
    config.loaded_length = 8;
    config.loaded_width = 4;
    config.loaded_front = 4;
    return config;
}

void SetVehicleFeedback(StartupObservation& observation, bool automatic, double speed,
                        bool loaded, double now, bool forward = true) {
    observation.SetVehicle(automatic, loaded, now, forward);
    observation.SetSpeed(speed, now);
}

void Arm(StartupObservation& observation, double now = 100, bool loaded = false) {
    SetVehicleFeedback(observation, false, 0, loaded, now - 0.1);
    observation.SetPose(0, 0, 90, now);
    SetVehicleFeedback(observation, true, 0, loaded, now);
}

void TestLifecycle() {
    const CONFIG_S config = Config();
    StartupObservation observation;
    Check(!observation.MustStop(config, 100), "manual mode adds no stop");
    Arm(observation);
    Check(observation.MustStop(config, 100), "auto waits for an actual new frame");
    observation.Observe(Frame(99.99), 100, config.input_timeout);
    Check(observation.MustStop(config, 100), "pre-switch empty frame cannot release startup");
    observation.Observe(Frame(100), 100, config.input_timeout);
    Check(!observation.MustStop(config, 100), "fresh empty frame permits immediate startup without timer");
    observation.Observe(Frame(100.1, {Box(0, 3)}), 100.1, config.input_timeout);
    Check(observation.MustStop(config, 100.1), "new nearby obstacle reblocks before successful motion");
    for(int i = 0; i < 50; ++i) Check(observation.MustStop(config, 100.1), "planning polls cannot count as navigation motion");
    SetVehicleFeedback(observation, true, 0.3f, false, 100.1);
    Check(observation.MustStop(config, 100.1), "one moving sample does not finish startup");
    SetVehicleFeedback(observation, true, 0.2f, false, 100.15);
    SetVehicleFeedback(observation, true, 0.3f, false, 100.2);
    Check(observation.MustStop(config, 100.2), "exact threshold interrupts consecutive samples");
    SetVehicleFeedback(observation, true, 0.3f, false, 100.2);
    Check(observation.MustStop(config, 100.2), "same-time repeated navigation sample cannot finish");
    SetVehicleFeedback(observation, true, 0.3f, false, 100.25);
    Check(!observation.MustStop(config, 100.25), "two consecutive samples above 0.2 finish observation");
    SetVehicleFeedback(observation, true, 0, false, 200);
    observation.Observe(Frame(200, {Box(0, 0)}), 200, config.input_timeout);
    Check(!observation.MustStop(config, 200), "automatic stop does not rearm and stale input no longer blocks");
    Arm(observation, 201);
    Check(observation.MustStop(config, 201), "manual then auto rearms and discards previous clearance");
    SetVehicleFeedback(observation, true, -0.3f, false, 201.1);
    SetVehicleFeedback(observation, true, -0.3f, false, 201.2);
    Check(!observation.MustStop(config, 201.2), "reverse motion also completes startup");
    Arm(observation, 202);
    SetVehicleFeedback(observation, true, 0.3f, false, 202.1);
    SetVehicleFeedback(observation, true, std::numeric_limits<double>::quiet_NaN(), false, 202.2);
    SetVehicleFeedback(observation, true, 0.3f, false, 202.3);
    Check(observation.MustStop(config, 202.3), "invalid speed interrupts confirmation");
    SetVehicleFeedback(observation, true, 0.3f, false, 203);
    Check(observation.MustStop(config, 203), "widely separated moving samples do not accumulate");
    SetVehicleFeedback(observation, false, 0, false, 203.1);
    Check(!observation.MustStop(config, 203.1), "manual immediately cancels this stop source");
    StartupObservation first_auto;
    SetVehicleFeedback(first_auto, true, 0, false, 1);
    Check(first_auto.MustStop(config, 1), "node first seeing stationary auto is guarded");
}

void TestFreshnessAndBadFrames() {
    const CONFIG_S config = Config();
    StartupObservation observation;
    Arm(observation);
    observation.Observe(Frame(100, {Box(0, 0)}), 100, config.input_timeout);
    observation.Observe(Frame(100), 100.1, config.input_timeout);
    Check(observation.MustStop(config, 100.1), "duplicate empty frame cannot erase an obstacle");
    observation.Observe(Frame(99.9), 100.1, config.input_timeout);
    Check(observation.MustStop(config, 100.1), "out-of-order empty frame cannot erase an obstacle");
    observation.Observe(Frame(100.1), 100.1, config.input_timeout);
    Check(!observation.MustStop(config, 100.1), "new valid empty frame releases only this stop");
    observation.SetPose(0, 0, 90, 100.71);
    SetVehicleFeedback(observation, true, 0, false, 100.71);
    observation.Observe(Frame(100.1), 100.71, config.input_timeout);
    Check(observation.MustStop(config, 100.71), "replayed empty frame cannot refresh timeout");
    observation.Observe(Frame(100.71), 100.71, config.input_timeout);
    Check(!observation.MustStop(config, 100.71), "fresh data recovers timeout");
    for(int bad = 0; bad < 5; ++bad) {
        const double now = 100.8 + bad * 0.1;
        auto invalid = Box(100, 100);
        if(bad == 0) invalid.polygons.clear();
        if(bad == 1) invalid.polygons[0].x = std::numeric_limits<double>::quiet_NaN();
        if(bad == 2) invalid.polygons[2].y += 1;
        if(bad == 3) invalid.dx = -1;
        if(bad == 4) std::swap(invalid.polygons[1], invalid.polygons[3]);
        observation.Observe(Frame(now, {Box(100, 100), invalid}), now, config.input_timeout);
        Check(observation.MustStop(config, now), "bad frame cannot partially clear startup");
    }
    observation.Observe(Frame(101.3), 101.3, config.input_timeout);
    observation.SetPose(0, 0, 90, 101.3);
    Check(observation.MustStop(config, 101.32), "stale CAN blocks startup even with fresh perception");
    SetVehicleFeedback(observation, true, 0, false, 101.32);
    Check(!observation.MustStop(config, 101.32), "fresh CAN recovers");
    observation.Observe(Frame(102), 102, config.input_timeout);
    SetVehicleFeedback(observation, true, 0, false, 102);
    Check(observation.MustStop(config, 102), "stale pose cannot clear a current observation");
    observation.SetPose(0, 0, std::numeric_limits<double>::infinity(), 102);
    Check(observation.MustStop(config, 102), "invalid pose blocks even an empty frame");
    observation.SetPose(0, 0, 90, 102);
    Check(!observation.MustStop(config, 102), "fresh valid pose restores clearance");
    observation.Observe(Frame(105), 102, config.input_timeout);
    Check(observation.MustStop(config, 102), "future stamp invalidates clearance");
    observation.Observe(Frame(102.1), 102.1, config.input_timeout);
    Check(!observation.MustStop(config, 102.1), "future packet cannot poison later valid timestamps");
    Check(observation.MustStop(config, 99), "clock rollback is never a fresh clearance");
}

void TestLeftSecondLaneFilter() {
    const CONFIG_S config = Config();
    StartupObservation observation;
    Arm(observation);
    auto object = Box(0, 0);
    object.id = 7;
    double now = 100;
    for(int type : {0, 1, 2, 3, 4, 5, 255, 2}) {
        object.type = type;
        observation.SetPose(0, 0, 90, now);
        SetVehicleFeedback(observation, true, 0, false, now);
        observation.Observe(Frame(now, {object}), now, config.input_timeout);
        const auto status = observation.CheckStatus(config, now);
        Check(status.stop == (type != 2) && status.objects == (type == 2 ? 0u : 1u),
              "startup removes only left-second boxes and clears the previous frame cache");
        now += 0.01;
    }
    auto invalid = object;
    invalid.polygons.clear();
    observation.Observe(Frame(now, {invalid}), now, config.input_timeout);
    Check(!observation.MustStop(config, now), "invalid excluded left-second geometry does not block startup");
    invalid.type = 0;
    now += 0.001;
    observation.Observe(Frame(now, {object, invalid}), now, config.input_timeout);
    Check(observation.MustStop(config, now), "malformed retained geometry still blocks startup");
}

void TestDistances() {
    const CONFIG_S config = Config();
    for(double heading : {0.0, 37.0, 90.0, 180.0, 270.0, 359.9, 360.0}) {
        const double yaw = (90 - heading) * M_PI / 180;
        const double c = std::cos(yaw), s = std::sin(yaw);
        struct CASE_S { double x, y; bool blocked; };
        const CASE_S cases[] = {{5.5, 0, true}, {5.501, 0, false}, {-3.5, 0, true}, {-3.501, 0, false},
                                {0, 3.5, true}, {0, 3.501, false}, {0, -3.5, true}, {0, -3.501, false},
                                {3.5 + std::sqrt(2.0), 1.5 + std::sqrt(2.0), true},
                                {5, 3, false}, {0, 0, true}, {2, 1, true}};
        for(const auto& item : cases) {
            StartupObservation observation;
            Arm(observation);
            observation.SetPose(120, -40, heading, 100);
            auto object = Box(120 + item.x * c - item.y * s, -40 + item.x * s + item.y * c, 1, 1, yaw);
            for(int type = 0; type < 5; ++type) {
                object.type = type;
                object.vx = -15;
                object.heading = 211;
                const double time = 100 + type * 0.01;
                observation.Observe(Frame(time, {object}), time, config.input_timeout);
                Check(observation.MustStop(config, time) == (type != 2 && item.blocked),
                      "physical-box distance preserves all headings and other lanes while excluding left-second");
            }
        }
    }
    for(bool loaded : {false, true}) {
        StartupObservation observation;
        Arm(observation, 100, loaded);
        observation.Observe(Frame(100, {Box(-5, 0)}), 100, config.input_timeout);
        Check(observation.MustStop(config, 100) == loaded, "loaded dimensions include extended rear body");
    }
    StartupObservation observation;
    Arm(observation);
    observation.Observe(Frame(100, {Box(0, 0, 0.5, 20)}), 100, config.input_timeout);
    Check(observation.MustStop(config, 100), "edge-crossing overlap with no contained vertices is blocked");
    observation.Observe(Frame(100.1, {Box(0, 2, 8, 2)}), 100.1, config.input_timeout);
    Check(observation.MustStop(config, 100.1), "elongated real box is not removed by driving filters");
}

// 独立判据：角点绕数/线段相交/点到线段距离，不复用生产的矩形投影和点到矩形公式。
using Point = geometry_msgs::Point;
double Cross(const Point& a, const Point& b, const Point& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}
bool Inside(const Point& p, const std::vector<Point>& polygon) {
    for(int i = 0; i < 4; ++i) if(Cross(polygon[i], polygon[(i + 1) % 4], p) < 0) return false;
    return true;
}
double PointSegment(const Point& p, const Point& a, const Point& b) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double t = std::max(0.0, std::min(1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy)));
    return std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
}
double OracleDistance(const std::vector<Point>& a, const std::vector<Point>& b) {
    if(Inside(a[0], b) || Inside(b[0], a)) return 0;
    double distance = 1e9;
    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 4; ++j) {
            const Point& a2 = a[(i + 1) % 4];
            const Point& b2 = b[(j + 1) % 4];
            if(Cross(a[i], a2, b[j]) * Cross(a[i], a2, b2) < 0 &&
               Cross(b[j], b2, a[i]) * Cross(b[j], b2, a2) < 0) return 0;
            distance = std::min(distance, PointSegment(a[i], b[j], b2));
            distance = std::min(distance, PointSegment(b[j], a[i], a2));
        }
    }
    return distance;
}

void TestIndependentGeometry() {
    std::mt19937 random(20260919);
    std::uniform_real_distribution<double> position(-10, 10), angle(-M_PI, M_PI), size(0.1, 12);
    const CONFIG_S config = Config();
    for(int i = 0; i < 2000; ++i) {
        const double yaw = angle(random), x = position(random), y = position(random);
        const auto vehicle = Box(x + std::cos(yaw), y + std::sin(yaw), 4, 2, yaw);
        const auto object = Box(position(random), position(random), size(random), size(random), angle(random));
        const double distance = OracleDistance(vehicle.polygons, object.polygons);
        StartupObservation observation;
        Arm(observation);
        observation.SetPose(x, y, 90 - yaw * 180 / M_PI, 100);
        observation.Observe(Frame(100, {object}), 100, config.input_timeout);
        Check(observation.MustStop(config, 100) == (distance <= 2), "independent polygon distance agrees with guard");
    }
}

void TestStopDiagnostics() {
    const CONFIG_S config = Config();
    StartupObservation observation;
    auto check = [&](const char* reason, bool stop, double now) {
        const auto status = observation.CheckStatus(config, now);
        Check(std::strcmp(status.reason, reason) == 0, "diagnostic reports the actual stop source");
        Check(status.stop == stop && observation.MustStop(config, now) == stop,
              "diagnostic and safety decision agree");
    };
    check("inactive", false, 100);
    Arm(observation);
    check("waiting_perception_planning", true, 100);
    Check(observation.CheckStatus(config, 100).perception_age == -1, "missing observation age is explicit");
    observation.Observe(Frame(100), 100, config.input_timeout);
    check("clear", false, 100);
    auto object = Box(0, 0);
    object.id = 27;
    observation.Observe(Frame(100.1, {object}), 100.1, config.input_timeout);
    check("nearby_obstacle", true, 100.1);
    Check(observation.CheckStatus(config, 100.1).object_id == 27, "blocking object ID is reported");
    object.polygons.clear();
    observation.Observe(Frame(100.2, {object}), 100.2, config.input_timeout);
    check("invalid_perception_box", true, 100.2);
    Check(observation.CheckStatus(config, 100.2).object_id == 27, "malformed object ID is reported");
    observation.Observe(Frame(200), 100.3, config.input_timeout);
    check("invalid_perception_time", true, 100.3);
    observation.Observe(Frame(100.3), 100.3, config.input_timeout);
    observation.SetPose(0, 0, std::numeric_limits<double>::quiet_NaN(), 100.3);
    check("invalid_or_missing_pose", true, 100.3);
    observation.SetPose(0, 0, 90, 100.3);
    SetVehicleFeedback(observation, true, std::numeric_limits<double>::quiet_NaN(), false, 100.3);
    check("invalid_navigation_speed", true, 100.3);
    SetVehicleFeedback(observation, true, 0, false, 100.3);
    check("clock_mismatch", true, 99);
    observation.SetPose(0, 0, 90, 101);
    SetVehicleFeedback(observation, true, 0, false, 101);
    check("perception_timeout", true, 101);
    observation.Observe(Frame(101), 101, config.input_timeout);
    observation.SetPose(0, 0, 90, 100);
    check("pose_timeout", true, 101);
    observation.SetPose(0, 0, 90, 101);
    SetVehicleFeedback(observation, true, 0, false, 100);
    check("can_timeout", true, 101);
    SetVehicleFeedback(observation, true, 0, false, 101);
    check("clear", false, 101);
}

int main(int argc, char** argv) {
    TestLeftSecondLaneFilter();
    if(argc == 2 && std::strcmp(argv[1], "--left-second-only") == 0) {
        std::cout << "PASS startup left-second filter: " << checks << " checks\n";
        return 0;
    }
    TestLifecycle();
    TestFreshnessAndBadFrames();
    TestDistances();
    TestIndependentGeometry();
    TestStopDiagnostics();
    std::cout << "PASS startup observation core: " << checks << " checks\n";
}
