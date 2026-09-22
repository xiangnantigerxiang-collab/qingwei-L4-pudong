// 与规划测试分进程，避免两个节点的不同 Pose2d 定义在同一程序中发生 ODR 冲突。
#include <iostream>
#include <fstream>
#include <cstdlib>
#define private public
#include "robot_control/stanley_controller/stanley_controller.h"
#undef private
#include "robot_control/control_comply.h"

int checks = 0;
ControlComply control;

void Check(bool passed, const std::string &name) {
    if(!passed) {
        std::cerr << "FAIL: " << name << '\n';
        std::exit(1);
    }
    ++checks;
}

void TestFeedforward() {
    LatController lateral;
    lateral.setParameters(1.6, 22, 8);
    std::vector<Pose2d> trajectory;
    for(int i = 0; i <= 80; ++i) {
        const double s = i * 0.25;
        trajectory.emplace_back(100 + s, 100 + 0.03 * s * s);
    }
    math_utils::computePoseAttr(trajectory);
    Pose2d ego = trajectory[16];
    const double full = lateral.feedforward(trajectory, ego, 0.3);
    std::vector<Pose2d> cropped(trajectory.begin() + 12, trajectory.end());
    math_utils::computePoseAttr(cropped);
    Check(std::abs(full - lateral.feedforward(cropped, ego, 0.3)) < 1e-10,
          "feedforward does not change when points behind ego are added or removed");
    trajectory.resize(19);
    const double short_angle = lateral.feedforward(trajectory, ego, 0.3);
    const Pose2d target = lateral.toBaseLink(trajectory.back(), ego);
    const double expected = std::atan(3.2 * target.y / (target.x * target.x + target.y * target.y));
    Check(std::isfinite(short_angle) && std::abs(short_angle - expected) < 1e-10,
          "short terminal reference uses route endpoint instead of world origin");
    Check(lateral.feedforward(trajectory, trajectory.back(), 0.3) == 0,
          "coincident endpoint does not divide by zero");
}

void TestReverseTerminalSteering() {
    GeometricConstrol reverse;
    XYZ_COOR_S origin{};
    Check(reverse.LateralControlTrack1({}, origin, 0, 0.3, GEAR_R) == 0,
          "empty reverse trajectory has no invalid preview access");
    Check(reverse.LateralControlTrack1({origin}, origin, 0, 0.3, GEAR_R) == 0,
          "single-point reverse trajectory has no unsigned index underflow");
    for(double heading : {0.0, 90.0, 180.0, 270.0, 360.0}) {
        std::vector<XYZ_COOR_S> straight;
        for(int i = 0; i <= 100; ++i) {
            XYZ_COOR_S point{};
            point.x_axis = 100 - i * 0.1 * std::sin(heading * M_PI / 180);
            point.y_axis = 100 - i * 0.1 * std::cos(heading * M_PI / 180);
            point.heading = heading;
            straight.push_back(point);
        }
        for(int key : {60, 90, 92, 95, 98, 99}) {
            Check(reverse.LateralControlTrack1(straight, straight[key], key, 0.3, GEAR_R) == 0,
                  "short straight reverse path stays straight for every cardinal heading including 360 degrees");
        }
    }
    for(int side : {-1, 1}) {
        std::vector<XYZ_COOR_S> path;
        for(int i = 0; i <= 100; ++i) {
            const double theta = i * 0.1 / 10;
            XYZ_COOR_S point{};
            point.x_axis = 100 - 10 * std::sin(theta);
            point.y_axis = 100 + side * 10 * (1 - std::cos(theta));
            point.heading = 90 + side * theta * 180 / M_PI;
            path.push_back(point);
        }
        const double expected = -side * std::atan(1.6 / 10) * 180 / M_PI;
        for(int key : {60, 88, 90, 91, 92, 95, 98, 99}) {
            const double angle = reverse.LateralControlTrack1(path, path[key], key, 0.3, GEAR_R);
            Check(std::isfinite(angle) && angle * expected > 0 && std::abs(angle - expected) < 0.3,
                  "reverse circular path keeps the geometric steering angle through the final 0.1 to 0.8 m");
        }
        Check(reverse.LateralControlTrack1(path, path.back(), 100, 0.3, GEAR_R) == 0,
              "coincident reverse endpoint returns zero without inventing a forward target");
    }
}

int main(int argc, char **argv) {
    Check(argc == 2, "planning message fixture supplied");
    TestFeedforward();
    TestReverseTerminalSteering();
    std::ifstream input(argv[1]);
    Check(input.is_open(), "planning output can be opened");
    int cases = 0, gear = 0, state = 0, path_id = 0, task_type = 0, desired_gear = 0;
    double speed = 0, x = 0, y = 0, heading = 0, desired = 0, distance = 0;
    bool safety = false;
    std::size_t count = 0;
    ros::param::set("/planning/alive", 1);
    while(input >> gear >> speed >> x >> y >> heading >> state >> desired >> task_type >> desired_gear >> distance >> path_id >> safety >> count) {
        robot::navigation_msg navigation;
        navigation.xAxis = x;
        navigation.yAxis = y;
        navigation.heading = heading;
        navigation.gpsSpeed = speed;
        robot::can_msg can;
        can.controlPanelState = 1;
        can.curGear = gear;
        can.vehicleSpeed = speed;
        robot::path_plan_status status;
        status.taskExecuStatus = state;
        status.distance2Stop = distance;
        robot::task_plan_msg task;
        task.taskType = task_type;
        task.desireGear = desired_gear;
        robot::path_plan_msg path;
        path.desireSpeed = desired;
        path.Path_Id = path_id;
        path.safety = safety;
        path.x.resize(count);
        path.y.resize(count);
        for(std::size_t i = 0; i < count; ++i) input >> path.x[i] >> path.y[i];
        Check(bool(input) && (cases == 16 ? count == 0 : count >= 10),
              "complete approach or endpoint message read from actual planner output");
        control.SetNavigationData(navigation);
        control.SetCanData(can);
        control.SetPathStatusData(status);
        control.setTaskPlanData(task);
        control.SetPathPlanData(path);
        control.VehicleControl();
        robot::control_msg output;
        ros::Publisher publisher;
        publisher.capture = [&](const std::type_info &, const void *raw) {
            output = *static_cast<const robot::control_msg *>(raw);
        };
        control.PublishMessage(publisher);
        if(cases == 16) {
            Check(output.throttlePercent == 0 && output.brakePercent == 5 && output.desireSpeed == 0,
                  "actual planner arrival enters low-speed D terminal brake ramp");
            navigation.gpsSpeed = 0.02;
            control.SetNavigationData(navigation);
            control.SetCanData(can);
            control.VehicleControl();
            ros::testTime() += 0.35;
            control.SetNavigationData(navigation);
            control.SetCanData(can);
            control.VehicleControl();
            control.PublishMessage(publisher);
            Check(output.throttlePercent == 0 && output.brakePercent == 80,
                  "actual terminal chain holds original brake once stopped");
        } else {
            Check(output.throttlePercent > 0 && output.brakePercent == 0,
                  "actual D/R controller keeps approaching on shortened terminal reference");
            Check(std::isfinite(output.wheelAngle) && std::abs(output.biaDistance) < 0.1,
                  "terminal reference spline and steering remain usable");
        }
        if(cases >= 10 && cases < 16) {
            Check(gear == GEAR_R && std::abs(output.wheelAngle) > 1,
                  "actual shortened reverse curve never loses steering during terminal approach");
        }
        ++cases;
    }
    Check(cases == 17, "all D/R approach messages and the final ordinary D stop reached controller");
    std::cout << "PASS terminal control: " << checks << " checks\n";
    return 0;
}
