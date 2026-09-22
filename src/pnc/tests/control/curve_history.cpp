// 链接真实ControlComply；访问头由验证脚本生成，不修改生产可见性。
#include "curve_history_access.h"
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <new>
#include <sys/resource.h>
#include <fstream>

ControlComply control;
ControlComply second_control;
static int checks = 0;
static bool count_allocations = false;
static std::size_t allocations = 0;
static bool independent_curve_cases = false;

void* operator new(std::size_t size) {
    void* result = std::malloc(size ? size : 1);
    if(!result) throw std::bad_alloc();
    if(count_allocations) ++allocations;
    return result;
}
// 避免GCC内联后把自定义malloc/free统计器误报为new/delete不匹配。
__attribute__((noinline)) void operator delete(void* pointer) noexcept { std::free(pointer); }
void* operator new[](std::size_t size) { return ::operator new(size); }
__attribute__((noinline)) void operator delete[](void* pointer) noexcept { ::operator delete(pointer); }

void Check(bool result, const char* name) {
    if(!result) {
        std::cerr << "FAIL: " << name << '\n';
        std::exit(1);
    }
    ++checks;
}

std::vector<XYZ_COOR_S> Path(double curvature, std::size_t count = 60) {
    std::vector<XYZ_COOR_S> path(count);
    for(std::size_t i = 0; i < count; ++i) {
        path[i].x_axis = i * 0.1;
        path[i].curvature = curvature;
    }
    return path;
}

double Limit(ControlComply& target, const std::vector<XYZ_COOR_S>& path) {
#ifdef CURVE_HISTORY_LEGACY_BASELINE
    return target.CurveLimitSpeed(path);
#else
#ifndef CURVE_HISTORY_BASELINE
    if(&path == &target.mPathList) return target.ForwardCurveLimitSpeed(target.mForwardPathList);
#endif
    return target.ForwardCurveLimitSpeed(path);
#endif
}

void Seed(ControlComply& target) {
    for(int i = 0; i < 30; ++i) Limit(target, Path(i % 2 ? 0.05 : 0.04));
}

struct Inputs {
    robot::can_msg can;
    robot::task_plan_msg task;
    robot::path_plan_msg plan;
    robot::path_plan_status status;
};

Inputs Prepare(int gear = GEAR_D, double curve = 0.0, int task_id = 1) {
    if(independent_curve_cases) {
        // 独立曲率用例不继承上一停车用例的真实液压在途状态或回跳时钟。
        // 真正跨挡/跨任务执行器生命周期由 forward_brake_feedback/non_d_brake_compat 验证。
        control.~ControlComply();
        new(&control) ControlComply();
    }
    ros::testTime() = 100;
    ros::param::set("/planning/sensorstate", 0);
    ros::param::set("/robot/control/accswitch", 0);
    ros::param::set("/planning/alive", 1);
    ros::param::set("/robot/planning/netcheck", 0);
#ifdef CURVE_HISTORY_LEGACY_BASELINE
    control.ResetLaunchSpeed();
#else
    control.ResetForwardCurveHistory();
#endif
    control.ResetBrakeIntegral();
    Inputs input;
    input.can.curGear = gear;
    input.can.vehicleSpeed = 3;
    input.can.controlPanelState = 1;
    control.SetCanData(input.can);
    input.task.task_id = task_id;
    input.task.taskType = TRACKPATH;
    input.task.desireGear = gear;
    input.task.pathList.push_back("pudong_air/312_cargo_01_01");
    control.setTaskPlanData(input.task);
    robot::navigation_msg nav;
    nav.heading = 90;
    nav.gpsSpeed = 3;
    control.SetNavigationData(nav);
    input.status.taskExecuStatus = 1;
    control.SetPathStatusData(input.status);
    input.plan.Path_Id = 1;
    input.plan.desireSpeed = 3;
    for(int i = 0; i <= 100; ++i) {
        const double x = (gear == GEAR_R ? -1 : 1) * i * 0.2;
        input.plan.x.push_back(x);
        input.plan.y.push_back(0.2 + curve * x * x);
    }
    control.SetPathPlanData(input.plan);
    return input;
}

std::vector<robot::control_msg> Tick(Inputs& input) {
    control.SetCanData(input.can);
    robot::navigation_msg navigation = control.mNavData;
    navigation.gpsSpeed = input.can.vehicleSpeed;
    control.SetNavigationData(navigation);
    std::vector<robot::control_msg> messages;
    ros::Publisher publisher;
    publisher.name = "/control_msg";
    publisher.capture = [&](const std::type_info& type, const void* raw) {
        if(type == typeid(robot::control_msg)) messages.push_back(*static_cast<const robot::control_msg*>(raw));
    };
    control.VehicleControl();
    control.PublishMessage(publisher);
    ros::testTime() += 0.05;
    return messages;
}

void Trace(const std::vector<robot::control_msg>& messages) {
    for(const auto& msg : messages) {
        std::cout << std::setprecision(9) << "TRACE " << +msg.throttlePercent << ' ' << +msg.brakePercent
                  << ' ' << msg.desireSpeed << ' ' << msg.wheelAngle << ' ' << msg.biaDistance
                  << ' ' << msg.biaAngle << ' ' << msg.preCurve << ' ' << msg.preAngleDev
                  << ' ' << msg.desireAcc << ' ' << msg.vehicleSpeed << ' ' << +msg.remoteEnable
                  << ' ' << +msg.bypassProcessing << ' ' << msg.faultCode.size();
        for(const auto code : msg.faultCode) std::cout << ' ' << +code;
        std::cout << '\n';
    }
}

#ifndef CURVE_HISTORY_BASELINE
void TestHistory() {
    robot::navigation_msg nav;
    nav.gpsSpeed = 1.5;  // 4秒预瞄为6m，保留原历史/窗口用例的空间范围。
    control.SetNavigationData(nav);
    const auto straight = Path(0);
    Check(Limit(control, straight) == 600, "fresh straight path retains the 0.6 coefficient");
    Seed(control);
    double previous = 0;
    for(int i = 1; i <= 30; ++i) {
        const double value = Limit(control, straight);
        Check(value >= previous, "straight path releases old curve restriction monotonically");
        if(i < 30) Check(value < 600, "30-sample release history is retained");
        previous = value;
    }
    Check(previous == 600, "all previous curves expire after 30 straight calculations");
    for(int i = 0; i < 600; ++i) Check(Limit(control, straight) == 600, "no old curvature reappears");
    for(double curvature : {0.0005, 0.005, 0.04, 0.09}) {
        control.ResetForwardCurveHistory();
        const auto path = Path(curvature);
        double value = 0;
        for(int i = 0; i < 30; ++i) value = Limit(control, path);
        const double coefficient = curvature <= 0.005 ? 0.6 : 0.28;
        Check(std::abs(value - coefficient / std::sqrt(curvature)) < 1e-5,
              "constant tight turns use the 1.4m/s reference and near-straight paths retain 0.6");
    }
    for(std::size_t size : {1U, 2U, 29U, 30U, 60U, 10000U}) {
        control.ResetForwardCurveHistory();
        const auto path = Path(0.04, size);
        Check(std::abs(Limit(control, path) - 1.4) < 1e-5,
              "short paths do not dilute a tight turn with zero padding");
    }
    control.ResetForwardCurveHistory();
    auto beyond = Path(0);
    for(int i = 30; i < 60; ++i) beyond[i].curvature = 0.1;
    Check(std::abs(Limit(control, beyond) - 0.28 / std::sqrt(0.1)) < 1e-5,
          "a bend 3 to 6m ahead is detected on the first cycle");
    Seed(control);
    Check(Limit(second_control, straight) == 600, "D instances do not share history");
    Check(Limit(control, {}) == 600 && Limit(control, straight) == 600, "empty input discards D history");

    for(int reason = 0; reason < 12; ++reason) {
        Inputs input = Prepare();
        Seed(control);
        if(reason == 0) {
            ++input.task.task_id;
            control.setTaskPlanData(input.task);
        } else if(reason == 1) {
            input.task.taskType = DOACTION;
            control.setTaskPlanData(input.task);
        } else if(reason == 2) {
            input.task.desireGear = GEAR_R;
            control.setTaskPlanData(input.task);
        } else if(reason == 3) {
            input.can.curGear = GEAR_R;
            control.SetCanData(input.can);
            input.can.curGear = GEAR_D;
            control.SetCanData(input.can);
        } else if(reason == 4) {
            input.can.controlPanelState = 0;
            control.SetCanData(input.can);
            input.can.controlPanelState = 1;
            control.SetCanData(input.can);
        } else if(reason == 5) {
            ++input.plan.Path_Id;
            control.SetPathPlanData(input.plan);
        } else if(reason == 6) {
            input.plan.x.pop_back();
            control.SetPathPlanData(input.plan);
        } else if(reason == 7) {
            input.plan.x.resize(9);
            input.plan.y.resize(9);
            control.SetPathPlanData(input.plan);
        } else if(reason == 8) {
            input.plan.x.assign(10, 0);
            input.plan.y.assign(10, 0);
            control.SetPathPlanData(input.plan);
        } else if(reason == 9) {
            input.plan.desireSpeed = 0;
            control.SetPathPlanData(input.plan);
            Tick(input);
        } else if(reason == 10) {
            input.status.taskExecuStatus = TASKFINISHED;
            control.SetPathStatusData(input.status);
            input.plan.desireSpeed = 0;
            control.SetPathPlanData(input.plan);
            Tick(input);
        } else {
            input.task.pathList[0] = "pudong_air/312_charge_01_01";
            control.setTaskPlanData(input.task);
        }
        Check(Limit(control, straight) == 600, "context changes and stopped control discard D history");
    }
    {
        Inputs input = Prepare();
        Seed(control);
        control.SetCanData(input.can);
        control.setTaskPlanData(input.task);
        control.SetPathPlanData(input.plan);
        Check(Limit(control, straight) < 3, "repeated same task, CAN and path do not erase valid history");
    }
    for(int stop : {0, 1, 3}) {
        Inputs input = Prepare();
        Seed(control);
        if(stop == 0) {
            input.plan.safety = true;
            control.SetPathPlanData(input.plan);
        } else if(stop == 1) {
            ros::param::set("/planning/sensorstate", 8);
        } else {
            control.mPathList.clear();
        }
        const auto messages = Tick(input);
        const int expected = stop < 2 ? 100 : 50;
        Check(!messages.empty() && messages.back().throttlePercent == 0 && messages.back().brakePercent == expected,
              "original independent stop keeps zero throttle and its brake priority");
        Check(Limit(control, straight) == 600, "independent stop clears only D curve history");
    }
}

double FreshLimit(std::vector<XYZ_COOR_S> path, double x = 0, double y = 0, int key = 0, double speed = 1.5) {
    ControlComply target;
    robot::navigation_msg nav;
    nav.xAxis = x;
    nav.yAxis = y;
    nav.gpsSpeed = speed;
    target.SetNavigationData(nav);
    target.mKeyPoint = key;  // 主流程已计算最近点；这里单独验证其相邻线段投影及6m窗口。
    return Limit(target, path);
}

void TestPreview() {
    // 独立标定锚点，而非在测试里复制生产分段公式。
    const double calibration[][2] = {
        {0.0005, 26.832815730}, {0.005, 8.485281374}, {0.01, 5.377777778},
        {0.0125, 4.288396501}, {0.015, 3.420238345}, {0.02, 2.426860311},
        {0.025, 2.023857703}, {0.03, 1.713518577}, {0.035, 1.511840754},
        {0.039, 1.417977748}, {0.04, 1.4}, {0.041, 1.382821435},
        {0.1, 0.885437745}, {0.2, 0.626099034}
    };
    for(const auto& row : calibration) {
        for(double sign : {-1.0, 1.0}) {
            Check(std::fabs(FreshLimit(Path(sign * row[0], 100)) - row[1]) < 2e-5,
                  "left and right bends match the agreed speed calibration");
        }
    }
    double previous = 600;
    for(int i = 1; i <= 4000; ++i) {
        const double curvature = i * 0.0001;
        const double value = FreshLimit(Path(curvature));
        Check(std::isfinite(value) && value > 0 && value <= previous + 1e-5,
              "larger curvature never increases the speed limit");
        Check(value >= 0.28 / std::sqrt(curvature) - 1e-5 && value <= 0.6 / std::sqrt(curvature) + 1e-5,
              "curve stays between the 0.28 tight-turn and 0.6 straight envelopes");
        if(curvature >= 0.04) {
            Check(std::fabs(value - 0.28 / std::sqrt(curvature)) < 1e-5,
                  "turns at and beyond the reference curvature keep the requested reduced coefficient");
        }
        previous = value;
    }
    for(double edge : {0.005, 0.01, 0.02, 0.04}) {
        Check(std::fabs(FreshLimit(Path(edge - 1e-8)) - FreshLimit(Path(edge + 1e-8))) < 3e-5,
              "transition boundaries have no speed step");
        const auto before = Path(edge - 1e-6), center = Path(edge), after = Path(edge + 1e-6);
        const double center_limit = FreshLimit(center);
        const double left = (center_limit - FreshLimit(before)) / (center[0].curvature - before[0].curvature);
        const double right = (FreshLimit(after) - center_limit) / (after[0].curvature - center[0].curvature);
        Check(std::fabs(left - right) < 1.0 + 0.005 * std::fabs(left),
              "existing and new transitions retain continuous speed slope");
    }

    auto path = Path(0, 201);
    for(auto& point : path) point.x_axis -= 10;
    path[20].curvature = 0.2;
    path[161].curvature = 0.2;
    Check(FreshLimit(path, 0, 0, 100) == 600, "old points behind and peaks beyond 6m are excluded");
    path[160].curvature = 0.04;
    Check(std::fabs(FreshLimit(path, 0, 0, 100) - 1.4) < 1e-5, "a peak exactly 6m ahead is included");
    path[160].curvature = 0;
    path[159].curvature = -0.1;
    Check(std::fabs(FreshLimit(path, 0, 0, 100) - 0.885437745) < 1e-5,
          "negative curvature at 5.9m is included without averaging away a short bend");
    path[159].curvature = 0;
    Check(FreshLimit(path, 0.2, 0, 102) < 1, "moving the vehicle advances the preview window");

    auto sparse = Path(0, 5);
    for(std::size_t i = 0; i < sparse.size(); ++i) sparse[i].x_axis = i * 2;
    sparse.back().curvature = 0.08;
    Check(std::fabs(FreshLimit(sparse, 1.5, 0, 1) - 0.28 / std::sqrt(0.06)) < 1e-5,
          "projection within the previous segment and the 6m boundary are interpolated");
    Check(std::fabs(FreshLimit(sparse, 0.5, 0, 0) - 2.426860311) < 1e-5,
          "projection within the next segment uses physical distance rather than point count");
    auto duplicate = sparse;
    duplicate.insert(duplicate.begin() + 2, duplicate[1]);
    Check(FreshLimit(sparse, 1.5, 0, 1) == FreshLimit(duplicate, 1.5, 0, 1),
          "repeated points add no lookahead distance and do not change the limit");

    auto folded = Path(0, 5);
    const double xy[][2] = {{0, 0}, {2, 0}, {2, 2}, {0, 2}, {0, 0.5}};
    for(std::size_t i = 0; i < folded.size(); ++i) {
        folded[i].x_axis = xy[i][0];
        folded[i].y_axis = xy[i][1];
    }
    folded.back().curvature = 0.2;
    Check(FreshLimit(folded) == 600, "U-turn preview uses arc length, not a 6m Euclidean circle");
    folded[2].curvature = -0.04;
    Check(std::fabs(FreshLimit(folded) - 1.4) < 1e-5, "the turn within the first 6m still limits speed");
    Check(std::fabs(FreshLimit(Path(0.04, 1)) - 1.4) < 1e-5, "a single real endpoint is not zero padded");
    Check(FreshLimit(Path(0.04), 0, 0, -1) == 0, "invalid nearest index cannot raise the limit");
    Check(FreshLimit(Path(0.04), 0, 0, 60) == 0, "out of range nearest index cannot read past the path");
    for(double invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        auto bad = Path(0, 100);
        bad[20].curvature = invalid;
        Check(FreshLimit(bad) == 0, "nonfinite curvature cannot silently remove the limit");
        bad = Path(0, 100);
        bad[20].x_axis = invalid;
        Check(FreshLimit(bad) == 0, "nonfinite preview geometry cannot silently remove the limit");
    }

    for(double sign : {-1.0, 1.0}) {
        Inputs input = Prepare();
        control.mPathList = Path(0, 201);
        control.mPathList[50].curvature = sign * 0.04;
        control.mForwardPathList = control.mPathList;  // 已知曲率注入，只测最终限速/制动优先级。
        const auto messages = Tick(input);
        Check(messages.size() == 1 && messages.front().throttlePercent == 18 && messages.front().brakePercent == 0,
              "full D control applies a 5m-ahead limit at existing 13.5 calibration without inventing braking");
    }
    // 从真实圆弧坐标经过SetPathPlanData的过滤/样条/曲率，再驱动完整控制流程。
    for(double radius : {5.0, 10.0, 25.0}) {
        double left = 0;
        for(double sign : {-1.0, 1.0}) {
            Inputs input = Prepare();
            input.plan.x.clear();
            input.plan.y.clear();
            for(int i = -5; i <= 100; ++i) {
                const double angle = i * 0.2 / radius;
                input.plan.x.push_back(radius * std::sin(angle));
                input.plan.y.push_back(0.2 + sign * radius * (1 - std::cos(angle)));
            }
            control.SetPathPlanData(input.plan);
            const auto messages = Tick(input);
            ControlComply snapshot = control;
            const double value = Limit(snapshot, snapshot.mPathList);
            Check(std::fabs(value / (0.28 * std::sqrt(radius)) - 1) < 0.06,
                  "real resampled left/right turn geometry agrees with the 1.4m/s reference calibration");
            Check(!messages.empty() && messages.back().brakePercent == 0 && messages.back().throttlePercent <= 18,
                  "real tight turns reduce the motor speed command before the curve");
            if(sign < 0) left = value;
            else Check(std::fabs(value - left) < 0.02, "left and right real circular paths remain symmetric");
        }
    }
}

void TestSpeedGap() {
    const double cases[][3] = {
        {3, 0, 6}, {3, 0.1, 8}, {3, 0.5, 13}, {3, 1, 20}, {3, 2, 33},
        {3, 3, 40}, {0.5, 0, 6}, {0.49, 0, 6}, {0.51, 0, 6},
        {0.2, 0, 5}, {0, 0, 0}, {1, 2, 13},
        {4.5, 3, 47}, {4.5, 3.9, 59}, {4.5, 4, 60}, {4.5, 4.5, 60},
        {5, 0, 6}, {5, 4, 60}, {5, 4.4, 66}, {5, 4.5, 67}, {5, 5, 67}
    };
    for(const auto& row : cases) {
        Inputs input = Prepare();
        input.plan.desireSpeed = row[0];
        control.SetPathPlanData(input.plan);
        control.ResetBrakeIntegral();
        input.can.vehicleSpeed = row[1];
        for(int i = 0; i < 3; ++i) {
            const auto messages = Tick(input);
            Check(messages.size() == 1 && messages.front().throttlePercent == row[2] &&
                  messages.front().brakePercent == 0,
                  "restored current-speed plus 0.5 cap preserves 13.5 scaling and low-speed floor");
        }
    }
    for(double actual : {0.0, 1.0, 2.0, 3.0}) {
        Inputs input = Prepare();
        control.mPathList = Path(0.04, 101);
        control.mForwardPathList = control.mPathList;
        input.can.vehicleSpeed = actual;
        const int expected = actual == 0 ? 6 : 18;
        const auto messages = Tick(input);
        Check(messages.size() == 1 && messages.front().throttlePercent == expected,
              "speed-gap cap and dynamic curve cap take the stricter result");
    }
}

void TestDynamicPreview() {
    // 线性曲率只用于精确观测取点范围；空间限速标定由上方独立锚点检查。
    auto ramp = Path(0, 121);
    for(std::size_t i = 0; i < ramp.size(); ++i) {
        ramp[i].x_axis = i * 0.25;
        ramp[i].curvature = i * 0.00025;
    }
    for(double speed : {-1.0, 0.0, 0.05, 0.125, 0.5, 1.0, 1.5, 2.0, 3.0, 4.5, 5.0, 10.0}) {
        ControlComply target;
        robot::navigation_msg nav;
        nav.xAxis = 1.125;  // 从线段内投影起算，不从第0点或最近点整数位置起算。
        nav.gpsSpeed = speed;
        target.SetNavigationData(nav);
        target.mKeyPoint = 4;
        const double limit = Limit(target, ramp);
        const double endpoint = std::min(30.0, 1.125 + 4.0 * std::max(0.0, speed));
        Check(std::fabs(target.mForwardCurvatures[0] - endpoint * 0.001) < 1e-8,
              "4-second preview follows navigation speed and clips only at the real endpoint");
        Check(std::isfinite(limit) && limit > 0, "dynamic window produces a finite positive cap");
    }
    auto peak = Path(0, 301);
    peak[80].curvature = -0.04;  // 前方8m的右转峰值。
    Check(FreshLimit(peak, 0, 0, 0, 1.0) == 600, "1m/s previews 4m without retaining a 6m floor");
    Check(FreshLimit(peak, 0, 0, 0, 1.5) == 600, "1.5m/s still excludes an 8m bend");
    Check(std::fabs(FreshLimit(peak, 0, 0, 0, 2.0) - 1.4) < 1e-5,
          "2m/s includes a bend exactly on the 8m boundary");
    Check(std::fabs(FreshLimit(peak, 0, 0, 0, 5.0) - 1.4) < 1e-5,
          "5m/s sees the bend beyond the former fixed 6m window");
    peak[80].curvature = 0;
    peak[200].curvature = 0.04;
    Check(std::fabs(FreshLimit(peak, 0, 0, 0, 5.0) - 1.4) < 1e-5,
          "5m/s includes the exact 20m boundary");
    peak[200].curvature = 0;
    peak[201].curvature = 0.04;
    Check(FreshLimit(peak, 0, 0, 0, 5.0) == 600, "a 20.1m peak is outside the 5m/s window");
    peak[201].curvature = 0;
    peak[1].curvature = 0.04;
    Check(FreshLimit(peak, 0, 0, 0, 0.0) == 600, "zero speed does not scan a positive lookahead distance");
    peak[0].curvature = 0.04;
    Check(std::fabs(FreshLimit(peak, 0, 0, 0, 0.0) - 1.4) < 1e-5,
          "zero speed still respects curvature at the current projection");
    for(double invalid : {std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
        Check(FreshLimit(peak, 0, 0, 0, invalid) == 0,
              "nonfinite navigation speed cannot silently release curvature limiting");
    }

    // 只更新导航回调，规划和几何缓存不变，预瞄范围须当周期跟随当前车速变化。
    Inputs input = Prepare();
    input.plan.desireSpeed = 5;
    control.SetPathPlanData(input.plan);
    control.mPathList = Path(0, 301);
    control.mPathList[80].curvature = 0.04;
    control.mForwardPathList = control.mPathList;
    input.can.vehicleSpeed = 99;
    control.SetCanData(input.can);
    robot::navigation_msg nav = control.mNavData;
    nav.gpsSpeed = 1;
    control.SetNavigationData(nav);
    control.VehicleControl();
    Check(control.mControlData.throttlePercent == 20 && control.mControlData.brakePercent == 0,
          "preview uses current navigation speed, not CAN speed or the planning target");
    nav.gpsSpeed = 3;
    control.SetNavigationData(nav);
    control.VehicleControl();
    Check(control.mControlData.throttlePercent == 18 && control.mControlData.brakePercent == 0,
          "a navigation-only speed increase immediately brings a farther bend into the limit");
    for(int i = 0; i < 30; ++i) Limit(control, control.mPathList);
    nav.gpsSpeed = 1;
    control.SetNavigationData(nav);
    double previous = 1.4;
    for(int i = 0; i < 30; ++i) {
        const double limit = Limit(control, control.mPathList);
        Check(limit >= previous, "shrinking preview retains monotonic history release");
        if(i < 29) Check(limit < 600, "shrinking preview does not discard the 30-cycle history");
        previous = limit;
    }
    Check(previous == 600, "old distant bend expires after 30 smaller-window observations");
}

void TestCurveNoise() {
    // 保存的实车输入由准确几何匹配的88帧构成；必须从真实规划回调经过样条再测最终输出。
    const std::string source = __FILE__;
    Inputs input;
    for(double target : {3.0, 4.5, 5.0}) {
        std::ifstream file(source.substr(0, source.find_last_of('/')) + "/fixtures/curve_noise_20260921.txt");
        Check(bool(file), "recorded straight-path fixture exists");
        input = Prepare();
        int count = 0, previous_plan = -1;
        file >> count;
        Check(count == 88, "recorded fixture retains every matched straight control frame");
        for(int frame = 0; frame < count; ++frame) {
            double time;
            int plan, path_id, safety, points;
            robot::navigation_msg nav;
            file >> time >> plan >> input.plan.desireSpeed >> nav.gpsSpeed >> nav.xAxis >> nav.yAxis >> nav.heading
                 >> path_id >> safety >> points;
            input.plan.Path_Id = path_id;
            input.plan.safety = safety;
            input.plan.desireSpeed = target;
            if(target > 3.0) nav.gpsSpeed = target;  // 同一实车几何的4.5/5m/s输出验收，非原记录实测速度。
            input.plan.x.resize(points);
            input.plan.y.resize(points);
            for(int j = 0; j < points; ++j) file >> input.plan.x[j] >> input.plan.y[j];
            Check(bool(file), "recorded frame is complete");
            ros::testTime() = 1000 + time;
            input.can.vehicleSpeed = nav.gpsSpeed;
            control.SetCanData(input.can);
            control.SetNavigationData(nav);
            if(plan != previous_plan) control.SetPathPlanData(input.plan);
            previous_plan = plan;
            control.VehiclePoseCalculation();
            ControlComply snapshot = control;
            const double limit = Limit(snapshot, snapshot.mPathList);
            const auto messages = Tick(input);
            Check(limit >= target, "real millimetre straight-path noise permits 3, 4.5 and 5m/s targets");
            const int expected_oil = target == 3.0 ? 40 : (target == 4.5 ? 60 : 67);
            Check(messages.size() == 1 && messages.front().throttlePercent == expected_oil &&
                  messages.front().brakePercent == 0,
                  "all 88 real straight geometries retain a steady motor command at each target speed");
        }
    }

    // 连续改变噪声相位，覆盖直线、左右圆弧及掉头；保持真实几何，不用伪造曲率字段代替路径。
    const double noisy_calibration[][2] = {
        {0, 5}, {5, 0.626099034}, {10, 0.885437745}, {25, 1.4},
        {100.0 / 3.0, 1.713518577}, {40, 2.023857703}, {50, 2.426860311},
        {80, 4.288396501}, {100, 5.377777778}
    };
    for(const auto& row : noisy_calibration) {
        const double radius = row[0];
        for(double sign : {-1.0, 1.0}) {
            input = Prepare();
            input.can.vehicleSpeed = radius == 0 || radius > 25 ? 5 : 3;
            input.plan.desireSpeed = input.can.vehicleSpeed;
            double lowest = 600, highest = 0;
            int lowest_oil = 100, highest_oil = 0;
            for(int frame = 0; frame < 100; ++frame) {
                input.plan.x.clear();
                input.plan.y.clear();
                for(int j = -5; j <= 150; ++j) {
                    const double s = j * 0.2;
                    const double offset = 0.01 * std::sin(2.0 * M_PI * s / 0.9 + frame * 0.31);
                    const double angle = radius > 0 ? s / radius : 0.0;
                    const double x = radius > 0 ? radius * std::sin(angle) - offset * std::sin(angle) : s;
                    const double y = radius > 0 ? radius * (1.0 - std::cos(angle)) + offset * std::cos(angle) : offset;
                    input.plan.x.push_back(x);
                    input.plan.y.push_back(0.2 + sign * y);
                }
                control.SetPathPlanData(input.plan);
                const auto messages = Tick(input);
                ControlComply snapshot = control;
                const double limit = Limit(snapshot, snapshot.mPathList);
                lowest = std::min(lowest, limit);
                highest = std::max(highest, limit);
                const int oil = messages.back().throttlePercent;
                lowest_oil = std::min(lowest_oil, oil);
                highest_oil = std::max(highest_oil, oil);
                if(radius == 0) {
                    Check(oil == 67 && messages.back().brakePercent == 0,
                          "changing centimetre noise on straight geometry never shakes the 5m/s command");
                } else {
                    const double nominal = row[1];
                    Check(limit > 0 && limit <= nominal * 1.03,
                          "noisy left/right turns cannot increase the ideal circle speed limit");
                    if(radius <= 25) {
                        Check(limit >= nominal * 0.90,
                              "reference and tighter turns retain the existing noise-bias bound");
                    }
                    Check(messages.back().brakePercent == 0, "curve-only limits preserve planning-triggered brake gating");
                }
            }
            std::cout << "NOISE_LIMIT " << std::setprecision(9) << radius << ' ' << sign << ' '
                      << lowest << ' ' << highest << ' ' << lowest_oil << ' ' << highest_oil << '\n';
            if(radius > 0) {
                // 过渡区原公式已有噪声峰值偏置，不用理想圆弧的10%静态偏差替代跨帧抖动检查。
                // 保留原急弯约束；新增过渡区约束为整组给定波动<2%、整数指令最多1个百分点。
                Check(highest - lowest < (radius <= 25 ? 0.04 : 0.02 * lowest) && highest_oil - lowest_oil <= 1,
                      "turn noise cannot create large consecutive speed-command oscillations");
            }
        }
    }

    // 用户追问的90度转弯、180度掉头：同半径同限速，均从真实坐标经过完整消息入口。
    const double circle_calibration[][2] = {
        {5, 0.626099034}, {8, 0.791959595}, {10, 0.885437745}, {15, 1.084435337},
        {20, 1.252198067}, {25, 1.4}, {100.0 / 3.0, 1.713518577},
        {40, 2.023857703}, {50, 2.426860311}, {80, 4.288396501}, {100, 5.377777778}
    };
    for(const auto& row : circle_calibration) {
        const double radius = row[0];
        for(double angle : {M_PI / 2.0, M_PI}) {
            for(double sign : {-1.0, 1.0}) {
                input = Prepare();
                input.plan.x.clear();
                input.plan.y.clear();
                const int points = static_cast<int>(std::ceil(radius * angle / 0.2));
                for(int j = -5; j <= points; ++j) {
                    const double theta = std::min(angle, j * 0.2 / radius);
                    input.plan.x.push_back(radius * std::sin(theta));
                    input.plan.y.push_back(0.2 + sign * radius * (1.0 - std::cos(theta)));
                }
                control.SetPathPlanData(input.plan);
                control.VehiclePoseCalculation();
                ControlComply snapshot = control;
                const double limit = Limit(snapshot, snapshot.mPathList);
                Check(std::fabs(limit / row[1] - 1.0) < 0.015,
                      "90-degree turns and 180-degree U-turns follow the same radius speed table");
            }
        }
    }

    // 入弯时真正的曲率仍必须提前降低给定，不用延迟降速的时间滤波掩盖直线抖动。
    for(double radius : {5.0, 10.0}) {
        for(double sign : {-1.0, 1.0}) {
            input = Prepare();
            input.plan.x.clear();
            input.plan.y.clear();
            for(int j = -5; j <= 150; ++j) {
                const double s = j * 0.2;
                const double angle = std::max(0.0, s - 10.0) / radius;
                input.plan.x.push_back(s <= 10.0 ? s : 10.0 + radius * std::sin(angle));
                input.plan.y.push_back(0.2 + sign * radius * (1.0 - std::cos(angle)));
            }
            control.SetPathPlanData(input.plan);
            for(double distance : {10.0, 6.0, 4.0, 2.0, 0.0}) {
                robot::navigation_msg nav = control.mNavData;
                nav.gpsSpeed = 1.5;  // 原6m空间断言对应当前1.5m/s；动态距离另有专项。
                nav.xAxis = 10.0 - distance;
                nav.yAxis = 0.2;
                control.SetNavigationData(nav);
                control.VehiclePoseCalculation();
                ControlComply snapshot = control;
                const double limit = Limit(snapshot, snapshot.mPathList);
                if(distance == 10.0) Check(limit >= 3.0, "distant bend does not restrict the straight planning speed");
                if(distance == 6.0) Check(limit < 2.0, "real left/right turn starts limiting while still 6m away");
                if(distance <= 4.0) {
                    Check(std::fabs(limit / (0.28 * std::sqrt(radius)) - 1.0) < 0.03,
                          "full bend limit is established before reaching the turn");
                }
            }
        }
    }

    input = Prepare();
    const auto original = control.mPathList;
    control.CalcuForwardPathCurve();
    Check(control.mForwardPathList.size() == original.size(), "D geometry cache covers exactly the current path");
    for(std::size_t i = 0; i < original.size(); ++i) {
        Check(control.mPathList[i].curvature == original[i].curvature &&
              control.mForwardPathList[i].x_axis == original[i].x_axis &&
              control.mForwardPathList[i].y_axis == original[i].y_axis,
              "speed curvature generation preserves lateral curvature and every original coordinate");
    }
    input.can.curGear = GEAR_R;
    control.SetCanData(input.can);
    Check(control.mForwardPathList.empty() && control.mForwardPathDistances.empty(), "leaving D invalidates its geometry cache");
    input.can.curGear = GEAR_D;
    control.SetCanData(input.can);
    Check(control.mForwardPathList.size() == original.size(), "returning to D rebuilds geometry without requiring a new plan");
    control.mPathList[20].x_axis = std::numeric_limits<double>::quiet_NaN();
    control.CalcuForwardPathCurve();
    Check(control.mForwardPathList.empty(), "invalid path geometry invalidates the entire speed cache");
}

void BenchmarkGeometry(std::size_t count) {
    if(!std::freopen("/dev/null", "w", stdout)) std::exit(2);
    control.mPathList = Path(0.04, count);
    for(std::size_t i = 0; i < count; ++i) {
        const double angle = i * 0.1 / 25.0;
        control.mPathList[i].x_axis = 25.0 * std::sin(angle);
        control.mPathList[i].y_axis = 25.0 * (1.0 - std::cos(angle));
    }
    for(int i = 0; i < 1000; ++i) control.CalcuForwardPathCurve();
    std::vector<double> times;
    times.reserve(10000);
    allocations = 0;
    count_allocations = true;
    for(int i = 0; i < 10000; ++i) {
        const auto start = std::chrono::steady_clock::now();
        control.CalcuForwardPathCurve();
        const auto stop = std::chrono::steady_clock::now();
        times.push_back(std::chrono::duration<double, std::micro>(stop - start).count());
    }
    count_allocations = false;
    std::sort(times.begin(), times.end());
    struct rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    std::cerr << std::setprecision(9) << "GEOMETRY_BENCH {\"points\":" << count
              << ",\"p50_us\":" << times[5000] << ",\"p99_us\":" << times[9900]
              << ",\"max_us\":" << times.back() << ",\"cpp_allocations\":" << allocations
              << ",\"cache_bytes\":" << control.mForwardPathList.capacity() * sizeof(XYZ_COOR_S) +
                  control.mForwardPathDistances.capacity() * sizeof(double)
              << ",\"rss_kib\":" << usage.ru_maxrss << "}\n";
}
#endif

void Benchmark(std::size_t count, double speed) {
    if(!std::freopen("/dev/null", "w", stdout)) std::exit(2);
    const auto path = Path(0.04, count);
    robot::navigation_msg nav;
    nav.gpsSpeed = speed;
    control.SetNavigationData(nav);
    volatile double sink = 0;
    for(int i = 0; i < 1000; ++i) sink = Limit(control, path);
    std::vector<double> times;
    times.reserve(10000);
    allocations = 0;
    count_allocations = true;
    for(int i = 0; i < 10000; ++i) {
        const auto start = std::chrono::steady_clock::now();
        sink = Limit(control, path);
        const auto stop = std::chrono::steady_clock::now();
        times.push_back(std::chrono::duration<double, std::micro>(stop - start).count());
    }
    count_allocations = false;
    std::sort(times.begin(), times.end());
    struct rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    std::cerr << std::setprecision(9) << "BENCH {\"points\":" << count
              << ",\"speed_mps\":" << speed
              << ",\"p50_us\":" << times[5000] << ",\"p99_us\":" << times[9900]
              << ",\"max_us\":" << times.back() << ",\"cpp_allocations\":" << allocations
              << ",\"controller_bytes\":" << sizeof(ControlComply) << ",\"rss_kib\":" << usage.ru_maxrss
              << ",\"last_limit\":" << sink << "}\n";
}

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "check";
    if(mode == "benchmark" && (argc == 3 || argc == 4)) {
        const int count = std::atoi(argv[2]);
        if(count < 1) return 2;
        Benchmark(count, argc == 4 ? std::atof(argv[3]) : 1.5);
    } else if(mode == "preview_trace") {
        for(int scene = 0; scene < 4; ++scene) {
            auto path = Path(0, 161);
            for(std::size_t i = 0; i < path.size(); ++i) {
                path[i].x_axis = i * 0.25;
                if(scene == 0) path[i].curvature = i * 0.0005;
                if(scene == 1 && i % 37 == 0) path[i].curvature = -0.04;
                if(scene == 2 && i >= 80) path[i].curvature = 0.1;
                if(scene == 3) path[i].curvature = (i % 19) * 0.003;
            }
            for(double position : {0.0, 1.125, 35.125}) {
                for(double speed : {0.0, 0.05, 0.5, 1.0, 1.5, 3.0, 4.5, 5.0}) {
                    ControlComply target;
                    robot::navigation_msg nav;
                    nav.xAxis = position;
                    nav.gpsSpeed = speed;
                    target.SetNavigationData(nav);
                    target.mKeyPoint = static_cast<int>(position / 0.25);
                    std::cout << "PREVIEW " << std::setprecision(17) << scene << ' ' << position << ' '
                              << speed << ' ' << Limit(target, path) << '\n';
                }
            }
        }
    } else if(mode == "calibration_trace") {
        Prepare();
        for(int i = 0; i <= 4900; ++i) {
            const double curvature = i <= 1000 ? i * 0.00001 : 0.01 + (i - 1000) * 0.0001;
#ifndef CURVE_HISTORY_LEGACY_BASELINE
            control.ResetForwardCurveHistory();
#endif
            std::cout << "CALIBRATION " << std::setprecision(17) << curvature << ' '
                      << Limit(control, Path(curvature)) << '\n';
        }
    } else if(mode == "history_trace") {
        Inputs input = Prepare();
        Seed(control);
        for(int i = 0; i < 90; ++i) {
#ifndef CURVE_HISTORY_LEGACY_BASELINE
            // 生产数值转储已移除；在副本上读取本次限速，不多推进真实测试对象的历史。
            ControlComply snapshot = control;
            std::cout << "HISTORY_LIMIT " << std::setprecision(9) << Limit(snapshot, snapshot.mPathList) << '\n';
#endif
            Trace(Tick(input));
        }
    } else if(mode == "mixed_reverse_trace") {
        for(int round = 0; round < 2; ++round) {
            Inputs forward = Prepare(GEAR_D, 0.01, 100 + round * 2);
            Seed(control);
            for(int i = 0; i < 5; ++i) Tick(forward);
            Inputs reverse = Prepare(GEAR_R, round == 0 ? 0.01 : -0.01, 101 + round * 2);
            for(int i = 0; i < 20; ++i) Trace(Tick(reverse));
        }
    } else if(mode == "reverse_trace") {
        int id = 1;
        for(double curve : {-0.01, 0.0, 0.01}) {
            Inputs input = Prepare(GEAR_R, curve, id++);
            for(int i = 0; i < 40; ++i) {
                if(i == 10) {
                    input.plan.desireSpeed = 0.5;
                    control.SetPathPlanData(input.plan);
                }
                if(i == 20) {
                    input.plan.safety = true;
                    input.plan.desireSpeed = 0;
                    control.SetPathPlanData(input.plan);
                }
                if(i == 30) {
                    input.plan.safety = false;
                    input.plan.desireSpeed = 3;
                    control.SetPathPlanData(input.plan);
                }
                Trace(Tick(input));
            }
        }
#ifndef CURVE_HISTORY_BASELINE
    } else if(mode == "geometry_benchmark" && argc == 3) {
        const int count = std::atoi(argv[2]);
        if(count < 3) return 2;
        BenchmarkGeometry(count);
    } else if(mode == "check") {
        independent_curve_cases = true;
        TestHistory();
        TestPreview();
        TestSpeedGap();
        TestDynamicPreview();
        TestCurveNoise();
        std::cout << "PASS curve history: " << checks << " checks\n";
#endif
    } else {
        return 2;
    }
    return 0;
}
