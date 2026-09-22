// 规划下降沿的输入/发布回归，不读取积分私有状态。
#define main ExistingControlTestsMain
#include "launch_speed.cpp"
#undef main

int request_checks = 0;
std::string request_scene;

void CheckRequest(bool condition, const char* message) {
    if(!condition) {
        std::cerr << "FAIL " << request_scene << ": " << message << '\n';
        std::exit(1);
    }
    ++request_checks;
}

void StartRequestTest(ControlComply& target, double desired = 2, double actual = 2) {
    ros::testTime() = 100;
    ros::param::set("/planning/sensorstate", 0);
    ros::param::set("/robot/control/accswitch", 0);
    ros::param::set("/robot/control/brake_integral_threshold", 0.1);
    can_data = robot::can_msg();
    navigation = robot::navigation_msg();
    status = robot::path_plan_status();
    SetUp(target, desired, actual, GEAR_D);
    navigation.yAxis = 0.2;  // 避开既有横向几何精确零偏差的 NaN 边界。
    target.SetNavigationData(navigation);
}

robot::control_msg RequestStep(ControlComply& target, double actual) {
    ros::testTime() += 0.05;
    can_data.vehicleSpeed = 99;  // 有意冲突，普通减速必须仍然使用导航实际车速。
    navigation.gpsSpeed = actual;
    target.SetCanData(can_data);
    target.SetNavigationData(navigation);
    const auto output = Tick(target);
    CheckRequest(output.size() == 1, "one control publication per cycle");
    CheckRequest(output.front().vehicleSpeed == navigation.gpsSpeed, "navigation speed feedback");
    CheckRequest(output.front().throttlePercent == 0 || output.front().brakePercent == 0,
                 "drive and brake remain mutually exclusive");
    return output.front();
}

void HoldRequest(ControlComply& target, double actual, int ticks, int brake) {
    for(int i = 0; i < ticks; ++i) {
        target.SetPathPlanData(plan);
        CheckRequest(RequestStep(target, actual).brakePercent == brake, "expected brake command");
    }
}

void ArmRequest(ControlComply& target) {
    RequestDeceleration(target);
    HoldRequest(target, 2, 3, 0);
    HoldRequest(target, 2, 1, 15);
}

int main() {
    for(const std::string scene : {"first_overspeed", "small_overspeed", "oscillating", "increasing"}) {
        request_scene = scene;
        ControlComply target;
        StartRequestTest(target);
        for(int i = 0; i < 260; ++i) {
            if(scene == "increasing") plan.desireSpeed = 2 + i * 0.001;
            target.SetPathPlanData(plan);
            const double actual = scene == "small_overspeed" ? 2.01 :
                                  (scene == "oscillating" && i % 30 >= 25 ? 1.99 : 3.0);
            CheckRequest(RequestStep(target, actual).brakePercent == 0,
                         "overspeed without planning decline never starts integral braking");
        }
    }
    {
        request_scene = "decline_then_hold";
        ControlComply target;
        StartRequestTest(target, 1, 2);
        ArmRequest(target);
        HoldRequest(target, 2, 30, 15);
    }
    {
        request_scene = "successive_declines";
        ControlComply target;
        StartRequestTest(target, 2.5, 3);
        plan.desireSpeed = 2;
        HoldRequest(target, 3, 1, 0);
        plan.desireSpeed = 1.9;
        HoldRequest(target, 3, 1, 0);
        plan.desireSpeed = 1.8;
        HoldRequest(target, 3, 1, 18);  // 0.055 + 0.060m，连续下降不清零已有积分。
    }
    // 检查真实发布的力度、整数最小值及100%限幅；不读取内部积分标志。
    const double brake_calibration[][2] = {
        {0.01, 1}, {0.2, 3}, {0.5, 8}, {0.6, 9}, {1.0, 15},
        {1.2, 18}, {2.0, 30}, {3.4, 51}, {7.0, 100}
    };
    for(const auto& row : brake_calibration) {
        request_scene = "reduced_brake_gain";
        ControlComply target;
        StartRequestTest(target, 1, 1 + row[0]);
        RequestDeceleration(target);
        CheckRequest(RequestStep(target, 1 + row[0]).brakePercent == 0,
                     "reduced brake still waits for positive integrated error");
        robot::control_msg last;
        for(int i = 0; i < 240; ++i) {
            last = RequestStep(target, 1 + row[0]);
            CheckRequest(last.brakePercent == 0 || last.brakePercent == row[1],
                         "supplemental brake applies the reduced gain without accumulating output");
        }
        CheckRequest(last.brakePercent == row[1] && last.throttlePercent == 0,
                     "persistent deceleration reaches the requested reduced brake strength");
    }
    for(const std::string scene : {"reached", "underspeed", "target_rise", "callbacks_rise",
                                  "task_id", "task_type", "task_gear", "task_route", "path_id",
                                  "manual", "neutral", "reverse", "can_estop", "safety",
                                  "gnss", "lateral", "short_path", "mismatched_path", "filtered_path"}) {
        request_scene = scene;
        ControlComply target;
        StartRequestTest(target, 1, 2);
        ArmRequest(target);
        if(scene == "reached" || scene == "underspeed") {
            CheckRequest(RequestStep(target, scene == "reached" ? 1 : 0.9).brakePercent == 0,
                         "reaching planning target releases the request");
        } else if(scene == "target_rise" || scene == "callbacks_rise") {
            if(scene == "callbacks_rise") {
                plan.desireSpeed = 0.5;
                target.SetPathPlanData(plan);
            }
            plan.desireSpeed = 1.5;
            target.SetPathPlanData(plan);
        } else if(scene.compare(0, 5, "task_") == 0) {
            robot::task_plan_msg task;
            if(scene == "task_id") task.task_id = 1;
            if(scene == "task_type") task.taskType = TRACKPATH;
            if(scene == "task_gear") task.desireGear = GEAR_D;
            if(scene == "task_route") task.pathList.push_back("different_route");
            target.setTaskPlanData(task);
        } else if(scene == "path_id") {
            ++plan.Path_Id;
            plan.desireSpeed = 0.5;
            target.SetPathPlanData(plan);
        } else if(scene == "manual" || scene == "neutral" || scene == "reverse" || scene == "can_estop") {
            if(scene == "manual") can_data.controlPanelState = 0;
            if(scene == "neutral") can_data.curGear = GEAR_N;
            if(scene == "reverse") can_data.curGear = GEAR_R;
            if(scene == "can_estop") can_data.emergencyStop = 1;
            target.SetCanData(can_data);
            can_data.controlPanelState = 1;
            can_data.curGear = GEAR_D;
            can_data.emergencyStop = 0;
            target.SetCanData(can_data);  // 一个控制周期内的状态变化也必须复位。
        } else if(scene == "safety" || scene == "gnss" || scene == "lateral") {
            if(scene == "safety") {
                plan.safety = true;
                target.SetPathPlanData(plan);
            }
            if(scene == "gnss") ros::param::set("/planning/sensorstate", 8);
            if(scene == "lateral") navigation.yAxis = 6;
            CheckRequest(RequestStep(target, 2).brakePercent == (scene == "lateral" ? 70 : 100),
                         "independent safety braking retains its priority");
            plan.safety = false;
            ros::param::set("/planning/sensorstate", 0);
            navigation.yAxis = 0.2;
        } else {
            robot::path_plan_msg invalid = plan;
            invalid.desireSpeed = 0.5;
            if(scene == "short_path") {
                invalid.x.resize(4);
                invalid.y.resize(4);
            } else if(scene == "mismatched_path") {
                invalid.y.pop_back();
            } else {
                invalid.x.assign(20, 0);
                invalid.y.assign(20, 0);
            }
            target.SetPathPlanData(invalid);
        }
        // 无新下降沿时，持续超速/重复消息均不能恢复上一次积分。
        HoldRequest(target, 2, 20, 0);
        plan.desireSpeed -= 0.25;
        target.SetPathPlanData(plan);
        CheckRequest(RequestStep(target, 2).brakePercent == 0, "new decline starts from zero integral");
        robot::control_msg last;
        for(int i = 0; i < 10; ++i) last = RequestStep(target, 2);
        CheckRequest(last.brakePercent > 0 && last.throttlePercent == 0,
                     "a later real decline can start a new deceleration request");
    }
    {
        request_scene = "decline_while_below_target";
        ControlComply target;
        StartRequestTest(target, 3, 1);
        plan.desireSpeed = 2;
        HoldRequest(target, 1, 1, 0);
        HoldRequest(target, 3, 30, 0);
    }
    {
        request_scene = "local_sensor_limit";
        ControlComply target;
        StartRequestTest(target, 2, 2);
        ros::param::set("/planning/sensorstate", 2);
        HoldRequest(target, 2, 30, 0);
        ros::param::set("/planning/sensorstate", 8);
        HoldRequest(target, 2, 1, 100);
    }
    for(bool decline : {false, true}) {
        request_scene = decline ? "reached_target_under_curve_limit" : "local_curve_limit";
        ControlComply target;
        StartRequestTest(target, decline ? 3 : 2, 2);
        plan.desireSpeed = 2;
        plan.x.clear();
        plan.y.clear();
        for(int i = 0; i <= 100; ++i) {
            const double x = i * 0.2;
            plan.x.push_back(x);
            plan.y.push_back(0.05 * x * x);
        }
        target.SetPathPlanData(plan);
        const auto first = RequestStep(target, 2);
        CheckRequest(first.throttlePercent < 30 && first.brakePercent == 0,
                     "curve lowers motor speed without inventing a planning deceleration request");
        HoldRequest(target, decline ? 3 : 2, 40, 0);
    }
    {
        request_scene = "route_change_before_first_control_tick";
        ControlComply target;
        StartRequestTest(target, 1, 2);
        RequestDeceleration(target);
        robot::task_plan_msg task;
        task.pathList.push_back("new_route");
        target.setTaskPlanData(task);
        HoldRequest(target, 2, 20, 0);
    }
    std::cout << "PASS planning brake request: " << request_checks << " checks\n";
}
