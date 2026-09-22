// 实际 ControlComply 输入/输出回归:每个场景使用独立进程,不访问私有状态。
#include "control_comply.h"
#include <cstdlib>
#include <iomanip>

ControlComply control;
ControlComply second_control;
robot::can_msg can_data;
robot::navigation_msg navigation;
robot::path_plan_msg plan;
robot::path_plan_status status;
ros::Publisher publisher;

void SetVehicleFeedback(ControlComply& target) {
    target.SetCanData(can_data);
    navigation.gpsSpeed = can_data.vehicleSpeed;
    target.SetNavigationData(navigation);
}

void SetUp(ControlComply& target, double desired, double actual, int gear,
           double x = 0.0, bool with_path = true) {
    navigation.xAxis = x;
    navigation.yAxis = 0.0;
    navigation.heading = 90.0;
    navigation.gpsSpeed = actual;
    target.SetNavigationData(navigation);
    can_data.curGear = gear;
    can_data.vehicleSpeed = actual;
    can_data.wheelAngle = 0.0;
    can_data.controlPanelState = 1;
    SetVehicleFeedback(target);
    status.taskExecuStatus = 1;
    target.SetPathStatusData(status);
    robot::task_plan_msg task;
    target.setTaskPlanData(task);
    plan.Path_Id = 1;
    plan.desireSpeed = desired;
    plan.safety = false;
    plan.x.clear();
    plan.y.clear();
    if(with_path) {
        for(int i = 0; i <= 40; ++i) {
            plan.x.push_back(x + i);
            plan.y.push_back(0.0f);
        }
    }
    target.SetPathPlanData(plan);
}

void RequestDeceleration(ControlComply& target) {
    // 明确发送规划下降沿；仅设置较高实车速度不能代替规划减速请求。
    robot::path_plan_msg previous = plan;
    previous.desireSpeed += 1.0;
    target.SetPathPlanData(previous);
    target.SetPathPlanData(plan);
}

std::vector<robot::control_msg> Tick(ControlComply& target = control) {
    std::vector<robot::control_msg> messages;
    publisher.capture = [&](const std::type_info& type, const void* raw) {
        if(type == typeid(robot::control_msg)) {
            messages.push_back(*static_cast<const robot::control_msg*>(raw));
        }
    };
    target.VehicleControl();
    target.PublishMessage(publisher);
    for(size_t i = 0; i < messages.size(); ++i) {
        const robot::control_msg& msg = messages[i];
        std::cout << "TRACE " << +msg.throttlePercent << ' ' << +msg.brakePercent
                  << ' ' << msg.desireSpeed << ' ' << msg.wheelAngle
                  << ' ' << msg.biaDistance << ' ' << msg.biaAngle
                  << ' ' << msg.preCurve << ' ' << msg.preAngleDev << ' ' << msg.desireAcc
                  << ' ' << msg.vehicleSpeed << ' ' << +msg.remoteEnable << ' ' << +msg.bypassProcessing
                  << ' ' << msg.faultCode.size();
        for(const auto code : msg.faultCode) std::cout << ' ' << +code;
        std::cout << '\n';
    }
    return messages;
}

int main(int argc, char** argv) {
    if(argc < 2) return 2;
    const std::string scenario = argv[1];
    const double value = argc > 2 ? std::atof(argv[2]) : 0.3;
    ros::param::set("/planning/sensorstate", 0);
    ros::param::set("/robot/control/accswitch", 0);
    publisher.name = "/control_msg";
    std::cout << std::setprecision(9);

    if(scenario.compare(0, 15, "reverse_compat_") == 0) {
        const double origin = scenario == "reverse_compat_traffic1" ? 1555.0 :
                              (scenario == "reverse_compat_traffic2" ? 428.0 : 0.0);
        ros::testTime() = 100;
        SetUp(control, scenario == "reverse_compat_launch" ? value : 1.0,
              scenario == "reverse_compat_launch" ? 0 : 1.0 + value, GEAR_R,
              origin, scenario != "reverse_compat_empty");
        plan.Path_Id = 20;
        if(scenario == "reverse_compat_safety" || scenario == "reverse_compat_safety_zero" ||
           scenario == "reverse_compat_safety_path5") plan.safety = true;
        if(scenario == "reverse_compat_safety_path5") plan.Path_Id = 5;
        if(scenario == "reverse_compat_safety_zero" || scenario == "reverse_compat_terminal") plan.desireSpeed = 0;
        control.SetPathPlanData(plan);
        if(scenario == "reverse_compat_terminal") {
            status.taskExecuStatus = TASKFINISHED;
            control.SetPathStatusData(status);
        }
        if(scenario == "reverse_compat_gnss" || scenario == "reverse_compat_sensor_slow") {
            ros::param::set("/planning/sensorstate", scenario == "reverse_compat_gnss" ? 8 : 2);
        }
        if(scenario == "reverse_compat_lateral") {
            navigation.yAxis = 6;
            control.SetNavigationData(navigation);
        }
        if(scenario == "reverse_compat_traffic1" || scenario == "reverse_compat_traffic2") {
            robot::TLStatus light;
            light.light_status = 0;
            control.SetTlStatusData(light);
        }
        if(scenario == "reverse_compat_manual") can_data.controlPanelState = 0;
        if(scenario == "reverse_compat_acc") ros::param::set("/robot/control/accswitch", 1);
        for(int i = 0; i < 30; ++i) {
            ros::testTime() = 100 + 0.05 * i;
            SetVehicleFeedback(control);
            Tick();
        }
        return 0;
    }

    if(scenario.compare(0, 9, "integral_") == 0) {
        ros::testTime() = 100.0;
        SetUp(control, 1.0, scenario == "integral_small" ? 1.01 : 2.0, GEAR_D);
        RequestDeceleration(control);
        if(scenario == "integral_threshold") {
            ros::param::set("/robot/control/brake_integral_threshold", value);
        }
        std::vector<double> times;
        if(scenario == "integral_period") {
            for(int i = 0; i <= static_cast<int>(0.5 / value); ++i) times.push_back(i * value);
        } else if(scenario == "integral_irregular") {
            times = {0, 0.02, 0.07, 0.1, 0.11};
        } else if(scenario == "integral_gap") {
            times = {0, 0.05, 1.0, 1.05, 1.1, 1.15};
        } else if(scenario == "integral_clockback") {
            times = {0, 0.05, -1.0, -0.95, -0.9, -0.85};
        } else if(scenario == "integral_duplicate") {
            times = {0, 0.05};
            times.insert(times.end(), 30, 0.05);
            times.push_back(0.1);
            times.push_back(0.15);
        } else if(scenario == "integral_stale_pending") {
            times = {0, 0.25, 0.3, 0.35, 0.4};
        } else {
            const int count = scenario == "integral_small" ? 240 :
                              (scenario == "integral_saturation" ? 200 : 12);
            for(int i = 0; i < count; ++i) times.push_back(i * 0.05);
        }
        for(std::size_t i = 0; i < times.size(); ++i) {
            ros::testTime() = 100 + times[i];
            if(scenario.compare(0, 15, "integral_reset_") == 0) {
                can_data.curGear = i == 4 && scenario == "integral_reset_neutral" ? GEAR_N :
                                   (i == 4 && scenario == "integral_reset_reverse" ? GEAR_R : GEAR_D);
                can_data.controlPanelState = i == 4 && scenario == "integral_reset_manual" ? 0 : 1;
                plan.safety = i == 4 && scenario == "integral_reset_safety";
                plan.desireSpeed = i == 4 && scenario == "integral_reset_equal" ? 2 :
                                   (i == 4 && scenario == "integral_reset_accelerate" ? 3 : 1);
                control.SetPathPlanData(plan);
                ros::param::set("/planning/sensorstate", i == 4 && scenario == "integral_reset_gnss" ? 8 : 0);
                navigation.yAxis = i == 4 && scenario == "integral_reset_lateral" ? 6 : 0;
                control.SetNavigationData(navigation);
                if(scenario == "integral_reset_task") {
                    robot::task_plan_msg task;
                    task.task_id = i >= 4 ? 1 : 0;
                    control.setTaskPlanData(task);
                }
            }
            if(scenario == "integral_pulses") {
                // 两段各 0.05m 的短暂误差，中间恢复目标速度，积分不得跨段叠加。
                can_data.vehicleSpeed = i == 2 || i == 5 ? 1 : 2;
            }
            if(scenario == "integral_command_repeat") {
                control.SetPathPlanData(plan);
                robot::task_plan_msg task;
                control.setTaskPlanData(task);
            }
            const bool stale = scenario == "integral_stale_pending" || scenario == "integral_stale_active";
            if(!stale || i <= (scenario == "integral_stale_pending" ? 0U : 3U)) {
                SetVehicleFeedback(control);
            }
            if((scenario == "integral_pulses" && (i == 3 || i == 6)) ||
               (scenario.compare(0, 15, "integral_reset_") == 0 &&
                i == (scenario == "integral_reset_task" ? 4U : 5U))) {
                RequestDeceleration(control);
            }
            Tick();
        }
        if(scenario == "integral_instances") {
            SetUp(second_control, 1, 2, GEAR_D);
            Tick(second_control);
        }
        if(scenario == "integral_saturation") {
            can_data.vehicleSpeed = 1;
            ros::testTime() += 0.05;
            SetVehicleFeedback(control);
            Tick();
            can_data.vehicleSpeed = 2;
            for(int i = 0; i < 4; ++i) {
                ros::testTime() += 0.05;
                SetVehicleFeedback(control);
                if(i == 0) RequestDeceleration(control);
                Tick();
            }
        }
        return 0;
    }

    if(scenario.compare(0, 12, "deceleration") == 0) {
        SetUp(control, 1.0, 1.0 + value, scenario == "deceleration_reverse" ? GEAR_R : GEAR_D);
        if(scenario == "deceleration_fractional") SetUp(control, 0.4, 1.0, GEAR_D);
        if(can_data.curGear == GEAR_D) RequestDeceleration(control);
        if(scenario == "deceleration_safety" || scenario == "deceleration_safety_reverse") {
            plan.safety = true;
            if(scenario == "deceleration_safety_reverse") {
                plan.Path_Id = 5;
                can_data.curGear = GEAR_R;
                SetVehicleFeedback(control);
            }
            control.SetPathPlanData(plan);
        }
        if(scenario == "deceleration_gnss") ros::param::set("/planning/sensorstate", 8);
        if(scenario == "deceleration_lateral") {
            navigation.yAxis = 6;
            control.SetNavigationData(navigation);
        }
        const int count = scenario == "deceleration" || scenario == "deceleration_fractional" ? 60 : 5;
        for(int i = 0; i < count; ++i) {
            ros::testTime() = 100.0 + 0.05 * i;
            SetVehicleFeedback(control);
            Tick();
        }
        const auto advance = [&]() {
            ros::testTime() += 0.05;
            SetVehicleFeedback(control);
            Tick();
        };
        if(scenario == "deceleration_release") {
            can_data.vehicleSpeed = 1.0;
            advance();
            can_data.vehicleSpeed = 1.5;
            RequestDeceleration(control);
            advance();
            can_data.vehicleSpeed = 1.6;
            for(int i = 0; i < 4; ++i) advance();
            plan.safety = true;
            control.SetPathPlanData(plan);
            advance();
            plan.safety = false;
            control.SetPathPlanData(plan);
            can_data.vehicleSpeed = 0;
            for(int i = 0; i < 8; ++i) advance();
        }
        return 0;
    }

    if(scenario == "planning_acceleration") {
        ros::testTime() = 100.0;
        SetUp(control, 0.0, 0.0, GEAR_D);
        for(double desired : {0.0, 0.2, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5}) {
            ros::testTime() += 1.0;
            can_data.vehicleSpeed = std::max(0.0, desired - 0.2);
            SetVehicleFeedback(control);
            plan.desireSpeed = desired;
            control.SetPathPlanData(plan);
            Tick();
        }
        return 0;
    }
    if(scenario == "steady" || scenario == "reverse" || scenario == "low_target") {
        SetUp(control, value, scenario == "low_target" ? 0.0 : value,
              scenario == "reverse" ? GEAR_R : GEAR_D);
        const int count = scenario == "low_target" ? 80 : 1;
        for(int i = 0; i < count; ++i) Tick();
        return 0;
    }
    if(scenario == "slope") {
        ros::param::set("/robot/control/launch_speed_slope", value);
    } else if(scenario == "old_params") {
        ros::param::set("/robot/control/throttle_slope", 0.0);
        ros::param::set("/robot/control/launch_speed_gap", 0.01);
        ros::param::set("/robot/control/launch_min_throttle", 90.0);
    }
    if(scenario.compare(0, 9, "terminal_") == 0) {
        ros::testTime() = 100.0;
        ros::param::set("/planning/alive", 1);
        ros::param::set("/robot/planning/netcheck", 0);
        SetUp(control, 0.0, 0.3, GEAR_D);
        robot::task_plan_msg task;
        task.taskType = TRACKPATH;
        task.desireGear = GEAR_D;
        control.setTaskPlanData(task);
        status.taskExecuStatus = TASKFINISHED;
        control.SetPathStatusData(status);
        for(int i = 0; i < 20; ++i) {
            ros::testTime() = 100.0 + 0.05 * i;
            if(scenario == "terminal_gap" && i >= 4) ros::testTime() += 0.5;
            if(scenario == "terminal_clockback" && i >= 4) ros::testTime() -= 1.0;
            can_data.vehicleSpeed = scenario == "terminal_fast" ? 0.6 : 0.3;
            if(scenario == "terminal_value") can_data.vehicleSpeed = value;
            if(scenario == "terminal_stopped" && i >= 4) can_data.vehicleSpeed = 0.02;
            if(scenario == "terminal_reverse") can_data.curGear = GEAR_R;
            if(scenario == "terminal_can_estop" && i >= 4) can_data.emergencyStop = 1;
            if(scenario != "terminal_stale" || i == 0) SetVehicleFeedback(control);
            if(scenario == "terminal_safety" && i >= 4) {
                plan.safety = true;
                control.SetPathPlanData(plan);
            }
            if(scenario == "terminal_hold") {
                plan.safety = i == 0;
                control.SetPathPlanData(plan);
            }
            if(scenario == "terminal_sensor" && i >= 4) ros::param::set("/planning/sensorstate", 8);
            if(scenario == "terminal_network" && i >= 4) ros::param::set("/robot/planning/netcheck", 1);
            if(scenario == "terminal_dead" && i >= 4) ros::param::set("/planning/alive", 0);
            if(scenario == "terminal_other_task") {
                task.taskType = ADAPTIVEHOOK;
                control.setTaskPlanData(task);
            }
            Tick();
        }
        SetUp(control, 2.0, 0.0, GEAR_D);
        for(int i = 0; i < 8; ++i) Tick();
        return 0;
    }
    double origin = scenario == "traffic1" ? 1555.0 :
                    (scenario == "traffic2" ? 428.0 : 0.0);
    SetUp(control, 2.0, 0.0, GEAR_D, origin, scenario != "empty");

    if(scenario == "feedback") {
        for(int i = 0; i < 320; ++i) {
            const std::vector<robot::control_msg> messages = Tick();
            can_data.vehicleSpeed = messages.back().throttlePercent / 15.0;
            SetVehicleFeedback(control);
        }
        return 0;
    }
    if(scenario == "default" || scenario == "slope" || scenario == "old_params") {
        for(int i = 0; i < 40; ++i) Tick();
        return 0;
    }
    if(scenario == "empty") {
        Tick();
        SetUp(control, 2.0, 0.0, GEAR_D);
        for(int i = 0; i < 8; ++i) Tick();
        return 0;
    }

    // 先建立正常驱动,再触发停车/降速,检查安全覆盖和恢复输出。
    for(int i = 0; i < 40; ++i) Tick();
    std::cout << "PHASE after_warmup\n";
    if(scenario == "neutral" || scenario == "gear_reverse") {
        can_data.curGear = scenario == "neutral" ? GEAR_N : GEAR_R;
        SetVehicleFeedback(control);
    } else if(scenario == "path_safety") {
        plan.Path_Id = 5;
        plan.safety = true;
        control.SetPathPlanData(plan);
    } else if(scenario == "safety_zero" || scenario == "zero") {
        plan.desireSpeed = 0.0;
        plan.safety = scenario == "safety_zero";
        control.SetPathPlanData(plan);
    } else if(scenario == "task_stop") {
        plan.desireSpeed = 0.4;
        control.SetPathPlanData(plan);
        status.taskExecuStatus = 2;
        control.SetPathStatusData(status);
    } else if(scenario == "gnss" || scenario == "sensor_slow") {
        ros::param::set("/planning/sensorstate", scenario == "gnss" ? 8 : 2);
    } else if(scenario == "lateral") {
        navigation.yAxis = 6.0;
        control.SetNavigationData(navigation);
    } else if(scenario == "traffic1" || scenario == "traffic2") {
        robot::TLStatus light;
        light.light_status = 0;
        control.SetTlStatusData(light);
    } else if(scenario == "decrease") {
        plan.desireSpeed = 0.04;
        control.SetPathPlanData(plan);
    } else if(scenario == "instances") {
        SetUp(second_control, 2.0, 0.0, GEAR_D);
        for(int i = 0; i < 8; ++i) Tick(second_control);
        return 0;
    } else {
        return 2;
    }
    Tick();
    if(scenario == "decrease") return 0;

    can_data.curGear = GEAR_D;
    SetVehicleFeedback(control);
    plan.desireSpeed = 2.0;
    plan.Path_Id = 1;
    plan.safety = false;
    control.SetPathPlanData(plan);
    status.taskExecuStatus = 1;
    control.SetPathStatusData(status);
    navigation.yAxis = 0.0;
    control.SetNavigationData(navigation);
    ros::param::set("/planning/sensorstate", 0);
    if(scenario == "traffic1" || scenario == "traffic2") {
        robot::TLStatus light;
        light.light_status = 2;
        control.SetTlStatusData(light);
    }
    const int count = (scenario == "traffic1" || scenario == "traffic2") ? 13 : 8;
    for(int i = 0; i < count; ++i) Tick();
    return 0;
}
