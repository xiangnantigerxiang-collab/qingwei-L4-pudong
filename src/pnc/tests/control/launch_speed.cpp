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
    target.SetCanData(can_data);
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
                  << ' ' << msg.biaDistance << ' ' << msg.biaAngle << '\n';
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
    double origin = scenario == "traffic1" ? 1555.0 :
                    (scenario == "traffic2" ? 428.0 : 0.0);
    SetUp(control, 2.0, 0.0, GEAR_D, origin, scenario != "empty");

    if(scenario == "feedback") {
        for(int i = 0; i < 160; ++i) {
            const std::vector<robot::control_msg> messages = Tick();
            can_data.vehicleSpeed = messages.back().throttlePercent / 18.0;
            control.SetCanData(can_data);
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
        for(int i = 0; i < 4; ++i) Tick();
        return 0;
    }

    // 先建立非零斜坡状态,再触发停车/降速,检查是否遗留旧状态。
    for(int i = 0; i < 40; ++i) Tick();
    std::cout << "PHASE after_warmup\n";
    if(scenario == "neutral" || scenario == "gear_reverse") {
        can_data.curGear = scenario == "neutral" ? GEAR_N : GEAR_R;
        control.SetCanData(can_data);
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
    } else if(scenario == "fence") {
        if(argc < 4) return 2;
        ros::param::set("path_dir", std::string(argv[3]) + "/");
        control.LoadPathFile("outside");
        control.FenceAlarm();
    } else if(scenario == "traffic1" || scenario == "traffic2") {
        robot::TLStatus light;
        light.light_status = 0;
        control.SetTlStatusData(light);
    } else if(scenario == "decrease") {
        plan.desireSpeed = 0.04;
        control.SetPathPlanData(plan);
    } else if(scenario == "instances") {
        SetUp(second_control, 2.0, 0.0, GEAR_D);
        for(int i = 0; i < 4; ++i) Tick(second_control);
        return 0;
    } else {
        return 2;
    }
    Tick();
    if(scenario == "decrease") return 0;

    can_data.curGear = GEAR_D;
    control.SetCanData(can_data);
    plan.desireSpeed = 2.0;
    plan.Path_Id = 1;
    plan.safety = false;
    control.SetPathPlanData(plan);
    status.taskExecuStatus = 1;
    control.SetPathStatusData(status);
    navigation.yAxis = 0.0;
    control.SetNavigationData(navigation);
    ros::param::set("/planning/sensorstate", 0);
    if(scenario == "fence") {
        control.LoadPathFile("inside");
        control.FenceAlarm();
    }
    if(scenario == "traffic1" || scenario == "traffic2") {
        robot::TLStatus light;
        light.light_status = 2;
        control.SetTlStatusData(light);
    }
    const int count = (scenario == "traffic1" || scenario == "traffic2") ? 9 : 4;
    for(int i = 0; i < count; ++i) Tick();
    return 0;
}
