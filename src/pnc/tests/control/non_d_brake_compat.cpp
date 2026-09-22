// 链接修改前后真实控制源码，逐周期保留所有非D发布字段和消息顺序。
// 每个用例单独进程；全局对象先零初始化，使历史未初始化m_acc_last在两版同为0。
#include "non_d_brake_access.h"
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <limits>

ControlComply control;
robot::can_msg can_data;
robot::navigation_msg navigation;
robot::path_plan_msg plan;
robot::path_plan_status status;
robot::task_plan_msg task;
ros::Publisher publisher;

void FloatField(float value) {
    if(std::isnan(value)) {
        // 只规范化NaN载荷；NaN与有限值、符号无穷之间的变化仍导致差分失败。
        std::cout << " nan";
    } else {
        uint32_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        std::cout << ' ' << bits;
    }
}

void Trace(int tick, int index, const robot::control_msg& msg) {
    std::cout << "MESSAGE " << tick << ' ' << index;
    FloatField(msg.biaDistance);
    FloatField(msg.biaAngle);
    FloatField(msg.preCurve);
    FloatField(msg.preAngleDev);
    FloatField(msg.desireSpeed);
    FloatField(msg.desireAcc);
    std::cout << ' ' << +msg.throttlePercent << ' ' << +msg.brakePercent;
    FloatField(msg.wheelAngle);
    FloatField(msg.vehicleSpeed);
    std::cout << ' ' << +msg.remoteEnable << ' ' << +msg.bypassProcessing
              << ' ' << msg.faultCode.size();
    for(auto code : msg.faultCode) std::cout << ' ' << +code;
    std::cout << '\n';
}

void Feedback(bool fresh_can = true, bool fresh_navigation = true) {
    if(fresh_can) control.SetCanData(can_data);
    if(fresh_navigation) control.SetNavigationData(navigation);
}

void Geometry(int gear, double curvature, bool empty = false) {
    plan.x.clear();
    plan.y.clear();
    if(!empty) {
        for(int i = 0; i <= 150; ++i) {
            const double x = (gear == GEAR_R ? -1 : 1) * i * 0.2;
            plan.x.push_back(x);
            plan.y.push_back(0.2 + curvature * x * x);
        }
    }
    control.SetPathPlanData(plan);
}

void Tick(int tick, bool emit) {
    int count = 0;
    publisher.capture = [&](const std::type_info& type, const void* raw) {
        if(type != typeid(robot::control_msg)) std::exit(7);
        if(emit) Trace(tick, count, *static_cast<const robot::control_msg*>(raw));
        ++count;
    };
    control.VehicleControl();
    control.PublishMessage(publisher);
    if(emit) std::cout << "COUNT " << tick << ' ' << count << '\n';
}

int main(int argc, char** argv) {
    if(argc != 7) return 2;
    const std::string scene = argv[1];
    const int gear = std::atoi(argv[2]);
    const double desired = std::atof(argv[3]);
    const double actual = std::atof(argv[4]);
    const double curvature = std::atof(argv[5]);
    const std::string prefix = argv[6];
    if(gear != GEAR_R && gear != GEAR_N) return 3;
    ros::testTime() = 100;
    ros::param::set("/planning/sensorstate", 0);
    ros::param::set("/robot/control/accswitch", 0);
    ros::param::set("/planning/alive", 1);
    ros::param::set("/robot/planning/netcheck", 0);
    publisher.name = "/control_msg";
    control.mControlData.remoteEnable = 7;
    control.mControlData.bypassProcessing = 9;
    control.mControlData.faultCode = {1, 3, 250};
    control.mControlData.desireAcc = 0;
    can_data.curGear = prefix == "none" ? gear : GEAR_D;
    can_data.controlPanelState = 1;
    can_data.wheelAngle = 25;
    can_data.vehicleSpeed = 99;  // 防止测试误用已弃用的CAN速度。
    navigation.heading = 90;
    navigation.gpsSpeed = prefix == "none" ? actual : 3.0;
    status.taskExecuStatus = 1;
    status.distance2Stop = 10;
    task.task_id = 1;
    task.taskType = TRACKPATH;
    task.desireGear = can_data.curGear;
    task.pathList.push_back("non_d_brake_compat");
    Feedback();
    control.setTaskPlanData(task);
    control.SetPathStatusData(status);
    plan.Path_Id = 1;
    plan.desireSpeed = prefix == "none" ? desired : 3.0;
    Geometry(can_data.curGear, curvature, scene == "empty" && prefix == "none");

    if(prefix != "none") {
        for(int i = 0; i < 50; ++i) {
            ros::testTime() = 100 + 0.05 * i;
            if(i == 2 && prefix != "drive") {
                plan.desireSpeed = prefix == "terminal" ? 0.0 : 1.0;
                if(prefix == "safety") plan.safety = true;
                control.SetPathPlanData(plan);
            }
            if(prefix == "terminal" && i >= 2) {
                navigation.gpsSpeed = 0.2;
                status.taskExecuStatus = TASKFINISHED;
                status.distance2Stop = 0.2;
                control.SetPathStatusData(status);
            }
            Feedback();
            Tick(i, false);  // D输出允许改变；切挡后的所有字段必须继续对照。
        }
        ros::testTime() += 0.05;
        plan.safety = false;
        plan.desireSpeed = desired;
        can_data.curGear = gear;
        task.desireGear = gear;
        navigation.gpsSpeed = actual;
        status.taskExecuStatus = 1;
        status.distance2Stop = 10;
        Feedback();
        if(scene != "gear_only_transition") control.setTaskPlanData(task);
        control.SetPathStatusData(status);
        if(scene != "gear_only_transition") Geometry(gear, curvature, scene == "empty");
    }

    const double start = ros::testTime();
    for(int i = 0; i < 36; ++i) {
        ros::testTime() = start + i * 0.05;
        if(scene == "duplicate" && i >= 8 && i <= 12) ros::testTime() = start + 0.35;
        if(scene == "clockback" && i >= 8) ros::testTime() -= 1.0;
        if(scene == "gap" && i >= 8) ros::testTime() += 1.0;
        can_data.controlPanelState = scene == "manual" ||
            (scene == "mode_cycle" && i >= 10 && i < 20) ? 0 : 1;
        can_data.emergencyStop = scene == "can_estop" && i >= 5;
        can_data.curGear = scene == "gear_cycle" && i >= 10 && i < 20 ? GEAR_N : gear;
        navigation.gpsSpeed = scene == "speed_ramp" ? actual * (35 - i) / 35.0 : actual;
        navigation.yAxis = scene == "lateral" && i >= 5 ? 6.0 : 0;
        if(scene == "navigation_nan" && i >= 5 && i < 20)
            navigation.gpsSpeed = std::numeric_limits<float>::quiet_NaN();
        if(scene == "navigation_inf" && i >= 5 && i < 20)
            navigation.gpsSpeed = std::numeric_limits<float>::infinity();
        Feedback(!(scene == "stale_can" && i > 5), !(scene == "stale_navigation" && i > 5));
        ros::param::set("/planning/sensorstate", i >= 5 ?
            (scene == "gnss" ? 8 : (scene == "sensor_slow" ? 2 : 0)) : 0);
        ros::param::set("/robot/control/accswitch", scene == "acc" ? 1 : 0);
        if(scene == "terminal" && i >= 5) {
            status.taskExecuStatus = TASKFINISHED;
            status.distance2Stop = 0.2;
            plan.desireSpeed = 0;
        }
        plan.safety = (scene == "safety" || scene == "safety_zero" || scene == "safety_path5") && i >= 5;
        if(scene == "safety_zero" && i >= 5) plan.desireSpeed = 0;
        if(scene == "safety_path5" && i >= 5) plan.Path_Id = 5;
        if(scene == "target_cycle") plan.desireSpeed = i < 10 ? desired : (i < 20 ? 0.2 : desired + 0.5);
        if(scene == "task_change" && i == 10) {
            task.task_id += 1;
            control.setTaskPlanData(task);
        }
        if(scene == "path_change" && i == 10) plan.Path_Id += 1;
        if(scene == "traffic") {
            robot::TLStatus light;
            light.light_status = i < 20 ? 0 : 2;
            control.SetTlStatusData(light);
        }
        control.SetPathStatusData(status);
        // 单独CAN换挡后的前8周期不送任务/规划，检查是否仅靠实际挡位就阻止D状态泄漏。
        if(!(scene == "gear_only_transition" && i < 8) &&
           (i % 2 == 0 || scene == "target_cycle" || scene == "terminal")) control.SetPathPlanData(plan);
        Tick(i, true);
    }
    return 0;
}
