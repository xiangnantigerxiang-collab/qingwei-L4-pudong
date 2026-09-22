#include "control_access.h"
#define main ExistingControlTestsMain
#include "../control/launch_speed.cpp"
#undef main
#include "robot_can_comm/can_comm_comply.h"

int navigation_checks = 0;

void Check(bool condition, const char* name) {
    if(!condition) {
        std::cerr << "FAIL: " << name << '\n';
        std::exit(1);
    }
    ++navigation_checks;
}

int main(int argc, char** argv) {
    if(argc != 4) return 2;
    const std::string mode = argv[1];
    const double can_speed = std::strtod(argv[2], nullptr);
    const double nav_speed = std::strtod(argv[3], nullptr);
    ros::param::set("/planning/sensorstate", 0);
    ros::param::set("/robot/control/accswitch", 0);
    ros::testTime() = 100;
    const bool invalid_test = mode == "invalid_forward" || mode == "invalid_reverse";
    SetUp(control, mode == "brake" ? 1 : 2, invalid_test ? 1 : nav_speed,
          mode == "reverse" || mode == "invalid_reverse" ? GEAR_R : GEAR_D);
    if(mode == "brake") RequestDeceleration(control);
    std::cout << std::setprecision(9);
    can_data.vehicleSpeed = can_speed;
    if(invalid_test) {
        // 避免旧横向几何中精确零偏差的独立 NaN 边界，单独检查速度来源。
        navigation.yAxis = 0.2;
        control.SetNavigationData(navigation);
        Check(std::isfinite(Tick().front().wheelAngle), "valid navigation produces finite steering");
        navigation.gpsSpeed = nav_speed;
        control.SetNavigationData(navigation);
        for(int i = 0; i < 3; ++i) {
            ros::testTime() += 0.05;
            control.SetCanData(can_data);
            const auto output = Tick();
            Check(output.size() == 1, "invalid navigation publishes one stop command");
            const auto& message = output.front();
            Check(message.throttlePercent == 0 && message.brakePercent == 100,
                  "invalid navigation disables drive and requests safety brake");
            Check(message.wheelAngle == 0 && message.desireSpeed == 0 && message.desireAcc == 0,
                  "invalid navigation never reaches steering or target calculations");
            Check(control.mBrakeIntegral == 0 && control.mTerminalBrakeStartTime < 0,
                  "invalid navigation resets ordinary brake history");
        }
        navigation.gpsSpeed = 1;
        control.SetNavigationData(navigation);
        plan.safety = true;
        plan.desireSpeed = 0;
        control.SetPathPlanData(plan);
        auto output = Tick();
        Check(output.size() == 1 && output.front().throttlePercent == 0 && output.front().brakePercent == 100,
              "navigation recovery cannot release an independent safety stop");
        plan.safety = false;
        plan.desireSpeed = 2;
        control.SetPathPlanData(plan);
        for(int i = 0; i < 20; ++i) {
            ros::testTime() += 0.05;
            control.SetCanData(can_data);
            control.SetNavigationData(navigation);
            output = Tick();
            Check(std::isfinite(output.front().wheelAngle) && std::isfinite(output.front().desireAcc),
                  "valid navigation recovers without poisoned controller state");
        }
        Check(output.front().throttlePercent > 0 && output.front().brakePercent == 0,
              "valid navigation restores existing D/R drive after safety clears");
        std::cout << "PASS invalid navigation control: " << navigation_checks << " checks\n";
        return 0;
    }
    if(mode == "freshness") {
        control.SetCanData(can_data);
        plan.desireSpeed = 0;
        RequestDeceleration(control);
        Check(!control.UpdateBrakeIntegral(1), "first sample does not integrate");
        ros::testTime() = 100.25;
        control.SetCanData(can_data);
        Check(!control.UpdateBrakeIntegral(1), "fresh CAN cannot integrate stale navigation");
        Check(control.mBrakeIntegral == 0, "pending integral resets on navigation timeout");
        control.SetNavigationData(navigation);
        Check(!control.UpdateBrakeIntegral(1), "recovery establishes a new time origin");
        ros::testTime() = 100.3;
        Check(!control.UpdateBrakeIntegral(1), "partial fresh error below threshold");
        ros::testTime() = 100.4;
        Check(control.UpdateBrakeIntegral(1), "fresh navigation enables braking integral");
        ros::testTime() = 100.6;
        control.SetCanData(can_data);
        Check(control.UpdateBrakeIntegral(1), "navigation loss retains already active brake");
        control.ResetBrakeIntegral();
        ros::testTime() = 101;
        control.SetNavigationData(navigation);
        RequestDeceleration(control);
        Check(!control.UpdateBrakeIntegral(1), "navigation cannot replace stale CAN status");
        can_data.controlPanelState = 0;
        control.SetCanData(can_data);
        Check(!control.UpdateBrakeIntegral(1), "manual mode still disables D brake integral");
        can_data.controlPanelState = 1;
        control.SetCanData(can_data);
        navigation.gpsSpeed = 0.3;
        control.SetNavigationData(navigation);
        control.mSpeed = 0;
        control.mTaskInfo.taskType = TRACKPATH;
        control.mTaskInfo.desireGear = GEAR_D;
        control.mPathStatus.taskExecuStatus = TASKFINISHED;
        control.mPathStatus.distance2Stop = 0.1;
        ros::param::set("/planning/alive", 1);
        Check(control.TerminalStopBrake() == 5, "terminal creep brake uses navigation rather than CAN 99");
        ros::testTime() += 0.25;
        control.SetCanData(can_data);
        Check(control.TerminalStopBrake() == 80, "fresh CAN cannot enable gentle braking with stale navigation");
        navigation.gpsSpeed = 0.02;
        control.SetNavigationData(navigation);
        Check(control.TerminalStopBrake() == 80, "navigation standstill uses original holding brake");
        std::cout << "PASS navigation control freshness: 12 checks\n";
        return 0;
    }
    for(int i = 0; i < 12; ++i) {
        ros::testTime() = 100 + 0.05 * i;
        // 两种回调顺序都必须只保留导航实际速度。
        if(i % 2) {
            control.SetCanData(can_data);
            control.SetNavigationData(navigation);
        } else {
            control.SetNavigationData(navigation);
            control.SetCanData(can_data);
        }
        Check(control.mVehicleSpeed == navigation.gpsSpeed, "control feedback uses navigation");
        for(const auto& message : Tick()) {
            Check(message.vehicleSpeed == navigation.gpsSpeed, "control output reports navigation");
            CanCommComply can_comm;
            can_comm.SetControlData(message);
            can_comm.CanCommProcess();
            Check(can_comm.CanCommMsg.vehicleSpeed == navigation.gpsSpeed, "CAN command forwarding reports navigation");
        }
    }
}
