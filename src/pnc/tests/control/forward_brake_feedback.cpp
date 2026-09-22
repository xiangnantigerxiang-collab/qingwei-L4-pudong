// D 挡执行反馈专项：通过真实输入回调和最终发布验证，不访问控制器私有状态。
// 历史夹具仅复用消息构造/发布捕获，不执行其旧 dec*15 和立即恢复的断言。
#define main ExistingControlTestsMain
#include "launch_speed.cpp"
#undef main

int forward_checks = 0;
std::string forward_scene;

void CheckForward(bool condition, const char* message) {
    if(!condition) {
        std::cerr << "FAIL " << forward_scene << ": " << message << '\n';
        std::exit(1);
    }
    ++forward_checks;
}

void StartForward(ControlComply& target, double desired = 3.0, double actual = 3.0) {
    ros::testTime() = 100.0;
    ros::param::set("/planning/sensorstate", 0);
    ros::param::set("/robot/planning/netcheck", 0);
    ros::param::set("/planning/alive", 1);
    ros::param::set("/robot/control/accswitch", 0);
    ros::param::set("/robot/control/brake_integral_threshold", 0.1);
    can_data = robot::can_msg();
    navigation = robot::navigation_msg();
    status = robot::path_plan_status();
    plan = robot::path_plan_msg();
    SetUp(target, desired, actual, GEAR_D);
    navigation.yAxis = 0.2;  // 避开旧横向几何在精确零偏差处的奇异边界。
    target.SetNavigationData(navigation);
}

robot::control_msg ForwardStep(ControlComply& target, double actual, int feedback = 0,
                               double dt = 0.05, bool refresh_can = true,
                               bool refresh_navigation = true) {
    ros::testTime() += dt;
    can_data.vehicleSpeed = 99;  // CAN 速度故意冲突，控制仍必须取导航。
    can_data.brakePercent = feedback;
    navigation.gpsSpeed = actual;
    if(refresh_can) target.SetCanData(can_data);
    if(refresh_navigation) target.SetNavigationData(navigation);
    const auto output = Tick(target);
    CheckForward(output.size() == 1, "D publishes exactly one final command");
    const auto result = output.front();
    CheckForward(result.throttlePercent == 0 || result.brakePercent == 0,
                 "drive and hydraulic requests are mutually exclusive");
    CheckForward(result.throttlePercent <= 100 && result.brakePercent <= 100,
                 "actuator commands remain bounded");
    CheckForward(result.vehicleSpeed == navigation.gpsSpeed, "actual speed remains navigation speed");
    return result;
}

void SetForwardTarget(ControlComply& target, double desired) {
    plan.desireSpeed = desired;
    target.SetPathPlanData(plan);
}

robot::control_msg ArmForwardBrake(ControlComply& target) {
    SetForwardTarget(target, 1.4);
    for(int i = 0; i < 40; ++i) {
        const auto output = ForwardStep(target, 3.0);
        if(output.brakePercent > 0) {
            CheckForward(output.brakePercent == 3, "ordinary first request uses the small initial command");
            CheckForward(output.throttlePercent == 0, "hydraulic entry clears motor command");
            return output;
        }
    }
    CheckForward(false, "persistent failed motor deceleration eventually requests hydraulic assistance");
    return robot::control_msg();
}

void ReachForwardBrakeCap(ControlComply& target) {
    ArmForwardBrake(target);
    for(int i = 0; i < 180; ++i) {
        const auto output = ForwardStep(target, 3.0, 3);
        CheckForward(output.brakePercent <= 15, "normal cap is reached before capacity failure can be confirmed");
        if(output.brakePercent == 15) return;
    }
    CheckForward(false, "persistent confirmed insufficient braking reaches the normal cap");
}

void CompleteForwardRelease(ControlComply& target, double actual = 3.0) {
    bool resumed = false;
    for(int i = 0; i < 45; ++i) {
        const auto output = ForwardStep(target, actual, 0);
        CheckForward(output.brakePercent == 0, "confirmed normal release does not create new braking");
        if(i < 2) CheckForward(output.throttlePercent == 0, "one or two zero frames cannot confirm release");
        resumed = resumed || output.throttlePercent > 0;
        CheckForward(output.throttlePercent <= 47, "restored drive respects current speed plus 0.5 limit");
    }
    CheckForward(resumed, "fresh stable zero feedback eventually permits drive restoration");
}

int main() {
    {
        forward_scene = "straight_5mps";
        ControlComply target;
        StartForward(target, 5.0, 5.0);
        for(int i = 0; i < 100; ++i) {
            const auto output = ForwardStep(target, 5.0);
            CheckForward(output.throttlePercent == 67 && output.brakePercent == 0,
                         "straight 5 m/s retains the 13.5 steady-speed calibration");
        }
    }
    {
        forward_scene = "overspeed_without_planning_decline";
        ControlComply target;
        StartForward(target, 2.0, 3.0);
        for(int i = 0; i < 100; ++i) {
            target.SetPathPlanData(plan);
            CheckForward(ForwardStep(target, 3.0).brakePercent == 0,
                         "constant planning target cannot initiate ordinary hydraulic braking");
        }
    }
    {
        forward_scene = "current_speed_plus_half_cap";
        ControlComply target;
        StartForward(target, 5.0, 2.0);
        for(int i = 0; i < 20; ++i) {
            const auto output = ForwardStep(target, 2.0);
            CheckForward(output.throttlePercent == 33 && output.brakePercent == 0,
                         "new deceleration controller cannot raise the current-speed-plus-0.5 cap");
        }
    }
    {
        forward_scene = "curve_only_no_planning_decline";
        ControlComply target;
        StartForward(target);
        plan.x.clear();
        plan.y.clear();
        for(int i = 0; i <= 100; ++i) {
            const double x = i * 0.2;
            plan.x.push_back(x);
            plan.y.push_back(0.05 * x * x);
        }
        target.SetPathPlanData(plan);
        for(int i = 0; i < 40; ++i) {
            const auto output = ForwardStep(target, 3.0);
            CheckForward(output.throttlePercent < 40 && output.brakePercent == 0,
                         "curvature lowers the motor command but cannot invent a planning descent request");
        }
    }
    {
        forward_scene = "motor_sufficient";
        ControlComply target;
        StartForward(target);
        SetForwardTarget(target, 2.0);
        for(int i = 1; i <= 30; ++i) {
            const double speed = std::max(2.0, 3.0 - 0.04 * i);
            CheckForward(ForwardStep(target, speed).brakePercent == 0,
                         "sufficient observed motor deceleration avoids hydraulic assistance");
        }
    }
    {
        forward_scene = "active_brake_adjustment_rate_and_cap";
        ControlComply target;
        StartForward(target);
        auto previous = ArmForwardBrake(target);
        const double response_time = ros::testTime() + 0.05;
        double increase_time = response_time;
        double cap_time = -1.0;
        bool increased = false;
        bool capacity_fault = false;
        for(int i = 0; i < 240; ++i) {
            const auto output = ForwardStep(target, 3.0, 3);
            if(output.brakePercent == 100) {
                CheckForward(cap_time >= 0.0 && ros::testTime() - cap_time >= 2.79,
                             "capacity failure waits for the latest increase response and sustained insufficiency");
                capacity_fault = true;
                break;
            }
            CheckForward(output.brakePercent >= previous.brakePercent && output.brakePercent <= 15,
                         "constant insufficient deceleration produces bounded monotone assistance");
            if(output.brakePercent == 15 && cap_time < 0.0) cap_time = ros::testTime();
            if(ros::testTime() - response_time < 0.29) {
                CheckForward(output.brakePercent == 3, "first actual response is observed before increasing brake");
            }
            if(output.brakePercent > previous.brakePercent) {
                CheckForward(output.brakePercent > previous.brakePercent &&
                             output.brakePercent <= previous.brakePercent + 3 &&
                             ros::testTime() - increase_time >= (increased ? 0.59 : 0.29),
                             "large verified shortage allows at most three points with response/settling time");
                increase_time = ros::testTime();
                increased = true;
            }
            previous = output;
        }
        CheckForward(increased && capacity_fault,
                     "persistent actual deceleration failure escalates instead of remaining indefinitely at 15");
    }
    {
        forward_scene = "saturation_condition_interrupted";
        ControlComply target;
        StartForward(target);
        ReachForwardBrakeCap(target);
        for(int i = 0; i < 36; ++i) {
            CheckForward(ForwardStep(target, 3.0, 3).brakePercent <= 15,
                         "latest cap increase receives its execution-delay observation interval");
        }
        for(int i = 0; i < 12; ++i) {
            CheckForward(ForwardStep(target, 3.0, 3).brakePercent <= 15,
                         "less than one second of saturation is not a confirmed capacity failure");
        }
        CheckForward(ForwardStep(target, 3.0, 0).brakePercent <= 15,
                     "one fresh zero response interrupts the active-braking saturation condition");
        for(int i = 0; i < 12; ++i) {
            CheckForward(ForwardStep(target, 3.0, 3).brakePercent <= 15,
                         "interrupted saturation evidence cannot be accumulated across intervals");
        }
        robot::control_msg output;
        for(int i = 0; i < 14; ++i) output = ForwardStep(target, 3.0, 3);
        CheckForward(output.brakePercent == 100, "a later continuous failure interval still escalates");
    }
    {
        forward_scene = "delayed_normal_cap_response";
        ControlComply target;
        StartForward(target);
        ReachForwardBrakeCap(target);
        for(int i = 0; i < 34; ++i) {
            CheckForward(ForwardStep(target, 3.0, 3).brakePercent <= 15,
                         "earlier smaller-brake feedback cannot prematurely discredit the latest 15 percent command");
        }
        bool released = false;
        for(int i = 1; i <= 20; ++i) {
            const auto output = ForwardStep(target, 3.0 - 0.04 * i, 3);
            CheckForward(output.brakePercent <= 15,
                         "a useful response arriving 1.7 seconds after the increase does not trigger fault braking");
            released = released || output.brakePercent == 0;
        }
        CheckForward(released, "late useful response proceeds to anticipatory release");
    }
    {
        forward_scene = "small_error_does_not_trip_capacity_failure";
        ControlComply target;
        StartForward(target);
        ReachForwardBrakeCap(target);
        // 跳变只用于隔离误差边界；过快的速度跳变应令加速度估计失效，而非模拟真实动力学。
        ForwardStep(target, 1.9, 3, 0.01);
        for(int i = 0; i < 70; ++i) {
            const auto output = ForwardStep(target, 1.9, 3);
            CheckForward(output.brakePercent <= 15,
                         "remaining speed error at 0.5 cannot trigger the saturated-capacity fault");
        }
    }
    {
        forward_scene = "predictive_release_before_target";
        ControlComply target;
        StartForward(target);
        ArmForwardBrake(target);
        ForwardStep(target, 3.0, 3);
        bool released = false;
        for(int i = 1; i <= 12; ++i) {
            const double speed = 3.0 - 0.05 * i;
            const auto output = ForwardStep(target, speed, 3);
            if(output.brakePercent == 0) {
                CheckForward(speed > plan.desireSpeed + 0.5,
                             "measured deceleration predicts release before actual speed reaches the target");
                CheckForward(output.throttlePercent == 0,
                             "predicted release still respects actual hydraulic engagement");
                released = true;
                break;
            }
        }
        CheckForward(released, "decelerating feedback causes anticipatory hydraulic release");
    }
    {
        forward_scene = "lost_engagement_while_command_remains_positive";
        ControlComply target;
        StartForward(target);
        ArmForwardBrake(target);
        ForwardStep(target, 3.0, 3);
        robot::control_msg output;
        for(int i = 0; i < 45; ++i) {
            output = ForwardStep(target, 3.0, 0);
            CheckForward(output.throttlePercent == 0,
                         "loss of actual engagement never permits drive under a positive brake request");
        }
        CheckForward(output.brakePercent == 0 && output.throttlePercent == 0,
                     "lost small-command engagement becomes unconfirmed without treating deadzone as hardware fault");
    }
    {
        forward_scene = "short_request_late_feedback";
        ControlComply target;
        StartForward(target);
        ArmForwardBrake(target);
        SetForwardTarget(target, 3.2);
        for(int i = 0; i < 6; ++i) {
            const auto output = ForwardStep(target, 3.0, 0);
            CheckForward(output.brakePercent == 0 && output.throttlePercent == 0,
                         "cancelled positive request remains in flight before delayed engagement");
        }
        for(int i = 0; i < 5; ++i) {
            const auto output = ForwardStep(target, 3.0, 8);
            CheckForward(output.brakePercent == 0 && output.throttlePercent == 0,
                         "delayed actual engagement prevents drive even after command cancellation");
        }
        for(int i = 0; i < 3; ++i) {
            CheckForward(ForwardStep(target, 3.0, 0).throttlePercent == 0,
                         "single zero callback cannot prove release of an acknowledged request");
        }
        bool resumed = false;
        for(int i = 0; i < 25; ++i) {
            const auto output = ForwardStep(target, 3.0, 0);
            CheckForward(output.brakePercent == 0, "release remains free of fresh hydraulic pulses");
            resumed = resumed || output.throttlePercent > 0;
        }
        CheckForward(resumed, "acknowledged release eventually resumes tracking");
    }
    for(const std::string reset : {"task_id", "task_route", "task_type", "task_gear", "path_id",
                                   "manual", "reverse_roundtrip", "invalid_path"}) {
        forward_scene = "pending_survives_" + reset;
        ControlComply target;
        StartForward(target);
        ArmForwardBrake(target);
        if(reset.compare(0, 5, "task_") == 0) {
            robot::task_plan_msg task;
            if(reset == "task_id") task.task_id = 11;
            if(reset == "task_route") task.pathList.push_back("new_route");
            if(reset == "task_type") task.taskType = TRACKPATH;
            if(reset == "task_gear") task.desireGear = GEAR_D;
            target.setTaskPlanData(task);
        } else if(reset == "path_id") {
            ++plan.Path_Id;
        } else if(reset == "manual") {
            can_data.controlPanelState = 0;
            target.SetCanData(can_data);
            can_data.controlPanelState = 1;
            target.SetCanData(can_data);
        } else if(reset == "reverse_roundtrip") {
            can_data.curGear = GEAR_R;
            target.SetCanData(can_data);
            can_data.curGear = GEAR_D;
            target.SetCanData(can_data);
        } else {
            auto invalid = plan;
            invalid.y.pop_back();
            target.SetPathPlanData(invalid);
        }
        SetForwardTarget(target, 3.2);
        const auto pending = ForwardStep(target, 3.0, 0);
        CheckForward(pending.throttlePercent == 0, "context reset must retain pending actuator memory");
        const auto active = ForwardStep(target, 3.0, 8);
        CheckForward(active.throttlePercent == 0, "late engagement survives context reset");
        CompleteForwardRelease(target);
    }
    {
        forward_scene = "planning_noise_and_cumulative_rise";
        ControlComply target;
        StartForward(target);
        ArmForwardBrake(target);
        for(int i = 0; i < 8; ++i) {
            SetForwardTarget(target, i % 2 ? 1.4 : 1.4015);
            const auto output = ForwardStep(target, 3.0, 0);
            CheckForward(output.brakePercent == 3 && output.throttlePercent == 0,
                         "tiny target oscillation neither cancels nor increases the pending command");
        }
        for(double speed : {1.43, 1.47, 1.51}) {
            SetForwardTarget(target, speed);
            ForwardStep(target, 3.0, 0);
        }
        const auto output = ForwardStep(target, 3.0, 8);
        CheckForward(output.brakePercent == 0 && output.throttlePercent == 0,
                     "rise is measured from the request minimum and passes through release");
    }
    for(const std::string emergency : {"planning_safety", "gnss", "can_estop", "gnss_and_lateral"}) {
        forward_scene = "independent_" + emergency;
        ControlComply target;
        StartForward(target);
        if(emergency == "planning_safety") {
            plan.safety = true;
            target.SetPathPlanData(plan);
        }
        if(emergency == "gnss" || emergency == "gnss_and_lateral") {
            ros::param::set("/planning/sensorstate", 8);
        }
        if(emergency == "can_estop") can_data.emergencyStop = 1;
        if(emergency == "gnss_and_lateral") navigation.yAxis = 6.0;
        const auto stop = ForwardStep(target, 3.0);
        CheckForward(stop.brakePercent == 100 && stop.throttlePercent == 0,
                     "independent emergency immediately overrides comfort and weaker safety outputs");
        plan.safety = false;
        target.SetPathPlanData(plan);
        ros::param::set("/planning/sensorstate", 0);
        can_data.emergencyStop = 0;
        navigation.yAxis = 0.2;
        CheckForward(ForwardStep(target, 3.0, 0).throttlePercent == 0,
                     "clearing emergency does not forget its in-flight hydraulic request");
        CheckForward(ForwardStep(target, 3.0, 30).throttlePercent == 0,
                     "actual emergency braking remains interlocked after safety clears");
        CompleteForwardRelease(target);
    }
    {
        forward_scene = "actual_brake_without_control_request";
        ControlComply target;
        StartForward(target);
        CheckForward(ForwardStep(target, 3.0, 8).throttlePercent == 0,
                     "actual hydraulic engagement prevents drive without a matching local request");
        CheckForward(ForwardStep(target, 3.0, 0, 0.0).throttlePercent == 0,
                     "same-time zero callback cannot erase actual engagement");
    }
    {
        forward_scene = "distinct_feedback_confirmation";
        ControlComply target;
        StartForward(target);
        ArmForwardBrake(target);
        SetForwardTarget(target, 3.2);
        for(int i = 0; i < 38; ++i) ForwardStep(target, 3.0, 8);
        CheckForward(ForwardStep(target, 3.0, 0).throttlePercent == 0,
                     "first zero feedback begins confirmation only");
        for(int i = 0; i < 30; ++i) {
            CheckForward(ForwardStep(target, 3.0, 0, 0.0).throttlePercent == 0,
                         "duplicate-time callbacks do not add release confirmation");
        }
        for(int i = 0; i < 3; ++i) {
            CheckForward(ForwardStep(target, 3.0, 0, 0.055, false).throttlePercent == 0,
                         "repeated control cycles do not confirm an unchanged CAN observation");
        }
    }
    for(bool stale_can : {false, true}) {
        forward_scene = stale_can ? "straight_brief_can_gap" : "straight_brief_navigation_gap";
        ControlComply target;
        StartForward(target, 5.0, 5.0);
        for(int i = 0; i < 7; ++i) {
            const auto output = ForwardStep(target, 5.0, 0, 0.05, !stale_can, stale_can);
            CheckForward(output.brakePercent == 0 && output.throttlePercent == 67,
                         "brief input gaps outside hydraulic operation do not invent a new emergency stop");
        }
    }
    for(bool stale_can : {false, true}) {
        forward_scene = stale_can ? "stale_can_during_release" : "stale_navigation_during_release";
        ControlComply target;
        StartForward(target);
        ArmForwardBrake(target);
        SetForwardTarget(target, 3.2);
        ForwardStep(target, 3.0, 8);
        for(int i = 0; i < 7; ++i) {
            const auto output = ForwardStep(target, 3.0, 0, 0.05, !stale_can, stale_can);
            CheckForward(output.throttlePercent == 0, "stale feedback cannot authorize drive restoration");
        }
    }
    {
        forward_scene = "active_brake_stale_grace_then_fault";
        ControlComply target;
        StartForward(target);
        ArmForwardBrake(target);
        ForwardStep(target, 3.0, 3);
        for(int i = 0; i < 8; ++i) {
            const auto output = ForwardStep(target, 3.0, 3, 0.05, false);
            CheckForward(output.throttlePercent == 0 && output.brakePercent == 3,
                         "short active-feedback outage holds the last brake without an immediate jump to 100");
        }
        robot::control_msg output;
        for(int i = 0; i < 10; ++i) output = ForwardStep(target, 3.0, 3, 0.05, false);
        CheckForward(output.throttlePercent == 0 && output.brakePercent == 100,
                     "continued active-feedback outage eventually escalates to fault braking");
    }
    {
        forward_scene = "cancelled_unknown_engagement_and_manual_reset";
        ControlComply target;
        StartForward(target);
        ArmForwardBrake(target);
        SetForwardTarget(target, 3.2);
        robot::control_msg output;
        for(int i = 0; i < 48; ++i) {
            output = ForwardStep(target, 3.0, 0);
            CheckForward(output.throttlePercent == 0 && output.brakePercent == 0,
                         "cancelled unacknowledged request waits without drive or timeout-only strong braking");
        }
        for(int i = 0; i < 12; ++i) {
            output = ForwardStep(target, 0.0, 0);
            CheckForward(output.throttlePercent == 0 && output.brakePercent == 0,
                         "automatic standstill retains unknown execution without inventing a deceleration failure");
        }
        can_data.controlPanelState = 0;
        for(int i = 0; i < 12; ++i) ForwardStep(target, 0.0, 0);
        can_data.controlPanelState = 1;
        const auto recovered = ForwardStep(target, 0.0, 0);
        CheckForward(recovered.brakePercent == 0 && recovered.throttlePercent > 0,
                     "manual standstill and stable fresh zero feedback permit deliberate recovery");
    }
    {
        forward_scene = "missing_small_engagement_with_persistent_shortage_unknown";
        ControlComply target;
        StartForward(target);
        ArmForwardBrake(target);
        robot::control_msg output;
        for(int i = 0; i < 48; ++i) {
            output = ForwardStep(target, 3.0, 0);
            CheckForward(output.throttlePercent == 0,
                         "persistent unmet deceleration never restores drive while response is missing");
        }
        CheckForward(output.brakePercent == 0,
                     "unacknowledged 3 percent is cancelled and classified unknown rather than automatic 100");
        for(int i = 0; i < 12; ++i) {
            const auto stopped = ForwardStep(target, 0.0, 0);
            CheckForward(stopped.brakePercent == 0 && stopped.throttlePercent == 0,
                         "automatic standstill cannot infer release of an unseen hydraulic request");
        }
    }
    {
        forward_scene = "manual_park_then_automatic";
        ControlComply target;
        StartForward(target, 0.0, 0.0);
        can_data.controlPanelState = 0;
        status.taskExecuStatus = TASKFINISHED;
        target.SetPathStatusData(status);
        for(int i = 0; i < 65; ++i) {
            const auto output = ForwardStep(target, 0.0, 0);
            CheckForward(output.brakePercent == 80,
                         "manual parking retains its existing output without inventing an execution fault");
        }
        status.taskExecuStatus = 1;
        target.SetPathStatusData(status);
        SetForwardTarget(target, 3.0);
        can_data.controlPanelState = 1;
        const auto output = ForwardStep(target, 0.0, 0);
        CheckForward(output.brakePercent == 0 && output.throttlePercent > 0,
                     "unexecuted manual parking commands do not lock the next automatic session");
    }
    {
        forward_scene = "manual_takeover_releases_confirmed_brake";
        ControlComply target;
        StartForward(target);
        ArmForwardBrake(target);
        ForwardStep(target, 3.0, 8);
        can_data.controlPanelState = 0;
        SetForwardTarget(target, 3.2);
        for(int i = 0; i < 55; ++i) ForwardStep(target, 0.0, 8);
        for(int i = 0; i < 12; ++i) ForwardStep(target, 0.0, 0);
        can_data.controlPanelState = 1;
        const auto output = ForwardStep(target, 0.0, 0);
        CheckForward(output.brakePercent == 0 && output.throttlePercent > 0,
                     "actual manual release is recognized before automatic timeout evaluation");
    }
    {
        forward_scene = "release_timeout";
        ControlComply target;
        StartForward(target);
        ArmForwardBrake(target);
        ForwardStep(target, 3.0, 8);
        SetForwardTarget(target, 3.2);
        robot::control_msg output;
        for(int i = 0; i < 66; ++i) {
            output = ForwardStep(target, 3.0, 8);
            CheckForward(output.throttlePercent == 0, "failed release never permits opposing drive");
        }
        CheckForward(output.brakePercent == 100, "persistent actual braking after release request latches fault");
    }
    {
        forward_scene = "clock_rollback";
        ControlComply target;
        StartForward(target);
        ArmForwardBrake(target);
        SetForwardTarget(target, 3.2);
        const auto output = ForwardStep(target, 3.0, 0, -10.0);
        CheckForward(output.brakePercent == 100 && output.throttlePercent == 0,
                     "backward clock cannot clear in-flight braking and triggers the D fault path");
        for(int i = 0; i < 10; ++i) {
            CheckForward(ForwardStep(target, 3.0, 0).throttlePercent == 0,
                         "fresh automatic frames after rollback cannot silently release the fault");
        }
        const double old_clock = ros::testTime() + 8.0;
        can_data.controlPanelState = 0;
        for(int i = 0; i < 12; ++i) ForwardStep(target, 0.0, 0);
        can_data.controlPanelState = 1;
        const auto recovered = ForwardStep(target, 0.0, 0);
        CheckForward(ros::testTime() < old_clock && recovered.brakePercent == 0 &&
                     recovered.throttlePercent > 0,
                     "manual recovery after rollback does not wait for the previous absolute clock value");
    }
    {
        forward_scene = "positive_feedback_clock_rollback_between_ticks";
        ControlComply target;
        StartForward(target);
        ArmForwardBrake(target);
        ros::testTime() += 0.15;
        can_data.brakePercent = 3;
        target.SetCanData(can_data);
        ros::testTime() -= 0.05;
        const auto output = ForwardStep(target, 3.0, 3, 0.0);
        CheckForward(output.brakePercent == 100 && output.throttlePercent == 0,
                     "positive-feedback clock rollback is detected even between two control cycles");
    }
    {
        forward_scene = "terminal_continuity_from_active_brake";
        ControlComply target;
        StartForward(target);
        const auto first = ArmForwardBrake(target);
        robot::task_plan_msg task;
        task.taskType = TRACKPATH;
        task.desireGear = GEAR_D;
        target.setTaskPlanData(task);
        status.taskExecuStatus = TASKFINISHED;
        status.distance2Stop = 0.2;
        target.SetPathStatusData(status);
        SetForwardTarget(target, 0.0);
        auto last = ForwardStep(target, 0.2, 3);
        CheckForward(last.brakePercent >= first.brakePercent && last.brakePercent < 80,
                     "ordinary low-speed terminal transition continues small braking without jumping to 80");
        for(int i = 0; i < 10; ++i) {
            const auto output = ForwardStep(target, 0.2, 3);
            CheckForward(output.brakePercent >= last.brakePercent &&
                         output.brakePercent <= last.brakePercent + 1,
                         "moving terminal brake follows bounded integer command increments");
            last = output;
        }
        const auto first_stopped = ForwardStep(target, 0.0, 3);
        CheckForward(first_stopped.brakePercent < 80, "one zero-speed observation does not abruptly apply hold");
        plan.safety = true;
        target.SetPathPlanData(plan);
        CheckForward(ForwardStep(target, 0.0, 3).brakePercent == 100,
                     "terminal comfort never delays an independent emergency");
    }
    std::cout << "PASS forward brake feedback: " << forward_checks << " checks\n";
    return 0;
}
