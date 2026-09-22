// 公开消息入口业务验收：不读取或注入制动器私有状态，不模拟控制实现。
// 每个用例独立进程；全局对象与真实control_node一致，历史未初始化成员先零初始化。
#include "control_comply.h"
#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <string>

ControlComply control;
robot::can_msg can_data;
robot::navigation_msg navigation;
robot::path_plan_msg plan;
robot::path_plan_status status;
robot::task_plan_msg task;
ros::Publisher publisher;
robot::control_msg output;
int checks = 0;
int failures = 0;
int published = 0;
double scenario_time = 0.0;
double clock_offset = 0.0;

void Check(bool condition, const std::string& reason) {
    ++checks;
    if(!condition) {
        ++failures;
        std::cerr << "FAIL t=" << scenario_time << " " << reason
                  << " throttle=" << +output.throttlePercent << " brake=" << +output.brakePercent << '\n';
    }
}

void SetGeometry() {
    plan.x.clear();
    plan.y.clear();
    for(int i = 0; i < 60; ++i) {
        plan.x.push_back(i * 0.5);
        plan.y.push_back(0.2);
    }
}

void Setup(bool geometry = true) {
    ros::testTime() = 100.0;
    ros::param::set("/planning/sensorstate", 0);
    ros::param::set("/planning/alive", 1);
    ros::param::set("/robot/planning/netcheck", 0);
    ros::param::set("/robot/control/accswitch", 0);
    task.task_id = 1;
    task.taskType = TRACKPATH;
    task.desireGear = GEAR_D;
    task.pathList.push_back("brake_state_revision");
    control.setTaskPlanData(task);
    plan.Path_Id = 1;
    if(geometry) SetGeometry();
    navigation.heading = 90;
    navigation.yAxis = 0.2;
    can_data.vehicleSpeed = 99;  // 实际速度必须来自导航。
    publisher.name = "/control_msg";
    publisher.capture = [&](const std::type_info& type, const void* raw) {
        if(type != typeid(robot::control_msg)) std::exit(4);
        output = *static_cast<const robot::control_msg*>(raw);
        ++published;
    };
    std::cout << "TRACE,t,clock,gear,automatic,speed,target,feedback,finished,safety,throttle,brake,published\n";
}

void Step(double t, int gear, bool automatic, double speed, double target,
          int feedback, bool finished = false, bool safety = false,
          bool fresh_navigation = true, bool fresh_can = true) {
    scenario_time = t;
    ros::testTime() = 100 + t + clock_offset;
    can_data.curGear = gear;
    can_data.controlPanelState = automatic;
    can_data.brakePercent = feedback;
    navigation.gpsSpeed = speed;
    plan.desireSpeed = target;
    plan.safety = safety;
    status.taskExecuStatus = finished ? TASKFINISHED : 1;
    status.distance2Stop = finished ? 0.2 : 10;
    if(fresh_can) control.SetCanData(can_data);
    if(fresh_navigation) control.SetNavigationData(navigation);
    control.SetPathStatusData(status);
    control.SetPathPlanData(plan);
    published = 0;
    control.VehicleControl();
    control.PublishMessage(publisher);
    if(gear == GEAR_D) {
        Check(published == 1, "D must publish one command");
        Check(!(output.throttlePercent > 0 && output.brakePercent > 0), "D throttle/brake mutual exclusion");
    }
    std::cout << std::fixed << std::setprecision(6) << "TRACE," << t << ','
              << ros::testTime() - 100 << ',' << gear << ',' << automatic << ',' << speed
              << ',' << target << ',' << feedback << ',' << finished << ',' << safety
              << ',' << +output.throttlePercent << ',' << +output.brakePercent << ',' << published << '\n';
}

void OldManualFeedback(const std::string& scenario) {
    const bool empty = scenario == "manual_n_empty";
    const bool moving = scenario == "manual_n_moving_hold";
    Setup(!empty);
    bool resumed = false;
    for(int i = 0; i <= 140; ++i) {
        const double t = i * 0.05;
        if(t < 3.5) {
            Step(t, GEAR_N, false, moving ? 0.2 : 0.0, 0.0, t < 0.5 ? 20 : 0, true);
            continue;
        }
        if(empty && i == 80) SetGeometry();
        const int feedback = t >= 3.65 && t < 4.2 ? 80 : 0;
        Step(t, GEAR_D, true, t < 4 && moving ? 0.2 : 0.0,
             t < 4 && !empty ? 0.0 : 1.0, feedback, t < 4 && !empty);
        Check(output.brakePercent < 100, "old released manual feedback must not fault new D request");
        if(i == 70) Check(output.brakePercent > 0, "first D branch must actually request stop");
        if(t >= 4.0 && output.throttlePercent > 0) resumed = true;
        if(feedback != 0) Check(output.throttlePercent == 0, "actual brake must block new-task propulsion");
    }
    Check(resumed, "new task resumes after actual response and release");
}

void NeutralPending(bool delayed_response) {
    Setup();
    bool resumed = false;
    for(int i = 0; i <= 100; ++i) {
        const double t = i * 0.05;
        const int feedback = delayed_response && t >= 1.2 && t < 1.6 ? 80 : 0;
        Step(t, t < 0.5 ? GEAR_N : GEAR_D, true, 0.0, 1.0, feedback);
        if(t < 0.5) Check(output.throttlePercent == 0 && output.brakePercent == 80, "N output remains original stop");
        else if(!delayed_response || t < 1.6)
            Check(output.throttlePercent == 0, "unanswered automatic N request must survive D switch");
        else if(output.throttlePercent > 0) resumed = true;
    }
    if(delayed_response) Check(resumed, "N-origin request can finish after actual delayed response/release");
}

void ClockRewind() {
    Setup();
    double first_brake = -1.0;
    for(int i = 0; i <= 48; ++i) {
        const double t = i * 0.05;
        clock_offset = t >= 0.5 ? -10.0 : 0.0;
        Step(t, GEAR_D, true, 2.0, t >= 0.2 ? 1.0 : 2.0, 0);
        if(output.brakePercent > 0 && first_brake < 0) {
            first_brake = t;
            Check(output.brakePercent == 3, "rebased ordinary request retains first calibrated command");
        }
    }
    Check(first_brake >= 0.5 && first_brake <= 2.4, "request survives rewind and reacts without waiting old clock");
}

void UnknownCancelled(bool delayed_response, bool safety) {
    Setup();
    double first_brake = -1.0;
    bool cancelled = false;
    bool resumed = false;
    for(int i = 0; i <= 220; ++i) {
        const double t = i * 0.05;
        if(first_brake >= 0 && t >= first_brake + 0.2 - 1e-9) cancelled = true;
        const double target = t < 0.2 || cancelled ? 2.0 : 1.0;
        const int feedback = delayed_response && t >= 7.0 && t < 7.4 ? 3 : 0;
        const bool emergency = safety && t >= 4.0;
        Step(t, GEAR_D, true, 2.0, target, feedback, false, emergency);
        if(first_brake < 0 && output.brakePercent > 0) {
            first_brake = t;
            Check(output.brakePercent == 3, "ordinary command begins at unchanged 3 percent");
        }
        if(emergency) {
            Check(output.throttlePercent == 0 && output.brakePercent == 100, "safety preempts unknown state immediately");
        } else if(cancelled) {
            Check(output.brakePercent < 100, "no speed error or independent safety: unanswered small command is not hard fault");
            if(!delayed_response || t < 7.4)
                Check(output.throttlePercent == 0, "unseen or active delayed response cannot be released by elapsed time");
            else if(output.throttlePercent > 0) resumed = true;
        }
    }
    Check(first_brake >= 0, "test must create a real ordinary request before cancelling");
    if(delayed_response) Check(resumed, "late actual response then release resolves unknown interlock");
}

void FailedDeceleration(bool cancel_target, bool delayed_response = false) {
    Setup();
    double first_brake = -1.0;
    bool fault_stop = false;
    for(int i = 0; i <= 240; ++i) {
        const double t = i * 0.05;
        const bool cancelled = cancel_target && first_brake >= 0 && t >= first_brake + 0.2 - 1e-9;
        const double target = t < 0.2 ? 3.0 :
            (cancelled ? (delayed_response ? 1.55 : 1.5) : (delayed_response ? 1.4 : 1.0));
        const int feedback = delayed_response && first_brake >= 0 &&
            t >= first_brake + 1.9 - 1e-9 && t < first_brake + 2.1 - 1e-9 ? 3 : 0;
        Step(t, GEAR_D, true, 3.0, target, feedback);
        if(first_brake < 0 && output.brakePercent > 0) first_brake = t;
        if(output.brakePercent == 100 && output.throttlePercent == 0) fault_stop = true;
        if(first_brake >= 0 && (!delayed_response || t < first_brake + 2.25 - 1e-9))
            Check(output.throttlePercent == 0, "unseen or still-releasing small request blocks propulsion");
        Check(output.brakePercent <= 3, "unconfirmed small request alone is not evidence for hard fault braking");
    }
    Check(first_brake >= 0, "failure case must issue actual ordinary request");
    Check(!fault_stop, "small unknown request cannot be classified as confirmed hardware failure");
}

void SaturationNeedsNewNavigation() {
    Setup();
    double first_brake = -1.0;
    double first_saturation = -1.0;
    bool fault_after_new_sample = false;
    bool withheld_navigation = false;
    for(int i = 0; i <= 400; ++i) {
        const double t = i * 0.05;
        const int feedback = first_brake >= 0 && t >= first_brake + 0.2 - 1e-9 ? 3 : 0;
        // 15%上限先给足1.8秒执行观察；在1秒失效确认的末尾只推进控制/CAN。
        // 最后新导航在上限后2.7秒；2.75/2.8/2.85秒均仍满足输入新鲜度。
        const bool fresh = first_saturation < 0 || t < first_saturation + 2.75 - 1e-9 ||
            t >= first_saturation + 2.9 - 1e-9;
        withheld_navigation = withheld_navigation || !fresh;
        Step(t, GEAR_D, true, 3.0, t < 0.2 ? 3.0 : 1.2, feedback, false, false, fresh);
        if(first_brake < 0 && output.brakePercent > 0) first_brake = t;
        if(first_saturation < 0 && output.brakePercent == 15) first_saturation = t;
        if(first_saturation < 0 || t <= first_saturation + 2.85 + 1e-9)
            Check(output.brakePercent <= 15, "saturation failure confirmation advances only on new acceleration evidence");
        else if(output.brakePercent == 100) fault_after_new_sample = true;
    }
    Check(first_saturation >= 0, "ordinary feedback case must reach actual published 15 percent");
    Check(withheld_navigation, "saturation case must exercise control-only ticks within navigation freshness");
    Check(fault_after_new_sample, "continued fresh ineffective saturation must eventually force safety brake");
}

void RepeatedUnansweredSafety() {
    Setup();
    double first_strong = -1.0;
    bool tested_released_safety = false;
    for(int i = 0; i <= 120; ++i) {
        const double t = i * 0.05;
        const bool safety = i >= 4 && ((i - 4) / 6) % 2 == 0;
        Step(t, GEAR_D, true, 3.0, 3.0, 0, false, safety);
        if(first_strong < 0 && output.brakePercent == 100) first_strong = t;
        if(safety) Check(output.brakePercent == 100, "each independent safety request applies immediately");
        if(first_strong >= 0) Check(output.throttlePercent == 0, "unanswered safety request remains interlocked between repeated requests");
        if(first_strong >= 0 && t >= first_strong + 2.05 - 1e-9) {
            Check(output.brakePercent == 100, "repeated safety cancellation/reissue must not postpone first unanswered strong-request timeout");
            if(!safety) tested_released_safety = true;
        }
    }
    Check(first_strong >= 0 && tested_released_safety, "test must cover unanswered strong timeout while safety input is false");
}

void OrdinaryThroughBriefInputGap() {
    Setup();
    double first_brake = -1.0;
    bool tested_stale_hold = false;
    bool cancelled = false;
    for(int i = 0; i <= 200; ++i) {
        const double t = i * 0.05;
        const bool gap = first_brake >= 0 && t >= first_brake + 0.15 - 1e-9 &&
            t < first_brake + 0.5 - 1e-9;
        cancelled = first_brake >= 0 && t >= first_brake + 0.5 - 1e-9;
        const double target = cancelled ? 2.5 : (t < 0.2 ? 2.0 : 1.0);
        Step(t, GEAR_D, true, 2.0, target, 0, false, false, !gap, !gap);
        if(first_brake < 0 && output.brakePercent > 0) first_brake = t;
        Check(output.brakePercent <= 3, "brief stale hold must not reclassify an unanswered ordinary request as a hard-brake fault");
        if(first_brake >= 0) Check(output.throttlePercent == 0, "brief gap and cancelled request cannot infer unseen hydraulic release");
        if(gap && t >= first_brake + 0.4 - 1e-9 && output.brakePercent == 3)
            tested_stale_hold = true;
    }
    Check(first_brake >= 0 && tested_stale_hold && cancelled, "test must create ordinary request, stale hold and fresh target cancellation");
}

void ReverseOrdinaryPending(bool still_overspeed) {
    Setup();
    bool reverse_small_request = false;
    bool fault_stop = false;
    for(int i = 0; i <= 120; ++i) {
        const double t = i * 0.05;
        const bool reverse = i < 50;
        // 切D时重新提供完整有效直线路径，避免测试结果由R遗留路径几何主导。
        if(i == 50) SetGeometry();
        Step(t, reverse ? GEAR_R : GEAR_D, true, 3.0,
             reverse ? 0.5 : (still_overspeed ? 1.0 : 3.5), 0);
        if(reverse) {
            Check(output.brakePercent > 0 && output.brakePercent <= 5,
                  "R phase must publish its original ordinary 1-to-5 percent brake");
            reverse_small_request = reverse_small_request || output.brakePercent > 0;
        } else {
            Check(output.throttlePercent == 0, "unanswered R ordinary brake must interlock new D propulsion");
            if(!still_overspeed)
                Check(output.brakePercent < 100, "unanswered small R request is not a strong-request failure after D switch");
            else {
                if(t < 3.5) Check(output.brakePercent < 100, "R-to-D physical failure needs new continuous D deceleration evidence");
                if(output.brakePercent == 100) fault_stop = true;
            }
        }
    }
    Check(reverse_small_request, "R-to-D case must originate from an actual published small R brake");
    if(still_overspeed) Check(!fault_stop, "small unanswered R request remains unknown in D instead of declaring hardware failure");
}

void SmallUnanswered() {
    Setup();
    bool requested = false;
    for(int i = 0; i <= 200; ++i) {
        const double t = i * 0.05;
        Step(t, GEAR_D, true, 3.0, t < 0.2 ? 3.0 : 2.65, 0);
        if(output.brakePercent > 0) requested = true;
        Check(output.brakePercent <= 3, "unanswered small request without significant overspeed must not jump to hard brake");
        if(requested) Check(output.throttlePercent == 0, "active unanswered small request cannot release by timeout");
    }
    Check(requested, "small-deficit case must actually enter ordinary hydraulic request");
}

void ObservationAfterNavigationGap() {
    Setup();
    bool ordinary_seen = false;
    for(int i = 0; i <= 40; ++i) {
        const double t = i * 0.05;
        const bool fresh = i <= 8 || i >= 20;
        Step(t, GEAR_D, true, 2.0, t < 0.2 ? 2.0 : 0.5, 0, false, false, fresh);
        if(i == 22) {
            for(int repeat = 0; repeat < 30; ++repeat) {
                Step(t, GEAR_D, true, 2.0, 0.5, 0);
                Check(output.brakePercent == 0, "same-time navigation callbacks do not confirm continuous motor deficit");
            }
        }
        if(t < 1.35) Check(output.brakePercent == 0, "navigation gap requires a new continuous deficit observation");
        if(output.brakePercent > 0 && output.brakePercent < 100) ordinary_seen = true;
    }
    Check(ordinary_seen, "valid continuous feedback eventually permits ordinary braking after gap");
}

void FailureNeedsNewNavigation() {
    Setup();
    double first_brake = -1.0;
    bool cancelled = false;
    bool fault_after_new_sample = false;
    for(int i = 0; i <= 180; ++i) {
        const double t = i * 0.05;
        if(first_brake >= 0 && t >= first_brake + 0.2 - 1e-9) cancelled = true;
        double target = t < 0.2 || cancelled ? 2.0 : 1.0;
        if(i >= 120) target = 0.5;
        // 目标在6秒降低；最后一次新加速度估计在6.9秒，
        // 7.0/7.05只有控制/CAN继续推进，导航仍在允许时效内。
        const bool fresh = i != 140 && i != 141;
        Step(t, GEAR_D, true, 2.0, target, 0, false, false, fresh);
        if(first_brake < 0 && output.brakePercent > 0) first_brake = t;
        if(i <= 141) Check(output.brakePercent < 100, "one-second failed deceleration needs new navigation evidence, not control ticks");
        else if(output.brakePercent == 100) fault_after_new_sample = true;
        if(cancelled && i < 142) Check(output.throttlePercent == 0, "unknown execution remains interlocked throughout observation");
    }
    Check(first_brake >= 0, "failure-confirmation case must start with a real ordinary command");
    Check(!fault_after_new_sample, "fresh speed error alone cannot identify an uncalibrated small-command deadzone as hardware failure");
}

int main(int argc, char** argv) {
    if(argc != 2) return 2;
    const std::string scenario = argv[1];
    if(scenario == "manual_n_hold" || scenario == "manual_n_empty" || scenario == "manual_n_moving_hold") OldManualFeedback(scenario);
    else if(scenario == "auto_n_unanswered") NeutralPending(false);
    else if(scenario == "auto_n_delayed_response") NeutralPending(true);
    else if(scenario == "clock_rewind") ClockRewind();
    else if(scenario == "unknown_cancelled") UnknownCancelled(false, false);
    else if(scenario == "unknown_late_response") UnknownCancelled(true, false);
    else if(scenario == "unknown_safety") UnknownCancelled(false, true);
    else if(scenario == "failed_deceleration") FailedDeceleration(false);
    else if(scenario == "cancelled_still_overspeed") FailedDeceleration(true);
    else if(scenario == "cancelled_still_overspeed_late_response") FailedDeceleration(true, true);
    else if(scenario == "small_unanswered") SmallUnanswered();
    else if(scenario == "observation_navigation_gap") ObservationAfterNavigationGap();
    else if(scenario == "failure_new_navigation") FailureNeedsNewNavigation();
    else if(scenario == "saturation_new_navigation") SaturationNeedsNewNavigation();
    else if(scenario == "repeated_unanswered_safety") RepeatedUnansweredSafety();
    else if(scenario == "ordinary_brief_input_gap") OrdinaryThroughBriefInputGap();
    else if(scenario == "reverse_ordinary_unknown") ReverseOrdinaryPending(false);
    else if(scenario == "reverse_ordinary_failed_deceleration") ReverseOrdinaryPending(true);
    else return 3;
    std::cout << "RESULT " << scenario << " checks=" << checks << " failures=" << failures << '\n';
    return failures ? 1 : 0;
}
