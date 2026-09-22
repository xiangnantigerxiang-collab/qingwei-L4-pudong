// 真实C++状态机回归。私有访问只用于曲率/参考的独立边界；执行器用公开消息入口。
#include "forward_smooth_access.h"
#define main historical_brake_state_main
#include "brake_state_revision.cpp"
#undef main
#include <limits>

void CachedPath(bool noise, double offset = 0.0) {
    control.mPathList.clear();
    for(int i = 0; i <= 500; ++i) {
        XYZ_COOR_S p = {};
        p.x_axis = offset + i * 0.1;
        p.y_axis = 0.2 + (noise ? 0.002 * std::sin(p.x_axis * 6.283185307) : 0.0);
        control.mPathList.push_back(p);
    }
    control.CalcuForwardPathCurve();
}

void AddPeak() {
    for(std::size_t i = 0; i < control.mForwardPathList.size(); ++i) {
        auto& p = control.mForwardPathList[i];
        p.curvature = std::fabs(p.x_axis - 15.0) < 0.4 ? 0.04 : 0.0;
        const double v = control.ForwardCurvatureSpeed(p.curvature);
        control.mForwardCurveSpeedSquares[i] = v * v;
    }
}

void SpatialPreview() {
    Setup();
    Step(0, GEAR_D, true, 4, 4, 0);
    CachedPath(false);
    AddPeak();
    control.mKeyPoint = 0;
    const double initial = control.ForwardCurveLimitSpeed(control.mForwardPathList);
    const double initial_soft = control.mForwardSpeed.softCurveLimit;
    for(double speed : {3.5, 3.0, 2.5, 1.0}) {
        control.mVehicleSpeed = speed;
        const double limit = control.ForwardCurveLimitSpeed(control.mForwardPathList);
        Check(std::fabs(initial - limit) < 1e-6, "same unpassed curve cannot release just because 4v shrinks");
        Check(std::fabs(initial_soft - control.mForwardSpeed.softCurveLimit) < 1e-6,
              "comfort buffer cannot shrink and undo deceleration");
    }
    CachedPath(false, 0.1);
    AddPeak();
    Check(control.mForwardSpeed.endValid, "same-route resampling reprojects retained endpoint");
    control.ForwardCurveLimitSpeed(control.mForwardPathList);
    Check(control.mForwardSpeed.previewCurvature > 0.039, "resampling retains the unpassed peak");
    control.mForwardPathList[0].curvature = 0.16;
    Check(control.ForwardCurveLimitSpeed(control.mForwardPathList) <= 0.71,
          "new immediate tight curve cannot be diluted by history or recovery smoothing");
    control.ResetForwardCurveHistory();
    AddPeak();
    control.mNavData.xAxis = 16.2;
    control.mKeyPoint = 161;
    Check(control.ForwardCurveLimitSpeed(control.mForwardPathList) > 5,
          "actually passing peak releases straight speed without indefinite low cap");
    CachedPath(false, 100.0);
    Check(!control.mForwardSpeed.endValid, "unrelated geometry invalidates retained endpoint");
}

void CalibrationGeometry() {
    Setup();
    Step(0, GEAR_D, true, 5, 5, 0);
    Check(output.throttlePercent == 67 && output.brakePercent == 0, "5m/s straight keeps integer 13.5 mapping");
    control.ResetForwardCurveHistory();
    CachedPath(true);
    control.mKeyPoint = 0;
    Check(control.ForwardCurveLimitSpeed(control.mForwardPathList) >= 5,
          "millimetre trajectory noise must not suppress 5m/s straight travel");
    for(int sign : {-1, 1}) {
        control.ResetForwardCurveHistory();
        control.mPathList.clear();
        for(int i = 0; i < 500; ++i) {
            XYZ_COOR_S p = {};
            p.x_axis = 25 * std::sin(i * 0.004);
            p.y_axis = sign * 25 * (1 - std::cos(i * 0.004));
            control.mPathList.push_back(p);
        }
        control.CalcuForwardPathCurve();
        control.mNavData.xAxis = control.mNavData.yAxis = 0;
        control.mKeyPoint = 0;
        const double cap = control.ForwardCurveLimitSpeed(control.mForwardPathList);
        Check(std::fabs(cap - 1.4) < 0.005, "left/right radius25m retains calibrated 1.4m/s curve speed");
    }
    control.mPathList.assign(3, XYZ_COOR_S{});
    control.CalcuForwardPathCurve();
    Check(control.mForwardPathList.empty(), "degenerate geometry invalidates D speed cache");
}

void MotorReference() {
    Setup();
    Step(0, GEAR_D, true, 4, 4, 0);
    double previous = control.mForwardSpeed.reference;
    double previous_acceleration = control.mForwardSpeed.referenceAcceleration;
    control.mForwardSpeed.softCurveLimit = 3;
    for(int i = 1; i <= 100; ++i) {
        ros::testTime() = 100 + i * 0.05;
        control.SetCanData(can_data);
        control.SetNavigationData(navigation);
        const double v = control.SmoothForwardSpeed(4);
        Check(v <= 4 && v >= 3, "anticipated curve reference stays inside feasible speed range");
        Check(previous - v <= 0.03001 && v <= previous + 1e-9, "normal curve command has bounded monotone descent");
        Check(std::fabs(control.mForwardSpeed.referenceAcceleration - previous_acceleration) <= 0.02501,
              "uninterrupted constant comfort target respects internal jerk limit");
        previous_acceleration = control.mForwardSpeed.referenceAcceleration;
        previous = v;
    }
    Check(std::fabs(previous - 3) < 1e-5, "normal reference reaches curve speed");
    control.mForwardSpeed.softCurveLimit = 600;
    for(int i = 101; i <= 200; ++i) {
        ros::testTime() = 100 + i * 0.05;
        control.SetCanData(can_data);
        control.SetNavigationData(navigation);
        const double v = control.SmoothForwardSpeed(4);
        Check(v - previous <= 0.02001 && v >= previous - 1e-9, "recovery acceleration does not surge");
        Check(std::fabs(control.mForwardSpeed.referenceAcceleration - previous_acceleration) <= 0.02501,
              "uninterrupted recovery respects internal jerk limit");
        previous_acceleration = control.mForwardSpeed.referenceAcceleration;
        previous = v;
    }
    Check(std::fabs(previous - 4) < 1e-5, "recovery does not leave a persistent straight underspeed");
    Check(control.SmoothForwardSpeed(1) <= 1, "abrupt hard ceiling preempts comfort reference immediately");
    ros::testTime() += 0.3;
    Check(control.SmoothForwardSpeed(4) <= 1, "stale feedback freezes recovery instead of raising motor target");
    ros::testTime() += 0.05;
    control.SetCanData(can_data);
    control.SetNavigationData(navigation);
    Check(control.SmoothForwardSpeed(4) <= 1.01, "first fresh feedback continues recovery without reinitializing high");
    Step(11, GEAR_D, true, 4, 4, 0, false, true);
    Check(output.throttlePercent == 0 && output.brakePercent == 100, "independent safety preempts smoothing");
}

void TerminalDistance() {
    Setup();
    Step(0, GEAR_D, true, 0.2935, 0, 0, true);
    Check(output.brakePercent >= 5 && output.brakePercent < 80, "normal terminal begins gently");
    for(int i = 1; i <= 12; ++i) {
        ros::testTime() = 100 + i * 0.05;
        can_data.brakePercent = 5;
        navigation.gpsSpeed = i < 5 ? 0.29 : (i % 2 ? 0.02 : 0.04);
        control.SetCanData(can_data);
        control.SetNavigationData(navigation);
        status.distance2Stop = -9.76447;
        control.SetPathStatusData(status);
        control.VehicleControl();
        control.PublishMessage(publisher);
        if(i < 11) Check(output.brakePercent < 80, "distance recalculation does not interrupt a valid terminal stop");
    }
    Check(output.brakePercent >= 80, "low-speed hysteresis confirms stop then applies hold");
    control.mForwardBrake.terminalLatched = false;
    control.mControlData.brakePercent = 0;
    Check(control.ForwardTerminalStopBrake() >= 80, "negative distance cannot authorize first terminal entry");
    status.distance2Stop = 0.2;
    control.SetPathStatusData(status);
    control.mControlData.brakePercent = 0;
    control.ForwardTerminalStopBrake();
    navigation.xAxis = 2;
    ros::testTime() += 0.05;
    control.SetNavigationData(navigation);
    Check(control.ForwardTerminalStopBrake() >= 80, "real displacement/overshoot must reject latched comfort stop");
}

void ManualNeutralReset() {
    Setup();
    for(int i = 0; i < 55; ++i) Step(i * 0.05, GEAR_D, true, 2, 2, 0, false, true);
    Check(control.mForwardBrake.fault, "test establishes real unanswered strong-request fault");
    for(int i = 55; i < 80; ++i) Step(i * 0.05, GEAR_N, false, 0, 0, 0, true);
    Check(!control.mForwardBrake.fault && !control.mForwardBrake.pending, "manual N fresh stopped-zero feedback clears D fault");
    Step(4, GEAR_D, true, 0, 1, 0);
    Check(output.brakePercent == 0 && output.throttlePercent >= 5, "return to D after manual N does not relatch fault");
}

void ConfirmedRelease(bool increase) {
    Setup();
    double first = -1, resumed = -1, increased_at = -1;
    int previous = 0;
    for(int i = 0; i < 110; ++i) {
        const double t = i * 0.05;
        const double age = first < 0 ? -1 : t - first;
        const double cancel_age = increase ? 1.2 : 0.4;
        const int feedback = age >= 0.2 && age < cancel_age + 0.2 ? 3 : 0;
        Step(t, GEAR_D, true, 3, t < 0.2 || age >= cancel_age ? 3 : 1, feedback);
        if(first < 0 && output.brakePercent > 0) first = t;
        if(output.brakePercent > previous && previous > 0) increased_at = t;
        previous = output.brakePercent;
        if(first >= 0 && output.throttlePercent > 0 && resumed < 0) resumed = t;
        Check(output.brakePercent < 100, "ordinary successful release never requires fault brake");
        if(feedback > 0) Check(output.throttlePercent == 0, "actual hydraulic response always blocks motor");
    }
    Check(first >= 0 && resumed >= 0, "test traverses actual request/response/withdrawal/recovery");
    if(increase) {
        Check(increased_at > first && resumed - increased_at >= 1.8 - 1e-6,
              "unresolved increased command retains its late-response window");
    } else {
        Check(resumed - first < 1.3, "constant request releases on fresh confirmed zero without old 1.8s hold");
    }
}

void PlanningRipple() {
    Setup();
    double first = -1;
    for(int i = 0; i < 35; ++i) {
        const double t = i * 0.05;
        const double target = i < 4 ? 2 : (i % 2 ? 1.1 : 1.0);
        Step(t, GEAR_D, true, 2, target, 0);
        if(first < 0 && output.brakePercent > 0) first = t;
    }
    Check(first >= 0 && first < 1.4, "float 0.1 planning rebound cannot repeatedly restart observation");
}

void SufficientRegeneration() {
    Setup();
    for(int i = 0; i < 80; ++i) {
        const double t = i * 0.05;
        const double v = std::max(1.0, 4.0 - 0.9 * t);
        Step(t, GEAR_D, true, v, i < 4 ? 4 : 1, 0);
        Check(output.brakePercent == 0, "large error alone cannot add hydraulic when motor deceleration is sufficient");
    }
}

void DelayedRegeneration() {
    Setup();
    for(int i = 0; i < 55; ++i) {
        const double t = i * 0.05;
        // 指令下降后先有0.3s执行/测量延迟，再建立强电机减速；不能被低通延迟抢先补刹。
        const double v = std::max(2.4, 4.0 - std::max(0.0, t - 0.5) * 2.0);
        Step(t, GEAR_D, true, v, i < 4 ? 4 : 2.4, 0);
        Check(output.brakePercent == 0, "delayed motor onset is observed before adding hydraulic assistance");
    }
}

void TerminalCallbackEvidence() {
    Setup();
    Step(0, GEAR_D, true, 0.2935, 0, 0);
    ros::testTime() += 0.01;
    status.taskExecuStatus = TASKFINISHED;
    status.distance2Stop = 0.4157;
    control.SetPathStatusData(status);
    status.distance2Stop = -9.76447;
    ros::testTime() += 0.03;
    control.SetPathStatusData(status);
    control.VehicleControl();
    control.PublishMessage(publisher);
    Check(output.brakePercent >= 5 && output.brakePercent < 80,
          "valid completed evidence survives two status callbacks before one control tick");
    task.pathList.push_back("new_route_same_task");
    control.setTaskPlanData(task);
    Check(!control.mForwardBrake.terminalLatched, "same task ID with changed route clears terminal evidence");
    control.VehicleControl();
    control.PublishMessage(publisher);
    Check(output.brakePercent >= 80, "new route cannot reuse old completed evidence for negative distance");
}

int main(int argc, char** argv) {
    if(argc != 2) return 2;
    const std::string name = argv[1];
    if(name == "spatial_preview") SpatialPreview();
    else if(name == "geometry") CalibrationGeometry();
    else if(name == "motor") MotorReference();
    else if(name == "terminal") TerminalDistance();
    else if(name == "manual_n_reset") ManualNeutralReset();
    else if(name == "release") ConfirmedRelease(false);
    else if(name == "increase_release") ConfirmedRelease(true);
    else if(name == "planning_ripple") PlanningRipple();
    else if(name == "regeneration") SufficientRegeneration();
    else if(name == "delayed_regeneration") DelayedRegeneration();
    else if(name == "terminal_callbacks") TerminalCallbackEvidence();
    else return 3;
    std::cout << "RESULT " << name << " checks=" << checks << " failures=" << failures << '\n';
    return failures ? 1 : 0;
}
