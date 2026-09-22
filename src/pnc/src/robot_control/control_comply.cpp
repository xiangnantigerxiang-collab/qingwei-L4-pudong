// 控制编排器 ControlComply 实现：路径预处理、VehicleControl
// 主流程（横纵向调度）、Stanley/R 挡几何入口和控制消息输出。
// ACC/AEB 纵向控制实现位于 longitudinal_acc_control.inc。

#include "control_comply.h"

namespace {
// D挡普通参考的工程初值；安全速度上限下降始终可抢占，不是实车减速度保证。
const double kForwardCurveDeceleration = 0.6;      // m/s²，提前收速参考。
const double kForwardCurveHardDeceleration = 0.8;  // m/s²，沿程速度包络。
const double kForwardDriveAcceleration = 0.4;      // m/s²，减速后的恢复。
const double kForwardMotorJerk = 0.5;              // m/s³，内部连续参考。
const double kForwardMotorDelay = 0.35;            // s，电机/导航链路预留，待实车标定。
}

ControlComply::ControlComply() {
    mPathList.clear();
    mGear = GEAR_N;
    mSpeed = 0;
    mKeyPoint = 0;
    mTlStatus.light_status = 2;
}

ControlComply::~ControlComply() {
}

// ---------------------------------------------------------------------------
// ROS 消息输入
// ---------------------------------------------------------------------------

void ControlComply::SetPathStatusData(robot::path_plan_status path_status_t) {
    mPathStatus = path_status_t;
    // 两条状态可能在同一控制周期内先有效到位、再重算距离；到位证据不能只在20Hz主循环采集。
    if(mGear == GEAR_D && !mForwardBrake.terminalLatched &&
       mPathStatus.taskExecuStatus == TASKFINISHED) ForwardTerminalStopBrake();
    if(mGear == GEAR_D && mPathStatus.taskExecuStatus != TASKFINISHED) {
        mForwardBrake.terminalLatched = false;
        mForwardBrake.terminalTime = mForwardBrake.stoppedSince = -1.0;
    }
}

void ControlComply::setTaskPlanData(robot::task_plan_msg task_info) {
    if(task_info.task_id != mTaskInfo.task_id || task_info.taskType != mTaskInfo.taskType ||
       task_info.desireGear != mTaskInfo.desireGear) {
        ResetBrakeIntegral();
        ResetForwardCurveHistory();
        mForwardBrake.terminalLatched = false;
    } else if(mGear == GEAR_D && (mForwardCurveHistoryValid || mBrakePlanningSpeedValid ||
              mForwardBrake.terminalLatched) &&
              task_info.pathList != mTaskInfo.pathList) {
        // 同 ID 更换路线也不能沿用上一条路线的减速请求。
        ResetBrakeIntegral();
        ResetForwardCurveHistory();
        mForwardBrake.terminalLatched = false;
    }
    mTaskInfo = task_info;
}

void ControlComply::SetCanData(robot::can_msg can_msg_t) {
    const bool automatic = can_msg_t.controlPanelState == 1;
    const bool gear_changed = mGear != can_msg_t.curGear;
    if(mBrakeAutomatic != automatic) mForwardBrake.activeCommand = false;
    if(mGear != can_msg_t.curGear || mBrakeAutomatic != automatic) {
        ResetBrakeIntegral();
        ResetForwardCurveHistory();
        mForwardBrake.terminalTime = mForwardBrake.stoppedSince = -1.0;
        mForwardBrake.terminalLatched = false;
        mForwardBrake.accelerationValid = false;
        mForwardBrake.navigationTime = -1.0;
    }
    mBrakeAutomatic = automatic;
    mGear = can_msg_t.curGear;
    if(gear_changed) {
        if(mGear == GEAR_D) {
            CalcuForwardPathCurve();
        } else {
            mForwardPathList.clear();
            mForwardPathDistances.clear();
            mForwardCurveSpeedSquares.clear();
        }
    }
    mSteerAngle = -can_msg_t.wheelAngle / 22.0 / 180 * 3.1415926;
    mLastCanTime = ros::Time::now().toSec();
    mCanEmergencyStop = can_msg_t.emergencyStop != 0;
    if(mCanEmergencyStop) ResetBrakeIntegral();
    ObserveForwardBrakeFeedback(can_msg_t.brakePercent, mLastCanTime);
    RecoverForwardBrakeIfStopped(mLastCanTime);
}

void ControlComply::SetNavigationData(robot::navigation_msg navigation_t) {
    const double now = ros::Time::now().toSec();
    if(mGear == GEAR_D && mLastNavigationTime >= 0.0 &&
       (mForwardSpeed.endValid || mForwardSpeed.referenceValid)) {
        const double dt = now - mLastNavigationTime;
        const double displacement = std::hypot(double(navigation_t.xAxis) - mNavData.xAxis,
                                               double(navigation_t.yAxis) - mNavData.yAxis);
        if(!std::isfinite(displacement) || dt < 0.0 ||
           (dt <= 0.2 && displacement > std::max(2.0, std::fabs(double(mVehicleSpeed)) * dt * 2.0 + 0.5))) {
            ResetForwardCurveHistory();
            mForwardBrake.terminalLatched = false;
        }
    }
    mNavData = navigation_t;
    // 实际车速统一取导航；CAN 回调只更新挡位、转角和安全状态。
    mVehicleSpeed = navigation_t.gpsSpeed;
    mControlData.vehicleSpeed = mVehicleSpeed;
    mLastNavigationTime = now;
    UpdateForwardAcceleration(mVehicleSpeed, mLastNavigationTime);
    RecoverForwardBrakeIfStopped(mLastNavigationTime);
    ego_pose2d.x = mNavData.xAxis;
    ego_pose2d.y = mNavData.yAxis;
    ego_pose2d.heading = azimuthToYaw(mNavData.heading);
}

void ControlComply::SetPathPlanData(robot::path_plan_msg path_plan_t) {
    XYZ_COOR_S xyz_temp;
    std::vector<XYZ_COOR_S> src_path;
    if(mGear == GEAR_D && mForwardBrake.terminalLatched && mPathid != path_plan_t.Path_Id)
        mForwardBrake.terminalLatched = false;

    if(path_plan_t.x.size() != path_plan_t.y.size()) {
        ResetBrakeIntegral();
        if(mGear == GEAR_D) ResetForwardCurveHistory();
        return;
    }

    src_path.clear();

    // 只用原始规划目标的下降沿发起积分减速；实际超速、曲率/故障限速不能自行发起。
    // 在回调中比较，避免一个控制周期内多条规划消息的下降/回升被遗漏。
    if(!mBrakeAutomatic || mGear != GEAR_D || mCanEmergencyStop || path_plan_t.safety ||
       !std::isfinite(path_plan_t.desireSpeed) || path_plan_t.desireSpeed < 0.0) {
        ResetBrakeIntegral();
    } else {
        if(mBrakePlanningSpeedValid && (path_plan_t.Path_Id != mPathid ||
           (path_plan_t.desireSpeed > mSpeed && (!mBrakeDecelerationRequested ||
            double(path_plan_t.desireSpeed) > mForwardBrake.targetFloor + 0.1 + 1e-5)))) {
            ResetBrakeIntegral();
        }
        if(mBrakePlanningSpeedValid && path_plan_t.desireSpeed < mSpeed) {
            if(!mBrakeDecelerationRequested) {
                mForwardBrake.requestTime = ros::Time::now().toSec();
                mForwardBrake.targetFloor = path_plan_t.desireSpeed;
            }
            mBrakeDecelerationRequested = true;
        }
        if(mBrakeDecelerationRequested) {
            mForwardBrake.targetFloor = std::min(mForwardBrake.targetFloor,
                                                double(path_plan_t.desireSpeed));
        }
        mBrakePlanningSpeedValid = true;
    }
    mSpeed = path_plan_t.desireSpeed;
    mPathid = path_plan_t.Path_Id;
    mPathsafety = path_plan_t.safety;

    robot::path_plan_msg path_recv = path_plan_t;
    // 反转路径顺序
    std::reverse(path_recv.x.begin(), path_recv.x.end());
    std::reverse(path_recv.y.begin(), path_recv.y.end());

    if(path_recv.x.size() < 10) {
        ResetBrakeIntegral();
        if(mGear == GEAR_D) ResetForwardCurveHistory();
        return;
    }

    robot::path_plan_msg path_filter;
    int psize = path_recv.x.size();

    for(int i = 0; i < psize; i++) {
        if(i == 0) {
            path_filter.x.push_back(path_recv.x[i]);
            path_filter.y.push_back(path_recv.y[i]);
            continue;
        }

        int fsize = path_filter.x.size();

        double x0 = path_filter.x[fsize - 1];
        double y0 = path_filter.y[fsize - 1];
        double x1 = path_recv.x[i];
        double y1 = path_recv.y[i];
        // 与路径起点(因为路径反转了)的距离
        double len = hypot(x1 - x0, y1 - y0);

        // 取与路径终点的距离大于0.1m或者小于3.0m的点
        if(len > 0.1 && len < 3.0) {
            path_filter.x.push_back(path_recv.x[i]);
            path_filter.y.push_back(path_recv.y[i]);
        }
    }

    std::reverse(path_filter.x.begin(), path_filter.x.end());
    std::reverse(path_filter.y.begin(), path_filter.y.end());
    path_plan_t = path_filter;

    if(path_plan_t.x.size() < 5) {
        ResetBrakeIntegral();
        if(mGear == GEAR_D) ResetForwardCurveHistory();
        return;
    }

    if(mPathid == 20) {
        CSpline spline;
        for(int i = 0; i < path_plan_t.x.size(); ++i) {
            xyz_temp.x_axis = path_plan_t.x[i];
            xyz_temp.y_axis = path_plan_t.y[i];
            src_path.push_back(xyz_temp);
        }

        mPathList.clear();
        mKeyPoint = 0;
        spline.SplinePointSet(src_path, mPathList, 0.1);
    } else {
        CSpline spline;

        for(int i = 0; i < path_plan_t.x.size(); ++i) {
            xyz_temp.x_axis = path_plan_t.x[i];
            xyz_temp.y_axis = path_plan_t.y[i];
            src_path.push_back(xyz_temp);
        }

        mPathList.clear();
        mKeyPoint = 0;
        spline.SplinePointSet(src_path, mPathList, 0.1);
    }

    CalcuPathHead(mPathList);
    CalcuPathCurve(mPathList);
    if(mGear == GEAR_D) CalcuForwardPathCurve();

    if(mGear == GEAR_D && mForwardCurvePathId != mPathid) {
        ResetForwardCurveHistory();
        mForwardCurvePathId = mPathid;
    }

    mControlData.bypassProcessing = path_plan_t.bypassProcessing;
}

bool ControlComply::IsGreenLight(uint8_t light_state) {
    static uint8_t light_buffer[5] = {0};

    for(int i = 0; i < 4; ++i)
        light_buffer[i] = light_buffer[i + 1];

    light_buffer[4] = light_state;

    for(int i = 0; i < 5; ++i) {
        if(light_buffer[i] != 2) return false;
    }

    return true;
}

void ControlComply::SetTlStatusData(robot::TLStatus tl_status) {
    mTlStatus = tl_status;
}

// ---------------------------------------------------------------------------
// 20 Hz 控制主流程
//
// 执行顺序：停车门控 -> 位姿/纵向基础量 -> 横向控制 -> 速度约束 ->
// 传感器与安全覆盖 -> 输出限幅。顺序会影响最终控制量，请勿随意调整。
// ---------------------------------------------------------------------------

void ControlComply::VehicleControl() {
    if(mGear != GEAR_D) ResetForwardCurveHistory();
    if(mGear != GEAR_D || mPathStatus.taskExecuStatus != TASKFINISHED ||
       fabs(mSpeed) >= 0.5 || mPathsafety) {
        mTerminalBrakeStartTime = -1.0;
        if(mGear == GEAR_D) {
            mForwardBrake.terminalTime = -1.0;
            mForwardBrake.stoppedSince = -1.0;
            mForwardBrake.terminalLatched = false;
        }
    }

    if(mGear == GEAR_D && (mCanEmergencyStop || !std::isfinite(mSpeed) || mSpeed < 0.0)) {
        mControlData.desireSpeed = 0;
        mControlData.desireAcc = 0;
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 100;
        mControlData.wheelAngle = 0;
        ResetForwardCurveHistory();
        ResetBrakeIntegral();
        return;
    }

    // D 挡 safety 直接急停；其他挡位保留原目标速度/路径编号的停车判定。
    if(mPathsafety && (mGear == GEAR_D || fabs(mSpeed) < 0.1)) {
        printf("mPathsafety:%d\n", mPathsafety);
        mControlData.desireSpeed = 0;
        mControlData.desireAcc = 0;
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 100;
        mControlData.wheelAngle = 0;
        ResetForwardCurveHistory();
        ResetBrakeIntegral();
        return;
    }

    if((fabs(mSpeed) < 0.5 && mPathStatus.taskExecuStatus == 2) ||
       mGear == GEAR_N) {
        printf("mGear:%d, execute stop:%d\n", mGear,
               mPathStatus.taskExecuStatus);
        mControlData.desireSpeed = 0;
        mControlData.desireAcc = 0;
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = TerminalStopBrake();
        mControlData.wheelAngle = 0;
        ResetForwardCurveHistory();
        ResetBrakeIntegral();
        return;
    }

    if(mPathList.empty()) {
        printf("空路径\n");
        mControlData.desireSpeed = 0;
        mControlData.desireAcc = 0;
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 50;
        mControlData.wheelAngle = 0;
        ResetForwardCurveHistory();
        ResetBrakeIntegral();
        return;
    }

    // 保留上方既有停车优先级；导航异常值不能进入行驶计算，也不能回退到已弃用的 CAN 速度。
    if(!std::isfinite(mVehicleSpeed)) {
        ROS_WARN_THROTTLE(1.0, "Invalid navigation speed, stopping: %f", mVehicleSpeed);
        mControlData.desireSpeed = 0;
        mControlData.desireAcc = 0;
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 100;
        mControlData.wheelAngle = 0;
        mTerminalBrakeStartTime = -1.0;
        ResetForwardCurveHistory();
        ResetBrakeIntegral();
        return;
    }
    // 这里计算了距离车辆最近点的曲率和横向误差赋值给了mControlData
    // (但是这里并没有用到预瞄参数)
    VehiclePoseCalculation();
    // 这里的mSpeed是通过path_plan_msg传入的期望速度
    mControlData.desireSpeed = mSpeed;
    // 这里基于期望速度和当前速度的差值调用模糊pid算法计算了期望加速度
    mControlData.desireAcc = spCtr_c.AccelerationCalculateBySpeed_P(
        mControlData.desireSpeed, mVehicleSpeed);

    // 这里根据期望速度和期望加速度计算了油门和刹车的百分比
    // (但是这里并没有用到期望加速度)，被后面代码覆盖，未使用
    VehicleVerticalControl(mControlData.desireSpeed, mNavData.gpsSpeed,
                           mControlData.desireAcc, mControlData.throttlePercent,
                           mControlData.brakePercent);

    if(mPathid == 5 && mPathsafety) {
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 100;
        ResetForwardCurveHistory();
    } else {
        // 横向控制，输出转向角
        if(mGear == GEAR_R) {
            mControlData.wheelAngle = VehicleLateralControl();
        } else {
            mControlData.wheelAngle = VehicleStanleyControl();
        }
        //
        {
            auto path2d = toPath2d(mPathList);
            auto foot_pose = math_utils::getFootPose(ego_pose2d, path2d);
            double lat_dev = ego_pose2d.distanceTo(foot_pose);
            double angle_dev = fabs(ego_pose2d.heading - foot_pose.heading);
            mControlData.biaDistance = lat_dev;
            mControlData.biaAngle = angle_dev;
        }

        // 处理指令边界
        double speed_now = mVehicleSpeed;
        double speed_cmd = mSpeed;  // 期望速度，路径发下来的
        double delta = speed_cmd - speed_now;

        // 恢复原速度差上限：导航当前车速+0.5m/s；随后仍与曲率/故障限速取小。
        if(delta > 0.5) speed_cmd = speed_now + 0.5;

        // D挡按导航当前车速的4秒行程预瞄曲率限速；R挡继续执行原曲率函数和后续油门覆盖。
        double curvelimitspeed = mGear == GEAR_D
            ? (mForwardPathList.size() == mPathList.size() ? ForwardCurveLimitSpeed(mForwardPathList) : 0.0)
            : CurveLimitSpeed(mPathList);
        speed_cmd = std::min<double>(speed_cmd, curvelimitspeed);

        //////////////////
        {
            // 缓停逻辑
            int current_state = 0;
            ros::param::get("/planning/sensorstate", current_state);
            if((current_state & 0x02) == 2 || (current_state & 0x04) == 4) {
                printf("huanman tingche\n");
                // 减速停车
                double dcc = 8.0;
                double dcc_speed = speed_now - dcc * 0.05;
                speed_cmd = std::min<double>(speed_cmd, dcc_speed);
                speed_cmd = std::max<double>(speed_cmd, 0.0);
                printf("slow down to stop, speed_cmd:%f\n", speed_cmd);
            }
            // 急刹车
            if((current_state & 0x08) == 8) {  // gnss故障，急停
                printf("gnss error, jiting\n");
                speed_cmd = 0.0;
                mControlData.brakePercent = 100;
                ResetBrakeIntegral();
            } else {
                // D 挡普通减速由规划减速请求和积分门槛控制；其他挡位保留原速度差制动。
                if(mGear != GEAR_D && delta < -1.5) {
                    if(mControlData.brakePercent < 5) {
                        mControlData.brakePercent += 1;
                    }
                } else {
                    mControlData.brakePercent = 0;
                }
            }
        }

        // dec 沿用曲率/故障限速后的速度差口径（m/s），不能使用历史 PID 的 desireAcc。
        // 是否允许积分仍由规划目标下降请求决定，本地限速不能自行触发补刹。
        const double dec = speed_now - speed_cmd;
        if(mGear == GEAR_D) {
            // 起步加速由规划负责，控制保留原 5% 低速保底和速度到油门的标定比例。
            if(speed_cmd - speed_now > 0.05 && speed_cmd * 13.5 < 5.0) {
                speed_cmd = 5.0 / 13.5;
            }
            if(speed_cmd <= 0.0) {
                speed_cmd = 0.0;
                ResetForwardCurveHistory();
            }
        }
        const double motor_reference = mGear == GEAR_D ? SmoothForwardSpeed(speed_cmd) : speed_cmd;
        mControlData.throttlePercent = mGear == GEAR_D
            ? std::max(0.0, std::min(100.0, motor_reference * 13.5)) : speed_cmd * 13.5;
        if(mGear != GEAR_D && speed_cmd - speed_now > 0.05) {
            mControlData.throttlePercent =
                mControlData.throttlePercent < 5 ? 5 : mControlData.throttlePercent;
        }
        if(mGear == GEAR_D && mControlData.brakePercent == 0) ApplyForwardDeceleration(speed_cmd);
        if(mGear != GEAR_D) printf("speed_cmd: %f, speed_now: %f, dec: %f, brake_integral: %.3f, brake_active: %d\n",
                    speed_cmd, speed_now, dec, mBrakeIntegral, 0);

        // 冲出跑道
        if(mControlData.biaDistance > 5.5 ||
           (mControlData.biaDistance > 4.0 && mControlData.biaAngle > 0.3)) {
            printf("横向偏差 > 2.5m, 停车\n");
            mControlData.brakePercent = mGear == GEAR_D
                ? std::max<int>(70, mControlData.brakePercent) : 70;
            mControlData.throttlePercent = 0;
            ResetForwardCurveHistory();
            ResetBrakeIntegral();
        }

        // 处理边界条件
        if(mControlData.brakePercent > 100) mControlData.brakePercent = 100;
        if(mControlData.throttlePercent > 100) mControlData.throttlePercent = 100;
        // 处理倒挡
        if(mGear == GEAR_R) {
            mControlData.throttlePercent = mSpeed * 10;
            if(mControlData.throttlePercent > 45) mControlData.throttlePercent = 45;
        }
    }
}

bool ControlComply::UpdateBrakeIntegral(double tDec) {
    const double now = ros::Time::now().toSec();
    // 规划目标下降才允许累计正速度误差（m/s），积分单位为 m。
    if(!mBrakeAutomatic || mGear != GEAR_D || mCanEmergencyStop ||
       !std::isfinite(tDec) || !std::isfinite(mSpeed) || !std::isfinite(mVehicleSpeed)) {
        ResetBrakeIntegral();
        return false;
    }
    if(!mBrakeDecelerationRequested || tDec <= 0.0 || mVehicleSpeed <= mSpeed) {
        // 达到规划目标后结束本次请求。保留目标比较基准，后续超速须等新的规划下降沿。
        ResetBrakeIntegral(false);
        return false;
    }
    double threshold = 0.1;  // m，仅在实测电机减速不足时积分；液压力度由D专用协调器给出。
    ros::param::getCached("/robot/control/brake_integral_threshold", threshold);
    if(!std::isfinite(threshold) || threshold <= 0.0 || threshold > 10.0) threshold = 0.1;

    const double dt = now - mBrakeIntegralTime;
    const bool active = mBrakeIntegral > threshold + 1e-9;
    const bool fresh = std::isfinite(now) && mLastCanTime >= 0 &&
                       now >= mLastCanTime && now - mLastCanTime <= 0.2 &&
                       mLastNavigationTime >= 0 && now >= mLastNavigationTime &&
                       now - mLastNavigationTime <= 0.2;
    if(!fresh || mBrakeIntegralTime < 0 || dt < 0 || dt > 0.2) {
        // 无效时间、反馈断流或控制中断不能计入未知误差，也不能单凭断流释放已接入的补刹。
        if(!active) mBrakeIntegral = 0.0;
        mBrakeIntegralTime = fresh ? now : -1.0;
        return active;
    }
    mBrakeIntegralTime = now;
    // 实际时间积分，重复周期 dt=0 不增加；限幅防止长期减速时积分无限增长。
    mBrakeIntegral = std::min(2.0 * threshold, mBrakeIntegral + tDec * dt);
    return mBrakeIntegral > threshold + 1e-9;
}

void ControlComply::ResetBrakeIntegral(bool reset_speed_history) {
    mBrakeIntegral = 0.0;
    mBrakeIntegralTime = -1.0;
    mBrakeDecelerationRequested = false;
    if(reset_speed_history) mBrakePlanningSpeedValid = false;
    mForwardBrake.requestTime = -1.0;
    mForwardBrake.demand = 0.0;
    mForwardBrake.controlTime = -1.0;
    mForwardBrake.ordinaryBrake = 0;
    mForwardBrake.saturationTime = -1.0;
    mForwardBrake.saturationSampleTime = -1.0;
    mForwardBrake.releasing = false;
    mForwardBrake.motorInsufficient = false;
    mForwardBrake.insufficientSince = mForwardBrake.insufficientSampleTime = -1.0;
    // 不清 pending/responseSeen/feedback/fault；规划复位不会取消已发出的液压指令。
}

uint8_t ControlComply::TerminalStopBrake() {
    if(mGear == GEAR_D) return ForwardTerminalStopBrake();
    const double now = ros::Time::now().toSec();
    // 仅普通 D 挡循迹已到终点、真实车速已降至蠕行时柔化最后一下制动。
    // 高速到位、反馈异常、倒车/作业任务、已有制动及任何安全停车仍按原值立即制动。
    const bool normal_stop = mGear == GEAR_D && mTaskInfo.desireGear == GEAR_D &&
        mTaskInfo.taskType == TRACKPATH && mPathStatus.taskExecuStatus == TASKFINISHED &&
        std::isfinite(mSpeed) && fabs(mSpeed) < 0.1 && !mPathsafety && !mCanEmergencyStop &&
        std::isfinite(mVehicleSpeed) && mVehicleSpeed > 0.05f && mVehicleSpeed <= 0.35f &&
        std::isfinite(mPathStatus.distance2Stop) && mPathStatus.distance2Stop >= 0 &&
        mPathStatus.distance2Stop < 0.5 &&
        std::isfinite(mControlData.biaDistance) && std::isfinite(mControlData.biaAngle) &&
        mControlData.biaDistance <= 5.5 &&
        !(mControlData.biaDistance > 4.0 && mControlData.biaAngle > 0.3) &&
        std::isfinite(now) && mLastCanTime >= 0 && now >= mLastCanTime && now - mLastCanTime <= 0.2 &&
        mLastNavigationTime >= 0 && now >= mLastNavigationTime && now - mLastNavigationTime <= 0.2;
    int sensor_state = 0, network_down = 0, planning_alive = 0;
    if(normal_stop) {
        ros::param::getCached("/planning/sensorstate", sensor_state);
        ros::param::getCached("/robot/planning/netcheck", network_down);
        ros::param::getCached("/planning/alive", planning_alive);
    }
    if(!normal_stop || sensor_state != 0 || network_down != 0 || !planning_alive ||
       (mTerminalBrakeStartTime >= 0 && now < mTerminalBrakeStartTime)) {
        mTerminalBrakeStartTime = -1.0;
        return 80;
    }
    if(mTerminalBrakeStartTime < 0) {
        if(mControlData.brakePercent > 0) return 80;
        mTerminalBrakeStartTime = now;
    }
    // 100%/s：20Hz 下首帧 5%，约 0.8s 达到 80%；以真实时间推进，卡顿不拖长建压。
    // 车速降至 0.05m/s 或安全条件变化时，上方立即返回原 80% 保持制动。
    const double brake = 100.0 * (now - mTerminalBrakeStartTime + 0.05);
    return static_cast<uint8_t>(std::min(80.0, std::max<double>(mControlData.brakePercent, brake)) + 1e-8);
}

// ---------------------------------------------------------------------------
// 控制消息输出
// ---------------------------------------------------------------------------

void ControlComply::PublishMessage(ros::Publisher& tPub) {
    if(mGear == GEAR_D) {
        FinalizeForwardBrakeCommand();
        TraceForwardLongitudinal(mForwardSpeed.lastCeiling);
    } else {
        // 只记录来源；R原有1～5%普通补刹不能在切D后被误当强停车无响应。
        const bool ordinary = mGear == GEAR_R && mControlData.brakePercent > 0 &&
            mControlData.brakePercent <= 5;
        RecordForwardBrakeCommand(mControlData.brakePercent, ordinary, ros::Time::now().toSec());
    }
    if(mControlData.throttlePercent > 0 && mControlData.brakePercent > 0) {
        robot::control_msg throttle_msg = mControlData;
        robot::control_msg brake_msg = mControlData;
        throttle_msg.brakePercent = 0;
        brake_msg.throttlePercent = 0;
        tPub.publish(throttle_msg);
        tPub.publish(brake_msg);
    } else {
        tPub.publish(mControlData);
    }
}

void ControlComply::ResetForwardCurveHistory() {
    mForwardCurvePathId = -1;
    mForwardSpeed.endValid = false;
    mForwardSpeed.nominalPreview = mForwardSpeed.retainedPreview = 0.0;
    mForwardSpeed.previewCurvature = mForwardSpeed.peakDistance = 0.0;
    mForwardSpeed.softCurveLimit = mForwardSpeed.hardCurveLimit = 600.0;
    mForwardSpeed.bufferSpeed = 0.0;
    ResetForwardMotorReference();
    if(!mForwardCurveHistoryValid) return;
    for(auto& curvature : mForwardCurvatures) curvature = 0.0;
    mForwardCurveIndex = 0;
    mForwardCurveHistoryValid = false;
}

void ControlComply::ResetForwardMotorReference() {
    mForwardSpeed.referenceValid = false;
    mForwardSpeed.reference = 0.0;
    mForwardSpeed.lastCeiling = 0.0;
    mForwardSpeed.referenceTime = -1.0;
    mForwardSpeed.referenceAcceleration = 0.0;
    mForwardSpeed.recovering = false;
}

double ControlComply::SmoothForwardSpeed(double ceiling) {
    ForwardSpeedState& state = mForwardSpeed;
    const double now = ros::Time::now().toSec();
    if(!mBrakeAutomatic || !std::isfinite(ceiling) || ceiling <= 0.0) {
        ResetForwardMotorReference();
        return ceiling;
    }
    if(mForwardBrake.pending || mForwardBrake.feedback != 0 || mControlData.brakePercent > 0) {
        ResetForwardMotorReference();
        state.recovering = true;
        return 0.0;
    }
    if(!ForwardBrakeInputsFresh(now)) {
        // 短断流不能用旧实际车速推进恢复，也不能因停用滤波突然跳回较高给定。
        state.referenceAcceleration = 0.0;
        state.recovering = true;
        state.reference = std::min(ceiling, state.referenceValid ? state.reference :
                                   std::max(0.0, double(mVehicleSpeed)));
        if(state.referenceValid) state.referenceTime = now;
        state.lastCeiling = ceiling;
        return state.reference;
    }
    const double target = std::min(ceiling, state.softCurveLimit);
    const double dt = now - state.referenceTime;
    const bool initializing = !state.referenceValid || dt < 0.0 || dt > 0.2;
    const bool curve_deceleration = target < ceiling - 1e-5;
    const bool ceiling_drop = state.referenceValid && ceiling < state.lastCeiling - 1e-5;
    state.recovering = state.recovering || curve_deceleration || ceiling_drop;
    if(initializing) {
        // 卸压后的电机承接从实际车速附近开始；从零慢爬会继续产生反拖。
        state.reference = state.recovering ? std::min(ceiling, std::max(0.0, double(mVehicleSpeed))) : ceiling;
        if(mVehicleSpeed <= 0.1 && ceiling > mVehicleSpeed + 0.05)
            state.reference = std::max(state.reference, std::min(ceiling, 5.0 / 13.5));
        state.referenceAcceleration = 0.0;
        state.referenceValid = true;
    } else if(!state.recovering) {
        // 原普通起步仍跟随planning，不重新叠加曾删除的起步斜坡。
        state.reference = ceiling;
        state.referenceAcceleration = 0.0;
    } else if(dt > 0.0) {
        const double error = target - state.reference;
        const double acceleration_limit = error >= 0.0 ? kForwardDriveAcceleration : kForwardCurveDeceleration;
        // 离散控制需预留一帧jerk裕量，否则到目标才把非零加速度清零会再造一次顿挫。
        const double desired_acceleration = (error >= 0.0 ? 1.0 : -1.0) *
            std::min(acceleration_limit, std::max(0.0,
                std::sqrt(2.0 * kForwardMotorJerk * std::fabs(error)) - kForwardMotorJerk * dt));
        const double previous_acceleration = state.referenceAcceleration;
        state.referenceAcceleration += std::max(-kForwardMotorJerk * dt,
            std::min(kForwardMotorJerk * dt, desired_acceleration - state.referenceAcceleration));
        const double next = state.reference + 0.5 * (previous_acceleration + state.referenceAcceleration) * dt;
        if((error >= 0.0 && next >= target) || (error <= 0.0 && next <= target)) {
            state.reference = target;
            state.referenceAcceleration = 0.0;
        } else {
            state.reference = next;
        }
    }
    // 规划/故障/当前位置曲率的硬下降不能被舒适性滤波抬高或推迟。
    if(state.reference > ceiling || state.reference < 0.0) {
        state.reference = std::max(0.0, std::min(ceiling, state.reference));
        state.referenceAcceleration = 0.0;
    }
    state.referenceTime = now;
    state.lastCeiling = ceiling;
    if(!curve_deceleration && std::fabs(state.reference - ceiling) < 1e-5 &&
       std::fabs(state.referenceAcceleration) < 1e-5) state.recovering = false;
    return state.reference;
}

void ControlComply::TraceForwardLongitudinal(double ceiling) {
    const double now = ros::Time::now().toSec();
    if(mForwardSpeed.logTime >= 0.0 && now >= mForwardSpeed.logTime && now - mForwardSpeed.logTime < 1.0) return;
    mForwardSpeed.logTime = now;
    printf("D longitudinal: plan=%.3f actual=%.3f preview=%.2f/%.2f k=%.5f at=%.2f curve=%.3f/%.3f ceiling=%.3f reference=%.3f motor=%.3f acc=%.3f I=%.3f brake=%u feedback=%u pending=%d unknown=%d fault=%d\n",
           double(mSpeed), double(mVehicleSpeed), mForwardSpeed.nominalPreview, mForwardSpeed.retainedPreview,
           mForwardSpeed.previewCurvature, mForwardSpeed.peakDistance, mForwardSpeed.softCurveLimit,
           mForwardSpeed.hardCurveLimit, ceiling, mForwardSpeed.reference,
           double(mControlData.throttlePercent) / 13.5, mForwardBrake.acceleration,
           mBrakeIntegral, unsigned(mControlData.brakePercent), unsigned(mForwardBrake.feedback),
           int(mForwardBrake.pending), int(mForwardBrake.unconfirmed), int(mForwardBrake.fault));
}

void ControlComply::CalcuForwardPathCurve() {
    // 原1.1m航向差会放大毫米级路径起伏。D纵向独立使用沿线前后各2m的三点圆曲率，
    // 左右转弯对称、理想圆弧精确为1/R；不改供横向和R挡使用的mPathList.curvature。
    // 曲率估计支撑长度仍为4m；预瞄距离由ForwardCurveLimitSpeed按当前车速选择。
    const double half_window = 2.0;
    const std::size_t size = mPathList.size();
    mForwardPathList = mPathList;
    mForwardPathDistances.resize(size);
    mForwardCurveSpeedSquares.resize(size);
    if(size == 0) {
        mForwardSpeed.endValid = false;
        return;
    }
    double distance = 0.0;
    double end_distance2 = 1e30;
    double end_s = 0.0;
    for(std::size_t i = 0; i < size; ++i) {
        if(!std::isfinite(mPathList[i].x_axis) || !std::isfinite(mPathList[i].y_axis)) {
            mForwardPathList.clear();
            mForwardPathDistances.clear();
            mForwardCurveSpeedSquares.clear();
            mForwardSpeed.endValid = false;
            return;  // 缓存无效时主流程给定零速，不能退回噪声曲率或放开限速。
        }
        if(i > 0) {
            const double dx = double(mPathList[i].x_axis) - mPathList[i - 1].x_axis;
            const double dy = double(mPathList[i].y_axis) - mPathList[i - 1].y_axis;
            const double length = std::hypot(dx, dy);
            if(mForwardSpeed.endValid && length > 1e-6 &&
               (dx * mForwardSpeed.endTangentX + dy * mForwardSpeed.endTangentY) > 0.5 * length) {
                const double ratio = std::max(0.0, std::min(1.0,
                    ((mForwardSpeed.endX - mPathList[i - 1].x_axis) * dx +
                     (mForwardSpeed.endY - mPathList[i - 1].y_axis) * dy) / (length * length)));
                const double ex = mPathList[i - 1].x_axis + ratio * dx - mForwardSpeed.endX;
                const double ey = mPathList[i - 1].y_axis + ratio * dy - mForwardSpeed.endY;
                if(ex * ex + ey * ey < end_distance2) {
                    end_distance2 = ex * ex + ey * ey;
                    end_s = distance + ratio * length;
                }
            }
            distance += length;
        }
        mForwardPathDistances[i] = distance;
    }
    if(mForwardSpeed.endValid) {
        // 新轨迹须与已检查位置及切向相容；换路不能沿用旧路线的预瞄状态。
        if(end_distance2 <= 0.25) mForwardSpeed.endS = end_s;
        else {
            mForwardSpeed.endValid = false;
            mForwardSpeed.bufferSpeed = 0.0;
            for(auto& curvature : mForwardCurvatures) curvature = 0.0;
            mForwardCurveIndex = 0;
            mForwardCurveHistoryValid = false;
            // 同ID规划几何改变只清失效曲率；保留电机参考，不能借缓存失效跳过恢复斜坡。
        }
    }
    if(size < 3 || distance <= 1e-6) {
        // 无法建立几何曲率时使D缓存无效，主流程保持停车；不沿用旧速度缓存。
        mForwardPathList.clear();
        mForwardPathDistances.clear();
        mForwardCurveSpeedSquares.clear();
        mForwardSpeed.endValid = false;
        return;
    }

    const double half = std::min(half_window, distance / 2.0);
    std::size_t cursors[3] = {};
    double last_center = -1.0;
    double curvature = 0.0;
    float previous_curvature = -1.0f;
    double previous_square = 0.0;
    for(std::size_t i = 0; i < size; ++i) {
        // 两端平移完整窗口，不缩成短差分；短于4m的路径用实际可用长度，不补零/外推。
        const double center = std::max(half, std::min(distance - half, mForwardPathDistances[i]));
        if(center != last_center) {
            double xy[3][2];
            for(int j = 0; j < 3; ++j) {
                const double target = center + (j - 1) * half;
                std::size_t& cursor = cursors[j];
                while(cursor + 1 < size && mForwardPathDistances[cursor + 1] <= target) ++cursor;
                const std::size_t next = std::min(cursor + 1, size - 1);
                const double length = mForwardPathDistances[next] - mForwardPathDistances[cursor];
                const double ratio = length > 1e-12 ?
                    std::max(0.0, std::min(1.0, (target - mForwardPathDistances[cursor]) / length)) : 0.0;
                xy[j][0] = mPathList[cursor].x_axis + ratio *
                    (double(mPathList[next].x_axis) - mPathList[cursor].x_axis);
                xy[j][1] = mPathList[cursor].y_axis + ratio *
                    (double(mPathList[next].y_axis) - mPathList[cursor].y_axis);
            }
            const double abx = xy[1][0] - xy[0][0], aby = xy[1][1] - xy[0][1];
            const double bcx = xy[2][0] - xy[1][0], bcy = xy[2][1] - xy[1][1];
            const double acx = xy[2][0] - xy[0][0], acy = xy[2][1] - xy[0][1];
            const double denominator = std::sqrt((abx * abx + aby * aby) *
                (bcx * bcx + bcy * bcy) * (acx * acx + acy * acy));
            curvature = denominator > 1e-12 ? 2.0 * std::fabs(abx * bcy - aby * bcx) / denominator :
                        std::fabs(mPathList[i].curvature);
            last_center = center;
        }
        mForwardPathList[i].curvature = curvature;
        if(mForwardPathList[i].curvature != previous_curvature) {
            previous_curvature = mForwardPathList[i].curvature;
            const double curve_speed = ForwardCurvatureSpeed(previous_curvature);
            previous_square = curve_speed * curve_speed;
        }
        mForwardCurveSpeedSquares[i] = previous_square;
    }
}

float ControlComply::ForwardCurveLimitSpeed(const std::vector<XYZ_COOR_S>& pathlist) {
    const double preview_distance = std::max(0.0, double(mVehicleSpeed)) * 4.0;  // 导航车速m/s × 4s，无最小距离。
    if(pathlist.empty()) {
        ResetForwardCurveHistory();
        return 600.0f;  // 与0.6系数的全零曲率上限一致；主流程仍保留空路径停车。
    }
    if(mKeyPoint < 0 || static_cast<std::size_t>(mKeyPoint) >= pathlist.size() ||
       !std::isfinite(mNavData.xAxis) || !std::isfinite(mNavData.yAxis) || !std::isfinite(mVehicleSpeed)) {
        ResetForwardCurveHistory();
        return 0.0f;
    }

    // 复用本周期VehiclePoseCalculation已找到的最近点，只检查相邻两段的投影。
    // 从导航参考点的路径投影沿线起算，兼容车后拟合点与非均匀采样，不以点数代替距离。
    const std::size_t key = static_cast<std::size_t>(mKeyPoint);
    std::size_t start = key;
    double start_ratio = 0.0;
    const double key_dx = mNavData.xAxis - pathlist[key].x_axis;
    const double key_dy = mNavData.yAxis - pathlist[key].y_axis;
    double best_distance2 = key_dx * key_dx + key_dy * key_dy;
    for(std::size_t i = key > 0 ? key - 1 : key; i < pathlist.size() - 1 && i <= key; ++i) {
        const double dx = pathlist[i + 1].x_axis - pathlist[i].x_axis;
        const double dy = pathlist[i + 1].y_axis - pathlist[i].y_axis;
        const double length2 = dx * dx + dy * dy;
        if(length2 <= 1e-12 || !std::isfinite(length2)) continue;
        const double ratio = std::max(0.0, std::min(1.0,
            ((mNavData.xAxis - pathlist[i].x_axis) * dx +
             (mNavData.yAxis - pathlist[i].y_axis) * dy) / length2));
        const double px = pathlist[i].x_axis + ratio * dx - mNavData.xAxis;
        const double py = pathlist[i].y_axis + ratio * dy - mNavData.yAxis;
        const double distance2 = px * px + py * py;
        if(distance2 < best_distance2) {
            best_distance2 = distance2;
            start = i;
            start_ratio = ratio;
        }
    }

    const bool cached = &pathlist == &mForwardPathList && mForwardPathDistances.size() == pathlist.size() &&
        mForwardCurveSpeedSquares.size() == pathlist.size();
    double start_s = 0.0;
    if(cached) {
        start_s = mForwardPathDistances[start];
        if(start + 1 < pathlist.size()) start_s += start_ratio *
            (mForwardPathDistances[start + 1] - mForwardPathDistances[start]);
    }
    ForwardSpeedState& state = mForwardSpeed;
    const double retained = cached && state.endValid ? std::max(0.0, state.endS - start_s) : 0.0;
    const double horizon = std::max(preview_distance, retained);
    const bool extend_end = cached && (preview_distance >= retained || !state.endValid);
    double preview_curvature = std::fabs(pathlist[start].curvature);
    if(start + 1 < pathlist.size()) preview_curvature += start_ratio *
        (std::fabs(pathlist[start + 1].curvature) - preview_curvature);
    if(!std::isfinite(preview_curvature) || !std::isfinite(best_distance2)) {
        ResetForwardCurveHistory();
        return 0.0f;
    }

    // 历史仅对当前位置延缓出弯释放；前方峰值用其位置保留，不能平均成当前位置的急弯。
    mForwardCurvatures[mForwardCurveIndex] = preview_curvature;
    mForwardCurveIndex = (mForwardCurveIndex + 1) % 30;
    mForwardCurveHistoryValid = true;
    double history = 0.0;
    for(double curvature : mForwardCurvatures) history += curvature / 30.0;
    const double local_limit = ForwardCurvatureSpeed(std::max(preview_curvature, history));
    double hard_square = local_limit * local_limit;
    double soft_square = hard_square;
    const double speed_now = std::max(0.0, double(mVehicleSpeed));
    // 同一弯道减速时响应预留也不能随车速骤缩，否则仍会反复抬高速度参考。
    state.bufferSpeed = std::max(state.bufferSpeed, speed_now);
    const double hard_buffer = state.bufferSpeed * kForwardMotorDelay;
    const double soft_buffer = state.bufferSpeed * (kForwardMotorDelay + kForwardCurveDeceleration / kForwardMotorJerk);
    double traversed = 0.0;
    double remaining = horizon;
    double end_x = pathlist[start].x_axis;
    double end_y = pathlist[start].y_axis;
    double end_tx = 1.0, end_ty = 0.0;
    state.peakDistance = 0.0;
    for(std::size_t i = start; i + 1 < pathlist.size(); ++i) {
        const double begin_curvature = std::fabs(pathlist[i].curvature);
        const double end_curvature = std::fabs(pathlist[i + 1].curvature);
        const double dx = double(pathlist[i + 1].x_axis) - pathlist[i].x_axis;
        const double dy = double(pathlist[i + 1].y_axis) - pathlist[i].y_axis;
        const double length = cached ? mForwardPathDistances[i + 1] - mForwardPathDistances[i] : std::hypot(dx, dy);
        if(!std::isfinite(length) || !std::isfinite(begin_curvature) ||
           !std::isfinite(end_curvature)) {
            ResetForwardCurveHistory();
            return 0.0f;
        }
        const double begin_ratio = i == start ? start_ratio : 0.0;
        const double available = length * (1.0 - begin_ratio);
        const bool boundary = available > remaining;
        const double end_ratio = boundary && length > 1e-12 ? begin_ratio + remaining / length : 1.0;
        const double curvature = begin_curvature + (end_curvature - begin_curvature) * end_ratio;
        traversed += std::min(available, remaining);
        if(curvature > preview_curvature) {
            preview_curvature = curvature;
            state.peakDistance = traversed;
        }
        const double curve_speed = cached && !boundary ? 0.0 : ForwardCurvatureSpeed(curvature);
        const double curve_square = cached && !boundary ? mForwardCurveSpeedSquares[i + 1] : curve_speed * curve_speed;
        hard_square = std::min(hard_square, curve_square + 2.0 * kForwardCurveHardDeceleration *
                               std::max(0.0, traversed - hard_buffer));
        soft_square = std::min(soft_square, curve_square + 2.0 * kForwardCurveDeceleration *
                               std::max(0.0, traversed - soft_buffer));
        end_x = pathlist[i].x_axis + end_ratio * dx;
        end_y = pathlist[i].y_axis + end_ratio * dy;
        if(length > 1e-12) { end_tx = dx / length; end_ty = dy / length; }
        remaining -= std::min(available, remaining);
        if(boundary || remaining <= 1e-6) break;
    }
    if(!std::isfinite(best_distance2) || !std::isfinite(preview_curvature)) {
        ResetForwardCurveHistory();
        return 0.0f;
    }

    if(extend_end) {
        state.endValid = true;
        state.endX = end_x; state.endY = end_y;
        state.endTangentX = end_tx; state.endTangentY = end_ty;
        state.endS = start_s + traversed;
    }
    state.nominalPreview = preview_distance;
    state.retainedPreview = traversed;
    state.previewCurvature = preview_curvature;
    if(preview_curvature <= 0.01 && history <= 0.01) state.bufferSpeed = speed_now;
    state.softCurveLimit = std::sqrt(soft_square);
    state.hardCurveLimit = std::sqrt(hard_square);
    return state.hardCurveLimit;
}

float ControlComply::ForwardCurvatureSpeed(double curvature) {
    // 已实车标定的静态曲线逐值保留；沿程参考与缓存共用这一份公式。
    const double straight_curvature = 0.005;
    const double turn_curvature = 0.02;
    const double straight_coefficient = 0.6;
    const double turn_coefficient = 0.36;
    const double turn_adjust_begin = 0.01;
    const double turn_reference_curvature = 0.04;
    const double turn_speed_ratio = 1.4 / 1.8;
    // 保留上一版基础曲线：近直线系数0.6、弯道0.36；下方再平滑施加本轮弯道降速倍率。
    const double blend = std::max(0.0, std::min(1.0,
        (curvature - straight_curvature) / (turn_curvature - straight_curvature)));
    const double weight = blend * blend * (3.0 - 2.0 * blend);
    const double coefficient = straight_coefficient + (turn_coefficient - straight_coefficient) * weight;
    double speed = sqrt(curvature);
    if(speed < 0.001) speed = 0.001;
    // 已通过实车的直线曲线原样保留；新增五次过渡在两端一、二阶导数均为0。
    // 原限速与倍率均随曲率单调下降，因此不产生限速反弹，也不引入降速时间延迟。
    const double turn_blend = std::max(0.0, std::min(1.0,
        (curvature - turn_adjust_begin) / (turn_reference_curvature - turn_adjust_begin)));
    const double turn_weight = turn_blend * turn_blend * turn_blend *
        (10.0 + turn_blend * (-15.0 + 6.0 * turn_blend));
    const double speed_ratio = 1.0 + (turn_speed_ratio - 1.0) * turn_weight;
    return coefficient * speed_ratio / speed;
}

float ControlComply::CurveLimitSpeed(std::vector<XYZ_COOR_S> pathlist) {
    static std::vector<double> last_curvatures(30, 0);
    int n = pathlist.size();
    int batch = 60;

    std::vector<double> curvatures;

    if(n < batch) {
        for(int i = 0; i < n; i++)
            curvatures.push_back(pathlist[i].curvature);
        for(int i = n; i < batch; i++)
            curvatures.push_back(0);
    } else {
        for(int i = 0; i < batch; i++)
            curvatures.push_back(pathlist[i].curvature);
    }

    double bias = last_curvatures.back() - curvatures.front();
    if(fabs(bias) > 0.001) {
        last_curvatures.erase(last_curvatures.begin());
        last_curvatures.push_back(curvatures.front());
    }

    double speed = 0.0;
    for(auto c : last_curvatures) {
        speed += c;
    }
    int num = 0;
    for(auto i : curvatures) {
        if(num++ < 30) speed += i;
    }
    speed = sqrt(speed / (double)(batch));
    if(speed < 0.001) speed = 0.001;
    double final_speed = 0.3 / speed;
    return final_speed;
}

// ---------------------------------------------------------------------------
// 路径几何与车辆相对位姿
// ---------------------------------------------------------------------------

void ControlComply::CalcuPathCurve(vector<XYZ_COOR_S>& path_list) {
    int size = path_list.size();
    float curve_temp = 0;

    if(size <= 1) return;

    for(int i = 0; i < size - 2; ++i) {
        if(i < size - 12) {
            float iTemp = 0;
            int count = 0;
            float length = 0;

            for(int j = i; j < i + 11; ++j) {
                float sub = path_list.at(j + 1).heading - path_list.at(j).heading;

                if(sub < 0) count++;

                iTemp += sub;
                length += 0.1;
            }

            if(iTemp > 180) iTemp -= 360;
            if(iTemp < -180) iTemp += 360;

            curve_temp = iTemp * M_PI / 180 / length;

            if(curve_temp < 0) curve_temp = -1 * curve_temp;

            path_list.at(i).curvature = curve_temp;
        }
    }

    for(int i = size - 1; i > size - 13; i--)
        path_list.at(i).curvature = path_list.at(size - 13).curvature;
}

void ControlComply::CalcuPathHead(vector<XYZ_COOR_S>& path_list) {
    int size = path_list.size();
    XYZ_COOR_S xyz_array[2];
    float angle_temp = 0;

    if(size == 0) return;

    for(int i = 0; i < size - 1; ++i) {
        xyz_array[0] = path_list.at(i);
        xyz_array[1] = path_list.at(i + 1);
        angle_temp = pubalgor.CalculatePoint2PointAngle_P(
            xyz_array[0].x_axis, xyz_array[0].y_axis, xyz_array[1].x_axis,
            xyz_array[1].y_axis);

        path_list.at(i).heading = angle_temp;
        path_list.at(i).z_axis = 0;
        path_list.at(i).p2pDistance = 0.1;
        path_list.at(i).velocity = 5;
    }

    path_list.at(size - 1).heading = path_list.at(size - 2).heading;
    path_list.at(size - 1).z_axis = 0;
    path_list.at(size - 1).p2pDistance = 0.1;
    path_list.at(size - 1).velocity = 5;
}

void ControlComply::BiaAngleCalculate(
    std::vector<XYZ_COOR_S> path_list, CONTROL_PARAM_IN para_in,
    robot::control_msg& para_out) {
    if(path_list.size() < 3) {
        return;
    }
    float distance_temp;
    int new_key_point = 0;
    XYZ_COOR_S xyz_temp;
    float delta_x[2], delta_y[2];
    float min_distance = 100;
    int size = path_list.size();
    float cur_x = para_in.cur_position.x_axis;
    float cur_y = para_in.cur_position.y_axis;
    float cur_head = para_in.cur_position.heading;

    for(int i = 0; i < size; i++) {
        xyz_temp = path_list.at(i);
        distance_temp =
            sqrt((xyz_temp.x_axis - cur_x) * (xyz_temp.x_axis - cur_x) +
                 (xyz_temp.y_axis - cur_y) * (xyz_temp.y_axis - cur_y));

        if(min_distance > distance_temp) {
            min_distance = distance_temp;
            new_key_point = i % size;
        }
    }

    mKeyPoint = new_key_point;
    para_out.preCurve = path_list.at(mKeyPoint).curvature;

    if(path_list.at(path_list.size() - 3).curvature > para_out.preCurve)
        para_out.preCurve = path_list.at(path_list.size() - 3).curvature;

    delta_x[0] = cur_x - path_list.at(new_key_point).x_axis;
    delta_y[0] = cur_y - path_list.at(new_key_point).y_axis;
    delta_x[1] = path_list.at((new_key_point + 2) % size).x_axis -
                 path_list.at(new_key_point).x_axis;
    delta_y[1] = path_list.at((new_key_point + 2) % size).y_axis -
                 path_list.at(new_key_point).y_axis;

    distance_temp = delta_x[1] * delta_y[0] - delta_y[1] * delta_x[0];

    if(distance_temp > 0)
        para_out.biaDistance =
            sqrtf(delta_x[0] * delta_x[0] + delta_y[0] * delta_y[0]);
    else
        para_out.biaDistance =
            -1 * sqrtf(delta_x[0] * delta_x[0] + delta_y[0] * delta_y[0]);

    para_out.preAngleDev = 0;
}

void ControlComply::VehiclePoseCalculation() {
    CONTROL_PARAM_IN para_in;
    para_in.cur_position.x_axis = mNavData.xAxis;
    para_in.cur_position.y_axis = mNavData.yAxis;
    para_in.cur_position.heading = mNavData.heading;
    // 这里计算了距离车辆最近点的曲率和横向误差赋值给了mControlData
    // (但是这里并没有用到预瞄参数)
    BiaAngleCalculate(mPathList, para_in, mControlData);
}

void ControlComply::VehicleVerticalControl(float tDesireSpeed, float tCurSpeed,
                                           float tAcc, uint8_t& tThrottle, uint8_t& tBrake) {
    spCtr_c.SpeedTrack(tDesireSpeed, tCurSpeed, tAcc, tThrottle, tBrake);

    if(tDesireSpeed == 0) {
        tThrottle = 0;
        tBrake = 70;
    } else {
        tBrake = 0;
    }
}

// ---------------------------------------------------------------------------
// 横向控制入口
// ---------------------------------------------------------------------------

float ControlComply::VehicleLateralControl() {
    XYZ_COOR_S xyz_temp;
    xyz_temp.x_axis = mNavData.xAxis;
    xyz_temp.y_axis = mNavData.yAxis;
    xyz_temp.heading = mNavData.heading;
    // 寻找最近点
    int keyPointTemp = pubalgor.FindKeyPointByTargetPoint_P(
        mPathList, mNavData.xAxis, mNavData.yAxis);
    // 横向控制(这里通过横向和航向误差，使用pid进行的转向角计算)
    double rtn_value = geoCon_c.LateralControlTrack1(
        mPathList, xyz_temp, keyPointTemp, mNavData.gpsSpeed, mGear);

    return rtn_value;
}

double ControlComply::azimuthToYaw(const double& azimuth) {
    // 转为与正东夹角，逆时针为正
    double angle = 90 - azimuth;

    if(angle < -180) {
        angle += 360;
    }
    // 转为弧度
    double rad_angle = angle / 180 * M_PI;
    return rad_angle;
}

float ControlComply::VehicleStanleyControl() {
    lat_controller.setParameters(1.6, 22, 8);
    std::vector<Pose2d> path_2d = toPath2d(mPathList);
    Pose2d ego_pose = Pose2d(mNavData.xAxis, mNavData.yAxis);
    ego_pose.heading = azimuthToYaw(mNavData.heading);
    float steering_angle =
        lat_controller.calculate(path_2d, ego_pose, mVehicleSpeed, mSteerAngle);
    return -1.0 * steering_angle * 180.0 / M_PI;
}

std::vector<Pose2d> ControlComply::toPath2d(
    const std::vector<XYZ_COOR_S>& old_path) {
    std::vector<Pose2d> new_path;
    for(const auto& point : old_path) {
        new_path.emplace_back(point.x_axis, point.y_axis);
    }
    math_utils::computePoseAttr(new_path);
    return new_path;
}

// 保持 ACC/AEB 与主编排处于原翻译单元，避免既有未初始化状态受链接布局影响。
#include "longitudinal_acc_control.inc"
#include "forward_brake_control.inc"
