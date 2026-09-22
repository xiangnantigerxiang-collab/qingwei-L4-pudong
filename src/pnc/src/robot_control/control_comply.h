// 控制编排器 ControlComply 类声明：路径接收与重采样、横纵向控制调度、
// 多级停车判据、ACC/AEB 接口

#ifndef ROBOT_CONTROL_CONTROL_COMPLY_H_
#define ROBOT_CONTROL_CONTROL_COMPLY_H_

#include "ros/ros.h"
#include "Eigen/Dense"
#include "unsupported/Eigen/CXX11/Tensor"
#include <cstdio>
#include <iostream>
#include <vector>
#include <string.h>

#include "common/fault_code.h"
#include "robot_path_plan/common/spline/Spline.h"
#include "robot_path_plan/common/pubalgor/pubalgor.h"

#include "robot_control/lateral_reverse_control.h"
#include "robot_control/longitudinal_speed_control.h"

#include "robot/navigation_msg.h"
#include "robot/path_plan_msg.h"
#include "robot/path_plan_status.h"
#include "robot/task_plan_msg.h"
#include "robot/can_msg.h"
#include "robot/control_msg.h"
#include "robot/TLStatus.h"
#include "robot/perception.h"
#include "robot/object.h"
#include "robot/acc.h"
#include "robot_control/stanley_controller/stanley_controller.h"
#include "robot_control/stanley_controller/math_utils.h"

typedef struct control_param_in {
    XYZ_COOR_S cur_position;
} CONTROL_PARAM_IN;

class ControlComply {
public:
    ControlComply();
    ~ControlComply();

    // ROS 消息输入。这里只更新 ControlComply 持有的输入快照。
    void SetCanData(robot::can_msg can_msg_t);
    void SetNavigationData(robot::navigation_msg navigation_t);
    void SetPathPlanData(robot::path_plan_msg path_plan_t);
    void SetTlStatusData(robot::TLStatus tl_status);
    void SetPathStatusData(robot::path_plan_status path_status_t);
    void setTaskPlanData(robot::task_plan_msg task_info);

    // 20 Hz 周期入口与控制消息发布。
    void VehicleControl();
    void PublishMessage(ros::Publisher& tPub);


    robot::perception LidarObject;
    robot::acc acc_msg;

    int SoundPlayCommand = 0;

private:
    // 路径预处理和限速。
    bool IsGreenLight(uint8_t light_state);
    float CurveLimitSpeed(std::vector<XYZ_COOR_S> pathlist);
    float ForwardCurveLimitSpeed(const std::vector<XYZ_COOR_S>& pathlist);
    static float ForwardCurvatureSpeed(double curvature);
    void CalcuForwardPathCurve();
    void ResetForwardCurveHistory();
    double SmoothForwardSpeed(double ceiling);
    void ResetForwardMotorReference();
    void TraceForwardLongitudinal(double ceiling);
    void CalcuPathCurve(vector<XYZ_COOR_S>& path_list);
    void CalcuPathHead(vector<XYZ_COOR_S>& path_list);

    // 车辆相对路径的位姿量。
    void BiaAngleCalculate(vector<XYZ_COOR_S> path_list, CONTROL_PARAM_IN para_in,
                           robot::control_msg& para_out);
    void VehiclePoseCalculation();

    // 纵向控制。ACC/AEB 实现在 longitudinal_acc_control.inc。
    void VehicleVerticalControl(float tDesireSpeed, float tCurSpeed,
                                float tAcc, uint8_t& tThrottle, uint8_t& tBrake);
    bool UpdateBrakeIntegral(double tDec);
    void ResetBrakeIntegral(bool reset_speed_history = true);
    uint8_t TerminalStopBrake();
    // D 挡专用。执行器记忆独立于规划积分复位，实现在 forward_brake_control.inc。
    void ObserveForwardBrakeFeedback(uint8_t brake, double now);
    void UpdateForwardAcceleration(double speed, double now);
    bool ForwardBrakeInputsFresh(double now) const;
    void CheckForwardBrakeTime(double now);
    void CompleteForwardBrakeRelease(double now);
    void RecoverForwardBrakeIfStopped(double now);
    void BeginForwardBrakeRequest(bool ordinary, double now);
    void RecordForwardBrakeCommand(uint8_t brake, bool ordinary, double now);
    void UpdateForwardBrakeShortage(double error, double now);
    bool ForwardBrakeFailureConfirmed(double now) const;
    void ApplyForwardDeceleration(double speed_cmd);
    void FinalizeForwardBrakeCommand();
    uint8_t ForwardTerminalStopBrake();
    double LongitudinalFeedforwardControl(robot::acc& pub);
    double LongitudinalFeedbackControl();
    double LongitudinalControlOutput(robot::acc& pub);

    // 横向控制：R 挡几何法，其他挡位为 Stanley + 纯追踪前馈。
    float VehicleLateralControl();
    float VehicleStanleyControl();
    double azimuthToYaw(const double& azimuth);
    std::vector<Pose2d> toPath2d(const std::vector<XYZ_COOR_S>& old_path);

private:
    // 路径状态。
    std::vector<XYZ_COOR_S> mPathList;
    // D纵向专用几何曲率；只随路径/挡位重建，原路径曲率继续供横向与R挡使用。
    std::vector<XYZ_COOR_S> mForwardPathList;
    std::vector<double> mForwardPathDistances;
    std::vector<double> mForwardCurveSpeedSquares;
    uint8_t mGear;

    float mSpeed;
    int mKeyPoint;
    int mPathid;
    bool mPathsafety;

    // 控制算法对象。
    PubAlgor pubalgor;
    GeometricConstrol geoCon_c;
    SpeedControl spCtr_c;

    // 接收消息快照。
    robot::navigation_msg mNavData;
    robot::TLStatus mTlStatus;
    robot::path_plan_status mPathStatus;
    robot::task_plan_msg mTaskInfo;

    // 发布消息和安全覆盖状态。
    robot::control_msg mControlData;
    int AccSwitch = 0;

    // D 挡 Stanley 控制所需状态。
    LatController lat_controller;
    Pose2d ego_pose2d;
    float mVehicleSpeed = 0.0;  // /navigation_msg.gpsSpeed，单位 m/s。
    float mSteerAngle = 0.0;

    double mLastCanTime = -1.0;
    double mLastNavigationTime = -1.0;
    bool mCanEmergencyStop = false;
    double mTerminalBrakeStartTime = -1.0;
    double mBrakeIntegral = 0.0;
    double mBrakeIntegralTime = -1.0;
    bool mBrakeAutomatic = false;
    bool mBrakePlanningSpeedValid = false;
    bool mBrakeDecelerationRequested = false;

    struct ForwardBrakeState {
        uint8_t feedback = 0;
        double feedbackTime = -1.0;
        double zeroSince = -1.0;
        bool pending = false;
        // 物理执行记忆跨挡保留；非D只登记实际发布请求，不干预其输出。
        bool commandIssued = false;
        bool activeCommand = false;
        bool ordinaryRequest = false;
        bool unconfirmed = false;
        bool responseSeen = false;
        double responseTime = -1.0;
        bool fault = false;
        double firstCommandTime = -1.0;
        // 最早未确认的强请求不能被反复撤销/重发延后；独立于每次新请求时窗。
        double unansweredStrongTime = -1.0;
        double lastCommandTime = -1.0;
        // 已有请求上的再加压可能迟到；同值保持不重置此窗口。
        double pressureIncreaseTime = -1.0;
        uint8_t peakRequestedBrake = 0;
        double releaseTime = -1.0;
        double finalizeTime = -1.0;
        double clockTime = -1.0;
        double staleSince = -1.0;
        uint8_t lastPublishedBrake = 0;
        double acceleration = 0.0;
        double recentDeceleration = 0.0;
        bool accelerationValid = false;
        double accelerationTime = -1.0;
        double accelerationSpeed = 0.0;
        double navigationTime = -1.0;
        double requestTime = -1.0;
        double targetFloor = 0.0;
        double demand = 0.0;
        double controlTime = -1.0;
        double adjustTime = -1.0;
        double commandIncreaseTime = -1.0;
        double saturationTime = -1.0;
        double saturationSampleTime = -1.0;
        bool motorInsufficient = false;
        double insufficientSince = -1.0;
        double insufficientSampleTime = -1.0;
        // 在途请求的执行失效监视独立于规划积分，取消规划下降不能清除此计时。
        double failureSince = -1.0;
        double failureSampleTime = -1.0;
        uint8_t ordinaryBrake = 0;
        bool releasing = false;
        double terminalTime = -1.0;
        double terminalBrake = 0.0;
        double stoppedSince = -1.0;
        bool terminalLatched = false;
        double terminalStopX = 0.0;
        double terminalStopY = 0.0;
        double terminalStartX = 0.0;
        double terminalStartY = 0.0;
    } mForwardBrake;

    struct ForwardSpeedState {
        // 已检查过的前方终点在地图系的位置；新路径到来时重新投影，不随降速后退。
        bool endValid = false;
        double endX = 0.0;
        double endY = 0.0;
        double endTangentX = 1.0;
        double endTangentY = 0.0;
        double endS = 0.0;
        double nominalPreview = 0.0;
        double retainedPreview = 0.0;
        double previewCurvature = 0.0;
        double peakDistance = 0.0;
        double softCurveLimit = 600.0;
        double hardCurveLimit = 600.0;
        double bufferSpeed = 0.0;
        bool referenceValid = false;
        bool recovering = false;
        double reference = 0.0;
        double referenceAcceleration = 0.0;
        double referenceTime = -1.0;
        double lastCeiling = 0.0;
        double logTime = -1.0;
    } mForwardSpeed;

    // D的当前位置曲率历史只延缓驶过弯道后的释放；前方约束另按位置保留。
    double mForwardCurvatures[30] = {};
    unsigned int mForwardCurveIndex = 0;
    int mForwardCurvePathId = -1;
    bool mForwardCurveHistoryValid = false;
};

#endif
