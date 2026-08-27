#ifndef CONTROL_COMPLY_H
#define CONTROL_COMPLY_H

#include "ros/ros.h"
#include "Eigen/Dense"
#include "unsupported/Eigen/CXX11/Tensor"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdio>
#include <iostream>
#include <vector>
#include <string.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "yaml-cpp/yaml.h"

#include "common/fault_code.h"
#include "../common/spline/Spline.h"
#include "../common/pubalgor/pubalgor.h"

#include "robot_control/lateral_control.h"
#include "robot_control/geometric_control.h"
#include "robot_control/speed_control.h"
#include "least_squares.h"

#include "robot/navigation_msg.h"
#include "robot/path_plan_msg.h"
#include "robot/path_plan_status.h"
#include "robot/task_plan_msg.h"
#include "robot/can_msg.h"
#include "robot/control_msg.h"
#include "robot/hook_position.h"
#include "robot/TLStatus.h"
#include "robot/perception.h"
#include "robot/object.h"
#include "robot/acc.h"
#include "lat_controller/lat_controller.h"
#include "lat_controller/math_utils.h"

 using point_type = boost::geometry::model::d2::point_xy<double>;
using polygon_type = boost::geometry::model::polygon<point_type>;

using namespace path_plan;

typedef struct control_param_in
{
    XYZ_COOR_S cur_position;
    int key_point;
    float forward_preview_dis; // for lateral control
    float back_preview_dis;
    float preview_time;
    float curve_preview_dis; // for speed control
    float preview_time2;
    uint8_t cur_gear;
    float vehicle_speed; // km/h
} CONTROL_PARAM_IN;

class ControlComply
{
public:
    ControlComply();
    ~ControlComply();

    void InitParameter();
    void SetCanData(robot::can_msg can_msg_t);
    void SetNavigationData(robot::navigation_msg navigation_t);
    void SetPathPlanData(robot::path_plan_msg path_plan_t);
    void SetPalletCoorData(robot::hook_position hook_pos_t);
    void SetTlStatusData(robot::TLStatus tl_status);
    void SetPathStatusData(robot::path_plan_status path_status_t);
    void setTaskPlanData(robot::task_plan_msg task_info);

    void VehicleControl();
    void PublishMessage(ros::Publisher &tPub);

    void LoadPathFile(std::string tPath);
    void FenceAlarm();
    XYZ_COOR_S local2global2(double ox, double oy, double oheading,
                                       double lx, double ly, double lheading);
    bool isWithinFence();

    robot::perception LidarObject;
    robot::acc acc_msg;

    double speedCMD;
    int LaneChangeCommand;
    int SoundPlayCommand;

    float wheelAngle = 0.0;
    float headingErr = 0.0;
    float poseErr = 0.0;

private:
    bool IsGreenLight(uint8_t light_state);

    // speed limit
    float CurveLimitSpeed(std::vector<XYZ_COOR_S> pathlist);
    float BiaAngleLimitSpeed(const float tBiaAngle);
    float BiaDisLimitSpeed(const float tBiaDistance);
    float SpeedJudge(float tDesireSpeed); // m/s

    // pose
    void CalcuPathCurve(vector<XYZ_COOR_S> &path_list);
    void CalcuPathHead(vector<XYZ_COOR_S> &path_list);
    void BiaAngleCalculate(vector<XYZ_COOR_S> path_list, CONTROL_PARAM_IN para_in, robot::control_msg &para_out);
    void VehiclePoseCalculation();
    // speed
    void VehicleVerticalControl(float tDesireSpeed, float tCurSpeed, float tAcc, uint8_t &tThrottle, uint8_t &tBrake);
    // lateral control
    float VehicleLateralControl();

    double LongitudinalFeedforwardControl(robot::acc &pub);
    double LongitudinalFeedbackControl();
    double LongitudinalControlOutput(robot::acc &pub);

    // stanley横向控制
    float VehicleStanleyControl();
    double azimuthToYaw(const double &azimuth);
    std::vector<Pose2d> toPath2d(const std::vector<XYZ_COOR_S> &old_path);

private:
    std::vector<XYZ_COOR_S> mPathList;
    std::vector<XYZ_COOR_S> mFenceList;
    uint8_t mGear;
    
    float mSpeed;
    int mKeyPoint;
    bool mGpsFixed;
    int mPathid;
    bool mPathsafety;
    float mPlanspeed;
    uint8_t mTaskType;

    std::vector<float> mCurveX;
    std::vector<float> mCurveY;

    // class
    PubAlgor pubalgor;
    YAML::Node config;
    LateralControl latCon_c;
    GeometricConstrol geoCon_c;
    SpeedControl spCtr_c;

    // message-receive
    robot::navigation_msg mNavData;
    robot::hook_position mHookPos;
    robot::TLStatus mTlStatus;
    robot::path_plan_status mPathStatus;
    robot::task_plan_msg mTaskInfo;

    // message-send
    robot::control_msg mControlData;
    int AccSwitch = 0;
    int FenceWarning = 0;

    LatController lat_controller;
    Pose2d ego_pose2d;
    float mVehicleSpeed = 0.0;
    float mSteerAngle = 0.0;
};

#endif
