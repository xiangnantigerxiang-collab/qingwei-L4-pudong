// 控制编排器 ControlComply 类声明：路径接收与重采样、横纵向控制调度、
// 电子围栏、多级停车判据、ACC/AEB 接口

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

  // 电子围栏在 control_node 主循环中独立于 VehicleControl 执行。
  void LoadPathFile(std::string tPath);
  void FenceAlarm();
  XYZ_COOR_S local2global2(double ox, double oy, double oheading,
                           double lx, double ly, double lheading);

  robot::perception LidarObject;
  robot::acc acc_msg;

  int SoundPlayCommand = 0;

 private:
  // 路径预处理和限速。
  bool IsGreenLight(uint8_t light_state);
  float CurveLimitSpeed(std::vector<XYZ_COOR_S> pathlist);
  void CalcuPathCurve(vector<XYZ_COOR_S>& path_list);
  void CalcuPathHead(vector<XYZ_COOR_S>& path_list);

  // 车辆相对路径的位姿量。
  void BiaAngleCalculate(vector<XYZ_COOR_S> path_list, CONTROL_PARAM_IN para_in,
                         robot::control_msg& para_out);
  void VehiclePoseCalculation();

  // 纵向控制。ACC/AEB 实现在 longitudinal_acc_control.inc。
  void VehicleVerticalControl(float tDesireSpeed, float tCurSpeed,
                              float tAcc, uint8_t& tThrottle, uint8_t& tBrake);
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
  std::vector<XYZ_COOR_S> mFenceList;
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
  int FenceWarning = 0;

  // D 挡 Stanley 控制所需状态。
  LatController lat_controller;
  Pose2d ego_pose2d;
  float mVehicleSpeed = 0.0;
  float mSteerAngle = 0.0;
};

#endif
