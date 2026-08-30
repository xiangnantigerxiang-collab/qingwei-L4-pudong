// D 挡横向控制器 LatController 类声明：Stanley + 纯追踪前馈融合、
// 低通滤波、转角斜率限幅。
#ifndef ROBOT_CONTROL_STANLEY_CONTROLLER_H_
#define ROBOT_CONTROL_STANLEY_CONTROLLER_H_

#include <vector>
#include "pose2d.h"

class LatController {
 public:
  LatController() {}
  ~LatController() {}

  /**
   *角度制
   */
  void setParameters(double wheelbase, double max_steer_,
                     double max_steer_rate);
  double calculate(const std::vector<Pose2d>& trajectory,
                   const Pose2d& ego_pose, double ego_speed,
                   double ego_steer_angle, uint Gear = 4);

 private:
  double calcSteer(const std::vector<Pose2d>& trajectory,
                   const Pose2d& ego_pose, const double& ego_speed);
  double feedforward(const std::vector<Pose2d>& trajectory,
                     const Pose2d& ego_pose, const double& ego_speed);
  double steerLimit(double desired_steer_angle, double current_steer_angle);
  double lowPassFilter(double new_val, double old_val, double alpha);
  Pose2d toBaseLink(const Pose2d& pose, const Pose2d& ego_pose);

  double calcCurature(const std::vector<Pose2d>& trajectory,
                      const Pose2d& ego_pose);

 private:
  double wheelbase_ = 1.6;      // 轴距
  double max_steer_ = 24.0;     // 控制频率
  double max_steer_rate_ = 8.0; // 最大转角变化 rad/hz
};

#endif
