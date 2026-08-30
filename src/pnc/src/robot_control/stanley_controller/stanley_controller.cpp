// D 挡横向控制器 LatController 实现：
// calculate 主入口、calcSteer（Stanley 项）、feedforward（纯追踪项）、
// steerLimit/lowPassFilter。
#include <iostream>
#include "math_utils.h"
#include "stanley_controller.h"

void LatController::setParameters(double wheelbase, double max_steer,
                                  double max_steer_rate) {
  wheelbase_ = wheelbase;
  max_steer_ = max_steer / 180 * M_PI;
  max_steer_rate_ = max_steer_rate / 180 * M_PI;
}

double LatController::calculate(const std::vector<Pose2d>& trajectory,
                                const Pose2d& ego_pose, double ego_speed,
                                double ego_steer_angle, uint Gear) {
  // Gear 1 P 2 N 3 R 4 D
  if (trajectory.size() < 2 || (Gear != 3 && Gear != 4)) {
    return 0.0;
  }
  double curvature = calcCurature(trajectory, ego_pose);
  double k = 1 / (1 + 1.1 * curvature);
  printf("=============================LatController calculate=============================\n");
  printf("自车转向角: %.1f, 自车车速:%.1f, 自车曲率:%.1f, k系数:%.1f\n",
         ego_steer_angle / M_PI * 180, ego_speed, curvature, k);
  printf("----------------------------------------------------------------------------\n");
  double vehicle_speed = ego_speed < 0.5 ? 0.5 : ego_speed;
  double stanley_steer_angle = calcSteer(trajectory, ego_pose, vehicle_speed);
  double feedforwad_steer_angle = feedforward(trajectory, ego_pose, ego_speed);
  k = 0.3;
  double fusion_angle =
      (1 - k) * stanley_steer_angle + k * feedforwad_steer_angle;
  printf("融合后转向角 %.1f 度\n", fusion_angle / M_PI * 180);

  double filter_steer_angle =
      this->lowPassFilter(fusion_angle, ego_steer_angle, 0.8);
  printf("滤波后转向角 %.1f 度\n", filter_steer_angle / M_PI * 180);

  double final_steer_angle = steerLimit(filter_steer_angle, ego_steer_angle);
  printf("结果: %.1f 度\n", final_steer_angle / M_PI * 180);
  printf("============================================================================\n");
  return final_steer_angle;
}

double LatController::calcSteer(const std::vector<Pose2d>& trajectory,
                                const Pose2d& ego_pose,
                                const double& ego_speed) {
  // 1. 计算前轮中心点坐标
  double heading = ego_pose.heading;
  Pose2d front_pose;
  front_pose.x = ego_pose.x + wheelbase_ * std::cos(heading);
  front_pose.y = ego_pose.y + wheelbase_ * std::sin(heading);
  front_pose.heading = ego_pose.heading;
  // 2. 计算最近点垂足坐标
  auto foot_pose = math_utils::getFootPose(front_pose, trajectory);
  // 3. 计算横向偏差和航向偏差
  // 计算方向
  double vec12_x = std::cos(heading);
  double vec12_y = std::sin(heading);
  double vec13_x = foot_pose.x - ego_pose.x;
  double vec13_y = foot_pose.y - ego_pose.y;
  double cross = vec12_x * vec13_y - vec12_y * vec13_x;
  double sign = std::fabs(cross) / cross;  // 左侧为正
  // 计算大小
  double e_t = sign * math_utils::distance(foot_pose, front_pose);
  double phi_t =
      math_utils::minusPi2Pi(foot_pose.heading - front_pose.heading);
  // 4. 套用stanley公式，求解转向角
  double k = 0.6;
  double phi_weight = 0.4;
  double lat_weight = 0.4;
  double v_smooth = 0.01;
  double angle1 = phi_weight * phi_t;
  double angle2 = lat_weight * std::atan(k * e_t / (ego_speed + v_smooth));
  double steer_angle = angle1 + angle2;
  printf("LatController: 横向偏差:%.2f, 航向偏差:%.2f,偏角1:%.1f, 偏角2:%.1f,转向角: %.1f 度\n",
         e_t, phi_t / M_PI * 180, angle1 / M_PI * 180, angle2 / M_PI * 180,
         steer_angle / M_PI * 180);
  return steer_angle;
}

double LatController::feedforward(const std::vector<Pose2d>& trajectory,
                                  const Pose2d& ego_pose,
                                  const double& ego_speed) {
  double forward_distance = 2 * ego_speed;
  if (forward_distance < 3.0) {
    forward_distance = 3.0;
  }
  // 查找前馈点
  Pose2d ff_point;
  for (const auto& pt : trajectory) {
    if (pt.s > forward_distance) {
      ff_point = pt;
      break;
    }
  }
  Pose2d p_to_base = toBaseLink(ff_point, ego_pose);
  double real_forward_dist = ff_point.distanceTo(ego_pose);
  double turn_radius =
      (std::pow(p_to_base.x, 2) + std::pow(p_to_base.y, 2)) /
      (2 * p_to_base.y);
  // 3. 计算转向角
  double steer_angle = atan(wheelbase_ / turn_radius);
  printf("puresuit 前视距离:%f, 转弯半径:%f, 转向角: %f 度\n",
         real_forward_dist, turn_radius, steer_angle / M_PI * 180);
  return steer_angle;
}

double LatController::steerLimit(double desired_steer_angle,
                                 double current_steer_angle) {
  double steer_diff = desired_steer_angle - current_steer_angle;
  if (fabs(steer_diff) > max_steer_rate_) {
    steer_diff = (steer_diff > 0) ? max_steer_rate_ : -max_steer_rate_;
  }
  double steer_angle = current_steer_angle + steer_diff;
  steer_angle = math_utils::clamp(steer_angle, -max_steer_, max_steer_);
  return steer_angle;
}

double LatController::lowPassFilter(double new_val, double old_val,
                                    double alpha) {
  return alpha * new_val + (1.0 - alpha) * old_val;
}

Pose2d LatController::toBaseLink(const Pose2d& pose, const Pose2d& ego_pose) {
  Pose2d result;
  double dx = pose.x - ego_pose.x;
  double dy = pose.y - ego_pose.y;
  result.x = dx * cos(ego_pose.heading) + dy * sin(ego_pose.heading);
  result.y = -dx * sin(ego_pose.heading) + dy * cos(ego_pose.heading);
  result.heading = pose.heading - ego_pose.heading;
  return result;
}

double LatController::calcCurature(const std::vector<Pose2d>& trajectory,
                                   const Pose2d& ego_pose) {
  double curature = 0.0;
  int idx = math_utils::getNearestIndex(ego_pose, trajectory);
  double s = trajectory[idx].s;
  for (int i = idx; i < trajectory.size() - 1; i++) {
    Pose2d cur_pose = trajectory[i];
    if (cur_pose.s - s > 2) {
      break;
    }
    curature += cur_pose.curvature;
  }
  return curature;
}
