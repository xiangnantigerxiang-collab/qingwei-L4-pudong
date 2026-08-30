// 纵向速度控制：SpeedControl 类声明
// 模糊 PID 期望加速度（AccelerationCalculateBySpeed_P）
// 与油门/刹车分配（SpeedTrack）

#ifndef ROBOT_CONTROL_LONGITUDINAL_SPEED_CONTROL_H_
#define ROBOT_CONTROL_LONGITUDINAL_SPEED_CONTROL_H_

#include <cstdint>
#include <iostream>
#include <math.h>

using namespace std;

class SpeedControl {
 public:
  SpeedControl();
  ~SpeedControl() {}
  // 期望加速度 PID 参数与计算
  float AccelerationCalculateBySpeed_P(float tDesiredSpeed, float tCurSpeed);
  // 油门/刹车分配
  void SpeedTrack(float tDesireSpeed, float tCurSpeed, float tAcc,
                   uint8_t& tThrottle, uint8_t& tBrake);
 private:
  // 模糊 PID 参数整定
  void SpeedFuzzyPIDControl(float tBiaV, float tBiaV_d, float& tKp,
                            float& tKi, float& tKd);
  // 油门增量 PID 与模糊推理
  float SpeedIncreasePID(const float delta_error);
  int FuzzyQuantization(const float tTemp);
  int SpeedFuzzyControl(const float delta_error);
 private:
  // pid parameter
  float m_kp;
  float m_ki;
  float m_kd;
  float m_min_acc;
  float m_max_acc;
  // acc parameter
  float m_acc_last;
  float m_speed_dev;
  float m_speed_dev_last;
  float m_speed_add;
  float m_speed_dif;
  float m_speed_array[6];

  float m_throttle_kp;
  float m_throttle_ki;
  float m_throttle_kd;
};

#endif
