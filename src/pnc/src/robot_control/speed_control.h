#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include <iostream>
#include <math.h>

using namespace std;

//Wheel radius
#define R_WHEEL     0.3
//Transmission ratio
#define RATIO        9
//Transfer efficiency
#define YITA       0.9

class SpeedControl{
public:
	SpeedControl();
	~SpeedControl(){
	}
	//part1
	void SetAccPidParameter(float tKp,float tKi,float tKd,float tAccMin,float tAccMax);
	float AccelerationCalculateBySpeed_P(float tDesiredSpeed,float tCurSpeed);
	void SetVehicleParameter(float tVehicleMass,float tSlopePara,float tFroll,float tFwind,float tArea,\
			float tDriveTorque,float tBrakeTorque,float tThr_dis,float tBrk_dis);
	void SetThroPidParameter(float tKp,float tKi,float tKd);
	void SpeedTrack0(float tDesireSpeed,float tCurSpeed,float tAcc,float tPreSlope,uint8_t &tThrottle,uint8_t &tBrake);
	//part2
	void SpeedTrack1(float tDesireSpeed,float tCurSpeed,float tAcc,uint8_t &tThrottle,uint8_t &tBrake);
private:
	//part1
	void SpeedFuzzyPIDControl(float tBiaV,float tBiaV_d,float &tKp,float &tKi,float &tKd);
	//part 2
	float SpeedIncreasePID(const float delta_error);
	int FuzzyQuantization(const float tTemp);
	int SpeedFuzzyControl(const float delta_error);
private:
	//pid parameter
	float m_kp;
	float m_ki;
	float m_kd;
	float m_min_acc;
	float m_max_acc;
	//acc parameter
	float m_acc_last;
	float m_speed_dev;
	float m_speed_dev_last;
	float m_speed_add;
	float m_speed_dif;
	float m_speed_array[6];	
	//throttle and brake control
	float m_vehicle_mass;
	float m_control_slope_para;
	float m_control_froll;
	float m_control_fwind;
	float m_control_aera;
	float m_control_drive_torque;
	float m_control_brake_torque;
	float m_vehicle_thr_dis;
	float m_vehicle_brk_dis;
	float m_drive_last;

	float m_throttle_kp;
	float m_throttle_ki;
	float m_throttle_kd;
};



#endif

