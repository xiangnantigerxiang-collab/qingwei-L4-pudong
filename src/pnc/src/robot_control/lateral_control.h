#ifndef LATERAL_CONTROL_H
#define LATERAL_CONTROL_H

#include <iostream>
#include <vector>
#include <array>
#include "../common/pubalgor/pubalgor.h"
using namespace std;

//#define GEAR_N (0x02)
//#define GEAR_R (0x03)
//#define GEAR_D (0x04)


class LateralControl{
public:
	LateralControl();
	~LateralControl(){
	}
	void SetControlParameter(array<float,5> tSpeedListForward,array<float,5> tAngleListForward,vector<vector<float>> tPListForward,\
			vector<vector<float>> tP2ListForward,array<float,5> tSpeedListBackward,array<float,5> tAngleListBackward,\
			vector<vector<float>> tPListBackward,vector<vector<float>> tP2ListBackward);
	float LateralControlTrack(float tPreAngleDev,float tPreHeadDev,float tSpeed,uint8_t tGear);
private:
	float BiaAngleFilter(const float tAngle);
	float BiaHeadFilter(const float tHead);
	void FuzzyPidAngleForward(float &fAP1,float &fAP2,const float dev_angle,const float tSpeed);
	void FuzzyPidAngleBackward(float &fAP1,float &fAP2,const float dev_angle,const float tSpeed);
private:
	array<float,5> speed_list_forward;
	array<float,5> angle_list_forward;
	vector<vector<float> > p_list_forward;
	vector<vector<float> > p2_list_forward;
	array<float,5> speed_list_backward;
	array<float,5> angle_list_backward;
	vector<vector<float>> p_list_backward;
	vector<vector<float>> p2_list_backward;
};

#endif
