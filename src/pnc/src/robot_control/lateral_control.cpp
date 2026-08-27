#include "lateral_control.h"

LateralControl::LateralControl(){

	p_list_forward.resize(5);
	p2_list_forward.resize(5);
	p_list_backward.resize(5);
	p2_list_backward.resize(5);
	
	for(int i = 0;i < p_list_forward.size();i++){
		p_list_forward[i].resize(5);
	    p2_list_forward[i].resize(5);
		p_list_backward[i].resize(5);
		p2_list_backward[i].resize(5);
	}

	float speed_list[] = {0,7,15,22,30};
	float angle_list[] = {0,5,12,20,30};
	float p_para_list[5][5] = {{132,147,162,176,191},{132,140,147,162,176},
		{132,132,140,147,162},{132,132,132,132,147},{118,118,118,118,132}};

	float p2_para_list[5][5] = {{88,103,118,125,132},{81,95,103,110,118},
		{73,88,88,95,103},{59,59,73,73,88},{59,59,66,73,81}};

	float speed_list1[] = {0,7,15,22,30};
	float angle_list1[] = {0,5,12,20,30};
	float p_para_list1[5][5] = {{132,147,162,176,191},{132,140,147,162,176},
		{132,132,140,147,162},{132,132,132,132,147},{118,118,118,118,132}};

	float p2_para_list1[5][5] = {{88,103,118,125,132},{81,95,103,110,118},
		{73,88,88,95,103},{59,59,73,73,88},{59,59,66,73,81}};

	for(int i = 0;i < sizeof(speed_list)/sizeof(speed_list[0]);i++){
		speed_list_forward[i] = speed_list[i];
		angle_list_forward[i] = angle_list[i];
		speed_list_backward[i] = speed_list1[i];
		angle_list_backward[i] = angle_list1[i];
	}
	for(int i = 0;i < 5;i++){
		for(int k = 0;k < 5;k++){
			p_list_forward[i][k] = p_para_list[i][k];
			p2_list_forward[i][k] = p2_para_list[i][k];
			p_list_backward[i][k] = p_para_list1[i][k];
			p2_list_backward[i][k] = p2_para_list1[i][k];
		}
	}
}

void LateralControl::SetControlParameter(array<float,5> tSpeedListForward,array<float,5> tAngleListForward,vector<vector<float>> tPListForward,\
             vector<vector<float>> tP2ListForward,array<float,5> tSpeedListBackward,array<float,5> tAngleListBackward,\
             vector<vector<float>> tPListBackward,vector<vector<float>> tP2ListBackward){
	
	speed_list_forward = tSpeedListForward;
	angle_list_forward = tAngleListForward;
	p_list_forward = tPListForward;
	p2_list_forward = tPListForward;

	speed_list_backward = tSpeedListBackward;
	angle_list_backward = tAngleListBackward;
	p_list_backward = tPListBackward;
	p2_list_backward = tPListBackward;
}

float LateralControl::BiaAngleFilter(const float tAngle){
	static float bia_angle = 0;
	static float bia_angle_last = 0;
	static float bia_angle_d = 0;
	static float bia_angle_record[6] = {0,0,0,0,0,0};
	float bia_angle_temp = tAngle;
	float bia_angle_temp2 = 0;

	bia_angle_last = bia_angle;
	bia_angle_temp2 = bia_angle_last + bia_angle_d/9;
	bia_angle = bia_angle_temp2*0.5 + bia_angle_temp*0.5;

	for(int i = 0;i < 5;++i)
		bia_angle_record[i] = bia_angle_record[i+1];
	bia_angle_record[5] = bia_angle;
	bia_angle_d = bia_angle_record[5] + bia_angle_record[4] + bia_angle_record[3]-\
				  bia_angle_record[2] - bia_angle_record[1] - bia_angle_record[0];
	return bia_angle;
}

float LateralControl::BiaHeadFilter(const float tHead){
	static float bia_head = 0;
	static float bia_head_last = 0;
	static float bia_head_d = 0;
	static float bia_head_record[6] = {0,0,0,0,0,0};
	float bia_head_temp = tHead;
	float bia_head_temp2 = 0;
	bia_head_last = bia_head;
	bia_head_temp2 = bia_head_last + bia_head_d/9;
	bia_head = bia_head_temp2*0.5 + bia_head_temp*0.5;

	for(int i = 0;i < 5;++i)
		bia_head_record[i] = bia_head_record[i+1];
	bia_head_record[5] = bia_head;
	bia_head_d  = bia_head_record[5] + bia_head_record[4] + bia_head_record[3] - \
				  bia_head_record[2] - bia_head_record[1] - bia_head_record[0];
	return bia_head;
}
void LateralControl::FuzzyPidAngleForward(float &fAP1,float &fAP2,const float dev_angle,const float tSpeed){
	int i_temp = 0,j_temp = 0;
	float para_speed[2],para_angle[2],para_matrix[4];
	//float speed_list[] = {0,7,15,22,30};
	//float angle_list[] = {0,5,12,20,30};
	//float p_para_list[5][5] = {{150,155,165,176,191},{150,155,165,170,180},\
		{140,145,150,155,162},{132,132,132,132,147},{118,118,118,118,132}};
	//float p2_para_list[5][5] = {{88,103,118,125,132},{81,95,103,110,118},\
		{73,88,88,95,103},{59,59,73,73,88},{59,59,66,73,81}};
	//float i_para_list[5][5] = {{0.3,0.3,0.3,0.3,0.3},{0.3,0.3,0.3,0.3,0.3},\
		{0.3,0.3,0.3,0.3,0.3},{0.3,0.3,0.3,0.3,0.3},{0.2,0.2,0.2,0.2,0.2}};
	//float d_para_list[5][5] = {{30,30,25,25,25},{30,30,25,25,25},\
		{28,27,27,22,22},{25,25,25,20,20},{20,20,20,20,20}};
	array<float, 5> speed_list = speed_list_forward;
	array<float, 5> angle_list = angle_list_forward;
	vector<vector<float>> p_para_list = p_list_forward;
	vector<vector<float>> p2_para_list = p2_list_forward;

	float speed_temp = tSpeed;
	float angle_temp = (dev_angle > 0)?(dev_angle):(-1*dev_angle);

	if(speed_temp < speed_list[0])
		speed_temp = speed_list[0];
	else if(speed_temp > speed_list[4])
		speed_temp = speed_list[4];
	if(angle_temp < angle_list[0])
		angle_temp = angle_list[0];
	else if(angle_temp > angle_list[4])
		angle_temp = angle_list[4];

	for(i_temp = 0;i_temp < 5;++i_temp){
		if(speed_temp < speed_list[i_temp])
			break;
	}
	if(i_temp < 1)
		i_temp = 1;
	else if(i_temp > 4)
		i_temp = 4;

	for(j_temp = 0;j_temp < 5;++j_temp){
		if(angle_temp < angle_list[j_temp])
			break;
	}
	if(j_temp < 1)
		j_temp = 1;
	else if(j_temp > 4)
		j_temp = 4;
	para_speed[0] = (speed_list[i_temp] - speed_temp)/(speed_list[i_temp] - speed_list[i_temp - 1]);
	para_speed[1] = (speed_temp - speed_list[i_temp - 1])/(speed_list[i_temp] - speed_list[i_temp - 1]);

	para_angle[0] = (angle_list[j_temp] - angle_temp)/(angle_list[j_temp] - angle_list[j_temp - 1]);
	para_angle[1] = (angle_temp - angle_list[j_temp - 1])/(angle_list[j_temp] - angle_list[j_temp - 1]);

	para_matrix[0] = para_speed[0]*para_angle[0];
	para_matrix[1] = para_speed[0]*para_angle[1];
	para_matrix[2] = para_speed[1]*para_angle[0];
	para_matrix[3] = para_speed[1]*para_angle[1];

	fAP1 = para_matrix[0]*p_para_list[i_temp - 1][j_temp - 1] + para_matrix[1]*p_para_list[i_temp - 1][j_temp]+\
		   para_matrix[2]*p_para_list[i_temp][j_temp - 1] + para_matrix[3]*p_para_list[i_temp][j_temp];
	fAP2 = para_matrix[0]*p2_para_list[i_temp - 1][j_temp - 1] + para_matrix[1]*p2_para_list[i_temp - 1][j_temp]+\
		   para_matrix[2]*p2_para_list[i_temp][j_temp - 1] + para_matrix[3]*p2_para_list[i_temp][j_temp];
}
void LateralControl::FuzzyPidAngleBackward(float &fAP1,float &fAP2,const float dev_angle,const float tSpeed){
	int i_temp = 0,j_temp = 0;
	float para_speed[2],para_angle[2],para_matrix[4];
	//float speed_list[] = {0,7,15,22,30};
	//float angle_list[] = {0,5,12,20,30};
	//float p_para_list[5][5] = {{132,147,162,176,191},{132,140,147,162,176},\
		{132,132,140,147,162},{132,132,132,132,147},{118,118,118,118,132}};

	//float p2_para_list[5][5] = {{88,103,118,125,132},{81,95,103,110,118},\
		{73,88,88,95,103},{59,59,73,73,88},{59,59,66,73,81}};
	//float i_para_list[5][5] = {{0.3,0.3,0.3,0.3,0.3},{0.3,0.3,0.3,0.3,0.3},\
		{0.3,0.3,0.3,0.3,0.3},{0.3,0.3,0.3,0.3,0.3},{0.2,0.2,0.2,0.2,0.2}};
	//float d_para_list[5][5] = {{30,30,25,25,25},{30,30,25,25,25},\
		{28,27,27,22,22},{25,25,25,20,20},{20,20,20,20,20}};
	array<float, 5> speed_list = speed_list_backward;
	array<float, 5> angle_list = angle_list_backward;
	vector<vector<float>> p_para_list = p_list_backward;
	vector<vector<float>> p2_para_list = p2_list_backward;

	float speed_temp = tSpeed;
	float angle_temp = (dev_angle > 0)?(dev_angle):(-1*dev_angle);

	if(speed_temp < speed_list[0])
		speed_temp = speed_list[0];
	else if(speed_temp > speed_list[4])
		speed_temp = speed_list[4];
	if(angle_temp < angle_list[0])
		angle_temp = angle_list[0];
	else if(angle_temp > angle_list[4])
		angle_temp = angle_list[4];

	for(i_temp = 0;i_temp < 5;++i_temp){
		if(speed_temp < speed_list[i_temp])
			break;
	}
	if(i_temp < 1)
		i_temp = 1;
	else if(i_temp > 4)
		i_temp = 4;
	for(j_temp = 0;j_temp < 5;++j_temp){
		if(angle_temp < angle_list[j_temp])
			break;

	}
	if(j_temp < 1)
		j_temp = 1;
	else if(j_temp > 4)
		j_temp = 4;
	para_speed[0] = (speed_list[i_temp] - speed_temp)/(speed_list[i_temp] - speed_list[i_temp - 1]);
	para_speed[1] = (speed_temp - speed_list[i_temp - 1])/(speed_list[i_temp] - speed_list[i_temp - 1]);

	para_angle[0] = (angle_list[j_temp] - angle_temp)/(angle_list[j_temp] - angle_list[j_temp - 1]);
	para_angle[1] = (angle_temp - angle_list[j_temp - 1])/(angle_list[j_temp] - angle_list[j_temp - 1]);

	para_matrix[0] = para_speed[0]*para_angle[0];
	para_matrix[1] = para_speed[0]*para_angle[1];
	para_matrix[2] = para_speed[1]*para_angle[0];
	para_matrix[3] = para_speed[1]*para_angle[1];

	fAP1 = para_matrix[0]*p_para_list[i_temp - 1][j_temp - 1] + para_matrix[1]*p_para_list[i_temp - 1][j_temp]+\
		   para_matrix[2]*p_para_list[i_temp][j_temp - 1] + para_matrix[3]*p_para_list[i_temp][j_temp];
	fAP2 = para_matrix[0]*p2_para_list[i_temp - 1][j_temp - 1] + para_matrix[1]*p2_para_list[i_temp - 1][j_temp]+\
		   para_matrix[2]*p2_para_list[i_temp][j_temp - 1] + para_matrix[3]*p2_para_list[i_temp][j_temp];
}


float LateralControl::LateralControlTrack(float tPreAngleDev,float tPreHeadDev,float tSpeed,uint8_t tGear){

	static float desire_wheel_angle = 0;
	float fAP1,fAP2;
	float angle_temp = 0;
	float filter_angle[2] = {0};
	//float Vehicle_max_front_wheel = 0;

	filter_angle[0] = BiaAngleFilter(tPreAngleDev);
	filter_angle[1] = BiaHeadFilter(tPreHeadDev);

	if(tGear != GEAR_R)
		FuzzyPidAngleForward(fAP1,fAP2,filter_angle[0],tSpeed);
	else
		FuzzyPidAngleBackward(fAP1,fAP2,filter_angle[0],tSpeed);
	angle_temp = fAP1 * filter_angle[0] + fAP2 * filter_angle[1];
	if(angle_temp > 9900)
		angle_temp = 9900;
	else if(angle_temp < - 9900)
		angle_temp = -9900;
	
	if(tGear == GEAR_R)
		angle_temp = -1*angle_temp;

	if(angle_temp > (desire_wheel_angle + 1000))
		desire_wheel_angle += 1000;
	else if(angle_temp < (desire_wheel_angle - 1000))
		desire_wheel_angle -= 1000;

	return desire_wheel_angle;// /10000*Vehicle_max_front_wheel;
}

