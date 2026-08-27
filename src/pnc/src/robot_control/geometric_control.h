#ifndef GEOMETRIC_CONTROL_H
#define GEOMETRIC_CONTROL_H

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <vector>

#include "common/struct_type.h"
#include <cmath>
#include <limits>

using namespace std;

class GeometricConstrol{
public:	
	GeometricConstrol();
	~GeometricConstrol(){
	}
	void SetParameter(float tPreDisForward,float tPreDisBackward,float tWB,float tKV);	
	float LateralControlTrack1(vector<XYZ_COOR_S> tPathList,XYZ_COOR_S tPosition,const int tKeyPoint,\
			float tSpeed,uint8_t tGear);
	float LateralControlTrack2(vector<XYZ_COOR_S> tPathList,XYZ_COOR_S tPosition,\
									float tSpeed,int tKeyPoint);

        int LaneChangeCommand;
        float TurnHeading;
        float wheelAngle = 0.0;
        float headingErr = 0.0;
        float poseErr = 0.0;

private:
	float CalculatePoint2PointAngle_P(float start_x,float start_y,float stop_x,float stop_y);
	float CalculatePoint2PointDistance_P(float x1_coor,float y1_coor,float z1_coor,
		float x2_coor,float y2_coor,float z2_coor);
	float CalcuLateralBiaDistance(vector<XYZ_COOR_S> tPathList,XYZ_COOR_S tPosition,const float tX,const float tY);
	XYZ_COOR_S CalculNextPointByAngle_P(float x_coor,float y_coor,float heading,float distance);
	int FindKeyPointByTargetPoint_P(vector<XYZ_COOR_S> xyz_list,const float x_coor,const float y_coor);
    //0403
	unsigned int FindNearestPoint2VehicleID(vector<XYZ_COOR_S> lpath,XYZ_COOR_S tPosition);
	XYZ_COOR_S global2local(double ox, double oy, double oheading,double gx, double gy,double gheading, double lx, double ly,double lheading);
	float GetPreviewDistance(float speed,uint8_t tGear);
	unsigned int FindPreviewPointOnPath(vector<XYZ_COOR_S> lpath, float d,int nearest_id);
	double GetTurningRadiusByPosAndHeading(vector<XYZ_COOR_S> lpath,int prev_id);
	double LimitedTurningRadiusByCenterAcc(double r,double speed);
	double GetDesiredSteeringAng(double L, double r);
        double GetLength(double x1, double y1, double x2, double y2);
        double CalculateLineDirection(XYZ_COOR_S p2);
private:
	float pre_distance_forward;
	float pre_distance_backward;
	float wheelbase;
	float speed_k;

	XYZ_COOR_S nearest_p_;
	const double very_less_value_ = std::numeric_limits<double>::min();
    const double very_most_value_ = std::numeric_limits<double>::max();
    const double very_less_dir_ = 0.000001;
    const double MAX_Centr_ACC_ = 5.0;

    const double max_turning_R_ = 100000.0;
};


#endif
