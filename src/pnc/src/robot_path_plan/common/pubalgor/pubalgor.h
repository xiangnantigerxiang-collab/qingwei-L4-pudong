#ifndef PUBALGOR_H
#define PUBALGOR_H

#include <cstdlib>
#include <string>
#include <fstream>
#include <cmath>
#include "Eigen/Eigen"
#include "common/struct_type.h"

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using namespace std;

#define MULTIPLE90(angle) ((angle == 0)||(angle == 90)||(angle == 180)||(angle == 270)||(angle == 360))

class PubAlgor
{
public:
    PubAlgor();
    ~PubAlgor(){
    }
	//math basic calculation
	float FuzzyDataProcess(const float *data_array,const float *limit_array,const int size,const float data);
	float CalculatePoint2PointAngleByDelta_P(float detaX,float detaY);
	float CalculatePoint2PointDistance_P(float x1_coor,float y1_coor,float z1_coor,
		float x2_coor,float y2_coor,float z2_coor);
	float CalculatePoint2PointAngle_P(float start_x,float start_y,float stop_x,float stop_y);
	int FindKeyPointByTargetPoint_P(vector<XYZ_COOR_S> xyz_list,const float x_coor,const float y_coor);
	int FindKeyPointByTargetPoint_P(vector<XYZ_COOR_S> xyz_list,const float x_coor,const float y_coor,\
			const int tSrc,const int tDes);
	XYZ_COOR_S CalculNextPointByAngle_P(float x_coor,float y_coor,float heading,float distance);

	float CalculateMinDistance_P(vector<XYZ_COOR_S> road_list,float x_axis,float y_axis);
	XYZ_COOR_S CaculateCrossPoint_P(float xPoint1,float yPoint1,float angle1,float xPoint2,float yPoint2,float angle2);
	//string operation
	vector<string> SplitString(string str,string flag);
	vector<string> Split(string str,char del);
	//data type conversion
	int16_t Uchar2Int16(const uint8_t *p_ch);
	int32_t Uchar2Int32(const uint8_t *p_ch);
	int16_t Uchar2Int16_r(const uint8_t *p_ch);
	int32_t Uchar2Int32_r(const uint8_t *p_ch);
	//path deal with
	vector<XYZ_COOR_S> GeneratePoint2PointPath(const float start_x,const float start_y,const float stop_x,const float stop_y);
	vector<XYZ_COOR_S> GenerateBezierPath_P(const float start_x,const float start_y,const float start_angle,
							const float stop_x,const float stop_y,const float stop_angle);
	vector<XYZ_COOR_S> GenerateBezierPathBy4Point_P(vector<float> point_x,vector<float> point_y);
	vector<XYZ_COOR_S> PanPath_P(const vector<XYZ_COOR_S> converPathList,float detaX,float detaY);
	vector<XYZ_COOR_S> GenerateExtendedPath(float tStartX,float tStartY,float tStartAngle,float tExtendDis);
};


#endif
