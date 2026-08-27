#ifndef SPEED_PLANNING_H
#define SPEED_PLANNING_H

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <limits>
#include <numeric>
#include <sys/time.h>
#include "common/struct_type.h"
#include "common/config/avoid_plan.h"

using namespace planning;

#define kCurvatureDefault -1

//path param
#define kSpdMax_Forward 10.0/3.6
#define kSpdMin_Forward 2.0/3.6
#define kSpdMax_Reverse 2.0/3.6
#define kSpdMin_Reverse 1.5/3.6
#define kStop 0.0

//tunnel param
#define kSpdMax_T1 4.0/3.6
#define kSpdMin_T1 2.8/3.6
#define kSpdMax_T2 3.5/3.6
#define kSpdMin_T2 2.8/3.6

// cruv range
#define Cur2Speed_0 0.025
#define Cur2Speed_1 0.08
#define Cur2Speed_2 0.12
#define Cur2Speed_Limit 0.20

#define kPassWidthLower 0.7
#define kPassWidthUpper 2.1

#define max_motion(a,b) (((a) > (b))?(a) : (b))
#define min_motion(a,b) (((a) < (b))?(a) : (b))

class Speed_Planning
{
public:
    Speed_Planning() = default; 
    explicit Speed_Planning(
                 std::vector<OriginalInsData> *path,
                 double speed,std::vector<sCellMsg> lidarobjs_global);
   
    float SpeedCalculateInGuidePth(std::vector<OriginalInsData> *path);

private:
    void FreeDrivingInPath(
             std::vector<OriginalInsData> *path, 
             int road_direction_);

    double SpeedValue(
               double datamax, double datamin, 
               double datak, double param);

    void VelocityGenerationBasedTrafficFlow(
             std::vector<OriginalInsData> *path, 
             std::vector<sObjPosInfo> &ObjSelected);

    void ObstaclesProcess();

    void CollectObjs(std::vector<OriginalInsData> *path);

    int CollectObj(
            const int& PosIdMax, const double& PathEndLength,
            sObjPosInfo& obstacle, std::vector<OriginalInsData> *path);

    void GetPosOfObsInFrenet(
             sObjPosInfo& obstacle_, std::vector<OriginalInsData> *path);

    void SmoothingVelocity(std::vector<OriginalInsData> *path);

    void ObjBesideProcess(
             double objLength, double objDist, int posid,
             bool isObjStatic, std::vector<OriginalInsData> *path, 
      	     bool isObjNearPlan,float speed_obj);
    
    sObjPos CalcuObjPos(
                double x, double y, std::vector<OriginalInsData> *path);

    float CrossSpeedCalculation(
              double objDist, float speedBase, float objSpeed);
    
    std::vector<OriginalInsData> *planpath_;
    double speed_;
    std::vector<sCellMsg> lidarobjs_global_;
    std::vector<sObjPosInfo> obsInfo;
    std::vector<sObjPosInfo> ObjSelected;

    double vehSpeed;
    double vehSpeed_last;
    double vehWidth = 1.506;
    double BesideObjCrossSpeedMin = 0.5;
    double PassWidthLowerOffset = 0.1;
    double PassWidthLower = 0.5 * vehWidth + PassWidthLowerOffset;
    double PassWidthUpperOffsetMin = 0.5;
    double PassWidthUpperOffsetMiddle1 = 0.8;
    double PassWidthUpperOffsetMiddle2 = 1.2;
    double PassWidthUpperOffsetMax = 1.5;

    double PassWidthUpper1 = 0.5 * vehWidth + PassWidthUpperOffsetMin;
    double PassWidthUpper2 = 0.5 * vehWidth + PassWidthUpperOffsetMiddle1;
    double PassWidthUpper3 = 0.5 * vehWidth + PassWidthUpperOffsetMiddle2;
    double PassWidthUpper4 = 0.5 * vehWidth + PassWidthUpperOffsetMax;

    double veh2kPass1 = 0.5;
    double veh2kPass2 = 0.7;
    double veh2kPass3 = 1.0;

    const double VelocityInvalid = -88.0;
    float vehSpeedFixStart = 0.1;
    double blindsScope = 2.128;
    double BesideObjBackHoldDist = 2.0;
    double BesideObjFrontHoldDist = 2.0;
    double BesideObjPreviewDistBias = 3.0;

    int spd_down_i_;

    const float parking_spd_ = 3.0/3.6;
    const float start_acc_ = 0.2;
    const float acel_min_ = -5.0;
    const float acel_max_ = 1.0;
};

#endif //IMC_CTRL_H
