#include "speed_planning.h"

Speed_Planning::Speed_Planning(
    std::vector<OriginalInsData> *path,double speed,
    std::vector<sCellMsg> lidarobjs_global)
        :planpath_(path), speed_(speed), lidarobjs_global_(lidarobjs_global)
{
    //nop
}


float Speed_Planning::SpeedCalculateInGuidePth(
          std::vector<OriginalInsData> *path)
{
    vehSpeed = speed_;
    
    FreeDrivingInPath(path, 1);

    ObstaclesProcess();

    VelocityGenerationBasedTrafficFlow(path, ObjSelected);

    double spda = 0.0;
    double spdb = 0.0;

    for(int i = 0; i < 10; i++) spda += (*path).at(i).speed;

    spda = spda / 10.0;
    spdb = (*path).at(1).speed;

    return std::min<double>(spda, spdb);
}

void Speed_Planning::FreeDrivingInPath(
         std::vector<OriginalInsData> *path, int road_direction_)
{
    if ((*path).empty()) return;

    double SpdMax = 0.1;
    double SpdCurK = 1.0;
    double SpdCurMin = 0.1;

    switch(road_direction_) {
    case 1:
        SpdMax = set_speed_max;
        SpdCurMin = kSpdMin_Forward; break;
    case 2: 
        SpdMax = kSpdMax_Reverse;
        SpdCurMin = kSpdMin_Reverse; break;
 
    default: 
        SpdMax = set_speed_max;
        SpdCurMin = kSpdMin_Forward; break;
    }

    if(SpdMax < std::numeric_limits<double>::min()) SpdMax = 0.1;

    for(size_t i = 0; i < (*path).size(); ++i) {
      double spd = SpeedValue(SpdMax, SpdCurMin, SpdCurK, (*path).at(i).kappa);
      (*path).at(i).speed = spd;
    }

    SmoothingVelocity(path);
}

double Speed_Planning::SpeedValue(
           double datamax, double datamin, double datak, double param)
{
    double kdef = kCurvatureDefault;
    double vi = VelocityInvalid;
    double v0 = Cur2Speed_0; 
    double v1 = Cur2Speed_1; 
    double v2 = Cur2Speed_2; 
    double vt = 3.5 / 3.6;
    double vm = Cur2Speed_Limit;

    if(fabs(param - kdef) < 0.01) vt = vi;
    else {
        if(fabs(param) < v0) {
            vt = datamax;
        }

        if(fabs(param) >= v0 && fabs(param) < v1) {
            vt = datamax - (datamax - datak*datamax) * (param-v0) / (v1-v0);
        }

        if(fabs(param) >= v1 && fabs(param) < v2) {
            vt = datak * datamax - (datak*datamax - datamin) * (param-v1) / (v2-v1);
        }

        if(fabs(param) >= v2 && fabs(param) < vm) {
            vt = datamin;
        }
    }

    if(vt > datamax ) vt = datamax;
    if(vt < datamin ) vt = datamin;
    
    return vt;
}

void Speed_Planning::SmoothingVelocity(std::vector<OriginalInsData> *path)
{
    if((*path).empty()) return;

    double kAccelerationDefault = 0.1;

    for(size_t j = 1; j < (*path).size(); ++j) { //acc smooth
        const float kDiffMaxAcceleration = 0.01;

        if(fabs((*path).at(j).speed - VelocityInvalid) > 0.1) {
            float dv = (*path).at(j).speed - (*path).at(j-1).speed;

            if(dv > kDiffMaxAcceleration) {
                float vt = sqrt((*path).at(j-1).speed *
                           (*path).at(j-1).speed + 2 * kAccelerationDefault *
                           ((*path).at(j).length - (*path).at(j-1).length));

                vt = min_motion(vt, set_speed_max);

                if((*path).at(j).speed > vt) (*path).at(j).speed = vt;
            }
        }
    }

    for(size_t j = (*path).size()-1; j > 0; --j) {
        const float kDiffMaxDeceleration = 0.02;

        if(fabs((*path).at(j).speed - VelocityInvalid) > 0.1 ) { //don't smooth
            float dv = (*path).at(j-1).speed - (*path).at(j).speed;

            if(dv > kDiffMaxDeceleration) {
                double accel = -0.5 * ((*path).at(0).speed *
                               (*path).at(0).speed -
                               (*path).at(j).speed *
                               (*path).at(j).speed) / (*path).at(j).length;

                accel = min_motion(accel, -0.2);

                float vt = sqrt((*path).at(j).speed *
                           (*path).at(j).speed - 2 * accel *
                           ((*path).at(j).length - (*path).at(j-1).length));

                vt = min_motion(vt, set_speed_max);

                if((*path).at(j-1).speed > vt) (*path).at(j-1).speed = vt;
            }
        }
    }

    for(size_t i = 0; i < (*path).size(); ++i) {
        double SpeedStart = (*path).at(i).speed;
        double LenStart = (*path).at(i).length;
        double SearchTime = 3.0;
        double SearchDistMin = 0.1;
        double SearchDist = 0;
        int SpeedUpFlag = 0;

        SearchDist = max_motion(SpeedStart * SearchTime, SearchDistMin);

        for(size_t j = i+1; j < (*path).size(); ++j) {
            double DistNow = (*path).at(j).length - LenStart;
            double SpeedNow = (*path).at(j).speed;

            if(DistNow > SearchDist)break;

            if(SpeedUpFlag == 0) {
                if(SpeedNow > SpeedStart) SpeedUpFlag = 1;
                else break;
            }else {
                if(SpeedNow <= SpeedStart && SpeedNow >= vehSpeedFixStart) {
                    for(size_t m = i; m < j; ++m) {
                        (*path).at(m).speed = SpeedNow;
                        SpeedStart = SpeedNow;
                        SpeedUpFlag = 0;
                    }
                }
            }
        }
    }
}

void Speed_Planning::ObstaclesProcess()  //TBD
{
  CollectObjs(planpath_);
}

void Speed_Planning::CollectObjs(std::vector<OriginalInsData> *path)
{
  if ((*path).empty() || (lidarobjs_global_.size() == 0) ) return;
  int PosIdMax = static_cast<int>((*path).size()) - 1;
  double PathEndLength = (*path).back().length;
 
  for (size_t i = 0; i < lidarobjs_global_.size(); ++i)
  {
          sObjPosInfo center_point;
          center_point.xg = lidarobjs_global_[i].xg;
          center_point.yg = lidarobjs_global_[i].yg;
          center_point.heading = lidarobjs_global_[i].heading;
          obsInfo.emplace_back(center_point);
  }
  
  for(auto obsInfo_:obsInfo)
  { 
    CollectObj(PosIdMax, PathEndLength, obsInfo_, path);
    ObjSelected.emplace_back(obsInfo_);
  }
}

int Speed_Planning::CollectObj( const int& PosIdMax, const double& PathEndLength,
                                 sObjPosInfo& obstacle, std::vector<OriginalInsData> *path) {
  if ((*path).empty()) return -1;
 
  GetPosOfObsInFrenet(obstacle, path);
  return 0;
}

void Speed_Planning::GetPosOfObsInFrenet(sObjPosInfo& obstacle,
                                         std::vector<OriginalInsData> *path)
{
    sObjPos objPosTemp = CalcuObjPos(obstacle.xg,obstacle.yg, path);
    //最近侧向距离
    if (obstacle.dist > objPosTemp.dist)
    {
      obstacle.dist = objPosTemp.dist;
    }
    //占用道路的cell
    if ( obstacle.FrontLength > objPosTemp.length && objPosTemp.dist <= kPassWidthLower)
    {
      obstacle.FrontLength = objPosTemp.length;
      obstacle.type = "FRONT";
    }

    //道路旁边的cell
    if ( obstacle.BesideLength > objPosTemp.length && objPosTemp.dist > kPassWidthLower && objPosTemp.dist < kPassWidthUpper)
    {
      obstacle.BesideLength = objPosTemp.length;
      obstacle.type = "BESIDE";
    }

    if (obstacle.dist <= kPassWidthLower)
    {
      obstacle.Beside_dist = kPassWidthLower + 0.01;
    }
    else
    {
      obstacle.Beside_dist = obstacle.dist;
    }

  obstacle.s = objPosTemp.length;
  obstacle.l = objPosTemp.dist;
}

void Speed_Planning::VelocityGenerationBasedTrafficFlow(
  std::vector<OriginalInsData> *path, std::vector<sObjPosInfo> &ObjSelected)
{
  if ((*path).empty())
  {
    return;
  }
  double objLength = 88;
  double objDist = 88;
  int posid = 8888;

  double objLengthBeside = 88;
  double objDistBeside = 88;
  int posidBeside = 8888;

  bool hasObjFront = false;
  bool hasObjBeside = false;
  bool hasObjNear = false;
  bool isStaticObj = true;
  float objSpeed = 0;

  for (size_t i = 0 ; i < ObjSelected.size(); ++i)
  {
    if (ObjSelected.at(i).type == "FRONT" )
    {
      hasObjFront = true;
    }
    if ( ObjSelected.at(i).type == "BESIDE" )
    {
      hasObjBeside = true;
    }
  }

  if (hasObjBeside) //路旁的障碍物进行减速
  {
    for (size_t i = 0 ; i < ObjSelected.size(); ++i)
    {
      if (ObjSelected.at(i).type == "BESIDE")
      {
        objLengthBeside = ObjSelected.at(i).s;
        objDistBeside = ObjSelected.at(i).l;
        ObjBesideProcess(objLengthBeside, objDistBeside, i,
                         isStaticObj, path, hasObjNear, objSpeed);
      }
    }
  }

  if (hasObjFront)  //正前方障碍物减速到零
  {
    for (size_t i = 0 ; i < ObjSelected.size(); ++i)
    {
      if (ObjSelected.at(i).type == "FRONT" )
      {
        objLength = ObjSelected.at(i).s;
        objDist = ObjSelected.at(i).l;
      }
    }
  }
}

void Speed_Planning::ObjBesideProcess(   double objLength, double objDist,
    int posid, bool isStaticObj, std::vector<OriginalInsData> *path,bool hasObjNearPlan, float speed_obj)
{
  if ((*path).empty()) return;
  float crossSpeedBase = 2.0/3.6;
  vehSpeed = vehSpeed/3.6;
  float passSpeed = CrossSpeedCalculation(objDist, crossSpeedBase, speed_obj);
  float accelPlan = fabs(0.2);
  float decelpreview = max_motion(0,
                                  (vehSpeed * vehSpeed - passSpeed * passSpeed) / (2 * accelPlan));
  float distPreview = max_motion(blindsScope ,
                                 decelpreview + BesideObjFrontHoldDist + BesideObjPreviewDistBias );

  if ( posid > 1 && posid < static_cast<int>((*path).size()) && objLength < distPreview )
  {
  
    for (int i = posid; i > 0; --i)
    {
      (*path).at(i).speed = min_motion(passSpeed,
          (*path).at(i).speed);
      if (objLength - (*path).at(i).length > BesideObjFrontHoldDist)
      {
        break;
      }
    }
    for (int i = posid; i < static_cast<int>((*path).size()); ++i)
    {
      (*path).at(i).speed = min_motion(passSpeed,
          (*path).at(i).speed);
      if ((*path).at(i).length - objLength > BesideObjBackHoldDist)
      {
        break;
      }
    }
  }
}

sObjPos Speed_Planning::CalcuObjPos(double x, double y,
                                    std::vector<OriginalInsData> *path)
{
  sObjPos posInfo;
  double disThre = 88888;
  size_t indexStamp = 8888;

  if ((*path).empty())return posInfo;

  //间隔路点搜索 
  int JumpSearchCnt = 10;
  for (size_t i = 0; i < (*path).size(); i = i + JumpSearchCnt)
  {
    double ptX = (*path).at(i).x;
    double ptY = (*path).at(i).y;
    double dis = (x - ptX) * (x - ptX) + (y - ptY) * (y - ptY);

    if (disThre > dis)
    {
      disThre = dis;
      indexStamp = i;
    }
  }

  //精细搜索 
  int SearchStart = indexStamp - JumpSearchCnt;
  int SearchEnd   = indexStamp + JumpSearchCnt;
  if (SearchStart < 0)SearchStart = 0;
  if (SearchEnd > (*path).size()) SearchEnd = (*path).size();
  for (int j = SearchStart; j < SearchEnd; ++j)
  {
    double ptX = (*path).at(j).x;
    double ptY = (*path).at(j).y;
    double dis = (x - ptX) * (x - ptX) + (y - ptY) * (y - ptY);
    if (disThre > dis)
    {
      disThre = dis;
      indexStamp = j;
    }
    posInfo.posid  = indexStamp;
  }
  posInfo.dist = sqrt(disThre);
  posInfo.length = (*path).at(min_motion(int(indexStamp ),
                                  int((*path).size() - 1))).length;
  return posInfo;
}

float Speed_Planning::CrossSpeedCalculation(double objDist, float speedBase, float objSpeed)
{
  float crossSpeed = 0.0;
  if ( objDist <= PassWidthLower )
  {
  }
  else if ( objDist < PassWidthUpper1)
  {
    crossSpeed = BesideObjCrossSpeedMin + (veh2kPass1 - BesideObjCrossSpeedMin) *
                 ( objDist - PassWidthLower ) / ( PassWidthUpper1 -  PassWidthLower) + objSpeed;
  }
  else if ( objDist < PassWidthUpper2)
  {
    crossSpeed = veh2kPass1 + (veh2kPass2 -  veh2kPass1) *
                 ( objDist - PassWidthUpper1 ) / ( PassWidthUpper2 -  PassWidthUpper1) + objSpeed;
  }
  else if ( objDist < PassWidthUpper3)
  {
    crossSpeed = veh2kPass2 + (veh2kPass3 - veh2kPass2) *
                 ( objDist - PassWidthUpper2 ) / ( PassWidthUpper3 -  PassWidthUpper2) + objSpeed;
  }
  else if ( objDist < PassWidthUpper4)
  {
    crossSpeed = veh2kPass3 + (speedBase  -  veh2kPass3) *
                 ( objDist - PassWidthUpper3 ) / ( PassWidthUpper4 -  PassWidthUpper3) + objSpeed;
  }
  else
  {
    crossSpeed = speedBase;
  }
  crossSpeed = min_motion(crossSpeed, speedBase); //不能超过基准速度
  crossSpeed = max_motion(crossSpeed, float(BesideObjCrossSpeedMin));     //不能低于0.3m/s
  return crossSpeed;
}
