#pragma once

#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <tuple>
#include "common/struct_type.h"
#include <cmath>
#include "common/config/vehicle_param_config/vehicle_param_config.h" 
#include "common/pnc_point/trajectory_point.h"
  
class LaneChangeDecision 
{
public:
    LaneChangeDecision(const ChangeLane_Path &trajectory,
                       const bool is_free_lanechange, 
                       const bool is_current_path_blocked,
                       const VehicleParamConfig &vehicle_param,
                       LidarObjections *lidarobjs_global,
                       const VehicleStatus car_status);

    ~LaneChangeDecision() = default;

    void FreeLaneChange(bool is_free_lanechange);

    //void BlockLaneChange(bool is_current_blocked_);
    
private:
    std::string GetCurrentPathId(ChangeLane_Path& reference_line);
    
    void PrioritizeChangeLane(const bool change_line_to_direction,
                              ChangeLane_Path& reference_line);

    bool IsLaneStraight(const ChangeLane_Path& reference_line,int start_id);
    bool IsLanelengthEnough(const ChangeLane_Path& reference_line);
    bool IsObstacleBlocked(const ChangeLane_Path& reference_line);

    void UpdatePathStatus(ChangeLaneStatus::Status status_code,
                          const std::string& path_id);
   
    float CalMeanCurvity(const ChangeLane_Path& reference_line, 
                         int start_id, float d);

    int GetOnePointOnPathByDistacne(const ChangeLane_Path& reference_line, 
                                    int start_id, float d);

    void BuildObstacleBox(LidarObjections *lidarobjs_global);
    bool collisionDetection(const Point& reference_line);

    int FindNearestPoint2VehicleID2(
                     const VehicleStatus car_status, ChangeLane_Path& path);

    double GetCross(double x,double y, Prk_Point p1, Prk_Point p2);

    bool IsPointInRectangle(double x, double y, 
             Prk_Point p1, Prk_Point p2, Prk_Point p3, Prk_Point p4);
    
    ChangeLane_Path trajectory_;

    bool change_line_to_direction_left_;
    bool is_free_lanechange_;
    bool is_current_path_blocked_;

    VehicleParamConfig vehicleparam_;
    VehicleStatus car_status_;
 
    ChangeLaneStatus::Status current_status;
    std::string current_pathId;
    std::vector<sCellMsg> obstacle;

    double mini_preview_d_ = 10.0;
};

