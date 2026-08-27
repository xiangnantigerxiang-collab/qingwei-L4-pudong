#include "lanechange_decision.h" 

LaneChangeDecision::LaneChangeDecision(
    const ChangeLane_Path &trajectory,
    const bool is_free_lanechange, 
    const bool is_current_path_blocked,
    const VehicleParamConfig &vehicle_param,
    LidarObjections *lidarobjs_global,
    const VehicleStatus car_status)
        : trajectory_(trajectory),
          is_free_lanechange_(is_free_lanechange),
          is_current_path_blocked_(is_current_path_blocked),
          vehicleparam_(vehicle_param),
          car_status_(car_status)
{
    BuildObstacleBox(lidarobjs_global);
}

std::string LaneChangeDecision::GetCurrentPathId(ChangeLane_Path& reference_line)
{
  if(reference_line.changelane_path.empty()) return "";
  
  return reference_line.pathId;
}

int LaneChangeDecision::GetOnePointOnPathByDistacne(
        const ChangeLane_Path& reference_line, int start_id, float d)
{
    if(d == 0.0) return start_id;

    int path_num = reference_line.changelane_path.size();

    if(d > 0.0) {
        for(int i = start_id; i < path_num; i++) {
            double s0 = reference_line.changelane_path[start_id].length;
            double si = reference_line.changelane_path[i].length;

            if((si-s0) >= d) return i;
        }
        
        return path_num - 1;
    }else {
        for(int i = start_id; i > 4; i--) {
            double s0 = reference_line.changelane_path[start_id].length;
            double si = reference_line.changelane_path[i].length;

            if((si-s0) <= d ) return i;
        }

        return 4;
    }
}

float LaneChangeDecision::CalMeanCurvity(
          const ChangeLane_Path& reference_line, int start_id, float d)
{
    float c = 0.0;
   
    int end_id = GetOnePointOnPathByDistacne(reference_line, start_id, d);

    if(end_id < start_id) end_id = start_id;

    for(int i = start_id; i <= end_id; i++) {
        c += reference_line.changelane_path[i].curv;
    }

   return c / (end_id - start_id + 1);
}

bool LaneChangeDecision::IsLaneStraight(
         const ChangeLane_Path& reference_line, int start_id)
{
    if(reference_line.changelane_path.empty()) return false;

    double mean = CalMeanCurvity(reference_line, start_id, mini_preview_d_);

    if (mean >= 0.1) return false;
    else return true;
}

bool LaneChangeDecision::IsLanelengthEnough(const ChangeLane_Path& reference_line)
{
   double length_threshold = 10.0;
   int path_size = reference_line.changelane_path.size();

   if (path_size == 0) return false;

   double length = reference_line.changelane_path[path_size -1].length;
   if (length >= length_threshold) return true;
   else  return false;
}

void LaneChangeDecision::PrioritizeChangeLane(
         const bool change_line_to_direction, ChangeLane_Path& reference_line)
{
    if(reference_line.changelane_path.empty()) return;

    std::string pathId = GetCurrentPathId(reference_line);

    if(pathId == "left" && change_line_to_direction) {
        current_status = ChangeLaneStatus::Status::CHANGE_RIGHT;
    }else if(pathId == "center" && change_line_to_direction) {
        current_status = ChangeLaneStatus::Status::CHANGE_LEFT;
    }else if (pathId == "right" && change_line_to_direction) {
        current_status = ChangeLaneStatus::Status::CHANGE_LEFT;
    }else current_status = ChangeLaneStatus::Status::KEEP_LANE;

    UpdatePathStatus(current_status,pathId);
}

void LaneChangeDecision::UpdatePathStatus(
         ChangeLaneStatus::Status status_code, const std::string& path_id)
{
    trajectory_.pathId = path_id;
    trajectory_.pathStatus = status_code;
}

void LaneChangeDecision::BuildObstacleBox(LidarObjections *lidarobjs_global)
{
    obstacle.clear();
    
    int obj_nums = lidarobjs_global->objs.size();
    int subcon_num = 0;

    if(obj_nums < 1) return;
    
    for(int i = 0; i < obj_nums; i++) {// objections
        subcon_num = lidarobjs_global->objs[i].subCon.size();

        for(int j = 0; j < subcon_num; j++) {    
            sCellMsg center_point;

            double x0 = lidarobjs_global->objs[i].subCon[j].subVert[0].x;
            double y0 = lidarobjs_global->objs[i].subCon[j].subVert[0].y;
            double x2 = lidarobjs_global->objs[i].subCon[j].subVert[2].x;
            double y2 = lidarobjs_global->objs[i].subCon[j].subVert[2].y;

            center_point.xg = (x0 + x2) / 2.0;
            center_point.yg = (y0 + y2) / 2.0;

            obstacle.emplace_back(center_point);
        }
    }
}

bool LaneChangeDecision::IsObstacleBlocked(
         const ChangeLane_Path& reference_line)
{
    if(reference_line.changelane_path.empty()) return false;
   
    for(auto reference_line_:reference_line.changelane_path)
        if(collisionDetection(reference_line_)) return true;

    return false;
}                          

bool LaneChangeDecision::collisionDetection(const Point& reference_line) 
{  
    double wight_delt = 0.3;
    double x0 = reference_line.x;
    double y0 = reference_line.y;
    double l1 = vehicleparam_.rear_axis_to_front;
    double l2 = vehicleparam_.rear_axis_to_rear;
    double w = vehicleparam_.car_width / 2.0 + wight_delt;
    double psi = 0;

    Prk_Point p1,p2,p3,p4;

    p1.x = x0 + l1 * cos(psi) - w * sin(psi);
    p1.y = y0 + l1 * sin(psi) + w * cos(psi);
    p2.x = x0 - l2 * cos(psi) - w * sin(psi);
    p2.y = y0 - l2 * sin(psi) + w * cos(psi);
    p3.x = x0 - l2 * cos(psi) + w * sin(psi);
    p3.y = y0 - l2 * sin(psi) - w * cos(psi);
    p4.x = x0 + l1 * cos(psi) + w * sin(psi);
    p4.y = y0 + l1 * sin(psi) - w * cos(psi);

    for(int i = 0; i < obstacle.size(); i++)
        if(IsPointInRectangle(obstacle[i].xg,obstacle[i].yg,p1,p2,p3,p4))  return true;

    return false;
}

int LaneChangeDecision::FindNearestPoint2VehicleID2(
        const VehicleStatus car_status, ChangeLane_Path& path)
{
    int nearest_id = 0;
    float mini_dis = 1000000.0;
    int path_num = path.changelane_path.size();

    if(path_num < 1) return 0;

    for(int i = 0; i < path_num; i++) {
        double dx = path.changelane_path[i].x - car_status.g_x;
        double dy = path.changelane_path[i].y - car_status.g_y;
        double temp_dis = hypot(dx, dy);

        if(temp_dis < mini_dis) {
            mini_dis = temp_dis;
            nearest_id = i;
        }
    }

    return nearest_id;
}

void LaneChangeDecision::FreeLaneChange(bool is_free_lanechange)
{
    bool change_line_to_direction = false;
    int  nearest_id = FindNearestPoint2VehicleID2(car_status_, trajectory_);
    bool Lane_straight = IsLaneStraight(trajectory_, nearest_id);
    bool Lane_length_enougth = IsLanelengthEnough(trajectory_);
    bool Lane_blocked = IsObstacleBlocked(trajectory_);

    if(!is_free_lanechange) return;
    if(!Lane_straight) return;
    if(!Lane_length_enougth) return;
    if(Lane_blocked) return;

    change_line_to_direction = true;
    PrioritizeChangeLane(change_line_to_direction, trajectory_);
}

double LaneChangeDecision::GetCross(
           double x, double y, Prk_Point p1, Prk_Point p2)
{
    return (p2.x - p1.x) * (y - p1.y) - (x - p1.x) * (p2.y - p1.y);
}

bool LaneChangeDecision::IsPointInRectangle(
         double x, double y, Prk_Point p1, Prk_Point p2, Prk_Point p3, Prk_Point p4)
{
    double p12 = GetCross(x, y, p1, p2);
    double p34 = GetCross(x, y, p3, p4);
    double p23 = GetCross(x, y, p2, p3);
    double p41 = GetCross(x, y, p4, p1);

    if(((p12 * p34) >= 0) && ((p23 * p41) >= 0)) return 1;
    else return 0;
}
