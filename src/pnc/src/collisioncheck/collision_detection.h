#ifndef _COLLISIONDETECTION
#define _COLLISIONDETECTION

#include <vector>
#include <iostream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <unordered_map>
#include "common/pnc_point/trajectory_point.h"
#include "common/config/vehicle_param_config/vehicle_param_config.h"
#include "common/surface/box2d.h"
#include "common/struct_type.h"
#include "common/config/dp_poly_path_config.h"

class CollisionCheck 
{
public:
    CollisionCheck() = default;

    explicit CollisionCheck(
                 const std::vector<std::vector<TrajectoryPoint>> &trajectorypoint,
                 const VehicleParamConfig &vehicle_param,
                 std::vector<sCellMsg> lidarobjs_global);

    std::vector<std::vector<TrajectoryPoint>> Collisionproperty();

    std::tuple<double,bool> CollisionCost(
        const std::vector<TrajectoryPoint> &trajectory, int path_id);

    std::vector<Point> GetRect(Point p);

    double CheckRelation(
        std::vector<Point> &rect1, std::vector<Point> &rect2, std::vector<Point> &res);

private:
    void BuildObstacleBox(std::vector<sCellMsg> lidarobjs_global);
    void GetObstacleCentral(std::vector<sCellMsg> lidarobjs_global);
    bool InCollision(const std::vector<TrajectoryPoint>& trajectory);

    std::vector<Box2d> obstacle_bound_rectangles_;
    std::vector<std::vector<TrajectoryPoint>> trajectorypoint_;
    VehicleParamConfig vehicle_param_;
    std::vector<sCellMsg> lidarobjs_global_;
    DpPolyPathConfig config_;
    std::vector<sCellMsg> obstacle;
};

#endif //_COLLISIONDETECTION
