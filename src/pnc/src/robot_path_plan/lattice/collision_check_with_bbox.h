#ifndef COLLISION_CHECK_WITH_BBOX_H
#define COLLISION_CHECK_WITH_BBOX_H

#include <nav_msgs/OccupancyGrid.h>
#include "planner_utils.h"
#include "pose2d.h"

struct Obstacle
{
    double x;
    double y;
    double heading;
    double width;
    double length;
    double cth = 1.0;
    double sth = 0.0;
    std::vector<int> history = {1, 1, 1, 1,1,1,1,1};
    Obstacle() = default;
    Obstacle(double obstacle_cx, double obstacle_cy,
                       double obstacle_w, double obstacle_l, double obstacle_yaw)
    {
        x = obstacle_cx;
        y = obstacle_cy;
        width = obstacle_w;
        length = obstacle_l;
        heading = obstacle_yaw;
        cth = std::cos(heading);
        sth = std::sin(heading);
    }
    int historySum()
    {
        int sum = 0;
        for (int i = 0; i < history.size(); i++)
        {
            sum += history[i];
        }
        return sum;
    }
};
typedef std::shared_ptr<Obstacle> ObstaclePtr;
typedef Obstacle OBB;

class CollisionCheckWithBBox
{
public:
    CollisionCheckWithBBox() = delete;
    ~CollisionCheckWithBBox() = default;
    CollisionCheckWithBBox(CarModel &car_model);

    bool isCollision(const Obstacle &obstacle, const Pose2d &ego_pose,
                     const double &inflation_w = 0.0, const double &inflation_l = 0.0);
    CarModel getCarModel() { return car_model_; }

private:
    bool isAABBOverlap(const Obstacle &obb1, const Obstacle &obb2);
    bool isSeparatedAxis(double nx, double ny, const Obstacle &a, const Obstacle &b);

    OBB getCarOBB(const Pose2d &ego_pose, const double &inflation_w = 0.0, const double &inflation_l = 0.0);

private:
    CarModel car_model_;
};

typedef std::shared_ptr<CollisionCheckWithBBox> CollisionCheckWithBBoxSPtr;

#endif