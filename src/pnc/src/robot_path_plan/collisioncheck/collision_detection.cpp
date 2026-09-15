#include "collision_detection.h"

// 判断点是否在矩形内部 (射线法)
bool isPointInsideRect(const Point& p, const std::vector<Point>& rect) {
    int numIntersections = 0;
    for(int i = 0; i < 4; i++) {
        const Point& p1 = rect[i];
        const Point& p2 = rect[(i + 1) % 4];
        if(p1.y == p2.y && p.y == p1.y && p.x >= std::min(p1.x, p2.x) && p.x <= std::max(p1.x, p2.x)) {
            return true;  // 点在矩形边上
        }
        if((p1.y > p.y) != (p2.y > p.y) && p.x < (p2.x - p1.x) * (p.y - p1.y) / (p2.y - p1.y) + p1.x) {
            numIntersections++;
        }
    }
    return numIntersections % 2 == 1;  // 奇数个交点表示点在矩形内部
}

// 判断两个矩形是否相交 (分离轴定理)
bool isRectIntersect(const std::vector<Point>& rect1, const std::vector<Point>& rect2) {
    auto project = [](const std::vector<Point>& rect, const Point& axis) {
        double minVal = std::numeric_limits<double>::max();
        double maxVal = std::numeric_limits<double>::lowest();
        for(const Point& p : rect) {
            double proj = p.x * axis.x + p.y * axis.y;
            minVal = std::min(minVal, proj);
            maxVal = std::max(maxVal, proj);
        }
        return std::make_pair(minVal, maxVal);
    };

    for(int i = 0; i < 4; i++) {
        Point axis1 = {rect1[(i + 1) % 4].y - rect1[i].y, rect1[i].x - rect1[(i + 1) % 4].x};
        Point axis2 = {rect2[(i + 1) % 4].y - rect2[i].y, rect2[i].x - rect2[(i + 1) % 4].x};
        auto proj1 = project(rect1, axis1);
        auto proj2 = project(rect2, axis1);
        auto proj3 = project(rect1, axis2);
        auto proj4 = project(rect2, axis2);

        if(proj1.second < proj2.first || proj2.second < proj1.first || proj3.second < proj4.first || proj4.second < proj3.first) {
            return false;  // 在某个轴上投影不相交
        }
    }
    return true;  // 在所有轴上投影都相交
}

CollisionCheck::CollisionCheck(
    const std::vector<std::vector<TrajectoryPoint>>& trajectorypoint,
    const VehicleParamConfig& vehicle_param, std::vector<sCellMsg> lidarobjs_global)
    : trajectorypoint_(trajectorypoint), vehicle_param_(vehicle_param) {
    GetObstacleCentral(lidarobjs_global);
    BuildObstacleBox(lidarobjs_global);
}

std::vector<Point> CollisionCheck::GetRect(Point p) {
    double theta = (90.0 - p.heading) * M_PI / 180.0;

    while(theta > M_PI) theta -= 2.0 * M_PI;
    while(theta <= -M_PI) theta += 2.0 * M_PI;

    std::vector<Point> points;
    points.resize(4);

    double l = p.length / 2.0;
    double w = p.width / 2.0;

    points[0].x = l * cos(theta) - w * sin(theta) + p.x;
    points[0].y = l * sin(theta) + w * cos(theta) + p.y;
    points[1].x = l * cos(theta) + w * sin(theta) + p.x;
    points[1].y = l * sin(theta) - w * cos(theta) + p.y;
    points[2].x = -l * cos(theta) + w * sin(theta) + p.x;
    points[2].y = -l * sin(theta) - w * cos(theta) + p.y;
    points[3].x = -l * cos(theta) - w * sin(theta) + p.x;
    points[3].y = -l * sin(theta) + w * cos(theta) + p.y;

    return points;
}

double CollisionCheck::CheckRelation(
    std::vector<Point>& rect1, std::vector<Point>& rect2, std::vector<Point>& res) {
    // 特殊情况：相交或包含
    if(isRectIntersect(rect1, rect2)) return 0;
    if(isPointInsideRect(rect1[0], rect2)) return -1;
    if(isPointInsideRect(rect2[0], rect1)) return -2;

    // 一般情况：分离
    double minDist = std::numeric_limits<double>::max();

    res.clear();
    res.resize(2);

    for(const Point& p1 : rect1) {
        for(const Point& p2 : rect2) {
            double dist = hypot(p1.x - p2.x, p1.y - p2.y);
            if(dist < minDist) {
                minDist = dist;
                res[0] = p1;
                res[1] = p2;
            }
        }
    }

    return hypot(res[0].x - res[1].x, res[0].y - res[1].y);
}

std::vector<std::vector<TrajectoryPoint>> CollisionCheck::Collisionproperty() {
    for(int i = 0; i < trajectorypoint_.size(); ++i) {
        double path_cost = trajectorypoint_[i].back().cost();
        auto statu = CollisionCost(trajectorypoint_[i], i);

        double path_collision_cost = std::get<0>(statu);
        bool is_collision = std::get<1>(statu);

        path_cost += path_collision_cost;
        trajectorypoint_[i].back().setCost(path_cost);
        trajectorypoint_[i].back().setSafeProperty(is_collision);
    }

    return trajectorypoint_;
}

std::tuple<double, bool> CollisionCheck::CollisionCost(
    const std::vector<TrajectoryPoint>& trajectory, int path_id) {
    double wight_offset = 1.5;
    double ego_width_half = vehicle_param_.car_width / 2.0;
    double ego_length = vehicle_param_.car_length;
    double safe_radiu = ego_width_half + wight_offset;
    double collision_cost_ = 0.0;
    double collision_cost = 0.0;
    double path_range_number = 50;
    bool is_path_collision = false;

    if(trajectory.size() <= path_range_number) {
        path_range_number = trajectory.size();
    }

    for(size_t i = 0; i < path_range_number; i++) {
        for(auto& obstacle_ : obstacle) {
            double dx = trajectory[i].path_point.x() - obstacle_.xg;
            double dy = trajectory[i].path_point.y() - obstacle_.yg;
            double distance_path_to_obj = std::hypot(dx, dy);

            if(distance_path_to_obj <= safe_radiu) {
                is_path_collision = true;

                if(distance_path_to_obj < 0.1) distance_path_to_obj = 0.5;
                collision_cost = config_.getCollisionConfig().obstacleCollisionCost / distance_path_to_obj;
            } else {
                collision_cost = config_.getCollisionConfig().obstacleNoCollision;
            }

            collision_cost_ += collision_cost;
        }
    }

    return std::forward_as_tuple(collision_cost_, is_path_collision);
}

bool CollisionCheck::InCollision(
    const std::vector<TrajectoryPoint>& trajectory) {
    double wight_offset = 0.2;
    double ego_length = vehicle_param_.car_length;
    double ego_width = vehicle_param_.car_width + wight_offset;

    for(size_t i = 0; i < trajectory.size(); ++i) {
        const auto& trajectory_point = trajectory[i];
        double ego_theta = trajectory_point.path_point.theta();

        Box2d ego_box(
            {trajectory_point.path_point.x(), trajectory_point.path_point.y()},
            ego_theta, ego_length, ego_width);

        double shift_distance =
            ego_length / 2.0 - vehicle_param_.back_edge_to_center;

        Vec2d shift_vec{shift_distance * std::cos(ego_theta),
                        shift_distance * std::sin(ego_theta)};

        ego_box.Shift(shift_vec);

        for(const auto& obstacle_box : obstacle_bound_rectangles_) {
            if(ego_box.HasOverlap(obstacle_box)) return true;
        }
    }

    return false;
}

void CollisionCheck::GetObstacleCentral(std::vector<sCellMsg> lidarobjs_global) {
    obstacle.clear();

    int obj_nums = lidarobjs_global.size();

    if(obj_nums < 1) return;

    for(int i = 0; i < obj_nums; i++) {
        sCellMsg center_point;
        center_point.xg = lidarobjs_global[i].xg;
        center_point.yg = lidarobjs_global[i].yg;
        center_point.heading = lidarobjs_global[i].heading;
        obstacle.emplace_back(center_point);
    }
}

void CollisionCheck::BuildObstacleBox(std::vector<sCellMsg> lidarobjs_global) {
    obstacle_bound_rectangles_.clear();

    for(auto& obstacle_ : obstacle) {
        Box2d obstacle_box({obstacle_.xg, obstacle_.yg},
                           obstacle_.heading, 0.15, 0.15);

        obstacle_bound_rectangles_.push_back(obstacle_box);
    }
}
