#include "collision_check_with_bbox.h"

CollisionCheckWithBBox::CollisionCheckWithBBox(CarModel &car_model) : car_model_(car_model)
{
}

// 第一步：AABB粗判，快速过滤完全不相交的OBB，true=AABB相交需要进SAT
bool CollisionCheckWithBBox::isAABBOverlap(const Obstacle &obb1, const Obstacle &obb2)
{
    // 求obb1的AABB四个边界
    double x1_min = obb1.x - obb1.width * fabs(obb1.cth) - obb1.length * fabs(obb1.sth);
    double x1_max = obb1.x + obb1.width * fabs(obb1.cth) + obb1.length * fabs(obb1.sth);
    double y1_min = obb1.y - obb1.width * fabs(obb1.sth) - obb1.length * fabs(obb1.cth);
    double y1_max = obb1.y + obb1.width * fabs(obb1.sth) + obb1.length * fabs(obb1.cth);

    // 求obb2的AABB四个边界
    double x2_min = obb2.x - obb2.width * fabs(obb2.cth) - obb2.length * fabs(obb2.sth);
    double x2_max = obb2.x + obb2.width * fabs(obb2.cth) + obb2.length * fabs(obb2.sth);
    double y2_min = obb2.y - obb2.width * fabs(obb2.sth) - obb2.length * fabs(obb2.cth);
    double y2_max = obb2.y + obb2.width * fabs(obb2.sth) + obb2.length * fabs(obb2.cth);

    // AABB分离条件：一个在另一个左边 / 右边 / 下边 / 上边 任意满足则无交集
    if (x1_max < x2_min || x2_max < x1_min)
        return false;
    if (y1_max < y2_min || y2_max < y1_min)
        return false;
    return true;
}

// 单条轴投影检测：返回true=该轴是分离轴（无碰撞）
bool CollisionCheckWithBBox::isSeparatedAxis(double nx, double ny,
                                             const Obstacle &a, const Obstacle &b)
{
    // 两OBB中心连线在轴上投影
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double dist_proj = dx * nx + dy * ny;
    double dist_abs = fabs(dist_proj);

    // OBBa在该轴上的投影半径
    double ra = a.width * fabs(a.cth * nx + a.sth * ny) + a.length * fabs(-a.sth * nx + a.cth * ny);
    // OBBb在该轴上的投影半径
    double rb = b.width * fabs(b.cth * nx + b.sth * ny) + b.length * fabs(-b.sth * nx + b.cth * ny);

    // 中心投影距离 > 两半半径之和 → 分离
    return dist_abs > (ra + rb);
}

// SAT核心：OBB碰撞检测入口
// return true = 两OBB发生碰撞
bool CollisionCheckWithBBox::isCollision(const Obstacle &obstacle, const Pose2d &ego_pose,
                                         const double &inflation_w, const double &inflation_l)
{
    OBB car_obb = getCarOBB(ego_pose, inflation_w, inflation_l);
    // 先粗筛AABB，不相交直接返回无碰撞
    OBB obstacle_obb = obstacle;
    obstacle_obb.width = obstacle.width / 2.0;
    obstacle_obb.length = obstacle.length / 2.0;
    if (!isAABBOverlap(car_obb, obstacle))
        return false;
    // 4条分离轴：car两个主轴、obs两个主轴
    // 轴1: car长轴 (cosθ, sinθ)
    if (isSeparatedAxis(car_obb.cth, car_obb.sth, car_obb, obstacle))
        return false;
    // 轴2: car短轴 (-sinθ, cosθ)
    if (isSeparatedAxis(-car_obb.sth, car_obb.cth, car_obb, obstacle))
        return false;
    // 轴3: obs长轴
    if (isSeparatedAxis(obstacle.cth, obstacle.sth, car_obb, obstacle))
        return false;
    // 轴4: obs短轴
    if (isSeparatedAxis(-obstacle.sth, obstacle.cth, car_obb, obstacle))
        return false;

    // 四条轴都不分离 → 碰撞
    return true;
}

OBB CollisionCheckWithBBox::getCarOBB(const Pose2d &ego_pose, const double &inflation_w, const double &inflation_l)
{
    double car_l = car_model_.base_to_front + car_model_.base_to_back + inflation_l; // 车长
    double car_w = car_model_.width + inflation_w;
    double car_yaw = ego_pose.heading;

    double d_c_b = (car_model_.base_to_front - car_model_.base_to_back) / 2.0; // center到base的距离
    double cx = ego_pose.x + d_c_b * cos(ego_pose.heading);
    double cy = ego_pose.y + d_c_b * sin(ego_pose.heading);

    OBB car_obb(cx, cy, car_w / 2.0, car_l / 2.0, car_yaw);
    return car_obb;
}
