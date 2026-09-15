// Pose2d 二维位姿基础类型：x/y/heading 及距离、角度工具
// 提供距离、平方距离计算与加减、数乘、一元负号等位姿运算
#ifndef ROBOT_CONTROL_STANLEY_CONTROLLER_POSE2D_H_
#define ROBOT_CONTROL_STANLEY_CONTROLLER_POSE2D_H_

#include <cmath>

class Pose2d {
public:
    Pose2d(const double in_x, const double in_y, const double in_heading = 0.0)
        : x(in_x), y(in_y), heading(in_heading) {
    }
    Pose2d() = default;
    ~Pose2d() = default;

    //! Returns the distance to the given vector
    double distanceTo(const Pose2d& other) const {
        return hypot(x - other.x, y - other.y);
    }

    //! Returns the squared distance to the given vector
    double distanceSquareTo(const Pose2d& other) const {
        const double dx = x - other.x;
        const double dy = y - other.y;
        return dx * dx + dy * dy;
    }

    // 加
    Pose2d operator+(const Pose2d& other) const {
        return Pose2d(x + other.x, y + other.y);
    }

    // 减
    Pose2d operator-(const Pose2d& other) const {
        return Pose2d(x - other.x, y - other.y);
    }

    // 一元负号
    Pose2d operator-() const {
        return Pose2d(-x, -y, heading);
    }

    Pose2d operator*(double scalar) const {
        return Pose2d(x * scalar, y * scalar, heading);
    }

    friend Pose2d operator*(double scalar, const Pose2d& p) {
        return Pose2d(p.x * scalar, p.y * scalar, p.heading);
    }

public:
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;

    double curvature = 0.0;
    double s = 0.0;
    double v = 0.0;
    double a = 0.0;
};

#endif
