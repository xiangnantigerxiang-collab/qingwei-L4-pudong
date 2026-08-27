#ifndef PERCEPTION_MSG_CONVERT
#define PERCEPTION_MSG_CONVERT

#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <sstream>
#include <fstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "ivmsglocpos.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "jsk_recognition_msgs/BoundingBoxArray.h"
#include "robot/object.h"
#include "robot/perception.h"
#include "robot/navigation_msg.h"

struct Vec2d {
    double x, y;
    Vec2d(double _x = 0, double _y = 0) : x(_x), y(_y) {}
};

class PerceptionBoundary {
public:
    std::vector<Vec2d> boundary_points_;
    
    bool LoadBoundary(const std::string& file_path);
    bool IsPointInBoundary(double x, double y) const;
    bool IsPointInBoundary(const Vec2d& point) const;

private:
    int Next(int at, int n) const { return at >= n - 1 ? 0 : at + 1; }
    double CrossProd(const Vec2d& p, const Vec2d& a, const Vec2d& b) const {
        return (a.x - p.x) * (b.y - p.y) - (a.y - p.y) * (b.x - p.x);
    }
};

#endif //PERCEPTION_MSG_CONVERT
