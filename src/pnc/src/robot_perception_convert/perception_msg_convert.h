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
#include "perception_temporal_filter.h"
#include "perception_confidence_filter.h"

struct Vec2d {
    double x, y;
    Vec2d(double _x = 0, double _y = 0)
        : x(_x), y(_y) {
    }
};

class PerceptionBoundary {
public:
    // 排除区域多边形集合：配置目录下每个 .csv 文件对应一个多边形，文件内每行一个 "x,y" 顶点
    std::vector<std::vector<Vec2d>> polygons_;

    // dir_path 为配置目录，加载其中所有 .csv 文件为排除多边形
    bool LoadBoundary(const std::string& dir_path);
    // 点落在任一多边形内返回 true（调用处据此剔除障碍物）
    bool IsPointInExclusion(double x, double y) const;
};

#endif  //PERCEPTION_MSG_CONVERT
