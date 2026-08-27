

#pragma once

#include <cstdio>
//#define SIMULATION
#include <cmath>

/// Message definitions

#ifndef SIMULATION
#define OUTLABLE "VehicleState"
#else
#define OUTLABLE "VehicleOut"
#endif

#define HEARTBEAT "heartbeat"
#define INS_DATA "InsData"
#define TRAJECTORY "TRAJECTORY"
#define LIDAR_BROAD "1myBroadcast"
#define LIDAR_BROAD_UPPER "uplidar"
#define RADAR "RadarData"
#define MO_OBJS "MO_Objs"
#define VISUAL_LANE_DECTECTION "MULTILANEDETECTION"
#define LIGHT_SIGNAL "VLIGHT"
#define FUSION_OBJECTS "SensorFusion"
#define LIDARAREADETECTION "tunnel_info"
#define CAMERA_DETECTION "CAMERADETECTION"
#define IMU_DATA "ImuData"

//曼哈顿距离
#define MANHATTAN_DIS(x1,y1,x2,y2) (fabs(x1 - x2) + fabs(y1 - y2))

//欧式距离
#define EUCLIDEAN(x1,y1,x2,y2) (sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 -y2)))

//定义生成候选轨迹宽度
#define CANDIDATEWIDTHINTERVAL 0.2

//定义轨迹计算heading时的间隔
#define HEADING_INTERVAL 0.1

//物体预测时间周期
#define PREDICT_PERIOD 0.5

//车辆大小
#define CAR_SIZE_WIDTH 2.11
#define CAR_SIZE_LONG 4.765
//最近距离
#define MIN_DIS 1000

//时间限制
#define TIME_THRESHOLD 0.5

//车道宽度
#ifndef SIMULATION
#define LANE_WIDTH 3.2001

#else
#define LANE_WIDTH 5.2001
#endif

// 组合定位结果按照*100000000 之后作为整形发出，因此此处接收到需要转化
#define SCALE   100000000.0
#define HEADINGSCALE 10000.0

//决策层
#define CRASH_COST 2.0

//周期
#define TIME_PERIOD 100000

//横向最大加速度
#ifndef SIMULATION
#define VERTICAL_MAX_ACC 1.0
#else
#define VERTICAL_MAX_ACC 2.0
#endif

//提前减速距离
#define BRAKE_FORWARD_DISTANCE 20

//车辆航向角与惯导航向角的偏差度数，右偏为负，左为正
#ifndef SIMULATION
#define B_COURSE_ANGLE_DEVIATION 0.5   //wei 0.0 wei1 0 wei2 0
#else
#define B_COURSE_ANGLE_DEVIATION 0.0
#endif

//雷达和cpt位置偏差
#ifndef SIMULATION
#define LIDAR_TO_CPT 3.9
#else
#define LIDAR_TO_CPT 1.5
#endif

//毫米波和cpt位置偏差
#define RADAR_TO_IGM 3.8

//视觉看到的起点距离车辆后备箱的距离
#define VISUAL_TO_CPT 7.5

//将车扩大一圈
#ifndef SIMULATION
#define ZOOM_SIZE 0.0
#else
#define ZOOM_SIZE 0.0
#endif

#define L 2.95        //轴距
#define WHEELANGLERATIO 16.0

//定义速度下的安全长度问题
#define SAFELENGTH(v) 3*(v) + 1

#define MAX_COST_VALUE 10000

#define HUMAN_MIN_SPEED 1.0
#define CAR_MIN_SPEED 1.0

const bool FLAGS_use_navigation_mode = false;
const double FLAGS_centralMinDiff = 0.001;
const double FLAGS_replan_lateral_distance_threshold = 5.0;
const double FLAGS_replan_longitudinal_distance_threshold = 5.0;

