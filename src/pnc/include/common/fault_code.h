#ifndef FAULT_CODE_H
#define FAULT_CODE_H

#define NONEERROR_D (0)
#define SERIOUTERROR_D (1)
#define LOGICALERROR_D (2)
#define TEMPORARYERROR_D (3)

typedef enum fault_code {
    NONE = 0,  //无故障

    //严重故障
    BRAKE_FAULT = 1,              //刹车故障
    WHEEL_FAULT = 2,              //转向故障
    VEHICLE_BODY_FAULT = 3,       //车身故障
    COUPLE_FAULT = 4,             //托挂钩执行机构故障
    VEHICLE_COMM_FAULT = 5,       //车身通讯故障
    SOFT_FAULT = 6,               //软件故障(指的是软件模块程序挂掉)
    GPS_NOFIXED = 7,              //GPS差分丢失
    SICK_FAULT = 8,               //单线雷达故障
    FRONT_LIDAR16_FAULT = 9,      //前向16线雷达故障
    REAR_LIDAR16_FAULT = 10,      //后向16线雷达故障
    CAMERA_FAULT = 11,            //摄像头
    RADAR_FAULT = 12,             //毫米波故障
    LEFT_TIM_FAULT = 13,          //左侧Tim故障
    RIGHT_TIM_FAULT = 14,         //右侧Tim故障
    FRONT_ULTRASOUND_FAULT = 15,  //超声波前向故障
    REAR_ULTRASOUND_FAULT = 16,   //超声波后向故障
    LEFT_ULTRASOUND_FAULT = 17,   //超声波左向故障
    RIGHT_ULTRASOUND_FAULT = 18,  //超声波右向故障
    PLATFORM_POS_FAULT = 19,      //平台车位置错误

    //逻辑故障
    LOCATION_PATH_FAULT = 31,  //路径搜索不到错误
    FMS_COMM_FAULT = 32,       //FMS通讯故障(包含超时和数据错误)
    SOFT_LOGIC_FAULT = 33,     //软件逻辑故障
    OFFSET_PATH__FAULT = 34,   //路径偏移错误。

    //暂时性故障
    SAFE_TOUCH_FAULT = 51,
    SICK_DETECT_OBSTACLE = 52,
    FRONT_LIDAR16_DETECT_OBSTACLE = 53,
    REAR_LIDAR16_DETECT_OBSTACLE = 54,
    CAMERA_DETECT_OBSTACLE = 55,
    RADAR_DETECT_OBSTACLE = 56,
    LEFT_TIM_DETECT_OBSTACLE = 57,
    RIGHT_TIM_DETECT_OBSTACLE = 58,
    FRONT_ULTRASOUND_DETECT_OBSTACLE = 59,
    REAR_ULTRASOUND_DETECT_OBSTACLE = 60,
    LEFT_ULTRASOUND_DETECT_OBSTACLE = 61,
    RIGHT_ULTRASOUND_DETECT_OBSTACLE = 62,
} FAULT_CODE_E;

#endif
