#ifndef STRUCT_TYPE_H
#define STRUCT_TYPE_H

#include  <iostream>
#include <vector>

#define MIN_ 0.0001
#include<array>
#include<string>

using Vec_f=std::vector<float>;
using Poi_f=std::array<float, 2>;
using Vec_Poi=std::vector<Poi_f>;

typedef struct xyz_coor_s
{
    xyz_coor_s(float x_axis = 0, 
	       float y_axis = 0, 
	       float z_axis = 0,
	       float heading = 0,
	       float p2pDistance = 0,
               float curvature = 0,
               float velocity = 0,
               float dist_origin = 0)
    {
        this->x_axis = x_axis;
        this->y_axis = y_axis;
        this->z_axis = z_axis;
        this->heading = heading;
        this->p2pDistance = p2pDistance;
        this->curvature = curvature;
        this->velocity = velocity;
        this->dist_origin = dist_origin;
    }

    float x_axis;
    float y_axis;
    float z_axis;
    float heading;
    float p2pDistance;
    float curvature;
    float velocity;
    float dist_origin;//Distance from the origin
}XYZ_COOR_S;

typedef enum work_status
{
    PAUSEWORKINGSTATUSMODE = 0,
    MANUALCONTROLMODE,
    NOMALWORKINGMODE,
    REMOTEMODE,
    FAULTMODE,
    WRAPAROUNDOBSTABLESMODE,
    REVERTTOORIGINPATH,
}WORK_STATUS;

typedef enum init_step
{
    NONE_INIT = 0,
    RTK_FIXED,
    SELF_CHECK,
    DOWNLOAD_MAP,
    NOMAL_RUNNING,
}INIT_STEP_E;

typedef enum subActionValue
{
    NO_OPERATION = 0,
    GEAR_N = 2,
    GEAR_R = 3,
    GEAR_D = 4,
    HOOKOPERATION = 5,
    DECOUPLING = 6,
    WAITING = 7,
    COUNTERPOINT = 8,
    ASKPALLETPOS = 9,
    CALCUBACKPATH = 10,
    UNLOAD_LOWER = 11,
}SUBACTION_E;

typedef enum faultCode
{
    NORMALWORK = 0,
    RTKNOTFIXED = 1,
    VEHICLEFAULT = 2,
    SOFTWAREFAULT = 3,	
    OFFLANEFAULT = 4,
}FAULTCODE;

typedef enum taskType
{
    NOTHING = 0,
    TRACKPATH = 1,
    ADAPTIVELOADPATH = 2,
    ADAPTIVEUNLOADPATH = 3,
    ADAPTIVEHOOK = 4,
    ADAPTIVEBACK = 5,
    DOACTION = 6,
    ADAPTIVEPARK = 7,
}TASKTYPE_E;

typedef enum taskStatus
{
    NOTASK = 0,
    TASKINPROGRESS = 1,
    TASKFINISHED = 2,
}TASKSTATUS;

typedef enum pathType
{
    INVALID = 0,
    FORWARD = 1,
    BACKWARD = 2,
    ADAPTIVE = 3,
}PATHTYPE;

typedef struct fms_path_s
{
    std::string id;
    XYZ_COOR_S start_point;
    XYZ_COOR_S end_point;
    std::vector<float> x_array;
    std::vector<float> y_array;
    std::string path_head;
    
    std::string left_lamp;
    int left_lamp_start;
    int left_lamp_end;
    std::string right_lamp;
    int right_lamp_start;
    int right_lamp_end;
    std::string whistle;
    int whistle_start;
    int whistle_end;

    std::string allow_detour;
    int allow_detour_start;
    int allow_detour_end;

    std::string absolute_speed;
    std::string relative_speed;
    PATHTYPE path_type;
}FMS_PATH_S;

typedef struct path_info
{
    std::string curPathName;
    std::vector<XYZ_COOR_S> curPathList;
    std::string next1PathName;
    std::vector<XYZ_COOR_S> next1PathList;
    std::string next2PathName;
    std::vector<XYZ_COOR_S> next2PathList;
}PATH_INFO_S;

struct LidarPos
{
    double x;
    double y;
    double heading;
    double curv;
    double length;
    LidarPos()
    {
        x = 0.0; // x is the initial direction of the vehicle's longitudinal axis.
        y = 0.0; // y is vertical x and it's direction is right.
        heading = 0.0;// left to right [-180, 180]
        curv = 0.0;
        length = 0.0;
    }
};

struct LidarSubContourInfo
{
    std::vector<LidarPos> subVert;
};

struct LidarObjInfo
{
    int8_t type; // 类型，0：未识别，1：车辆（暂不区分车辆类型）2：行人 3：自行车。。
    int16_t trackId0;
    float speed0; // 0.01m/s
    int16_t age0;
    float dist0;// 到当前自车的直线距离 cm
    float heading;//orient0;// 预测目标OBJ行驶方向，0.1度，正北为0，顺时针为正 [0, 3600]
    int16_t acc; // 加速度大小 0.01m/s^2
    int16_t accOrient; // 加速度方向 注：毫米波加速度方向延速度方向或反方向，根据正负给出方向，加速度大小为绝对值即可
    int16_t    contourHeight0; // cm
    LidarPos   big_vert[5];//float    contour0[8]; // 到当前自车x、y轴方向的距离，两个一对
//    LidarPos cent_p;
    int16_t    subConNum;
    std::vector< LidarSubContourInfo > subCon;
    LidarObjInfo()
    {
        type = 0; // 类型，0：未识别，1：车辆（暂不区分车辆类型）2：行人 3：自行车。。
        trackId0 = 0;
        speed0 = 0.0; // 0.01m/s
        age0 = 0;
        dist0 = 0;// 到当前自车的直线距离 cm
        heading = 0.0;// 预测目标OBJ行驶方向，0.1度，正北为0，顺时针为正 [0, 3600]
        acc = 0; // 加速度大小 0.01m/s^2
        accOrient = 0; // 加速度方向 注：毫米波加速度方向延速度方向或反方向，根据正负给出方向，加速度大小为绝对值即可
        contourHeight0 = 0; // cm
        // central point
        subConNum = 0;
        subCon.clear();
    }
};

struct LidarObjections
{
    pthread_mutex_t mtx;
    int64_t    longitude;// 经度 乘以10的8次方发送
    int64_t    latitude;// 纬度 乘以10的8次方发送
    int16_t    heading;// 车辆heading, 单位0.1°，取值范围0~3600
    int64_t    timestamp0;
    int32_t    objNum;
    std::vector< LidarObjInfo > objs;
    LidarObjections()
    {
        longitude = 0;
        latitude = 0;
        heading = 0;
        timestamp0 = 0;
        objNum  = 0;
        objs.clear();
    }
};

struct OriginalInsData
{
    double x;
    double y;
    double heading;
    double length;
    double speed;
    int type;
    double kappa;
    double dkappa;
    int Id_path;
    bool safety;
};

struct SDPoint
{
    double s;
    double s_dot;
    double s_ddot;
    double d;
    double d_prime;
    double d_pprime;
};

struct sCellMsg
{
    double xg;
    double yg;
    double heading;
};

class Point
{
public:
    Point(double x_val, double y_val) : x(x_val), y(y_val) {}
    Point() {}

    float x = 0.0;  
    float y = 0.0;       
    float length = 0.0;
    float width = 0.0;
    float curv = 0.0;
    float speed = 0.0;
    float heading = 0.0; 
    float pitch = 0.0;
    float roll = 0.0;
};

struct Prk_Point
{
    double x; //单位:m
    double y; //单位:m
    double angle; //单位:°
    double len; // m
    Prk_Point()
    {
        x     = 0.0;
        y     = 0.0;
        angle     = 0.0;
        len    = 0.0;
    }
    void reset()
    {
        x     = 0.0;
        y     = 0.0;
        angle  = 0.0;
        len    = 0.0;
    }
};

struct ChangeLaneStatus
{
     enum Status
     {
         KEEP_LANE,
         CHANGE_LEFT,
         CHANGE_RIGHT,
     };
     Status status;
     ChangeLaneStatus()
     {
         status = KEEP_LANE;
      }
 };

 struct ChangeLane_Path
{
    std::string pathId;
    ChangeLaneStatus::Status pathStatus;
    std::vector<Point> changelane_path;
    ChangeLane_Path()
    {
        pathId = "center";
        pathStatus = ChangeLaneStatus::Status::KEEP_LANE;
        changelane_path.clear();
    }
    void reset()
    {   
        pathId = "center";
        pathStatus = ChangeLaneStatus::Status::KEEP_LANE;
        changelane_path.clear();
    }
};

typedef enum _DriveState
{
    NORMAL_STOP_ = 0,
    FORWARD_,
    BACKWARD_,
    FOLLOW_,
    STOP_BY_OBSTACALE
} DriveState;

struct VehicleStatus
{
    pthread_mutex_t mtx;
    int32_t time;
    bool  inflame;  // true:inflame, false: flameout
    double g_x;          // m
    double g_y;          // m
    int32_t next_map;
    float mileage;      // mileage += speed*time; obtained by integrating the speed(m/s) when the vehicle starts to drive or getts a new map.
    float speed;
    float acc;
    float heading;      //int16_t heading;   //unit: 0.01
    float pitch;
    float roll;
    float steeringangle;
    int gear;           // 0：D；1：R；2：P；3：N
    float throttle;
    float brake;
    bool is_auto;
    int drive_state;
    VehicleStatus()
    {
        time = 0;
        inflame = true;
        g_x     = 0;
        g_y     = 0;
        next_map = -1;
        mileage = 0.0;
//        curv    = 0.0;
        speed   = 0.0;
        acc     = 0.0;
        heading = 0.0;
        pitch         = 0.0;
        roll          = 0.0;
        steeringangle = 0.0;
        gear          = 2;
        throttle      = 0.0;
        brake         = 0.0;
        is_auto       = false;
        drive_state   = NORMAL_STOP_;
    }
    void reset()
    {
        inflame = true;
        g_x     = 0;
        g_y     = 0;
        mileage = 0.0;
//        curv    = 0.0;
        speed   = 0.0;
        acc = 0.0;
        heading = 0.0;
        pitch   = 0.0;
        roll    = 0.0;
        steeringangle = 0.0;
        gear          = 2;
        throttle      = 0.0;
        brake         = 0.0;
        is_auto       = false;
        drive_state   = NORMAL_STOP_;
    }
};

struct sObjPosInfo
{
  std::string type;
  double xg;
  double yg;
  double heading;
  double s;
  double l;
  double dist;
  double Front_dist;
  double Beside_dist;
  double FrontLength;
  double BesideLength;
  sObjPosInfo() {
    type = "NONE";
    xg = 888.0;
    yg = 888.0;
    heading = 888.0;
    s = 88888.0;
    l = 88888.0; 
    Front_dist = 888.0;
    Beside_dist = 888.0;
    FrontLength = 888.0;
    BesideLength = 888.0;
  }
  void reset() {
    type = "NONE";
    xg = 0;
    yg = 0;
    heading = 888.0;
    s = 88888.0;
    l = 88888.0;
    Front_dist = 888.0;
    Beside_dist = 888.0;
    FrontLength = 888.0;
    BesideLength = 888.0;
  }
};

typedef struct sObjPos
{
  int posid;
  double dist;
  double length;
} sObjPos;

#endif
