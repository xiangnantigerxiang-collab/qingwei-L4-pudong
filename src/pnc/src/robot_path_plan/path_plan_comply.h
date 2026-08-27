#ifndef PATH_PLAN_COMPLY
#define PATH_PLAN_COMPLY

#include <jsoncpp/json/json.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <iostream>
#include <vector>
#include <algorithm>

#include "geometry_utils.h"
#include "adaptiveHook.h"
#include "common/pnc_point/trajectory_point.h"
#include "common/surface/vec2d.h"
#include "common/fault_code.h"
#include "../lattice_plan/lattice_planner.h"
#include "../common/pubalgor/pubalgor.h"
#include "jsk_recognition_msgs/BoundingBoxArray.h"
#include "reference_line/reference_line.h"
#include "reference_line/reference_line_provider.h"
#include "robot/can_msg.h"
#include "robot/hook_position.h"
#include "robot/navigation_msg.h"
#include "robot/object.h"
#include "robot/palletpos.h"
#include "robot/path_plan_msg.h"
#include "robot/path_plan_status.h"
#include "robot/perception.h"
#include "robot/sound_light_msg.h"
#include "robot/task_plan_msg.h"
#include "robot/rs232.h"
#include "robot/AirCraftParkingPort.h"
#include "sensor_msgs/CompressedImage.h"
#include "ros/ros.h"
#include "speedplan/speed_planning.h"
#include "robot/v2nCommandFeedback.h"
#include "robot/CommandMsg.h"
#include "common/struct_type.h"
#include "../common/spline/Spline.h"
#include "../trans/trans_data.h"
#include <ros/ros.h>
#include "lattice/my_lattice_planner.h"

using namespace planning;
using namespace adaptive;
namespace bg = boost::geometry;
using GeoPoint = bg::model::d2::point_xy<double>;
using GeoLine = bg::model::segment<GeoPoint>;
using GeoBox = bg::model::box<GeoPoint>;

struct AirCraftParkingPort{
    int id; //飞机位id
    uint parking = 0; //0 非入位中 1 入位中
    Point go_stop_point;
    Point back_stop_point;
    int go_stop_index = 0;
    int back_stop_index = 0;
};

class PathPlanComply
{
public:
    PathPlanComply();
    ~PathPlanComply();
    void InitParameter();
    void ResetParameter();
    void loadAirCraftPorts();
    // set parameter
    void SetTaskPlanData(robot::task_plan_msg task_plan_t);
    void SetCanData(robot::can_msg can_msg_t);
    void SetNavigationData(robot::navigation_msg navigation_msg_t);
    void SetPerceptionData(robot::perception perception_t);
    void SetFrontScanData(jsk_recognition_msgs::BoundingBoxArray msg);
    void SetBackScanData(jsk_recognition_msgs::BoundingBoxArray msg);
    void SetPalletCoorData(robot::hook_position hook_pos_t);
    void SetPerceptionData2(robot::perception perception_t);
    void SetLoadPosition(robot::palletpos load_pos_t);
    void SetButtonData(robot::rs232 button_data_t);
    void SetCommandState(int in_command) { command_state = in_command; }
    void resetTask();
    
    void updateAirPortStopIndex(std::vector<XYZ_COOR_S> &global_path, bool is_go_path);
    void updateAirPortInfo(robot::AirCraftParkingPort airport);
    AirCraftParkingPort findNextAirCraftParkingPort();

    // main function
    void PathPlanProcess();
    void PublishPathPlanStatus(ros::Publisher &tPub);
    void PublishReferPath(ros::Publisher &tPub);
    void PublishPlanPath(ros::Publisher &tPub1, ros::Publisher &tPub2);

    void pubLatticeTrajs(ros::Publisher &tPub);
    void pubObstacles(ros::Publisher &tPub);

    void handleDrivingPath();
double getLaneLimitSpeedTest(double cur_lane_speed, double next_lane_speed);
    bool checkAroundObstacle();
    bool checkTJunctionObstacle();
    bool checkIsInWaiting();
ObstaclePtr findObstacle(std::vector<ObstaclePtr> &obstacles, double x, double y);

    std::vector<GeoPolygon> obstacle_obbs_;
    GeoPolygon vehicle_obb_;

    int sumVec(std::vector<bool> &vec)
    {
        int sum = 0;
        for (int i = 0; i < vec.size(); i++)
        {
            sum += int(vec[i]);
        }
        return sum;
    }

    int AccSwitch = 0;
    int PalletType = 0; //0-static pallet pos 1-reflect pallet pos
    int LaneChangeRequset = 0; // 0-none 1-left 2-right
    int workMode = 0;
    int InitSafetyCheck = 0;
    int emergencyStop = 0;

    std::vector<OriginalInsData> path_final;
    std::vector<OriginalInsData> path_final_last;

    struct SysTime
    {
        double now;
        double init;
        double planningStop;
        double taskStart;
        double vehicleStop;
        double vehicleRun;
        double idPass;
        double msgPerception;
        double msgNavigation;
        double msgLidar;
        double msgCamera;
    } sysTime;

private:
    bool JudgeTaskPlanMsgChanged(robot::task_plan_msg tLast, robot::task_plan_msg tNow);
    bool JudgeShiftParkingPoint(robot::task_plan_msg tLast, robot::task_plan_msg tNow);
    void calcuGlobalPath(robot::task_plan_msg task_plan_t);
    std::vector<XYZ_COOR_S> LoadPathFile(string tPath);
    void GenerateHookPath(robot::hook_position hook_pos);
    void GenerateParkPath(robot::palletpos park_pos, int withdraw);
    void GenerateLoadPath(robot::task_plan_msg task_plan_t);
    void UpdateStopPoint(robot::task_plan_msg task_plan_t);
    double Distance2PerceptionObjs(double x, double y);
    double Distance2PerceptionObjs(GeoLine line);

    float CalcuParkPointTangentDistance(float tCurX,
                                        float tCurY,
                                        float tTargetX,
                                        float tTargetY,
                                        float tTargetAngle);

    float LimitSpeedByDistanceToStop(float tCurX, float tCurY, float tCurAngle,
                                     float tTargetX, float tTargetY, float tTargetAngle);

    void UpdatePathInfo();

    std::vector<OriginalInsData> LatticePlan(std::vector<float> x,
                                             std::vector<float> y,
                                             std::vector<float> heading);

    double GetLineDirection(double xsecond, double ysecond,
                            double xfirst, double yfirst);


    int FindNearestPoint2VehicleID(double x, double y, vector<OriginalInsData> lpath);

    bool IsApproach(const double num1, const double num2, const double factor);

    std::vector<sCellMsg> Obj_Projecte_Map(robot::perception object,
                                           std::vector<sCellMsg> ldobjs_global);

    LidarPos local2global2(double ox, double oy, double oheading, double lx, double ly, double lheading);
    double PointDirectionToMe(Point point, Point me);
    float distan2Line(GeoPoint point, GeoLine line);
    void calcuPath(std::vector<XYZ_COOR_S> &path);
    int findNearestIndexOnPath(std::vector<XYZ_COOR_S> &path, double x, double y);
    robot::can_msg mVehicleData;

private:
    float mDesireSpeed;
    bool mRcvGpsData;
    int mKeypoint = 0;
    std::vector<XYZ_COOR_S> mDrivingPath;
    int path_id = 0;
    bool path_all_collision = false;
    float remain_distance_ = 100;
    int mStopIndex = 0;
    // class
    PubAlgor pubalgor;
    AdaptiveHook adap_hook_c;
    // message - receive
    robot::task_plan_msg mTaskPlanData;

    int command_state = 2;

    robot::navigation_msg mNavData;
    robot::perception mPerception;
    robot::perception mPerceptionFrontScan;
    robot::perception mPerceptionBackScan;
    robot::hook_position mHookPos;
    robot::palletpos mParkPos;
    robot::palletpos mLoadPos;

    // message - send
    robot::path_plan_msg mReferPath;
    robot::path_plan_msg mPlanPath;
    robot::path_plan_status mPathPlanStatus;
    robot::sound_light_msg mSoundLightData;

    // path plan
    TransData transData;
    ReferenceLineProvide referenceLineProvider;
    LatticePlanner latticeplanner;

    std::vector<sCellMsg> lidarobjs_global_;

    int mLaneChangeSwitch = 0;
    int mLaneChangeStatus = 0; // 0-done 1-running 2-l enable 3-r enable
    int mLaneId = 0;           // 0-origin lane 1-virtual lane
    std::vector<XYZ_COOR_S> mOriginPath;
    std::vector<XYZ_COOR_S> mVirtualPath;
    void TrajectoryMove(double offset);
    void LaneChange(int quest);
    int mVirtualKeypoint;
    
     //test
    std::vector<std::vector<TrajectoryPoint>> lattice_trajs_;
    std::shared_ptr<MyLatticePlanner> my_lattice_planner_;
    CollisionCheckWithBBoxSPtr collision_ptr_;
    State state_;
    std::vector<ObstaclePtr> obstacles_;
    std::vector<AirCraftParkingPort>  aircraft_parking_ports_; //飞机位信息
    int go_task_id_ = 0;
    int back_task_id_ = 0;
};

#endif

