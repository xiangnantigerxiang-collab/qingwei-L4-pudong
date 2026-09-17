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
#include "lattice_plan/lattice_planner.h"
#include "common/pubalgor/pubalgor.h"
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
#include "common/spline/Spline.h"
#include "trans/trans_data.h"
#include <ros/ros.h>
#include "lattice/my_lattice_planner.h"
#include "safety/perception_safety.h"

using namespace planning;
using namespace adaptive;
namespace bg = boost::geometry;
using GeoPoint = bg::model::d2::point_xy<double>;
using GeoLine = bg::model::segment<GeoPoint>;
using GeoBox = bg::model::box<GeoPoint>;

struct AirCraftParkingPort {
    int id;            //飞机位id
    uint parking = 0;  //0 非入位中 1 入位中
    Point go_stop_point;
    Point back_stop_point;
    int go_stop_index = 0;
    int back_stop_index = 0;
};

class PathPlanComply {
public:
    // 生命周期与配置初始化。
    PathPlanComply();
    ~PathPlanComply();
    void InitParameter();
    void ResetParameter();
    void loadAirCraftPorts();
    // 输入快照：由 path_plan_node 的 ROS 回调写入。
    void SetTaskPlanData(robot::task_plan_msg task_plan_t);
    void SetCanData(robot::can_msg can_msg_t);
    void SetNavigationData(robot::navigation_msg navigation_msg_t);
    void SetPerceptionData(const robot::perception &perception_t);
    void SetPlanningPerceptionData(const robot::perception &perception_t);
    bool ConfigurePerceptionSafety();
    void SetGantryState(bool active, bool gantry_open);
    void SetFrontScanData(jsk_recognition_msgs::BoundingBoxArray msg);
    void SetBackScanData(jsk_recognition_msgs::BoundingBoxArray msg);
    void SetPalletCoorData(robot::hook_position hook_pos_t);
    void SetPerceptionData2(robot::perception perception_t);
    void SetLoadPosition(robot::palletpos load_pos_t);
    void SetButtonData(robot::rs232 button_data_t);
    void SetCommandData(robot::v2nCommandFeedback command_t);
    void SetCommandState(int in_command) {
        command_state = in_command;
    }
    void resetTask();

    // 飞机位动态占用信息。
    void updateAirPortStopIndex(std::vector<XYZ_COOR_S> &global_path, bool is_go_path);
    void updateAirPortInfo(robot::AirCraftParkingPort airport);
    AirCraftParkingPort findNextAirCraftParkingPort();

    // 10 Hz 周期规划与发布。
    void PathPlanProcess();
    void PublishPathPlanStatus(ros::Publisher &tPub);
    void PublishReferPath(ros::Publisher &tPub);
    void PublishPlanPath(ros::Publisher &tPub1, ros::Publisher &tPub2);

    void handleDrivingPath();
    double getLaneLimitSpeedTest(double cur_lane_speed, double next_lane_speed);
    bool checkAroundObstacle();
    bool checkTJunctionObstacle();
    bool checkIsInWaiting();
    ObstaclePtr findObstacle(std::vector<ObstaclePtr> &obstacles, double x, double y);

    std::vector<GeoPolygon> obstacle_obbs_;
    GeoPolygon vehicle_obb_;

    int sumVec(std::vector<bool> &vec) {
        int sum = 0;
        for(int i = 0; i < vec.size(); i++) {
            sum += int(vec[i]);
        }
        return sum;
    }

    int AccSwitch = 0;
    int PalletType = 0;         //0-static pallet pos 1-reflect pallet pos
    int LaneChangeRequset = 0;  // 0-none 1-left 2-right
    // 挂钩销/鞍座位置判定线(0x285 码值, 小=高), 由节点主循环热读 canbus
    // 发布的 rosparam 注入; 初值=config.cfg 默认值(参数缺失时保持)
    int HookPosMin = 185;    // 销<min=钩完全升起
    int HookPosMax = 240;    // 销>max=销退到底
    int PalletPosMin = 130;  // 鞍座<min=托盘升顶
    int PalletPosMax = 240;  // 鞍座>max=托盘落底
    int workMode = 0;
    int InitSafetyCheck = 0;
    int emergencyStop = 0;

    std::vector<OriginalInsData> path_final;
    std::vector<OriginalInsData> path_final_last;

    struct SysTime {
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
    // 单点区域限速：初始化时读文件，发布时只遍历缓存。
    void LoadPointSpeedLimits();
    void ApplyPointSpeedLimit();

    // 全局路径构建阶段；实现见 task/global_path.inc。
    void LoadTaskPaths(const robot::task_plan_msg &task_plan_t);
    void FindTaskStartPoint();
    void ConnectVehicleToPath();
    void FindTaskStopPoint(const robot::task_plan_msg &task_plan_t);
    void ResetTaskHistory();

    // 任务停车规则；实现见 task/stop_speed.inc。
    void UpdateActionTaskStatus();
    float LimitParkTaskSpeed(float tCurX, float tCurY, float tCurAngle,
                             float tTargetX, float tTargetY);
    float LimitHookTaskSpeed();
    float LimitDrivingTaskSpeed();

    // 参考路径生成与碰撞检查；计算阶段只更新 mReferPath，不发布消息。
    void BuildReferencePathPoints(std::vector<float> &x, std::vector<float> &y,
                                  std::vector<float> &heading, int &LaneChangeSwitch);
    void UpdateReverseReferencePath(const std::vector<float> &x, const std::vector<float> &y);
    void UpdateForwardReferencePath(const std::vector<float> &x, const std::vector<float> &y,
                                    const std::vector<float> &heading, int LaneChangeSwitch);
    void ReusePreviousTrajectory();
    void CheckForwardReferenceSafety();
    void CheckTrackedReferenceSafety();
    void BuildPerceptionSafetyPath(double tLookahead);
    std::vector<robot::object> CollectReferenceRiskObjects();
    bool UpdateReferenceRiskHistory(std::vector<robot::object> &risk_objs);
    std::vector<robot::object> FilterForwardRiskObjects(
        const std::vector<robot::object> &risk_objs, double &dh);
    void ApplyReferenceCollisionSpeed(const std::vector<robot::object> &risk_objs_filter, double dh);

    // 最终路径覆盖顺序集中在 PublishPlanPath；实现见 path_plan_output.inc。
    void PublishFinalPlanPath(ros::Publisher &tPub);
    void PublishStoppedPath(ros::Publisher &tPub1);
    void UpdateStartupSafety(bool is_around_unsafe, bool is_T_unsafe);
    void CopyReferencePath();
    void SmoothPlanSpeed();
    void ApplyEmergencyAndNetworkStop();
    void UpdateSoundLightCommand(int &HornCmd, int &LightCmd);
    void ApplyWaitingAreaStop(bool is_T_unsafe, int &LightCmd);

    // 任务变化判断与路径构建。
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
    // 下列成员顺序关系到对象布局和初始化语义，整理时不重排。
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

    // 跨帧去抖/迟滞状态(原为各函数内 static, 转成员以便任务切换时统一复位;
    // 两处同名 safety_check_counter 因合并类作用域被迫消歧为 _percep/_refer)
    int safety_check_counter_percep = 0;                       // SetPerceptionData 安全过滤去抖
    int safety_check_counter_refer = 0;                        // PublishReferPath safety 去抖
    std::vector<std::vector<robot::object>> history_risk_vec;  // 4 帧风险障碍环形历史
    int HornFlag = 1;                                          // PublishPlanPath 喇叭去抖
    std::vector<bool> unsafe_vec;                              // checkTJunctionObstacle 4 帧去抖
    std::vector<bool> history_unsafe;                          // checkAroundObstacle 30 帧起步观察
    int backdist_flag = 0;                                     // ADAPTIVEHOOK 倒车距离迟滞

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
    int mLaneChangeStatus = 0;  // 0-done 1-running 2-l enable 3-r enable
    int mLaneId = 0;            // 0-origin lane 1-virtual lane
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
    std::vector<AirCraftParkingPort> aircraft_parking_ports_;  //飞机位信息
    int go_task_id_ = 0;
    int back_task_id_ = 0;
    bool mGantryStop = false;  // 未收到有效闸机关闭状态时，不叠加安全标志。
    planning_perception::PerceptionSafety mPerceptionSafety;
    std::vector<planning_perception::PATH_POINT_S> mPerceptionSafetyPath;
    planning_perception::RESULT_S mPerceptionSafetyResult;
    double mPerceptionSafetyLogTime = -1;
    int mPerceptionSafetyLogReason = -1;
    struct POINT_SPEED_LIMIT_S {
        double x, y, speed_limit;
    };
    std::vector<POINT_SPEED_LIMIT_S> mPointSpeedLimits;
    std::vector<std::uint32_t> mPerceptionOutsideMarks = std::vector<std::uint32_t>(32768, 0);
    std::uint32_t mPerceptionOutsideGeneration = 0;
};

#endif
