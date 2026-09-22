#ifndef ULTRA_COMMAND_COMPLY_H
#define ULTRA_COMMAND_COMPLY_H

// ultra_command 业务逻辑(零 ROS 依赖, 可独立编译单测):
//   1. SetTaskPlanPaths: 按 /task_plan_msg.pathList 精确匹配任务路径名, 切换监控模式
//        pudong_air/312_316 / pudong_air/312_cargo -> LEFT  (left1 + left2)
//        pudong_air/312_charge                     -> RIGHT (right)
//        上述三者的 _01_01 后缀变体 -> 初始化监控 INIT_LEFT / INIT_RIGHT:
//        仅起步前(车速<0.5m/s)监测对应矩形, 首次车速>0.5m/s 后无条件 safe=0
//   2. SetObstacles: 缓存最新一帧 /perception 障碍物中心点(地图系)
//   3. SetVehicleSpeed: 车速输入(/navigation_msg.gpsSpeed), 初始化监控的起步判据
//   4. JudgeSafeStatus: 有障碍物或监控不可用返回 1, 确认无障碍返回 0
//      (初始化模式起步后恒 0)
//   5. LoadZones: 从 path_dir/pudong_air/{left1,left2,right}.csv 加载监控矩形
//      (顶点射线法判内, 与 perception_convert IsPointInExclusion 同款算法)
//
// 坐标系: 矩形 csv 与任务路径 csv 同目录同坐标系(定位 xAxis/yAxis 地图系),
// /perception 障碍物 x/y 已由 perception_convert 转到同一地图系, 可直接比较。
#include <string>
#include <utility>
#include <vector>

// 一帧感知障碍物中心点(地图系)
struct UltraObstacle
{
    double x;
    double y;
};

class UltraCommandComply
{
public:
    // 监控模式: 无 / 左区 / 右区 / 初始化监控(仅起步前监测, 起步后恒 0)
    enum ZoneMode
    {
        ZONE_MODE_NONE = 0,
        ZONE_MODE_LEFT = 1,
        ZONE_MODE_RIGHT = 2,
        ZONE_MODE_INIT_LEFT = 3,   // 312_316_01_01 / 312_cargo_01_01
        ZONE_MODE_INIT_RIGHT = 4,  // 312_charge_01_01
    };

    UltraCommandComply();

    // 当前任务块 pathList(如 ["pudong_air/312_charge"]) + task_id -> 模式切换
    void SetTaskPlanPaths(const std::vector<std::string> &path_list,
                          long long task_id = 0);

    // 最新一帧感知障碍物; t_now_sec = 本帧接收时刻(秒)
    void SetObstacles(const std::vector<UltraObstacle> &objs, double t_now_sec);

    // 最新车速 m/s(节点侧来自 /navigation_msg.gpsSpeed)
    void SetVehicleSpeed(double speed_mps);

    // 加载监控矩形; path_dir 为 pnc 全局参数 path_dir 的值(自动补尾 '/')
    // 已按当前目录加载成功时直接返回, 支持主循环每周期调用/失败重试
    bool LoadZones(const std::string &path_dir);

    int GetMode() const { return mMode; }
    bool ZonesReady() const { return mZonesReady; }

    // 1 = 有障碍物落入当前监控矩形, 或感知/矩形/初始化车速不可用(故障安全)
    // 0 = 普通模式连续三帧确认无障碍 / 初始化模式当前帧确认无障碍;
    //     模式为 NONE 或初始化模式起步后(首次车速>0.5m/s)恒 0
    int JudgeSafeStatus(double t_now_sec);

private:
    typedef std::pair<double, double> ZonePoint;  // (x, y)

    // 射线法判点在多边形内(与 perception_convert IsPointInExclusion 同款)
    static bool IsPointInPolygon(double x, double y,
                                 const std::vector<ZonePoint> &poly);

    // 解析矩形 csv: 每行 "x,y,..." 取前两列(第三列恒 1, stod 遇逗号自然截断)
    static std::vector<ZonePoint> LoadPolygonCsv(const std::string &file_path);

    int mMode;
    bool mZonesReady;
    std::string mLoadedPathDir;
    std::vector<ZonePoint> mLeft1;
    std::vector<ZonePoint> mLeft2;
    std::vector<ZonePoint> mRight;

    std::vector<UltraObstacle> mObstacles;  // 最新一帧障碍物(地图系)
    double mLastObstacleTime;               // 最近一帧 /perception 接收时刻(秒)
    bool mRcvObstacle;                      // 是否收到过感知帧
    unsigned long long mObstacleFrameSeq;   // 感知帧序号(防主循环重复计帧)
    unsigned long long mLastJudgeFrameSeq;  // 上次判定消费的感知帧序号
    int mClearFrameCount;                   // 连续无障碍帧数
    int mSafeStatus;                        // 当前滤波后状态
    int mLastLoggedSafeStatus;              // 状态变化打印用
    double mVehicleSpeed;                   // 最新车速 m/s
    bool mRcvVehicleSpeed;                  // 是否收到过有效车速
    bool mStarted;                          // 初始化模式: 已首次超 0.5m/s(单向闩锁)
    std::string mActiveTaskPath;             // 当前命中的完整任务路径
    long long mActiveTaskId;                 // 当前任务 ID(同路径新任务识别用)
};

#endif
