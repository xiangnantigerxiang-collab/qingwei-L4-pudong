#ifndef ULTRA_COMMAND_COMPLY_H
#define ULTRA_COMMAND_COMPLY_H

// ultra_command 业务逻辑(零 ROS 依赖, 可独立编译单测):
//   1. SetTaskPlanPaths: 按 /task_plan_msg.pathList 精确匹配任务路径名, 切换监控模式
//        pudong_air/312_316_01 / pudong_air/312_cargo_01 -> LEFT  (left1 + left2)
//        pudong_air/312_charge_01                        -> RIGHT (right)
//   2. SetObstacles: 缓存最新一帧 /perception 障碍物中心点(地图系)
//   3. JudgeSafeStatus: 有障碍物落入监控矩形返回 1, 否则 0
//   4. LoadZones: 从 path_dir/pudong_air/{left1,left2,right}.csv 加载监控矩形
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
    // 监控模式: 无 / 左区(312_316_01,312_cargo_01) / 右区(312_charge_01)
    enum ZoneMode
    {
        ZONE_MODE_NONE = 0,
        ZONE_MODE_LEFT = 1,
        ZONE_MODE_RIGHT = 2,
    };

    UltraCommandComply();

    // 当前任务块 pathList(如 ["pudong_air/312_charge_01"]) -> 模式切换
    void SetTaskPlanPaths(const std::vector<std::string> &path_list);

    // 最新一帧感知障碍物; t_now_sec = 本帧接收时刻(秒)
    void SetObstacles(const std::vector<UltraObstacle> &objs, double t_now_sec);

    // 加载监控矩形; path_dir 为 pnc 全局参数 path_dir 的值(自动补尾 '/')
    // 已按当前目录加载成功时直接返回, 支持主循环每周期调用/失败重试
    bool LoadZones(const std::string &path_dir);

    int GetMode() const { return mMode; }
    bool ZonesReady() const { return mZonesReady; }

    // 1 = 有障碍物落入当前监控矩形(按规格写 /ultra/status/safe = 1)
    // 0 = 矩形内无障碍 / 感知帧超时 / 矩形未加载
    // 模式为 NONE 时恒 0(节点侧此时不写参数, 此处仅作查询值)
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
    int mLastSafeStatus;                    // 状态变化打印用
};

#endif
