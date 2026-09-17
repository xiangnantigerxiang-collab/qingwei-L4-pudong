#include "ultra_command_comply.h"

#include <cmath>
#include <cstdio>
#include <fstream>

// /perception 无新帧超过该时长视为感知链不可用, 按有障碍处理
static const double kObstacleStaleSec = 1.0;
// 障碍物即时置 1; 清 0 需连续新感知帧确认, 避免边界噪声反复翻转
static const int kClearConfirmFrames = 3;
// 初始化监控的起步判据: 首次车速超过该值后初始化监控退出(单向, safe 恒 0)
static const double kStartSpeedMps = 0.5;

UltraCommandComply::UltraCommandComply()
    : mMode(ZONE_MODE_NONE), mZonesReady(false), mLastObstacleTime(0.0),
      mRcvObstacle(false), mObstacleFrameSeq(0), mLastJudgeFrameSeq(0),
      mClearFrameCount(0), mSafeStatus(0), mLastLoggedSafeStatus(-1),
      mVehicleSpeed(0.0), mRcvVehicleSpeed(false), mStarted(false),
      mActiveTaskPath(), mActiveTaskId(0) {}

void UltraCommandComply::SetTaskPlanPaths(
    const std::vector<std::string> &path_list, long long task_id)
{
    int new_mode = ZONE_MODE_NONE;
    std::string active_path;
    for (size_t i = 0; i < path_list.size(); i++)
    {
        // 精确匹配目标任务路径; 同族 02/03 变体与其它路径不监控
        if (path_list[i] == "pudong_air/312_316_01" ||
            path_list[i] == "pudong_air/312_cargo_01")
        {
            new_mode = ZONE_MODE_LEFT;
            active_path = path_list[i];
            break;
        }
        if (path_list[i] == "pudong_air/312_charge_01")
        {
            new_mode = ZONE_MODE_RIGHT;
            active_path = path_list[i];
            break;
        }
        // _01_01 后缀变体 -> 初始化监控(仅起步前监测)
        if (path_list[i] == "pudong_air/312_316_01_01" ||
            path_list[i] == "pudong_air/312_cargo_01_01")
        {
            new_mode = ZONE_MODE_INIT_LEFT;
            active_path = path_list[i];
            break;
        }
        if (path_list[i] == "pudong_air/312_charge_01_01")
        {
            new_mode = ZONE_MODE_INIT_RIGHT;
            active_path = path_list[i];
            break;
        }
    }

    bool init_mode = (new_mode == ZONE_MODE_INIT_LEFT ||
                      new_mode == ZONE_MODE_INIT_RIGHT);
    bool task_changed = (new_mode != mMode);
    if (init_mode &&
        (active_path != mActiveTaskPath || task_id != mActiveTaskId))
        task_changed = true;

    if (task_changed)
    {
        printf("====ultra_command==== 监控模式/任务切换: %d -> %d "
               "(task:%lld path:%s; 0=无 1=left 2=right 3=init-left 4=init-right)\n",
               mMode, new_mode, task_id, active_path.c_str());
        mMode = new_mode;
        mRcvObstacle = false;  // 切换后必须由新感知帧确认当前区域
        mClearFrameCount = 0;
        mSafeStatus = (new_mode == ZONE_MODE_NONE) ? 0 : 1;
        mLastJudgeFrameSeq = mObstacleFrameSeq;
        mLastLoggedSafeStatus = -1;  // 模式切换后首次判定重新打印
        // 回调到达顺序不固定: 进入初始化任务时同时使用已缓存的当前车速
        mStarted = init_mode && mRcvVehicleSpeed &&
                   mVehicleSpeed > kStartSpeedMps;
        if (mStarted)
            printf("====ultra_command==== 进入初始化任务时车速已超过 %.1fm/s, "
                   "初始化监控退出\n", kStartSpeedMps);
    }
    mActiveTaskPath = active_path;
    mActiveTaskId = task_id;
}

void UltraCommandComply::SetObstacles(const std::vector<UltraObstacle> &objs,
                                      double t_now_sec)
{
    mObstacles = objs;
    mRcvObstacle = true;
    mLastObstacleTime = t_now_sec;
    mObstacleFrameSeq++;
}

void UltraCommandComply::SetVehicleSpeed(double speed_mps)
{
    if (!std::isfinite(speed_mps))
        return;
    mVehicleSpeed = speed_mps;
    mRcvVehicleSpeed = true;
    // 起步闩锁: 首次超过 0.5m/s 即锁定, 此后不随降速回退;
    // 非初始化任务只缓存车速, 供随后进入初始化任务时判断当前状态
    if (!mStarted && speed_mps > kStartSpeedMps &&
        (mMode == ZONE_MODE_INIT_LEFT || mMode == ZONE_MODE_INIT_RIGHT))
    {
        mStarted = true;
        printf("====ultra_command==== 车速首次超过 %.1fm/s, 初始化监控退出\n",
               kStartSpeedMps);
    }
}

bool UltraCommandComply::LoadZones(const std::string &path_dir)
{
    // 已按当前目录加载成功: 直接返回, 支持主循环每周期调用
    if (mZonesReady && path_dir == mLoadedPathDir)
        return true;

    bool dir_changed = (path_dir != mLoadedPathDir);
    std::string dir = path_dir;
    if (!dir.empty() && dir[dir.size() - 1] != '/')
        dir += '/';

    // 三个监控矩形与任务路径同目录(pnc/path/pudong_air/), 固定文件名
    mLeft1 = LoadPolygonCsv(dir + "pudong_air/left1.csv");
    mLeft2 = LoadPolygonCsv(dir + "pudong_air/left2.csv");
    mRight = LoadPolygonCsv(dir + "pudong_air/right.csv");

    bool ready = (mLeft1.size() >= 3 && mLeft2.size() >= 3 && mRight.size() >= 3);
    // 结果变化或目录变化才打印, 避免失败期间每周期刷屏
    if (ready != mZonesReady || path_dir != mLoadedPathDir)
    {
        if (ready)
            printf("====ultra_command==== 监控矩形加载成功: %spudong_air/"
                   "{left1,left2,right}.csv (%zu/%zu/%zu 顶点)\n",
                   dir.c_str(), mLeft1.size(), mLeft2.size(), mRight.size());
        else
            printf("====ultra_command==== 监控矩形加载失败: %s (left1:%zu left2:%zu "
                   "right:%zu 顶点, 加载成功前按有障碍处理)\n",
                   dir.c_str(), mLeft1.size(), mLeft2.size(), mRight.size());
    }
    mZonesReady = ready;
    mLoadedPathDir = path_dir;
    if (dir_changed && mMode != ZONE_MODE_NONE)
    {
        mRcvObstacle = false;
        mClearFrameCount = 0;
        mSafeStatus = 1;
        mLastJudgeFrameSeq = mObstacleFrameSeq;
    }
    return ready;
}

int UltraCommandComply::JudgeSafeStatus(double t_now_sec)
{
    if (mMode == ZONE_MODE_NONE)
    {
        mClearFrameCount = 0;
        mSafeStatus = 0;
        return mSafeStatus;
    }

    bool init_mode = (mMode == ZONE_MODE_INIT_LEFT ||
                      mMode == ZONE_MODE_INIT_RIGHT);

    // 初始化监控起步后: 无条件输出 0, 不依赖感知/矩形状态
    // (规格: 起步完成后无论监控区内是否有障碍物, safe 均置 0)
    if (mStarted && init_mode)
    {
        mClearFrameCount = 0;
        mSafeStatus = 0;
        mLastJudgeFrameSeq = mObstacleFrameSeq;  // 跳过的帧不积累确认计数
        if (mLastLoggedSafeStatus != 0)
        {
            printf("====ultra_command==== /ultra/status/safe: %d -> 0 "
                   "(已起步, 初始化监控退出)\n",
                   mLastLoggedSafeStatus);
            mLastLoggedSafeStatus = 0;
        }
        return mSafeStatus;
    }

    int status = 1;
    const char *reason = "监控区内有障碍物";
    bool fresh_frame = (mObstacleFrameSeq != mLastJudgeFrameSeq);

    if (!mZonesReady)
    {
        reason = "监控矩形未加载, 按有障碍处理";
    }
    else if (init_mode && !mRcvVehicleSpeed)
    {
        reason = "车速未收到, 按有障碍处理";
    }
    else if (!mRcvObstacle || t_now_sec - mLastObstacleTime > kObstacleStaleSec)
    {
        reason = "感知帧超时/未收到, 按有障碍处理";
    }
    else
    {
        status = 0;
        reason = "监控区内无障碍物";
        for (size_t i = 0; i < mObstacles.size(); i++)
        {
            if (mMode == ZONE_MODE_LEFT || mMode == ZONE_MODE_INIT_LEFT)
            {
                if (IsPointInPolygon(mObstacles[i].x, mObstacles[i].y, mLeft1) ||
                    IsPointInPolygon(mObstacles[i].x, mObstacles[i].y, mLeft2))
                {
                    status = 1;
                    reason = "障碍物落入 left1/left2 监控区";
                    break;
                }
            }
            else if (mMode == ZONE_MODE_RIGHT || mMode == ZONE_MODE_INIT_RIGHT)
            {
                if (IsPointInPolygon(mObstacles[i].x, mObstacles[i].y, mRight))
                {
                    status = 1;
                    reason = "障碍物落入 right 监控区";
                    break;
                }
            }
        }
    }

    if (init_mode)
    {
        // 初始化阶段按当前有效感知帧直接反映 1/0, 不套用普通模式解除防抖
        mSafeStatus = status;
        mClearFrameCount = 0;
    }
    else if (status == 1)
    {
        mSafeStatus = 1;
        mClearFrameCount = 0;
    }
    else if (fresh_frame)
    {
        mClearFrameCount++;
        if (mClearFrameCount >= kClearConfirmFrames)
            mSafeStatus = 0;
        else
            reason = "等待连续三帧确认监控区无障碍物";
    }

    if (fresh_frame)
        mLastJudgeFrameSeq = mObstacleFrameSeq;

    if (mSafeStatus != mLastLoggedSafeStatus)
    {
        printf("====ultra_command==== /ultra/status/safe: %d -> %d (%s)\n",
               mLastLoggedSafeStatus, mSafeStatus, reason);
        mLastLoggedSafeStatus = mSafeStatus;
    }
    return mSafeStatus;
}

bool UltraCommandComply::IsPointInPolygon(double x, double y,
                                          const std::vector<ZonePoint> &poly)
{
    int n = (int)poly.size(), j = n - 1;
    bool inside = false;
    for (int i = 0; i < n; j = i++)
    {
        if (((poly[i].second > y) != (poly[j].second > y)) &&
            (x < (poly[j].first - poly[i].first) * (y - poly[i].second) /
                         (poly[j].second - poly[i].second) + poly[i].first))
            inside = !inside;
    }
    return inside;
}

std::vector<UltraCommandComply::ZonePoint> UltraCommandComply::LoadPolygonCsv(
    const std::string &file_path)
{
    std::vector<ZonePoint> poly;
    std::ifstream file(file_path.c_str());
    if (!file.is_open())
        return poly;

    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty())
            continue;
        size_t comma_pos = line.find(',');
        if (comma_pos == std::string::npos)
            continue;
        try
        {
            // stod 遇第二个逗号自然截断, 第三列(恒 1)不参与解析
            double x = std::stod(line.substr(0, comma_pos));
            double y = std::stod(line.substr(comma_pos + 1));
            // nan/inf 字面量 stod 不抛异常, 顶点非有限值按坏行跳过
            // (否则 NaN 使射线法相邻边静默失效, 判定漏检无诊断)
            if (!std::isfinite(x) || !std::isfinite(y))
                continue;
            poly.push_back(ZonePoint(x, y));
        }
        catch (...)
        {
            // 坏行跳过(顶点数下限由 LoadZones 把关)
        }
    }
    return poly;
}
