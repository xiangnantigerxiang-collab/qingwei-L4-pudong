#include "ultra_command_comply.h"

#include <cmath>
#include <cstdio>
#include <fstream>

// /perception 在空场景不发布(perception_convert 空帧早退, 不发空障碍列表),
// 无新帧超过该时长按"无障碍"处理, 防止 safe=1 永久粘滞; 时长对齐 canbus 指令老化 1.0s
static const double kObstacleStaleSec = 1.0;

UltraCommandComply::UltraCommandComply()
    : mMode(ZONE_MODE_NONE), mZonesReady(false), mLastObstacleTime(0.0),
      mRcvObstacle(false), mLastSafeStatus(-1) {}

void UltraCommandComply::SetTaskPlanPaths(
    const std::vector<std::string> &path_list)
{
    int new_mode = ZONE_MODE_NONE;
    for (size_t i = 0; i < path_list.size(); i++)
    {
        // 精确匹配三个目标任务路径; 同族 02/03 变体与其它路径不监控
        if (path_list[i] == "pudong_air/312_316_01" ||
            path_list[i] == "pudong_air/312_cargo_01")
        {
            new_mode = ZONE_MODE_LEFT;
            break;
        }
        if (path_list[i] == "pudong_air/312_charge_01")
        {
            new_mode = ZONE_MODE_RIGHT;
            break;
        }
    }
    if (new_mode != mMode)
    {
        printf("====ultra_command==== 监控模式切换: %d -> %d (0=无 1=left1+left2 2=right)\n",
               mMode, new_mode);
        mMode = new_mode;
        mLastSafeStatus = -1;  // 模式切换后首次判定重新打印
    }
}

void UltraCommandComply::SetObstacles(const std::vector<UltraObstacle> &objs,
                                      double t_now_sec)
{
    mObstacles = objs;
    mRcvObstacle = true;
    mLastObstacleTime = t_now_sec;
}

bool UltraCommandComply::LoadZones(const std::string &path_dir)
{
    // 已按当前目录加载成功: 直接返回, 支持主循环每周期调用
    if (mZonesReady && path_dir == mLoadedPathDir)
        return true;

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
                   "right:%zu 顶点, 加载成功前按无障碍处理)\n",
                   dir.c_str(), mLeft1.size(), mLeft2.size(), mRight.size());
    }
    mZonesReady = ready;
    mLoadedPathDir = path_dir;
    return ready;
}

int UltraCommandComply::JudgeSafeStatus(double t_now_sec)
{
    if (mMode == ZONE_MODE_NONE)
        return 0;  // 未激活监控: 节点侧不写参数

    int status = 0;
    const char *reason = "监控区内无障碍物";

    if (!mZonesReady)
    {
        status = 0;
        reason = "监控矩形未加载, 按无障碍处理";
    }
    else if (!mRcvObstacle || t_now_sec - mLastObstacleTime > kObstacleStaleSec)
    {
        // 空场景 /perception 静默, 超时按无障碍(见文件头 kObstacleStaleSec 注释)
        status = 0;
        reason = "感知帧超时/未收到, 按无障碍处理";
    }
    else
    {
        for (size_t i = 0; i < mObstacles.size(); i++)
        {
            if (mMode == ZONE_MODE_LEFT)
            {
                if (IsPointInPolygon(mObstacles[i].x, mObstacles[i].y, mLeft1) ||
                    IsPointInPolygon(mObstacles[i].x, mObstacles[i].y, mLeft2))
                {
                    status = 1;
                    reason = "障碍物落入 left1/left2 监控区";
                    break;
                }
            }
            else if (mMode == ZONE_MODE_RIGHT)
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

    if (status != mLastSafeStatus)
    {
        printf("====ultra_command==== /ultra/status/safe: %d -> %d (%s)\n",
               mLastSafeStatus, status, reason);
        mLastSafeStatus = status;
    }
    return status;
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
