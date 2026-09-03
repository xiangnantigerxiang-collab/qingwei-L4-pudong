#include "path_plan_comply.h"
#include "robot/can_msg.h"
#include <cmath>
#include <fstream>
#include <visualization_msgs/MarkerArray.h>
#include "lattice/math_utils.h"
#include "lattice/gis_utils.h"
#include <yaml-cpp/yaml.h>

using Dot = std::pair<double, double>;
const double EPS = 1e-9;

std::vector<Dot> getRectangleVertices(const std::vector<Dot> &centerLine, double width)
{
    if (centerLine.size() != 2)
    {
        throw invalid_argument("centerLine 必须包含两个点");
    }

    if (width <= 0)
    {
        throw invalid_argument("矩形宽度必须大于 0");
    }

    double x1 = centerLine[0].first;
    double y1 = centerLine[0].second;
    double x2 = centerLine[1].first;
    double y2 = centerLine[1].second;

    double vx = x2 - x1;
    double vy = y2 - y1;

    double len = sqrt(vx * vx + vy * vy);

    if (len == 0)
    {
        throw invalid_argument("输入的两个点不能重合");
    }

    double halfWidth = width / 2.0;

    // 垂直于中心线的单位向量
    double nx = -vy / len;
    double ny = vx / len;

    return {
        {x1 + nx * halfWidth, y1 + ny * halfWidth},
        {x2 + nx * halfWidth, y2 + ny * halfWidth},
        {x2 - nx * halfWidth, y2 - ny * halfWidth},
        {x1 - nx * halfWidth, y1 - ny * halfWidth}};
}

double cross(const Dot &A, const Dot &B, const Dot &P)
{
    double ABx = B.first - A.first;
    double ABy = B.second - A.second;
    double APx = P.first - A.first;
    double APy = P.second - A.second;

    return ABx * APy - ABy * APx;
}

// 判断点 P 是否在线段 AB 上
bool onSegment(const Dot &A, const Dot &B, const Dot &P)
{
    if (fabs(cross(A, B, P)) > EPS)
    {
        return false;
    }

    return P.first >= min(A.first, B.first) - EPS &&
           P.first <= max(A.first, B.first) + EPS &&
           P.second >= min(A.second, B.second) - EPS &&
           P.second <= max(A.second, B.second) + EPS;
}

// 将四个顶点按顺时针或逆时针排序
void sortRectangleVertices(std::vector<Dot> &rect)
{
    double cx = 0.0;
    double cy = 0.0;

    for (const auto &p : rect)
    {
        cx += p.first;
        cy += p.second;
    }

    cx /= rect.size();
    cy /= rect.size();

    sort(rect.begin(), rect.end(), [cx, cy](const Dot &a, const Dot &b)
         {
        double angleA = atan2(a.second - cy, a.first - cx);
        double angleB = atan2(b.second - cy, b.first - cx);
        return angleA < angleB; });
}

// 判断点是否在矩形内
// 返回值：0 外部，1 内部，2 边上
int pointInRectangle(std::vector<Dot> rect, const Dot &p)
{
    if (rect.size() != 4)
    {
        throw invalid_argument("rect 必须包含四个顶点");
    }

    sortRectangleVertices(rect);

    bool hasPositive = false;
    bool hasNegative = false;

    for (int i = 0; i < 4; ++i)
    {
        const Dot &A = rect[i];
        const Dot &B = rect[(i + 1) % 4];

        if (onSegment(A, B, p))
        {
            return 2;
        }

        double c = cross(A, B, p);

        if (c > EPS)
        {
            hasPositive = true;
        }
        else if (c < -EPS)
        {
            hasNegative = true;
        }

        if (hasPositive && hasNegative)
        {
            return 0;
        }
    }

    return 1;
}


// 下列职责分片仍在同一翻译单元内编译，避免改变既有静态状态和链接行为。
// 修改功能时按职责进入对应文件，不要把这些 .inc 单独加入 CMake。
// 任务接收、CSV 路径装载、作业路径生成和机场停车位。
#include "path_plan_task.inc"
// CAN、定位、感知输入及障碍物距离计算。
#include "path_plan_perception.inc"
// 10 Hz 规划主流程、参考路径截取及前向碰撞判断。
#include "path_plan_reference.inc"
// 最终路径、安全覆盖、停车限速、路径索引推进和可视化。
#include "path_plan_output.inc"
// 当前休眠的 lattice 入口及坐标转换辅助函数。
#include "path_plan_experimental.inc"
