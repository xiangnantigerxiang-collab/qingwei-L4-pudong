#include "hdmap/trajectory_processor.h"
#include "hdmap/coordinate.h"
#include "cubic_spline.h"
#include <algorithm>
#include <cmath>

namespace hdmap
{
namespace
{
STATUS_S ValidateOptions(const PROCESS_OPTIONS_S& tOptions)
{
    if (!std::isfinite(tOptions.anchor_interval) || tOptions.anchor_interval < 1.0 ||
        !std::isfinite(tOptions.sample_interval) || tOptions.sample_interval <= 0.0)
    {
        return STATUS_S(INVALID_ARGUMENT, "拟合节点间距必须 >= 1 米，输出间距必须为有限正数");
    }
    if (tOptions.direction != DIRECTION_AUTO && tOptions.direction != DIRECTION_FORWARD &&
        tOptions.direction != DIRECTION_REVERSE)
    {
        return STATUS_S(INVALID_ARGUMENT, "未知的行驶方向");
    }
    double heading = 0.0;
    return Coordinate::ConvertHeading(0.0, tOptions.input_heading_type, heading);
}

STATUS_S ValidatePoints(const MapPointList& tPoints)
{
    if (tPoints.size() < 2) return STATUS_S(INVALID_DATA, "轨迹至少需要两个有效点");
    for (std::size_t i = 0; i < tPoints.size(); ++i)
    {
        if (!std::isfinite(tPoints[i].x_axis) || !std::isfinite(tPoints[i].y_axis) ||
            !std::isfinite(tPoints[i].heading))
        {
            return STATUS_S(INVALID_DATA, "第 " + std::to_string(i + 1) + " 个点包含非有限数值");
        }
    }
    return STATUS_S();
}

double Distance(const MAP_POINT_S& tFirst, const MAP_POINT_S& tSecond)
{
    return std::hypot(tFirst.x_axis - tSecond.x_axis, tFirst.y_axis - tSecond.y_axis);
}
}

STATUS_S TrajectoryProcessor::NormalizeHeadings(const MapPointList& tInput,
                                               HEADING_TYPE_E tType,
                                               MapPointList& tOutput) const
{
    STATUS_S status = ValidatePoints(tInput);
    if (!status.IsOk()) return status;
    MapPointList points;
    points.reserve(tInput.size());
    for (std::size_t i = 0; i < tInput.size(); ++i)
    {
        MAP_POINT_S point(tInput[i].x_axis, tInput[i].y_axis);
        status = Coordinate::ConvertHeading(tInput[i].heading, tType, point.heading);
        if (!status.IsOk()) return status;
        points.push_back(point);
    }
    tOutput.swap(points);
    return STATUS_S();
}

STATUS_S TrajectoryProcessor::ResolveDirection(const MapPointList& tInput,
                                              DIRECTION_E tRequested,
                                              DIRECTION_E& tDirection) const
{
    STATUS_S status = ValidatePoints(tInput);
    if (!status.IsOk()) return status;
    if (tRequested != DIRECTION_AUTO && tRequested != DIRECTION_FORWARD &&
        tRequested != DIRECTION_REVERSE) return STATUS_S(INVALID_ARGUMENT, "未知的行驶方向");
    double forward_length = 0.0, reverse_length = 0.0, total_length = 0.0;
    std::size_t previous = 0;
    for (std::size_t i = 1; i < tInput.size(); ++i)
    {
        double dx = tInput[i].x_axis - tInput[previous].x_axis;
        double dy = tInput[i].y_axis - tInput[previous].y_axis;
        double distance = std::hypot(dx, dy);
        if (!std::isfinite(distance)) return STATUS_S(INVALID_DATA, "轨迹点距离溢出");
        // 累积到厘米级位移再判断，既过滤静止抖动，也支持毫米级的密集输入。
        if (distance < 0.01) continue;
        double yaw = Coordinate::HeadingToYaw(tInput[previous].heading);
        double alignment = dx / distance * std::cos(yaw) + dy / distance * std::sin(yaw);
        total_length += distance;
        if (alignment > 0.5) forward_length += distance;
        if (alignment < -0.5) reverse_length += distance;
        previous = i;
    }
    if (!std::isfinite(total_length) || total_length <= 0.0)
    {
        return STATUS_S(INVALID_DATA, "轨迹没有足够位移判断行驶方向");
    }
    DIRECTION_E direction = tRequested;
    if (direction == DIRECTION_AUTO)
    {
        direction = forward_length >= reverse_length ? DIRECTION_FORWARD : DIRECTION_REVERSE;
    }
    double matched_length = direction == DIRECTION_FORWARD ? forward_length : reverse_length;
    // 至少 90% 里程的车头与位移方向一致。混合前进/倒车应先分段，不能跨换挡点拟合。
    if (matched_length < 0.9 * total_length)
    {
        return STATUS_S(INVALID_DATA, "heading 与行驶方向不一致；请核对角度制，或将混合前进/倒车轨迹分段");
    }
    tDirection = direction;
    return STATUS_S();
}

STATUS_S TrajectoryProcessor::SampleAnchorPoints(const MapPointList& tInput,
                                                double tInterval,
                                                MapPointList& tOutput) const
{
    STATUS_S status = ValidatePoints(tInput);
    if (!status.IsOk()) return status;
    if (!std::isfinite(tInterval) || tInterval < 1.0)
    {
        return STATUS_S(INVALID_ARGUMENT, "拟合节点间距必须为不小于 1 米的有限数值");
    }
    MapPointList points(1, tInput.front());
    for (std::size_t i = 1; i + 1 < tInput.size(); ++i)
    {
        double distance = Distance(points.back(), tInput[i]);
        if (!std::isfinite(distance)) return STATUS_S(INVALID_DATA, "拟合节点距离溢出");
        if (distance >= tInterval) points.push_back(tInput[i]);
    }
    // 保留原始终点；尾段不足 1 米时向前合并节点，而非追加一个过密拟合节点。
    while (points.size() > 1 && Distance(points.back(), tInput.back()) < tInterval)
    {
        points.pop_back();
    }
    double last_distance = Distance(points.back(), tInput.back());
    if (!std::isfinite(last_distance) || last_distance < tInterval)
    {
        return STATUS_S(INVALID_DATA, "轨迹过短或范围过小，无法形成满足间距的两个拟合节点");
    }
    points.push_back(tInput.back());
    tOutput.swap(points);
    return STATUS_S();
}

STATUS_S TrajectoryProcessor::FitAndResample(const MapPointList& tAnchors,
                                            DIRECTION_E tDirection,
                                            const PROCESS_OPTIONS_S& tOptions,
                                            MapPointList& tOutput,
                                            PROCESS_REPORT_S& tReport) const
{
    STATUS_S status = ValidateOptions(tOptions);
    if (!status.IsOk()) return status;
    status = ValidatePoints(tAnchors);
    if (!status.IsOk()) return status;
    for (std::size_t i = 1; i < tAnchors.size(); ++i)
    {
        if (Distance(tAnchors[i - 1], tAnchors[i]) < tOptions.anchor_interval)
        {
            return STATUS_S(INVALID_DATA, "拟合节点间距小于配置下限");
        }
    }
    detail::CubicSpline spline;
    status = spline.Fit(tAnchors, tDirection);
    if (!status.IsOk()) return status;
    double length = spline.GetLength();
    double sample_count = std::floor(length / tOptions.sample_interval);
    // 例如理论 10 米的旋转直线可能积分为 9.999999999999998，
    // 只在数值容差内对齐整步，避免 floor 错丢最后一个等距点。
    double nearest_count = std::round(length / tOptions.sample_interval);
    double tolerance = std::min(1e-8, tOptions.sample_interval * 1e-6);
    if (std::abs(nearest_count * tOptions.sample_interval - length) <= tolerance)
        sample_count = nearest_count;
    // 防止错误间距使点数溢出或意外分配超大内存；需要长图时应按道路分文件。
    if (!std::isfinite(sample_count) || sample_count > 10000000.0)
    {
        return STATUS_S(INVALID_ARGUMENT, "输出点数超过一千万，请增大采样间距或拆分轨迹");
    }
    if (sample_count < 1.0 && !tOptions.include_endpoint)
    {
        return STATUS_S(INVALID_DATA, "轨迹弧长小于输出间距，无法输出两个等距点");
    }
    std::size_t count = static_cast<std::size_t>(sample_count);
    MapPointList points;
    points.reserve(count + 2);
    for (std::size_t i = 0; i <= count; ++i)
    {
        MAP_POINT_S point;
        double target = static_cast<double>(i) * tOptions.sample_interval;
        status = spline.EvaluateByLength(std::min(target, length), point);
        if (!status.IsOk()) return status;
        point.dist_origin = target;
        if (!points.empty()) point.p2pDistance = Distance(points.back(), point);
        points.push_back(point);
    }
    if (tOptions.include_endpoint && length - points.back().dist_origin > 1e-8)
    {
        MAP_POINT_S point;
        status = spline.EvaluateByLength(length, point);
        if (!status.IsOk()) return status;
        point.p2pDistance = Distance(points.back(), point);
        points.push_back(point);
    }
    PROCESS_REPORT_S report;
    report.input_count = tAnchors.size();
    report.anchor_count = tAnchors.size();
    report.output_count = points.size();
    report.direction = tDirection;
    report.spline_length = length;
    report.remaining_length = std::max(0.0, length - points.back().dist_origin);
    tOutput.swap(points);
    tReport = report;
    return STATUS_S();
}

STATUS_S TrajectoryProcessor::Process(const MapPointList& tInput,
                                     const PROCESS_OPTIONS_S& tOptions,
                                     MapPointList& tOutput,
                                     PROCESS_REPORT_S& tReport) const
{
    STATUS_S status = ValidateOptions(tOptions);
    if (!status.IsOk()) return status;
    MapPointList normalized, anchors;
    status = NormalizeHeadings(tInput, tOptions.input_heading_type, normalized);
    if (!status.IsOk()) return status;
    DIRECTION_E direction = DIRECTION_AUTO;
    status = ResolveDirection(normalized, tOptions.direction, direction);
    if (!status.IsOk()) return status;
    status = SampleAnchorPoints(normalized, tOptions.anchor_interval, anchors);
    if (!status.IsOk()) return status;
    PROCESS_REPORT_S report;
    status = FitAndResample(anchors, direction, tOptions, tOutput, report);
    if (!status.IsOk()) return status;
    report.input_count = normalized.size();
    tReport = report;
    return STATUS_S();
}
}
