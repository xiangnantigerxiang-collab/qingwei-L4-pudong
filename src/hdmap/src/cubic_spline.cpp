#include "cubic_spline.h"
#include "hdmap/coordinate.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace hdmap
{
namespace detail
{
double CubicSpline::Polynomial::Value(double tU) const
{
    return ((d * tU + c) * tU + b) * tU + a;
}

double CubicSpline::Polynomial::First(double tU) const
{
    return (3.0 * d * tU + 2.0 * c) * tU + b;
}

double CubicSpline::Polynomial::Second(double tU) const
{
    return 6.0 * d * tU + 2.0 * c;
}

double CubicSpline::Segment::Speed(double tU) const
{
    return std::hypot(x.First(tU), y.First(tU));
}

bool CubicSpline::BuildAxis(const std::vector<double>& tValues,
                            const std::vector<double>& tSpans, double tStart,
                            double tEnd, std::vector<Polynomial>& tPolynomials)
{
    const std::size_t count = tValues.size();
    std::vector<double> diagonal(count), upper(count, 0.0), right(count);
    // 夹持边界使用原始车头方向转换出的行进切线；倒车时调用方已翻转切线。
    // 解三对角方程得到二阶导，使相邻段的位置、一阶和二阶导连续。
    diagonal[0] = 2.0 * tSpans[0];
    upper[0] = tSpans[0];
    right[0] = 6.0 * ((tValues[1] - tValues[0]) / tSpans[0] - tStart);
    for (std::size_t i = 1; i < count; ++i)
    {
        double lower = tSpans[i - 1];
        if (i + 1 < count)
        {
            diagonal[i] = 2.0 * (tSpans[i - 1] + tSpans[i]);
            upper[i] = tSpans[i];
            right[i] = 6.0 * ((tValues[i + 1] - tValues[i]) / tSpans[i] -
                             (tValues[i] - tValues[i - 1]) / tSpans[i - 1]);
        }
        else
        {
            diagonal[i] = 2.0 * lower;
            right[i] = 6.0 * (tEnd - (tValues[i] - tValues[i - 1]) / lower);
        }
        double factor = lower / diagonal[i - 1];
        diagonal[i] -= factor * upper[i - 1];
        right[i] -= factor * right[i - 1];
        if (!std::isfinite(diagonal[i]) || diagonal[i] <= 0.0 ||
            !std::isfinite(right[i])) return false;
    }
    std::vector<double> second(count);
    second.back() = right.back() / diagonal.back();
    for (std::size_t i = count - 1; i > 0; --i)
    {
        second[i - 1] = (right[i - 1] - upper[i - 1] * second[i]) / diagonal[i - 1];
    }
    for (std::size_t i = 0; i + 1 < count; ++i)
    {
        double span = tSpans[i];
        Polynomial polynomial;
        polynomial.a = tValues[i];
        polynomial.b = (tValues[i + 1] - tValues[i]) / span -
                       span * (2.0 * second[i] + second[i + 1]) / 6.0;
        polynomial.c = second[i] / 2.0;
        polynomial.d = (second[i + 1] - second[i]) / (6.0 * span);
        if (!std::isfinite(polynomial.b) || !std::isfinite(polynomial.c) ||
            !std::isfinite(polynomial.d)) return false;
        tPolynomials.push_back(polynomial);
    }
    return true;
}

bool CubicSpline::RefineIntegral(const Segment& tSegment, double tFrom, double tTo,
                                 double tLeft, double tMiddle, double tRight,
                                 double tWhole, double tTolerance, int tDepth,
                                 double& tLength)
{
    double middle = 0.5 * (tFrom + tTo);
    double left_middle = tSegment.Speed(0.5 * (tFrom + middle));
    double right_middle = tSegment.Speed(0.5 * (middle + tTo));
    double left = (middle - tFrom) * (tLeft + 4.0 * left_middle + tMiddle) / 6.0;
    double right = (tTo - middle) * (tMiddle + 4.0 * right_middle + tRight) / 6.0;
    double error = left + right - tWhole;
    if (!std::isfinite(error)) return false;
    if (std::abs(error) <= 15.0 * tTolerance)
    {
        tLength = left + right + error / 15.0;
        return std::isfinite(tLength) && tLength >= 0.0;
    }
    if (tDepth == 0) return false;
    double left_length = 0.0, right_length = 0.0;
    if (!RefineIntegral(tSegment, tFrom, middle, tLeft, left_middle, tMiddle,
                        left, tTolerance * 0.5, tDepth - 1, left_length) ||
        !RefineIntegral(tSegment, middle, tTo, tMiddle, right_middle, tRight,
                        right, tTolerance * 0.5, tDepth - 1, right_length)) return false;
    tLength = left_length + right_length;
    return true;
}

bool CubicSpline::Integrate(const Segment& tSegment, double tFrom, double tTo,
                            double tTolerance, double& tLength)
{
    if (tFrom == tTo)
    {
        tLength = 0.0;
        return true;
    }
    double left = tSegment.Speed(tFrom);
    double middle = tSegment.Speed(0.5 * (tFrom + tTo));
    double right = tSegment.Speed(tTo);
    double whole = (tTo - tFrom) * (left + 4.0 * middle + right) / 6.0;
    return RefineIntegral(tSegment, tFrom, tTo, left, middle, right, whole,
                          tTolerance, 20, tLength);
}

bool CubicSpline::HasRegularTangent(const Segment& tSegment)
{
    // 切线为零时 heading 和曲率无定义。检查两个导数多项式的根，
    // 避免仅在输出采样点检查而漏掉段内尖点。
    std::vector<double> candidates;
    candidates.push_back(0.0);
    candidates.push_back(tSegment.span);
    const Polynomial axes[] = {tSegment.x, tSegment.y};
    for (int i = 0; i < 2; ++i)
    {
        double a = 3.0 * axes[i].d * tSegment.span * tSegment.span;
        double b = 2.0 * axes[i].c * tSegment.span;
        double c = axes[i].b;
        if (std::abs(a) < 1e-15)
        {
            if (std::abs(b) > 1e-15) candidates.push_back(-c / b * tSegment.span);
            continue;
        }
        double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0) continue;
        double q = -0.5 * (b + std::copysign(std::sqrt(discriminant), b));
        candidates.push_back(q / a * tSegment.span);
        if (q != 0.0) candidates.push_back(c / q * tSegment.span);
    }
    for (std::size_t i = 0; i < candidates.size(); ++i)
    {
        double u = candidates[i];
        if (u >= 0.0 && u <= tSegment.span &&
            (!std::isfinite(tSegment.Speed(u)) || tSegment.Speed(u) < 1e-8)) return false;
    }
    return true;
}

STATUS_S CubicSpline::Fit(const MapPointList& tPoints, DIRECTION_E tDirection)
{
    mSegments.clear();
    mLengths.clear();
    if (tPoints.size() < 2 ||
        (tDirection != DIRECTION_FORWARD && tDirection != DIRECTION_REVERSE))
    {
        return STATUS_S(INVALID_ARGUMENT, "拟合需要至少两个节点及明确的行驶方向");
    }
    std::vector<double> x, y, spans;
    for (std::size_t i = 0; i < tPoints.size(); ++i)
    {
        if (!std::isfinite(tPoints[i].x_axis) || !std::isfinite(tPoints[i].y_axis) ||
            !std::isfinite(tPoints[i].heading)) return STATUS_S(INVALID_DATA, "拟合节点包含非有限数值");
        x.push_back(tPoints[i].x_axis);
        y.push_back(tPoints[i].y_axis);
        if (i == 0) continue;
        double span = std::hypot(x[i] - x[i - 1], y[i] - y[i - 1]);
        if (!std::isfinite(span) || span < 1e-8) return STATUS_S(INVALID_DATA, "相邻拟合节点重合或距离溢出");
        spans.push_back(span);
    }
    double start_yaw = Coordinate::HeadingToYaw(tPoints.front().heading);
    double end_yaw = Coordinate::HeadingToYaw(tPoints.back().heading);
    double sign = static_cast<int>(tDirection);
    std::vector<Polynomial> x_polynomials, y_polynomials;
    if (!BuildAxis(x, spans, sign * std::cos(start_yaw), sign * std::cos(end_yaw), x_polynomials) ||
        !BuildAxis(y, spans, sign * std::sin(start_yaw), sign * std::sin(end_yaw), y_polynomials))
    {
        return STATUS_S(NUMERICAL_ERROR, "样条三对角方程求解失败");
    }
    std::vector<Segment> segments;
    std::vector<double> lengths(1, 0.0);
    for (std::size_t i = 0; i < spans.size(); ++i)
    {
        Segment segment;
        segment.x = x_polynomials[i];
        segment.y = y_polynomials[i];
        segment.span = spans[i];
        if (!HasRegularTangent(segment)) return STATUS_S(NUMERICAL_ERROR, "样条出现零切线尖点，请检查轨迹方向及节点");
        // u 是弦长参数，并非真实弧长；积分 |r'(u)| 后才能按米等距采样。
        if (!Integrate(segment, 0.0, segment.span, 1e-9, segment.length) || segment.length <= 0.0)
        {
            return STATUS_S(NUMERICAL_ERROR, "样条弧长积分未收敛");
        }
        double length = lengths.back() + segment.length;
        if (!std::isfinite(length) || length <= lengths.back())
        {
            return STATUS_S(NUMERICAL_ERROR, "样条累计弧长溢出或失去精度");
        }
        segments.push_back(segment);
        lengths.push_back(length);
    }
    mSegments.swap(segments);
    mLengths.swap(lengths);
    mDirection = tDirection;
    return STATUS_S();
}

double CubicSpline::GetLength() const
{
    return mLengths.empty() ? 0.0 : mLengths.back();
}

STATUS_S CubicSpline::EvaluateByLength(double tLength, MAP_POINT_S& tPoint) const
{
    if (mSegments.empty() || !std::isfinite(tLength) || tLength < 0.0 || tLength > GetLength())
    {
        return STATUS_S(INVALID_ARGUMENT, "采样弧长超出样条范围");
    }
    std::size_t index = static_cast<std::size_t>(
        std::upper_bound(mLengths.begin(), mLengths.end(), tLength) - mLengths.begin() - 1);
    index = std::min(index, mSegments.size() - 1);
    const Segment& segment = mSegments[index];
    double target = tLength - mLengths[index];
    double low = 0.0, high = segment.span;
    double u = segment.span * target / segment.length;
    bool converged = false;
    // 牛顿法加区间约束；弧长单调递增，牛顿步越界时改用二分。
    for (int i = 0; i < 60; ++i)
    {
        double length = 0.0;
        if (!Integrate(segment, 0.0, u, 1e-10, length))
        {
            return STATUS_S(NUMERICAL_ERROR, "采样点弧长积分未收敛");
        }
        double error = length - target;
        if (std::abs(error) <= 1e-8)
        {
            converged = true;
            break;
        }
        if (error > 0.0) high = u;
        else low = u;
        double next = u - error / segment.Speed(u);
        u = std::isfinite(next) && next > low && next < high ? next : 0.5 * (low + high);
    }
    if (!converged) return STATUS_S(NUMERICAL_ERROR, "弧长反解未收敛");
    double dx = segment.x.First(u), dy = segment.y.First(u);
    double speed = std::hypot(dx, dy);
    if (speed < 1e-8) return STATUS_S(NUMERICAL_ERROR, "采样点切线退化");
    MAP_POINT_S point;
    point.x_axis = segment.x.Value(u);
    point.y_axis = segment.y.Value(u);
    double sign = static_cast<int>(mDirection);
    point.heading = Coordinate::YawToHeading(std::atan2(sign * dy, sign * dx));
    point.signed_curvature = (dx * segment.y.Second(u) - dy * segment.x.Second(u)) /
                            (speed * speed * speed);
    point.curvature = std::abs(point.signed_curvature);
    point.dist_origin = tLength;
    if (!std::isfinite(point.x_axis) || !std::isfinite(point.y_axis) ||
        !std::isfinite(point.heading) || !std::isfinite(point.curvature))
    {
        return STATUS_S(NUMERICAL_ERROR, "采样坐标或曲率溢出");
    }
    tPoint = point;
    return STATUS_S();
}
}
}
