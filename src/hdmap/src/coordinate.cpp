#include "hdmap/coordinate.h"
#include <cmath>

namespace hdmap
{
namespace
{
const double PI = 3.14159265358979323846;
}

double Coordinate::NormalizeHeading(double tHeading)
{
    double heading = std::fmod(tHeading, 360.0);
    if (heading < 0.0) heading += 360.0;
    return heading == 0.0 || heading >= 360.0 ? 0.0 : heading;
}

double Coordinate::HeadingToYaw(double tHeading)
{
    return std::remainder(90.0 - NormalizeHeading(tHeading), 360.0) * PI / 180.0;
}

double Coordinate::YawToHeading(double tYaw)
{
    return NormalizeHeading(90.0 - std::remainder(tYaw, 2.0 * PI) * 180.0 / PI);
}

STATUS_S Coordinate::ConvertHeading(double tHeading, HEADING_TYPE_E tType,
                                   double& tPncHeading)
{
    if (!std::isfinite(tHeading)) return STATUS_S(INVALID_DATA, "heading 必须为有限数值");
    switch (tType)
    {
        case PNC_HEADING_DEG:
            tPncHeading = NormalizeHeading(tHeading);
            break;
        case MATH_YAW_RAD:
            tPncHeading = YawToHeading(tHeading);
            break;
        case MATH_YAW_DEG:
            tPncHeading = NormalizeHeading(90.0 - std::remainder(tHeading, 360.0));
            break;
        default:
            return STATUS_S(INVALID_ARGUMENT, "未知的输入 heading 类型");
    }
    return STATUS_S();
}
}
