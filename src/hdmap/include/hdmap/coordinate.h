#ifndef HDMAP_COORDINATE_H
#define HDMAP_COORDINATE_H

#include "hdmap/hdmap_types.h"
#include "hdmap/export.h"

namespace hdmap
{
class HDMAP_API Coordinate
{
public:
    static double NormalizeHeading(double tHeading);
    static double HeadingToYaw(double tHeading);
    static double YawToHeading(double tYaw);
    static STATUS_S ConvertHeading(double tHeading, HEADING_TYPE_E tType,
                                   double& tPncHeading);
};
}

#endif
