#ifndef HDMAP_TRAJECTORY_PROCESSOR_H
#define HDMAP_TRAJECTORY_PROCESSOR_H

#include "hdmap/hdmap_types.h"
#include "hdmap/export.h"

namespace hdmap
{
// 无跨调用缓存；各阶段均先生成临时结果，成功后才替换调用方输出。
class HDMAP_API TrajectoryProcessor
{
public:
    STATUS_S NormalizeHeadings(const MapPointList& tInput, HEADING_TYPE_E tType,
                               MapPointList& tOutput) const;
    STATUS_S ResolveDirection(const MapPointList& tInput, DIRECTION_E tRequested,
                              DIRECTION_E& tDirection) const;
    STATUS_S SampleAnchorPoints(const MapPointList& tInput, double tInterval,
                                MapPointList& tOutput) const;
    STATUS_S FitAndResample(const MapPointList& tAnchors, DIRECTION_E tDirection,
                           const PROCESS_OPTIONS_S& tOptions, MapPointList& tOutput,
                           PROCESS_REPORT_S& tReport) const;
    STATUS_S Process(const MapPointList& tInput, const PROCESS_OPTIONS_S& tOptions,
                     MapPointList& tOutput, PROCESS_REPORT_S& tReport) const;
};
}

#endif
