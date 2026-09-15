#ifndef HDMAP_SERVER_H
#define HDMAP_SERVER_H

#include "hdmap/hdmap_types.h"
#include "hdmap/export.h"

namespace hdmap
{
// SDK 门面：负责文件与处理流程编排，算法、CSV 和 PNC 适配各自独立。
class HDMAP_API HdMapServer
{
public:
    STATUS_S ReadTrajectory(const std::string& tFile, MapPointList& tOutput) const;
    STATUS_S LoadProcessedTrajectory(const std::string& tFile, MapPointList& tOutput,
                                     DIRECTION_E& tDirection) const;
    STATUS_S SaveTrajectory(const std::string& tFile, const MapPointList& tPoints,
                            DIRECTION_E tDirection) const;
    STATUS_S ProcessTrajectory(const MapPointList& tInput,
                               const PROCESS_OPTIONS_S& tOptions,
                               MapPointList& tOutput, PROCESS_REPORT_S& tReport) const;
    STATUS_S ProcessFile(const std::string& tInputFile, const std::string& tOutputFile,
                         const PROCESS_OPTIONS_S& tOptions,
                         PROCESS_REPORT_S& tReport) const;
    // 只扫描目录直属普通 .csv 文件，按文件名排序；每条轨迹独立处理，不跨文件连线。
    // 批处理返回每个文件的成功/失败，任意失败时整体返回失败；已成功文件仍保留。
    STATUS_S ProcessDirectory(const std::string& tInputDir, const std::string& tOutputDir,
                              const PROCESS_OPTIONS_S& tOptions,
                              std::vector<FILE_RESULT_S>& tResults) const;
};
}

#endif
