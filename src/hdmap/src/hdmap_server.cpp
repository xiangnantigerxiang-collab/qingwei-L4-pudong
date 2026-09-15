#include "hdmap/hdmap_server.h"
#include "hdmap/trajectory_processor.h"
#include "trajectory_csv.h"

namespace hdmap
{
STATUS_S HdMapServer::ReadTrajectory(const std::string& tFile, MapPointList& tOutput) const
{
    DIRECTION_E direction = DIRECTION_AUTO;
    return detail::TrajectoryCsv::Read(tFile, false, tOutput, direction);
}

STATUS_S HdMapServer::LoadProcessedTrajectory(const std::string& tFile, MapPointList& tOutput,
                                             DIRECTION_E& tDirection) const
{
    return detail::TrajectoryCsv::Read(tFile, true, tOutput, tDirection);
}

STATUS_S HdMapServer::SaveTrajectory(const std::string& tFile, const MapPointList& tPoints,
                                    DIRECTION_E tDirection) const
{
    return detail::TrajectoryCsv::Write(tFile, tPoints, tDirection);
}

STATUS_S HdMapServer::ProcessTrajectory(const MapPointList& tInput,
                                       const PROCESS_OPTIONS_S& tOptions,
                                       MapPointList& tOutput, PROCESS_REPORT_S& tReport) const
{
    return TrajectoryProcessor().Process(tInput, tOptions, tOutput, tReport);
}

STATUS_S HdMapServer::ProcessFile(const std::string& tInputFile, const std::string& tOutputFile,
                                 const PROCESS_OPTIONS_S& tOptions,
                                 PROCESS_REPORT_S& tReport) const
{
    if (tInputFile == tOutputFile || detail::IsSameFile(tInputFile, tOutputFile))
        return STATUS_S(INVALID_ARGUMENT, "输入与输出不能是同一文件");
    MapPointList input, output;
    STATUS_S status = ReadTrajectory(tInputFile, input);
    if (!status.IsOk()) return status;
    PROCESS_REPORT_S report;
    status = ProcessTrajectory(input, tOptions, output, report);
    if (!status.IsOk()) return STATUS_S(status.code, tInputFile + ": " + status.message);
    status = SaveTrajectory(tOutputFile, output, report.direction);
    if (!status.IsOk()) return status;
    tReport = report;
    return STATUS_S();
}

STATUS_S HdMapServer::ProcessDirectory(const std::string& tInputDir,
                                      const std::string& tOutputDir,
                                      const PROCESS_OPTIONS_S& tOptions,
                                      std::vector<FILE_RESULT_S>& tResults) const
{
    std::vector<std::string> files;
    STATUS_S status = detail::ListCsvFiles(tInputDir, files);
    if (!status.IsOk()) return status;
    status = detail::PrepareOutputDirectory(tInputDir, tOutputDir);
    if (!status.IsOk()) return status;
    std::vector<FILE_RESULT_S> results;
    std::size_t failed_count = 0;
    for (std::size_t i = 0; i < files.size(); ++i)
    {
        FILE_RESULT_S result;
        result.input_file = files[i];
        std::string name = files[i].substr(files[i].find_last_of('/') + 1);
        result.output_file = tOutputDir + "/" + name.substr(0, name.size() - 4) + "_spline.csv";
        result.status = ProcessFile(result.input_file, result.output_file, tOptions, result.report);
        if (!result.status.IsOk()) ++failed_count;
        results.push_back(result);
    }
    tResults.swap(results);
    if (failed_count != 0)
        return STATUS_S(INVALID_DATA, std::to_string(failed_count) + " 条轨迹处理失败，详见逐文件结果");
    return STATUS_S();
}
}
