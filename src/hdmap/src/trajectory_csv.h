#ifndef HDMAP_TRAJECTORY_CSV_H
#define HDMAP_TRAJECTORY_CSV_H

#include "hdmap/hdmap_types.h"

namespace hdmap
{
namespace detail
{
class TrajectoryCsv
{
public:
    static STATUS_S Read(const std::string& tFile, bool tProcessed,
                         MapPointList& tOutput, DIRECTION_E& tDirection);
    static STATUS_S Write(const std::string& tFile, const MapPointList& tPoints,
                          DIRECTION_E tDirection);
};

STATUS_S ListCsvFiles(const std::string& tDirectory, std::vector<std::string>& tFiles);
STATUS_S PrepareOutputDirectory(const std::string& tInput, const std::string& tOutput);
bool IsSameFile(const std::string& tFirst, const std::string& tSecond);
}
}

#endif
