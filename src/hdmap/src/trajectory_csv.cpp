#include "trajectory_csv.h"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

namespace hdmap
{
namespace detail
{
namespace
{
const char* PROCESSED_HEADER = "x,y,heading,curvature,signed_curvature,dist_origin,p2p_distance,direction";

std::string Trim(const std::string& tText)
{
    std::size_t start = tText.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    return tText.substr(start, tText.find_last_not_of(" \t\r\n") - start + 1);
}

std::vector<std::string> Split(const std::string& tLine)
{
    std::vector<std::string> fields;
    std::size_t start = 0;
    while (true)
    {
        std::size_t end = tLine.find(',', start);
        fields.push_back(Trim(tLine.substr(start, end == std::string::npos ? end : end - start)));
        if (end == std::string::npos) break;
        start = end + 1;
    }
    return fields;
}

STATUS_S DataError(const std::string& tFile, std::size_t tLine, const std::string& tMessage)
{
    return STATUS_S(INVALID_DATA, tFile + ":" + std::to_string(tLine) + ": " + tMessage);
}

STATUS_S ValidateProcessedPoint(const MAP_POINT_S& tPoint, const MapPointList& tPrevious)
{
    const double values[] = {tPoint.x_axis, tPoint.y_axis, tPoint.heading, tPoint.curvature,
        tPoint.signed_curvature, tPoint.dist_origin, tPoint.p2pDistance};
    for (std::size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i)
    {
        if (!std::isfinite(values[i])) return STATUS_S(INVALID_DATA, "地图点包含非有限数值");
    }
    if (tPoint.heading < 0.0 || tPoint.heading >= 360.0 || tPoint.curvature < 0.0 ||
        tPoint.dist_origin < 0.0 || tPoint.p2pDistance < 0.0 ||
        std::abs(tPoint.curvature - std::abs(tPoint.signed_curvature)) > 1e-9)
    {
        return STATUS_S(INVALID_DATA, "航向、曲率或距离不符合 SDK 约定");
    }
    if (tPrevious.empty())
    {
        if (tPoint.dist_origin != 0.0 || tPoint.p2pDistance != 0.0)
            return STATUS_S(INVALID_DATA, "首点里程及点间距离必须为 0");
    }
    else
    {
        const MAP_POINT_S& previous = tPrevious.back();
        double chord = std::hypot(tPoint.x_axis - previous.x_axis, tPoint.y_axis - previous.y_axis);
        double arc = tPoint.dist_origin - previous.dist_origin;
        if (!std::isfinite(chord) || arc <= 0.0 || chord > arc + 1e-6 ||
            std::abs(chord - tPoint.p2pDistance) > 1e-6)
        {
            return STATUS_S(INVALID_DATA, "累计弧长不递增或与点间距离不一致");
        }
    }
    return STATUS_S();
}

STATUS_S FileError(const std::string& tFile)
{
    return STATUS_S(FILE_ERROR, tFile + ": " + std::strerror(errno));
}
}

STATUS_S TrajectoryCsv::Read(const std::string& tFile, bool tProcessed,
                            MapPointList& tOutput, DIRECTION_E& tDirection)
{
    std::ifstream stream(tFile.c_str());
    if (!stream.is_open()) return FileError(tFile);
    stream.imbue(std::locale::classic());
    MapPointList points;
    DIRECTION_E direction = DIRECTION_AUTO;
    std::string line;
    std::size_t line_number = 0;
    bool first_record = true;
    while (std::getline(stream, line))
    {
        ++line_number;
        if (line_number == 1 && line.compare(0, 3, "\xEF\xBB\xBF") == 0) line.erase(0, 3);
        line = Trim(line);
        if (line.empty() || line[0] == '#') continue;
        std::vector<std::string> fields = Split(line);
        if (first_record)
        {
            first_record = false;
            std::vector<std::string> header = Split(tProcessed ? PROCESSED_HEADER : "x,y,heading");
            if (fields == header) continue;
            if (tProcessed) return DataError(tFile, line_number, "缺少 SDK 输出格式表头");
        }
        std::size_t columns = tProcessed ? 8 : 3;
        if (fields.size() != columns)
            return DataError(tFile, line_number, "列数错误，期望 " + std::to_string(columns) + " 列");
        double values[8] = {};
        for (std::size_t i = 0; i < columns; ++i)
        {
            std::istringstream field(fields[i]);
            field.imbue(std::locale::classic());
            // 整字段解析，拒绝 1.2abc、空字段、NaN/Inf；不把坏行静默丢弃。
            if (!(field >> values[i]) || !std::isfinite(values[i]))
                return DataError(tFile, line_number, "第 " + std::to_string(i + 1) + " 列不是有限数值");
            field >> std::ws;
            if (!field.eof()) return DataError(tFile, line_number, "数值字段包含多余字符");
        }
        MAP_POINT_S point(values[0], values[1], values[2]);
        if (tProcessed)
        {
            point.curvature = values[3];
            point.signed_curvature = values[4];
            point.dist_origin = values[5];
            point.p2pDistance = values[6];
            if (values[7] != 1.0 && values[7] != -1.0)
                return DataError(tFile, line_number, "direction 必须为 1 或 -1");
            DIRECTION_E row_direction = values[7] == 1.0 ? DIRECTION_FORWARD : DIRECTION_REVERSE;
            if (direction != DIRECTION_AUTO && direction != row_direction)
                return DataError(tFile, line_number, "同一文件不允许混合前进和倒车");
            direction = row_direction;
            STATUS_S status = ValidateProcessedPoint(point, points);
            if (!status.IsOk()) return DataError(tFile, line_number, status.message);
        }
        points.push_back(point);
    }
    if (stream.bad()) return STATUS_S(FILE_ERROR, tFile + ": 读取失败");
    if (points.size() < 2) return DataError(tFile, line_number, "轨迹至少需要两个点");
    tOutput.swap(points);
    tDirection = direction;
    return STATUS_S();
}

STATUS_S TrajectoryCsv::Write(const std::string& tFile, const MapPointList& tPoints,
                             DIRECTION_E tDirection)
{
    if (tFile.empty() || tPoints.size() < 2 ||
        (tDirection != DIRECTION_FORWARD && tDirection != DIRECTION_REVERSE))
        return STATUS_S(INVALID_ARGUMENT, "导出需要文件名、至少两个点和明确行驶方向");
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << PROCESSED_HEADER << '\n' << std::setprecision(17);
    MapPointList previous;
    for (std::size_t i = 0; i < tPoints.size(); ++i)
    {
        const MAP_POINT_S& point = tPoints[i];
        STATUS_S status = ValidateProcessedPoint(point, previous);
        if (!status.IsOk()) return DataError(tFile, i + 2, status.message);
        stream << point.x_axis << ',' << point.y_axis << ',' << point.heading << ','
               << point.curvature << ',' << point.signed_curvature << ',' << point.dist_origin << ','
               << point.p2pDistance << ',' << static_cast<int>(tDirection) << '\n';
        if (previous.empty()) previous.push_back(point);
        else previous[0] = point;
    }
    std::string text = stream.str();
    std::string pattern = tFile + ".tmp.XXXXXX";
    std::vector<char> temporary(pattern.begin(), pattern.end());
    temporary.push_back('\0');
    int descriptor = mkstemp(temporary.data());
    if (descriptor < 0) return FileError(tFile);
    STATUS_S status;
    std::size_t offset = 0;
    while (offset < text.size())
    {
        ssize_t written = write(descriptor, text.data() + offset, text.size() - offset);
        if (written < 0 && errno == EINTR) continue;
        if (written <= 0)
        {
            status = FileError(tFile);
            break;
        }
        offset += static_cast<std::size_t>(written);
    }
    if (status.IsOk() && fsync(descriptor) != 0) status = FileError(tFile);
    if (close(descriptor) != 0 && status.IsOk()) status = FileError(tFile);
    // 临时文件与目标同目录；link 原子发布且不覆盖既有文件（包括原图/软链接）。
    if (status.IsOk() && link(temporary.data(), tFile.c_str()) != 0) status = FileError(tFile);
    unlink(temporary.data());
    return status;
}

bool IsSameFile(const std::string& tFirst, const std::string& tSecond)
{
    struct stat first, second;
    return stat(tFirst.c_str(), &first) == 0 && stat(tSecond.c_str(), &second) == 0 &&
           first.st_dev == second.st_dev && first.st_ino == second.st_ino;
}

STATUS_S ListCsvFiles(const std::string& tDirectory, std::vector<std::string>& tFiles)
{
    DIR* directory = opendir(tDirectory.c_str());
    if (!directory) return FileError(tDirectory);
    std::vector<std::string> files;
    STATUS_S status;
    while (true)
    {
        errno = 0;
        struct dirent* entry = readdir(directory);
        if (!entry)
        {
            if (errno != 0) status = FileError(tDirectory);
            break;
        }
        std::string name(entry->d_name);
        if (name.empty() || name[0] == '.' || name.size() <= 4 ||
            name.substr(name.size() - 4) != ".csv") continue;
        std::string path = tDirectory + "/" + name;
        struct stat info;
        if (lstat(path.c_str(), &info) != 0)
        {
            status = FileError(path);
            break;
        }
        if (S_ISREG(info.st_mode)) files.push_back(path);
    }
    closedir(directory);
    if (!status.IsOk()) return status;
    if (files.empty()) return STATUS_S(INVALID_DATA, tDirectory + ": 没有普通 .csv 轨迹文件");
    std::sort(files.begin(), files.end());
    tFiles.swap(files);
    return STATUS_S();
}

STATUS_S PrepareOutputDirectory(const std::string& tInput, const std::string& tOutput)
{
    if (tOutput.empty()) return STATUS_S(INVALID_ARGUMENT, "输出目录不能为空");
    for (std::size_t i = 1; i <= tOutput.size(); ++i)
    {
        if (i != tOutput.size() && tOutput[i] != '/') continue;
        std::string part = tOutput.substr(0, i);
        if (mkdir(part.c_str(), 0755) != 0 && errno != EEXIST) return FileError(part);
        struct stat info;
        if (stat(part.c_str(), &info) != 0) return FileError(part);
        if (!S_ISDIR(info.st_mode)) return STATUS_S(FILE_ERROR, part + ": 不是目录");
    }
    if (IsSameFile(tInput, tOutput)) return STATUS_S(INVALID_ARGUMENT, "输入和输出目录必须不同");
    return STATUS_S();
}
}
}
