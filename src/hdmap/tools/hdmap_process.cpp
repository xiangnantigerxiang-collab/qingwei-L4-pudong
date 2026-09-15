#include "hdmap/hdmap_server.h"
#include <cstdio>
#include <locale>
#include <sstream>

namespace
{
void PrintUsage(const char* tProgram)
{
    std::printf("用法: %s <原始 CSV 目录> <新输出目录> [选项]\n"
                "  --heading pnc|yaw-rad|yaw-deg  输入航向制，默认 pnc\n"
                "  --direction auto|forward|reverse  行驶方向，默认 auto\n"
                "  --anchor-interval <米>  拟合节点最小距离，默认 1.0，必须 >= 1\n"
                "  --sample-interval <米>  输出弧长间距，默认 0.5\n"
                "  --include-endpoint  额外保留原始终点，尾段可不足采样间距\n"
                "已有文件不覆盖；重复处理请指定新的输出目录。\n", tProgram);
}

bool ParseNumber(const std::string& tText, double& tValue)
{
    std::istringstream stream(tText);
    stream.imbue(std::locale::classic());
    if (!(stream >> tValue)) return false;
    stream >> std::ws;
    return stream.eof();
}

bool ParseOptions(int tCount, char** tArgs, hdmap::PROCESS_OPTIONS_S& tOptions)
{
    for (int i = 3; i < tCount; ++i)
    {
        std::string option(tArgs[i]);
        if (option == "--include-endpoint")
        {
            tOptions.include_endpoint = true;
            continue;
        }
        if (i + 1 >= tCount) return false;
        std::string value(tArgs[++i]);
        if (option == "--heading")
        {
            if (value == "pnc") tOptions.input_heading_type = hdmap::PNC_HEADING_DEG;
            else if (value == "yaw-rad") tOptions.input_heading_type = hdmap::MATH_YAW_RAD;
            else if (value == "yaw-deg") tOptions.input_heading_type = hdmap::MATH_YAW_DEG;
            else return false;
        }
        else if (option == "--direction")
        {
            if (value == "auto") tOptions.direction = hdmap::DIRECTION_AUTO;
            else if (value == "forward") tOptions.direction = hdmap::DIRECTION_FORWARD;
            else if (value == "reverse") tOptions.direction = hdmap::DIRECTION_REVERSE;
            else return false;
        }
        else if (option == "--anchor-interval")
        {
            if (!ParseNumber(value, tOptions.anchor_interval)) return false;
        }
        else if (option == "--sample-interval")
        {
            if (!ParseNumber(value, tOptions.sample_interval)) return false;
        }
        else return false;
    }
    return true;
}
}

int main(int argc, char** argv)
{
    if (argc == 2 && std::string(argv[1]) == "--help")
    {
        PrintUsage(argv[0]);
        return 0;
    }
    hdmap::PROCESS_OPTIONS_S options;
    if (argc < 3 || !ParseOptions(argc, argv, options))
    {
        PrintUsage(argv[0]);
        return 2;
    }
    hdmap::HdMapServer server;
    std::vector<hdmap::FILE_RESULT_S> results;
    hdmap::STATUS_S status = server.ProcessDirectory(argv[1], argv[2], options, results);
    for (std::size_t i = 0; i < results.size(); ++i)
    {
        const hdmap::FILE_RESULT_S& result = results[i];
        if (!result.status.IsOk())
        {
            std::fprintf(stderr, "失败: %s\n", result.status.message.c_str());
            continue;
        }
        const hdmap::PROCESS_REPORT_S& report = result.report;
        std::printf("%s -> %s\n  原始 %zu 点，拟合 %zu 点，输出 %zu 点，%s，"
                    "总弧长 %.9f 米，未输出尾段 %.9f 米\n",
                    result.input_file.c_str(), result.output_file.c_str(),
                    report.input_count, report.anchor_count, report.output_count,
                    report.direction == hdmap::DIRECTION_REVERSE ? "倒车" : "前进",
                    report.spline_length, report.remaining_length);
    }
    if (!status.IsOk())
    {
        std::fprintf(stderr, "%s\n", status.message.c_str());
        return 1;
    }
    std::printf("完成：%zu 条轨迹。\n", results.size());
    return 0;
}
