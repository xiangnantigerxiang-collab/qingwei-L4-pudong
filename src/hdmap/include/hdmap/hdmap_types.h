#ifndef HDMAP_TYPES_H
#define HDMAP_TYPES_H

#include <cstddef>
#include <string>
#include <vector>

namespace hdmap
{
enum ERROR_CODE_E
{
    SUCCESS = 0,
    INVALID_ARGUMENT,
    FILE_ERROR,
    INVALID_DATA,
    NUMERICAL_ERROR
};

typedef struct status_s
{
    status_s(ERROR_CODE_E tCode = SUCCESS, const std::string& tMessage = "")
        : code(tCode), message(tMessage) {}
    bool IsOk() const { return code == SUCCESS; }
    ERROR_CODE_E code;
    std::string message;
} STATUS_S;

enum HEADING_TYPE_E
{
    PNC_HEADING_DEG = 0, // 正北为 0 度，顺时针为正。
    MATH_YAW_RAD,        // 正东为 0 弧度，逆时针为正。
    MATH_YAW_DEG
};

enum DIRECTION_E
{
    DIRECTION_AUTO = 0,
    DIRECTION_FORWARD = 1,
    DIRECTION_REVERSE = -1
};

// 与 PNC XYZ_COOR_S 的平面坐标、heading、曲率量纲对齐，计算保留 double 精度。
typedef struct map_point_s
{
    map_point_s(double tX = 0.0, double tY = 0.0, double tHeading = 0.0)
        : x_axis(tX), y_axis(tY), heading(tHeading), curvature(0.0),
          signed_curvature(0.0), dist_origin(0.0), p2pDistance(0.0) {}
    double x_axis;           // 原地图平面 x，米；不改变定位原点。
    double y_axis;           // 原地图平面 y，米。
    double heading;          // 车头方位角 [0, 360)，度；倒车时与点序切线相反。
    double curvature;        // 曲率绝对值，1/m，与 PNC 控制侧一致。
    double signed_curvature; // 沿点序左转为正、右转为负，1/m。
    double dist_origin;      // 从起点累计的样条真实弧长，米。
    double p2pDistance;      // 到前一个输出点的弦长，米；首点为 0。
} MAP_POINT_S;

typedef std::vector<MAP_POINT_S> MapPointList;

typedef struct process_options_s
{
    process_options_s()
        : input_heading_type(PNC_HEADING_DEG), direction(DIRECTION_AUTO),
          anchor_interval(1.0), sample_interval(0.5), include_endpoint(false) {}
    HEADING_TYPE_E input_heading_type;
    DIRECTION_E direction;
    double anchor_interval; // 相邻拟合节点的最小平面距离，必须 >= 1 米。
    double sample_interval; // 默认严格按 0.5 米弧长采样。
    bool include_endpoint;  // 为 true 时额外保留终点，最后一段可不足采样间距。
} PROCESS_OPTIONS_S;

typedef struct process_report_s
{
    process_report_s()
        : input_count(0), anchor_count(0), output_count(0),
          direction(DIRECTION_AUTO), spline_length(0.0), remaining_length(0.0) {}
    std::size_t input_count;
    std::size_t anchor_count;
    std::size_t output_count;
    DIRECTION_E direction;
    double spline_length;
    double remaining_length; // 严格等距模式下未输出的末端弧长。
} PROCESS_REPORT_S;

typedef struct file_result_s
{
    std::string input_file;
    std::string output_file;
    STATUS_S status;
    PROCESS_REPORT_S report;
} FILE_RESULT_S;
}

#endif
