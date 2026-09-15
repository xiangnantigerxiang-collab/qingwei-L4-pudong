#include "hdmap/hdmap_server.h"
#include "hdmap/coordinate.h"
#include "hdmap/trajectory_processor.h"
#include "hdmap/pnc_adapter.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>

namespace
{
const double PI = 3.14159265358979323846;
int check_count = 0;

void Check(bool tCondition, const char* tMessage)
{
    ++check_count;
    if (!tCondition) throw std::runtime_error(tMessage);
}

void Near(double tActual, double tExpected, double tTolerance, const char* tMessage)
{
    Check(std::isfinite(tActual) && std::abs(tActual - tExpected) <= tTolerance, tMessage);
}

void Good(const hdmap::STATUS_S& tStatus)
{
    if (!tStatus.IsOk()) throw std::runtime_error(tStatus.message);
    ++check_count;
}

hdmap::MapPointList Straight(double tLength, double tHeading = 90.0)
{
    hdmap::MapPointList points;
    int count = static_cast<int>(std::ceil(tLength / 0.1));
    for (int i = 0; i <= count; ++i)
        points.push_back(hdmap::MAP_POINT_S(tLength * i / count, 0.0, tHeading));
    return points;
}

void TestCoordinates()
{
    const double headings[] = {0.0, 90.0, 180.0, 270.0, 359.999};
    for (std::size_t i = 0; i < 5; ++i)
    {
        Near(hdmap::Coordinate::YawToHeading(hdmap::Coordinate::HeadingToYaw(headings[i])),
             headings[i], 1e-12, "heading/yaw roundtrip");
    }
    Near(hdmap::Coordinate::NormalizeHeading(-721.0), 359.0, 1e-12, "negative heading");
    Near(hdmap::Coordinate::HeadingToYaw(0), PI / 2.0, 1e-12, "north yaw");
    double heading = 12.0;
    Good(hdmap::Coordinate::ConvertHeading(PI, hdmap::MATH_YAW_RAD, heading));
    Near(heading, 270.0, 1e-12, "math yaw radians");
    Good(hdmap::Coordinate::ConvertHeading(450, hdmap::MATH_YAW_DEG, heading));
    Near(heading, 0.0, 1e-12, "math yaw degrees");
    Check(!hdmap::Coordinate::ConvertHeading(std::numeric_limits<double>::quiet_NaN(),
          hdmap::PNC_HEADING_DEG, heading).IsOk(), "reject nan");
}

void TestStraightAndSampling()
{
    hdmap::HdMapServer server;
    hdmap::TrajectoryProcessor processor;
    hdmap::PROCESS_OPTIONS_S options;
    hdmap::PROCESS_REPORT_S report;
    hdmap::MapPointList input = Straight(10.2), output, anchors;
    Good(processor.SampleAnchorPoints(input, 1.0, anchors));
    Near(anchors.front().x_axis, 0.0, 0.0, "first anchor");
    Near(anchors.back().x_axis, 10.2, 0.0, "last anchor");
    for (std::size_t i = 1; i < anchors.size(); ++i)
        Check(anchors[i].x_axis - anchors[i - 1].x_axis >= 1.0, "minimum anchor spacing");
    Good(server.ProcessTrajectory(input, options, output, report));
    Check(output.size() == 21 && report.input_count == input.size(), "strict sample count");
    Near(report.remaining_length, 0.2, 1e-8, "strict endpoint remainder");
    for (std::size_t i = 0; i < output.size(); ++i)
    {
        Near(output[i].x_axis, i * 0.5, 1e-8, "straight x");
        Near(output[i].y_axis, 0, 1e-10, "straight y");
        Near(output[i].heading, 90, 1e-10, "straight heading");
        Near(output[i].curvature, 0, 1e-10, "straight curvature");
        Near(output[i].dist_origin, i * 0.5, 1e-12, "straight mileage");
        Near(output[i].p2pDistance, i == 0 ? 0 : 0.5, 1e-8, "PNC chord");
    }
    options.include_endpoint = true;
    Good(server.ProcessTrajectory(input, options, output, report));
    Check(output.size() == 22, "append endpoint count");
    Near(output.back().x_axis, 10.2, 1e-8, "preserve endpoint");
    Near(report.remaining_length, 0, 1e-10, "endpoint no remainder");
    options.include_endpoint = false;
    input = Straight(10.0, 270.0);
    Good(server.ProcessTrajectory(input, options, output, report));
    Check(report.direction == hdmap::DIRECTION_REVERSE, "detect reverse");
    Near(output[3].heading, 270.0, 1e-10, "reverse body heading");
    input = Straight(3.0, 0.0);
    options.input_heading_type = hdmap::MATH_YAW_RAD;
    Good(server.ProcessTrajectory(input, options, output, report));
    Near(output.front().heading, 90.0, 1e-10, "input yaw applied");
    options = hdmap::PROCESS_OPTIONS_S();
    input = Straight(3.0);
    std::size_t input_count = input.size();
    Good(server.ProcessTrajectory(input, options, input, report));
    Check(report.input_count == input_count && input.size() == 7, "aliased input/output");
    input.clear();
    input.push_back(hdmap::MAP_POINT_S(0, 0, 0));
    input.push_back(hdmap::MAP_POINT_S(0, 2, 0));
    Good(server.ProcessTrajectory(input, options, output, report));
    Near(output.back().y_axis, 2, 1e-8, "two point north");
    Near(output[1].heading, 0, 1e-9, "two point north heading");
    input = Straight(3.0);
    input.insert(input.begin() + 5, input[5]);
    Good(server.ProcessTrajectory(input, options, output, report));
    Check(output.size() == 7, "duplicate source point");
}

void TestCircle(int tTurn, bool tReverse)
{
    hdmap::MapPointList input, output;
    const double radius = 10.0;
    for (int i = 0; i <= 300; ++i)
    {
        double angle = -0.1 + tTurn * 0.005 * i;
        double yaw = angle + tTurn * PI / 2.0 + (tReverse ? PI : 0.0);
        input.push_back(hdmap::MAP_POINT_S(radius * std::cos(angle), radius * std::sin(angle),
                        hdmap::Coordinate::YawToHeading(yaw)));
    }
    hdmap::HdMapServer server;
    hdmap::PROCESS_OPTIONS_S options;
    hdmap::PROCESS_REPORT_S report;
    options.include_endpoint = true;
    Good(server.ProcessTrajectory(input, options, output, report));
    Near(report.spline_length, 15.0, 0.0002, "circle analytic length");
    Check(report.direction == (tReverse ? hdmap::DIRECTION_REVERSE : hdmap::DIRECTION_FORWARD),
          "circle direction");
    for (std::size_t i = 0; i < output.size(); ++i)
    {
        const hdmap::MAP_POINT_S& point = output[i];
        double angle = std::atan2(point.y_axis, point.x_axis);
        double yaw = angle + tTurn * PI / 2.0 + (tReverse ? PI : 0.0);
        double expected_heading = hdmap::Coordinate::YawToHeading(yaw);
        Near(std::hypot(point.x_axis, point.y_axis), radius, 0.0002, "circle radius");
        Near(point.signed_curvature, tTurn / radius, 0.001, "signed circle curvature");
        Near(point.curvature, 1.0 / radius, 0.001, "absolute circle curvature");
        Near(std::remainder(point.heading - expected_heading, 360.0), 0, 0.02, "circle heading");
        if (i > 0) Check(point.p2pDistance <= point.dist_origin - output[i - 1].dist_origin + 1e-7,
                         "chord cannot exceed arc");
    }
}

void TestRotatedStraight()
{
    hdmap::HdMapServer server;
    hdmap::PROCESS_OPTIONS_S options;
    hdmap::PROCESS_REPORT_S report;
    hdmap::MapPointList output;
    for (int degree = 0; degree < 360; degree += 7)
    {
        double yaw = degree * PI / 180.0;
        hdmap::MapPointList input;
        input.push_back(hdmap::MAP_POINT_S(0, 0, hdmap::Coordinate::YawToHeading(yaw)));
        input.push_back(hdmap::MAP_POINT_S(10 * std::cos(yaw), 10 * std::sin(yaw), input[0].heading));
        Good(server.ProcessTrajectory(input, options, output, report));
        Check(output.size() == 21, "rotated straight exact endpoint rounding");
    }
    hdmap::MapPointList dense;
    for (int i = 0; i <= 2000; ++i) dense.push_back(hdmap::MAP_POINT_S(i * 0.001, 0, 90));
    Good(server.ProcessTrajectory(dense, options, output, report));
    Check(output.size() == 5, "millimeter dense points direction");
}

void TestInvalidInput()
{
    hdmap::HdMapServer server;
    hdmap::PROCESS_OPTIONS_S options;
    hdmap::PROCESS_REPORT_S report;
    hdmap::MapPointList output(1, hdmap::MAP_POINT_S(123, 456, 789));
    report.input_count = 123;
    Check(!server.ProcessTrajectory(hdmap::MapPointList(), options, output, report).IsOk(), "empty input");
    Check(output.size() == 1 && output[0].x_axis == 123 && report.input_count == 123,
          "failure leaves output and report intact");
    Check(!server.ProcessTrajectory(Straight(0.5), options, output, report).IsOk(), "too short");
    Check(!server.ProcessTrajectory(hdmap::MapPointList(8), options, output, report).IsOk(), "stationary");
    options.anchor_interval = 0.99;
    Check(!server.ProcessTrajectory(Straight(3), options, output, report).IsOk(), "anchor spacing rejected");
    options = hdmap::PROCESS_OPTIONS_S();
    options.sample_interval = 0.0;
    Check(!server.ProcessTrajectory(Straight(3), options, output, report).IsOk(), "zero interval");
    options.sample_interval = 1e-20;
    Check(!server.ProcessTrajectory(Straight(3), options, output, report).IsOk(), "point count limit");
    options = hdmap::PROCESS_OPTIONS_S();
    options.direction = hdmap::DIRECTION_REVERSE;
    Check(!server.ProcessTrajectory(Straight(3), options, output, report).IsOk(), "wrong explicit direction");
    options = hdmap::PROCESS_OPTIONS_S();
    hdmap::MapPointList mixed = Straight(5), back = Straight(5);
    for (std::size_t i = back.size() - 1; i > 0; --i) mixed.push_back(back[i - 1]);
    Check(!server.ProcessTrajectory(mixed, options, output, report).IsOk(), "mixed direction rejected");
    mixed = Straight(3);
    mixed[4].x_axis = std::numeric_limits<double>::infinity();
    Check(!server.ProcessTrajectory(mixed, options, output, report).IsOk(), "infinite coordinate");
    hdmap::TrajectoryProcessor processor;
    mixed.clear();
    mixed.push_back(hdmap::MAP_POINT_S(0, 0, 270));
    mixed.push_back(hdmap::MAP_POINT_S(2, 0, 270));
    Check(!processor.FitAndResample(mixed, hdmap::DIRECTION_FORWARD, options, output, report).IsOk(),
          "zero tangent inside a segment rejected");
}

// 形状只用于模板边界测试；另有使用真实 XYZ_COOR_S 头文件的链接示例。
struct PncPoint
{
    float x_axis, y_axis, z_axis, heading, p2pDistance, curvature, velocity, dist_origin;
};

void TestPncAdapter()
{
    hdmap::MapPointList input(1, hdmap::MAP_POINT_S(2, 3, 359.9999999));
    input[0].curvature = 0.1;
    input[0].signed_curvature = -0.1;
    std::vector<PncPoint> output;
    Good(hdmap::ConvertToPncPath(input, output));
    Near(output[0].x_axis, 2, 0, "PNC x");
    Near(output[0].z_axis, 0, 0, "PNC lane switch clear");
    Near(output[0].velocity, 0, 0, "PNC velocity clear");
    Near(output[0].curvature, 0.1, 1e-7, "PNC curvature magnitude");
    Check(output[0].heading < 360.0f, "float heading wrap");
    input[0].x_axis = 1e100;
    Check(!hdmap::ConvertToPncPath(input, output).IsOk() && output[0].x_axis == 2,
          "adapter overflow preserves result");
}

void WriteText(const std::string& tFile, const std::string& tText)
{
    std::ofstream stream(tFile.c_str(), std::ios::binary);
    stream << tText;
    Check(stream.good(), "test fixture write");
}

void TestFiles()
{
    char pattern[] = "/tmp/hdmap_test_XXXXXX";
    char* directory = mkdtemp(pattern);
    Check(directory != NULL, "test directory");
    std::string root(directory), source = root + "/input.csv", output_file = root + "/output.csv";
    hdmap::HdMapServer server;
    hdmap::PROCESS_OPTIONS_S options;
    hdmap::PROCESS_REPORT_S report;
    hdmap::MapPointList points;
    hdmap::DIRECTION_E direction = hdmap::DIRECTION_AUTO;
    WriteText(source, "\xEF\xBB\xBF# map\r\nx,y,heading\r\n0,0,90\r\n\r\n1,0,90\r\n2,0,90\r\n");
    Good(server.ReadTrajectory(source, points));
    Check(points.size() == 3, "BOM header CRLF blank comments");
    Good(server.ProcessFile(source, output_file, options, report));
    Good(server.LoadProcessedTrajectory(output_file, points, direction));
    Check(points.size() == 5 && direction == hdmap::DIRECTION_FORWARD, "CSV roundtrip");
    Near(points[3].x_axis, 1.5, 1e-8, "CSV geometry");
    Check(!server.ProcessFile(source, source, options, report).IsOk(), "no overwrite source");
    Check(!server.ProcessFile(source, output_file, options, report).IsOk(), "no overwrite result");
    Good(server.LoadProcessedTrajectory(output_file, points, direction));
    std::string alias = root + "/alias.csv";
    Check(symlink(source.c_str(), alias.c_str()) == 0, "source symlink");
    Check(!server.ProcessFile(source, alias, options, report).IsOk(), "symlink source protected");
    unlink(alias.c_str());
    const char* bad_rows[] = {"1,0,nan", "1,0,inf", "1,0,90junk", "1,,90", "1,0,90,7"};
    for (std::size_t i = 0; i < 5; ++i)
    {
        WriteText(source, std::string("0,0,90\n") + bad_rows[i] + "\n2,0,90\n");
        hdmap::STATUS_S status = server.ReadTrajectory(source, points);
        Check(!status.IsOk() && status.message.find(":2:") != std::string::npos, "bad row location");
        Check(points.size() == 5, "failed read preserves result");
    }
    WriteText(source, "0,0,90\n1,0,90\n2,0,90\n");
    unlink(output_file.c_str());
    std::string broken = root + "/broken.csv";
    WriteText(broken, "bad\n");
    std::vector<hdmap::FILE_RESULT_S> results;
    hdmap::STATUS_S status = server.ProcessDirectory(root, root + "/new/maps", options, results);
    Check(!status.IsOk() && results.size() == 2, "batch returns all outcomes");
    Check(!results[0].status.IsOk() && results[1].status.IsOk(), "batch sorted partial success");
    Good(server.LoadProcessedTrajectory(root + "/new/maps/input_spline.csv", points, direction));
    Check(!server.ProcessDirectory(root, root + "/.", options, results).IsOk(), "same directory alias");
    unlink((root + "/new/maps/input_spline.csv").c_str());
    rmdir((root + "/new/maps").c_str());
    rmdir((root + "/new").c_str());
    unlink(source.c_str());
    unlink(broken.c_str());
    rmdir(root.c_str());
}
}

int main()
{
    try
    {
        TestCoordinates();
        TestStraightAndSampling();
        TestRotatedStraight();
        TestCircle(1, false);
        TestCircle(-1, false);
        TestCircle(1, true);
        TestCircle(-1, true);
        TestInvalidInput();
        TestPncAdapter();
        TestFiles();
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "FAIL after %d checks: %s\n", check_count, error.what());
        return 1;
    }
    std::printf("PASS: %d checks\n", check_count);
    return 0;
}
