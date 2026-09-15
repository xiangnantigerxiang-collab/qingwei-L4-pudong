#include "hdmap/lane_map_server.h"
#include "hdmap/hdmap_server.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sys/stat.h>
#include <unistd.h>

namespace {
    int checks = 0;
    const double PI = 3.14159265358979323846;
    void Check(bool tCondition, const char* tMessage) {
        ++checks;
        if(!tCondition) {
            throw std::runtime_error(tMessage);
        }
    }
    void Good(const hdmap::STATUS_S& tStatus) {
        ++checks;
        if(!tStatus.IsOk()) {
            throw std::runtime_error(tStatus.message);
        }
    }
    void Near(double a, double b, double tolerance, const char* message) {
        Check(std::isfinite(a) && std::abs(a - b) <= tolerance, message);
    }
    struct Directory {
        Directory() {
            char path[] = "/tmp/hdmap_lanes_XXXXXX";
            char* result = mkdtemp(path);
            if(!result) {
                throw std::runtime_error("mkdtemp");
            }
            name = result;
        }
        ~Directory() {
            for(std::size_t i = 0; i < files.size(); ++i) {
                std::remove(files[i].c_str());
            }
            rmdir(name.c_str());
        }
        std::string Add(const std::string& tName) {
            files.push_back(name + "/" + tName + ".csv");
            return files.back();
        }
        std::string name;
        std::vector<std::string> files;
    };
    void Save(Directory& dir, const std::string& name, hdmap::MapPointList points) {
        for(std::size_t i = 0; i < points.size(); ++i) {
            double span = i == 0 ? 0 : std::hypot(points[i].x_axis - points[i - 1].x_axis, points[i].y_axis - points[i - 1].y_axis);
            points[i].p2pDistance = span;
            points[i].dist_origin = i == 0 ? 0 : points[i - 1].dist_origin + span;
        }
        Good(hdmap::HdMapServer().SaveTrajectory(dir.Add(name), points, hdmap::DIRECTION_FORWARD));
    }
    void Straight(Directory& dir, const std::string& name, double offset,
                  double angle = 0, double origin = 0, bool reverse = false) {
        hdmap::MapPointList points;
        for(int i = 0; i <= 200; ++i) {
            double x = i * 0.5, y = offset;
            points.push_back(hdmap::MAP_POINT_S(origin + x * std::cos(angle) - y * std::sin(angle),
                                                origin + x * std::sin(angle) + y * std::cos(angle),
                                                hdmap::Coordinate::YawToHeading(angle + (reverse ? PI : 0))));
        }
        if(reverse) {
            std::reverse(points.begin(), points.end());
        }
        Save(dir, name, points);
    }
    hdmap::OBSTACLE_BOX_S Box(double x, double y, double dx = 2, double dy = 2, double yaw = 0) {
        hdmap::OBSTACLE_BOX_S box;
        box.x = x;
        box.y = y;
        box.dx = dx;
        box.dy = dy;
        box.yaw = yaw;
        return box;
    }
    hdmap::VEHICLE_POSE_S Pose(double x, double y, double heading = 90) {
        hdmap::VEHICLE_POSE_S pose;
        pose.x = x;
        pose.y = y;
        pose.heading = heading;
        return pose;
    }
    hdmap::LANE_MATCH_S Match(const hdmap::LaneMapServer& server, const hdmap::VEHICLE_POSE_S& pose,
                              const hdmap::OBSTACLE_BOX_S& box, int expected) {
        std::vector<hdmap::LANE_MATCH_S> matches;
        Good(server.ClassifyBoxes(pose, {box}, matches));
        Check(matches.size() == 1 && matches[0].type == expected, "unexpected lane classification");
        return matches[0];
    }
    void TestStraightAndMaximumArea() {
        Directory dir;
        Straight(dir, "a_current", 0);
        Straight(dir, "b_left", 4);
        Straight(dir, "c_oncoming", 8, 0, 0, true);
        hdmap::LaneMapServer server;
        Good(server.LoadMap(dir.name));
        Check(server.GetMapInfo().lane_count == 3 && server.GetLaneNames()[0] == "a_current.csv", "map info");
        hdmap::VEHICLE_POSE_S pose = Pose(10, 0);
        for(int lane = 0; lane < 3; ++lane) {
            auto result = Match(server, pose, Box(30, lane * 4), lane);
            Near(result.overlap_area, 4, 1e-8, "full box area");
        }
        Match(server, pose, Box(30, 14), 3);
        Match(server, pose, Box(30, -6), 4);
        Near(Match(server, pose, Box(30, 2.8, 2, 4), 1).overlap_area, 5.6, 1e-8, "maximum coverage left one");
        Near(Match(server, pose, Box(30, 6.8, 2, 4), 2).overlap_area, 5.6, 1e-8, "maximum coverage left two");
        Match(server, pose, Box(30, 2, 2, 4), 0);   // 相同面积与距离，稳定选本车道。
        Match(server, pose, Box(30, 5, 2, 16), 1);  // 三车道面积相等时取最近中心线。
        Match(server, pose, Box(30, 11, 2, 4), 2);  // 中心在车道外，框仍占据左二。
        Match(server, pose, Box(30, 11, 2, 2), 3);  // 仅接触边界，面积为零。
        Match(server, pose, Box(-5, 0), 3);         // 不把车道起点外无限延伸。
        Match(server, Pose(10, 8, 270), Box(30, 8), 0);
        Match(server, Pose(10, 8, 270), Box(30, 4), 1);
        Match(server, Pose(10, 8, 270), Box(30, 0), 2);
        Match(server, Pose(10, -10), Box(30, 0), 3);  // 自车离道，即使障碍物在车道上也只分左右。
        Match(server, Pose(10, 20), Box(30, 0), 4);
        Match(server, Pose(10, 20), Box(30, 20), 3);
        Match(server, Pose(10, 20, 270), Box(30, 0), 3);
        Match(server, Pose(10, 20), Box(30, 20, 2, 20), 3);  // 跨自车轴线按中心侧别。
        std::vector<hdmap::LANE_MATCH_S> output(1);
        output[0].type = 99;
        auto invalid = Box(1, 0, 0, 2);
        Check(!server.ClassifyBoxes(pose, {invalid}, output).IsOk() && output[0].type == 99, "invalid box atomic");
        invalid = Box(1, 0);
        invalid.yaw = std::numeric_limits<double>::quiet_NaN();
        Check(!server.ClassifyBoxes(pose, {invalid}, output).IsOk(), "NaN yaw rejected");
        auto bad_pose = pose;
        bad_pose.z = std::numeric_limits<double>::infinity();
        Check(!server.ClassifyBoxes(bad_pose, {}, output).IsOk(), "invalid height rejected");
        Good(server.ClassifyBoxes(pose, {}, output));
        Check(output.empty(), "empty perception clears result");
        Check(!server.LoadMap(dir.name + "/missing").IsOk() && server.IsLoaded(), "failed reload retains map");
        Match(server, pose, Box(30, 0), 0);
        hdmap::LANE_OPTIONS_S options;
        options.lane_width = 0;
        Check(!server.LoadMap(dir.name, options).IsOk(), "invalid lane width");
        std::ofstream(dir.Add("broken")) << "not a processed map\n";
        Check(!server.LoadMap(dir.name).IsOk(), "corrupt map rejected atomically");
        Match(server, pose, Box(30, 4), 1);
        hdmap::LaneMapServer empty;
        Check(!empty.ClassifyBoxes(pose, {}, output).IsOk(), "unloaded map rejected");
    }
    void TestRotationAndCurve() {
        for(int degree = 0; degree < 360; degree += 45) {
            double angle = degree * PI / 180;
            Directory dir;
            Straight(dir, "current", 0, angle, 1000000);
            Straight(dir, "left", 4, angle, 1000000);
            hdmap::LaneMapServer server;
            Good(server.LoadMap(dir.name));
            auto pose = Pose(1000000 + 10 * std::cos(angle), 1000000 + 10 * std::sin(angle),
                             hdmap::Coordinate::YawToHeading(angle));
            for(int lane = 0; lane < 2; ++lane) {
                auto box = Box(1000000 + 30 * std::cos(angle) - lane * 4 * std::sin(angle),
                               1000000 + 30 * std::sin(angle) + lane * 4 * std::cos(angle), 5, 2, angle);
                Near(Match(server, pose, box, lane).overlap_area, 10, 0.001, "rotated area large coordinates");
            }
        }
        Directory dir;
        for(int lane = 0; lane < 3; ++lane) {
            hdmap::MapPointList points;
            double radius = 30 - 4 * lane;
            for(int i = 0; i <= 200; ++i) {
                double angle = i * PI / 200;
                points.push_back(hdmap::MAP_POINT_S(radius * std::cos(angle), radius * std::sin(angle),
                                                    hdmap::Coordinate::YawToHeading(angle + PI / 2)));
            }
            Save(dir, std::to_string(lane), points);
        }
        hdmap::LaneMapServer server;
        Good(server.LoadMap(dir.name));
        auto pose = Pose(30, 0, 0);
        for(int lane = 0; lane < 3; ++lane) {
            double angle = PI * 0.75, radius = 30 - 4 * lane;
            Match(server, pose, Box(radius * std::cos(angle), radius * std::sin(angle), 2, 1, angle + PI / 2), lane);
        }
        Directory closed;
        hdmap::MapPointList ring;
        for(int i = 0; i <= 240; ++i) {
            double angle = i * 2 * PI / 240;
            ring.push_back(hdmap::MAP_POINT_S(20 * std::cos(angle), 20 * std::sin(angle),
                                              hdmap::Coordinate::YawToHeading(angle + PI / 2)));
        }
        Save(closed, "circle", ring);
        Good(server.LoadMap(closed.name));
        Match(server, Pose(0, 0), Box(20, 0), 3);     // 环形车道内洞不属于车道；轴线上统一左。
        Match(server, Pose(20, 0, 0), Box(0, 0), 3);  // 内洞没有覆盖面积。
        Near(Match(server, Pose(20, 0, 0), Box(20, 0, 1, 1), 0).overlap_area, 1, 0.001, "closed seam filled");
    }
}

int main() {
    try {
        TestStraightAndMaximumArea();
        TestRotationAndCurve();
        std::printf("PASS: %d lane checks\n", checks);
    } catch(const std::exception& e) {
        std::fprintf(stderr, "FAIL after %d checks: %s\n", checks, e.what());
        return 1;
    }
    return 0;
}
