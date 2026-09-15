#include "hdmap/lane_map_server.h"
#include "hdmap/hdmap_server.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sys/resource.h>
#ifdef HDMAP_BENCH_ROBOT
#include <robot/navigation_msg.h>
#include <robot/perception.h>
#endif

int main(int argc, char** argv) {
    if(argc != 2) {
        return 2;
    }
    typedef std::chrono::steady_clock Clock;
    hdmap::LaneMapServer server;
    struct rusage initial_usage;
    getrusage(RUSAGE_SELF, &initial_usage);
    auto begin = Clock::now();
    auto status = server.LoadMap(argv[1]);
    if(!status.IsOk()) {
        std::fprintf(stderr, "%s\n", status.message.c_str());
        return 1;
    }
    double load_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    auto info = server.GetMapInfo();
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    std::printf("load_ms=%.3f lanes=%zu segments=%zu cells=%zu vertices=%zu peak_rss_kib=%ld\n",
                load_ms, info.lane_count, info.segment_count, info.cell_count, info.vertex_count, usage.ru_maxrss);
    std::ifstream memory("/proc/self/status");
    std::string line;
    while(std::getline(memory, line)) {
        if(line.compare(0, 6, "VmRSS:") == 0) {
            std::printf("%s initial_peak_rss_kib=%ld\n", line.c_str(), initial_usage.ru_maxrss);
        }
    }
    hdmap::MapPointList path;
    hdmap::DIRECTION_E direction;
    status = hdmap::HdMapServer().LoadProcessedTrajectory(std::string(argv[1]) + "/lane_S2N_spline.csv", path, direction);
    if(!status.IsOk()) {
        std::fprintf(stderr, "%s\n", status.message.c_str());
        return 1;
    }
    unsigned long checksum = 0;
    const int counts[] = {1, 10, 100, 300};
    for(std::size_t c = 0; c < 4; ++c) {
        std::vector<double> times;
        for(int frame = 0; frame < 320; ++frame) {
            const auto& point = path[100 + (frame * 37) % (path.size() - 300)];
            hdmap::VEHICLE_POSE_S pose;
            pose.x = point.x_axis;
            pose.y = point.y_axis;
            pose.heading = point.heading;
            double yaw = hdmap::Coordinate::HeadingToYaw(pose.heading);
            std::vector<hdmap::OBSTACLE_BOX_S> boxes(counts[c]);
            for(std::size_t i = 0; i < boxes.size(); ++i) {
                double along = 5 + (i % 20) * 2.5, lateral = (static_cast<int>(i % 7) - 2) * 3.8;
                boxes[i].x = pose.x + along * std::cos(yaw) - lateral * std::sin(yaw);
                boxes[i].y = pose.y + along * std::sin(yaw) + lateral * std::cos(yaw);
                boxes[i].dx = 4.5;
                boxes[i].dy = 2;
                boxes[i].yaw = yaw + (i % 3) * 0.2;
            }
#ifdef HDMAP_BENCH_ROBOT
            robot::navigation_msg navigation;
            navigation.xAxis = pose.x;
            navigation.yAxis = pose.y;
            navigation.heading = pose.heading;
            robot::perception perception;
            perception.objs.resize(boxes.size());
            for(std::size_t i = 0; i < boxes.size(); ++i) {
                perception.objs[i].x = boxes[i].x;
                perception.objs[i].y = boxes[i].y;
                perception.objs[i].dx = boxes[i].dx;
                perception.objs[i].dy = boxes[i].dy;
                perception.objs[i].heading = hdmap::Coordinate::NormalizeHeading(
                    boxes[i].yaw * 180.0 / 3.14159265358979323846 + 2 * navigation.heading);
            }
            begin = Clock::now();
            robot::perception result = server.ClassifyPerception(navigation, perception);
            double elapsed = std::chrono::duration<double, std::micro>(Clock::now() - begin).count();
            for(std::size_t i = 0; i < result.objs.size(); ++i) {
                checksum += result.objs[i].type;
            }
#else
            std::vector<hdmap::LANE_MATCH_S> matches;
            begin = Clock::now();
            status = server.ClassifyBoxes(pose, boxes, matches);
            double elapsed = std::chrono::duration<double, std::micro>(Clock::now() - begin).count();
            if(!status.IsOk()) {
                std::fprintf(stderr, "%s\n", status.message.c_str());
                return 1;
            }
            for(std::size_t i = 0; i < matches.size(); ++i) {
                checksum += matches[i].type;
            }
#endif
            if(frame >= 20) {
                times.push_back(elapsed);
            }
        }
        std::sort(times.begin(), times.end());
        std::printf("objects=%d frames=%zu p50_us=%.3f p95_us=%.3f p99_us=%.3f max_us=%.3f\n",
                    counts[c], times.size(), times[times.size() / 2], times[times.size() * 95 / 100],
                    times[times.size() * 99 / 100], times.back());
    }
    std::printf("checksum=%lu\n", checksum);
    return 0;
}
