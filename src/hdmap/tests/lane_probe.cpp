#include "hdmap/lane_map_server.h"
#include <iomanip>
#include <iostream>

// 独立验证桥：每帧输入 x y z heading count，随后 count 行 x y dx dy yaw。
// 不读取 ROS 录包，输出真实 .so 的 type/lane_index/area，供外部几何实现对照。
int main(int argc, char** argv) {
    if(argc < 2 || argc > 3) {
        return 2;
    }
    hdmap::LaneMapServer server;
    hdmap::LANE_OPTIONS_S options;
    if(argc == 3) {
        options.grid_size = std::stod(argv[2]);
    }
    auto status = server.LoadMap(argv[1], options);
    if(!status.IsOk()) {
        std::cerr << status.message << '\n';
        return 1;
    }
    hdmap::VEHICLE_POSE_S pose;
    std::size_t count = 0, frame = 0;
    std::cout << std::setprecision(17);
    while(std::cin >> pose.x >> pose.y >> pose.z >> pose.heading >> count) {
        if(count > 100000) {
            return 2;
        }
        std::vector<hdmap::OBSTACLE_BOX_S> boxes(count);
        for(std::size_t i = 0; i < count; ++i) {
            if(!(std::cin >> boxes[i].x >> boxes[i].y >> boxes[i].dx >> boxes[i].dy >> boxes[i].yaw)) {
                return 2;
            }
        }
        std::vector<hdmap::LANE_MATCH_S> matches;
        status = server.ClassifyBoxes(pose, boxes, matches);
        if(!status.IsOk()) {
            std::cerr << status.message << '\n';
            return 1;
        }
        for(std::size_t i = 0; i < matches.size(); ++i) {
            std::cout << frame << ',' << i << ',' << static_cast<int>(matches[i].type) << ','
                      << matches[i].lane_index << ',' << matches[i].overlap_area << '\n';
        }
        ++frame;
    }
    return std::cin.eof() ? 0 : 2;
}
