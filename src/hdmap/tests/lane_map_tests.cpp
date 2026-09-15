#include "hdmap/lane_map_server.h"
#include "hdmap/hdmap_server.h"
#include <cstdio>

// 遍历真实地图全部中心线点，专门检查分段接缝、端点和重叠区域的自车归属。
int main(int argc, char** argv) {
    if(argc != 2) {
        return 2;
    }
    hdmap::LaneMapServer server;
    auto status = server.LoadMap(argv[1]);
    if(!status.IsOk()) {
        std::fprintf(stderr, "%s\n", status.message.c_str());
        return 1;
    }
    std::size_t checks = 0;
    auto names = server.GetLaneNames();
    for(std::size_t lane = 0; lane < names.size(); ++lane) {
        hdmap::MapPointList points;
        hdmap::DIRECTION_E direction;
        status = hdmap::HdMapServer().LoadProcessedTrajectory(std::string(argv[1]) + "/" + names[lane], points, direction);
        if(!status.IsOk()) {
            return 1;
        }
        for(std::size_t i = 0; i < points.size(); ++i) {
            hdmap::VEHICLE_POSE_S pose;
            pose.x = points[i].x_axis;
            pose.y = points[i].y_axis;
            pose.heading = points[i].heading;
            hdmap::OBSTACLE_BOX_S box;
            box.x = pose.x;
            box.y = pose.y;
            box.dx = 0.1;
            box.dy = 0.1;
            box.yaw = hdmap::Coordinate::HeadingToYaw(pose.heading);
            std::vector<hdmap::LANE_MATCH_S> result;
            status = server.ClassifyBoxes(pose, {box}, result);
            if(!status.IsOk() || result.size() != 1 || result[0].type != hdmap::LANE_CURRENT) {
                std::fprintf(stderr, "self lane failure %s point %zu: %s\n", names[lane].c_str(), i, status.message.c_str());
                return 1;
            }
            ++checks;
        }
    }
    std::printf("PASS: %zu real-map centerline positions\n", checks);
    return 0;
}
