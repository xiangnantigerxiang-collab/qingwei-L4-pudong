#include <robot/navigation_msg.h>
#include <robot/perception.h>
#include "hdmap/lane_map_server.h"
#include "hdmap/hdmap_server.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <unistd.h>

// robot 类型来自真实 .msg 的生成头文件；本机无 ROS 时由仓库既有生成器生成桩。
int main() {
    const double pi = 3.14159265358979323846;
    char directory[] = "/tmp/hdmap_perception_XXXXXX";
    if(!mkdtemp(directory)) {
        return 1;
    }
    std::string file = std::string(directory) + "/lane.csv";
    try {
        hdmap::MapPointList points;
        for(int i = 0; i <= 200; ++i) {
            points.push_back(hdmap::MAP_POINT_S(i * 0.5, 0, 90));
            points.back().dist_origin = i * 0.5;
            points.back().p2pDistance = i == 0 ? 0 : 0.5;
        }
        auto status = hdmap::HdMapServer().SaveTrajectory(file, points, hdmap::DIRECTION_FORWARD);
        if(!status.IsOk()) {
            throw std::runtime_error(status.message);
        }
        hdmap::LaneMapServer server;
        status = server.LoadMap(directory);
        if(!status.IsOk()) {
            throw std::runtime_error(status.message);
        }
        std::size_t checks = 0;
        for(int degree = 0; degree < 360; degree += 15) {
            robot::navigation_msg nav;
            nav.xAxis = 10;
            nav.yAxis = 0;
            nav.zAxis = 7;
            nav.heading = degree;
            robot::perception perception;
            perception.header.frame_id = "map";
            perception.header.seq = 123;
            perception.header.stamp = ros::Time(12.5);
            hdmap::VEHICLE_POSE_S pose;
            pose.x = nav.xAxis;
            pose.y = nav.yAxis;
            pose.z = nav.zAxis;
            pose.heading = nav.heading;
            std::vector<hdmap::OBSTACLE_BOX_S> expected_boxes;
            for(int angle = -180; angle <= 180; angle += 15) {
                robot::object obj;
                obj.id = angle;
                obj.type = 99;
                obj.x = 101;
                obj.y = static_cast<float>((angle / 15) % 4);
                obj.dx = 5;
                obj.dy = 0.4f;
                obj.height = 3;
                obj.vx = 2;
                obj.vy = 1;
                obj.polygons.resize(1);
                obj.polygons[0].z = 42;
                // 按原转换节点产生消息，而期望框直接用刚体旋转组合计算世界 yaw。
                obj.heading = static_cast<float>(std::fmod(angle + nav.heading + 450.0, 360.0));
                perception.objs.push_back(obj);
                hdmap::OBSTACLE_BOX_S box;
                box.x = obj.x;
                box.y = obj.y;
                box.dx = obj.dx;
                box.dy = obj.dy;
                box.yaw = (90 - nav.heading) * pi / 180 + angle * pi / 180;
                expected_boxes.push_back(box);
            }
            std::vector<hdmap::LANE_MATCH_S> expected;
            status = server.ClassifyBoxes(pose, expected_boxes, expected);
            if(!status.IsOk()) {
                throw std::runtime_error(status.message);
            }
            robot::perception result = server.ClassifyPerception(nav, perception);
            if(result.objs.size() != perception.objs.size() || result.header.seq != 123 ||
               result.header.frame_id != "map" || result.header.stamp.toSec() != 12.5) {
                throw std::runtime_error("message envelope not preserved");
            }
            for(std::size_t i = 0; i < result.objs.size(); ++i) {
                const auto& a = result.objs[i];
                const auto& b = perception.objs[i];
                if(a.type != expected[i].type || b.type != 99 || a.id != b.id || a.x != b.x || a.y != b.y ||
                   a.dx != b.dx || a.dy != b.dy || a.heading != b.heading || a.height != b.height ||
                   a.vx != b.vx || a.vy != b.vy || a.polygons.size() != 1 || a.polygons[0].z != 42) {
                    throw std::runtime_error("perception conversion or field preservation failed");
                }
                ++checks;
            }
            robot::perception empty;
            if(!server.ClassifyPerception(nav, empty).objs.empty()) {
                throw std::runtime_error("empty frame");
            }
            perception.objs[0].heading = std::numeric_limits<float>::quiet_NaN();
            bool rejected = false;
            try {
                server.ClassifyPerception(nav, perception);
            } catch(const std::runtime_error&) {
                rejected = true;
            }
            if(!rejected || perception.objs[0].type != 99) {
                throw std::runtime_error("invalid input contract");
            }
        }
        std::printf("PASS: %zu real-message adapter cases, header and all object fields preserved\n", checks);
    } catch(const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        std::remove(file.c_str());
        rmdir(directory);
        return 1;
    }
    std::remove(file.c_str());
    rmdir(directory);
    return 0;
}
