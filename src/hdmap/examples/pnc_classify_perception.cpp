#include <robot/navigation_msg.h>
#include <robot/perception.h>
#include "hdmap/lane_map_server.h"
#include <cstdio>

// 示例命令：pnc_classify_perception /absolute/path/map_processed xAxis yAxis heading
// 正式接入时把 server 放在业务类成员中，LoadMap 放初始化阶段，不能在感知回调内重载。
int main(int argc, char** argv) {
    if(argc != 5) {
        std::fprintf(stderr, "usage: %s map_processed xAxis yAxis heading\n", argv[0]);
        return 2;
    }
    hdmap::LaneMapServer server;
    hdmap::STATUS_S status = server.LoadMap(argv[1]);
    if(!status.IsOk()) {
        std::fprintf(stderr, "%s\n", status.message.c_str());
        return 1;
    }
    try {
        robot::navigation_msg navigation;
        navigation.xAxis = std::stof(argv[2]);
        navigation.yAxis = std::stof(argv[3]);
        navigation.heading = std::stof(argv[4]);
        robot::perception perception;
        robot::object obj;
        obj.x = navigation.xAxis;
        obj.y = navigation.yAxis;
        obj.dx = 4;
        obj.dy = 2;
        obj.heading = hdmap::Coordinate::NormalizeHeading(navigation.heading + 90);
        perception.objs.push_back(obj);
        robot::perception result = server.ClassifyPerception(navigation, perception);
        // type: 0 本车道，1 左一，2 左二，3 左侧车道外，4 右侧车道外。
        // 自车离道时只返回 3/4；其他消息字段和障碍物顺序原样保留。
        std::printf("type=%u\n", static_cast<unsigned>(result.objs[0].type));
    } catch(const std::exception& error) {
        std::fprintf(stderr, "车道判断失败：%s\n", error.what());
        return 1;
    }
    return 0;
}
