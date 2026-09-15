#ifndef HDMAP_LANE_MAP_SERVER_H
#define HDMAP_LANE_MAP_SERVER_H

#include "hdmap/lane_types.h"
#include "hdmap/coordinate.h"
#include "hdmap/export.h"
#include <memory>
#include <stdexcept>

namespace hdmap {

    // 持久地图服务：启动时 LoadMap 一次，随后每帧只做内存查询，不扫描或读取 CSV。
    // 几何实现在 .so 内；模板仅负责 robot 消息适配，SDK 无 ROS 运行时依赖。
    class HDMAP_API LaneMapServer {
    public:
        LaneMapServer();
        ~LaneMapServer();
        STATUS_S LoadMap(const std::string& tDirectory,
                         const LANE_OPTIONS_S& tOptions = LANE_OPTIONS_S());
        bool IsLoaded() const;
        LANE_MAP_INFO_S GetMapInfo() const;
        std::vector<std::string> GetLaneNames() const;
        // 失败时保持 tOutput 原值；地图重载失败也保留上一次成功加载的地图。
        STATUS_S ClassifyBoxes(const VEHICLE_POSE_S& tPose,
                               const std::vector<OBSTACLE_BOX_S>& tBoxes,
                               std::vector<LANE_MATCH_S>& tOutput) const;

        // PNC 调用：robot::perception result = server.ClassifyPerception(nav, perception);
        // TNavigation = robot::navigation_msg，TPerception = robot::perception。
        // 只修改返回副本的 objs[i].type（0 本车道/1 左一/2 左二/3 左外/4 右外）。
        // 地图未加载或输入非法时抛出 std::runtime_error，调用方应捕获并处理该帧失败。
        template <typename TNavigation, typename TPerception>
        TPerception ClassifyPerception(const TNavigation& tNavigation,
                                       const TPerception& tPerception) const {
            VEHICLE_POSE_S pose;
            pose.x = tNavigation.xAxis;
            pose.y = tNavigation.yAxis;
            pose.z = tNavigation.zAxis;
            pose.heading = tNavigation.heading;
            std::vector<OBSTACLE_BOX_S> boxes(tPerception.objs.size());
            const double ego_heading = Coordinate::NormalizeHeading(pose.heading);
            for(std::size_t i = 0; i < boxes.size(); ++i) {
                boxes[i].x = tPerception.objs[i].x;
                boxes[i].y = tPerception.objs[i].y;
                boxes[i].dx = tPerception.objs[i].dx;
                boxes[i].dy = tPerception.objs[i].dy;
                // perception_msg_convert.cpp: Hobj = yaw_local_deg + Hego + 90。
                // 世界角 yaw_world = (90-Hego) + yaw_local = Hobj - 2*Hego。
                // obj.x/y 已经转换到地图坐标，不能再乘车体旋转或再次平移。
                boxes[i].yaw = Coordinate::NormalizeHeading(
                                   tPerception.objs[i].heading - 2.0 * ego_heading) *
                               3.14159265358979323846 / 180.0;
            }
            std::vector<LANE_MATCH_S> matches;
            STATUS_S status = ClassifyBoxes(pose, boxes, matches);
            if(!status.IsOk()) {
                throw std::runtime_error(status.message);
            }
            TPerception result = tPerception;
            for(std::size_t i = 0; i < matches.size(); ++i) {
                result.objs[i].type = matches[i].type;
            }
            return result;
        }

    private:
        struct MapData;
        std::shared_ptr<const MapData> mMap;
    };

}
#endif
