// 仅由 perception_msg_convert.cpp 包含；使用现有 SDK，不修改 HDMap 的通用分类规则。
#include <cmath>
#include <stdexcept>

namespace perception_lane_detail {
    // 20% 指交面积 / 障碍框自身面积，不是 IoU，也不是占整条车道的比例。
    const double MIN_CURRENT_LANE_RATIO = 0.20;
    const double OVERLAP_RATIO_EPSILON = 1e-9;

    void SaveWorldCorners(robot::object& tObject, double tWorldYaw) {
        // polygons 仅在本节点的原始历史中保存四角，随最新观测一起经过时序滤波。
        // 发布前清除这份内部几何；运动 heading 与检测框的物理朝向相互独立。
        const double half_x = tObject.dx * 0.5;
        const double half_y = tObject.dy * 0.5;
        const double local_x[] = {-half_x, half_x, half_x, -half_x};
        const double local_y[] = {-half_y, -half_y, half_y, half_y};
        const double c = std::cos(tWorldYaw);
        const double s = std::sin(tWorldYaw);
        tObject.polygons.resize(4);
        for(std::size_t i = 0; i < 4; ++i) {
            tObject.polygons[i].x = tObject.x + local_x[i] * c - local_y[i] * s;
            tObject.polygons[i].y = tObject.y + local_x[i] * s + local_y[i] * c;
            tObject.polygons[i].z = 0;
        }
    }

    void ClassifyLaneTypes(const hdmap::LaneMapServer& tLaneMap, const robot::navigation_msg& tNavigation,
                           robot::perception& tPerception) {
        hdmap::VEHICLE_POSE_S pose;
        pose.x = tNavigation.xAxis;
        pose.y = tNavigation.yAxis;
        pose.z = tNavigation.zAxis;
        pose.heading = tNavigation.heading;
        std::vector<hdmap::OBSTACLE_BOX_S> boxes(tPerception.objs.size());
        for(std::size_t i = 0; i < boxes.size(); ++i) {
            const robot::object& object = tPerception.objs[i];
            if(object.polygons.size() != 4) {
                throw std::runtime_error("Missing historical obstacle world corners");
            }
            for(std::size_t j = 0; j < 4; ++j) {
                if(!std::isfinite(object.polygons[j].x) || !std::isfinite(object.polygons[j].y)) {
                    throw std::runtime_error("Invalid historical obstacle world corners");
                }
            }
            const double edge_x = object.polygons[1].x - object.polygons[0].x;
            const double edge_y = object.polygons[1].y - object.polygons[0].y;
            if(edge_x == 0 && edge_y == 0) {
                throw std::runtime_error("Degenerate historical obstacle world corners");
            }
            boxes[i].x = object.x;
            boxes[i].y = object.y;
            boxes[i].dx = object.dx;
            boxes[i].dy = object.dy;
            // obj.heading 现在是运动航向，框朝向始终由采集时保存的世界角点确定。
            boxes[i].yaw = std::atan2(edge_y, edge_x);
        }
        std::vector<hdmap::LANE_MATCH_S> matches;
        const hdmap::STATUS_S status = tLaneMap.ClassifyBoxes(pose, boxes, matches);
        if(!status.IsOk()) {
            throw std::runtime_error(status.message);
        }
        const double yaw = hdmap::Coordinate::HeadingToYaw(hdmap::Coordinate::NormalizeHeading(pose.heading));
        for(std::size_t i = 0; i < matches.size(); ++i) {
            tPerception.objs[i].type = matches[i].type;
            const double area = boxes[i].dx * boxes[i].dy;
            if(matches[i].type == hdmap::LANE_CURRENT &&
               matches[i].overlap_area / area + OVERLAP_RATIO_EPSILON < MIN_CURRENT_LANE_RATIO) {
                // 只否决不足 20% 的本车道判定；其余车道仍采用 SDK 结果。
                // 左右按当前自车朝向判断，与 SDK 的车道外约定一致。
                const double lateral = -(boxes[i].x - pose.x) * std::sin(yaw) +
                                        (boxes[i].y - pose.y) * std::cos(yaw);
                tPerception.objs[i].type = lateral >= -1e-10 ? hdmap::LANE_OUTSIDE_LEFT : hdmap::LANE_OUTSIDE_RIGHT;
            }
        }
    }
}
