#ifndef HDMAP_LANE_TYPES_H
#define HDMAP_LANE_TYPES_H

#include "hdmap/hdmap_types.h"
#include <cstdint>

namespace hdmap {

    // ClassifyPerception 返回消息的 objs[i].type 含义；不是输入消息原有的物体类别。
    // 正负仅用于横向距离，type 保持与 robot::object 的 uint8 字段兼容。
    enum LANE_TYPE_E {
        LANE_CURRENT = 0,       // 障碍物覆盖面积最大的车道为本车道。
        LANE_LEFT_ONE = 1,      // 障碍物覆盖面积最大的车道为左侧第 1 车道。
        LANE_LEFT_TWO = 2,      // 障碍物覆盖面积最大的车道为左侧第 2 车道。
        LANE_OUTSIDE_LEFT = 3,  // 自车左侧，未占据当前道路的三条候选车道。
        LANE_OUTSIDE_RIGHT = 4  // 自车右侧，未占据当前道路的三条候选车道。
    };

    typedef struct lane_options_s {
        lane_options_s()
            : lane_width(4.0), grid_size(8.0), neighbor_distance(12.0), parallel_angle_deg(45.0) {
        }
        double lane_width;          // 暂按中心线左右各 2 米，可按地图实际情况配置。
        double grid_size;           // 地图面积分块边长，米；仅影响缓存大小与查询性能。
        double neighbor_distance;   // 查询左侧中心线的最大横向距离，米。
        double parallel_angle_deg;  // 邻道与本车道的最大夹角；对向平行车道也计入左侧序号。
    } LANE_OPTIONS_S;

    typedef struct vehicle_pose_s {
        vehicle_pose_s()
            : x(0), y(0), z(0), heading(0) {
        }
        double x;
        double y;
        double z;        // 保留定位高度；当前中心线没有高程，判断仅使用二维地面投影。
        double heading;  // PNC 方位角：北 0 度、顺时针为正。
    } VEHICLE_POSE_S;

    typedef struct obstacle_box_s {
        obstacle_box_s()
            : x(0), y(0), dx(0), dy(0), yaw(0) {
        }
        double x;
        double y;
        double dx;   // 框局部 x 轴完整尺寸，米，必须 > 0。
        double dy;   // 框局部 y 轴完整尺寸，米，必须 > 0。
        double yaw;  // 框局部 x 轴在地图中的数学角，弧度；东 0、逆时针为正。
    } OBSTACLE_BOX_S;

    // 可选诊断结果，便于核对分类和基准测试；perception 接口只回写 type。
    typedef struct lane_match_s {
        lane_match_s()
            : type(LANE_OUTSIDE_LEFT), lane_index(-1), overlap_area(0) {
        }
        std::uint8_t type;
        int lane_index;       // LoadMap 后 GetLaneNames 的下标；车道外为 -1。
        double overlap_area;  // 与被选车道的真实投影重叠面积，平方米。
    } LANE_MATCH_S;

    typedef struct lane_map_info_s {
        lane_map_info_s()
            : lane_count(0), segment_count(0), cell_count(0), vertex_count(0) {
        }
        std::size_t lane_count;
        std::size_t segment_count;
        std::size_t cell_count;
        std::size_t vertex_count;
    } LANE_MAP_INFO_S;

}
#endif
