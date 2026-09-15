#include "hdmap/lane_map_server.h"
#include "trajectory_csv.h"
#include "lane_geometry.h"
#include <algorithm>
#include <cmath>
#include <set>

namespace hdmap {
    namespace {
        using detail::Vec2;
        using detail::Aabb;
        using detail::Ring;
        using detail::Segment;
        using detail::Projection;
        using detail::AabbIndex;
        using detail::Dot;
        using detail::Left;

        struct LaneData {
            std::string name;
            double max_half_width;
            std::vector<Segment> segments;
            std::vector<Ring> rings;
            AabbIndex index;
        };
        struct LaneCell {
            int lane;
            Aabb bounds;
            std::vector<Ring> rings;
        };

        bool FinitePosition(double tValue) {
            return std::isfinite(tValue) && std::abs(tValue) <= 1e8;
        }
        std::array<Vec2, 4> BoundsCorners(const Aabb& b) {
            return {{b.min, Vec2(b.max.x, b.min.y), b.max, Vec2(b.min.x, b.max.y)}};
        }
        STATUS_S BuildCells(const LaneData& tLane, int tLaneIndex, const LANE_OPTIONS_S& tOptions,
                            std::vector<LaneCell>& tCells) {
            std::set<std::pair<int, int>> active;
            double size = tOptions.grid_size;
            for(std::size_t i = 0; i < tLane.segments.size(); ++i) {
                Aabb bounds(tLane.segments[i].a, tLane.segments[i].b);
                // 最大斜接长度为 lane_width，覆盖全部折点，不会漏掉弯道外侧。
                bounds.min = bounds.min - Vec2(tOptions.lane_width, tOptions.lane_width);
                bounds.max = bounds.max + Vec2(tOptions.lane_width, tOptions.lane_width);
                int x0 = static_cast<int>(std::floor(bounds.min.x / size));
                int x1 = static_cast<int>(std::floor(bounds.max.x / size));
                int y0 = static_cast<int>(std::floor(bounds.min.y / size));
                int y1 = static_cast<int>(std::floor(bounds.max.y / size));
                if(static_cast<double>(x1 - x0 + 1) * (y1 - y0 + 1) > 2000000) {
                    return STATUS_S(INVALID_DATA, "单段地图跨度过大，无法建立车道缓存");
                }
                for(int x = x0; x <= x1; ++x) {
                    for(int y = y0; y <= y1; ++y) {
                        active.insert(std::make_pair(x, y));
                    }
                }
                if(active.size() > 2000000) {
                    return STATUS_S(INVALID_DATA, "车道缓存超过 200 万块");
                }
            }
            for(auto it = active.begin(); it != active.end(); ++it) {
                LaneCell cell;
                cell.lane = tLaneIndex;
                cell.bounds = Aabb(Vec2(it->first * size, it->second * size),
                                   Vec2((it->first + 1.0) * size, (it->second + 1.0) * size));
                double area = 0;
                for(std::size_t r = 0; r < tLane.rings.size(); ++r) {
                    Ring ring;
                    ring.sign = tLane.rings[r].sign;
                    ring.points = detail::ClipPolygon(tLane.rings[r].points, BoundsCorners(cell.bounds));
                    double ring_area = detail::PolygonArea(ring.points);
                    if(ring_area > 1e-12) {
                        area += ring.sign * ring_area;
                        cell.rings.push_back(std::move(ring));
                    }
                }
                // 栅格块内部互不重叠，运行时各块交面积可直接相加。
                if(area > 1e-10) {
                    tCells.push_back(std::move(cell));
                }
            }
            return STATUS_S();
        }
    }

    struct LaneMapServer::MapData {
        LANE_OPTIONS_S options;
        LANE_MAP_INFO_S info;
        std::vector<LaneData> lanes;
        std::vector<Segment> segments;
        std::vector<LaneCell> cells;
        AabbIndex segment_index, cell_index;

        int FindEgoLane(const Vec2& tPosition, const Vec2& tForward, Projection& tProjection) const;
        std::array<int, 3> FindRelativeLanes(int tEgoLane, const Projection& tReference,
                                             double tDirection) const;
    };

    LaneMapServer::LaneMapServer() {
    }
    LaneMapServer::~LaneMapServer() {
    }
    bool LaneMapServer::IsLoaded() const {
        return static_cast<bool>(mMap);
    }
    LANE_MAP_INFO_S LaneMapServer::GetMapInfo() const {
        return mMap ? mMap->info : LANE_MAP_INFO_S();
    }
    std::vector<std::string> LaneMapServer::GetLaneNames() const {
        std::vector<std::string> names;
        if(mMap) {
            for(std::size_t i = 0; i < mMap->lanes.size(); ++i) {
                names.push_back(mMap->lanes[i].name);
            }
        }
        return names;
    }

    STATUS_S LaneMapServer::LoadMap(const std::string& tDirectory, const LANE_OPTIONS_S& tOptions) {
        if(!std::isfinite(tOptions.lane_width) || tOptions.lane_width < 0.1 || tOptions.lane_width > 100 ||
           !std::isfinite(tOptions.grid_size) || tOptions.grid_size < 1 || tOptions.grid_size > 128 ||
           !std::isfinite(tOptions.neighbor_distance) || tOptions.neighbor_distance < tOptions.lane_width ||
           tOptions.neighbor_distance > 1000 || !std::isfinite(tOptions.parallel_angle_deg) ||
           tOptions.parallel_angle_deg <= 0 || tOptions.parallel_angle_deg >= 90) {
            return STATUS_S(INVALID_ARGUMENT, "车道宽度、分块大小、邻道搜索范围或夹角参数非法");
        }
        std::vector<std::string> files;
        STATUS_S status = detail::ListCsvFiles(tDirectory, files);
        if(!status.IsOk()) {
            return status;
        }
        std::shared_ptr<MapData> map(new MapData);
        map->options = tOptions;
        std::vector<Aabb> segment_bounds, cell_bounds;
        for(std::size_t i = 0; i < files.size(); ++i) {
            MapPointList points;
            DIRECTION_E direction = DIRECTION_AUTO;
            status = detail::TrajectoryCsv::Read(files[i], true, points, direction);
            if(!status.IsOk()) {
                return status;
            }
            if(points.size() < 2) {
                return STATUS_S(INVALID_DATA, files[i] + ": 中心线不足两个点");
            }
            LaneData lane;
            lane.name = files[i].substr(files[i].find_last_of('/') + 1);
            std::vector<Aabb> lane_bounds;
            for(std::size_t j = 0; j < points.size(); ++j) {
                if(!FinitePosition(points[j].x_axis) || !FinitePosition(points[j].y_axis) ||
                   std::abs(points[j].x_axis - points[0].x_axis) > 100000 ||
                   std::abs(points[j].y_axis - points[0].y_axis) > 100000) {
                    return STATUS_S(INVALID_DATA, files[i] + ": 坐标范围超出车道计算支持范围");
                }
                if(j == 0) {
                    continue;
                }
                Vec2 a(points[j - 1].x_axis, points[j - 1].y_axis), b(points[j].x_axis, points[j].y_axis);
                if(std::hypot(b.x - a.x, b.y - a.y) < 1e-6) {
                    return STATUS_S(INVALID_DATA, files[i] + ": 中心线包含重合点");
                }
                lane.segments.push_back(Segment(a, b, static_cast<int>(i)));
                lane_bounds.push_back(Aabb(a, b));
            }
            lane.index.Build(lane_bounds);
            lane.max_half_width = tOptions.lane_width * 0.5;
            for(std::size_t j = 0; j < lane.segments.size(); ++j) {
                const Segment& previous = j == 0 ? lane.segments.back() : lane.segments[j - 1];
                double denominator = 1.0 + Dot(previous.tangent, lane.segments[j].tangent);
                if(denominator <= 1e-8) {
                    continue;
                }
                double miter_length = tOptions.lane_width / std::sqrt(2 * denominator);
                if(miter_length <= tOptions.lane_width) {
                    lane.max_half_width = std::max(lane.max_half_width, miter_length);
                }
            }
            lane.max_half_width += 1e-4;  // 包含定点化和坐标转换的误差余量。
            status = detail::BuildLaneRings(lane.segments, tOptions.lane_width * 0.5, lane.rings);
            if(!status.IsOk()) {
                return STATUS_S(status.code, files[i] + ": " + status.message);
            }
            status = BuildCells(lane, static_cast<int>(i), tOptions, map->cells);
            if(!status.IsOk()) {
                return STATUS_S(status.code, files[i] + ": " + status.message);
            }
            map->segments.insert(map->segments.end(), lane.segments.begin(), lane.segments.end());
            segment_bounds.insert(segment_bounds.end(), lane_bounds.begin(), lane_bounds.end());
            map->lanes.push_back(std::move(lane));
        }
        for(std::size_t i = 0; i < map->cells.size(); ++i) {
            cell_bounds.push_back(map->cells[i].bounds);
            for(std::size_t j = 0; j < map->cells[i].rings.size(); ++j) {
                map->info.vertex_count += map->cells[i].rings[j].points.size();
            }
        }
        map->segment_index.Build(segment_bounds);
        map->cell_index.Build(cell_bounds);
        map->info.lane_count = map->lanes.size();
        map->info.segment_count = map->segments.size();
        map->info.cell_count = map->cells.size();
        if(map->cells.empty()) {
            return STATUS_S(INVALID_DATA, "地图没有有效车道区域");
        }
        // 所有 CSV、并集和索引都成功后再换入，坏文件不能破坏上一版可用地图。
        mMap = map;
        return STATUS_S();
    }

    int LaneMapServer::MapData::FindEgoLane(const Vec2& tPosition, const Vec2& tForward,
                                            Projection& tProjection) const {
        std::vector<std::size_t> candidates;
        Vec2 epsilon(1e-8, 1e-8);
        cell_index.Query(Aabb(tPosition - epsilon, tPosition + epsilon), candidates);
        std::vector<bool> visited(lanes.size(), false);
        int best = -1;
        double best_alignment = -1;
        for(std::size_t i = 0; i < candidates.size(); ++i) {
            int index = cells[candidates[i]].lane;
            if(visited[index]) {
                continue;
            }
            visited[index] = true;
            Projection projection = lanes[index].index.Nearest(tPosition, lanes[index].segments);
            // 先用实际最大斜接距离排除邻道，避免为远离车道的点遍历完整边界。
            if(projection.distance_squared > lanes[index].max_half_width * lanes[index].max_half_width) {
                continue;
            }
            const Segment& segment = lanes[index].segments[projection.segment];
            double along = Dot(tPosition - segment.a, segment.tangent);
            double interior_width = options.lane_width * 0.5 - 1e-4;
            // 原始线段矩形内部必属车道，直接投影既快也不受缓冲区定点化接缝影响。
            // 边界、拐角和端面再查完整并集，端点之外不会因最近距离小而误当成车道。
            bool inside_strip = along >= -1e-8 && along <= segment.length + 1e-8 &&
                                projection.distance_squared < interior_width * interior_width;
            if(!inside_strip && !detail::Contains(lanes[index].rings, tPosition)) {
                continue;
            }
            double alignment = std::abs(Dot(projection.tangent, tForward));
            if(projection.distance_squared < tProjection.distance_squared - 1e-10 ||
               (std::abs(projection.distance_squared - tProjection.distance_squared) <= 1e-10 &&
                (alignment > best_alignment + 1e-10 ||
                 (std::abs(alignment - best_alignment) <= 1e-10 && index < best)))) {
                best = index;
                tProjection = projection;
                best_alignment = alignment;
            }
        }
        return best;
    }

    std::array<int, 3> LaneMapServer::MapData::FindRelativeLanes(int tEgoLane,
                                                                 const Projection& tReference, double tDirection) const {
        std::array<int, 3> result{{tEgoLane, -1, -1}};
        Vec2 tangent = tReference.tangent * tDirection;
        Vec2 normal = Left(tangent), epsilon(1e-7, 1e-7);
        Aabb bounds(tReference.point, tReference.point + normal * options.neighbor_distance);
        bounds.min = bounds.min - epsilon;
        bounds.max = bounds.max + epsilon;
        std::vector<std::size_t> candidates;
        segment_index.Query(bounds, candidates);
        std::vector<double> distances(lanes.size(), std::numeric_limits<double>::infinity());
        double min_parallel = std::cos(options.parallel_angle_deg * 3.14159265358979323846 / 180.0);
        for(std::size_t i = 0; i < candidates.size(); ++i) {
            const Segment& segment = segments[candidates[i]];
            if(segment.lane == tEgoLane || std::abs(Dot(segment.tangent, tangent)) < min_parallel) {
                continue;
            }
            double start = Dot(segment.a - tReference.point, tangent);
            double end = Dot(segment.b - tReference.point, tangent);
            // 横断面必须穿过真实中心线段，端点之外不做无限延伸，也不把交叉路当邻道。
            if(std::abs(end - start) < 1e-10) {
                continue;
            }
            double ratio = -start / (end - start);
            if(ratio < -1e-8 || ratio > 1 + 1e-8) {
                continue;
            }
            Vec2 crossing = segment.a + (segment.b - segment.a) * std::max(0.0, std::min(1.0, ratio));
            double lateral = Dot(crossing - tReference.point, normal);
            if(lateral > 1e-6 && lateral <= options.neighbor_distance + 1e-8) {
                distances[segment.lane] = std::min(distances[segment.lane], lateral);
            }
        }
        std::vector<std::pair<double, int>> ordered;
        for(std::size_t i = 0; i < distances.size(); ++i) {
            if(std::isfinite(distances[i])) {
                ordered.push_back(std::make_pair(distances[i], static_cast<int>(i)));
            }
        }
        std::sort(ordered.begin(), ordered.end());
        for(std::size_t i = 0; i < ordered.size() && i < 2; ++i) {
            result[i + 1] = ordered[i].second;
        }
        return result;
    }

    STATUS_S LaneMapServer::ClassifyBoxes(const VEHICLE_POSE_S& tPose,
                                          const std::vector<OBSTACLE_BOX_S>& tBoxes, std::vector<LANE_MATCH_S>& tOutput) const {
        if(!mMap) {
            return STATUS_S(INVALID_DATA, "请先调用 LoadMap 加载 map_processed");
        }
        if(!FinitePosition(tPose.x) || !FinitePosition(tPose.y) || !FinitePosition(tPose.z) ||
           !std::isfinite(tPose.heading)) {
            return STATUS_S(INVALID_ARGUMENT, "自车定位包含非法值");
        }
        for(std::size_t i = 0; i < tBoxes.size(); ++i) {
            const OBSTACLE_BOX_S& box = tBoxes[i];
            if(!FinitePosition(box.x) || !FinitePosition(box.y) || !std::isfinite(box.yaw) ||
               !std::isfinite(box.dx) || !std::isfinite(box.dy) || box.dx <= 0 || box.dy <= 0 ||
               box.dx > 10000 || box.dy > 10000) {
                return STATUS_S(INVALID_ARGUMENT, "障碍物 " + std::to_string(i) + " 的位置、尺寸或朝向非法");
            }
        }
        const MapData& map = *mMap;
        std::vector<LANE_MATCH_S> output(tBoxes.size());
        if(tBoxes.empty()) {
            tOutput.swap(output);
            return STATUS_S();
        }
        Vec2 position(tPose.x, tPose.y);
        double yaw = Coordinate::HeadingToYaw(Coordinate::NormalizeHeading(tPose.heading));
        Vec2 forward(std::cos(yaw), std::sin(yaw));
        Projection ego_projection;
        int ego_lane = map.FindEgoLane(position, forward, ego_projection);
        double direction = Dot(ego_projection.tangent, forward) < 0 ? -1.0 : 1.0;
        std::vector<std::size_t> candidates;
        for(std::size_t i = 0; i < tBoxes.size(); ++i) {
            const OBSTACLE_BOX_S& box = tBoxes[i];
            Vec2 center(box.x, box.y);
            // 自车在车道外时，即使障碍物碰到别的地图车道，也只输出相对自车的左右。
            // 恰在前后轴线上时中心横向距离为零，统一归左，保证两类结果穷尽且确定。
            output[i].type = Dot(center - position, Left(forward)) >= -1e-10 ? LANE_OUTSIDE_LEFT : LANE_OUTSIDE_RIGHT;
            if(ego_lane < 0) {
                continue;
            }
            const LaneData& current = map.lanes[ego_lane];
            Projection reference = current.index.Nearest(center, current.segments);
            std::array<int, 3> relatives = map.FindRelativeLanes(ego_lane, reference, direction);
            std::array<Vec2, 4> corners = detail::BoxCorners(box);
            Aabb bounds;
            for(std::size_t c = 0; c < corners.size(); ++c) {
                bounds.Add(corners[c]);
            }
            map.cell_index.Query(bounds, candidates);
            double areas[3] = {0, 0, 0};
            for(std::size_t c = 0; c < candidates.size(); ++c) {
                const LaneCell& cell = map.cells[candidates[c]];
                for(std::size_t k = 0; k < relatives.size(); ++k) {
                    if(cell.lane == relatives[k]) {
                        areas[k] += detail::OverlapArea(cell.rings, corners);
                    }
                }
            }
            // 覆盖最全面 = 与障碍物地面矩形重叠面积最大；同一个障碍物的面积分母相同。
            // 面积相等时选中心线距离更近的车道，再按本车道/左一/左二打破平局。
            double best_distance = std::numeric_limits<double>::infinity();
            double tie = std::max(1e-9, box.dx * box.dy * 1e-8);
            for(std::size_t k = 0; k < relatives.size(); ++k) {
                if(relatives[k] < 0 || areas[k] <= 1e-10) {
                    continue;
                }
                const LaneData& lane = map.lanes[relatives[k]];
                double distance = lane.index.Nearest(center, lane.segments).distance_squared;
                if(output[i].lane_index < 0 || areas[k] > output[i].overlap_area + tie ||
                   (std::abs(areas[k] - output[i].overlap_area) <= tie && distance < best_distance - 1e-10)) {
                    output[i].type = static_cast<std::uint8_t>(k);
                    output[i].lane_index = relatives[k];
                    output[i].overlap_area = std::min(box.dx * box.dy, areas[k]);
                    best_distance = distance;
                }
            }
        }
        tOutput.swap(output);
        return STATUS_S();
    }

}
