// 由 perception_msg_convert.cpp 单独包含，保持现有 CMake 编译入口，避免改动目录外配置。
#include "perception_temporal_filter.h"
#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>

namespace perception_filter_detail {
    const double WINDOW_SECONDS = 2.0;
    const double DECAY_SECONDS = 1.0;
    const double CELL_SIZE = 1.0;  // 与中心距离门限相同，只需查询中心所在格及周围八格。

    struct BOX_S {
        double x, y, half_x, half_y, area;
    };

    struct CELL_S {
        std::int64_t x, y;
        bool operator==(const CELL_S& tOther) const {
            return x == tOther.x && y == tOther.y;
        }
    };

    struct CellHash {
        std::size_t operator()(const CELL_S& tCell) const {
            std::size_t x_hash = std::hash<std::int64_t>()(tCell.x);
            std::size_t y_hash = std::hash<std::int64_t>()(tCell.y);
            return x_hash ^ (y_hash + 0x9e3779b9U + (x_hash << 6) + (x_hash >> 2));
        }
    };

    CELL_S GetCell(double tX, double tY) {
        return CELL_S{static_cast<std::int64_t>(std::floor(tX / CELL_SIZE)),
                      static_cast<std::int64_t>(std::floor(tY / CELL_SIZE))};
    }

    bool IsCoordinateValid(double tValue) {
        // 为浮点转整数和相邻格加减留余量，拒绝无穷/NaN，避免索引溢出。
        return std::isfinite(tValue) && std::abs(tValue) < 4.0e18;
    }

    BOX_S ReadBox(const robot::object& tObject) {
        if(!IsCoordinateValid(tObject.x) || !IsCoordinateValid(tObject.y) ||
           !std::isfinite(tObject.dx) || !std::isfinite(tObject.dy) ||
           tObject.dx <= 0 || tObject.dy <= 0) {
            throw std::invalid_argument("FilterPerceptionHistory requires finite centers and positive box dimensions");
        }
        return BOX_S{tObject.x, tObject.y, tObject.dx * 0.5, tObject.dy * 0.5,
                     static_cast<double>(tObject.dx) * tObject.dy};
    }

    double OverlapArea(const BOX_S& tA, const BOX_S& tB) {
        // 只比较轴对齐框；先平移到 A 的中心，避免大地图坐标加减半尺寸时损失精度。
        double dx = tB.x - tA.x;
        double dy = tB.y - tA.y;
        double width = std::min(tA.half_x, dx + tB.half_x) - std::max(-tA.half_x, dx - tB.half_x);
        double height = std::min(tA.half_y, dy + tB.half_y) - std::max(-tA.half_y, dy - tB.half_y);
        return std::max(0.0, width) * std::max(0.0, height);
    }

    struct FRAME_S {
        const robot::perception* message;
        double timestamp, weight;
        std::size_t input_index;
    };

    std::vector<FRAME_S> SelectFrames(const std::vector<std::pair<robot::perception, double>>& tHistory,
                                      double tNow) {
        std::vector<FRAME_S> frames;
        frames.reserve(tHistory.size());
        for(std::size_t i = 0; i < tHistory.size(); ++i) {
            double timestamp = tHistory[i].second;
            if(!std::isfinite(timestamp) || timestamp < 0 || timestamp > tNow || tNow - timestamp >= WINDOW_SECONDS) {
                continue;
            }
            frames.push_back(FRAME_S{&tHistory[i].first, timestamp,
                                     std::exp(-(tNow - timestamp) / DECAY_SECONDS), i});
        }
        std::sort(frames.begin(), frames.end(), [](const FRAME_S& a, const FRAME_S& b) {
            return a.timestamp < b.timestamp || (a.timestamp == b.timestamp && a.input_index < b.input_index);
        });
        std::size_t count = 0;
        for(std::size_t i = 0; i < frames.size(); ++i) {
            if(count > 0 && frames[count - 1].timestamp == frames[i].timestamp) {
                frames[count - 1] = frames[i];  // 重复缓存同一时刻不能增加观测次数。
            } else {
                frames[count++] = frames[i];
            }
        }
        frames.resize(count);
        return frames;
    }

    struct TRACK_S {
        const robot::object* latest;
        BOX_S box;
        double weight;
        std::size_t last_frame, cell_slot;
        CELL_S cell;
    };

    struct MATCH_S {
        std::size_t track, observation;
        double distance_squared, overlap_ratio;
    };

    typedef std::unordered_map<CELL_S, std::vector<std::size_t>, CellHash> Grid;

    void AddToGrid(Grid& tGrid, std::vector<TRACK_S>& tTracks, std::size_t tIndex) {
        TRACK_S& track = tTracks[tIndex];
        track.cell = GetCell(track.box.x, track.box.y);
        std::vector<std::size_t>& bucket = tGrid[track.cell];
        track.cell_slot = bucket.size();
        bucket.push_back(tIndex);
    }

    void MoveInGrid(Grid& tGrid, std::vector<TRACK_S>& tTracks, std::size_t tIndex) {
        TRACK_S& track = tTracks[tIndex];
        if(track.cell == GetCell(track.box.x, track.box.y)) {
            return;
        }
        Grid::iterator old_cell = tGrid.find(track.cell);
        std::vector<std::size_t>& bucket = old_cell->second;
        // 用末项填空位，移动目标只更新两格，避免每帧重新索引整个两秒历史。
        std::size_t moved = bucket.back();
        bucket[track.cell_slot] = moved;
        tTracks[moved].cell_slot = track.cell_slot;
        bucket.pop_back();
        if(bucket.empty()) {
            tGrid.erase(old_cell);
        }
        AddToGrid(tGrid, tTracks, tIndex);
    }

    std::vector<MATCH_S> FindMatches(const std::vector<BOX_S>& tBoxes,
                                     const std::vector<TRACK_S>& tTracks, const Grid& tGrid) {
        std::vector<MATCH_S> matches;
        matches.reserve(tBoxes.size());
        for(std::size_t i = 0; i < tBoxes.size(); ++i) {
            const BOX_S& box = tBoxes[i];
            CELL_S center = GetCell(box.x, box.y);
            for(int x = -1; x <= 1; ++x) {
                for(int y = -1; y <= 1; ++y) {
                    Grid::const_iterator cell = tGrid.find(CELL_S{center.x + x, center.y + y});
                    if(cell == tGrid.end()) {
                        continue;
                    }
                    for(std::size_t k = 0; k < cell->second.size(); ++k) {
                        std::size_t track_index = cell->second[k];
                        const BOX_S& previous = tTracks[track_index].box;
                        double dx = box.x - previous.x;
                        double dy = box.y - previous.y;
                        double distance_squared = dx * dx + dy * dy;
                        if(distance_squared > 2.0) {
                            continue;
                        }
                        double area = OverlapArea(box, previous);
                        double smaller_area = std::min(box.area, previous.area);
                        // 至少覆盖任意一框的 30%，等价于交面积 / 较小框面积 >= 0.3，不能用 IoU 代替。
                        if(area > 0 && area >= 0.3 * smaller_area) {
                            matches.push_back(MATCH_S{track_index, i, distance_squared, area / smaller_area});
                        }
                    }
                }
            }
        }
        std::sort(matches.begin(), matches.end(), [](const MATCH_S& a, const MATCH_S& b) {
            if(a.distance_squared != b.distance_squared) {
                return a.distance_squared < b.distance_squared;
            }
            if(a.overlap_ratio != b.overlap_ratio) {
                return a.overlap_ratio > b.overlap_ratio;
            }
            return a.track < b.track || (a.track == b.track && a.observation < b.observation);
        });
        return matches;
    }

    void AccumulateFrame(const FRAME_S& tFrame, std::size_t tFrameIndex,
                         std::vector<TRACK_S>& tTracks, Grid& tGrid) {
        const auto& objects = tFrame.message->objs;
        std::vector<BOX_S> boxes;
        boxes.reserve(objects.size());
        for(std::size_t i = 0; i < objects.size(); ++i) {
            boxes.push_back(ReadBox(objects[i]));
        }
        std::vector<MATCH_S> matches = FindMatches(boxes, tTracks, tGrid);
        std::vector<bool> assigned(objects.size(), false);
        // 候选按中心距离、覆盖比例、稳定索引排序；一对一关联防止两个邻近目标被一帧吞并。
        for(std::size_t i = 0; i < matches.size(); ++i) {
            const MATCH_S& match = matches[i];
            TRACK_S& track = tTracks[match.track];
            if(assigned[match.observation] || track.last_frame == tFrameIndex) {
                continue;
            }
            assigned[match.observation] = true;
            track.latest = &objects[match.observation];
            track.box = boxes[match.observation];
            track.weight += tFrame.weight;
            track.last_frame = tFrameIndex;
            MoveInGrid(tGrid, tTracks, match.track);
        }
        // 同帧未匹配的框各自建轨迹，不在当前帧内反复关联、重复增加置信度。
        for(std::size_t i = 0; i < objects.size(); ++i) {
            if(!assigned[i]) {
                tTracks.push_back(TRACK_S{&objects[i], boxes[i], tFrame.weight, tFrameIndex, 0, CELL_S{0, 0}});
                AddToGrid(tGrid, tTracks, tTracks.size() - 1);
            }
        }
    }
}

robot::perception FilterPerceptionHistory(const std::vector<std::pair<robot::perception, double>>& tHistory) {
    using namespace perception_filter_detail;
    const double now = ros::Time::now().toSec();
    if(!std::isfinite(now) || now < 0) {
        throw std::invalid_argument("FilterPerceptionHistory requires a valid ROS time");
    }
    std::vector<FRAME_S> frames = SelectFrames(tHistory, now);
    robot::perception result;
    if(frames.empty()) {
        return result;
    }
    result.header = frames.back().message->header;
    double total_weight = 0;
    std::size_t observation_count = 0;
    for(std::size_t i = 0; i < frames.size(); ++i) {
        total_weight += frames[i].weight;
        observation_count += frames[i].message->objs.size();
    }
    std::vector<TRACK_S> tracks;
    tracks.reserve(observation_count);
    Grid grid;
    grid.reserve(observation_count);
    for(std::size_t i = 0; i < frames.size(); ++i) {
        AccumulateFrame(frames[i], i, tracks, grid);
    }
    result.objs.reserve(tracks.size());
    for(std::size_t i = 0; i < tracks.size(); ++i) {
        result.objs.push_back(*tracks[i].latest);
        // 分母含空帧/漏检帧；用全帧权重归一化，单次闪现不会因“按最大目标缩放”被抬到 1。
        result.objs.back().confidence = static_cast<float>(std::max(0.0, std::min(1.0, tracks[i].weight / total_weight)));
    }
    return result;
}
