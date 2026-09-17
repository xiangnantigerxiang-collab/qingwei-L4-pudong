// 由现有节点包含一次，无需改目录外 CMake 或消息定义。
#include "perception_object_tracker.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <bitset>
#include <cstdint>

namespace perception_tracking_detail {
    const double TRACK_LIFETIME = 2.0;       // 与原始观测窗口一致，漏检不能延长寿命。
    const double POSITION_VARIANCE = 0.04;  // 位置观测标准差 0.2 米。
    const double VELOCITY_VARIANCE = 100.0;  // 新目标速度未知，初始标准差 10 米/秒。
    const double ACCELERATION_VARIANCE = 4.0;  // 连续白噪声加速度模型，单位 m²/s³。
    const double MIN_VELOCITY_SPAN = 0.2;    // 至少跨 0.2 秒真实观测后才输出速度。
    const double STATIONARY_SPEED = 0.2;    // 米/秒；低速噪声归零，航向保持。
    const double MAX_ASSOCIATION_SPEED = 30.0;  // 米/秒；拒绝不合理的位置跳变。
    const double UNMATCHED_COST = 25.0;
    const double INVALID_COST = 1e6;
    const int MAX_TRACK_ID = 32767;  // object.id 为 int16，保留 0 表示未跟踪。

    struct ASSOCIATION_S {
        double gate, displacement, variance, variance_cost, short_side, long_side;
    };

    // 两秒内轨迹的距离门限严格小于 4 米；4 米格只需查询自身和相邻八格。
    // 开放寻址表和链表索引按峰值复用，避免每帧为每个格子申请/释放哈希节点。
    struct SPATIAL_INDEX_S {
        struct CELL_S {
            std::int64_t x, y;
            std::size_t generation;
            int head;
        };
        std::vector<CELL_S> cells;
        std::vector<int> next;
        std::size_t generation = 0;

        void Reset(std::size_t tCount) {
            std::size_t capacity = 16;
            while(capacity < tCount * 2) {
                capacity *= 2;
            }
            if(cells.size() < capacity) {
                cells.assign(capacity, CELL_S{});
            }
            if(++generation == 0) {
                std::fill(cells.begin(), cells.end(), CELL_S{});
                generation = 1;
            }
            next.resize(tCount);
        }

        static bool CanIndex(double tX, double tY) {
            // 极端有限坐标仍沿用原来的扫描路径，不作越界浮点转整数。
            return std::isfinite(tX) && std::isfinite(tY) && std::abs(tX) < 1e12 && std::abs(tY) < 1e12;
        }

        std::size_t Find(std::int64_t tX, std::int64_t tY) const {
            const std::uint64_t hash = static_cast<std::uint64_t>(tX) * 11400714819323198485ULL ^
                                       static_cast<std::uint64_t>(tY) * 14029467366897019727ULL;
            std::size_t slot = (hash ^ (hash >> 32)) & (cells.size() - 1);
            while(cells[slot].generation == generation && (cells[slot].x != tX || cells[slot].y != tY)) {
                slot = (slot + 1) & (cells.size() - 1);
            }
            return slot;
        }

        void Add(double tX, double tY, std::size_t tIndex) {
            const std::int64_t x = static_cast<std::int64_t>(std::floor(tX / 4.0));
            const std::int64_t y = static_cast<std::int64_t>(std::floor(tY / 4.0));
            CELL_S& cell = cells[Find(x, y)];
            if(cell.generation != generation) {
                cell = CELL_S{x, y, generation, -1};
            }
            next[tIndex] = cell.head;
            cell.head = static_cast<int>(tIndex);
        }

        int Head(std::int64_t tX, std::int64_t tY) const {
            const CELL_S& cell = cells[Find(tX, tY)];
            return cell.generation == generation ? cell.head : -1;
        }
    };

    float NavigationHeading(double tDegrees) {
        double heading = std::fmod(tDegrees, 360.0);
        if(heading < 0) {
            heading += 360.0;
        }
        const float result = static_cast<float>(heading);
        // float32 舍入可能把接近 360 的有效角变成 360，统一回到 [0,360)。
        return result >= 360.0f ? 0.0f : result;
    }

    // 矩形匈牙利分配：每个观测最多对应一条轨迹，额外列表示新目标。
    // 全局分配避免观测顺序变化时，局部最近邻抢占另一目标唯一的可行匹配。
    std::vector<int> Assign(const std::vector<std::vector<double>>& tCosts, std::size_t tTrackCount,
                            std::size_t tRowCount = std::numeric_limits<std::size_t>::max()) {
        const std::size_t row_count = std::min(tCosts.size(), tRowCount);
        std::vector<int> assignment(row_count, -1);
        if(tTrackCount == 0) {
            return assignment;
        }
        // 无可行边的观测必然新生，无可行边的旧轨迹不参与本次分配。
        // 先压缩索引，避免两秒内累计的漏检轨迹放大匈牙利搜索范围；轨迹本身仍保留。
        // 节点为单线程；这里只缓存工作容量，不保存跨帧身份或运动状态。
        static std::vector<std::size_t> active_rows, active_tracks;
        static std::vector<bool> has_candidate;
        active_rows.clear();
        active_tracks.clear();
        has_candidate.assign(tTrackCount, false);
        for(std::size_t i = 0; i < row_count; ++i) {
            bool found = false;
            for(std::size_t j = 0; j < tTrackCount; ++j) {
                if(tCosts[i][j] < UNMATCHED_COST) {
                    found = true;
                    has_candidate[j] = true;
                }
            }
            if(found) {
                active_rows.push_back(i);
            }
        }
        if(active_rows.empty()) {
            return assignment;
        }
        for(std::size_t j = 0; j < tTrackCount; ++j) {
            if(has_candidate[j]) {
                active_tracks.push_back(j);
            }
        }
        const std::size_t rows = active_rows.size();
        const std::size_t track_count = active_tracks.size();
        const std::size_t columns = track_count + rows;
        static std::vector<double> u, v, minimum;
        static std::vector<std::size_t> p, way;
        static std::vector<bool> used;
        u.assign(rows + 1, 0);
        v.assign(columns + 1, 0);
        p.assign(columns + 1, 0);
        way.resize(columns + 1);
        for(std::size_t row = 1; row <= rows; ++row) {
            p[0] = row;
            std::size_t column = 0;
            minimum.assign(columns + 1, INVALID_COST);
            used.assign(columns + 1, false);
            do {
                used[column] = true;
                const std::size_t current_row = p[column];
                double delta = INVALID_COST;
                std::size_t next = 0;
                for(std::size_t j = 1; j <= columns; ++j) {
                    if(used[j]) {
                        continue;
                    }
                    // 每个观测只使用自己的新生列，未匹配代价不变。
                    // 避免大量新目标在等价新生列之间反复增广，保留全局最优的一对一匹配。
                    const double cost = j <= track_count ? tCosts[active_rows[current_row - 1]][active_tracks[j - 1]] :
                        (j == track_count + current_row ? UNMATCHED_COST : INVALID_COST);
                    const double reduced = cost - u[current_row] - v[j];
                    if(reduced < minimum[j]) {
                        minimum[j] = reduced;
                        way[j] = column;
                    }
                    if(minimum[j] < delta) {
                        delta = minimum[j];
                        next = j;
                    }
                }
                for(std::size_t j = 0; j <= columns; ++j) {
                    if(used[j]) {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    } else {
                        minimum[j] -= delta;
                    }
                }
                column = next;
            } while(p[column] != 0);
            do {
                const std::size_t previous = way[column];
                p[column] = p[previous];
                column = previous;
            } while(column != 0);
        }
        for(std::size_t j = 1; j <= track_count; ++j) {
            if(p[j] != 0 && tCosts[active_rows[p[j] - 1]][active_tracks[j - 1]] < UNMATCHED_COST) {
                assignment[active_rows[p[j] - 1]] = static_cast<int>(active_tracks[j - 1]);
            }
        }
        return assignment;
    }

    std::size_t FindRoot(std::vector<std::size_t>& tParents, std::size_t tRow) {
        while(tParents[tRow] != tRow) {
            tParents[tRow] = tParents[tParents[tRow]];
            tRow = tParents[tRow];
        }
        return tRow;
    }

    std::vector<int> AssignCandidates(const std::vector<std::vector<std::pair<std::size_t, double>>>& tCandidates,
                                      std::size_t tTrackCount) {
        std::vector<int> assignment(tCandidates.size(), -1);
        static std::vector<std::size_t> parents, active_rows, track_columns;
        static std::vector<int> owners, heads, next, column_indices;
        static std::vector<std::vector<double>> costs;
        parents.resize(tCandidates.size());
        heads.assign(tCandidates.size(), -1);
        next.resize(tCandidates.size());
        owners.assign(tTrackCount, -1);
        column_indices.assign(tTrackCount, -1);
        for(std::size_t i = 0; i < tCandidates.size(); ++i) {
            parents[i] = i;
            for(std::size_t j = 0; j < tCandidates[i].size(); ++j) {
                const std::size_t track = tCandidates[i][j].first;
                if(owners[track] < 0) {
                    owners[track] = static_cast<int>(i);
                } else {
                    const std::size_t first = FindRoot(parents, i);
                    const std::size_t second = FindRoot(parents, owners[track]);
                    parents[std::max(first, second)] = std::min(first, second);
                }
            }
        }
        // 仅共享可行轨迹的观测才互相竞争；各连通分量可独立求解同一全局最优目标。
        // 链表逆序构建，求解仍按原来的观测/轨迹顺序，保持相等代价的选择稳定。
        for(std::size_t i = tCandidates.size(); i > 0; --i) {
            if(tCandidates[i - 1].empty()) {
                continue;
            }
            const std::size_t root = FindRoot(parents, i - 1);
            next[i - 1] = heads[root];
            heads[root] = static_cast<int>(i - 1);
        }
        for(std::size_t root = 0; root < heads.size(); ++root) {
            if(heads[root] < 0) {
                continue;
            }
            if(next[heads[root]] < 0) {
                // 单观测分量不需要匈牙利矩阵；多个等价候选仍取原来的最小轨迹索引。
                const std::size_t row = heads[root];
                double best = UNMATCHED_COST;
                for(std::size_t j = 0; j < tCandidates[row].size(); ++j) {
                    const auto& candidate = tCandidates[row][j];
                    if(candidate.second < best || (candidate.second == best &&
                       static_cast<int>(candidate.first) < assignment[row])) {
                        best = candidate.second;
                        assignment[row] = static_cast<int>(candidate.first);
                    }
                }
                continue;
            }
            active_rows.clear();
            track_columns.clear();
            for(int row = heads[root]; row >= 0; row = next[row]) {
                active_rows.push_back(row);
                for(std::size_t j = 0; j < tCandidates[row].size(); ++j) {
                    const std::size_t track = tCandidates[row][j].first;
                    if(column_indices[track] < 0) {
                        column_indices[track] = 0;
                        track_columns.push_back(track);
                    }
                }
            }
            std::sort(track_columns.begin(), track_columns.end());
            for(std::size_t j = 0; j < track_columns.size(); ++j) {
                column_indices[track_columns[j]] = static_cast<int>(j);
            }
            if(costs.size() < active_rows.size()) {
                costs.resize(active_rows.size());
            }
            // 仅处理本分量行数，其他缓存行不清零/扫描，保留容量供后续复用。
            for(std::size_t i = 0; i < active_rows.size(); ++i) {
                costs[i].assign(track_columns.size(), INVALID_COST);
                const auto& candidates = tCandidates[active_rows[i]];
                for(std::size_t j = 0; j < candidates.size(); ++j) {
                    costs[i][column_indices[candidates[j].first]] = candidates[j].second;
                }
            }
            const std::vector<int> result = Assign(costs, track_columns.size(), active_rows.size());
            for(std::size_t i = 0; i < active_rows.size(); ++i) {
                if(result[i] >= 0) {
                    assignment[active_rows[i]] = static_cast<int>(track_columns[result[i]]);
                }
            }
            for(std::size_t j = 0; j < track_columns.size(); ++j) {
                column_indices[track_columns[j]] = -1;
            }
        }
        return assignment;
    }
}

PerceptionObjectTracker::PerceptionObjectTracker(int tNextId)
    : mNextId(tNextId), mLastFrameTime(-1) {
    if(tNextId <= 0 || tNextId > perception_tracking_detail::MAX_TRACK_ID) {
        throw std::invalid_argument("Invalid first perception track ID");
    }
}

void PerceptionObjectTracker::Clear() {
    mTracks.clear();
    mLastFrameTime = -1;
    // 不将分配器归一，避免过期/时间回退后立刻把旧 ID 发给另一个目标。
}

void PerceptionObjectTracker::KeepIdSequence(const PerceptionObjectTracker& tPrevious) {
    // 运动状态回滚至同时间戳之前，已发布过的 ID 分配序列仍继续向前。
    mNextId = tPrevious.mNextId;
}

void PerceptionObjectTracker::Maintain(double tNow) {
    if(!std::isfinite(tNow) || tNow < 0 || (mLastFrameTime >= 0 && tNow < mLastFrameTime)) {
        Clear();
        return;
    }
    mTracks.erase(std::remove_if(mTracks.begin(), mTracks.end(), [tNow](const TRACK_S& track) {
        return tNow - track.last_seen >= perception_tracking_detail::TRACK_LIFETIME;
    }), mTracks.end());
}

void PerceptionObjectTracker::Predict(TRACK_S& tTrack, double tTimestamp) const {
    const double dt = tTimestamp - tTrack.state_time;
    const double q = perception_tracking_detail::ACCELERATION_VARIANCE;
    tTrack.x += tTrack.vx * dt;
    tTrack.y += tTrack.vy * dt;
    // 连续白噪声模型在分段预测与一次长间隔预测时给出相同协方差。
    tTrack.p00 += 2 * dt * tTrack.p01 + dt * dt * tTrack.p11 + q * dt * dt * dt / 3.0;
    tTrack.p01 += dt * tTrack.p11 + q * dt * dt / 2.0;
    tTrack.p11 += q * dt;
    tTrack.state_time = tTimestamp;
}

void PerceptionObjectTracker::Correct(TRACK_S& tTrack, const robot::object& tObject, double tTimestamp) const {
    const double r = perception_tracking_detail::POSITION_VARIANCE;
    const double innovation = tTrack.p00 + r;
    const double k_position = tTrack.p00 / innovation;
    const double k_velocity = tTrack.p01 / innovation;
    const double residual_x = tObject.x - tTrack.x;
    const double residual_y = tObject.y - tTrack.y;
    tTrack.x += k_position * residual_x;
    tTrack.y += k_position * residual_y;
    tTrack.vx += k_velocity * residual_x;
    tTrack.vy += k_velocity * residual_y;
    // Joseph 形式，避免反复更新时差分消去导致负方差。
    const double a = 1 - k_position;
    const double p00 = tTrack.p00;
    const double p01 = tTrack.p01;
    tTrack.p00 = a * a * p00 + k_position * k_position * r;
    tTrack.p01 = a * (p01 - k_velocity * p00) + k_position * k_velocity * r;
    tTrack.p11 += k_velocity * k_velocity * (p00 + r) - 2 * k_velocity * p01;
    tTrack.dx = tObject.dx;
    tTrack.dy = tObject.dy;
    tTrack.observed_x = tObject.x;
    tTrack.observed_y = tObject.y;
    tTrack.last_seen = tTimestamp;
    ++tTrack.observations;
}

void PerceptionObjectTracker::WriteMotion(TRACK_S& tTrack, robot::object& tObject) const {
    using namespace perception_tracking_detail;
    tObject.id = tTrack.id;
    tObject.vx = 0;
    tObject.vy = 0;
    if(tTrack.observations >= 2 && tTrack.last_seen - tTrack.first_seen + 1e-9 >= MIN_VELOCITY_SPAN &&
       std::hypot(tTrack.vx, tTrack.vy) >= STATIONARY_SPEED) {
        tObject.vx = tTrack.vx;
        tObject.vy = tTrack.vy;
        // vx 为地图东向、vy 为地图北向；atan2(东,北) 得到北零顺时针方位角。
        tTrack.heading = NavigationHeading(std::atan2(tObject.vx, tObject.vy) * 180.0 / M_PI);
    }
    // 速度未建立/接近零时方向无定义，保留上一有效运动航向；新轨迹用检测框世界朝向。
    tObject.heading = NavigationHeading(tTrack.heading);
}

void PerceptionObjectTracker::Update(robot::perception& tPerception, double tTimestamp, const robot::perception* tReplacedFrame) {
    using namespace perception_tracking_detail;
    if(!std::isfinite(tTimestamp) || tTimestamp < 0 ||
       (mLastFrameTime >= 0 && tTimestamp <= mLastFrameTime)) {
        throw std::invalid_argument("Tracker requires increasing finite observation timestamps");
    }
    if(tPerception.objs.size() > static_cast<std::size_t>(MAX_TRACK_ID)) {
        throw std::invalid_argument("Observation count exceeds the positive int16 track ID range");
    }
    for(std::size_t i = 0; i < tPerception.objs.size(); ++i) {
        const robot::object& object = tPerception.objs[i];
        if(!std::isfinite(object.x) || !std::isfinite(object.y) ||
           !std::isfinite(object.dx) || !std::isfinite(object.dy) ||
           !std::isfinite(object.heading) || object.dx <= 0 || object.dy <= 0) {
            throw std::invalid_argument("Tracker requires finite positions, headings and positive dimensions");
        }
    }
    Maintain(tTimestamp);
    std::bitset<MAX_TRACK_ID + 1> used_ids;
    std::size_t used_id_count = mTracks.size();
    // 临时索引在每次 Update 时重建，单线程下共享容量即可；不随跟踪工作副本反复复制。
    // 故障回滚、重复时间戳和不同跟踪器实例都不会继承上一次索引中的内容。
    static SPATIAL_INDEX_S grid;
    static std::vector<ASSOCIATION_S> associations;
    associations.resize(mTracks.size());
    bool use_grid = mTracks.size() >= 16 && !tPerception.objs.empty();
    for(std::size_t j = 0; j < mTracks.size(); ++j) {
        Predict(mTracks[j], tTimestamp);
        used_ids.set(mTracks[j].id);
        const TRACK_S& track = mTracks[j];
        const double gap = tTimestamp - track.last_seen;
        const double variance = track.p00 + POSITION_VARIANCE;
        associations[j] = ASSOCIATION_S{2.0 + gap, MAX_ASSOCIATION_SPEED * gap + 0.5, variance,
            std::log(variance / POSITION_VARIANCE), std::min(track.dx, track.dy), std::max(track.dx, track.dy)};
        use_grid = use_grid && SPATIAL_INDEX_S::CanIndex(track.x, track.y);
    }
    if(use_grid) {
        grid.Reset(mTracks.size());
        for(std::size_t j = 0; j < mTracks.size(); ++j) {
            grid.Add(mTracks[j].x, mTracks[j].y, j);
        }
    }
    // 只为可行候选分配代价列。漏检轨迹仍保存，但不再给每个新观测分配整行无效代价。
    static std::vector<std::vector<std::pair<std::size_t, double>>> candidates;
    candidates.resize(tPerception.objs.size());
    for(std::size_t i = 0; i < tPerception.objs.size(); ++i) {
        candidates[i].clear();
        const robot::object& object = tPerception.objs[i];
        const double short_side = std::min(object.dx, object.dy);
        const double long_side = std::max(object.dx, object.dy);
        const auto add_candidate = [&](std::size_t j) {
            const TRACK_S& track = mTracks[j];
            const ASSOCIATION_S& association = associations[j];
            // 圆门限的外接方框先筛掉远处轨迹，省去绝大多数无效候选的两次 hypot。
            // 通过方框后仍执行原来的圆门限、速度门限及尺寸/协方差判据。
            if(std::abs(object.x - track.x) > association.gate || std::abs(object.y - track.y) > association.gate) {
                return;
            }
            const double residual = std::hypot(object.x - track.x, object.y - track.y);
            const double displacement = std::hypot(object.x - track.observed_x, object.y - track.observed_y);
            const double variance = association.variance;
            const double short_ratio = short_side / association.short_side;
            const double long_ratio = long_side / association.long_side;
            if(residual > association.gate || displacement > association.displacement ||
               residual * residual > 9.0 * variance ||
               short_ratio < 1.0 / 3.0 || short_ratio > 3.0 || long_ratio < 1.0 / 3.0 || long_ratio > 3.0) {
                return;
            }
            const double shape_cost = std::pow(std::log(short_ratio), 2) + std::pow(std::log(long_ratio), 2);
            // 方差惩罚防止长期漏检、位置不确定的旧轨迹抢占稳定目标的观测。
            const double cost = residual * residual / variance + association.variance_cost + shape_cost;
            if(cost < UNMATCHED_COST) {
                candidates[i].push_back(std::make_pair(j, cost));
            }
        };
        if(use_grid && SPATIAL_INDEX_S::CanIndex(object.x, object.y)) {
            const std::int64_t x = static_cast<std::int64_t>(std::floor(object.x / 4.0));
            const std::int64_t y = static_cast<std::int64_t>(std::floor(object.y / 4.0));
            for(int dx = -1; dx <= 1; ++dx) {
                for(int dy = -1; dy <= 1; ++dy) {
                    for(int j = grid.Head(x + dx, y + dy); j >= 0; j = grid.next[j]) {
                        add_candidate(j);
                    }
                }
            }
        } else {
            for(std::size_t j = 0; j < mTracks.size(); ++j) {
                add_candidate(j);
            }
        }
    }
    std::vector<int> assignment = AssignCandidates(candidates, mTracks.size());
    std::vector<int> birth_ids(tPerception.objs.size(), 0);
    if(tReplacedFrame) {
        // 检查点中还没有“本帧才出生”的目标；按旧帧位置找回其 ID，观测顺序不作为身份。
        std::vector<std::size_t> births, hints;
        for(std::size_t i = 0; i < assignment.size(); ++i) {
            if(assignment[i] < 0) {
                births.push_back(i);
            }
        }
        for(std::size_t j = 0; j < tReplacedFrame->objs.size(); ++j) {
            const int id = tReplacedFrame->objs[j].id;
            if(id > 0 && !used_ids.test(id)) {
                hints.push_back(j);
                // 旧帧已发布过的 ID 即使本次被删除，也不能马上分配给另一个新目标。
                used_ids.set(id);
                ++used_id_count;
            }
        }
        std::vector<std::vector<double>> birth_costs(births.size(), std::vector<double>(hints.size(), INVALID_COST));
        for(std::size_t i = 0; i < births.size(); ++i) {
            const auto& object = tPerception.objs[births[i]];
            for(std::size_t j = 0; j < hints.size(); ++j) {
                const auto& hint = tReplacedFrame->objs[hints[j]];
                const double distance = std::hypot(object.x - hint.x, object.y - hint.y);
                if(distance <= 1.0 && object.dx >= hint.dx / 3.0 && object.dx <= hint.dx * 3.0 &&
                   object.dy >= hint.dy / 3.0 && object.dy <= hint.dy * 3.0) {
                    birth_costs[i][j] = distance * distance;
                }
            }
        }
        const auto birth_assignment = Assign(birth_costs, hints.size());
        for(std::size_t i = 0; i < births.size(); ++i) {
            if(birth_assignment[i] >= 0) {
                birth_ids[births[i]] = tReplacedFrame->objs[hints[birth_assignment[i]]].id;
            }
        }
    }
    for(std::size_t i = 0; i < tPerception.objs.size(); ++i) {
        robot::object& object = tPerception.objs[i];
        int index = assignment[i];
        if(index < 0) {
            int id = birth_ids[i];
            if(id == 0) {
                if(used_id_count >= static_cast<std::size_t>(MAX_TRACK_ID)) {
                    throw std::invalid_argument("All positive int16 perception track IDs are occupied");
                }
                while(used_ids.test(mNextId)) {
                    mNextId = mNextId == MAX_TRACK_ID ? 1 : mNextId + 1;
                }
                id = mNextId;
                used_ids.set(id);
                ++used_id_count;
                mNextId = mNextId == MAX_TRACK_ID ? 1 : mNextId + 1;
            }
            mTracks.push_back(TRACK_S{id, object.x, object.y, 0, 0, POSITION_VARIANCE, 0, VELOCITY_VARIANCE,
                                     object.dx, object.dy, object.heading, tTimestamp, tTimestamp, tTimestamp,
                                     object.x, object.y, 1});
            index = static_cast<int>(mTracks.size() - 1);
        } else {
            Correct(mTracks[index], object, tTimestamp);
        }
        WriteMotion(mTracks[index], object);
    }
    if(used_id_count < static_cast<std::size_t>(MAX_TRACK_ID)) {
        while(used_ids.test(mNextId)) {
            mNextId = mNextId == MAX_TRACK_ID ? 1 : mNextId + 1;
        }
    }
    mLastFrameTime = tTimestamp;
}
