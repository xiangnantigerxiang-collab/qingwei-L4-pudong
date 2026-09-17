#include "../perception_object_tracker.inc"
#include "../perception_temporal_filter.inc"
#include <chrono>
#include <cstdio>
#include <random>

namespace {
    int checks = 0;

    void Check(bool tCondition, const char* tMessage) {
        ++checks;
        if(!tCondition) {
            throw std::runtime_error(tMessage);
        }
    }

    void Near(double tActual, double tExpected, double tTolerance, const char* tMessage) {
        if(!std::isfinite(tActual) || std::abs(tActual - tExpected) > tTolerance) {
            std::fprintf(stderr, "%s: actual %.9f expected %.9f\n", tMessage, tActual, tExpected);
        }
        Check(std::isfinite(tActual) && std::abs(tActual - tExpected) <= tTolerance, tMessage);
    }

    robot::object Box(double tX, double tY, double tHeading = 135, double tDx = 0.4, double tDy = 0.4) {
        robot::object object;
        object.x = tX;
        object.y = tY;
        object.dx = tDx;
        object.dy = tDy;
        object.heading = tHeading;
        object.id = -42;  // 上游编号不得作为跨帧身份。
        object.type = 4;
        object.height = 1.2;
        object.confidence = 0.37;
        object.polygons.resize(1);
        object.polygons[0].x = 456;
        return object;
    }

    robot::perception Update(PerceptionObjectTracker& tTracker, double tTime, const std::vector<robot::object>& tObjects) {
        robot::perception frame;
        frame.header.stamp = ros::Time(tTime);
        frame.header.frame_id = "map";
        frame.objs = tObjects;
        tTracker.Update(frame, tTime);
        return frame;
    }

    void TestDirectionsAndLifetime() {
        const double velocities[][3] = {{2, 0, 90}, {0, 2, 0}, {-2, 0, 270}, {0, -2, 180},
                                         {2, 2, 45}, {2, -2, 135}, {-2, -2, 225}, {-2, 2, 315},
                                         {8, 0, 90}};
        for(const auto& velocity : velocities) {
            PerceptionObjectTracker tracker;
            int id = 0;
            for(int i = 0; i <= 50; ++i) {
                const double time = i * 0.1;
                const auto input = Box(20 + velocity[0] * time, 30 + velocity[1] * time);
                const auto frame = Update(tracker, time, {input});
                const auto& object = frame.objs[0];
                if(i == 0) {
                    id = object.id;
                }
                Check(id > 0 && object.id == id, "ID persists across more than two seconds of real observations");
                Check(object.x == input.x && object.y == input.y && object.dx == input.dx && object.dy == input.dy &&
                      object.height == input.height && object.type == input.type && object.confidence == input.confidence &&
                      object.polygons[0].x == 456 && frame.header.frame_id == "map",
                      "tracking changes only ID, velocity and heading");
                Check(object.heading >= 0 && object.heading < 360, "navigation heading range");
                if(i < 2) {
                    Check(object.vx == 0 && object.vy == 0 && object.heading == 135,
                          "new target reports zero until enough temporal baseline exists");
                }
                if(i >= 10) {
                    Near(object.vx, velocity[0], 0.03, "map x velocity converges in metres per second");
                    Near(object.vy, velocity[1], 0.03, "map y velocity converges in metres per second");
                    Near(object.heading, velocity[2], 0.01, "all cardinal and diagonal directions use navigation heading");
                    Near(object.heading, perception_tracking_detail::NavigationHeading(
                        std::atan2(object.vx, object.vy) * 180.0 / M_PI), 1e-6, "heading derives from published vx and vy");
                }
            }
        }
        Near(perception_tracking_detail::NavigationHeading(-0.000001), 0, 1e-6, "float rounding never publishes 360 degrees");
        Near(perception_tracking_detail::NavigationHeading(-810), 270, 1e-6, "negative angles normalize");
    }

    void TestStationaryAndStopping() {
        PerceptionObjectTracker tracker;
        int id = 0;
        for(int i = 0; i < 50; ++i) {
            const double jitter = i % 2 == 0 ? 0.015 : -0.015;
            const auto frame = Update(tracker, i * 0.1, {Box(100 + jitter, -100 - jitter, 123)});
            if(i == 0) {
                id = frame.objs[0].id;
            }
            Check(frame.objs[0].id == id, "stationary jitter preserves ID");
            Check(frame.objs[0].vx == 0 && frame.objs[0].vy == 0 && frame.objs[0].heading == 123,
                  "small stationary jitter does not invent a motion direction");
        }
        tracker.Clear();
        robot::perception frame;
        for(int i = 0; i <= 60; ++i) {
            frame = Update(tracker, 10 + i * 0.1, {Box(2 * std::min(i * 0.1, 2.0), 0)});
        }
        Check(frame.objs[0].vx == 0 && frame.objs[0].vy == 0, "velocity settles to zero after a real stop");
        Near(frame.objs[0].heading, 90, 1e-5, "stopped target retains its last valid course");
    }

    void TestIrregularTimeAndMapPrecision() {
        const double times[] = {0, 0.05, 0.23, 0.5, 0.58, 1.0, 1.3, 1.9, 2.4, 2.6, 3.0, 3.5, 3.9};
        for(bool large_coordinates : {false, true}) {
            PerceptionObjectTracker tracker;
            const double x = large_coordinates ? 456000 : 10;
            const double y = large_coordinates ? 4132000 : 20;
            int id = 0;
            double squared_error = 0;
            std::size_t estimates = 0;
            for(double time : times) {
                const auto frame = Update(tracker, 100 + time, {Box(x + 3 * time, y - time)});
                const auto& object = frame.objs[0];
                if(time == 0) {
                    id = object.id;
                }
                Check(object.id == id, "irregular sampling and map float32 quantization preserve identity");
                if(time >= 2) {
                    Near(object.vx, 3, large_coordinates ? 0.1 : 0.03, "velocity uses actual elapsed time rather than a fixed frame rate");
                    // 4132000 的 float32 步长为 0.25 米，0.2 秒内相邻样本可被量化为同一点。
                    // 对该噪声场景同时检查峰值误差和均方根，不能要求每帧等于无量化的真值。
                    Near(object.vy, -1, large_coordinates ? 0.5 : 0.03, "large-coordinate quantization remains bounded in velocity estimate");
                    squared_error += std::pow(object.vy + 1, 2);
                    ++estimates;
                }
            }
            Check(std::sqrt(squared_error / estimates) < (large_coordinates ? 0.3 : 0.03),
                  "irregular large-map samples meet the velocity RMSE budget");
        }
    }

    void TestPredictionAndCrossing() {
        PerceptionObjectTracker tracker;
        int first_id = 0, second_id = 0;
        for(int i = 0; i <= 30; ++i) {
            const double time = i * 0.1;
            auto first = Box(20 + 2 * time, 0);
            auto second = Box(24 - 2 * time, 0);
            const bool reversed = i % 2 != 0;
            const auto frame = Update(tracker, time, reversed ? std::vector<robot::object>{second, first} :
                                                              std::vector<robot::object>{first, second});
            if(i == 0) {
                first_id = frame.objs[0].id;
                second_id = frame.objs[1].id;
            }
            Check(first_id != second_id, "two simultaneous detections have different identities");
            if(i != 10) {  // 正好重合时两条观测没有可辨识信息，检查穿越后的身份连续性。
                Check(frame.objs[reversed ? 1 : 0].id == first_id && frame.objs[reversed ? 0 : 1].id == second_id,
                      "prediction preserves crossing identities despite alternating observation order");
            }
        }
        tracker.Clear();
        robot::perception last;
        for(int i = 0; i <= 10; ++i) {
            last = Update(tracker, 10 + i * 0.1, {Box(2 * i * 0.1, 0)});
        }
        const int id = last.objs[0].id;
        const double vx = last.objs[0].vx;
        for(int i = 1; i <= 7; ++i) {
            Check(Update(tracker, 11 + i * 0.1, {}).objs.empty(), "prediction never creates a synthetic observation");
        }
        auto recovered = Update(tracker, 11.8, {Box(3.6, 0)});
        Check(recovered.objs[0].id == id, "prediction reacquires a small box with no overlap with its old measured position");
        Near(recovered.objs[0].vx, vx, 0.03, "missed frames do not feed zero-displacement measurements into velocity");
        auto far = Update(tracker, 11.9, {Box(100, 100)});
        Check(far.objs[0].id != id, "implausible jump starts a new identity instead of producing a huge velocity");
        Check(far.objs[0].vx == 0 && far.objs[0].vy == 0, "unmatched outlier is a new unestimated target");
        recovered = Update(tracker, 12, {Box(4, 0)});
        Check(recovered.objs[0].id == id, "outlier does not contaminate old target prediction");
        tracker.Maintain(14);
        const auto expired = Update(tracker, 14, {Box(8, 0)});
        Check(expired.objs[0].id != id, "exact two-second missed-detection lifetime releases a track");
        Check(expired.objs[0].vx == 0, "expired identity cannot leak its old velocity");
    }

    double BestCost(const std::vector<std::vector<double>>& tCosts, std::size_t tRow, std::vector<bool>& tUsed) {
        if(tRow == tCosts.size()) {
            return 0;
        }
        double best = 25 + BestCost(tCosts, tRow + 1, tUsed);
        for(std::size_t i = 0; i < tUsed.size(); ++i) {
            if(!tUsed[i]) {
                tUsed[i] = true;
                best = std::min(best, tCosts[tRow][i] + BestCost(tCosts, tRow + 1, tUsed));
                tUsed[i] = false;
            }
        }
        return best;
    }

    void TestAssignmentAndIds() {
        std::mt19937 random(19);
        for(int sample = 0; sample < 300; ++sample) {
            const std::size_t rows = 1 + random() % 5, columns = random() % 5;
            std::vector<std::vector<double>> costs(rows, std::vector<double>(columns));
            for(auto& row : costs) {
                for(auto& value : row) {
                    value = random() % 4 == 0 ? 1e6 : static_cast<double>(random() % 230) / 10;
                }
            }
            const auto assignment = perception_tracking_detail::Assign(costs, columns);
            std::vector<bool> used(columns, false);
            const double best = BestCost(costs, 0, used);
            double actual = 0;
            for(std::size_t i = 0; i < rows; ++i) {
                const int column = assignment[i];
                if(column < 0) {
                    actual += 25;
                } else {
                    Check(!used[column], "assignment is one-to-one");
                    used[column] = true;
                    actual += costs[i][column];
                }
            }
            Near(actual, best, 1e-7, "Hungarian assignment agrees with independent exhaustive search");
        }
        const std::vector<std::vector<double>> mixed_costs = {
            {0, 24.99}, {0, 1e6}, {1e6, 25}, {25.1, 1e6}
        };
        Check(perception_tracking_detail::Assign(mixed_costs, 2) == std::vector<int>({1, 0, -1, -1}),
              "private birth columns preserve global matching and the unmatched cost boundary");
        const std::vector<std::vector<double>> new_targets(128, std::vector<double>(256, 1e6));
        Check(perception_tracking_detail::Assign(new_targets, 256) == std::vector<int>(128, -1),
              "simultaneous births do not claim invalid tracks");
        PerceptionObjectTracker tracker(32767);
        const auto first = Update(tracker, 0, {Box(0, 0), Box(10, 0)});
        Check(first.objs[0].id == 32767 && first.objs[1].id == 1, "int16 ID allocation wraps without emitting zero or negatives");
        const auto second = Update(tracker, 0.1, {Box(10, 0), Box(0, 0), Box(20, 0)});
        Check(second.objs[0].id == 1 && second.objs[1].id == 32767 && second.objs[2].id == 2,
              "wrapped allocation skips active IDs and ignores array order");
        tracker.Clear();
        Check(Update(tracker, 0, {Box(0, 0)}).objs[0].id == 3, "reset clears motion but does not immediately recycle IDs");
        const double invalid_times[] = {0, -1, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()};
        for(double time : invalid_times) {
            bool rejected = false;
            try {
                Update(tracker, time, {Box(0, 0)});
            } catch(const std::invalid_argument&) {
                rejected = true;
            }
            Check(rejected, "tracker rejects duplicate, backward and nonfinite time without division by zero");
        }
        auto invalid = Box(std::numeric_limits<double>::quiet_NaN(), 0);
        bool rejected = false;
        try {
            Update(tracker, 0.1, {invalid});
        } catch(const std::invalid_argument&) {
            rejected = true;
        }
        Check(rejected && Update(tracker, 0.1, {Box(0, 0)}).objs[0].id == 3,
              "invalid measurement is rejected before changing a valid tracking state");
    }

    void TestTrackedConfidence() {
        std::vector<std::pair<robot::perception, double>> history;
        robot::perception first;
        auto a = Box(0, 0), b = Box(0, 0);
        a.id = 11;
        b.id = 22;
        first.objs = {a, b};
        first.header.stamp = ros::Time(100);
        history.push_back({first, 100});
        robot::perception second;
        a.x = 10;
        a.vx = 10;
        a.heading = 90;
        second.objs = {a};
        second.header.stamp = ros::Time(101);
        history.push_back({second, 101});
        ros::testTime() = 101;
        const auto output = FilterTrackedPerceptionHistory(history);
        Check(output.objs.size() == 2 && output.objs[0].id == 11 && output.objs[1].id == 22,
              "co-located identities remain separate while a fast moving ID stays one target");
        Near(output.objs[0].confidence, 1, 1e-6, "same ID confidence spans non-overlapping observed boxes");
        Near(output.objs[1].confidence, std::exp(-1) / (1 + std::exp(-1)), 1e-6, "missing ID loses weighted confidence");
        Check(output.objs[0].x == 10 && output.objs[0].vx == 10 && output.objs[0].heading == 90,
              "tracked filter preserves latest motion fields");
        Check(history.front().first.objs[0].x == 0 && history.back().first.objs[0].confidence == a.confidence,
              "tracked confidence never mutates raw history");
        second.objs.clear();
        history.push_back({second, 101});
        const auto duplicate = FilterTrackedPerceptionHistory(history);
        Near(duplicate.objs[0].confidence, std::exp(-1) / (1 + std::exp(-1)), 1e-6,
             "last duplicate timestamp wins and an empty observation participates once");
        ros::testTime() = 102;
        Check(FilterTrackedPerceptionHistory(history).objs.empty(), "tracked observations expire at exactly two seconds");
        ros::testTime() = 101;
        history.back().first.objs = {a, a};
        bool rejected = false;
        try {
            FilterTrackedPerceptionHistory(history);
        } catch(const std::invalid_argument&) {
            rejected = true;
        }
        Check(rejected, "duplicate IDs in one frame cannot inflate confidence");
        history.back().first.objs = {a};
        history.back().first.objs[0].id = 0;
        rejected = false;
        try {
            FilterTrackedPerceptionHistory(history);
        } catch(const std::invalid_argument&) {
            rejected = true;
        }
        Check(rejected, "untracked input is not silently merged into identity zero");
        for(int sample = 0; sample < 20; ++sample) {
            // 上次工作区中的 latest 指针会悬空；后续调用只能借用容量，不能读取旧消息。
            robot::perception input;
            auto object = Box(sample, 0);
            object.id = sample % 2 == 0 ? 32767 : 1;
            input.objs = {object};
            ros::testTime() = sample % 3;
            input.header.stamp = ros::Time(ros::testTime());
            const auto result = FilterTrackedPerceptionHistory({{input, ros::testTime()}});
            Check(result.objs.size() == 1 && result.objs[0].id == object.id && result.objs[0].confidence == 1,
                  "history lookup cache is isolated across destroyed inputs, exceptions and clock rollback");
        }
    }

    void TestSparseAssignment() {
        std::mt19937 random(20260916);
        for(int sample = 0; sample < 1200; ++sample) {
            const std::size_t rows = 1 + random() % 70, columns = random() % 80;
            const std::size_t groups = 1 + random() % 10;
            std::vector<std::vector<double>> costs(rows, std::vector<double>(columns, 1e6));
            std::vector<std::vector<std::pair<std::size_t, double>>> candidates(rows);
            for(std::size_t i = 0; i < rows; ++i) {
                for(std::size_t j = 0; j < columns; ++j) {
                    if(i % groups == j % groups && random() % 3 == 0) {
                        // 整数代价刻意制造并列；边界 25 不应成为可行候选。
                        costs[i][j] = random() % 27;
                    }
                    if(costs[i][j] < 25) {
                        candidates[i].push_back(std::make_pair(j, costs[i][j]));
                    } else {
                        // 生产 Update 仅把 <25 的边写入矩阵，其他值统一为 INVALID_COST。
                        costs[i][j] = 1e6;
                    }
                }
                std::shuffle(candidates[i].begin(), candidates[i].end(), random);
            }
            Check(perception_tracking_detail::AssignCandidates(candidates, columns) ==
                  perception_tracking_detail::Assign(costs, columns),
                  "sparse components match full Hungarian including ties, shuffled edges and changing capacities");
        }
    }

    void TestSpatialIndexAndFleet() {
        using namespace perception_tracking_detail;
        SPATIAL_INDEX_S grid;
        std::mt19937 random(716);
        for(int sample = 0; sample < 60; ++sample) {
            const int count = 16 + random() % 300;
            grid.Reset(count);
            std::vector<std::pair<double, double>> points;
            for(int i = 0; i < count; ++i) {
                // 负坐标、整数格边界、重复位置以及稠密目标。
                points.push_back(std::make_pair((int(random() % 129) - 64) * 0.25,
                                                (int(random() % 129) - 64) * 0.25));
                grid.Add(points.back().first, points.back().second, i);
            }
            for(int query = 0; query < 30; ++query) {
                const double x = (int(random() % 129) - 64) * 0.25;
                const double y = (int(random() % 129) - 64) * 0.25;
                const std::int64_t cell_x = static_cast<std::int64_t>(std::floor(x / 4));
                const std::int64_t cell_y = static_cast<std::int64_t>(std::floor(y / 4));
                std::vector<bool> found(count, false);
                bool unique = true, complete = true;
                for(int dx = -1; dx <= 1; ++dx) {
                    for(int dy = -1; dy <= 1; ++dy) {
                        for(int j = grid.Head(cell_x + dx, cell_y + dy); j >= 0; j = grid.next[j]) {
                            unique = unique && !found[j];
                            found[j] = true;
                        }
                    }
                }
                for(int i = 0; i < count; ++i) {
                    if(std::hypot(points[i].first - x, points[i].second - y) <= 4) {
                        complete = complete && found[i];
                    }
                }
                Check(unique && complete, "grid is a duplicate-free superset of exhaustive four-metre neighbours");
            }
        }
        grid.generation = std::numeric_limits<std::size_t>::max();
        grid.Reset(16);
        Check(grid.Head(0, 0) == -1 && grid.Head(-1, -1) == -1, "grid generation wrap invalidates old buckets");
        Check(!SPATIAL_INDEX_S::CanIndex(1e30, 0), "extreme finite coordinates use the scan fallback");

        for(bool extreme : {false, true}) {
            PerceptionObjectTracker tracker, other;
            std::vector<int> ids(64);
            for(int frame = 0; frame < 60; ++frame) {
                std::vector<robot::object> objects;
                for(int i = 0; i < 64; ++i) {
                    const double x = extreme ? 1e20 : (i % 8 - 4) * 20 + 3.95 + frame * 0.025;
                    const double y = (i / 8 - 4) * 20 - 0.05 + (extreme ? i % 8 * 200 : frame * 0.05);
                    objects.push_back(Box(x, y));
                }
                if(frame % 2 != 0) {
                    std::reverse(objects.begin(), objects.end());
                }
                PerceptionObjectTracker candidate = tracker;
                const auto output = Update(candidate, frame * 0.05, objects);
                tracker = std::move(candidate);
                for(int i = 0; i < 64; ++i) {
                    const auto& object = output.objs[frame % 2 == 0 ? i : 63 - i];
                    if(frame == 0) {
                        ids[i] = object.id;
                    }
                    Check(object.id == ids[i], "grid crossing, reordered detections and tracker copies retain identity");
                    if(!extreme && frame >= 20) {
                        Near(object.vx, 0.5, 0.005, "grid fleet preserves east velocity");
                        Near(object.vy, 1.0, 0.005, "grid fleet preserves north velocity");
                    }
                }
                Update(other, frame * 0.05, {Box(frame * 0.1, 0)});
            }
        }
    }

    void Benchmark() {
        for(int count : {100, 300}) {
            for(int new_percent : {0, 50}) {
                PerceptionObjectTracker tracker;
                std::vector<double> milliseconds;
                // 20Hz,先填满两秒轨迹窗口;50% 每帧新生覆盖实车抖动/漏检时的分配压力。
                for(int frame = 0; frame < 150; ++frame) {
                    robot::perception input;
                    for(int i = 0; i < count; ++i) {
                        double x = i * 6 + frame * 0.1;
                        if(new_percent != 0 && i % 2 == 0) {
                            x += frame * 10000.0;
                        }
                        input.objs.push_back(Box(x, i % 3 * 6));
                    }
                    const auto start = std::chrono::steady_clock::now();
                    PerceptionObjectTracker candidate = tracker;
                    candidate.Update(input, frame * 0.05);
                    tracker = std::move(candidate);
                    const auto stop = std::chrono::steady_clock::now();
                    if(frame >= 50) {
                        milliseconds.push_back(std::chrono::duration<double, std::milli>(stop - start).count());
                    }
                }
                std::sort(milliseconds.begin(), milliseconds.end());
                std::printf("tracking objects=%d new_percent=%d p50_ms=%.4f p99_ms=%.4f max_ms=%.4f\n", count, new_percent,
                            milliseconds[50], milliseconds[98], milliseconds.back());
            }
        }
    }
}

int main(int argc, char**) {
    try {
        if(argc > 1) {
            Benchmark();
            return 0;
        }
        TestDirectionsAndLifetime();
        TestStationaryAndStopping();
        TestIrregularTimeAndMapPrecision();
        TestPredictionAndCrossing();
        TestAssignmentAndIds();
        TestTrackedConfidence();
        TestSparseAssignment();
        TestSpatialIndexAndFleet();
        std::printf("PASS: %d tracking and velocity checks\n", checks);
    } catch(const std::exception& error) {
        std::fprintf(stderr, "FAIL after %d checks: %s\n", checks, error.what());
        return 1;
    }
    return 0;
}
