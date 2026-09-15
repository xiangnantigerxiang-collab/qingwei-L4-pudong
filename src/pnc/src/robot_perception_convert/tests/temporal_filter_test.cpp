#include "../perception_temporal_filter.inc"
#include <chrono>
#include <cstdio>
#include <limits>
#include <random>
#include <sstream>

namespace {
    typedef std::vector<std::pair<robot::perception, double>> History;
    int checks = 0;

    void Check(bool tCondition, const char* tMessage) {
        ++checks;
        if(!tCondition) {
            throw std::runtime_error(tMessage);
        }
    }

    void Near(double tActual, double tExpected, const char* tMessage) {
        Check(std::isfinite(tActual) && std::abs(tActual - tExpected) < 1e-6, tMessage);
    }

    robot::object Box(float tX = 0, float tY = 0, float tDx = 2, float tDy = 2, int tId = 1) {
        robot::object box;
        box.id = tId;
        box.type = 2;
        box.x = tX;
        box.y = tY;
        box.dx = tDx;
        box.dy = tDy;
        box.heading = 321;
        box.height = 1.5;
        box.vx = 0.7;
        box.vy = -0.3;
        box.confidence = -5;  // 传入置信度不参与累计，避免把上次滤波结果重复当作证据。
        box.polygons.resize(1);
        box.polygons[0].x = 987;
        return box;
    }

    void AddFrame(History& tHistory, double tTimestamp, const std::vector<robot::object>& tBoxes) {
        robot::perception message;
        message.header.seq = tHistory.size() + 1;
        message.header.stamp = ros::Time(tTimestamp);
        message.header.frame_id = "map";
        message.objs = tBoxes;
        tHistory.push_back(std::make_pair(message, tTimestamp));
    }

    std::string Snapshot(const History& tHistory) {
        std::ostringstream stream;
        stream.precision(17);
        for(std::size_t i = 0; i < tHistory.size(); ++i) {
            const auto& message = tHistory[i].first;
            stream << tHistory[i].second << ',' << message.header.seq << ','
                   << message.header.stamp.toSec() << ',' << message.header.frame_id << ';';
            for(std::size_t j = 0; j < message.objs.size(); ++j) {
                const auto& box = message.objs[j];
                stream << box.id << ',' << int(box.type) << ',' << box.x << ',' << box.y << ','
                       << box.dx << ',' << box.dy << ',' << box.heading << ',' << box.height << ','
                       << box.vx << ',' << box.vy << ',' << box.confidence << ':';
                for(std::size_t k = 0; k < box.polygons.size(); ++k) {
                    stream << box.polygons[k].x << ',' << box.polygons[k].y << ',' << box.polygons[k].z << '/';
                }
            }
        }
        return stream.str();
    }

    void TestTimeAndConfidence() {
        ros::testTime() = 100;
        Check(FilterPerceptionHistory(History()).objs.empty(), "empty history");
        History history;
        AddFrame(history, 97.999, {Box(0, 0, 0, 0)});
        Check(FilterPerceptionHistory(history).objs.empty(), "prune expired frame before validating boxes");
        history.clear();
        AddFrame(history, 98, {Box()});
        AddFrame(history, 100, {});
        Check(FilterPerceptionHistory(history).objs.empty(), "exactly two seconds is expired");
        history[0].second = 98.0001;
        auto result = FilterPerceptionHistory(history);
        Check(result.objs.size() == 1, "less than two seconds is retained");
        Near(result.objs[0].confidence, std::exp(-1.9999) / (std::exp(-1.9999) + 1), "empty frame lowers confidence");
        Near(result.header.stamp.toSec(), 100, "latest input header");

        history.clear();
        AddFrame(history, 99, {Box(0, 0, 2, 2, 11)});
        auto latest = Box(0.2, 0, 3, 2, 19);
        latest.type = 4;
        latest.heading = 13;
        latest.confidence = std::numeric_limits<float>::quiet_NaN();
        AddFrame(history, 99.8, {latest});
        std::reverse(history.begin(), history.end());
        std::string snapshot = Snapshot(history);
        result = FilterPerceptionHistory(history);
        Check(result.objs.size() == 1 && result.objs[0].id == 19, "sort chronologically; do not associate by id");
        Near(result.objs[0].confidence, 1, "persistent track confidence");
        Near(result.objs[0].x, latest.x, "latest position retained without averaging");
        Near(result.objs[0].dx, latest.dx, "latest dimensions retained");
        Check(result.objs[0].type == 4 && result.objs[0].heading == 13, "latest HDMap type and heading retained");
        Check(result.objs[0].height == latest.height && result.objs[0].vx == latest.vx &&
                  result.objs[0].vy == latest.vy && result.objs[0].polygons.size() == 1 &&
                  result.objs[0].polygons[0].x == 987,
              "other fields retained");
        Check(Snapshot(history) == snapshot, "input history unchanged");

        history.clear();
        AddFrame(history, 99, {Box()});
        for(int i = 0; i < 100; ++i) {
            AddFrame(history, 99, {Box()});
        }
        AddFrame(history, 100, {});
        result = FilterPerceptionHistory(history);
        Near(result.objs[0].confidence, std::exp(-1) / (std::exp(-1) + 1), "duplicate timestamps do not multiply confidence");
        AddFrame(history, 99, {});
        Check(FilterPerceptionHistory(history).objs.empty(), "last duplicate timestamp wins");

        history.clear();
        AddFrame(history, 100.01, {Box(0, 0, 0, 0)});
        AddFrame(history, std::numeric_limits<double>::quiet_NaN(), {Box(0, 0, 0, 0)});
        AddFrame(history, std::numeric_limits<double>::infinity(), {Box(0, 0, 0, 0)});
        AddFrame(history, -1, {Box(0, 0, 0, 0)});
        AddFrame(history, 99.5, {Box()});
        result = FilterPerceptionHistory(history);
        Check(result.objs.size() == 1, "invalid and future timestamps skipped");
        Near(result.objs[0].confidence, 1, "invalid frames excluded from denominator");
        ros::testTime() = 102.1;
        Check(FilterPerceptionHistory(history).objs.empty(), "old track expires");
        ros::testTime() = 0;
        history.clear();
        AddFrame(history, 0, {Box()});
        Near(FilterPerceptionHistory(history).objs[0].confidence, 1, "simulation time zero is valid");
        ros::testTime() = 100;

        history.clear();
        for(int i = 0; i < 10; ++i) {
            std::vector<robot::object> boxes{Box(0, 0, 2, 2, 1)};
            if(i == 0) {
                boxes.push_back(Box(5, 0, 2, 2, 2));
            }
            if(i == 9) {
                boxes.push_back(Box(10, 0, 2, 2, 3));
            }
            AddFrame(history, 98.2 + i * 0.2, boxes);
        }
        result = FilterPerceptionHistory(history);
        Check(result.objs.size() == 3, "no implicit confidence deletion threshold");
        Near(result.objs[0].confidence, 1, "persistent target keeps unit confidence");
        Check(result.objs[1].confidence < result.objs[2].confidence && result.objs[2].confidence < 0.25,
              "recent isolated detection has more weight but stays below stable target");
    }

    std::size_t PairCount(const robot::object& tA, const robot::object& tB) {
        History history;
        AddFrame(history, 99, {tA});
        AddFrame(history, 100, {tB});
        return FilterPerceptionHistory(history).objs.size();
    }

    void TestAssociation() {
        Check(PairCount(Box(), Box(1, 0)) == 1, "one metre and exactly 50 percent match");
        Check(PairCount(Box(0, 0, 1, 1), Box(0.5, 0, 1, 1)) == 1, "dx dy are full dimensions");
        Check(PairCount(Box(0, 0, 1, 1), Box(0.5001, 0, 1, 1)) == 2, "below 50 percent does not match");
        Check(PairCount(Box(0, 0, 10, 10), Box(1.0001, 0, 10, 10)) == 2, "distance gate even when overlap is large");
        Check(PairCount(Box(0, 0, 1, 1), Box(0.8, 0, 4, 4)) == 1, "50 percent of either box; not IoU");
        Check(PairCount(Box(0, 0, 1, 1), Box(1, 0, 1, 1)) == 2, "boundary contact has no area");
        Check(PairCount(Box(0, 0, 0.1, 4), Box(0, 0, 4, 0.1)) == 2, "same center alone does not match");
        Check(PairCount(Box(0, 0, 1, 1), Box(0.7, 0.7, 1, 1)) == 2, "small diagonal overlap does not match");
        Check(PairCount(Box(-0.1, -0.1), Box(0.5, 0.4)) == 1, "negative cells and diagonal neighbor query");
        Check(PairCount(Box(1000000, -1000000), Box(1000001, -1000000)) == 1, "large map coordinates");
        auto first = Box(0, 0, 4, 0.2);
        auto second = first;
        first.heading = 0;
        second.heading = 90;
        second.polygons.clear();
        Check(PairCount(first, second) == 1, "heading and polygons do not affect axis-aligned association");
        Check(PairCount(Box(0, 0, 2, 2, 8), Box(5, 0, 2, 2, 8)) == 2, "same id does not force association");

        History history;
        AddFrame(history, 98.1, {Box(-3, 0)});
        for(int i = 1; i <= 6; ++i) {
            AddFrame(history, 98.1 + i * 0.3, {Box(-3 + 0.9 * i, 0)});
        }
        auto result = FilterPerceptionHistory(history);
        Check(result.objs.size() == 1, "associate with previous observation instead of original cluster center");
        Near(result.objs[0].x, 2.4, "track moves across grid cells");
        Near(result.objs[0].confidence, 1, "moving track confidence");

        history.clear();
        AddFrame(history, 98.2, {Box()});
        AddFrame(history, 99, {});
        AddFrame(history, 100, {Box(0.3, 0)});
        result = FilterPerceptionHistory(history);
        Check(result.objs.size() == 1, "reconnect last observation after a missed frame");
        Near(result.objs[0].confidence, (std::exp(-1.8) + 1) / (std::exp(-1.8) + std::exp(-1) + 1),
             "missed frame lowers track confidence");

        history.clear();
        AddFrame(history, 99, {Box()});
        AddFrame(history, 100, {Box(-0.1, 0, 2, 2, 2), Box(0.1, 0, 2, 2, 3)});
        result = FilterPerceptionHistory(history);
        Check(result.objs.size() == 2, "one earlier target cannot swallow two current targets");
        Near(result.objs[0].confidence, 1, "one score per target per frame");
        Near(result.objs[1].confidence, 1 / (std::exp(-1) + 1), "unmatched observation starts its own track");

        history.clear();
        AddFrame(history, 99, {Box(-0.3, 0, 2, 2, 1), Box(0.3, 0, 2, 2, 2)});
        AddFrame(history, 100, {Box(0.2, 0, 2, 2, 20), Box(-0.2, 0, 2, 2, 10)});
        result = FilterPerceptionHistory(history);
        Check(result.objs.size() == 2 && result.objs[0].id == 10 && result.objs[1].id == 20,
              "nearest one-to-one association despite reversed observation order");
    }

    void TestInvalidGeometry() {
        for(int i = 0; i < 6; ++i) {
            auto box = Box();
            if(i == 0) {
                box.dx = 0;
            } else if(i == 1) {
                box.dy = -1;
            } else if(i == 2) {
                box.x = std::numeric_limits<float>::quiet_NaN();
            } else if(i == 3) {
                box.y = std::numeric_limits<float>::infinity();
            } else if(i == 4) {
                box.dx = std::numeric_limits<float>::infinity();
            } else {
                box.x = std::numeric_limits<float>::max();
            }
            History history;
            AddFrame(history, 100, {box});
            bool rejected = false;
            try {
                FilterPerceptionHistory(history);
            } catch(const std::invalid_argument&) {
                rejected = true;
            }
            Check(rejected, "invalid geometry must fail explicitly");
        }
    }

    void TestRandomHistories() {
        // 构造已知真实身份且彼此相隔的目标：随机漏检、抖动和帧内/帧间乱序，独立累加预期权重。
        std::mt19937 random(20260915);
        std::uniform_real_distribution<float> jitter(-0.2, 0.2);
        for(int scenario = 0; scenario < 100; ++scenario) {
            const int count = 80;
            History history;
            std::vector<double> expected(count, 0);
            std::vector<robot::object> latest(count);
            double total_weight = 0;
            for(int frame = 0; frame < 25; ++frame) {
                double timestamp = 98.08 + frame * 0.08;
                double weight = std::exp(timestamp - 100);
                total_weight += weight;
                std::vector<robot::object> boxes;
                for(int id = 0; id < count; ++id) {
                    if(frame != 0 && random() % 4 == 0) {
                        continue;
                    }
                    auto box = Box((id % 10 - 5) * 4 + jitter(random), (id / 10 - 4) * 4 + jitter(random),
                                   2, 2, id);
                    box.type = frame % 5;
                    boxes.push_back(box);
                    expected[id] += weight;
                    latest[id] = box;
                }
                std::shuffle(boxes.begin(), boxes.end(), random);
                AddFrame(history, timestamp, boxes);
            }
            std::shuffle(history.begin(), history.end(), random);
            auto result = FilterPerceptionHistory(history);
            Check(result.objs.size() == count, "random histories preserve distinct ground-truth targets");
            std::vector<bool> seen(count, false);
            for(std::size_t i = 0; i < result.objs.size(); ++i) {
                const auto& box = result.objs[i];
                int id = box.id;
                Check(id >= 0 && id < count && !seen[id], "unique random target");
                seen[id] = true;
                Near(box.confidence, expected[id] / total_weight, "random time-weighted confidence");
                Check(box.x == latest[id].x && box.y == latest[id].y && box.type == latest[id].type,
                      "random target uses latest geometry and lane type");
            }
        }
    }

    void TestGridAgainstFullScan() {
        using namespace perception_filter_detail;
        std::mt19937 random(42);
        std::uniform_real_distribution<float> position(-10, 10);
        std::uniform_real_distribution<float> size(0.1, 4);
        std::vector<TRACK_S> tracks;
        Grid grid;
        for(int i = 0; i < 300; ++i) {
            auto object = Box(position(random), position(random), size(random), size(random));
            tracks.push_back(TRACK_S{nullptr, ReadBox(object), 1, 0, 0, CELL_S{0, 0}});
            AddToGrid(grid, tracks, tracks.size() - 1);
        }
        for(int step = 0; step < 100; ++step) {
            // 覆盖同格交换删除与迁移到已有/新格，不依赖函数的网格查询来构造参考答案。
            for(int i = 0; i < 30; ++i) {
                std::size_t index = random() % tracks.size();
                tracks[index].box = ReadBox(Box(position(random), position(random), size(random), size(random)));
                MoveInGrid(grid, tracks, index);
            }
            for(int query = 0; query < 30; ++query) {
                BOX_S box = ReadBox(Box(position(random), position(random), size(random), size(random)));
                auto matches = FindMatches(std::vector<BOX_S>{box}, tracks, grid);
                std::vector<std::size_t> actual, expected;
                for(std::size_t i = 0; i < matches.size(); ++i) {
                    actual.push_back(matches[i].track);
                }
                for(std::size_t i = 0; i < tracks.size(); ++i) {
                    const auto& other = tracks[i].box;
                    double width = std::max(0.0, std::min(box.x + box.half_x, other.x + other.half_x) -
                                                     std::max(box.x - box.half_x, other.x - other.half_x));
                    double height = std::max(0.0, std::min(box.y + box.half_y, other.y + other.half_y) -
                                                      std::max(box.y - box.half_y, other.y - other.half_y));
                    if(std::hypot(box.x - other.x, box.y - other.y) <= 1 &&
                       width * height > 0 && width * height >= 0.5 * std::min(box.area, other.area)) {
                        expected.push_back(i);
                    }
                }
                std::sort(actual.begin(), actual.end());
                Check(actual == expected, "spatial grid candidates agree with exhaustive scan");
            }
        }
    }

    void Benchmark() {
        for(int count : {100, 300}) {
            for(int frames : {40, 200}) {
                History history;
                for(int frame = 0; frame < frames; ++frame) {
                    std::vector<robot::object> boxes;
                    for(int id = 0; id < count; ++id) {
                        boxes.push_back(Box((id % 20) * 4 + (frame % 3) * 0.1, (id / 20) * 4, 2, 2, id));
                    }
                    AddFrame(history, 98.0 + (frame + 1) * 2.0 / frames, boxes);
                }
                std::vector<double> samples;
                for(int repetition = 0; repetition < 110; ++repetition) {
                    auto start = std::chrono::steady_clock::now();
                    auto result = FilterPerceptionHistory(history);
                    auto end = std::chrono::steady_clock::now();
                    Check(result.objs.size() == static_cast<std::size_t>(count), "benchmark track count");
                    if(repetition >= 10) {
                        samples.push_back(std::chrono::duration<double, std::milli>(end - start).count());
                    }
                }
                std::sort(samples.begin(), samples.end());
                std::printf("BENCH frames=%d objects_per_frame=%d observations=%d p50_ms=%.4f p99_ms=%.4f max_ms=%.4f\n",
                            frames, count, frames * count, samples[49], samples[98], samples.back());
            }
        }
    }
}

int main(int argc, char** argv) {
    try {
        if(argc > 1 && std::string(argv[1]) == "--benchmark") {
            Benchmark();
        } else {
            TestTimeAndConfidence();
            TestAssociation();
            TestInvalidGeometry();
            TestRandomHistories();
            TestGridAgainstFullScan();
        }
        std::printf("PASS: %d temporal filter checks\n", checks);
    } catch(const std::exception& error) {
        std::fprintf(stderr, "FAIL after %d checks: %s\n", checks, error.what());
        return 1;
    }
    return 0;
}
