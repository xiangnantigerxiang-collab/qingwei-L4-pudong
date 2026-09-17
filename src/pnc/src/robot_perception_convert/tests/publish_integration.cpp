#define main PerceptionConvertMain
#include "../perception_msg_convert.cpp"
#undef main
#include "../../robot_path_plan/safety/perception_safety.inc"

#include "hdmap/hdmap_server.h"
#include <cstdio>
#include <limits>
#include <sys/stat.h>
#include <unistd.h>

namespace {
    int checks = 0;
    std::vector<robot::perception> publications;

    void Check(bool tCondition, const char* tMessage) {
        ++checks;
        if(!tCondition) {
            throw std::runtime_error(tMessage);
        }
    }

    void Near(double tActual, double tExpected, const char* tMessage) {
        Check(std::isfinite(tActual) && std::abs(tActual - tExpected) < 1e-5, tMessage);
    }

    struct Fixture {
        std::string root, directory;
        std::vector<std::string> files;
        Fixture() {
            char path[] = "/tmp/perception_filter_publish_XXXXXX";
            char* created = mkdtemp(path);
            Check(created != nullptr, "create map fixture");
            root = created;
            directory = root + "/map_processed";
            Check(mkdir(directory.c_str(), 0700) == 0, "create map directory");
            for(int lane = 0; lane < 3; ++lane) {
                hdmap::MapPointList points;
                for(int i = 0; i <= 200; ++i) {
                    points.push_back(hdmap::MAP_POINT_S(i * 0.5, lane * 4, 90));
                    points.back().dist_origin = i * 0.5;
                    points.back().p2pDistance = i == 0 ? 0 : 0.5;
                }
                files.push_back(directory + "/lane" + std::to_string(lane) + ".csv");
                Check(hdmap::HdMapServer().SaveTrajectory(files.back(), points, hdmap::DIRECTION_FORWARD).IsOk(),
                      "save lane fixture");
            }
        }
        ~Fixture() {
            for(std::size_t i = 0; i < files.size(); ++i) {
                std::remove(files[i].c_str());
            }
            rmdir(directory.c_str());
            rmdir(root.c_str());
        }
    };

    void SetPose(double tX, double tY, double tHeading) {
        ivlocmsg::ivmsglocpos navigation;
        navigation.xg = tX;
        navigation.yg = tY;
        navigation.heading = tHeading;
        GNSSCallback(navigation);
    }

    visualization_msgs::Marker Marker(double tX, double tY, double tDx = 2, double tDy = 2) {
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose.position.x = tX;
        marker.pose.position.y = tY;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        marker.scale.x = tDx;
        marker.scale.y = tDy;
        return marker;
    }

    void Send(double tTimestamp, const std::vector<visualization_msgs::Marker>& tMarkers,
              bool tRefreshNavigation = true) {
        ros::testTime() = tTimestamp;
        if(tRefreshNavigation) {
            // 正常回归模拟定位持续输入，且把一次性时钟故障留给随后真实感知回调。
            int fail_after = ros::Time::testFailAfter();
            int change_after = ros::Time::testChangeAfter();
            ros::Time::testFailAfter() = -1;
            ros::Time::testChangeAfter() = -1;
            ivlocmsg::ivmsglocpos navigation;
            navigation.xg = mGPS.xAxis;
            navigation.yg = mGPS.yAxis;
            navigation.zg = mGPS.zAxis;
            navigation.heading = mGPS.heading;
            GNSSCallback(navigation);
            ros::Time::testFailAfter() = fail_after;
            ros::Time::testChangeAfter() = change_after;
        }
        visualization_msgs::MarkerArray message;
        message.markers = tMarkers;
        BoxMsgCallBack(message);
    }

    void Expect(double tTimestamp, const std::vector<visualization_msgs::Marker>& tMarkers,
                const std::vector<int>& tTypes, const std::vector<double>& tConfidence) {
        std::size_t previous = publications.size();
        Send(tTimestamp, tMarkers);
        Check(publications.size() == previous + 1, "one filtered publish per valid callback");
        const auto& message = publications.back();
        Check(message.objs.size() == tTypes.size() && tTypes.size() == tConfidence.size(), "filtered object count");
        Near(message.header.stamp.toSec(), tTimestamp, "published acquisition timestamp");
        std::unordered_set<int> ids;
        for(std::size_t i = 0; i < message.objs.size(); ++i) {
            Check(message.objs[i].type == tTypes[i], "current-pose lane classification must precede publish");
            Near(message.objs[i].confidence, tConfidence[i], "confidence must be calculated before publish");
            Check(message.objs[i].polygons.empty(), "internal world corners must not change published polygons");
            Check(message.objs[i].id > 0 && ids.insert(message.objs[i].id).second, "published IDs are positive and unique");
            Check(std::isfinite(message.objs[i].vx) && std::isfinite(message.objs[i].vy) &&
                      message.objs[i].heading >= 0 && message.objs[i].heading < 360,
                  "published velocity is finite and heading uses navigation range");
        }
    }

    void TestRawHistoryAndMisses() {
        mPerceptionHistory.clear();
        SetPose(10, 0, 90);
        Expect(100, {Marker(10, 0), Marker(10, 4)}, {0, 1}, {1, 1});
        const double decay = std::exp(-0.5);
        Expect(100.5, {Marker(10.2, 0), Marker(10, 8)}, {0, 1, 2}, {1, decay / (decay + 1), 1 / (decay + 1)});
        Near(publications.back().objs[0].x, 20.2, "latest position retained");
        Check(mPerceptionHistory.size() == 2 && mPerceptionHistory.back().first.objs.size() == 2,
              "cache raw observations rather than three filtered targets");
        Check(mPerceptionHistory.back().first.objs[1].type == 2, "cached observations already classified");
        Near(mPerceptionHistory.back().first.objs[0].confidence, 0, "filtered confidence does not feed back into cache");
        const double total = std::exp(-1) + decay + 1;
        Expect(101, {}, {0, 1, 2}, {(std::exp(-1) + decay) / total, std::exp(-1) / total, decay / total});
        Check(mPerceptionHistory.back().first.objs.empty(), "empty raw frame retained in denominator");
        const double remaining = std::exp(-1.5) / (std::exp(-1.5) + std::exp(-1) + 1);
        Expect(102, {}, {}, {});
        const auto unthresholded = FilterTrackedPerceptionHistory(mPerceptionHistory);
        Check(unthresholded.objs.size() == 2, "thresholded publication does not erase valid raw history");
        Near(unthresholded.objs[0].confidence, remaining, "first unpublished target retains original temporal evidence");
        Near(unthresholded.objs[1].confidence, remaining, "second unpublished target retains original temporal evidence");
        Check(mPerceptionHistory.front().second == 100.5, "exact two-second prefix removed");
        // 若将上一帧滤波结果回灌，已经消失的左一目标会被错误地继续保留。
        Expect(102.5, {}, {}, {});
        Check(mPerceptionHistory.front().second == 101, "last real detections expire despite intermediate publications");
    }

    void TestTimeAndBoundedCache() {
        mPerceptionHistory.clear();
        Expect(200, {}, {}, {});
        Expect(200, {Marker(10, 0)}, {0}, {1});
        Check(mPerceptionHistory.size() == 1, "same timestamp replaces empty frame");
        for(int i = 0; i < 100; ++i) {
            Expect(200, {}, {}, {});
            Check(mPerceptionHistory.size() == 1, "paused clock cannot grow history");
        }
        Expect(200.5, {Marker(10, 0)}, {0}, {1 / (std::exp(-0.5) + 1)});
        Expect(199.9, {}, {}, {});
        Check(mPerceptionHistory.size() == 1 && mPerceptionHistory.front().second == 199.9,
              "clock rollback resets the previous playback period");

        mPerceptionHistory.clear();
        for(int i = 0; i < 350; ++i) {
            double timestamp = 300 + i * 0.01;
            Expect(timestamp, {}, {}, {});
            Check(mPerceptionHistory.size() <= 201, "two-second cache is bounded at 100 Hz");
            Check(timestamp - mPerceptionHistory.front().second < 2, "cache contains no expired prefix");
        }
    }

    void TestFailuresAndRecovery() {
        mPerceptionHistory.clear();
        Expect(400, {Marker(10, 0)}, {0}, {1});
        std::size_t previous = publications.size();
        Send(400.2, {Marker(10, 0, 0, 2)});
        Check(publications.size() == previous && mPerceptionHistory.size() == 1, "HDMap failure neither publishes nor enters history");
        Near(mPerception.header.stamp.toSec(), 400, "failed conversion does not replace published snapshot");
        Near(mPerception.objs[0].dx, 2, "bad box never replaces cached valid geometry");
        Expect(400.5, {Marker(10.2, 0)}, {0}, {1});
        previous = publications.size();
        // 故障注入：回调首次取时正常，滤波内部取时失败，检查同时间戳帧的事务回滚。
        ros::Time::testFailAfter() = 1;
        Send(400.5, {Marker(10.3, 0)});
        Check(publications.size() == previous && mPerceptionHistory.size() == 2,
              "filter failure rolls back new duplicate and preserves valid history");
        Near(mPerceptionHistory.back().first.objs[0].x, 20.2, "failed duplicate does not replace valid observation");
        Near(mPerception.objs[0].x, 20.2, "filter failure keeps the last published snapshot");
        Expect(401, {Marker(10.4, 0)}, {0}, {1});
        previous = publications.size();
        Send(std::numeric_limits<double>::quiet_NaN(), {});
        Check(publications.size() == previous && mPerceptionHistory.empty() && mPerception.objs.empty(),
              "invalid callback time clears local state without publishing");
        mGPS.zAxis = std::numeric_limits<float>::quiet_NaN();
        Send(401.5, {});
        Check(publications.size() == previous && mPerceptionHistory.empty(), "invalid localization does not count as a missed detection");
        SetPose(10, 0, 90);
        Expect(403, {}, {}, {});
        Check(mPerceptionHistory.size() == 1, "recovery prunes old history");
    }

    void TestLocalizationValidity() {
        mPerceptionHistory.clear();
        mPerception = robot::perception();
        mGPS = robot::navigation_msg();
        mNavigationReceived = false;
        std::size_t previous = publications.size();
        Send(600, {Marker(10, 0)}, false);
        Send(600.1, {}, false);
        Check(publications.size() == previous && mPerceptionHistory.empty(),
              "neither empty nor occupied frame publishes before first localization");
        SetPose(10, 0, 90);
        Send(600.2, {Marker(10, 0)}, false);
        Check(publications.size() == previous + 1, "first valid localization enables publication");

        mPerceptionHistory.clear();
        ros::testTime() = 610;
        SetPose(10, 0, 90);
        Send(611.999, {Marker(10, 0)}, false);
        Check(publications.size() == previous + 2, "localization younger than two seconds is accepted");
        previous = publications.size();
        Send(612, {}, false);
        Check(publications.size() == previous && mPerceptionHistory.size() == 1,
              "exactly two-second-old localization rejects empty frame instead of lowering confidence");
        Send(613.999, {Marker(10, 0)}, false);
        Check(publications.size() == previous && mPerceptionHistory.empty() && mPerception.objs.empty(),
              "stale localization cannot prevent perception expiration");

        ros::testTime() = 620;
        SetPose(10, 0, 90);
        Send(619.9, {Marker(10, 0)}, false);
        Check(publications.size() == previous && mPerceptionHistory.empty(), "future localization after clock rollback is rejected");
        mNavigationTimestamp = std::numeric_limits<double>::quiet_NaN();
        Send(620, {}, false);
        Check(publications.size() == previous && mPerceptionHistory.empty(), "nonfinite localization timestamp is rejected");
        mNavigationTimestamp = -1;
        Send(0, {}, false);
        Check(publications.size() == previous && mPerceptionHistory.empty(), "negative localization timestamp is rejected");

        float robot::navigation_msg::*fields[] = {&robot::navigation_msg::xAxis, &robot::navigation_msg::yAxis};
        for(auto field : fields) {
            SetPose(10, 0, 90);
            mGPS.*field = std::numeric_limits<double>::quiet_NaN();
            Send(621, {Marker(10, 0)});
            Check(publications.size() == previous && mPerceptionHistory.empty(), "nonfinite position is rejected");
        }
        SetPose(10, 0, std::numeric_limits<double>::infinity());
        Send(621.1, {});
        Check(publications.size() == previous && mPerceptionHistory.empty(), "nonfinite heading rejects even empty frames");
        SetPose(10, 0, 90);
        Expect(622, {Marker(10, 0)}, {0}, {1});
    }

    void TestSilentExpiryAndRecovery() {
        mPerceptionHistory.clear();
        Expect(700, {Marker(10, 0)}, {0}, {1});
        std::size_t previous = publications.size();
        MaintainPerceptionHistory(701.999);
        Check(mPerceptionHistory.size() == 1 && mPerception.objs.size() == 1, "silent history remains valid before two seconds");
        MaintainPerceptionHistory(702);
        Check(mPerceptionHistory.empty() && mPerception.objs.empty(), "silent upstream clears raw and computed history at two seconds");
        MaintainPerceptionHistory(703);
        Check(publications.size() == previous, "silent cleanup never refreshes downstream receive watchdog with empty publications");
        Expect(703.1, {Marker(11, 0)}, {0}, {1});

        mPerceptionHistory.clear();
        Expect(710, {Marker(10, 0)}, {0}, {1});
        Expect(710.5, {Marker(10, 4)}, {0, 1}, {std::exp(-0.5) / (std::exp(-0.5) + 1), 1 / (std::exp(-0.5) + 1)});
        previous = publications.size();
        MaintainPerceptionHistory(712);
        Check(mPerceptionHistory.size() == 1 && mPerceptionHistory[0].second == 710.5,
              "periodic maintenance keeps only still-valid raw history");
        Check(mPerception.objs.empty() && publications.size() == previous,
              "partial expiration invalidates computed snapshot without synthetic publish");
        Expect(712.1, {Marker(10, 4)}, {1}, {1});

        mPerceptionHistory.clear();
        Expect(720, {Marker(10, 0)}, {0}, {1});
        previous = publications.size();
        Send(721, {Marker(10, 0, 0, 2)});
        Send(722, {Marker(10, 0, 0, 2)});
        Check(publications.size() == previous && mPerceptionHistory.empty() && mPerception.objs.empty(),
              "repeated invalid boxes cannot block expiration or publish a clear road");
        Expect(722.1, {}, {}, {});

        mPerceptionHistory.clear();
        Expect(730, {Marker(10, 0)}, {0}, {1});
        previous = publications.size();
        MaintainPerceptionHistory(729);
        Check(publications.size() == previous && mPerceptionHistory.empty() && mPerception.objs.empty(),
              "clock rollback without callbacks clears both local caches");
        Expect(729.1, {}, {}, {});
        MaintainPerceptionHistory(std::numeric_limits<double>::infinity());
        Check(mPerceptionHistory.empty() && mPerception.header.stamp.toSec() == 0,
              "invalid maintenance time clears local metadata too");
    }

    void TestClockChangesDuringProcessing() {
        // 第一次 now 是接收时间，第二次在滤波中，第三次在发布前。
        for(int stage = 1; stage <= 2; ++stage) {
            for(double jump : {-0.1, 2.0, 10.0}) {
                mPerceptionHistory.clear();
                Expect(800, {Marker(10, 0)}, {0}, {1});
                std::size_t previous = publications.size();
                ros::Time::testChangeAfter() = stage;
                ros::Time::testChangedTime() = 800.5 + jump;
                Send(800.5, {Marker(10, 4)});
                Check(publications.size() == previous && mPerceptionHistory.empty() && mPerception.objs.empty(),
                      "clock change inside filter or before publish rejects stale and default results");
                Expect(801, {Marker(10, 8)}, {2}, {1});
            }
        }
        mPerceptionHistory.clear();
        Expect(810, {Marker(10, 0)}, {0}, {1});
        std::size_t previous = publications.size();
        ros::Time::testFailAfter() = 2;
        Send(810.5, {});
        Check(publications.size() == previous && mPerceptionHistory.empty() && mPerception.objs.empty(),
              "nonfinite time immediately before publish clears state and rejects output");
        // 仿真从零开始时，默认空 header 与真实帧时间戳同为零，也不能吞掉非空输入。
        ros::Time::testChangeAfter() = 1;
        ros::Time::testChangedTime() = 2;
        Send(0, {Marker(10, 0)});
        Check(publications.size() == previous && mPerceptionHistory.empty() && mPerception.objs.empty(),
              "zero timestamp cannot disguise discarded nonempty input as a valid default empty result");
        Expect(0, {Marker(10, 0)}, {0}, {1});
        Expect(0, {}, {}, {});
        for(double invalid_time : {-1.0, std::numeric_limits<double>::infinity(),
                                   -std::numeric_limits<double>::infinity()}) {
            mPerceptionHistory.clear();
            Expect(820, {Marker(10, 0)}, {0}, {1});
            previous = publications.size();
            Send(invalid_time, {});
            Check(publications.size() == previous && mPerceptionHistory.empty() && mPerception.objs.empty(),
                  "negative or infinite receipt time clears state without publishing");
        }
        ros::testTime() = 830;
        SetPose(10, 0, 90);
        Send(831.8, {Marker(10, 0)}, false);
        previous = publications.size();
        ros::Time::testChangeAfter() = 2;
        ros::Time::testChangedTime() = 832.01;
        Send(831.9, {Marker(10, 0)}, false);
        Check(publications.size() == previous && mPerceptionHistory.empty() && mPerception.objs.empty(),
              "localization that expires during otherwise timely processing also blocks publication");
        Expect(833, {Marker(10, 0)}, {0}, {1});
    }

    void TestConfidenceFilter() {
        robot::perception input;
        input.header.seq = 12;
        input.header.stamp = ros::Time(456.75);
        input.header.frame_id = "map";
        const float scores[] = {0.75f, 0.25f, 0.5f, 1.0f};
        for(int i = 0; i < 4; ++i) {
            robot::object object;
            object.id = i + 10;
            object.type = i;
            object.x = 10 + i;
            object.y = 20 + i;
            object.dx = 2 + i;
            object.dy = 3 + i;
            object.heading = 30 + i;
            object.height = 1.5;
            object.vx = 0.5;
            object.vy = -0.5;
            object.confidence = scores[i];
            object.polygons.resize(1);
            object.polygons[0].x = 100 + i;
            object.polygons[0].y = 200 + i;
            object.polygons[0].z = 2 + i;
            input.objs.push_back(object);
        }
        const std::size_t publication_count = publications.size();
        const std::size_t history_count = mPerceptionHistory.size();
        auto output = FilterPerceptionByConfidence(input, 0.5);
        Check(output.objs.size() == 3, "confidence filter removes strictly lower scores");
        Check(output.header.seq == input.header.seq && output.header.stamp.toSec() == 456.75 &&
                  output.header.frame_id == input.header.frame_id,
              "confidence filter preserves full header");
        const std::size_t retained[] = {0, 2, 3};
        for(std::size_t i = 0; i < output.objs.size(); ++i) {
            const auto& actual = output.objs[i];
            const auto& expected = input.objs[retained[i]];
            Check(actual.id == expected.id && actual.type == expected.type && actual.confidence == expected.confidence,
                  "confidence filter preserves order, HDMap type, identity and score");
            Check(actual.x == expected.x && actual.y == expected.y && actual.dx == expected.dx && actual.dy == expected.dy &&
                      actual.heading == expected.heading && actual.height == expected.height &&
                      actual.vx == expected.vx && actual.vy == expected.vy,
                  "confidence filter preserves geometry and velocity");
            Check(actual.polygons.size() == 1 && actual.polygons[0].x == expected.polygons[0].x &&
                      actual.polygons[0].y == expected.polygons[0].y && actual.polygons[0].z == expected.polygons[0].z,
                  "confidence filter preserves polygons");
        }
        output.objs[0].polygons[0].x = -1;
        Check(input.objs.size() == 4 && input.objs[0].polygons[0].x == 100 && input.objs[1].confidence == 0.25f,
              "confidence filter output owns its data and input is unchanged");
        Check(FilterPerceptionByConfidence(input, 0).objs.size() == 4, "zero threshold retains every normalized score");
        Check(FilterPerceptionByConfidence(input, 1).objs.size() == 1, "unit threshold retains exactly unit confidence");
        Check(FilterPerceptionByConfidence(input, -0.1).objs.size() == 4, "finite thresholds are not clamped");
        output = FilterPerceptionByConfidence(input, 1.1);
        Check(output.objs.empty() && output.header.seq == input.header.seq, "all-filtered result retains header");

        robot::perception empty;
        empty.header = input.header;
        output = FilterPerceptionByConfidence(empty, 0.5);
        Check(output.objs.empty() && output.header.stamp.toSec() == 456.75 && output.header.frame_id == "map",
              "empty input preserves acquisition metadata");
        input.objs[0].confidence = std::nextafter(0.5f, 0.0f);
        input.objs[1].confidence = 0.5f;
        input.objs[2].confidence = std::nextafter(0.5f, 1.0f);
        input.objs[3].confidence = 0;
        output = FilterPerceptionByConfidence(input, 0.5);
        Check(output.objs.size() == 2 && output.objs[0].id == 11 && output.objs[1].id == 12,
              "boundary and adjacent float values use exact comparison without tolerance");

        input.objs[0].confidence = std::numeric_limits<float>::quiet_NaN();
        input.objs[1].confidence = std::numeric_limits<float>::infinity();
        input.objs[2].confidence = -std::numeric_limits<float>::infinity();
        input.objs[3].confidence = 1;
        output = FilterPerceptionByConfidence(input, 0);
        Check(output.objs.size() == 1 && output.objs[0].id == 13, "nonfinite confidence never passes filter");
        for(double threshold : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                                -std::numeric_limits<double>::infinity()}) {
            bool rejected = false;
            try {
                FilterPerceptionByConfidence(input, threshold);
            } catch(const std::invalid_argument&) {
                rejected = true;
            }
            Check(rejected, "nonfinite threshold is rejected instead of silently producing an empty result");
        }
        Check(publications.size() == publication_count && mPerceptionHistory.size() == history_count,
              "callable confidence filter neither publishes nor changes temporal history");
    }

    void TestSizeFilter() {
        robot::perception input;
        input.header.seq = 56;
        input.header.stamp = ros::Time(789);
        input.header.frame_id = "map";
        const float dimensions[][3] = {{0.25f, 2, 1}, {2, 0.25f, 1}, {0.5f, 1, 0.25f}, {2, 4, 0.125f}, {2, 4, 0.75f}};
        for(int i = 0; i < 5; ++i) {
            robot::object object;
            object.id = i;
            object.dx = dimensions[i][0];
            object.dy = dimensions[i][1];
            object.confidence = dimensions[i][2];
            object.heading = 90;
            object.type = 2;
            input.objs.push_back(object);
        }
        auto output = FilterPerceptionByConfidence(input, 0.25, 0.5, 1);
        Check(output.objs.size() == 2 && output.objs[0].id == 2 && output.objs[1].id == 4,
              "confidence, lateral width and longitudinal length must all meet their limits");
        Check(output.objs[0].dx == 0.5f && output.objs[0].dy == 1 && output.objs[0].confidence == 0.25f,
              "size and confidence equality boundaries are retained");
        Check(output.objs[0].heading == 90 && output.objs[0].type == 2,
              "size filtering neither rotates dimensions nor changes heading or HDMap type");
        Check(output.header.seq == 56 && output.header.stamp.toSec() == 789 && output.header.frame_id == "map",
              "size filtering preserves the original acquisition metadata");
        output = FilterPerceptionByConfidence(input, 0.25, 0.5, 0);
        Check(output.objs.size() == 3 && output.objs[0].id == 1, "width-only minimum filters dx independently of dy");
        output = FilterPerceptionByConfidence(input, 0.25, 0, 1);
        Check(output.objs.size() == 3 && output.objs[0].id == 0, "length-only minimum filters dy independently of dx");
        Check(FilterPerceptionByConfidence(input, 0.25).objs.size() == 4,
              "two-argument call keeps valid dimensions without additional size minimums");
        Check(input.objs.size() == 5 && input.objs[0].dx == 0.25f && input.objs[1].dy == 0.25f,
              "size filtering leaves its input untouched");
        output = FilterPerceptionByConfidence(input, 0.25, 10, 10);
        Check(output.objs.empty() && output.header.seq == 56, "all size-filtered result still has original header");

        input.objs.resize(1);
        input.objs[0].confidence = 1;
        input.objs[0].dx = 0.5f;
        input.objs[0].dy = 1;
        input.objs[0].dx = std::nextafter(0.5f, 0.0f);
        Check(FilterPerceptionByConfidence(input, 0.25, 0.5, 1).objs.empty(), "width just below minimum is removed");
        input.objs[0].dx = std::nextafter(0.5f, 1.0f);
        Check(FilterPerceptionByConfidence(input, 0.25, 0.5, 1).objs.size() == 1, "width just above minimum is retained");
        input.objs[0].dy = std::nextafter(1.0f, 0.0f);
        Check(FilterPerceptionByConfidence(input, 0.25, 0.5, 1).objs.empty(), "length just below minimum is removed");
        input.objs[0].dy = std::nextafter(1.0f, 2.0f);
        Check(FilterPerceptionByConfidence(input, 0.25, 0.5, 1).objs.size() == 1, "length just above minimum is retained");
        float robot::object::*fields[] = {&robot::object::dx, &robot::object::dy};
        for(auto field : fields) {
            for(float invalid : {0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN(),
                                 std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()}) {
                input.objs[0].dx = 2;
                input.objs[0].dy = 4;
                input.objs[0].*field = invalid;
                Check(FilterPerceptionByConfidence(input, 0.25).objs.empty(), "nonpositive or nonfinite dimensions are removed");
            }
        }
        input.objs.clear();
        output = FilterPerceptionByConfidence(input, 0.25, 0.5, 1);
        Check(output.objs.empty() && output.header.frame_id == "map", "empty size-filter input retains header");
        for(double invalid : {-1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity()}) {
            for(int dimension = 0; dimension < 2; ++dimension) {
                bool rejected = false;
                try {
                    FilterPerceptionByConfidence(input, 0.25, dimension == 0 ? invalid : 0, dimension == 1 ? invalid : 0);
                } catch(const std::invalid_argument&) {
                    rejected = true;
                }
                Check(rejected, "invalid size minimum is rejected even for empty input");
            }
        }
    }

    void TestSizeRangeFilter() {
        robot::perception input;
        input.header.seq = 78;
        input.header.stamp = ros::Time(234.5);
        input.header.frame_id = "map";
        const float cases[][3] = {{0.25f, 2, 0.75f}, {2.25f, 2, 0.75f}, {1, 0.75f, 0.75f}, {1, 4.25f, 0.75f}, {0.5f, 1, 0.25f}, {2, 4, 0.75f}, {1, 2, 0.125f}, {1, 2, 0.75f}};
        for(int i = 0; i < 8; ++i) {
            robot::object object;
            object.id = i;
            object.dx = cases[i][0];
            object.dy = cases[i][1];
            object.confidence = cases[i][2];
            object.type = 2;
            object.heading = 90;
            input.objs.push_back(object);
        }
        auto output = FilterPerceptionByConfidence(input, 0.25, 0.5, 2, 1, 4);
        Check(output.objs.size() == 3 && output.objs[0].id == 4 && output.objs[1].id == 5 && output.objs[2].id == 7,
              "both closed size intervals and confidence threshold must pass, preserving input order");
        Check(output.objs[0].dx == 0.5f && output.objs[0].dy == 1 && output.objs[0].confidence == 0.25f &&
                  output.objs[1].dx == 2 && output.objs[1].dy == 4,
              "all size interval endpoints and confidence equality are retained");
        Check(output.header.seq == 78 && output.header.stamp.toSec() == 234.5 && output.header.frame_id == "map" &&
                  output.objs[0].type == 2 && output.objs[0].heading == 90,
              "range filtering preserves acquisition metadata and object fields");
        Check(input.objs.size() == 8 && input.objs[0].dx == 0.25f && input.objs[3].dy == 4.25f,
              "range filtering does not modify input");
        const double max_size = std::numeric_limits<double>::infinity();
        output = FilterPerceptionByConfidence(input, 0.25, 0, 2, 0, max_size);
        Check(output.objs.size() == 6 && output.objs[2].id == 3, "width upper bound does not limit longitudinal length");
        output = FilterPerceptionByConfidence(input, 0.25, 0, max_size, 0, 4);
        Check(output.objs.size() == 6 && output.objs[1].id == 1, "length upper bound does not limit lateral width");
        output = FilterPerceptionByConfidence(input, 0.25, 10, 20, 10, 20);
        Check(output.objs.empty() && output.header.seq == 78, "fully range-filtered result keeps the original header");

        robot::object object = input.objs.back();
        input.objs.assign(1, object);
        Check(FilterPerceptionByConfidence(input, 0.25, 1, 1, 2, 2).objs.size() == 1,
              "equal lower and upper limits accept the exact positive dimensions");
        Check(FilterPerceptionByConfidence(input, 0.25, 0, 0, 0, 0).objs.empty(), "zero-only interval excludes positive dimensions");
        input.objs[0].dx = std::nextafter(2.0f, 1.0f);
        Check(FilterPerceptionByConfidence(input, 0.25, 0.5, 2, 1, 4).objs.size() == 1, "width just below upper limit is retained");
        input.objs[0].dx = std::nextafter(2.0f, 3.0f);
        Check(FilterPerceptionByConfidence(input, 0.25, 0.5, 2, 1, 4).objs.empty(), "width just above upper limit is removed");
        input.objs[0].dx = 1;
        input.objs[0].dy = std::nextafter(4.0f, 3.0f);
        Check(FilterPerceptionByConfidence(input, 0.25, 0.5, 2, 1, 4).objs.size() == 1, "length just below upper limit is retained");
        input.objs[0].dy = std::nextafter(4.0f, 5.0f);
        Check(FilterPerceptionByConfidence(input, 0.25, 0.5, 2, 1, 4).objs.empty(), "length just above upper limit is removed");
        input.objs[0].dx = std::numeric_limits<float>::max();
        input.objs[0].dy = std::numeric_limits<float>::max();
        Check(FilterPerceptionByConfidence(input, 0.25, 0, max_size, 0, max_size).objs.size() == 1,
              "positive infinite upper bounds accept every finite positive dimension");
        Check(FilterPerceptionByConfidence(input, 0.25, 0.5, 1).objs.size() == 1,
              "legacy four-argument call keeps unbounded upper limits");

        input.objs.clear();
        output = FilterPerceptionByConfidence(input, 0.25, 0.5, 2, 1, 4);
        Check(output.objs.empty() && output.header.frame_id == "map", "empty range input retains acquisition metadata");
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double invalid_ranges[][4] = {{2, 1, 0, 10}, {0, 10, 2, 1}, {0, -1, 0, 10}, {0, 10, 0, -1}, {0, nan, 0, 10}, {0, 10, 0, nan}, {0, -max_size, 0, 10}, {0, 10, 0, -max_size}, {nan, 10, 0, 10}, {0, 10, nan, 10}, {max_size, max_size, 0, 10}, {0, 10, max_size, max_size}};
        for(const auto& interval : invalid_ranges) {
            bool rejected = false;
            try {
                FilterPerceptionByConfidence(input, 0.25, interval[0], interval[1], interval[2], interval[3]);
            } catch(const std::invalid_argument&) {
                rejected = true;
            }
            Check(rejected, "reversed, negative or nonfinite interval configuration is rejected before processing empty input");
        }
    }

    void TestPublishConfidenceThreshold() {
        mPerceptionHistory.clear();
        SetPose(10, 0, 90);
        Expect(1000, {Marker(10, 0)}, {0}, {1});
        // 当前发布门槛为 0.15；两帧权重比为 3:17，验证等号边界。
        const double boundary_time = 1000 + std::log(17.0 / 3.0);
        Expect(boundary_time, {}, {0}, {0.15});
        Check(publications.back().objs[0].confidence == 0.15f, "publish retains confidence exactly equal to 0.15");
        Expect(boundary_time + 0.0001, {}, {}, {});
        Check(mPerceptionHistory.front().first.objs.size() == 1, "below-threshold historical target remains cached until expiration");

        mPerceptionHistory.clear();
        for(int i = 0; i < 10; ++i) {
            Expect(1100 + i * 0.1, {}, {}, {});
        }
        // 十个空观测后首次检出，置信度约 0.143；非空原始帧也须正常发布筛选后的空消息。
        Expect(1101, {Marker(10, 4)}, {}, {});
        Check(mPerceptionHistory.size() == 11 && mPerceptionHistory.back().first.objs.size() == 1,
              "nonempty low-confidence input remains raw history even when publication is empty");
        Check(mPerceptionHistory.back().first.objs[0].type == 1 && mPerceptionHistory.back().first.objs[0].confidence == 0,
              "raw history keeps HDMap classification without feeding back filtered scores");
        double total_weight = 0;
        for(int i = 0; i < 12; ++i) {
            total_weight += std::exp(-i * 0.1);
        }
        Expect(1101.1, {Marker(10.2, 4)}, {1}, {(std::exp(-0.1) + 1) / total_weight});
        Near(publications.back().objs[0].x, 20.2, "target recovers using earlier unpublished evidence and latest geometry");
    }

    void TestCurrentLaneOverlapThreshold() {
        SetPose(10, 0, 90);
        // 车道为 y=[-2,2]，框长为 5；y=-3.5 时交面积/框面积恰好 1/5。
        const double centers[] = {-4.51, -4.5, -4.499, -4.0, -3.55,
                                  std::nextafter(-3.5f, -4.0f), -3.5,
                                  std::nextafter(-3.5f, -3.0f), -3.0, 0};
        const int types[] = {4, 4, 4, 4, 4, 4, 0, 0, 0, 0};
        for(std::size_t i = 0; i < sizeof(centers) / sizeof(centers[0]); ++i) {
            mPerceptionHistory.clear();
            Expect(1200 + i, {Marker(10, centers[i], 2, 5)}, {types[i]}, {1});
            const auto& object = publications.back().objs[0];
            Near(object.x, 20, "overlap threshold keeps original world position x");
            Near(object.y, centers[i], "overlap threshold keeps original world position y");
            Near(object.dx, 2, "overlap threshold keeps original dx");
            Near(object.dy, 5, "overlap threshold keeps original dy");
            Near(object.heading, 90, "new target fallback heading uses north-zero clockwise convention");
            Check(object.id > 0 && object.vx == 0 && object.vy == 0 && object.height == 0,
                  "new target has an ID and unknown velocity is initialized to zero");
            Check(mPerceptionHistory.back().first.objs[0].type == types[i],
                  "raw current-lane classification uses the same twenty-percent gate");
        }

        // 保持 HDMap 的既有车道宽度，不能按 monitor 显示边界重新定义本车道。
        mPerceptionHistory.clear();
        Expect(1220, {Marker(10, -1.5, 0.4, 0.4)}, {0}, {1});
        // 左一/左二不足 20% 时仍采用 SDK 结果，本次门槛只作用于本车道。
        mPerceptionHistory.clear();
        Expect(1221, {Marker(-10.9, 4), Marker(10, 10.9)}, {1, 2}, {1, 1});

        SetPose(10, 8, 90);
        mPerceptionHistory.clear();
        Expect(1222, {Marker(10, 3.55, 2, 5)}, {3}, {1});
        mPerceptionHistory.clear();
        Expect(1223, {Marker(10, 3.5, 2, 5)}, {0}, {1});
        SetPose(10, 0, 90);
        // 旋转后的真实矩形面积判定：框局部 x 长 2.5，旋转 90 度后在 y 方向伸展。
        auto rotated = Marker(10, -2.75, 2.5, 1);
        rotated.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI / 2);
        mPerceptionHistory.clear();
        Expect(1224, {rotated}, {0}, {1});
        rotated.pose.position.y = std::nextafter(-2.75f, -3.0f);
        mPerceptionHistory.clear();
        Expect(1225, {rotated}, {4}, {1});
        rotated = Marker(10, -2.8);
        rotated.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI / 4);
        mPerceptionHistory.clear();
        Expect(1226, {rotated}, {4}, {1});
        rotated.pose.position.y = -2;
        mPerceptionHistory.clear();
        Expect(1227, {rotated}, {0}, {1});
    }

    void TestCurrentPoseReclassification() {
        mPerceptionHistory.clear();
        SetPose(10, 0, 90);
        Expect(1300, {Marker(20, 0)}, {0}, {1});
        const auto original = publications.back().objs[0];
        const double decay = std::exp(-0.1);
        SetPose(10, 4, 90);
        Expect(1300.1, {}, {4}, {decay / (decay + 1)});
        const auto& retained = publications.back().objs[0];
        Check(retained.x == original.x && retained.y == original.y && retained.dx == original.dx &&
                  retained.dy == original.dy && retained.heading == original.heading,
              "ego lane change only updates retained target lane type and confidence");
        Check(mPerceptionHistory.front().first.objs[0].type == 0 && mPerceptionHistory.back().first.objs.empty(),
              "reclassification never writes retained targets or current types back into raw history");
        Expect(1300.2, {Marker(20, -4)}, {4}, {(decay * decay + 1) / (decay * decay + decay + 1)});

        // 框世界朝向固定为北向；自车转向和运动 heading 都不能改变车道匹配使用的几何。
        mPerceptionHistory.clear();
        SetPose(10, 0, 90);
        auto rotated = Marker(10, -2.5, 4, 1);
        rotated.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI / 2);
        Expect(1310, {rotated}, {0}, {1});
        Near(publications.back().objs[0].heading, 0, "north-facing new target has navigation-compatible heading");
        SetPose(10, 0, 135);
        Expect(1310.1, {}, {0}, {decay / (decay + 1)});
        Near(publications.back().objs[0].heading, 0, "ego rotation does not rewrite historical heading");
        Near(publications.back().objs[0].y, -2.5, "ego rotation does not move historical geometry");
        // 新观测维持相同世界矩形，但车体局部角已改变；静止目标仍保持北向。
        rotated.pose.position.x = 12.5 / std::sqrt(2.0);
        rotated.pose.position.y = 7.5 / std::sqrt(2.0);
        rotated.pose.orientation = tf::createQuaternionMsgFromYaw(3 * M_PI / 4);
        Expect(1310.2, {rotated}, {0}, {(decay * decay + 1) / (decay * decay + decay + 1)});
        Near(publications.back().objs[0].heading, 0, "fresh matched observation supplies the latest heading");
        SetPose(10, 0, 90);
        Expect(1310.3, {}, {0}, {(decay * decay * decay + decay) / (decay * decay * decay + decay * decay + decay + 1)});

        mPerceptionHistory.clear();
        SetPose(10, 0, 90);
        Expect(1320, {Marker(10, -3.55, 2, 5)}, {4}, {1});
        SetPose(10, 0, 270);
        Expect(1320.1, {}, {3}, {decay / (decay + 1)});
        Near(publications.back().objs[0].y, -3.55, "outside side uses current heading without rotating the old box");

        mPerceptionHistory.clear();
        SetPose(10, 0, 90);
        Expect(1330, {Marker(10, 0)}, {0}, {1});
        const auto corners = mPerceptionHistory.front().first.objs[0].polygons;
        Check(corners.size() == 4, "raw observations retain world geometry internally");
        const auto previous = publications.size();
        mPerceptionHistory.front().first.objs[0].polygons.clear();
        Send(1330.1, {});
        Check(publications.size() == previous && mPerceptionHistory.size() == 1,
              "missing historical geometry rolls back the new frame without publishing stale types");
        Near(mPerception.header.stamp.toSec(), 1330, "reclassification failure preserves last valid snapshot");
        mPerceptionHistory.front().first.objs[0].polygons = corners;
        mPerceptionHistory.front().first.objs[0].polygons[0].x = std::numeric_limits<double>::quiet_NaN();
        Send(1330.1, {});
        Check(publications.size() == previous && mPerceptionHistory.size() == 1,
              "invalid historical geometry also rolls back without a synthetic empty publication");
        mPerceptionHistory.front().first.objs[0].polygons = corners;
        Expect(1330.1, {}, {0}, {decay / (decay + 1)});
    }

    void TestMainLoopWithoutCallbacks() {
        mPerceptionHistory.clear();
        Expect(900, {Marker(10, 0)}, {0}, {1});
        const auto publish_count = std::count(ros::param::events().begin(), ros::param::events().end(), "publish:/perception");
        int iterations = 0;
        ros::testLoopCount() = 2;
        ros::testSpinOnce() = [&iterations]() {
            // 不触发 GNSS 或感知回调，只向前推进 ROS 时间，检查真正 main 中的维护入口。
            Check(mPerceptionHistory.size() == 1 && mPerception.objs.size() == 1,
                  "main loop preserves history strictly younger than two seconds");
            ros::testTime() += 1;
            ++iterations;
        };
        char name[] = "perception_msg_convert";
        char* arguments[] = {name, nullptr};
        Check(PerceptionConvertMain(1, arguments) == 0, "node main exits normally after controlled loop");
        ros::testSpinOnce() = std::function<void()>();
        Check(iterations == 2 && mPerceptionHistory.empty() && mPerception.objs.empty(),
              "actual main loop expires history without any callbacks");
        Check(std::count(ros::param::events().begin(), ros::param::events().end(), "publish:/perception") == publish_count,
              "main cleanup never publishes a synthetic empty heartbeat");
    }

    visualization_msgs::Marker WorldMarker(double tX, double tY, double tDx = 0.4, double tDy = 0.4,
                                           double tYaw = 0) {
        const double yaw = (90 - mGPS.heading) * M_PI / 180.0;
        const double x = tX - mGPS.xAxis, y = tY - mGPS.yAxis;
        auto marker = Marker(x * std::cos(yaw) + y * std::sin(yaw), -x * std::sin(yaw) + y * std::cos(yaw), tDx, tDy);
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(tYaw - yaw);
        return marker;
    }

    robot::object PublishedObject(int tId) {
        for(const auto& object : publications.back().objs) {
            if(object.id == tId) {
                return object;
            }
        }
        throw std::runtime_error("Expected track ID missing from publication");
    }

    void TestMotionPublication() {
        mPerceptionHistory.clear();
        SetPose(10, 0, 90);
        Expect(1400, {WorldMarker(20, 0), WorldMarker(40, 4)}, {0, 1}, {1, 1});
        const int moving_id = publications.back().objs[0].id;
        const int stationary_id = publications.back().objs[1].id;
        Check(moving_id != stationary_id, "simultaneous objects get distinct tracking IDs");
        for(int i = 1; i <= 40; ++i) {
            // 自车同时平移和转向，地图内静止目标仍应为零速。
            SetPose(10 + 0.3 * i, 0, 45 + i % 3 * 45);
            const auto moving = WorldMarker(20 + 0.2 * i, 0);
            const auto stationary = WorldMarker(40, 4);
            const auto previous = publications.size();
            Send(1400 + 0.1 * i, i % 2 ? std::vector<visualization_msgs::Marker>{stationary, moving} : std::vector<visualization_msgs::Marker>{moving, stationary});
            Check(publications.size() == previous + 1 && publications.back().objs.size() == 2,
                  "continuous moving targets publish once without geometry-based ghost duplicates");
            const auto target = PublishedObject(moving_id);
            const auto fixed = PublishedObject(stationary_id);
            Check(target.type == 0 && fixed.type == 1 && target.confidence == 1 && fixed.confidence == 1,
                  "tracking identity remains stable as the confidence window and object order change");
            Near(target.x, 20 + 0.2 * i, "published position remains latest measured position");
            Check(fixed.vx == 0 && fixed.vy == 0, "ego motion is removed before estimating object velocity");
            if(i >= 10) {
                Check(std::abs(target.vx - 2) < 0.03 && std::abs(target.vy) < 0.01,
                      "node publishes absolute map velocity in metres per second");
                Near(target.heading, 90, "eastbound velocity generates navigation heading 90");
            }
        }
        const auto last_seen = PublishedObject(moving_id);
        for(int i = 1; i <= 5; ++i) {
            Send(1404 + i * 0.1, {WorldMarker(40, 4)});
            const auto missed = PublishedObject(moving_id);
            Check(missed.x == last_seen.x && missed.y == last_seen.y && missed.vx == last_seen.vx &&
                      missed.vy == last_seen.vy && missed.heading == last_seen.heading,
                  "missed target keeps measured geometry and predicted constant velocity without fake observations");
            Check(mPerceptionHistory.back().first.objs.size() == 1 &&
                      mPerceptionHistory.back().first.objs[0].id == stationary_id,
                  "missed target is never added back to raw evidence");
        }
        Send(1404.6, {WorldMarker(29.2, 0), WorldMarker(40, 4)});
        Check(publications.back().objs.size() == 2, "reacquisition avoids a second ghost object at the old position");
        Check(std::abs(PublishedObject(moving_id).vx - 2) < 0.03, "reacquired target keeps ID and velocity");

        // 物理长框竖直、运动向东；运动航向不能把 20% 的几何重叠错误变成零重叠。
        mPerceptionHistory.clear();
        SetPose(10, 0, 90);
        int id = 0;
        for(int i = 0; i <= 10; ++i) {
            Expect(1430 + i * 0.1, {WorldMarker(20 + i * 0.2, -2.75, 2.5, 1, M_PI / 2)}, {0}, {1});
            if(i == 0) {
                id = publications.back().objs[0].id;
                Near(publications.back().objs[0].heading, 0, "new vertical box fallback points north");
            }
            Check(publications.back().objs[0].id == id, "box and motion heading differences do not break tracking");
        }
        Near(publications.back().objs[0].heading, 90, "motion heading replaces fallback while lane geometry stays vertical");
    }

    void TestTrackingTransactions() {
        mPerceptionHistory.clear();
        SetPose(10, 0, 90);
        Expect(1450, {Marker(10, 0), Marker(10, 4)}, {0, 1}, {1, 1});
        const int first_id = publications.back().objs[0].id, second_id = publications.back().objs[1].id;
        Expect(1450, {Marker(10, 4), Marker(10, 0)}, {1, 0}, {1, 1});
        Check(publications.back().objs[0].id == second_id && publications.back().objs[1].id == first_id,
              "same-time replacement preserves newborn IDs despite reordered observations");
        Expect(1450, {Marker(10, 4)}, {1}, {1});
        Check(publications.back().objs[0].id == second_id, "same-time replacement preserves ID after another newborn is removed");
        Expect(1450, {}, {}, {});
        Expect(1450, {Marker(20, 0)}, {0}, {1});
        Check(publications.back().objs[0].id != first_id && publications.back().objs[0].id != second_id,
              "replacing a birth with empty input cannot recycle its published ID immediately");

        mPerceptionHistory.clear();
        Expect(1460, {Marker(10, 0)}, {0}, {1});
        const int id = publications.back().objs[0].id;
        Expect(1460.1, {Marker(10.2, 0)}, {0}, {1});
        Expect(1460.2, {Marker(10.4, 0)}, {0}, {1});
        const auto original = publications.back().objs[0];
        for(int i = 0; i < 20; ++i) {
            Expect(1460.2, {Marker(10.4, 0)}, {0}, {1});
            const auto repeated = publications.back().objs[0];
            Check(repeated.id == id && repeated.vx == original.vx && repeated.vy == original.vy &&
                      repeated.heading == original.heading && mPerceptionHistory.size() == 3,
                  "same timestamp does not repeatedly update Kalman state or confidence");
        }
        PerceptionObjectTracker control = mPerceptionTracker;
        robot::perception expected;
        expected.objs = {original};
        expected.objs[0].x = 20.6;
        control.Update(expected, 1460.3);
        const auto previous = publications.size();
        ros::Time::testFailAfter() = 1;
        Send(1460.3, {Marker(10.9, 0)});
        Check(publications.size() == previous && mPerceptionHistory.size() == 3,
              "filter failure does not commit the candidate tracking update");
        Expect(1460.3, {Marker(10.6, 0)}, {0}, {1});
        Check(publications.back().objs[0].id == id && publications.back().objs[0].vx == expected.objs[0].vx,
              "motion state after rollback agrees with processing only the successful observations");

        Expect(1459, {Marker(10, 0)}, {0}, {1});
        Check(publications.back().objs[0].id != id && publications.back().objs[0].vx == 0,
              "clock rollback clears motion history without immediately reusing an ID");
        const int reset_id = publications.back().objs[0].id;
        const auto count = publications.size();
        MaintainPerceptionHistory(1461);
        Check(publications.size() == count && mPerceptionHistory.empty(), "silent expiry clears tracks without publishing heartbeat");
        Expect(1461.1, {Marker(10, 0)}, {0}, {1});
        Check(publications.back().objs[0].id != reset_id && publications.back().objs[0].vx == 0,
              "first detection after silent expiry starts a fresh velocity estimate");
    }

    void TestClassificationAndExclusions() {
        mPerceptionHistory.clear();
        Expect(500, {Marker(10, 0), Marker(10, 4), Marker(10, 8), Marker(10, 14), Marker(10, -6)},
               {0, 1, 2, 3, 4}, {1, 1, 1, 1, 1});
        mPerceptionHistory.clear();
        Expect(501, {Marker(10, 2.8, 2, 4)}, {1}, {1});
        mPerceptionHistory.clear();
        SetPose(80, 8, 270);
        Expect(502, {Marker(10, 0), Marker(10, 4), Marker(10, 8)}, {0, 1, 2}, {1, 1, 1});
        mPerceptionHistory.clear();
        SetPose(10, 20, 90);
        Expect(503, {Marker(10, -12), Marker(10, 5)}, {4, 3}, {1, 1});
        mPerceptionHistory.clear();
        SetPose(10, 0, 90);
        auto text = Marker(10, 8);
        text.type = 9;  // TEXT_VIEW_FACING；现有轻量 ROS 桩未定义该常量。
        g_perception_boundary.polygons_ = {{Vec2d(19, -1), Vec2d(21, -1), Vec2d(21, 1), Vec2d(19, 1)}};
        Expect(504, {Marker(10, 0), Marker(10, 4), text}, {1}, {1});
        Expect(504.5, {Marker(10, 0), text}, {1}, {std::exp(-0.5) / (std::exp(-0.5) + 1)});
        Check(mPerceptionHistory.back().first.objs.empty(), "excluded and non-CUBE markers never enter raw history");
        Expect(506, {}, {}, {});
        g_perception_boundary.polygons_.clear();
    }

    void TestPlanningObservationPublication() {
        std::vector<robot::perception> planning;
        planning_perception_pub.capture = [&planning](const std::type_info& type, const void* value) {
            Check(type == typeid(robot::perception), "planning observation keeps existing ROS message definition");
            planning.push_back(*static_cast<const robot::perception*>(value));
        };
        mPerceptionHistory.clear();
        SetPose(10, 0, 90);
        auto marker = Marker(10, -2.5, 1, 4);
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI / 2);
        marker.color.a = 0.72;
        Send(5000, {marker});
        Check(planning.size() == 1 && planning.back().objs.size() == 1, "one real observation published");
        const auto object = planning.back().objs.front();
        Check(object.id > 0 && object.polygons.size() == 4, "stable ID and true world corners supplied together");
        Near(object.confidence, 0.72, "planning confidence is original detection score");
        Near(object.polygons[0].x, 22, "physical long side follows marker geometry");
        Near(object.polygons[0].y, -3, "physical short side follows marker geometry");
        planning_perception::PerceptionSafety safety;
        Check(safety.Observe(planning.back(), 5000), "real converter output is accepted by real planning algorithm");
        planning_perception::EGO_S ego;
        ego.x = 10;
        ego.heading = 90;
        ego.speed = 0.8;
        std::vector<planning_perception::PATH_POINT_S> path;
        for(int i = 0; i < 25; ++i) path.emplace_back(10 + i, 0);
        const auto risk = safety.Evaluate(path, ego, 3, 5000);
        Check(risk.reason == planning_perception::CLEAR && !risk.emergency,
              "converter physical side box stays clear through actual planning despite heading mismatch");
        Check(publications.back().objs.front().polygons.empty(), "legacy polygons unchanged");
        Near(publications.back().objs.front().confidence, 1, "legacy temporal confidence unchanged");
        Send(5000.1, {});
        Check(planning.size() == 2 && planning.back().objs.empty(), "missing object not re-published as observation");
        Check(publications.back().objs.size() == 1, "legacy history retention remains available to existing consumers");
        const auto count = planning.size();
        MaintainPerceptionHistory(5003);
        Check(planning.size() == count, "expiry cannot generate a false fresh planning frame");
        mNavigationReceived = false;
        Send(5004, {marker}, false);
        Check(planning.size() == count, "invalid localization blocks both outputs");
        SetPose(10, 0, 90);
        const auto before_burst = planning.size();
        for(int i = 0; i <= 20; ++i) Send(5100 + i * 0.01, {marker});
        Check(planning.size() == before_burst + 3, "100 Hz observations are bounded to 12.5 Hz planning publications");
        Send(5100.21, {});
        Check(planning.size() == before_burst + 4 && planning.back().objs.empty(), "empty transition bypasses rate gate");
        Send(5100.22, {marker});
        Check(planning.size() == before_burst + 5 && planning.back().objs.size() == 1,
              "new nonempty transition bypasses rate gate");
        planning_perception_pub.capture = {};
    }
}

int main() {
    try {
        Fixture fixture;
        ros::package::paths()["hdmap"] = fixture.root;
        ros::param::values().erase("/hdmap/map_processed_dir");
        char name[] = "perception_msg_convert";
        char* arguments[] = {name, nullptr};
        Check(PerceptionConvertMain(1, arguments) == 0 && mLaneMap.GetMapInfo().lane_count == 3,
              "node startup loads HDMap");
        g_perception_boundary.polygons_.clear();
        perception_pub.capture = [](const std::type_info& type, const void* value) {
            Check(type == typeid(robot::perception), "published message type");
            publications.push_back(*static_cast<const robot::perception*>(value));
        };
        TestRawHistoryAndMisses();
        TestTimeAndBoundedCache();
        TestFailuresAndRecovery();
        TestClassificationAndExclusions();
        TestLocalizationValidity();
        TestSilentExpiryAndRecovery();
        TestClockChangesDuringProcessing();
        TestConfidenceFilter();
        TestSizeFilter();
        TestSizeRangeFilter();
        TestPublishConfidenceThreshold();
        TestCurrentLaneOverlapThreshold();
        TestCurrentPoseReclassification();
        TestMotionPublication();
        TestTrackingTransactions();
        TestPlanningObservationPublication();
        SetPose(10, 0, 90);
        TestMainLoopWithoutCallbacks();
        std::printf("PASS: %d filtered perception publish checks\n", checks);
    } catch(const std::exception& error) {
        std::fprintf(stderr, "FAIL after %d checks: %s\n", checks, error.what());
        return 1;
    }
    return 0;
}
