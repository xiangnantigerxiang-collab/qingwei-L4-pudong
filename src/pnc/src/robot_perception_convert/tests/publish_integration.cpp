#define main PerceptionConvertMain
#include "../perception_msg_convert.cpp"
#undef main

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
        for(std::size_t i = 0; i < message.objs.size(); ++i) {
            Check(message.objs[i].type == tTypes[i], "HDMap classification must precede filter and publish");
            Near(message.objs[i].confidence, tConfidence[i], "confidence must be calculated before publish");
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
        Expect(101, {}, {0, 2}, {(std::exp(-1) + decay) / total, decay / total});
        Check(mPerceptionHistory.back().first.objs.empty(), "empty raw frame retained in denominator");
        const double remaining = std::exp(-1.5) / (std::exp(-1.5) + std::exp(-1) + 1);
        Expect(102, {}, {}, {});
        const auto unthresholded = FilterPerceptionHistory(mPerceptionHistory);
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

        float robot::navigation_msg::* fields[] = {&robot::navigation_msg::xAxis, &robot::navigation_msg::yAxis};
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
        const float dimensions[][3] = {{0.25f, 2, 1}, {2, 0.25f, 1}, {0.5f, 1, 0.25f},
                                      {2, 4, 0.125f}, {2, 4, 0.75f}};
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
        float robot::object::* fields[] = {&robot::object::dx, &robot::object::dy};
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
        const float cases[][3] = {{0.25f, 2, 0.75f}, {2.25f, 2, 0.75f}, {1, 0.75f, 0.75f}, {1, 4.25f, 0.75f},
                                 {0.5f, 1, 0.25f}, {2, 4, 0.75f}, {1, 2, 0.125f}, {1, 2, 0.75f}};
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
        const double invalid_ranges[][4] = {{2, 1, 0, 10}, {0, 10, 2, 1}, {0, -1, 0, 10}, {0, 10, 0, -1},
                                           {0, nan, 0, 10}, {0, 10, 0, nan}, {0, -max_size, 0, 10}, {0, 10, 0, -max_size},
                                           {nan, 10, 0, 10}, {0, 10, nan, 10}, {max_size, max_size, 0, 10}, {0, 10, max_size, max_size}};
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
        // 两帧权重比为 1:3，历史对象置信度恰为 0.25，验证发布时保留等号边界。
        const double boundary_time = 1000 + std::log(3.0);
        Expect(boundary_time, {}, {0}, {0.25});
        Check(publications.back().objs[0].confidence == 0.25f, "publish retains confidence exactly equal to 0.25");
        Expect(boundary_time + 0.0001, {}, {}, {});
        Check(mPerceptionHistory.front().first.objs.size() == 1, "below-threshold historical target remains cached until expiration");

        mPerceptionHistory.clear();
        for(int i = 0; i < 4; ++i) {
            Expect(1100 + i * 0.1, {}, {}, {});
        }
        // 四个空观测后首次检出，置信度约 0.242；原始帧非空也必须正常发布筛选后的空消息。
        Expect(1100.4, {Marker(10, 4)}, {}, {});
        Check(mPerceptionHistory.size() == 5 && mPerceptionHistory.back().first.objs.size() == 1,
              "nonempty low-confidence input remains raw history even when publication is empty");
        Check(mPerceptionHistory.back().first.objs[0].type == 1 && mPerceptionHistory.back().first.objs[0].confidence == 0,
              "raw history keeps HDMap classification without feeding back filtered scores");
        double total_weight = 0;
        for(int i = 0; i < 6; ++i) {
            total_weight += std::exp(-i * 0.1);
        }
        Expect(1100.5, {Marker(10.2, 4)}, {1}, {(std::exp(-0.1) + 1) / total_weight});
        Near(publications.back().objs[0].x, 20.2, "target recovers using earlier unpublished evidence and latest geometry");
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
        TestMainLoopWithoutCallbacks();
        std::printf("PASS: %d filtered perception publish checks\n", checks);
    } catch(const std::exception& error) {
        std::fprintf(stderr, "FAIL after %d checks: %s\n", checks, error.what());
        return 1;
    }
    return 0;
}
