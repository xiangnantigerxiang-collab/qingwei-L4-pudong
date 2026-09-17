#define main PerceptionConvertMain
#include "../../src/robot_perception_convert/perception_msg_convert.cpp"
#undef main

#include "hdmap/hdmap_server.h"
#include <cstdio>
#include <limits>
#include <sys/stat.h>
#include <unistd.h>

namespace {
    int checks = 0;
    std::vector<robot::perception> publications;

    void Check(bool condition, const char* message) {
        ++checks;
        if(!condition) {
            throw std::runtime_error(message);
        }
    }
    void Good(const hdmap::STATUS_S& status) {
        ++checks;
        if(!status.IsOk()) {
            throw std::runtime_error(status.message);
        }
    }

    struct Fixture {
        Fixture() {
            char path[] = "/tmp/perception_hdmap_XXXXXX";
            char* created = mkdtemp(path);
            if(!created) {
                throw std::runtime_error("mkdtemp failed");
            }
            root = created;
            directory = root + "/map_processed";
            if(mkdir(directory.c_str(), 0700) != 0) {
                throw std::runtime_error("mkdir failed");
            }
            for(int lane = 0; lane < 3; ++lane) {
                hdmap::MapPointList points;
                for(int i = 0; i <= 200; ++i) {
                    points.push_back(hdmap::MAP_POINT_S(i * 0.5, lane * 4, 90));
                    points.back().dist_origin = i * 0.5;
                    points.back().p2pDistance = i == 0 ? 0 : 0.5;
                }
                files.push_back(directory + "/lane" + std::to_string(lane) + ".csv");
                Good(hdmap::HdMapServer().SaveTrajectory(files.back(), points, hdmap::DIRECTION_FORWARD));
            }
        }
        ~Fixture() {
            for(std::size_t i = 0; i < files.size(); ++i) {
                std::remove(files[i].c_str());
                std::remove((files[i] + ".hidden").c_str());
            }
            rmdir(directory.c_str());
            rmdir(root.c_str());
        }
        std::string root, directory;
        std::vector<std::string> files;
    };

    int StartNode() {
        char name[] = "perception_msg_convert";
        char* arguments[] = {name, nullptr};
        return PerceptionConvertMain(1, arguments);
    }
    void SetPose(double x, double y, double heading) {
        ivlocmsg::ivmsglocpos navigation;
        navigation.xg = x;
        navigation.yg = y;
        navigation.zg = 3;
        navigation.heading = heading;
        GNSSCallback(navigation);
    }
    visualization_msgs::Marker Marker(double x, double y, double dx = 2, double dy = 2, double yaw = 0) {
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        marker.scale.x = dx;
        marker.scale.y = dy;
        return marker;
    }
    void Expect(const visualization_msgs::MarkerArray& input, const std::vector<int>& types) {
        std::size_t previous = publications.size();
        BoxMsgCallBack(input);
        Check(publications.size() == previous + 1, "expected exactly one publish");
        const auto& message = publications.back();
        Check(message.objs.size() == types.size(), "wrong published object count");
        for(std::size_t i = 0; i < types.size(); ++i) {
            Check(message.objs[i].type == types[i], "publish must contain classified type");
        }
    }

    void TestPublishIntegration() {
        Fixture fixture;
        ros::package::paths()["hdmap"] = fixture.root;
        ros::param::values().erase("/hdmap/map_processed_dir");
        Check(StartNode() == 0 && mLaneMap.GetMapInfo().lane_count == 3, "default package map path");
        // 直接捕获真实回调传给 Publisher 的值；若分类被放在 publish 之后，此处断言会失败。
        perception_pub.capture = [](const std::type_info& type, const void* value) {
            Check(type == typeid(robot::perception), "wrong published message type");
            publications.push_back(*static_cast<const robot::perception*>(value));
        };
        g_perception_boundary.polygons_.clear();
        SetPose(10, 0, 90);
        visualization_msgs::MarkerArray input;
        input.markers = {Marker(10, 0), Marker(10, 4), Marker(10, 8), Marker(10, 14), Marker(10, -6), Marker(10, 2.8, 2, 4)};
        Expect(input, {0, 1, 2, 3, 4, 1});
        for(std::size_t i = 0; i < input.markers.size(); ++i) {
            const auto& obj = publications.back().objs[i];
            Check(std::abs(obj.x - 20) < 1e-6 && std::abs(obj.y - input.markers[i].pose.position.y) < 1e-6, "map position preserved");
            Check(obj.dx == input.markers[i].scale.x && obj.dy == input.markers[i].scale.y && obj.heading == 180, "box fields preserved");
        }

        SetPose(80, 8, 270);
        input.markers = {Marker(10, 0), Marker(10, 4), Marker(10, 8)};
        Expect(input, {0, 1, 2});
        SetPose(10, 20, 90);
        input.markers = {Marker(10, -12), Marker(10, 5)};
        Expect(input, {4, 3});  // 障碍物落在实际车道上，但自车离道时仍只返回左右。

        // 自车与框朝向不同，框中心在中心线终点外、长边仍覆盖本车道，检查真实 heading 链路。
        SetPose(10, 0, 45);
        double theta = M_PI / 4;
        input.markers = {Marker(90.8 * std::cos(theta), -90.8 * std::sin(theta), 4, 0.5, -theta)};
        Expect(input, {0});
        Check(std::abs(publications.back().objs[0].x - 100.8) < 1e-4, "rotated global coordinate");

        SetPose(10, 0, 90);
        g_perception_boundary.polygons_ = {{Vec2d(19, -1), Vec2d(21, -1), Vec2d(21, 1), Vec2d(19, 1)}};
        auto text = Marker(10, 8);
        text.type = 9;
        input.markers = {Marker(10, 0), Marker(10, 4), text};
        Expect(input, {1});
        input.markers = {Marker(10, 0), text};
        Expect(input, {});
        g_perception_boundary.polygons_.clear();

        std::size_t previous = publications.size();
        input.markers.clear();
        BoxMsgCallBack(input);
        Check(publications.size() == previous, "existing empty-marker behavior changed");
        input.markers = {Marker(10, 4, 0, 2)};
        BoxMsgCallBack(input);
        Check(publications.size() == previous, "classification failure must not publish stale type");
        input.markers = {Marker(10, 4)};
        Expect(input, {1});
        previous = publications.size();
        mGPS.zAxis = std::numeric_limits<float>::quiet_NaN();
        BoxMsgCallBack(input);
        Check(publications.size() == previous, "invalid pose must not publish");
        SetPose(10, 0, 90);
        Expect(input, {1});

        // 加载后移开 CSV，分类仍能正常发布，证明热路径没有再次读取地图。
        for(std::size_t i = 0; i < fixture.files.size(); ++i) {
            Check(std::rename(fixture.files[i].c_str(), (fixture.files[i] + ".hidden").c_str()) == 0, "hide cached map");
        }
        Expect(input, {1});
        for(std::size_t i = 0; i < fixture.files.size(); ++i) {
            Check(std::rename((fixture.files[i] + ".hidden").c_str(), fixture.files[i].c_str()) == 0, "restore cached map");
        }

        ros::param::set("/hdmap/map_processed_dir", fixture.directory + "/missing");
        Check(StartNode() == 1, "invalid configured map path must fail startup");
        ros::package::paths()["hdmap"] = fixture.root + "/missing_package";
        ros::param::set("/hdmap/map_processed_dir", fixture.directory);
        Check(StartNode() == 0 && mLaneMap.IsLoaded(), "configured map path overrides package default");
    }
}

int main() {
    try {
        TestPublishIntegration();
        std::printf("PASS: %d perception publish integration checks\n", checks);
    } catch(const std::exception& error) {
        std::fprintf(stderr, "FAIL after %d checks: %s\n", checks, error.what());
        return 1;
    }
    return 0;
}
