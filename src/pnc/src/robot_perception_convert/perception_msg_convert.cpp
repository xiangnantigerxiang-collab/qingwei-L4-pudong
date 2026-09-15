#include "perception_msg_convert.h"
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <dirent.h>
#include <ros/package.h>
#include "hdmap/lane_map_server.h"
#include "perception_temporal_filter.inc"
#include "perception_confidence_filter.inc"
#include <limits>

robot::perception mPerception;
robot::navigation_msg mGPS;
hdmap::LaneMapServer mLaneMap;
std::vector<std::pair<robot::perception, double>> mPerceptionHistory;
bool mNavigationReceived = false;
double mNavigationTimestamp = 0;

ros::Subscriber gnss_sub;
ros::Subscriber box_sub;
ros::Publisher perception_pub;
ros::Publisher udp_pub;
ros::Publisher string_pub;

PerceptionBoundary g_perception_boundary;

// 感知排除区域实现：加载配置目录下所有 .csv 文件，每个文件作为一个排除多边形
bool PerceptionBoundary::LoadBoundary(const std::string& dir_path) {
    polygons_.clear();
    DIR* dir = opendir(dir_path.c_str());
    if(!dir) {
        ROS_ERROR("Failed to open perception boundary dir: %s", dir_path.c_str());
        return false;
    }

    std::vector<std::string> files;
    struct dirent* ent;
    while((ent = readdir(dir)) != nullptr) {
        std::string name = ent->d_name;
        // 只取 .csv 文件
        if(name.size() > 4 && name.compare(name.size() - 4, 4, ".csv") == 0)
            files.push_back(dir_path + "/" + name);
    }
    closedir(dir);
    std::sort(files.begin(), files.end());  // 文件名排序，保证加载顺序稳定

    for(const auto& file_path : files) {
        std::ifstream file(file_path);
        std::vector<Vec2d> poly;  // 当前文件对应的多边形
        std::string line;
        while(std::getline(file, line)) {
            if(line.empty())  // 跳过空行
                continue;
            size_t comma_pos = line.find(',');
            if(comma_pos != std::string::npos) {
                try {
                    poly.push_back(Vec2d(std::stod(line.substr(0, comma_pos)),
                                         std::stod(line.substr(comma_pos + 1))));
                } catch(...) {
                }
            }
        }
        if(poly.size() >= 3)
            polygons_.push_back(poly);  // 顶点不足 3 的文件忽略
    }

    ROS_INFO("Loaded %zu exclusion polygons from %s", polygons_.size(), dir_path.c_str());
    return !polygons_.empty();
}

bool PerceptionBoundary::IsPointInExclusion(double x, double y) const {
    for(const auto& poly : polygons_) {  // 依次判断每个多边形
        if(poly.size() < 3)
            continue;
        int n = (int)poly.size(), j = n - 1;
        bool inside = false;
        for(int i = 0; i < n; j = i++) {  // 射线法判断点在多边形内
            const Vec2d& a = poly[i];
            const Vec2d& b = poly[j];
            if(((a.y > y) != (b.y > y)) &&
               (x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x))
                inside = !inside;
        }
        if(inside)
            return true;  // 命中任一多边形即判定剔除
    }
    return false;
}

std::string StringToHex(const std::string& data) {
    const std::string hex = "0123456789ABCDEF";
    std::stringstream ss;

    for(std::string::size_type i = 0; i < data.size(); ++i)
        ss << hex[(unsigned char)data[i] >> 4] << hex[(unsigned char)data[i] & 0xf];

    return ss.str();
}

void GNSSCallback(const ivlocmsg::ivmsglocpos& msg) {
    mGPS.lat = msg.lat;
    mGPS.lon = msg.lon;
    mGPS.altitude = msg.height;
    mGPS.gpsSpeed = msg.velocity;
    mGPS.xAxis = msg.xg;
    mGPS.yAxis = msg.yg;
    mGPS.zAxis = msg.zg;
    mGPS.heading = msg.heading;
    mNavigationTimestamp = ros::Time::now().toSec();
    mNavigationReceived = true;
}

void MaintainPerceptionHistory(double tNow) {
    if(!std::isfinite(tNow) || tNow < 0 ||
       (!mPerceptionHistory.empty() && tNow < mPerceptionHistory.back().second)) {
        // 时钟非法或回退时，旧窗口与已计算结果都失效；等待新的真实有效帧。
        mPerceptionHistory.clear();
        mPerception = robot::perception();
        return;
    }
    auto first_valid = std::find_if(mPerceptionHistory.begin(), mPerceptionHistory.end(),
        [tNow](const std::pair<robot::perception, double>& frame) {
            return tNow - frame.second < 2.0;
        });
    if(first_valid != mPerceptionHistory.begin()) {
        mPerceptionHistory.erase(mPerceptionHistory.begin(), first_valid);
        // 结果由旧窗口计算，删除历史后不再保留该缓存；下一真实帧会重新计算。
        mPerception = robot::perception();
        if(mPerceptionHistory.empty()) {
            ROS_WARN("Perception history expired; waiting for valid input");
        }
    }
    // 这里仅清本地状态，不发布新空帧，避免刷新下游接收时间、掩盖上游断流。
}

bool IsNavigationFresh(double tFrameTime) {
    // 坐标转换与车道匹配需要已接收、未过期且数值有效的定位，不能使用默认零值定位。
    const double age = tFrameTime - mNavigationTimestamp;
    return mNavigationReceived && std::isfinite(mNavigationTimestamp) &&
           mNavigationTimestamp >= 0 && age >= 0 && age < 2.0 &&
           std::isfinite(mGPS.xAxis) && std::isfinite(mGPS.yAxis) &&
           std::isfinite(mGPS.zAxis) && std::isfinite(mGPS.heading);
}

void BoxMsgCallBack(const visualization_msgs::MarkerArray& msg) {
    const ros::Time timestamp = ros::Time::now();
    const double frame_time = timestamp.toSec();
    // 先清理再校验本帧，持续坏帧也不能阻止历史过期。
    MaintainPerceptionHistory(frame_time);
    if(!std::isfinite(frame_time) || frame_time < 0) {
        ROS_ERROR("Invalid perception timestamp");
        return;
    }
    if(!IsNavigationFresh(frame_time)) {
        ROS_WARN_THROTTLE(1.0, "Perception skipped: localization is missing, stale or invalid");
        return;
    }
    // 原始工作帧独立于最近发布结果，失败时不会把半转换数据留在 mPerception 中。
    robot::perception perception_temp;
    perception_temp.header.stamp = timestamp;
    // 空 MarkerArray 也是一次有效观测，需参与置信度分母，不能在此提前返回。

    double x = mGPS.xAxis;
    double y = mGPS.yAxis;
    double h = 90.0 - mGPS.heading;

    if(h > 360.0)
        h -= 360.0;
    if(h < 0.0)
        h += 360.0;

    double theta = h * M_PI / 180.0;

    for(auto i : msg.markers) {
        // 仅处理 CUBE 障碍框；CenterPoint 在同一数组里还发布 TEXT_VIEW_FACING
        // 标签(scale.x/y=0)，会被当作零尺寸幻影障碍物转发到云端
        if(i.type != visualization_msgs::Marker::CUBE)
            continue;
        robot::object obj;
        double x_ = i.pose.position.x;
        double y_ = i.pose.position.y;

        double heading_to_base = tf::getYaw(i.pose.orientation);
        printf("i 朝向: %f \n", heading_to_base / M_PI * 180);
        obj.x = x + x_ * cos(theta) - y_ * sin(theta);
        obj.y = y + x_ * sin(theta) + y_ * cos(theta);
        obj.dx = i.scale.x;
        obj.dy = i.scale.y;
        // 感知排除区域过滤：在局部坐标系下判断障碍物是否落在任一多边形内
        if(g_perception_boundary.IsPointInExclusion(obj.x, obj.y)) {
            printf("Filtered out obstacle inside exclusion zone at local (%.2f, %.2f)\n", obj.x, obj.y);
            continue;  // 剔除落在多边形内的障碍物
        }
        obj.heading = fmod(heading_to_base / M_PI * 180.0 + mGPS.heading + 90, 360.0);
        if(obj.heading < 0) {
            obj.heading += 360.0;
        }

        perception_temp.objs.push_back(obj);
    }

    try {
        // 与本帧坐标、heading 转换共用 mGPS，保证 SDK 还原障碍框朝向时使用同一定位。
        // type：0 本车道、1 左一、2 左二、3 左侧车道外、4 右侧车道外。
        perception_temp = mLaneMap.ClassifyPerception(mGPS, perception_temp);
    } catch(const std::runtime_error& error) {
        ROS_ERROR("HDMap perception classification failed: %s", error.what());
        return;
    }

    const bool replace_last = !mPerceptionHistory.empty() && mPerceptionHistory.back().second == frame_time;

    // 必须先缓存 HDMap 分类后的原始帧，禁止将带历史目标的滤波结果回灌为新观测。
    mPerceptionHistory.emplace_back(std::move(perception_temp), frame_time);
    robot::perception filtered_perception;
    try {
        filtered_perception = FilterPerceptionHistory(mPerceptionHistory);
    } catch(const std::invalid_argument& error) {
        mPerceptionHistory.pop_back();  // 回滚失败帧，后续正常帧仍可继续滤波。
        ROS_ERROR("Perception temporal filter failed: %s", error.what());
        return;
    }
    const double publish_time = ros::Time::now().toSec();
    if(!std::isfinite(publish_time) || publish_time < frame_time || publish_time - frame_time >= 2.0 ||
       !IsNavigationFresh(publish_time) ||
       filtered_perception.header.stamp.toSec() != frame_time ||
       (filtered_perception.objs.empty() && !mPerceptionHistory.back().first.objs.empty())) {
        // 两次取时之间跳变或处理过久时，不能把旧结果/默认空消息当作新感知发布，定位也必须仍有效。
        // ROS 时间为零时默认 header 也为零，因此还需检查当前非空观测没有全部丢失。
        mPerceptionHistory.clear();
        mPerception = robot::perception();
        ROS_ERROR("Perception or localization expired, or clock changed during processing");
        return;
    }
    if(replace_last) {
        // 滤波成功后再替换同时间戳旧帧；暂停的仿真时钟不会使历史不断增长。
        mPerceptionHistory[mPerceptionHistory.size() - 2] = std::move(mPerceptionHistory.back());
        mPerceptionHistory.pop_back();
    }

    // 有效性校验通过后再按 0.25 筛选；允许全部筛除后发布空结果，原始历史不受影响。
    // 尺寸参数依次为宽度 [下限, 上限]、长度 [下限, 上限]（米）；正无穷表示不设上限。
    double MinWidth = 0.0;
    double MaxWidth = 4.0;
    double MinLength = 0.0;
    double MaxLength = 15.0;
    mPerception = FilterPerceptionByConfidence(filtered_perception, 0.1, MinWidth, MaxWidth, MinLength, MaxLength);
    perception_pub.publish(mPerception);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "perception_msg_convert");
    ros::NodeHandle nh;

    // 地图和空间索引只在启动时加载一次，感知回调仅做内存查询。
    std::string map_dir = ros::package::getPath("hdmap");
    if(!map_dir.empty()) {
        map_dir += "/map_processed";
    }
    nh.param<std::string>("/hdmap/map_processed_dir", map_dir, map_dir);
    hdmap::STATUS_S map_status = mLaneMap.LoadMap(map_dir);
    if(!map_status.IsOk()) {
        ROS_ERROR("Failed to load HDMap lanes from %s: %s", map_dir.c_str(), map_status.message.c_str());
        return 1;
    }
    ROS_INFO("Loaded %zu HDMap lanes from %s", mLaneMap.GetMapInfo().lane_count, map_dir.c_str());

    // 加载感知排除区域配置（写死目录，目录下每个 .csv 文件为一个排除多边形）
    std::string boundary_dir = "/home/nvidia/qingwei-L4-No2/src/pnc/config";
    if(!g_perception_boundary.LoadBoundary(boundary_dir)) {
        ROS_WARN("Failed to load perception boundary, no filtering will be applied");
    }

    gnss_sub = nh.subscribe(
        "/localization", 1, GNSSCallback,
        ros::TransportHints().tcpNoDelay());
    box_sub = nh.subscribe(
        "/box", 1, BoxMsgCallBack,
        ros::TransportHints().tcpNoDelay());

    perception_pub = nh.advertise<robot::perception>(
        "/perception", 10);

    ros::Rate loop(100);

    while(ros::ok()) {
        ros::spinOnce();
        MaintainPerceptionHistory(ros::Time::now().toSec());
        loop.sleep();
    }

    return 0;
}
