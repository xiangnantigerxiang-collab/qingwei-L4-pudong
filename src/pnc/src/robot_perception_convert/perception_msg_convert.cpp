#include "perception_msg_convert.h"
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <dirent.h>

robot::perception mPerception;
robot::navigation_msg mGPS;

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
    if (!dir) {
        ROS_ERROR("Failed to open perception boundary dir: %s", dir_path.c_str());
        return false;
    }

    std::vector<std::string> files;
    struct dirent* ent;
    while ((ent = readdir(dir)) != nullptr) {
        std::string name = ent->d_name;
        // 只取 .csv 文件
        if (name.size() > 4 && name.compare(name.size() - 4, 4, ".csv") == 0)
            files.push_back(dir_path + "/" + name);
    }
    closedir(dir);
    std::sort(files.begin(), files.end());  // 文件名排序，保证加载顺序稳定

    for (const auto& file_path : files) {
        std::ifstream file(file_path);
        std::vector<Vec2d> poly;  // 当前文件对应的多边形
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty())  // 跳过空行
                continue;
            size_t comma_pos = line.find(',');
            if (comma_pos != std::string::npos) {
                try {
                    poly.push_back(Vec2d(std::stod(line.substr(0, comma_pos)),
                                         std::stod(line.substr(comma_pos + 1))));
                } catch (...) {}
            }
        }
        if (poly.size() >= 3)
            polygons_.push_back(poly);  // 顶点不足 3 的文件忽略
    }

    ROS_INFO("Loaded %zu exclusion polygons from %s", polygons_.size(), dir_path.c_str());
    return !polygons_.empty();
}

bool PerceptionBoundary::IsPointInExclusion(double x, double y) const {
    for (const auto& poly : polygons_) {  // 依次判断每个多边形
        if (poly.size() < 3)
            continue;
        int n = (int)poly.size(), j = n - 1;
        bool inside = false;
        for (int i = 0; i < n; j = i++) {  // 射线法判断点在多边形内
            const Vec2d& a = poly[i];
            const Vec2d& b = poly[j];
            if (((a.y > y) != (b.y > y)) &&
                (x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x))
                inside = !inside;
        }
        if (inside)
            return true;  // 命中任一多边形即判定剔除
    }
    return false;
}

std::string StringToHex(const std::string &data)
{
    const std::string hex = "0123456789ABCDEF";
    std::stringstream ss;

    for (std::string::size_type i = 0; i < data.size(); ++i)
        ss << hex[(unsigned char)data[i] >> 4] << hex[(unsigned char)data[i] & 0xf];

    return ss.str();
}

void GNSSCallback(const ivlocmsg::ivmsglocpos &msg)
{
    mGPS.lat = msg.lat;
    mGPS.lon = msg.lon;
    mGPS.altitude = msg.height;
    mGPS.gpsSpeed = msg.velocity;
    mGPS.xAxis = msg.xg;
    mGPS.yAxis = msg.yg;
    mGPS.zAxis = msg.zg;
    mGPS.heading = msg.heading;
}

void BoxMsgCallBack(const visualization_msgs::MarkerArray &msg)
{
    mPerception.objs.clear();

    if (!msg.markers.size())
        return;

    double x = mGPS.xAxis;
    double y = mGPS.yAxis;
    double h = 90.0 - mGPS.heading;

    if (h > 360.0)
        h -= 360.0;
    if (h < 0.0)
        h += 360.0;

    double theta = h * M_PI / 180.0;

    for (auto i : msg.markers)
    {
        // 仅处理 CUBE 障碍框；CenterPoint 在同一数组里还发布 TEXT_VIEW_FACING
        // 标签(scale.x/y=0)，会被当作零尺寸幻影障碍物转发到云端
        if (i.type != visualization_msgs::Marker::CUBE)
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
        if (g_perception_boundary.IsPointInExclusion(obj.x, obj.y)) {
            printf("Filtered out obstacle inside exclusion zone at local (%.2f, %.2f)\n", obj.x, obj.y);
            continue;  // 剔除落在多边形内的障碍物
        }
        obj.heading = fmod(heading_to_base / M_PI * 180.0 + mGPS.heading + 90, 360.0);
        if (obj.heading < 0)
        {
            obj.heading += 360.0;
        }

        mPerception.objs.push_back(obj);
    }

    perception_pub.publish(mPerception);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "perception_msg_convert");
    ros::NodeHandle nh;

    // 加载感知排除区域配置（写死目录，目录下每个 .csv 文件为一个排除多边形）
    std::string boundary_dir = "/home/nvidia/qingwei-L4-No2/src/pnc/config";
    if (!g_perception_boundary.LoadBoundary(boundary_dir)) {
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

    while (ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}

