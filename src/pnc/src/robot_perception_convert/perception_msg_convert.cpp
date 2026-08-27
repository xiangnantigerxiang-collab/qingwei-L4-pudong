#include "perception_msg_convert.h"
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Matrix3x3.h>

robot::perception mPerception;
robot::navigation_msg mGPS;

ros::Subscriber gnss_sub;
ros::Subscriber box_sub;
ros::Publisher perception_pub;
ros::Publisher udp_pub;
ros::Publisher string_pub;

PerceptionBoundary g_perception_boundary;

// 感知边界实现
bool PerceptionBoundary::LoadBoundary(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open perception boundary file: %s", file_path.c_str());
        return false;
    }
    
    boundary_points_.clear();
    std::string line;
    while (std::getline(file, line)) {
        // 跳过空行
        if (line.empty()) continue;
        
        // 解析 x,y 坐标
        size_t comma_pos = line.find(',');
        if (comma_pos != std::string::npos) {
            try {
                double x = std::stod(line.substr(0, comma_pos));
                double y = std::stod(line.substr(comma_pos + 1));
                boundary_points_.push_back(Vec2d(x, y));
            } catch (const std::exception& e) {
                ROS_WARN("Failed to parse boundary line: %s", line.c_str());
            }
        }
    }
    file.close();
    
    if (boundary_points_.empty()) {
        ROS_ERROR("No boundary points loaded");
        return false;
    }
    
    ROS_INFO("Loaded %zu perception boundary points", boundary_points_.size());
    return true;
}

bool PerceptionBoundary::IsPointInBoundary(double x, double y) const {
    return IsPointInBoundary(Vec2d(x, y));
}

bool PerceptionBoundary::IsPointInBoundary(const Vec2d& point) const {
    if (boundary_points_.size() < 3) {
        return true;  // 没有边界定义时，所有点都有效
    }
    
    const int n = boundary_points_.size();
    int j = n - 1;
    bool c = false;
    
    for (int i = 0; i < n; j = i++) {
        const Vec2d& pi = boundary_points_[i];
        const Vec2d& pj = boundary_points_[j];
        
        // 点在边的y范围内
        if (((pi.y > point.y) != (pj.y > point.y)) &&
            (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x)) {
            c = !c;
        }
    }
    
    return c;
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
        robot::object obj;
        double x_ = i.pose.position.x;
        double y_ = i.pose.position.y;
        

        
        double heading_to_base = tf::getYaw(i.pose.orientation);
        printf("i 朝向: %f \n", heading_to_base / M_PI * 180);
        obj.x = x + x_ * cos(theta) - y_ * sin(theta);
        obj.y = y + x_ * sin(theta) + y_ * cos(theta);
        obj.dx = i.scale.x;
        obj.dy = i.scale.y;
        // 感知边界过滤：在局部坐标系下判断障碍物是否在边界内
        if (!g_perception_boundary.IsPointInBoundary(obj.x, obj.y)) {
            printf("Filtered out obstacle at local (%.2f, %.2f)\n", obj.x, obj.y);
            continue;  // 跳过边界外的障碍物
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

    // 加载感知边界配置（写死路径）
    std::string boundary_file = "/home/nvidia/qingwei-L4-No2/src/pnc/config/perception_boundary.csv";
    if (!g_perception_boundary.LoadBoundary(boundary_file)) {
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

