#include "auto_couple_core.h"
#include <vector>
#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include "Eigen/Dense"
#include "auto_couple/center_position.h"
#include <pcl/filters/crop_box.h>

struct Point_tmp {
    double x, y;
    
    // 计算到原点的距离平方
    double distSqToOrigin() const {
        return x * x + y * y;
    }
    // 计算到原点的距离
    double distToOrigin() const {
        return sqrt(distSqToOrigin());
    }
    // 打印点
    void print() const {
        std::cout << "(" << x << ", " << y << ")";
    }
};

// 计算两点中垂线上距离AB直线为d的点，返回离原点更近的那个
Point_tmp nearestPointOnPerpBisectorAtDistance(const Point_tmp& A, const Point_tmp& B, double d) {
    Point_tmp result;
    
    // 计算差值
    double dx = B.x - A.x;
    double dy = B.y - A.y;
    
    // 计算AB长度
    double L = sqrt(dx * dx + dy * dy);
    
    // 如果两点重合，无法确定中垂线
    if (L < 1e-12) {
        cout << "错误：两点重合，无法确定中垂线！" << endl;
        result.x = result.y = 0;
        return result;
    }
    
    // 如果距离d为0，直接返回中点
    if (fabs(d) < 1e-12) {
        result.x = (A.x + B.x) / 2.0;
        result.y = (A.y + B.y) / 2.0;
        return result;
    }
    
    // 中点 M
    Point_tmp M;
    M.x = (A.x + B.x) / 2.0;
    M.y = (A.y + B.y) / 2.0;
    
    // 单位法向量（垂直于AB）
    double nx = -dy / L;
    double ny = dx / L;
    
    // 两个候选点：M + d*n 和 M - d*n
    Point_tmp P1, P2;
    P1.x = M.x + d * nx;
    P1.y = M.y + d * ny;
    P2.x = M.x - d * nx;
    P2.y = M.y - d * ny;
    
    // 选择离原点更近的那个
    if (P1.distSqToOrigin() <= P2.distSqToOrigin()) {
        result = P1;
    } else {
        result = P2;
    }
    
    return result;
}

double angleWithXAxisClockwise(const Point_tmp& P) {
    // 使用 atan2 计算角度（弧度），标准是逆时针为正
    double angleRad = atan2(P.y, P.x);
    
    // 转换为角度（逆时针为正）
    double angleDeg = angleRad * 180.0 / M_PI;
    
    // 转换为顺时针为正：取负
    double clockwiseAngle = -angleDeg;
    
    // 控制在 [-90°, 90°] 范围内
    if (clockwiseAngle > 90.0) {
        clockwiseAngle -= 180.0;
    } else if (clockwiseAngle < -90.0) {
        clockwiseAngle += 180.0;
    }
    
    return clockwiseAngle;
}

// 计算中垂线与X轴正方向的夹角
// 返回角度（度），顺时针为正，逆时针为负，范围 [-90°, 90°]
double angleOfPerpBisectorWithXAxis(const Point_tmp& A, const Point_tmp& B) {
    // 计算AB方向向量
    double dx = B.x - A.x;
    double dy = B.y - A.y;
    double L = sqrt(dx * dx + dy * dy);
    
    // 如果两点重合，无法确定中垂线
    if (L < 1e-12) {
        cout << "错误：两点重合，无法确定中垂线！" << endl;
        return 0;
    }
    
    // 中垂线方向向量（垂直于AB）
    // 这里取逆时针旋转90度: n = (-dy/L, dx/L)
    double nx = -dy / L;
    double ny = dx / L;
    
    // 计算与X轴正方向的夹角（弧度）
    // atan2(ny, nx) 返回的是逆时针为正的角度
    double angleRad = atan2(ny, nx);
    
    // 转换为角度（逆时针为正）
    double angleDeg = angleRad * 180.0 / M_PI;
    
    // 转换为顺时针为正（取负）
    double clockwiseAngle = -angleDeg;
    
    // 控制在 [-90°, 90°] 范围内
    if (clockwiseAngle > 90.0) {
        clockwiseAngle -= 180.0;
    } else if (clockwiseAngle < -90.0) {
        clockwiseAngle += 180.0;
    }
    
    return clockwiseAngle;
}


AutoCouple::AutoCouple(ros::NodeHandle &nh)
{
    nh.param("x_left", x_left, x_left);
    nh.param("y_left", y_left, y_left);
    nh.param("z_left", z_left, z_left);
    nh.param("roll_left", roll_left, roll_left);
    nh.param("pitch_left", pitch_left, pitch_left);
    nh.param("yaw_left", yaw_left, yaw_left);
    nh.param("x_right", x_right, x_right);
    nh.param("y_right", y_right, y_right);
    nh.param("z_right", z_right, z_right);
    nh.param("roll_right", roll_right, roll_right);
    nh.param("pitch_right", pitch_right, pitch_right);
    nh.param("yaw_right", yaw_right, yaw_right);
    nh.param("target_type", target_type,target_type);

    sub_localization_ = nh.subscribe("/localization", 10, &AutoCouple::localization_callback, this);

    sub_point_cloud_right_ = nh.subscribe("/back_left_scan", 10, &AutoCouple::point_plane1, this);
    sub_point_cloud_left_ = nh.subscribe("/back_right_scan", 10, &AutoCouple::point_plane, this);

    tfListener_.setExtrapolationLimit(ros::Duration(0.1));

    pub_bounding_boxs_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bounding_box", 10);
    pub_couple_ = nh.advertise<auto_couple::center_position>("/hook_position", 10);
    pub_couple1_ = nh.advertise<auto_couple::center_position>("/hook_position1", 10);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/back_pointcloud", 10);

    ros::spin();
}

AutoCouple::~AutoCouple() {}
void AutoCouple::localization_callback(const ivlocmsg::ivmsglocpos::ConstPtr &msg)
{
    vehicle_x_ = msg->xg;
    vehicle_y_ = msg->yg;
    vehicle_heading_ = msg->heading;
}
void AutoCouple::filter(
         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw, 
         pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_medium ,
         double min, double max)
{
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_region(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::CropBox<pcl::PointXYZI> region(true); 

    region.setMin(Eigen::Vector4f(0.3 , -3.0, -1.4 , 1.0));    
    region.setMax(Eigen::Vector4f(10.3 , 3.0 , 1.4 , 1.0));    
    region.setInputCloud(cloud_raw);  
    region.filter(*cloud_region); 
	
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_region, output);
    output.header.frame_id = "rslidar";

    pcl::PassThrough<pcl::PointXYZI> pass_i;    
    pass_i.setInputCloud(cloud_region);           
    pass_i.setFilterFieldName("intensity");    
    pass_i.setNegative(false);            
    pass_i.setFilterLimits(min, max);         
    pass_i.filter(*cloud_medium);             
}

void AutoCouple::segment2(
         pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc ,
         std::vector<Detected_Obj> &obj_list)
{
    auto_couple::center_position msg;

    // ==== 托盘的绝对位置（全局坐标系） ====
    // beta 约定：北向=0°，顺时针正，逆时针负，范围 ±180°
    // （与 /localization 的 heading 约定一致）
    double tray_abs_x    = -21.30;
    double tray_abs_y    = -18.92;
    double tray_abs_beta = -18.75;

    // ==== 车的绝对位姿（来自 /localization） ====
    double car_x           = vehicle_x_;
    double car_y           = vehicle_y_;
    double car_heading_deg = vehicle_heading_;  // 北向=0，顺正逆负，±180°

    // ==== 全局偏移 ====
    double dx_abs = tray_abs_x - car_x;
    double dy_abs = tray_abs_y - car_y;
    // ==== 调试：打印绝对位置和全局偏移 ====
    std::cout << "====== segment2 debug ======" << std::endl;
    std::cout << "tray_abs: (" << tray_abs_x << ", " << tray_abs_y << "), beta=" << tray_abs_beta << std::endl;
    std::cout << "car_abs:  (" << car_x << ", " << car_y << "), heading=" << car_heading_deg << std::endl;
    std::cout << "global offset (dx,dy)=(" << dx_abs << ", " << dy_abs << ")" << std::endl;
    std::cout << "global dist = " << hypot(dx_abs, dy_abs) << std::endl;
    // ==== 旋转到车辆相对坐标系 ====
    //
    // 车辆坐标系（与 segment() 一致）：
    //   x 向车后为正
    //   y = x 逆时针 90°（即车右侧为正）
    //   beta: x轴=0°，顺时针正，逆时针负
    //
    // heading=0（北）时：x后=(0,-1)=南, y右=(1,0)=东
    // 通用公式：
    //   车后向(x) 在全局 = (-sinθ, -cosθ)
    //   车右向(y) 在全局 = ( cosθ, -sinθ)
    //
    double heading_rad = car_heading_deg * M_PI / 180.0;
    double rel_x = -dx_abs * sin(heading_rad) - dy_abs * cos(heading_rad);   // 投影到车后向
    double rel_y =  dx_abs * cos(heading_rad) - dy_abs * sin(heading_rad);   // 投影到车右向
    std::cout << "before offset: rel_x=" << rel_x << ", rel_y=" << rel_y
              << ", dist=" << hypot(rel_x, rel_y) << std::endl;
              
    // ==== 原点修正：后轴中心后 0.5m（x 正方向），新_x = 旧_x - 0.5 ====
//    rel_x += 0.75;
    std::cout << "after offset: rel_x=" << rel_x << ", rel_y=" << rel_y << std::endl;

    // ==== 相对角度转换 ====
    // x_rear 在全局的朝向 = heading + 180°（从北向顺正）
    // rel_beta = tray_abs_beta - (heading + 180) = tray_abs_beta - heading - 180
    double rel_beta = tray_abs_beta - car_heading_deg - 180.0;

    // 归一化到 [-90°, 90°]，与 segment() 输出范围一致
    while (rel_beta > 90.0)  rel_beta -= 180.0;
    while (rel_beta < -90.0) rel_beta += 180.0;
    std::cout << "rel_beta = " << rel_beta << std::endl;
    std::cout << "============================" << std::endl;
    // ==== 发送消息（与 segment() 相同的相对坐标格式） ====
    msg.center_point_x = rel_x;
    msg.center_point_y = rel_y;
    msg.beta           = rel_beta;
    msg.center_distance = hypot(rel_x, rel_y);

    pub_couple1_.publish(msg);
}

void AutoCouple::segment1(
         pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc ,
         std::vector<Detected_Obj> &obj_list)
{//不对，这里应该发送和之前一样的坐标系
    auto_couple::center_position msg;
    msg.center_point_x = -21.30;
    msg.center_point_y = -18.92;
    msg.beta =  -18.75;//341.25;
    msg.center_distance = hypot(msg.center_point_x-vehicle_x_, msg.center_point_y-vehicle_y_);
    pub_couple_.publish(msg);


}
void AutoCouple::segment(
         pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc ,
         std::vector<Detected_Obj> &obj_list)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_pc, *cloud_2d);
    
    for(size_t i = 0; i < cloud_2d->points.size(); i++) cloud_2d->points[i].z = 0;

    if(cloud_2d->points.size() > 0) tree->setInputCloud(cloud_2d);

    std::vector<pcl::PointIndices> local_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> Euclid;

    Euclid.setInputCloud(cloud_2d);
    Euclid.setClusterTolerance(0.3);
    Euclid.setMinClusterSize(MIN_CLUSTER_SIZE);
    Euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
    Euclid.setSearchMethod(tree);
    Euclid.extract(local_indices);

    struct Point {
        double x;
        double y;
        double z;
    };

    std::vector<Point> points;
    
    Point point[50];
    Detected_Obj reflect_sticker;

    for(size_t i = 0; i < local_indices.size(); i++) {
        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();

        for(auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit) {
            pcl::PointXYZ p;

            p.x = in_pc->points[*pit].x;
            p.y = in_pc->points[*pit].y ;
            p.z = in_pc->points[*pit].z;

            reflect_sticker.centroid_.x += p.x;
            reflect_sticker.centroid_.y += p.y;
            reflect_sticker.centroid_.z += p.z;

            if(p.x < min_x) min_x = p.x;
            if(p.y < min_y) min_y = p.y;
            if(p.z < min_z) min_z = p.z;
            if(p.x > max_x) max_x = p.x;
            if(p.y > max_y) max_y = p.y;
            if(p.z > max_z) max_z = p.z;
        }
        
        reflect_sticker.min_point_.x = min_x;
        reflect_sticker.min_point_.y = min_y;
        reflect_sticker.min_point_.z = min_z;

        reflect_sticker.max_point_.x = max_x;
        reflect_sticker.max_point_.y = max_y;
        reflect_sticker.max_point_.z = max_z;
        
        if(local_indices[i].indices.size() > 0) {
            reflect_sticker.centroid_.x /= local_indices[i].indices.size();
            reflect_sticker.centroid_.y /= local_indices[i].indices.size();
            reflect_sticker.centroid_.z /= local_indices[i].indices.size();
        }
      
        double length_ = reflect_sticker.max_point_.x - reflect_sticker.min_point_.x;
        double width_ = reflect_sticker.max_point_.y - reflect_sticker.min_point_.y;
        double height_ = reflect_sticker.max_point_.z - reflect_sticker.min_point_.z;

        reflect_sticker.bounding_box_.header = point_cloud_header_;
        reflect_sticker.bounding_box_.pose.position.x = reflect_sticker.min_point_.x + length_ / 2;
        reflect_sticker.bounding_box_.pose.position.y = reflect_sticker.min_point_.y + width_ / 2;
        reflect_sticker.bounding_box_.pose.position.z = reflect_sticker.min_point_.z + height_ / 2;
        reflect_sticker.bounding_box_.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);
        reflect_sticker.bounding_box_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
        reflect_sticker.bounding_box_.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);

        obj_list.push_back(reflect_sticker);

        point[i].x=reflect_sticker.bounding_box_.pose.position.x;
        point[i].y=reflect_sticker.bounding_box_.pose.position.y;
        point[i].z=reflect_sticker.bounding_box_.pose.position.z;

        points.push_back(point[i]);
    }	 
    double dis_min = 2.8,dis_max = 3.4;
    // 聚类数量不足，无法计算
    if(local_indices.size() < 2) {
        std::cout << "[auto_couple] cluster count=" << local_indices.size() << "(<2), skip" << std::endl;
        return;
    }
    std::cout << "[auto_couple] total clusters: " << local_indices.size() << std::endl;
    for(size_t i = 0; i < local_indices.size(); i++) {
        std::cout << "  cluster[" << i << "]: x=" << point[i].x << " y=" << point[i].y
                  << "pts=" << local_indices[i].indices.size() << std::endl;
    }

    // 打印所有聚类对的距离
    for(size_t i = 0; i < local_indices.size(); i++) {
        for(size_t j = i + 1; j < local_indices.size(); j++) {
            double dist = hypot(point[i].x - point[j].x, point[i].y - point[j].y);
            std::cout << "  pair[" << i << "-" << j << "] dist=" << dist
                      << (dist >= dis_min && dist <= dis_max ? " [OK]" : " [OUT]") << std::endl;
        }
    }
    // 筛选距离在 [distance_min_, distance_max_] 范围内、最接近 distance_target_ 的一对聚类
    double distance_target_ = (dis_min + dis_max) / 2.0;
    size_t best_i = 0, best_j = 1;
    double best_diff = std::numeric_limits<double>::max();
    bool found = false;
    for(size_t i = 0; i < local_indices.size(); i++) {
        for(size_t j = i + 1; j < local_indices.size(); j++) {
            double dist = hypot(point[i].x - point[j].x, point[i].y - point[j].y);
            if(dist < dis_min || dist > dis_max) continue;  // 不在范围内，跳过
            double diff = std::abs(dist - distance_target_);
    double a;
    
    if(point[i].x > point[j].x) a = atan2(point[i].y-point[j].y, point[i].x-point[j].x);
    else a = atan2(point[j].y-point[i].y, point[j].x-point[i].x);
    
        double beta ,beta_out,angle_deg;
    beta_out = (a*180.) / M_PI;

    if(beta_out > (-90) && beta_out < 0) angle_deg = (-(90 + beta_out)) ;
    else if(beta_out>0 && beta_out< 90) angle_deg = 90 - beta_out ;
    
    
    
             std::cout << "  pair[" << i  << "-" << j << "] deg=" << angle_deg << std::endl;
             
             
             
             
             
            if(abs(angle_deg) > 40) continue;
            
//            if(diff < best_diff) {
//            if(angle_deg < best_diff) {
                best_diff = diff;
                best_i = i;
                best_j = j;
                found = true;
//            }
        }
    }
    if(!found) {
        std::cout << "[auto_couple] no pair in [" << dis_min << ", "
                  << dis_max << "], skip" << std::endl;
        return;
    }
    std::cout << "[auto_couple] selected pair[" << best_i << "-" << best_j << "], dist="
              << hypot(point[best_i].x - point[best_j].x, point[best_i].y - point[best_j].y) << std::endl;
    
    
    


    Point_tmp A1 = {point[best_i].x, point[best_i].y};
    Point_tmp B1 = {point[best_j].x, point[best_j].y};
    double d = 1.48;
    
    Point_tmp result = nearestPointOnPerpBisectorAtDistance(A1, B1, d);
    
    std::cout << "A(0,0), B(4,0), d=" << d << "meter" << std::endl;
    std::cout << "result：";
    result.print();
    std::cout << ",todisorin = " << sqrt(result.distSqToOrigin()) << std::endl;
    double angle = angleWithXAxisClockwise(result);
    double angle1 = angleOfPerpBisectorWithXAxis(A1, B1);

    std::cout << "angle_zyd = " << angle <<"  " <<angle1<<std::endl; //angle是拖挂挂钩和原点连线            angle1是反光板中垂线和车的偏差          之前的beta角是反光板中心与原点连线



    Point center_point ;
    reflect_sticker.center_point_.x=(point[best_i].x+point[best_j].x)/2;
    reflect_sticker.center_point_.y=(point[best_i].y+point[best_j].y)/2;

    double x = reflect_sticker.center_point_.x;
    double y = reflect_sticker.center_point_.y;
    reflect_sticker.center_distance_ = hypot(x, y);

    double a;
    if(point[best_i].x > point[best_j].x) a = atan2(point[best_i].y-point[best_j].y, point[best_i].x-point[best_j].x);
    else a = atan2(point[best_j].y-point[best_i].y, point[best_j].x-point[best_i].x);
    
    double beta ,beta_out;
    auto_couple::center_position msg;
    beta_out = (a*180.) / M_PI;

    if(beta_out > (-90) && beta_out < 0) msg.beta = (-(90 + beta_out)) ;
    else if(beta_out>0 && beta_out< 90) msg.beta = 90 - beta_out ;

    msg.center_point_x = reflect_sticker.center_point_.x - 1.48;//0.86;
    msg.center_point_y = reflect_sticker.center_point_.y ;

    x = msg.center_point_x;
    y = msg.center_point_y;
    msg.center_distance = hypot(x, y);




    msg.center_point_x = result.x;
    msg.center_point_y = result.y;
    msg.beta =  angle1;
    msg.center_distance = hypot(msg.center_point_x, msg.center_point_y);
    pub_couple_.publish(msg);

}
void AutoCouple::point_plane1(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    sensor_msgs::PointCloud2 cloud1;
    projector_.transformLaserScanToPointCloud("robot", *scan_msg, cloud1, tfListener_);

    cloud_raw1.reset(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(cloud1, *cloud_raw1);

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << x_left, y_left, z_left;
    transform_2.rotate(Eigen::AngleAxisf(roll_left, Eigen::Vector3f::UnitZ()));
    transform_2.rotate(Eigen::AngleAxisf(pitch_left, Eigen::Vector3f::UnitY()));
    transform_2.rotate(Eigen::AngleAxisf(yaw_left, Eigen::Vector3f::UnitX()));

    pcl::transformPointCloud(*cloud_raw1, *cloud_raw1, transform_2);
}

void AutoCouple::point_plane(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    double set_min = 600.0;//245
    double set_max = 1550.0;//255

    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("robot", *scan_msg, cloud, tfListener_);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_medium(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(cloud, *cloud_raw);

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << x_right, y_right, z_right;
    transform_2.rotate(Eigen::AngleAxisf(roll_right, Eigen::Vector3f::UnitZ()));
    transform_2.rotate(Eigen::AngleAxisf(pitch_right, Eigen::Vector3f::UnitY()));
    transform_2.rotate(Eigen::AngleAxisf(yaw_right, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*cloud_raw, *cloud_raw, transform_2);

    if(cloud_raw1 != nullptr) {
        for(unsigned int i = 0; i < cloud_raw1->points.size(); ++i) {
            const auto& in_pt = cloud_raw1->points[i];
            pcl::PointXYZI pt;
            pt.x = in_pt.x;
            pt.y = in_pt.y;
            pt.z = in_pt.z;
            pt.intensity = in_pt.intensity;

            cloud_raw->push_back(pt);
        }
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_raw, output);
    output.header.frame_id = "robot";
    pcl_pub.publish(output);

    if(cloud_raw ->empty()) std::cout<<"the input cloud is empty!!"<<std::endl;

    filter(cloud_raw,cloud_medium,set_min,set_max);

    sensor_msgs::PointCloud2 output_points;
    pcl::toROSMsg(*cloud_medium, output_points);

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    point_cloud_header_ = scan_msg->header;
    pcl::fromROSMsg(output_points, *current_pc_ptr);

    std::vector<Detected_Obj> global_obj_list;
    if(target_type == 0){
    segment1(current_pc_ptr, global_obj_list);
    segment2(current_pc_ptr, global_obj_list);

    }else
    {
    segment(current_pc_ptr, global_obj_list);
//    segment2(current_pc_ptr, global_obj_list);
    }
    jsk_recognition_msgs::BoundingBoxArray bbox_array;

    int n = global_obj_list.size();

    for(size_t i = 0; i < n; i++) bbox_array.boxes.push_back(global_obj_list[i].bounding_box_);
        
    bbox_array.header = point_cloud_header_;

    pub_bounding_boxs_.publish(bbox_array);
}
