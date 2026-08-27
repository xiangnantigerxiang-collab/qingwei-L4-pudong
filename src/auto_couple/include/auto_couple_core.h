#pragma once

#include <stdio.h>
#include <string>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "ros/package.h"
#include <stdio.h>
#include <sensor_msgs/PointCloud.h>

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/Header.h"

#include <std_msgs/Header.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/transforms.h>
#include <pcl/search/organized.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>

#include <boost/make_shared.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>

#include <sensor_msgs/Imu.h>
#include "std_msgs/String.h"
#include "Eigen/Dense"
#include <pcl/filters/crop_box.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ivlocmsg/ivmsglocpos.h>

#define MIN_CLUSTER_SIZE 2
#define MAX_CLUSTER_SIZE 1000

class AutoCouple
{
private:
    struct Detected_Obj
    {
        jsk_recognition_msgs::BoundingBox bounding_box_;

        pcl::PointXYZ min_point_;
        pcl::PointXYZ max_point_;
        pcl::PointXYZ centroid_;
        pcl::PointXYZ center_point_;

        double center_distance_;
        double k;
    };

    ros::Subscriber sub_point_cloud_left_;
    ros::Subscriber sub_point_cloud_right_;
    ros::Subscriber sub_localization_;

    ros::Publisher pub_bounding_boxs_;
    ros::Publisher pub_couple_ ,pub_couple1_,pub_cloud;
    ros::Publisher pcl_pub;
    ros::Publisher pub_filter_points;

    float x_left = 0.0;
    float y_left = 0.0;
    float z_left = 0.0;
    float roll_left = 0.0;
    float pitch_left = 0.0;
    float yaw_left = 0.0;
    float x_right = 0.0;
    float y_right = 0.0;
    float z_right = 0.0;
    float roll_right = 0.0;
    float pitch_right = 0.0;
    float yaw_right = 0.0;
    double vehicle_x_ = 0.0;
    double vehicle_y_ = 0.0;
    double vehicle_heading_ = 0.0;
    std::vector<double> seg_distance_;
    std::vector<double> cluster_distance_;
    int target_type = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw1;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw_two = nullptr;

    std_msgs::Header point_cloud_header_;

    void segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<Detected_Obj> &obj_list);
    void segment1(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<Detected_Obj> &obj_list);
    void segment2(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<Detected_Obj> &obj_list);

    void point_plane(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    void point_plane1(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    void localization_callback(const ivlocmsg::ivmsglocpos::ConstPtr &msg);

    void filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw,
                pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_medium,
                double min, double max);

    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_; 

public:
    AutoCouple(ros::NodeHandle &nh);
    ~AutoCouple();
};
