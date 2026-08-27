#pragma once

#include <deque>
#include <iostream>
#include <mutex>
#include <stdio.h>
#include <string>
#include <thread>
#include <vector>

#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <Eigen/Dense>

#include <boost/make_shared.hpp>

#include <tf/transform_listener.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <laser_geometry/laser_geometry.h>
#include <lidar_perception/perception_area.h>
#include <lidar_perception/perception_area_array.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

#include "auto_couple/center_position.h"

struct ROI {
  float min_x = 0.2;
  float max_x = 20.0;
  float min_y = -2.0;
  float max_y = 2.0;
  float min_z = -1.0;
  float max_z = 1.0;
  friend std::ostream &operator<<(std::ostream &os, ROI &roi) {
    os << "ROI params: min_x: " << roi.min_x << " max_x: " << roi.max_x
       << " min_y: " << roi.min_y << " max_y: " << roi.max_y
       << " min_z: " << roi.min_z << " max_z: " << roi.max_z << "\n";
    return os;
  }
};

struct Detected_Obj {
  int64_t id;
  jsk_recognition_msgs::BoundingBox bounding_box;
  lidar_perception::perception_area area;
  pcl::PointXYZ min_point;
  pcl::PointXYZ max_point;
  pcl::PointXYZ centroid;
  pcl::PointXYZ center_point;
  double center_distance;
  double k;
};

class Segmenter {
public:
  Segmenter() = default;
  Segmenter(ros::NodeHandle node, ros::NodeHandle private_handle);
  ~Segmenter() = default;

  void FrontCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

  void BackCallback(const sensor_msgs::PointCloud2::Ptr &scan_msg);

  void HookPosCallback(const auto_couple::center_position::Ptr &hook_pos_msg);

  void SegmentFront(std::vector<Detected_Obj> &obj_list);

  void SegmentBack(std::vector<Detected_Obj> &obj_list);

private:
  void GetFrontROIPc(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

  void GetBackROIPc(const sensor_msgs::PointCloud2::Ptr &scan_msg);

  void GetLastestHookPos();

  void FrontROIFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_raw,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_medium);

  void BackROIFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_medium);

  void GetObjects(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc,
                  const std::vector<pcl::PointIndices> &local_indices,
                  std::vector<Detected_Obj> &obj_list,
                  jsk_recognition_msgs::BoundingBoxArray &bbox_array);

  float Point2Distance(const pcl::PointXYZI &p) {
    return sqrt(p.x * p.x + p.y * p.y);
  }

  void BackROIFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_raw);

private:
  ros::NodeHandle node_;
  ros::NodeHandle private_nh_;

  std::string frame_id_ = "laser";
  std::string front_sub_topic_ = "/front_scan";
  std::string back_sub_topic_ = "/back_pointcloud";
  std::string hook_pos_sub_topic_ = "/hook_position";

  std::string front_bbox_pub_topic_ = "/perception_front_bbox";
  std::string back_bbox_pub_topic_ = "/perception_back_bbox";

  std::string segment_front_input_pc_ = "/segment_front_input_pc";
  std::string segment_back_input_pc_ = "/segment_back_input_pc";

  ROI front_roi_;
  ROI back_roi_;
  ROI back_roi_copy_;
  float back_min_x_buffer_ = 0.2;

  std::vector<double> dis_level_;

  ros::Subscriber front_pc_sub_;
  ros::Subscriber hook_pos_sub_;
  ros::Subscriber back_pc_sub_;

  ros::Publisher front_roi_pc_pub_;
  ros::Publisher back_roi_pc_pub_;

  ros::Publisher front_bbox_pub_;
  ros::Publisher back_bbox_pub_;
  int last_max_obj_ = 0;

  std::mutex mutex_;
  std::deque<sensor_msgs::PointCloud2::Ptr> back_pc_que_;
  std::deque<auto_couple::center_position::Ptr> hook_pos_que_;

  auto_couple::center_position::Ptr hook_pos_latest_ = nullptr;

  pcl::PointCloud<pcl::PointXYZI>::Ptr front_pc_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr back_pc_;
};
