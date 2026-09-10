#pragma once
// headers in STL
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
// header of 3rd-part
#include <yaml-cpp/yaml.h>
#include <boost/bind.hpp>
// header of ros stuff
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// #include <jsk_recognition_msgs/BoundingBoxArray.h>

// header of centerpoint
#include "centerpoint.h"

class CenterPointRos
{
private:
  ros::NodeHandle nh_;

  ros::Subscriber sub_points_l_;
  ros::Subscriber sub_points_r_;
  ros::Subscriber sub_points_m_;
  ros::Subscriber sub_points_f_;
  ros::Subscriber sub_points_bp_l_;
  ros::Subscriber sub_points_bp_r_;

  ros::Publisher pub_objects_;
  bool publisher_valid_ = true;  // publisher 是否有效 (lidar异常时设为false)

  std::vector<float> host_points_l_, host_points_r_; 
  std::vector<float> host_points_m_, host_points_f_; 
  std::vector<float> host_points_bp_l_, host_points_bp_r_; 
  std::vector<float> host_points_;

  std::unique_ptr<CenterPoint> centerpoint_ptr_;

  float* dev_points_;
  cudaStream_t stream_;
  size_t max_points_ = 50000;
  size_t max_all_points_ = 170000;

public:

  void PointsCallbackL(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ExtractPoints(msg, host_points_l_);
    MergePointClouds();
  }

  void PointsCallbackR(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ExtractPoints(msg, host_points_r_);
    MergePointClouds();
  }

  void PointsCallbackM(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ExtractPoints(msg, host_points_m_);
    MergePointClouds();
  }

  void PointsCallbackF(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ExtractPoints(msg, host_points_f_);
    MergePointClouds();
  }
 
  void PointsCallbackBPL(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ExtractPoints(msg, host_points_bp_l_);
    MergePointClouds();
  }

  void PointsCallbackBPR(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ExtractPoints(msg, host_points_bp_r_);
    MergePointClouds();
  }


  void ExtractPoints(const sensor_msgs::PointCloud2::ConstPtr& msg, std::vector<float>& host_points) {
    size_t point_count = msg->width * msg->height;
    size_t point_step = msg->point_step / sizeof(float);
    const float* data_ptr = reinterpret_cast<const float*>(&msg->data[0]);

    host_points.resize(point_count * 5);
    size_t num_valid_points = 0;
    for (int i = 0; i < point_count; ++i) {
      float x = data_ptr[i * point_step + 0]; 
      float y = data_ptr[i * point_step + 1];
      float z = data_ptr[i * point_step + 2];
      float intensity = data_ptr[i * point_step + 3];

      if (std::isnan(x) || std::isnan(y) || std::isnan(z) || 
      (x == 0.0f && y == 0.0f && z == 0.0f) ||
      x > 30.0f  || x < -12.0f || 
      (std::abs(y) > 20.0f) ||
      (z < -2.0f) || (z > 3.0f) ) { continue;}

      host_points[num_valid_points * 5 + 0] = x;
      host_points[num_valid_points * 5 + 1] = y;
      host_points[num_valid_points * 5 + 2] = z;
      host_points[num_valid_points * 5 + 3] = intensity;
      host_points[num_valid_points * 5 + 4] = 0.0f;
      num_valid_points++;
    }
    host_points.resize(num_valid_points * 5);
  }

  void MergePointClouds();
  void PubDetectedMarker(std::vector<Bndbox> boxes);
  void EgoVehicleFilter(std::vector<float>& points);
  void ShutdownPublisher();
  void RestorePublisher();
	

public:
    CenterPointRos();
    ~CenterPointRos();

    void CreateRosPubSub();
};
