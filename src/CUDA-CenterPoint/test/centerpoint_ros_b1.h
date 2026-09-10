#pragma once
// headers in STL
#include <stdio.h>
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
#include <jsk_recognition_msgs/BoundingBoxArray.h>

// header of centerpoint
#include "centerpoint.h"

class CenterPointRos
{
private:
    // initializer list
    
    // ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_points_;
    ros::Publisher pub_objects_;

    std::unique_ptr<CenterPoint> centerpoint_ptr_;

    float* dev_points_;
    float* host_points_l_;
    float* host_points_r_;
    float* host_points_;
    cudaStream_t stream_;

    /**
    * @brief publish DetectedObject
    * @param[in] detections Network output bounding box
    * @param[in] in_header Header from pointcloud
    * @details Convert std::vector to DetectedObject, and publish them
    */
    void PubDetectedMarker(std::vector<Bndbox> boxes, const std_msgs::Header& in_header);

    /**
    * @brief callback for pointcloud
    * @param[in] input pointcloud from lidar sensor
    * @details Call point_pillars to get 3D bounding box
    */
    void PointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

public:
    CenterPointRos();
    ~CenterPointRos();
    /**
    * @brief Create ROS pub/sub obejct
    * @details Create/Initializing ros pub/sub object
    */
    void CreateRosPubSub();

};
