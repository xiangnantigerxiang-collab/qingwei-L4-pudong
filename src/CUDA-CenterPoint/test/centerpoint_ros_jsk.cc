
#include <cmath>
#include <tf/transform_datatypes.h>
#include "centerpoint_ros.h"

#include "cuda_runtime.h"

// headers in STL
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>


CenterPointRos::CenterPointRos()
{
    std::string model_file = "../model/rpn_centerhead_sim.plan" ;
    stream_ = nullptr;
    checkCudaErrors(cudaStreamCreate(&stream_));
    
    bool verbose = false;
    centerpoint_ptr_.reset(new CenterPoint(model_file,verbose));
    centerpoint_ptr_->prepare();

    host_points_ = new float[220000 * 5];

    checkCudaErrors(cudaMalloc(reinterpret_cast<void**>(&dev_points_),
                        220000 * 5 * sizeof(float))); // [N,4]
}

CenterPointRos::~CenterPointRos()
{
    delete[] host_points_;
    checkCudaErrors(cudaFree(dev_points_));
    checkCudaErrors(cudaStreamDestroy(stream_));
}
void CenterPointRos::CreateRosPubSub()
{
    sub_points_ = nh_.subscribe<sensor_msgs::PointCloud2>("/rslidar_points_right", 1, &CenterPointRos::PointsCallback, this);
    pub_objects_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("boxes", 1);
}


void CenterPointRos::PointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    
    int num_points = msg->width;
    int num_dim = (int)msg->fields.size(); // 这个应该=8的吧 , 但是这边显示的是num_dim = 4
    float* data = (float*)msg->data.data();
    // float* points_array = new float[num_points * 4];
    for (int i = 0 ; i < num_points ; ++i) 
    {
        host_points_[i * 5 + 0]  =  (float)data[i * 8 + 0];
        host_points_[i * 5 + 1]  =  (float)data[i * 8 + 1];
        host_points_[i * 5 + 2]  =  (float)data[i * 8 + 2];
        host_points_[i * 5 + 3]  =  (float)data[i * 8 + 3];
        host_points_[i * 5 + 4]  =  0.0f; //(float)data[i * 8 + 4];
    }

    checkCudaErrors(cudaMemcpy(dev_points_, host_points_, num_points * 5 * sizeof(float), cudaMemcpyDefault));

    float det_outputs[1000 * 10] = {0}; //center(*3) size(*3) yaw(*1) scores(*1) label(*1)
    int det_num_boxes = 0;
    // checkCudaErrors(cudaDeviceSynchronize());
    centerpoint_ptr_->doinfer(
        dev_points_, 
        num_points, 
        stream_);
    checkCudaErrors(cudaDeviceSynchronize());

    PubDetectedMarker(centerpoint_ptr_->nms_pred_, msg->header);
}


void CenterPointRos::PubDetectedMarker(std::vector<Bndbox> boxes, const std_msgs::Header& in_header)
{
    jsk_recognition_msgs::BoundingBoxArray obb_array;
    jsk_recognition_msgs::BoundingBox obb;
    obb_array.header.frame_id = "rslidar";
    obb_array.header.stamp = ros::Time::now();
    obb.header.frame_id = "rslidar";
    obb.header.stamp = ros::Time::now();
    geometry_msgs::Quaternion q;

    for (const auto box : boxes) {

      // if (box.score < 0.3) return;
        
      obb.pose.position.x = box.x;
      obb.pose.position.y = box.y;
      obb.pose.position.z = box.z;
      obb.dimensions.x = box.w;
      obb.dimensions.y = box.l;
      obb.dimensions.z = box.h;
      q = tf::createQuaternionMsgFromYaw(box.rt);
      obb.pose.orientation.x = q.x;
      obb.pose.orientation.y = q.y;
      obb.pose.orientation.z = q.z;
      obb.pose.orientation.w = q.w;
      obb.value = box.score;
      obb.label = box.id; 
      obb_array.boxes.push_back(obb);
    }
    pub_objects_.publish(obb_array);

}

