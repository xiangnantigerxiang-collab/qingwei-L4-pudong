#include "lidar_perception.h"

constexpr int64_t MIN_CLUSTER_SIZE = 1;
constexpr int64_t MAX_CLUSTER_SIZE = 1000;
constexpr double kClusterTolerance = 0.15;

static int counter = 0;

Segmenter::Segmenter(ros::NodeHandle node, ros::NodeHandle private_handle) {
  node_ = node;
  private_nh_ = private_handle;

  private_nh_.getParam("frame_id", frame_id_);
  private_nh_.getParam("front_sub_topic", front_sub_topic_);
  private_nh_.getParam("back_sub_topic", back_sub_topic_);
  private_nh_.getParam("hook_pos_sub_topic", hook_pos_sub_topic_);

  private_nh_.getParam("front_bbox_pub_topic", front_bbox_pub_topic_);
  private_nh_.getParam("back_bbox_pub_topic", back_bbox_pub_topic_);

  private_nh_.getParam("segment_front_input_pc_", segment_front_input_pc_);
  private_nh_.getParam("segment_back_input_pc_", segment_back_input_pc_);

  private_nh_.getParam("front_min_x", front_roi_.min_x);
  private_nh_.getParam("front_max_x", front_roi_.max_x);
  private_nh_.getParam("front_min_y", front_roi_.min_y);
  private_nh_.getParam("front_max_y", front_roi_.max_y);
  private_nh_.getParam("front_min_z", front_roi_.min_z);
  private_nh_.getParam("front_max_z", front_roi_.max_z);

  private_nh_.getParam("dis_level", dis_level_);
  dis_level_.emplace_back(std::numeric_limits<double>::max());

  private_nh_.getParam("back_min_x_buffer", back_min_x_buffer_);
  private_nh_.getParam("back_min_x", back_roi_.min_x);
  private_nh_.getParam("back_max_x", back_roi_.max_x);
  private_nh_.getParam("back_min_y", back_roi_.min_y);
  private_nh_.getParam("back_max_y", back_roi_.max_y);
  private_nh_.getParam("back_min_z", back_roi_.min_z);
  private_nh_.getParam("back_max_z", back_roi_.max_z);
  back_roi_copy_ = back_roi_;

  front_pc_sub_ =
      node_.subscribe(front_sub_topic_, 10, &Segmenter::FrontCallback, this);
  front_roi_pc_pub_ =
      node_.advertise<sensor_msgs::PointCloud2>(segment_front_input_pc_, 10);

  back_pc_sub_ =
      node_.subscribe(back_sub_topic_, 10, &Segmenter::BackCallback, this);
  back_roi_pc_pub_ =
      node_.advertise<sensor_msgs::PointCloud2>(segment_back_input_pc_, 10);

  hook_pos_sub_ = node_.subscribe(hook_pos_sub_topic_, 10,
                                  &Segmenter::HookPosCallback, this);

  front_bbox_pub_ = node_.advertise<jsk_recognition_msgs::BoundingBoxArray>(
      front_bbox_pub_topic_, 10);
  back_bbox_pub_ = node_.advertise<jsk_recognition_msgs::BoundingBoxArray>(
      back_bbox_pub_topic_, 10);
}

void Segmenter::FrontROIFilter(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_raw,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_medium) {
  pcl::CropBox<pcl::PointXYZI> region(true);
  region.setMin(Eigen::Vector4f(front_roi_.min_x, front_roi_.min_y,
                                front_roi_.min_z, 1.0));
  region.setMax(Eigen::Vector4f(front_roi_.max_x, front_roi_.max_y,
                                front_roi_.max_z, 1.0));
  region.setInputCloud(cloud_raw);
  region.filter(*cloud_medium);
}

void Segmenter::BackROIFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_raw) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
    float max_x =  back_roi_copy_.max_x + back_min_x_buffer_;
  if(hook_pos_latest_ != nullptr){
    std::cout<<hook_pos_latest_->center_point_x<<"   "<<back_roi_.max_x<<"  "<<hook_pos_latest_->center_point_x\
  <<"   "<<back_roi_.min_x<<std::endl;}
    if (hook_pos_latest_ != nullptr && (hook_pos_latest_->center_point_x < back_roi_.max_x) &&
     (hook_pos_latest_->center_point_x > back_roi_.min_x)) {
        max_x =  hook_pos_latest_->center_point_x + back_min_x_buffer_;
            ROS_INFO("Back ROI min_x: %f, max_x: %f, hook position x: %f, Will set %f "
              "to ROI max_x!",
              back_roi_copy_.min_x, back_roi_copy_.max_x,
              hook_pos_latest_->center_point_x, max_x );
  } else if (hook_pos_latest_ != nullptr ){
          ROS_ERROR("Back ROI min_x: %f, max_x: %f, hook position x: %f, Will use default back ROI! ",
              back_roi_copy_.min_x, back_roi_copy_.max_x,
              hook_pos_latest_->center_point_x);
  }

  pcl::CropBox<pcl::PointXYZI> region(true);
  region.setMin(Eigen::Vector4f(back_roi_.min_x,
                                back_roi_.min_y, back_roi_.min_z, 1.0));
  region.setMax(
      Eigen::Vector4f(max_x, back_roi_.max_y, back_roi_.max_z, 1.0));

  region.setInputCloud(cloud_raw);
  region.filter(*cloud);
  cloud_raw.swap(cloud);
  sensor_msgs::PointCloud2 out;
  pcl::toROSMsg(*cloud, out);
  out.header.frame_id = frame_id_;
  back_roi_pc_pub_.publish(out);
}

void Segmenter::SegmentFront(std::vector<Detected_Obj> &obj_list) {
  // Segment front ROI.
  jsk_recognition_msgs::BoundingBoxArray front_bbox_array;
  front_bbox_array.header.frame_id = frame_id_;
  if (front_pc_ != nullptr && front_pc_->size() > 0) {
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(front_pc_);
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> euclid;
    euclid.setInputCloud(front_pc_);
    euclid.setClusterTolerance(kClusterTolerance);
    euclid.setMinClusterSize(MIN_CLUSTER_SIZE);
    euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
    euclid.setSearchMethod(tree);
    std::vector<pcl::PointIndices> local_indices;
    euclid.extract(local_indices);
    ROS_INFO("Detecting front ROI...");
    GetObjects(front_pc_, local_indices, obj_list, front_bbox_array);
  }
  // if (front_bbox_array.boxes.size() > 0) {
    front_bbox_pub_.publish(front_bbox_array);
  // }
  counter = 0;
}

void Segmenter::SegmentBack(std::vector<Detected_Obj> &obj_list) {
  // Segment back ROI.
  jsk_recognition_msgs::BoundingBoxArray back_bbox_array;
  back_bbox_array.header.frame_id = frame_id_;
  if (back_pc_ != nullptr && back_pc_->size() > 0) {
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(back_pc_);
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> euclid;
    euclid.setInputCloud(back_pc_);
    euclid.setClusterTolerance(kClusterTolerance);
    euclid.setMinClusterSize(MIN_CLUSTER_SIZE);
    euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
    euclid.setSearchMethod(tree);
    std::vector<pcl::PointIndices> local_indices;
    euclid.extract(local_indices);
    ROS_INFO("Detecting back ROI...");
    GetObjects(back_pc_, local_indices, obj_list, back_bbox_array);
  }
  // if (back_bbox_array.boxes.size() > 0) {
    back_bbox_pub_.publish(back_bbox_array);
  // }
  counter = 0;
}

void Segmenter::GetObjects(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc,
                           const std::vector<pcl::PointIndices> &local_indices,
                           std::vector<Detected_Obj> &obj_list,
                           jsk_recognition_msgs::BoundingBoxArray &bbox_array) {

  for (size_t i = 0; i < local_indices.size(); i++) {
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    float min_dis = std::numeric_limits<float>::max();

    Detected_Obj obj;
    // Find min, max, centroid points.
    for (auto pit = local_indices[i].indices.begin();
         pit != local_indices[i].indices.end(); ++pit) {
      pcl::PointXYZI p = in_pc->points[*pit];

      min_dis = std::min(min_dis, Point2Distance(p));
      obj.centroid.x += p.x;
      obj.centroid.y += p.y;
      obj.centroid.z += p.z;

      if (p.x < min_x)
        min_x = p.x;
      if (p.y < min_y)
        min_y = p.y;
      if (p.z < min_z)
        min_z = p.z;

      if (p.x > max_x)
        max_x = p.x;
      if (p.y > max_y)
        max_y = p.y;
      if (p.z > max_z)
        max_z = p.z;
    }

    obj.min_point = pcl::PointXYZ(min_x, min_y, min_z);
    obj.max_point = pcl::PointXYZ(max_x, max_y, max_z);
    obj.centroid.x /= local_indices[i].indices.size();
    obj.centroid.y /= local_indices[i].indices.size();
    obj.centroid.z /= local_indices[i].indices.size();

    double length_ = obj.max_point.x - obj.min_point.x;
    double width_ = obj.max_point.y - obj.min_point.y;
    double height_ = obj.max_point.z - obj.min_point.z;

    obj.bounding_box.header.frame_id = frame_id_;
    obj.bounding_box.pose.position.x = obj.min_point.x + length_ / 2;
    obj.bounding_box.pose.position.y = obj.min_point.y + width_ / 2;
    obj.bounding_box.pose.position.z = obj.min_point.z + height_ / 2;
    obj.bounding_box.pose.orientation.w = 1.0;
    if(obj.bounding_box.pose.position.x<0.3) continue;
    length_ = std::max(length_, 0.1);
    width_ = std::max(width_, 0.1);
    height_ = std::max(height_, 0.1);

    obj.bounding_box.dimensions.x = length_;
    obj.bounding_box.dimensions.y = width_;
    obj.bounding_box.dimensions.z = height_;

    for (int i = 1; i < dis_level_.size(); ++i) {
      if (min_dis < dis_level_[i]) {
        obj.area.area = static_cast<uint8_t>(i);
        break;
      }
    }
    obj.bounding_box.label = obj.area.area;
    obj.id = counter;

    // Point center_point;
    obj.center_point.x = obj.bounding_box.pose.position.x;
    obj.center_point.y = obj.bounding_box.pose.position.y;
    obj.center_point.z = obj.bounding_box.pose.position.z;
    obj.center_distance =
        sqrt(pow(obj.center_point.x, 2) + pow(obj.center_point.y, 2));

    bbox_array.boxes.emplace_back(obj.bounding_box);
    obj_list.push_back(obj);

    const auto &po = obj.bounding_box.pose.position;
    std::stringstream ss;
    ss << "bbox id: " << counter++ << " "
       << " cen: " << po.x << " " << po.y << " " << po.z << " dim: " << length_
       << " " << width_ << " " << height_ << " min_dis: " << min_dis
       << " area: " << (unsigned)obj.area.area << std::endl;
    ROS_INFO(ss.str().c_str());
  }
}

void Segmenter::BackCallback(const sensor_msgs::PointCloud2::Ptr &scan_msg) {
  const std::string info =
      "====== new back frame: " + std::to_string(scan_msg->header.seq) +
      " ======\n";
  ROS_INFO(info.c_str());
  GetBackROIPc(scan_msg);
  GetLastestHookPos();
  std::vector<Detected_Obj> global_obj_list;
  SegmentBack(global_obj_list);
  ROS_INFO("In behind ROI, total %d objects detected!", global_obj_list.size());
}

void Segmenter::GetBackROIPc(const sensor_msgs::PointCloud2::Ptr &scan_msg) {
  back_pc_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*scan_msg, *back_pc_);
  BackROIFilter(back_pc_);
    if (back_pc_->empty()) {
    ROS_WARN("After filter, back ROI pointcloud is empty!!");
    return;
  }
}

void Segmenter::HookPosCallback(
    const auto_couple::center_position::Ptr &hook_pos_msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  hook_pos_que_.push_back(hook_pos_msg);
}

void Segmenter::GetLastestHookPos() {
  std::lock_guard<std::mutex> lock(mutex_);
  // if (hook_pos_que_.empty()) {
  //   back_roi_ = back_roi_copy_;
  //   ROS_INFO("Get none hook position! Reset back ROI min_x: %f",
  //            back_roi_.min_x);
  //   return;
  // }

  // if ((hook_pos_que_.back()->center_point_x > back_roi_.max_x) ||
  //    (hook_pos_que_.back()->center_point_x < back_roi_.min_x)) {
  //   ROS_ERROR("Back ROI min_x: %f, max_x: %f, hook position x: %f, Will reset "
  //             "to default params!",
  //             back_roi_copy_.min_x, back_roi_copy_.max_x,
  //             hook_pos_que_.back()->center_point_x);
  //   back_roi_ = back_roi_copy_;
  // } else {
  //     back_roi_.max_x =
  //     std::min(back_roi_.max_x, hook_pos_que_.back()->center_point_x);
  // }

  // ROS_INFO("Set back ROI min_x: %f", back_roi_.min_x);
  // hook_pos_que_.clear();

  if (!hook_pos_que_.empty()) {
    hook_pos_latest_ = hook_pos_que_.back();
  }

  for (size_t i = hook_pos_que_.size(); i>4; --i){
      hook_pos_que_.pop_front();
  }
}

void Segmenter::GetFrontROIPc(
    const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
  // Extract sensor_msgs::PointCloud2 from  input LaserScan message.
  sensor_msgs::PointCloud2 cloud;
  tf::TransformListener tfListener;
  laser_geometry::LaserProjection projector;
  projector.transformLaserScanToPointCloud(frame_id_, *scan_msg, cloud,
                                           tfListener);

  // Convert sensor_msgs::PointCloud2 to pcl::PointCloud.
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(cloud, *cloud_raw);
  if (cloud_raw->empty()) {
    ROS_WARN("The front ROI input cloud is empty!!");
    return;
  }

  // Crop pointcloud by pcl::CropBox.
  front_pc_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  FrontROIFilter(cloud_raw, front_pc_);

  if (front_pc_->empty()) {
    ROS_WARN("After filter, front ROI pointcloud is empty!!");
    return;
  }
  // Publish input segmenter pointcloud.
  sensor_msgs::PointCloud2 output_points;
  pcl::toROSMsg(*front_pc_, output_points);
  front_roi_pc_pub_.publish(output_points);
}

void Segmenter::FrontCallback(
    const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
  const std::string info =
      "====== new front frame: " + std::to_string(scan_msg->header.seq) +
      " ======\n";
  ROS_INFO(info.c_str());

  GetFrontROIPc(scan_msg);
  std::vector<Detected_Obj> global_obj_list;
  SegmentFront(global_obj_list);
  ROS_INFO("In front ROI, total %d objects detected!", global_obj_list.size());
}
