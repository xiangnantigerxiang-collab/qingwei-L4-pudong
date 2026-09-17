#include "gantry_detect/gantry_detector.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <Eigen/Geometry>

#include <cmath>
#include <cctype>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace
{
constexpr double kDeg2Rad = 0.017453292519943295;

inline double deg2rad(double d) { return d * kDeg2Rad; }
}  // namespace

GantryDetector::GantryDetector( ros::NodeHandle &nh, const ros::NodeHandle &pnh)
{
  // ---------- 话题 ----------
  pnh.param<std::string>("localization_topic", localization_topic_, "/localization");
  pnh.param<std::string>("left_cloud_topic", left_cloud_topic_, "pcd0");
  pnh.param<std::string>("right_cloud_topic", right_cloud_topic_, "pcd1");
  pnh.param<std::string>("state_topic", state_topic_, "/gantry_state");
  pnh.param<std::string>("merged_cloud_topic", merged_cloud_topic_, "/gantry_detect/cloud_merged");
  pnh.param<std::string>("filtered_cloud_topic", filtered_cloud_topic_, "/gantry_detect/cloud_filtered");
  pnh.param<std::string>("markers_topic", markers_topic_, "/gantry_detect/markers");

  // ---------- 坐标 ----------
  pnh.param<std::string>("merged_frame", merged_frame_, "robot");
  pnh.param<std::string>("world_frame", world_frame_, "map");
  pnh.param("left_offset_x", left_ox_, 0.0);
  pnh.param("left_offset_y", left_oy_, 0.0);
  pnh.param("left_offset_yaw", left_oyaw_, 0.0);
  pnh.param("right_offset_x", right_ox_, 0.0);
  pnh.param("right_offset_y", right_oy_, 0.0);
  pnh.param("right_offset_yaw", right_oyaw_, 0.0);

  // ---------- 强度过滤 / 裁剪 ----------
  pnh.param("intensity_min", intensity_min_, 500.0);
  pnh.param("intensity_max", intensity_max_, 10000.0);
  pnh.param("enable_crop", enable_crop_, false);
  pnh.param("crop_x_min", crop_x_min_, 0.0);
  pnh.param("crop_x_max", crop_x_max_, 10.0);
  pnh.param("crop_y_min", crop_y_min_, -3.0);
  pnh.param("crop_y_max", crop_y_max_, 3.0);
  pnh.param("crop_z_min", crop_z_min_, -1.5);
  pnh.param("crop_z_max", crop_z_max_, 1.5);

  // ---------- 聚类 ----------
  pnh.param("cluster_tolerance", cluster_tolerance_, 0.1);
  pnh.param("min_cluster_pts", min_cluster_pts_, 3);
  pnh.param("max_cluster_pts", max_cluster_pts_, 20000);
  pnh.param("select_mode", select_mode_, 0);
  pnh.param("target_dist_max", target_dist_max_, 0.0);

  // ---------- 定位触发区域 ----------
  pnh.param("enable_roi", enable_roi_, true);
  pnh.param("roi_x_min", roi_x_min_, -40.0);
  pnh.param("roi_x_max", roi_x_max_, -30.0);
  pnh.param("roi_y_min", roi_y_min_, -10.0);
  pnh.param("roi_y_max", roi_y_max_, 10.0);

  // ---------- 状态判定 ----------
  pnh.param("target_means_closed", target_means_closed_, true);
  pnh.param("confirm_frames", confirm_frames_, 3);

  // ---------- 可视化 ----------
  pnh.param("publish_merged_cloud", publish_merged_cloud_, true);
  pnh.param("publish_filtered_cloud", publish_filtered_cloud_, true);
  pnh.param("publish_markers", publish_markers_, true);

  // ---------- 轨迹(csv: x,y 两列, rslidar 系, 单位米) ----------
  pnh.param<std::string>("trajectory_csv", trajectory_csv_, "");
  loadTrajectory();

  sub_localization_ =
      nh.subscribe(localization_topic_, 10, &GantryDetector::localizationCallback, this);
  sub_cloud_left_ =
      nh.subscribe(left_cloud_topic_, 5, &GantryDetector::leftCloudCallback, this);
  sub_cloud_right_ =
      nh.subscribe(right_cloud_topic_, 5, &GantryDetector::rightCloudCallback, this);

  pub_state_ = nh.advertise<gantry_detect::gantry_state>(state_topic_, 10);
  pub_merged_ = nh.advertise<sensor_msgs::PointCloud2>(merged_cloud_topic_, 1);
  pub_filtered_ = nh.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic_, 1);
  pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>(markers_topic_, 1);

  ROS_INFO("[gantry] node started: left=%s right=%s merged_frame=%s",
           left_cloud_topic_.c_str(), right_cloud_topic_.c_str(), merged_frame_.c_str());
  ROS_INFO("[gantry] intensity [%.0f, %.0f] roi(%.1f~%.1f, %.1f~%.1f) "
           "target_means_closed=%d confirm=%d",
           intensity_min_, intensity_max_, roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_,
           target_means_closed_ ? 1 : 0, confirm_frames_);
}

// ============================================================================
// 回调
// ============================================================================

void GantryDetector::localizationCallback(const ivlocmsg::ivmsglocpos::ConstPtr &msg)
{
  if (!localization_valid_) {
    ROS_INFO("[gantry] localization received.");
  }
  vehicle_x_ = msg->xg;
  vehicle_y_ = msg->yg;
  vehicle_vel_ = msg->velocity;
  localization_valid_ = true;
}

void GantryDetector::leftCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (!toXyzI(msg, cloud)) return;
  if (!moveToMergedFrame(cloud, true)) return;
  cloud_left_cache_ = cloud;
}

void GantryDetector::rightCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr right(new pcl::PointCloud<pcl::PointXYZI>);
  if (!toXyzI(msg, right)) return;
  if (!moveToMergedFrame(right, false)) return;

  // 合并左右雷达(以右雷达本帧为基准, 左雷达取最近一帧缓存)
  pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>);
  merged->header = right->header;
  merged->points = right->points;
  if (cloud_left_cache_ && !cloud_left_cache_->points.empty()) {
    merged->points.insert(merged->points.end(), cloud_left_cache_->points.begin(),
                          cloud_left_cache_->points.end());
  }
  if (merged->points.empty()) return;
  merged->header.frame_id = merged_frame_;

  if (publish_merged_cloud_) publishPointCloud(pub_merged_, merged);

  cloud_in_ = merged;
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> avg;
avg.setInputCloud(cloud_in_);
avg.setLeafSize(0.1f, 0.1f, 0.1f);
avg.filter(*cloud_in_);
  runDetection();
}

// ============================================================================
// 点云处理
// ============================================================================

bool GantryDetector::loadTrajectory()
{
  trajectory_.clear();
  if (trajectory_csv_.empty()) return false;

  std::ifstream fin(trajectory_csv_);
  if (!fin.is_open()) {
    ROS_WARN("[gantry] trajectory csv open failed: %s", trajectory_csv_.c_str());
    return false;
  }

  std::string line;
  long line_no = 0;
  while (std::getline(fin, line)) {
    ++line_no;
    // 去除行首 UTF-8 BOM(常见于 Windows 保存的 csv)
    if (line_no == 1 && line.size() >= 3 &&
        static_cast<unsigned char>(line[0]) == 0xEF &&
        static_cast<unsigned char>(line[1]) == 0xBB &&
        static_cast<unsigned char>(line[2]) == 0xBF) {
      line = line.substr(3);
    }
    // 去除首尾空白与 Windows 行尾 '\r'
    const size_t b = line.find_first_not_of(" \t\r");
    const size_t e = line.find_last_not_of(" \t\r");
    if (b == std::string::npos) continue;
    line = line.substr(b, e - b + 1);

    const size_t comma = line.find(',');
    if (comma == std::string::npos) {
      ROS_WARN_ONCE("[gantry] trajectory csv line %ld has no comma, skipped: %s", line_no,
                    line.c_str());
      continue;
    }
    double x = 0.0, y = 0.0;
    bool ok = true;
    try {
      x = std::stod(line.substr(0, comma));
      y = std::stod(line.substr(comma + 1));
    } catch (const std::exception &) {
      ok = false;
    }
    if (!ok) {
      // 通常为首行表头(x,y), 忽略
      ROS_WARN_ONCE("[gantry] trajectory csv non-numeric row skipped: %s", line.c_str());
      continue;
    }
    pcl::PointXY p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    trajectory_.push_back(p);
  }

  ROS_INFO("[gantry] trajectory loaded %zu points from %s", trajectory_.size(),
           trajectory_csv_.c_str());
  return !trajectory_.empty();
}

bool GantryDetector::toXyzI(const sensor_msgs::PointCloud2::ConstPtr &msg,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr &out) const
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
  try {
    pcl::fromROSMsg(*msg, *tmp);
  } catch (const std::exception &e) {
    ROS_WARN_THROTTLE(2.0, "[gantry] convert PointCloud2 failed: %s", e.what());
    return false;
  }
  if (tmp->empty()) return false;
  out = tmp;
  return true;
}

bool GantryDetector::moveToMergedFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                       bool is_left) const
{
  // 1. tf 转换到统一坐标系
  if (cloud->header.frame_id != merged_frame_) {
    tf::StampedTransform tr;
    try {
      tf_listener_.lookupTransform(merged_frame_, cloud->header.frame_id, ros::Time(0), tr);
    } catch (const tf::TransformException &ex) {
      ROS_WARN_THROTTLE(2.0, "[gantry] no tf %s -> %s: %s", merged_frame_.c_str(),
                        cloud->header.frame_id.c_str(), ex.what());
      return false;
    }
    const tf::Vector3 &o = tr.getOrigin();
    const tf::Quaternion &q = tr.getRotation();
    Eigen::Affine3f T = Eigen::Translation3f(o.x(), o.y(), o.z()) *
                        Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z());
    pcl::transformPointCloud(*cloud, *cloud, T);
  }

  // 2. 静态安装补偿(常用在左右雷达 frame 相同、仅手动标定的场景)
  double ox = is_left ? left_ox_ : right_ox_;
  double oy = is_left ? left_oy_ : right_oy_;
  double oyaw = is_left ? left_oyaw_ : right_oyaw_;
  if (std::fabs(ox) > 1e-9 || std::fabs(oy) > 1e-9 || std::fabs(oyaw) > 1e-9) {
    Eigen::Affine3f T = Eigen::Translation3f(ox, oy, 0.0f) *
                        Eigen::AngleAxisf(deg2rad(oyaw), Eigen::Vector3f::UnitZ());
    pcl::transformPointCloud(*cloud, *cloud, T);
  }

  cloud->header.frame_id = merged_frame_;
  return true;
}

void GantryDetector::cropAndFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const
{
  if (enable_crop_ && !cloud->empty()) {
    pcl::CropBox<pcl::PointXYZI> box;
    box.setMin(Eigen::Vector4f(crop_x_min_, crop_y_min_, crop_z_min_, 1.0f));
    box.setMax(Eigen::Vector4f(crop_x_max_, crop_y_max_, crop_z_max_, 1.0f));
    box.setInputCloud(cloud);
    box.filter(*cloud);
  }
  if (!cloud->empty()) {
    pcl::PassThrough<pcl::PointXYZI> pt;
    pt.setInputCloud(cloud);
    pt.setFilterFieldName("intensity");
    pt.setFilterLimits(intensity_min_, intensity_max_);
    pt.filter(*cloud);
  }
}

void GantryDetector::clusterize(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
  clusters_.clear();
  if (!cloud || cloud->empty()) return;

  // 2D 欧氏聚类(z 置 0), 闸机口高反目标在水平扫描平面内
  pcl::PointCloud<pcl::PointXYZ>::Ptr xy(new pcl::PointCloud<pcl::PointXYZ>);
  xy->reserve(cloud->size());
  for (const auto &p : cloud->points) {
    pcl::PointXYZ q;
    q.x = p.x;
    q.y = p.y;
    q.z = 0.0f;
    xy->push_back(q);
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(xy);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_pts_);
  ec.setMaxClusterSize(max_cluster_pts_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(xy);
  ec.extract(cluster_indices);

  for (const auto &idx : cluster_indices) {
    ClusterInfo c;
    c.points = static_cast<int>(idx.indices.size());
    double sx = 0.0, sy = 0.0;
    c.min_x = c.max_x = xy->points[idx.indices[0]].x;
    c.min_y = c.max_y = xy->points[idx.indices[0]].y;
    for (int i : idx.indices) {
      const auto &p = xy->points[i];
      sx += p.x;
      sy += p.y;
      c.min_x = std::min(c.min_x, p.x);
      c.max_x = std::max(c.max_x, p.x);
      c.min_y = std::min(c.min_y, p.y);
      c.max_y = std::max(c.max_y, p.y);
    }
    c.x = static_cast<float>(sx / idx.indices.size());
    c.y = static_cast<float>(sy / idx.indices.size());
    c.valid = true;
    clusters_.push_back(c);
  }
}

void GantryDetector::pickTarget(ClusterInfo *target) const
{
  target->valid = false;
  if (clusters_.empty()) return;

  const ClusterInfo *best = nullptr;
  double best_score = 1e18;
  for (const auto &c : clusters_) {
    const double dist = std::hypot(c.x, c.y);
    if (target_dist_max_ > 0.0 && dist > target_dist_max_) continue;
    const double score = (select_mode_ == 1) ? -static_cast<double>(c.points) : dist;
    if (score < best_score) {
      best_score = score;
      best = &c;
    }
  }
  if (best) {
    *target = *best;
    target->valid = true;
  }
}

// ============================================================================
// 状态判定与发布
// ============================================================================

bool GantryDetector::inTriggerArea() const
{
  if (!localization_valid_) return false;
  if (!enable_roi_) return true;
  return vehicle_x_ >= roi_x_min_ && vehicle_x_ <= roi_x_max_ &&
         vehicle_y_ >= roi_y_min_ && vehicle_y_ <= roi_y_max_;
}

void GantryDetector::updateState(bool has_target)
{
  // 检测到目标与闸机口开/闭的对应关系由参数决定:
  //   target_means_closed=true  : 有高反目标(闸杆放下) -> 闭
  const bool raw_open = target_means_closed_ ? !has_target : has_target;

  if (!raw_set_ || raw_open == last_raw_open_) {
    ++stable_frames_;
  } else {
    stable_frames_ = 1;
  }
  last_raw_open_ = raw_open;
  raw_set_ = true;

  if (stable_frames_ >= confirm_frames_) {
    state_active_ = true;
    if (state_open_ != raw_open) {
      state_open_ = raw_open;
      ROS_INFO("[gantry] state -> %s", state_open_ ? "GATE_OPEN" : "GATE_CLOSED");
    }
  }
}

void GantryDetector::publishState()
{
  gantry_detect::gantry_state msg;
  msg.active = state_active_;
  msg.gantry_open = state_open_;
  pub_state_.publish(msg);
}

// ============================================================================
// 主处理链
// ============================================================================

void GantryDetector::runDetection()
{
  const bool in_area = inTriggerArea();
  if (!in_area) {
    // 车不在触发区域, 不检测
    if (state_active_) {
      ROS_INFO("[gantry] vehicle out of trigger area, stop detect");
    }
    last_raw_open_ = false;
    raw_set_ = false;
    stable_frames_ = 0;
    state_active_ = false;
    state_open_ = false;
    publishState();
    publishVisualization(nullptr, false, false);
    return;
  }

  // 复制本帧合并点云 -> 裁剪 + 强度过滤
  pcl::PointCloud<pcl::PointXYZI>::Ptr work(new pcl::PointCloud<pcl::PointXYZI>);
  *work = *cloud_in_;
  cropAndFilter(work);

  if (publish_filtered_cloud_) publishPointCloud(pub_filtered_, work);

  clusterize(work);

  ClusterInfo target;
  pickTarget(&target);

  updateState(target.valid);
  publishState();
  publishVisualization(&target, target.valid, true);
}

// ============================================================================
// 可视化
// ============================================================================

void GantryDetector::publishPointCloud(
    const ros::Publisher &pub, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const
{
  if (pub.getNumSubscribers() == 0) return;
  sensor_msgs::PointCloud2 out;
  pcl::toROSMsg(*cloud, out);
  pub.publish(out);
}

std::string GantryDetector::stateText() const
{
  if (!localization_valid_) return "WAIT_LOCATION";
  if (enable_roi_ && !inTriggerArea()) return "OUT_OF_ROI";
  if (!state_active_) return "SEARCHING";
  return state_open_ ? "GATE_OPEN" : "GATE_CLOSED";
}

void GantryDetector::setMarkerCommon(visualization_msgs::Marker *m, int id,
                                     const std::string &ns, const std::string &frame,
                                     int type) const
{
  m->header.frame_id = frame;
  m->header.stamp = ros::Time::now();
  m->ns = ns;
  m->id = id;
  m->type = type;
  m->action = visualization_msgs::Marker::ADD;
  m->lifetime = ros::Duration(0);
  m->pose.orientation.w = 1.0;
  m->pose.position.z = 0.0;
  m->scale.x = 1.0;
  m->scale.y = 1.0;
  m->scale.z = 1.0;
  m->color.r = 1.0f;
  m->color.g = 1.0f;
  m->color.b = 1.0f;
  m->color.a = 1.0f;
}

void GantryDetector::publishVisualization(const ClusterInfo *target, bool has_target,
                                          bool in_area)
{
  if (!publish_markers_) return;

  visualization_msgs::MarkerArray ma;

  // 清空旧标记
  visualization_msgs::Marker del;
  del.header.stamp = ros::Time::now();
  del.action = visualization_msgs::Marker::DELETEALL;
  ma.markers.push_back(del);

  int id = 0;

  // 0) 空间裁剪区域(enable_crop 开启时在 rviz 显示, 车体系)
  if (enable_crop_) {
    const double cx0 = crop_x_min_, cx1 = crop_x_max_;
    const double cy0 = crop_y_min_, cy1 = crop_y_max_;
    const double cz0 = crop_z_min_, cz1 = crop_z_max_;

    // 半透明盒体: 便于观察整体空间范围
    visualization_msgs::Marker box;
    setMarkerCommon(&box, ++id, "crop", merged_frame_, visualization_msgs::Marker::CUBE);
    box.pose.position.x = (cx0 + cx1) / 2.0;
    box.pose.position.y = (cy0 + cy1) / 2.0;
    box.pose.position.z = (cz0 + cz1) / 2.0;
    box.scale.x = cx1 - cx0;
    box.scale.y = cy1 - cy0;
    box.scale.z = cz1 - cz0;
    box.color.r = 0.0f;
    box.color.g = 1.0f;
    box.color.b = 0.6f;
    box.color.a = 0.06f;
    ma.markers.push_back(box);

    // 线框: 12 条棱, 边界更清晰
    visualization_msgs::Marker wire;
    setMarkerCommon(&wire, ++id, "crop", merged_frame_,
                    visualization_msgs::Marker::LINE_LIST);
    wire.scale.x = 0.08;
    wire.color.r = 0.0f;
    wire.color.g = 1.0f;
    wire.color.b = 0.6f;
    wire.color.a = 0.9f;
    const auto mk = [](double x, double y, double z) {
      geometry_msgs::Point p;
      p.x = x;
      p.y = y;
      p.z = z;
      return p;
    };
    const geometry_msgs::Point verts[8] = {
        mk(cx0, cy0, cz0), mk(cx1, cy0, cz0), mk(cx1, cy1, cz0), mk(cx0, cy1, cz0),
        mk(cx0, cy0, cz1), mk(cx1, cy0, cz1), mk(cx1, cy1, cz1), mk(cx0, cy1, cz1),
    };
    static const int kEdge[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},  // 底面
        {4, 5}, {5, 6}, {6, 7}, {7, 4},  // 顶面
        {0, 4}, {1, 5}, {2, 6}, {3, 7},  // 立柱
    };
    for (const auto &e : kEdge) {
      wire.points.push_back(verts[e[0]]);
      wire.points.push_back(verts[e[1]]);
    }
    ma.markers.push_back(wire);
  }

  // 1) 聚类框
  for (const auto &c : clusters_) {
    visualization_msgs::Marker m;
    setMarkerCommon(&m, ++id, "cluster", merged_frame_, visualization_msgs::Marker::CUBE);
    m.pose.position.x = (c.min_x + c.max_x) / 2.0f;
    m.pose.position.y = (c.min_y + c.max_y) / 2.0f;
    m.scale.x = c.max_x - c.min_x + 0.1f;
    m.scale.y = c.max_y - c.min_y + 0.1f;
    m.scale.z = 0.05f;
    m.color.r = 1.0f;
    m.color.g = 0.6f;
    m.color.b = 0.0f;
    m.color.a = 0.6f;
    ma.markers.push_back(m);
  }

  // 2) 选中目标: 球 + 原点连线
  if (has_target && target) {
    visualization_msgs::Marker sphere;
    setMarkerCommon(&sphere, ++id, "target", merged_frame_,
                    visualization_msgs::Marker::SPHERE);
    sphere.pose.position.x = target->x;
    sphere.pose.position.y = target->y;
    sphere.scale.x = 0.4f;
    sphere.scale.y = 0.4f;
    sphere.scale.z = 0.4f;
    sphere.color.g = 1.0f;
    sphere.color.r = 0.0f;
    ma.markers.push_back(sphere);

    visualization_msgs::Marker line;
    setMarkerCommon(&line, ++id, "target", merged_frame_,
                    visualization_msgs::Marker::LINE_STRIP);
    line.scale.x = 0.05f;
    line.color.r = 1.0f;
    line.color.g = 1.0f;
    line.color.b = 0.0f;
    geometry_msgs::Point p0, p1;
    p1.x = target->x;
    p1.y = target->y;
    line.points.push_back(p0);
    line.points.push_back(p1);
    ma.markers.push_back(line);
  }

  // 3) 状态文字(车体系上方, 车前方 x 正方向)
  {
    visualization_msgs::Marker txt;
    setMarkerCommon(&txt, ++id, "info", merged_frame_,
                    visualization_msgs::Marker::TEXT_VIEW_FACING);
    txt.pose.position.z = 1.8;
    txt.scale.z = 0.6;
    const std::string text = stateText();
    txt.text = text;
    if (text == "GATE_OPEN") {
      txt.color.g = 1.0f;
      txt.color.r = 0.0f;
    } else if (text == "GATE_CLOSED") {
      txt.color.r = 1.0f;
      txt.color.g = 0.0f;
    } else {
      txt.color.r = 1.0f;
      txt.color.g = 1.0f;
      txt.color.b = 0.0f;
    }
    ma.markers.push_back(txt);
  }

  // 4) 触发区域 + 车辆位置(世界系)
  if (enable_roi_ && localization_valid_) {
    visualization_msgs::Marker roi;
    setMarkerCommon(&roi, ++id, "roi", world_frame_, visualization_msgs::Marker::LINE_STRIP);
    roi.scale.x = 0.15f;
    roi.color.b = 1.0f;
    roi.color.a = 0.8f;
    geometry_msgs::Point p[5];
    p[0].x = roi_x_min_; p[0].y = roi_y_min_;
    p[1].x = roi_x_max_; p[1].y = roi_y_min_;
    p[2].x = roi_x_max_; p[2].y = roi_y_max_;
    p[3].x = roi_x_min_; p[3].y = roi_y_max_;
    p[4] = p[0];
    for (const auto &pt : p) roi.points.push_back(pt);
    ma.markers.push_back(roi);

    visualization_msgs::Marker veh;
    setMarkerCommon(&veh, ++id, "roi", world_frame_, visualization_msgs::Marker::SPHERE);
    veh.pose.position.x = vehicle_x_;
    veh.pose.position.y = vehicle_y_;
    veh.scale.x = 0.6f;
    veh.scale.y = 0.6f;
    veh.scale.z = 0.6f;
    if (in_area) {
      veh.color.g = 1.0f;
    } else {
      veh.color.r = 1.0f;
      veh.color.g = 1.0f;
      veh.color.b = 1.0f;
    }
    ma.markers.push_back(veh);
  }

  // 5) 轨迹(静态折线, rslidar 系; z 略微抬高避免与地面网格重叠)
  if (!trajectory_.empty()) {
    visualization_msgs::Marker traj;
    setMarkerCommon(&traj, ++id, "trajectory", merged_frame_,
                    visualization_msgs::Marker::LINE_STRIP);
    traj.scale.x = 0.12;
    traj.color.b = 1.0f;
    traj.color.a = 0.85f;
    geometry_msgs::Point p;
    p.z = 0.05;
    for (const auto &tp : trajectory_) {
      p.x = tp.x;
      p.y = tp.y;
      traj.points.push_back(p);
    }
    ma.markers.push_back(traj);
  }

  pub_markers_.publish(ma);
}
