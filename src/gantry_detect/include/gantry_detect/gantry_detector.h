#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ivlocmsg/ivmsglocpos.h>
#include <gantry_detect/gantry_state.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_listener.h>

#include <string>
#include <vector>

// 闸机口识别:
//   订阅两路后向雷达点云(PointCloud2)合并 -> 按强度过滤(高反) ->
//   欧氏聚类 -> 车(定位)进入设定区域后判定闸机口 开/闭 并发布。
//   同时发布合并点云/过滤点云/聚类框/状态标记 到 rviz 便于调试。
class GantryDetector
{
public:
  GantryDetector( ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  ~GantryDetector() = default;

private:
  // 聚类结果(车体系, 车体前方 x 正方向)
  struct ClusterInfo
  {
    float x = 0.f, y = 0.f;  // 质心
    int   points = 0;        // 点数
    float min_x = 0.f, max_x = 0.f;
    float min_y = 0.f, max_y = 0.f;
    bool  valid = false;
  };

  // ---------- 话题 ----------
  ros::Subscriber sub_localization_;
  ros::Subscriber sub_cloud_left_;
  ros::Subscriber sub_cloud_right_;
  ros::Publisher  pub_state_;       // gantry_detect/gantry_state
  ros::Publisher  pub_merged_;      // sensor_msgs/PointCloud2 合并原始点云
  ros::Publisher  pub_filtered_;    // sensor_msgs/PointCloud2 强度过滤后
  ros::Publisher  pub_markers_;     // visualization_msgs/MarkerArray 调试标记

  std::string localization_topic_;
  std::string left_cloud_topic_;
  std::string right_cloud_topic_;
  std::string state_topic_;
  std::string merged_cloud_topic_;
  std::string filtered_cloud_topic_;
  std::string markers_topic_;

  // ---------- 坐标 ----------
  std::string merged_frame_;   // 点云统一坐标系(rviz 中查看)
  std::string world_frame_;    // 定位/触发区域坐标系
  tf::TransformListener tf_listener_;
  // 左右雷达安装微调(平移米, yaw 度), 一般 tf 不完善时使用
  double left_ox_ = 0.0, left_oy_ = 0.0, left_oyaw_ = 0.0;
  double right_ox_ = 0.0, right_oy_ = 0.0, right_oyaw_ = 0.0;

  // ---------- 强度过滤 / 空间裁剪(车体系) ----------
  double intensity_min_ = 0.0;
  double intensity_max_ = 10000.0;
  bool   enable_crop_ = false;
  double crop_x_min_ = 0.0, crop_x_max_ = 20.0;
  double crop_y_min_ = -10.0, crop_y_max_ = 10.0;
  double crop_z_min_ = -10, crop_z_max_ = 10;

  // ---------- 聚类 ----------
  double cluster_tolerance_ = 0.1;
  int    min_cluster_pts_ = 3;
  int    max_cluster_pts_ = 20000;
  int    select_mode_ = 0;          // 0=距车最近 1=点数最多
  double target_dist_max_ = 0.0;    // 目标最大允许距离, 0=不限

  // ---------- 定位触发区域(世界系矩形) ----------
  bool enable_roi_ = true;
  double roi_x_min_ = 0.0, roi_x_max_ = 0.0;
  double roi_y_min_ = 0.0, roi_y_max_ = 0.0;
  double vehicle_x_ = 0.0, vehicle_y_ = 0.0, vehicle_vel_ = 0.0;
  bool   localization_valid_ = false;

  // ---------- 状态判定 ----------
  bool target_means_closed_ = true;  // true: 检测到高反目标=闸机口关闭(挡杆放下)
  int  confirm_frames_ = 3;          // 去抖: 连续 N 帧一致才切换状态
  bool last_raw_open_ = false, raw_set_ = false;
  int  stable_frames_ = 0;
  bool state_open_ = false;          // 当前稳定输出(有效)
  bool state_active_ = false;        // 已连续确认足够帧数, 状态有效

  // ---------- 轨迹可视化(csv: x,y 两列, rslidar 系, 单位米) ----------
  std::string trajectory_csv_;               // csv 路径, 空则不加载
  std::vector<pcl::PointXY> trajectory_;     // 静态轨迹点
  bool loadTrajectory();

  // ---------- 可视化开关 ----------
  bool publish_merged_cloud_ = true;
  bool publish_filtered_cloud_ = true;
  bool publish_markers_ = true;

  // ---------- 缓存 ----------
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_left_cache_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_;  // 本帧合并原始点云
  std::vector<ClusterInfo> clusters_;

  // ---------- 内部方法 ----------
  void localizationCallback(const ivlocmsg::ivmsglocpos::ConstPtr &msg);
  void leftCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void rightCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void runDetection();

  bool toXyzI(const sensor_msgs::PointCloud2::ConstPtr &msg,
              pcl::PointCloud<pcl::PointXYZI>::Ptr &out) const;
  // 转换到 merged_frame_ 并施加对应侧静态补偿
  bool moveToMergedFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, bool is_left) const;
  void cropAndFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const;
  void clusterize(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
  void pickTarget(ClusterInfo *target) const;
  void updateState(bool has_target);
  bool inTriggerArea() const;

  void publishState();
  void publishPointCloud(const ros::Publisher &pub,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const;
  void publishVisualization(const ClusterInfo *target, bool has_target, bool in_area);
  std::string stateText() const;
  void setMarkerCommon(visualization_msgs::Marker *m, int id, const std::string &ns,
                       const std::string &frame, int type) const;
};
