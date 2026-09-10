/**

 * centerpoint_ros.cc  —  Optimized CenterPoint ROS wrapper

 *

 * Changes vs. original:

 *   1. [Config]  All tunables live in the "PARAMETERS" section below.

 *   2. [Pre-proc] EgoVehicleFilter() removes points inside configurable AABB

 *                 exclusion zones before they reach the inference engine.

 *   3. [Post-proc] PubDetectedMarker() now also emits a TEXT_VIEW_FACING

 *                  Marker that shows "<class> <score>" next to every box.

 */



#include <cmath>
#include <tf/transform_datatypes.h>
#include "centerpoint_ros.h"
#include "cuda_runtime.h"



#include <stdio.h>

#include <iostream>

#include <fstream>

#include <string>



// ============================================================

//  PARAMETERS — edit only this section to tune the detector

// ============================================================

namespace kParams {

  static const float kFrontXMin = 0.0f;
  static const float kFrontXMax = 20.0f;
  static const float kFrontY    = 3.0f;
  static const float kMinZ      = 0.6f;

  static const int   kMinPoints = 200;   // 防噪声

  // ── Model ──────────────────────────────────────────────────

  static const std::string kModelFile   = "../model/rpn_centerhead_sim.plan";
  static const bool        kVerbose     = false;


  // ── Point-cloud capacities ─────────────────────────────────

  static const int kMaxPointsPerLidar   = 70000;   // per-sensor ring buffer
  static const int kMaxAllPoints        = 280000;  // merged cloud hard cap

  // ── Lidar 健康检查配置 ─────────────────────────────────────
  //    检查每个 lidar 的点云数量是否足够
  //    如果某个 lidar 缺失或点云过少，则跳过发布 /box

  static const int kMinPointsPerLidar   = 100;     // 单个 lidar 最少点数 (低于此认为异常)
  static const int kMinTotalPoints      = 500;     // 合并后总点数 (低于此认为数据不足)

  // ── Ego-vehicle exclusion zones (axis-aligned boxes in sensor frame)
  //    Each entry: { x_min, x_max, y_min, y_max, z_min, z_max }
  //    Points whose (x,y,z) fall inside ANY zone are dropped.

  struct AABB { float x_min, x_max, y_min, y_max, z_min, z_max; };

  static const std::vector<AABB> kEgoZones = {

    // main vehicle body (front bumper ~ rear bumper)
    { -3.9f,  2.27f, -0.75f, 0.75f, -2.0f, 2.5f },
    // optional extra zone, e.g. roof sensor blind-spot – uncomment to use:
    { 0.5f,  1.0f, -2.0f, 2.0f,  -1.5f, 3.0f },
     { 2.5f,  3.5f, -4.0f, 1.0f,  -1.5f, 3.0f },

  };



  // ── Per-class score thresholds & colours  ──────────────────
  //    index matches NuScenes order:
  //    0=car  1=truck  2=construction_vehicle  3=bus  4=trailer
  //    5=barrier  6=motorcycle  7=bicycle  8=pedestrian  9=traffic_cone
  //
  // 配置格式: { x范围, y范围, 分数阈值, 颜色R, G, B }
  //   - 第一个区域: 默认阈值 (x_min=-1表示全范围)
  //   - 后续区域: 特定区域阈值
  //   - 阈值>0.99表示禁用该类别

  struct ClassZone {
    float x_min, x_max;    // x范围 (米), x_min=-1表示全范围
    float y_min, y_max;    // y范围 (米)
    float score_thresh;    // 分数阈值
  };

  struct ClassCfg {
    const char* label;
    float r, g, b;                    // marker RGB
    ClassZone zones[4];               // 区域配置 (首元素为默认)
  };

  // 宏: 简化区域配置书写
  // Z(x1,x2,y1,y2,th) = {x1,x2,y1,y2,th}
  #define Z(x1,x2,y1,y2,th) { x1, x2, y1, y2, th }

  static const ClassCfg kClassCfg[] = {

    /* 0 car               */ { "Car", 1.0f, 0.0f, 0.0f,
      { Z(-1, 80, -15, 15, 0.45f),      // 默认: 全范围0.45
        Z(0, 10, -15, 15, 0.30f),       // 近距离: 降低阈值减少漏检
        Z(30, 80, -15, 15, 0.60f) } },  // 远距离: 提高阈值减少误检

    /* 1 truck             */ { "Truck", 1.0f, 0.0f, 0.0f,
      { Z(-1, 80, -15, 15, 0.45f),
        Z(0, 15, -15, 15, 0.35f),
        Z(15, 80, -15, 15, 0.55f) } },

    /* 2 construction_veh  */ { "ConstrVeh", 1.0f, 0.0f, 0.0f,
      { Z(-1, 80, -15, 15, 0.65f), {} } },

    /* 3 bus               */ { "Bus", 1.0f, 0.0f, 0.0f,
      { Z(-1, 80, -15, 15, 0.88f), {} } },

    /* 4 trailer           */ { "Trailer", 1.0f, 0.0f, 0.0f,
      { Z(-1, 80, -15, 15, 0.80f),
        Z(0, 20, -15, 15, 0.85f),
        Z(20, 80, -15, 15, 0.85f) } },

    /* 5 barrier           */ { "Barrier", 0.5f, 0.5f, 0.5f,
      { Z(-1, 80, -15, 15, 0.99f), {} } },  // 禁用

    /* 6 motorcycle        */ { "Motorcycle", 0.0f, 0.0f, 1.0f,
      { Z(-1, 80, -15, 15, 0.60f),
        Z(0, 15, -15, 15, 0.50f),
        Z(15, 50, -15, 15, 0.70f) } },

    /* 7 bicycle           */ { "Bicycle", 0.0f, 0.0f, 1.0f,
      { Z(-1, 80, -15, 15, 0.60f),
        Z(0, 15, -15, 15, 0.50f),
        Z(15, 50, -15, 15, 0.70f) } },

    /* 8 pedestrian        */ { "Pedestrian", 0.0f, 1.0f, 0.0f,
      { Z(-1, 80, -15, 15, 0.30f),
        Z(0, 8, -15, 15, 0.20f),        // 近距离: 很低阈值
        Z(25, 50, -15, 15, 0.45f) } },  // 远距离: 提高

    /* 9 traffic_cone      */ { "Cone", 0.0f, 0.0f, 1.0f,
      { Z(-1, 80, -15, 15, 0.34f),
        Z(0, 10, -15, 15, 0.12f),
        Z(10, 20, -15, 15, 0.43f),
        Z(20, 40, -15, 15, 0.53f) } },

  };

  static const int kNumClasses = 10;

  // ── Helper: 根据检测框位置获取对应阈值 ─────────────────────
  static inline float GetThresholdForBox(int class_id, float x, float y) {
    if (class_id < 0 || class_id >= kNumClasses) return 0.5f;
    const ClassCfg& cfg = kClassCfg[class_id];
    // 遍历配置的区域 (跳过第0个默认区域)
    for (int i = 1; i < 4; ++i) {
      const ClassZone& z = cfg.zones[i];
      if (z.x_min < 0) continue;  // 跳过未配置的
      if (x >= z.x_min && x <= z.x_max && y >= z.y_min && y <= z.y_max) {
        return z.score_thresh;
      }
    }
    return cfg.zones[0].score_thresh;  // 返回默认阈值
  }



  // ── Text label (post-proc) ─────────────────────────────────

  static const float kTextOffsetZ    =  0.8f;  // metres above box centre
  static const float kTextScale      =  0.4f;  // Marker text height (m)
  static const float kTextColorR     =  1.0f;
  static const float kTextColorG     =  1.0f;
  static const float kTextColorB     =  1.0f;
  static const float kTextColorA     =  1.0f;
  static const float kMarkerLifetime =  0.15f; // seconds (0 = infinite)



  // ── ROS topics ─────────────────────────────────────────────

  static const std::string kTopicLidarLeft   = "rslidar_points_left";
  static const std::string kTopicLidarRight  = "rslidar_points_right";
  static const std::string kTopicLidarMid   = "rslidar_points_mid";
  static const std::string kTopicLidarFront  = "rslidar_points_front";

  static const std::string kTopicObjects     = "box";

  static const std::string kFrameId         = "rslidar";



} // namespace kParams

// ============================================================

//  END OF PARAMETERS

// ============================================================



#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>

struct Cell {
  int ix, iy;
  bool operator==(const Cell& o) const { return ix == o.ix && iy == o.iy; }
};

struct CellHash {
  size_t operator()(const Cell& c) const {
    return (uint64_t(c.ix) * 73856093ull) ^ (uint64_t(c.iy) * 19349663ull);
  }
};

bool ExtractFrontObstacleBoxes(const std::vector<float>& pts,
                               std::vector<Bndbox>& out_boxes)
{
  const int stride = 5;
  const int n = pts.size() / stride;

  // ===== 参数 =====
  const float GRID = 0.2f;
  const float X_MIN = 0.0f, X_MAX = 20.0f;
  const float Y_LIM = 10.0f;
  const float Z_MIN = 0.20f;

  const int   MIN_CELL = 6;
  const int   MIN_POINTS = 80;

  const float MIN_DX = 0.5f, MAX_DX = 4.5f;
  const float MIN_DY = 2.5f, MAX_DY = 4.5f;
  const float MIN_DZ = 0.1f, MAX_DZ = 6.5f;

  //const float MIN_DENSITY = 25.0f;
  //const float MAX_VAR_Z   = 0.25f;

  // ===== 1. grid =====
  std::unordered_map<Cell, int, CellHash> grid;
  grid.reserve(2048);

  for (int i = 0; i < n; ++i) {
    float x = pts[i*stride+0];
    float y = pts[i*stride+1];
    float z = pts[i*stride+2];

    if (x < X_MIN || x > X_MAX) continue;
    if (fabs(y) > Y_LIM) continue;
    if (z < Z_MIN) continue;

    int ix = int(x / GRID);
    int iy = int((y + Y_LIM) / GRID);

    grid[{ix, iy}]++;
  }

  if (grid.empty()) return false;

  // ===== 2. clustering =====
  std::unordered_map<Cell, int, CellHash> visited;
  visited.reserve(grid.size());

  const int dx[4] = {1,-1,0,0};
  const int dy[4] = {0,0,1,-1};

  struct Cluster {
    std::vector<Cell> cells;
  };

  std::vector<Cluster> clusters;
  clusters.reserve(32);

  for (auto& kv : grid) {
    const Cell& start = kv.first;
    if (visited[start]) continue;

    Cluster cluster;
    std::queue<Cell> q;

    q.push(start);
    visited[start] = 1;

    while (!q.empty()) {
      Cell c = q.front(); q.pop();
      cluster.cells.push_back(c);

      for (int k = 0; k < 4; ++k) {
        Cell nb{c.ix + dx[k], c.iy + dy[k]};
        if (grid.find(nb) == grid.end()) continue;
        if (visited[nb]) continue;

        visited[nb] = 1;
        q.push(nb);
      }
    }

    if ((int)cluster.cells.size() >= MIN_CELL)
      clusters.push_back(std::move(cluster));
  }

  if (clusters.empty()) return false;

  // ===== 3. cell → cluster =====
  std::unordered_map<Cell, int, CellHash> cell2cluster;
  cell2cluster.reserve(grid.size());

  for (int cid = 0; cid < (int)clusters.size(); ++cid) {
    for (const auto& c : clusters[cid].cells) {
      cell2cluster[c] = cid;
    }
  }

  // ===== 4. stats =====
  struct Stat {
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    int count;
    float sum_z, sum_z2;
  };

  std::vector<Stat> stats(clusters.size());

  for (auto& s : stats) {
    s.min_x = s.min_y = s.min_z =  std::numeric_limits<float>::max();
    s.max_x = s.max_y = s.max_z = -std::numeric_limits<float>::max();
    s.count = 0;
    s.sum_z = 0;
    s.sum_z2 = 0;
  }

  for (int i = 0; i < n; ++i) {
    float x = pts[i*stride+0];
    float y = pts[i*stride+1];
    float z = pts[i*stride+2];

    if (x < X_MIN || x > X_MAX) continue;
    if (fabs(y) > Y_LIM) continue;
    if (z < Z_MIN) continue;

    int ix = int(x / GRID);
    int iy = int((y + Y_LIM) / GRID);

    Cell c{ix, iy};

    auto it = cell2cluster.find(c);
    if (it == cell2cluster.end()) continue;

    Stat& s = stats[it->second];

    s.count++;
    s.sum_z += z;
    s.sum_z2 += z*z;

    s.min_x = std::min(s.min_x, x);
    s.min_y = std::min(s.min_y, y);
    s.min_z = std::min(s.min_z, z);

    s.max_x = std::max(s.max_x, x);
    s.max_y = std::max(s.max_y, y);
    s.max_z = std::max(s.max_z, z);
  }

  // ===== 5. 输出全部候选 =====
  out_boxes.clear();
  out_boxes.reserve(stats.size());

  for (int i = 0; i < (int)stats.size(); ++i) {
    const Stat& s = stats[i];

    if (s.count < MIN_POINTS) continue;

    float dx_ = s.max_x - s.min_x;
    float dy_ = s.max_y - s.min_y;
    float dz_ = s.max_z - s.min_z;

    if (dx_ < MIN_DX || dx_ > MAX_DX) continue;
    if (dy_ < MIN_DY || dy_ > MAX_DY) continue;
    //if (dz_ < MIN_DZ || dz_ > MAX_DZ) continue;

    float volume = dx_ * dy_ * dz_;
    float density = s.count / (volume + 1e-3f);
    //if (density < MIN_DENSITY) continue;

    float mean_z = s.sum_z / s.count;
    float var_z = s.sum_z2 / s.count - mean_z * mean_z;
    //if (var_z > MAX_VAR_Z) continue;

    Bndbox box;
    box.x = (s.min_x + s.max_x) * 0.5f;
    box.y = (s.min_y + s.max_y) * 0.5f;
    box.z = (s.min_z + s.max_z) * 0.5f;

    box.w = dx_;
    box.l = dy_;
    box.h = dz_;

    box.rt = 0.0f;
    box.score = 1.0f;
    box.id = 2;

    // ===== 极简去重 =====
    bool overlap = false;
    for (const auto& e : out_boxes) {
      if (fabs(e.x - box.x) < 1.0f &&
          fabs(e.y - box.y) < 1.0f) {
        overlap = true;
        break;
      }
    }

    if (!overlap)
      out_boxes.push_back(box);
  }

  return !out_boxes.empty();
}

// ── Helper: is point (x,y,z) inside any ego exclusion zone? ─

static inline bool IsEgoPoint(float x, float y, float z)

{

  for (const auto& zone : kParams::kEgoZones) {

    if (x >= zone.x_min && x <= zone.x_max &&

        y >= zone.y_min && y <= zone.y_max &&

        z >= zone.z_min && z <= zone.z_max) {

      return true;

    }

  }

  return false;

}



// ── Helper: char buffer for "ClassName 0.87" ────────────────

static std::string MakeLabelText(int class_id, float score)

{

  char buf[64];

  const char* label = (class_id >= 0 && class_id < kParams::kNumClasses)

                      ? kParams::kClassCfg[class_id].label

                      : "Unknown";

  snprintf(buf, sizeof(buf), "%s %.2f", label, score);

  return std::string(buf);

}





// ─────────────────────────────────────────────────────────────

//  Constructor / Destructor

// ─────────────────────────────────────────────────────────────

CenterPointRos::CenterPointRos()

  : max_points_(kParams::kMaxPointsPerLidar),

    max_all_points_(kParams::kMaxAllPoints)

{

  stream_ = nullptr;

  checkCudaErrors(cudaStreamCreate(&stream_));



  centerpoint_ptr_.reset(new CenterPoint(kParams::kModelFile, kParams::kVerbose));

  centerpoint_ptr_->prepare();



  host_points_l_.resize(max_points_ * 5);

  host_points_r_.resize(max_points_ * 5);

  host_points_bp_l_.resize(max_points_ * 5);

  host_points_bp_r_.resize(max_points_ * 5);

  host_points_.reserve(max_all_points_ * 5);



  checkCudaErrors(cudaMalloc(

      reinterpret_cast<void**>(&dev_points_),

      max_all_points_ * 5 * sizeof(float)));

}



CenterPointRos::~CenterPointRos()
{
  checkCudaErrors(cudaFree(dev_points_));
  checkCudaErrors(cudaStreamDestroy(stream_));
}

// ─────────────────────────────────────────────────────────────
//  Publisher 状态管理: 当 lidar 异常时 shutdown publisher
// ─────────────────────────────────────────────────────────────

void CenterPointRos::ShutdownPublisher()
{
  if (publisher_valid_) {
    pub_objects_.shutdown();
    publisher_valid_ = false;
    printf("[CenterPoint]  已关闭 /box publisher，下游无法接收\n");
  }
}

void CenterPointRos::RestorePublisher()
{
  if (!publisher_valid_) {
    pub_objects_ = nh_.advertise<visualization_msgs::MarkerArray>(
                      kParams::kTopicObjects, 1);
    publisher_valid_ = true;
    printf("[CenterPoint]  已恢复 /box publisher\n");
  }
}





// ─────────────────────────────────────────────────────────────

//  ROS pub/sub

// ─────────────────────────────────────────────────────────────

void CenterPointRos::CreateRosPubSub()

{

  sub_points_l_ = nh_.subscribe(kParams::kTopicLidarLeft,  1,
                                &CenterPointRos::PointsCallbackL, this);
  sub_points_r_ = nh_.subscribe(kParams::kTopicLidarRight, 1,
                                &CenterPointRos::PointsCallbackR, this);

  sub_points_m_ = nh_.subscribe(kParams::kTopicLidarMid, 1,
                                &CenterPointRos::PointsCallbackM, this);
  sub_points_f_ = nh_.subscribe(kParams::kTopicLidarFront, 1,
                                &CenterPointRos::PointsCallbackF, this);



  pub_objects_ = nh_.advertise<visualization_msgs::MarkerArray>(
                    kParams::kTopicObjects, 1);

}





// ─────────────────────────────────────────────────────────────

//  PRE-PROCESSING: ego-vehicle point-cloud filter

//  Input  : raw flat float buffer [x,y,z,i,r, x,y,z,i,r, ...]  (stride=5)

//  Output : filtered flat buffer  (points inside kEgoZones removed)

// ─────────────────────────────────────────────────────────────

void CenterPointRos::EgoVehicleFilter(std::vector<float>& points)

{

  const int stride = 5;

  int num_in  = static_cast<int>(points.size()) / stride;

  int write   = 0;



  for (int i = 0; i < num_in; ++i) {

    float x = points[i * stride + 0];

    float y = points[i * stride + 1];

    float z = points[i * stride + 2];



    if (IsEgoPoint(x, y, z)) continue;   // drop ego points



    if (write != i) {

      // compact in-place: copy only if write head lags behind read head

      std::copy(points.begin() + i    * stride,

                points.begin() + (i+1)* stride,

                points.begin() + write* stride);

    }

    ++write;

  }



  points.resize(write * stride);

}





// ─────────────────────────────────────────────────────────────

//  Merge + infer

// ─────────────────────────────────────────────────────────────

void CenterPointRos::MergePointClouds()

{

  host_points_.clear();



  // Guard: skip if either main lidar buffer is still at default (uninitialised)

  if (host_points_l_.size() == static_cast<size_t>(max_points_ * 5) ||

      host_points_r_.size() == static_cast<size_t>(max_points_ * 5)) return;



  // ── Lidar 健康检查 ─────────────────────────────────────────
  //    检查每个 lidar 的点云数量

  int pts_l = static_cast<int>(host_points_l_.size()) / 5;
  int pts_r = static_cast<int>(host_points_r_.size()) / 5;
  int pts_m = static_cast<int>(host_points_m_.size()) / 5;
  int pts_f = static_cast<int>(host_points_f_.size()) / 5;

  bool lidar_ok = true;
  std::string lidar_issue;

  if (pts_l < kParams::kMinPointsPerLidar) {
    lidar_ok = false;
    lidar_issue = "Left lidar 点数不足: " + std::to_string(pts_l);
  }
  if (pts_r < kParams::kMinPointsPerLidar) {
    lidar_ok = false;
    lidar_issue = "Right lidar 点数不足: " + std::to_string(pts_r);
  }
  if (pts_m < kParams::kMinPointsPerLidar) {
    lidar_ok = false;
    lidar_issue = "Mid lidar 点数不足: " + std::to_string(pts_m);
  }
  if (pts_f < kParams::kMinPointsPerLidar) {
    lidar_ok = false;
    lidar_issue = "Front lidar 点数不足: " + std::to_string(pts_f);
  }

  // ── Lidar 异常时发送默认 box ─────────────────────────────
  //    当检测到 lidar 异常时，创建一个默认 car box (score=-1) 发送给下游
  //    而不是关闭 publisher

  bool send_default_box = false;
  std::string default_box_reason;

  if (!lidar_ok) {
    send_default_box = true;
    default_box_reason = lidar_issue;
    printf("[CenterPoint]  Lidar健康检查失败: %s, 发送默认box\n", lidar_issue.c_str());
  }

  host_points_.insert(host_points_.end(),
                      host_points_l_.begin(), host_points_l_.end());

  host_points_.insert(host_points_.end(),
                      host_points_r_.begin(), host_points_r_.end());

  host_points_.insert(host_points_.end(),
                      host_points_m_.begin(), host_points_m_.end());

  host_points_.insert(host_points_.end(),
                      host_points_f_.begin(), host_points_f_.end());

  // ── 总点数检查 ─────────────────────────────────────────────

  int total_pts = static_cast<int>(host_points_.size()) / 5;
  if (total_pts < kParams::kMinTotalPoints) {
    send_default_box = true;
    default_box_reason = "总点数不足: " + std::to_string(total_pts) + " < " + std::to_string(kParams::kMinTotalPoints);
    printf("[CenterPoint] 总点数不足: %d < %d, 发送默认box\n", 
           total_pts, kParams::kMinTotalPoints);
  }

  // Uncomment when back-pair lidars are active:

  // host_points_.insert(host_points_.end(),

  //                     host_points_bp_l_.begin(), host_points_bp_l_.end());

  // host_points_.insert(host_points_.end(),

  //                     host_points_bp_r_.begin(), host_points_bp_r_.end());



  const int raw_pts = static_cast<int>(host_points_.size()) / 5;



  // ── PRE-PROCESSING: remove ego-vehicle points ───────────────

  EgoVehicleFilter(host_points_);

  const int filtered_pts = static_cast<int>(host_points_.size()) / 5;



  printf("[CenterPoint] raw=%d  after_ego_filter=%d  (removed %d pts)\n",

         raw_pts, filtered_pts, raw_pts - filtered_pts);



  // Hard cap

  if (host_points_.size() > static_cast<size_t>(max_all_points_ * 5))

    host_points_.resize(max_all_points_ * 5);



  checkCudaErrors(cudaMemcpy(dev_points_,
                             host_points_.data(),
                             host_points_.size() * sizeof(float),
                             cudaMemcpyHostToDevice));



  centerpoint_ptr_->doinfer(

      dev_points_,

      static_cast<int>(host_points_.size() / 5),

      stream_);



  checkCudaErrors(cudaDeviceSynchronize());


  // ── 如果 lidar 异常，只发送默认 box ─────────────────────────
  if (send_default_box) {
    std::vector<Bndbox> boxes;  // 创建新的空 vector

    Bndbox default_box;
    default_box.x = 5.0f;      // 前方 5 米
    default_box.y = 0.0f;
    default_box.z = 1.0f;
    default_box.w = 2.0f;      // 宽
    default_box.l = 4.5f;      // 长
    default_box.h = 2.0f;      // 高
    default_box.rt = 0.0f;
    default_box.id = 0;        // car
    default_box.score = -1.0f; // 表示无效/异常状态
    boxes.push_back(default_box);
    printf("[CenterPoint] Lidar异常，只发送默认box: %s\n", default_box_reason.c_str());

    PubDetectedMarker(boxes);
  } else {
    // ── 简单前向障碍物检测 ──
    std::vector<Bndbox> simple_boxes;
    bool has_simple = ExtractFrontObstacleBoxes(host_points_, simple_boxes);

    auto& boxes = centerpoint_ptr_->nms_pred_;

    if (has_simple) {
      boxes.insert(boxes.end(), simple_boxes.begin(), simple_boxes.end());
    }
    PubDetectedMarker(boxes);
  }
}




// ─────────────────────────────────────────────────────────────

//  POST-PROCESSING: publish box markers + text labels

// ─────────────────────────────────────────────────────────────

void CenterPointRos::PubDetectedMarker(std::vector<Bndbox> boxes)

{

  visualization_msgs::MarkerArray marker_array;



  // ── Template marker (shared fields) ─────────────────────────

  visualization_msgs::Marker box_marker;

  box_marker.header.frame_id = kParams::kFrameId;

  box_marker.header.stamp    = ros::Time::now();

  box_marker.ns              = "objects";

  box_marker.type            = visualization_msgs::Marker::CUBE;

  box_marker.action          = visualization_msgs::Marker::ADD;

  box_marker.lifetime        = ros::Duration(kParams::kMarkerLifetime);



  // ── Template text label ─────────────────────────────────────

  visualization_msgs::Marker text_marker;

  text_marker.header     = box_marker.header;

  text_marker.ns         = "labels";

  text_marker.type       = visualization_msgs::Marker::TEXT_VIEW_FACING;

  text_marker.action     = visualization_msgs::Marker::ADD;

  text_marker.lifetime   = box_marker.lifetime;

  text_marker.scale.z    = kParams::kTextScale;   // only z controls text height

  text_marker.color.r    = kParams::kTextColorR;

  text_marker.color.g    = kParams::kTextColorG;

  text_marker.color.b    = kParams::kTextColorB;

  text_marker.color.a    = kParams::kTextColorA;



  int marker_id = 0;



  for (const auto& box : boxes) {



    // ── Resolve class config ──────────────────────────────────

    if (box.id < 0 || box.id >= kParams::kNumClasses) continue;

    const kParams::ClassCfg& cfg = kParams::kClassCfg[box.id];



    // ── 特殊处理: score < 0 表示 lidar 异常时的默认 box ───────
    bool is_default_box = (box.score < 0.0f);

    // ── Global minimum score gate ─────────────────────────────
    //    默认 box (score=-1) 跳过此检查
    if (!is_default_box && box.score < 0.1f) continue;



    // ── Per-class + per-region score threshold ─────────────────
    //    默认 box 跳过阈值检查
    float thresh = 0.0f;
    if (!is_default_box) {
      thresh = kParams::GetThresholdForBox(box.id, box.x, box.y);
      if (thresh > 0.99f) continue;  // 禁用类别
      if (box.score < thresh) continue;
    }



    // ── Ego-region guard for detections (belt-and-suspenders) ─
    //    (points were already filtered, but model can still output
    //     boxes centred on the vehicle body from partial context)
    //    默认 box 不检查 ego 区域
    if (!is_default_box && IsEgoPoint(box.x, box.y, box.z)) continue;



    // ── BOX MARKER ────────────────────────────────────────────

    box_marker.id              = marker_id;

    box_marker.pose.position.x = box.x;

    box_marker.pose.position.y = box.y;

    box_marker.pose.position.z = box.z;

    box_marker.scale.x         = box.w;

    box_marker.scale.y         = box.l;

    box_marker.scale.z         = box.h;

    // ── 颜色: 默认 box 使用黄色，正常的 box 使用类别颜色 ─────
    if (is_default_box) {
      box_marker.color.r         = 1.0f;  // 黄色
      box_marker.color.g         = 1.0f;
      box_marker.color.b         = 0.0f;
      box_marker.color.a         = 1.0f;  // 不透明
    } else {
      box_marker.color.r         = cfg.r;
      box_marker.color.g         = cfg.g;
      box_marker.color.b         = cfg.b;
      box_marker.color.a         = box.score;       // alpha = confidence
    }

    box_marker.pose.orientation = tf::createQuaternionMsgFromYaw(-box.rt);

    marker_array.markers.push_back(box_marker);



    // ── TEXT LABEL MARKER ─────────────────────────────────────
    text_marker.id              = marker_id + 10000; // offset to avoid ID clash

    // 默认 box 显示 "INVALID" + 异常原因
    if (is_default_box) {
      text_marker.text            = "INVALID";
    } else {
      text_marker.text            = MakeLabelText(box.id, box.score);
    }

    text_marker.pose.position.x = box.x;

    text_marker.pose.position.y = box.y;

    // Float text above the top face of the box

    text_marker.pose.position.z = box.z + box.h * 0.5f + kParams::kTextOffsetZ;

    text_marker.pose.orientation.w = 1.0;

    marker_array.markers.push_back(text_marker);



    ++marker_id;

  }



  pub_objects_.publish(marker_array);

}

