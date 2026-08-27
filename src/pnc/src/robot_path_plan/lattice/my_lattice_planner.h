#ifndef MY_LATTICE_PLANNER_H
#define MY_LATTICE_PLANNER_H

#include <vector>
#include <iostream>
#include <memory>
#include <mutex>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "collision_check_with_bbox.h"
// #include "planner_utils.h"
#include "pose2d.h"

typedef std::array<double, 4> SState;
typedef std::array<double, 4> LState;

struct SPolyParam
{
    double end_t = 0.0;
    double end_v = 0.0;
    std::array<double, 6> params;
};

struct LPolyParam
{
    double end_s = 0.0;
    double end_l = 0.0;
    std::array<double, 6> params;
};

enum class State
{
    CRUISE,
    STOP,
    STOP_AHEAD_OBSTACLE,
    AVOID_OBSTACLE
};

struct FrenetPoint
{
    double s = 0.0;
    double ds_t = 0.0;  // ds/dt 一阶导
    double d2s_t = 0.0; // d2s/dt 二阶导

    double l = 0.0;
    double dl_s = 0.0;  // dl/ds 一阶导
    double d2l_s = 0.0; // d2l/ds    二阶导

    void printSelf()
    {
        printf("s:%f ds_t:%f d2s_t:%f l:%f dl_s:%f d2l_s:%f\n", s, ds_t, d2s_t, l, dl_s, d2l_s);
    }
};

typedef std::vector<FrenetPoint> FrenetPath;

struct Path2d
{
    double cost = 0.0;               // 总代价
    double lat_diff_ref_cost = 0.0;  // 与参考线横向偏移代价
    double lat_diff_last_cost = 0.0; // 与上一帧路径横向偏移代价
    double travelled_cost = 0.0;     // t时间内行驶的距离代价
    double smooth_cost = 0.0;        // 平滑代价
    double destination_cost = 0.0;   // 终点代价
    double obstacle_dist_cost = 0.0; // 障碍物距离代价
    std::vector<Pose2d> poses;
    void sumCost()
    {
        cost = lat_diff_ref_cost + lat_diff_last_cost + smooth_cost +
               travelled_cost + destination_cost + obstacle_dist_cost;
    }
};

class MyLatticePlanner
{
public:
    MyLatticePlanner() = delete;
    MyLatticePlanner(CollisionCheckWithBBoxSPtr &collision_check);
    ~MyLatticePlanner();

    std::vector<Pose2d> plan(const std::vector<Pose2d> &reference_path,
                             const Pose2d &ego_pose,
                             std::vector<ObstaclePtr> obstacles);

protected:
    void loadParams();
    std::vector<LPolyParam> latPlan(const FrenetPoint &sfre_pt, const Pose2d &end_ref_pt);
    std::vector<FrenetPath> combineSLTrajectory(const std::vector<LPolyParam> &lpoly_vec,
                                                const double &s0, const double &max_s);
    std::vector<Path2d> filterValidPaths(std::vector<FrenetPath> &fre_paths,
                                         const std::vector<Pose2d> &ref_ls,
                                         std::vector<ObstaclePtr> obstacles);

private:
    FrenetPoint toFrenet(const Pose2d &p, const Pose2d &r);
    Pose2d toCartesian(const FrenetPoint &p, const Pose2d &r);
    Pose2d findNearstRefPoint(const double &s, const std::vector<Pose2d> &ref_ls);
    Pose2d findMatchedPoint(const Pose2d &path_pt, const std::vector<Pose2d> &ref_ls);
    Pose2d findMatchedPoint(const double &s, const std::vector<Pose2d> &ref_ls);

    bool isCurvatureOk(const Pose2d &cs_pt);
    bool isCollision(const Pose2d &cs_pt, const std::vector<ObstaclePtr> &obstacles);

    SPolyParam computePolyParam(SState start, SState end);

    // 测试
    void writeToFile(const std::vector<SPolyParam> &spoly_vec, const std::vector<LPolyParam> &lpoly_vec);
    void drawSampledPath(std::vector<Path2d> &path_vec);
    // void drawOptimalPath(Path &optimal_path);

private:
    CollisionCheckWithBBoxSPtr collision_check_with_bbox_ptr_;
    // nav_msgs::OccupancyGrid cost_map_;
    // cv::Mat obstacle_distance_;
    std::vector<double> end_l_states_;
    double end_l_step_ = 0.2;
    int end_l_left_num_ = 25;
    int end_l_right_num_ = 25;
    double end_s_min_ = 5.0;
    int end_s_sample_num_ = 4;
    double weight_lat2ref_ = 1.0;
    double weight_lat2last_ = 1.0;
    double weight_travelled_ = 1.0;
    double weight_smooth_ = 1.0;
    double weight_destination_ = 1.0;
    double weight_obstacle_dist_ = 1.0;

    double inflation_w_ = 0.0;
    double inflation_l_ = 0.0;

    FrenetPath last_fre_path_;
    std::vector<Pose2d> last_path_;

    // 可视化
    ros::NodeHandle nh_;
    ros::Publisher sample_path_pub_;
    ros::Publisher optimal_path_pub_;
    // std::unique_ptr<CSVLogger> csv_logger_;

    double max_curvature_ = 0.22;
};

#endif