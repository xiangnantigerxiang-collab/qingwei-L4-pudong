#include <cmath>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include "math_utils.h"
#include "my_lattice_planner.h"

MyLatticePlanner::MyLatticePlanner(CollisionCheckWithBBoxSPtr &collision_check)
    : collision_check_with_bbox_ptr_(collision_check)

{
    this->loadParams();
    sample_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/sampled_path", 1);
    optimal_path_pub_ = nh_.advertise<nav_msgs::Path>("/optimal_path", 1);
    // csv_logger_ = std::make_unique<CSVLogger>("/root/workspace/chrtc_626project/", "lattice");
}

MyLatticePlanner::~MyLatticePlanner()
{
}

void MyLatticePlanner::loadParams()
{
    // 加载参数
    inflation_w_ = 0.1;
    inflation_l_ = 0.1;
    end_l_step_ = 2.0;
    end_l_left_num_ = 1;
    end_l_right_num_ = 0;
    end_s_sample_num_ = 2;
    end_s_min_ = 4.0;
    weight_lat2ref_ = 1.0;
    weight_lat2last_ = 3.0;
    weight_travelled_ = 0;
    weight_smooth_ = 10;
    weight_obstacle_dist_ = 0.0;
    weight_destination_ = 0.0;

    auto car_model = collision_check_with_bbox_ptr_->getCarModel();
    max_curvature_ = std::atan(car_model.l_wb / car_model.max_steer);
    printf("Lattice Planner参数:\n车宽膨胀值:%f\n车长膨胀值:%f\n最大曲率:%f\n参考线偏差权重:%f\n上一帧路径权重:%f\n",
           inflation_w_, inflation_l_, max_curvature_, weight_lat2ref_, weight_lat2last_);
    end_l_states_.push_back(0);
    for (int i = 1; i <= end_l_left_num_; ++i)
    {
        end_l_states_.push_back(end_l_step_ * i);
    }
    for (int i = 1; i <= end_l_right_num_; ++i)
    {
        end_l_states_.push_back(-end_l_step_ * i);
    }
    printf("横向采样: [");
    for (int i = 0; i < end_l_states_.size(); ++i)
    {
        double &l = end_l_states_[i];
        if (i == end_l_states_.size() - 1)
        {
            printf("%f]\n", l);
        }
        else
        {
            printf("%f,", l);
        }
    }
}

std::vector<Pose2d> MyLatticePlanner::plan(const std::vector<Pose2d> &reference_path,
                                           const Pose2d &ego_pose,
                                           std::vector<ObstaclePtr> obstacles)
{
    printf("=========================== 进入lattice规划 ===========================\n");
    TimeLogger timer;
    timer.start();
    if(reference_path.empty())
    {
        printf("参考轨迹为空\n");
        return {};
    }
    std::vector<Pose2d> ref_path = reference_path;
    math_utils::computePoseAttr(ref_path);
    // 轨迹拼接
    std::vector<Pose2d> base_path;
    Pose2d matched_pose;
    FrenetPoint sfre_pose;
    if (!last_path_.empty())
    {
        auto foot_pose = math_utils::getFootPose(ego_pose, last_path_);
        // 计算拼接点
        double base_length = std::max(ego_pose.v * 0.5, 2.0);
        printf("轨迹拼接, 拼接长度: %f\n", base_length);
        // 裁剪拼接路径
        for (auto &pose : last_path_)
        {
            double delta_s = pose.s - foot_pose.s;
            printf("foot_s:%f, pose_s:%f\n", foot_pose.s, pose.s);
            if (delta_s >= 0 && delta_s <= base_length)
            {
                base_path.push_back(pose);
            }
        }
        printf("base path size:%d\n", base_path.size());
        Pose2d stitch_pose = base_path.back();
        matched_pose = findMatchedPoint(stitch_pose, ref_path);
        sfre_pose = toFrenet(stitch_pose, matched_pose);
    }
    else
    {
        matched_pose = findMatchedPoint(ego_pose, ref_path);
        sfre_pose = toFrenet(ego_pose, matched_pose);
    }

    auto end_ref_pt = ref_path.back();
    auto lpoly_vec = latPlan(sfre_pose, end_ref_pt);
    // printf("frenet 匹配点dist:  %f", dist);
    // sfre_pt.printSelf();
    printf("拼接横纵向轨迹\n");
    auto fre_ls_vec = combineSLTrajectory(lpoly_vec, sfre_pose.s, end_ref_pt.s);
    // auto fre_ls_vec = combineSLTrajectory(spoly_vec, lpoly_vec, end_ref_pt.s);
    printf("拼接轨迹数量为:%d\n", fre_ls_vec.size());
    auto cs_paths = filterValidPaths(fre_ls_vec, ref_path, obstacles);
    printf("筛选出的轨迹数量为: %d\n", cs_paths.size());
    std::vector<Pose2d> result = base_path;
    if (!cs_paths.empty())
    {
        auto stitch_path = cs_paths.front();
        result.insert(result.end(), stitch_path.poses.begin(), stitch_path.poses.end());
    }
    math_utils::computePoseAttr(result);
    // drawSampledPath(cs_paths);
    //  drawOptimalPath(cs_paths.front());
     printf("================Lattice 规划完成， 耗时: %f ms\n", timer.duration());
    last_path_ = result;
    return result;
}

SPolyParam MyLatticePlanner::computePolyParam(SState start, SState end)
{
    // 起始条件
    double t_0 = start[0];
    double s_0 = start[1];
    double v_0 = start[2];
    double a_0 = start[3];
    // 终止条件
    double t_i = end[0];
    double s_i = end[1];
    double v_i = end[2];
    double a_i = end[3];

    // 5次多项式计算
    // s = a0 + a1*t = a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    // ds/dt = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4
    // d2s/dt = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3
    double c0 = (s_i - 0.5 * t_i * t_i * a_0 - v_0 * t_i - s_0) / std::pow(t_i, 3);
    double c1 = (v_i - a_0 * t_i - v_0) / std::pow(t_i, 2);
    double c2 = (a_i - a_0) / t_i;

    double a0 = s_0;
    double a1 = v_0;
    double a2 = 0.5 * a_0;
    double a3 = 0.5 * (20 * c0 - 8.0 * c1 + c2);
    double a4 = (-15.0 * c0 + 7.0 * c1 - c2) / t_i;
    double a5 = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / std::pow(t_i, 2);

    // std::array<double, 5> params = {a0, a1, a2, a3, a4};
    // printf("t_i: %f, v_i: %f, a_i: %f, b0: %f, b1: %f\n", t_i, v_i, a_i, b0, b1);
    // printf("i: %d, a0:%f a1:%f a2:%f a3:%f a4:%f\n", i++, a0, a1, a2, a3, a4);
    SPolyParam s_param;
    s_param.end_t = t_i;
    s_param.end_v = v_i;
    s_param.params = {a0, a1, a2, a3, a4, a5};
    return s_param;
}

std::vector<LPolyParam> MyLatticePlanner::latPlan(const FrenetPoint &sfre_pt, const Pose2d &end_ref_pt)
{
    std::vector<LPolyParam> lpoly_vec;
    // 建立五次多项式模型
    //  l_s = a0 + a1s + a2s^2 + a3s^3 + a4s^4 + a5s^5
    // dl_s = a1 + 2*a2s + 3*a3s^2 + 4a4s^3 + 5a5s^4
    // d2l_s = 2*a2 + 6*a4s + 12a4s^2 + 20a5s^3
    // 1. 起始条件
    double a0 = sfre_pt.l;
    double a1 = sfre_pt.dl_s;
    double a2 = 0.5 * sfre_pt.d2l_s;
    printf("起始状态s: %f, l:%f, 终止状态: s:%f\n", sfre_pt.s, sfre_pt.l, end_ref_pt.s);
    // 2. 采样末状态
    // c0 = (l_i - 0.5s_j^2*d2l_s_0 - dl_ds_0 *s_j - l_0)/s_j^3
    // std::array<double, 3> end_l_states = {0, -3.0, 3.0};
    // std::array<double, 4> end_s_states = {20, 30, 40, 50};
    // std::array<double, 4> end_l_states = {, -4, 4, -5};
    // std::array<double, 9> end_l_states = {0.01, -2, 2, -6.5, 4.5, 3.5, -3.5, 0.5, -0.5};
    // std::array<double, 4> end_s_states = {2, 4, 6, 8};
    double max_s = end_ref_pt.s - sfre_pt.s;
    max_s = max_s < end_s_min_ ? end_s_min_ : max_s;
    double s_step = (max_s - end_s_min_) / end_s_sample_num_;
    std::vector<double> end_s_states;
    for (int i = 1; i <= end_s_sample_num_; i++)
    {
        double end_s = end_s_min_ + s_step * i;
        printf("i:%d, end_s:%f, max_s:%f\n", i, end_s, max_s);
        end_s_states.push_back(end_s);
    }
    // std::array<double, 2> end_s_states = {8, 15};
    //  3. 生成五次多项式
    for (auto &end_s : end_s_states)
    {
        int i = 0;
        for (auto &end_l : end_l_states_)
        {

            double param_c0 = end_l - a0 - a1 * end_s - 2 * a2 * end_s * end_s;
            double param_c1 = -a1 - 2 * a2 * end_s;
            double param_c2 = -2 * a2;
            // printf("c0:%f c1:%f c2:%f\n", param_c0, param_c1, param_c2);
            double a5 = (12 * param_c0 / std::pow(end_s, 2) - 6 * param_c1 / end_s + param_c2) / (2 * std::pow(end_s, 3));
            double a4 = (-3 * param_c0 / end_s + param_c1 - 2 * std::pow(end_s, 4) * a5) / std::pow(end_s, 3);
            double a3 = (param_c0 - std::pow(end_s, 4) * a4 - std::pow(end_s, 5) * a5) / std::pow(end_s, 3);
            // std::array<double, 6> params = {a0, a1, a2, a3, a4, a5};

            // printf("i: %d, a0:%f a1:%f a2:%f a3:%f a4:%f a5:%f\n", i++, a0, a1, a2, a3, a4, a5);
            if (std::isnan(a3) || std::isnan(a4))
            {
                printf("============nannan===============\n");
                // printf("end_s:%f, end_l:%f, c0:%f, c1:%f, c2:%f\n", end_s, end_l, param_c0, param_c1, param_c2);
                printf("i: %d, a0:%f a1:%f a2:%f a3:%f a4:%f a5:%f\n", i++, a0, a1, a2, a3, a4, a5);
            }
            LPolyParam l_param;
            l_param.end_s = end_s;
            l_param.end_l = end_l;
            l_param.params = {a0, a1, a2, a3, a4, a5};

            lpoly_vec.emplace_back(l_param);
        }
    }
    return lpoly_vec;
}

std::vector<FrenetPath> MyLatticePlanner::combineSLTrajectory(const std::vector<LPolyParam> &lpoly_vec,
                                                              const double &s0, const double &max_s)
{
    std::vector<FrenetPath> frenet_paths;
    for (auto ldata : lpoly_vec)
    {
        auto lpoly = ldata.params;
        FrenetPath fre_path;
        FrenetPoint last_fre_pt;
        double s = s0;
        while (ros::ok())
        {
            double rs = s - s0;
            double l = lpoly[0] + lpoly[1] * rs + lpoly[2] * std::pow(rs, 2) + lpoly[3] * std::pow(rs, 3) + lpoly[4] * std::pow(rs, 4) +
                       lpoly[5] * std::pow(rs, 5);
            double dl_s = lpoly[1] + 2 * lpoly[2] * rs + 3 * lpoly[3] * std::pow(rs, 2) + 4 * lpoly[4] * std::pow(rs, 3) +
                          5 * lpoly[5] * std::pow(rs, 4);
            double d2l_s = 2 * lpoly[2] + 6 * lpoly[3] * rs + 12 * lpoly[4] * std::pow(rs, 2) + 20 * lpoly[5] * std::pow(rs, 3);
            // printf("0: %f, 1:%f, 2:%f, ")
            // printf("t: %f, s:%f s0:%f l:%f\n", t, s, s0, l);
            if (s > max_s)
            {
                // printf("达到终止条件, s:%f, s0:%f, max_s:%f\n", s, s0, max_s);
                break;
            }
            if (rs > ldata.end_s)
            {
                l = last_fre_pt.l;
                dl_s = last_fre_pt.dl_s;
                d2l_s = last_fre_pt.d2l_s;
            }
            s = s + 0.2;
            FrenetPoint fre_pt;
            fre_pt.s = s;
            fre_pt.ds_t = 0;
            fre_pt.d2s_t = 0;
            fre_pt.l = l;
            fre_pt.dl_s = dl_s;
            fre_pt.d2l_s = d2l_s;
            last_fre_pt = fre_pt;

            fre_path.emplace_back(fre_pt);
        }
        frenet_paths.push_back(std::move(fre_path));
    }

    return frenet_paths;
}

std::vector<Path2d> MyLatticePlanner::filterValidPaths(std::vector<FrenetPath> &fre_paths,
                                                       const std::vector<Pose2d> &ref_ls,
                                                       std::vector<ObstaclePtr> obstacles)
{
    std::vector<Path2d> results;
    double min_cost = 1e6;
    int min_idx = 0;
    for (int i = 0; i < fre_paths.size(); ++i)
    {
        Path2d cs_path;
        FrenetPath fre_path = fre_paths.at(i);
        bool check_ok = true;
        // LatticeCSVLogger logger("/root/workspace/chrtc_626project");
        // logger.start();
        double min_obstalce_dist = 1e6;
        FrenetPoint last_check_collison_point;
        for (int j = 0; j < fre_path.size(); ++j)
        {
            FrenetPoint fre_pt = fre_path.at(j);
            // logger.log(fre_pt.s, fre_pt.l);
            if (j == 0)
            {
                last_check_collison_point = fre_pt;
            }
            double dist = fre_pt.s - last_check_collison_point.s;
            auto matched_cs_pt = findMatchedPoint(fre_pt.s, ref_ls);
            auto cs_pt = toCartesian(fre_pt, matched_cs_pt);
            // if (i == 0)
            // {
            //     // printf("j:%d, x: %f, y:%f\n", j, cs_pt.x, cs_pt.y);
            // }
            // 检查曲率
            // if (!isCurvatureOk(cs_pt))
            // {
            //     // printf("i:%d, j:%d曲率不符合运动模型, 曲率: %f, 最大曲率:%f\n", i, j, cs_pt.k, max_curvature_);
            //     check_ok = false;
            //     break;
            // }
            // 检查碰撞
            if (dist > 1.0) //每隔1m进行碰撞检测
            {
                if (isCollision(cs_pt, obstacles))
                {
                    printf("碰撞, %d, %d\n", i, j);
                    check_ok = false;
                    break;
                }
                last_check_collison_point = fre_pt;
            }
            // 计算曲率代价
            // cs_path.curvature_cost += fabs(cs_pt.k);
            if (j > 0)
            {
                auto last_pt = cs_path.poses.back();
                cs_path.smooth_cost += fabs(cs_pt.heading - last_pt.heading);
            }
            // 计算与参考线的横向偏差代价
            cs_path.lat_diff_ref_cost += fabs(fre_pt.l);
            // 计算与上一帧路径的横向偏差代价
            if (!last_fre_path_.empty() && j < last_fre_path_.size())
            {
                double last_diff_l = fabs(last_fre_path_.at(j).l - fre_pt.l);
                cs_path.lat_diff_last_cost += last_diff_l;
            }
            cs_path.poses.push_back(cs_pt);
        }
        if (check_ok)
        {
            // printf("cs_path size:%d\n", cs_path.ls.size());
            double travelled_s = cs_path.poses.back().s;
            double travelled_l = cs_path.poses.back().l;
            // 障碍物距离代价
            cs_path.obstacle_dist_cost = weight_obstacle_dist_ * 15 / (1 + cs_path.obstacle_dist_cost);
            // 参考线偏移代价
            cs_path.lat_diff_ref_cost = weight_lat2ref_ * cs_path.lat_diff_ref_cost / cs_path.poses.size();
            // 上一路径偏移代价
            cs_path.lat_diff_last_cost = weight_lat2last_ * cs_path.lat_diff_last_cost / cs_path.poses.size();
            // 平滑代价
            cs_path.smooth_cost = weight_smooth_ * cs_path.smooth_cost / cs_path.poses.size();
            // 终点代价
            cs_path.destination_cost = weight_destination_ * fabs(cs_path.poses.back().l);
            // 行程代价
            cs_path.travelled_cost = weight_travelled_ / (1 + travelled_s);
            cs_path.sumCost();
            // printf("i: %d, travelled_s:%f,travelled_l:%f, lat_last:%f, lat_ref:%f, travel_cost:%f, sum_cost:%f\n",
            //        i, travelled_s, travelled_l, cs_path.lat_diff_last_cost, cs_path.lat_diff_ref_cost,
            //        cs_path.travelled_cost, cs_path.cost);
            results.emplace_back(cs_path);
            if (cs_path.cost < min_cost)
            {
                min_idx = i;
                min_cost = cs_path.cost;
            }
        }
    }
    printf("min idx: %d\n", min_idx);
    last_fre_path_ = fre_paths.at(min_idx);
    // 排序
    std::sort(results.begin(), results.end(), [](Path2d &left, Path2d &right)
              { return left.cost < right.cost; });
    return results;
}
FrenetPoint MyLatticePlanner::toFrenet(const Pose2d &path_pt, const Pose2d &ref_pt)
{
    FrenetPoint frenet_pt;

    // 计算l
    // T_r = ( cos(theta_r), sin(theta_r) )
    // N_r = ( -sin(theta_r), cos(theta_r) )
    //  l =dx*dx + dy*dy
    double dx = path_pt.x - ref_pt.x;
    double dy = path_pt.y - ref_pt.y;
    Eigen::Vector2f V_r_p(dx, dy);
    double cos_r = std::cos(ref_pt.heading);
    double sin_r = std::sin(ref_pt.heading);
    Eigen::Vector2f T_r(cos_r, sin_r);
    // Eigen::Vector2f V_rp(dx, dy);
    // Eigen::Vector2f N_r(-sin_r, cos_r);
    double dot_V_N = cos_r * dy - sin_r * dx;
    double l = copysign(std::sqrt(dx * dx + dy * dy), dot_V_N);

    // 计算l' = dl/ds
    // dl/ds= (1-l*k_r)tan(theta_p - theta_r)
    double delta_theta = path_pt.heading - ref_pt.heading;
    double tan_delta_theta = tan(delta_theta);
    double cos_delta_theta = cos(delta_theta);
    double dl_s = (1 - l * ref_pt.curvature) * tan_delta_theta;
    // 计算l'' = dl2/ds2
    // param = (1-l*k_r)/cos(theta_p-theta_r)
    // d2l/ds = -(l'*k_r + l*dkr/ds)*tan(theta_p - theta_r) + param*(param*k_p - k_r)
    double param = (1 - l * ref_pt.curvature) / cos_delta_theta;
    double param_2 = dl_s * ref_pt.curvature + l * ref_pt.dkappa;
    double d2l_s = -param_2 * tan_delta_theta + param * (param * path_pt.curvature - ref_pt.curvature);

    // 计算 s
    double s = ref_pt.s;
    // 计算 s.
    // ds/dt = v_p*cos(theta_p - theta_r)/(1-l*k_r)
    double ds_t = path_pt.v * cos_delta_theta / (1 - l * ref_pt.curvature);
    // 计算 s..
    // d2s/dt = (a_p *cos(theta_p - theta_r) - (ds/dt)^2*(dl_s*(param*k_p - k_r)-param2)/(1-l*k_r)
    double d2s_t = (path_pt.a * cos_delta_theta - ds_t * ds_t *
                                                      (dl_s * (param * path_pt.curvature - ref_pt.curvature) - param_2)) /
                   (1 - l * ref_pt.curvature);

    frenet_pt.s = s;
    frenet_pt.ds_t = ds_t;
    frenet_pt.d2s_t = d2s_t;
    frenet_pt.l = l;
    frenet_pt.dl_s = dl_s;
    frenet_pt.d2l_s = d2l_s;
    return frenet_pt;
}

Pose2d MyLatticePlanner::toCartesian(const FrenetPoint &fre, const Pose2d &ref)
{
    Pose2d carts_pt;
    // 计算位置x,y
    // printf("ref.x: %f, param:%f\n", ref.x, fre.l * sin(ref.heading));
    // printf("ref.y: %f, param:%f\n", ref.y, fre.l * cos(ref.heading));
    // printf("ref x: %f, y:%f, heading:%f, l:%f\n", ref.x, ref.y, ref.heading, fre.l);
    double x = ref.x - fre.l * sin(ref.heading);
    double y = ref.y + fre.l * cos(ref.heading);
    // 计算航向heading
    // heading = arctan(dl_s/(1-k_r*l))+theta_r
    double heading = std::atan(fre.dl_s / (1 - ref.curvature * fre.l)) + ref.heading;
    // 计算速度v
    double param = fre.ds_t * (1 - ref.curvature * fre.l);
    double v = std::sqrt(std::pow(param, 2) + std::pow(fre.ds_t * fre.dl_s, 2));

    // 计算曲率k
    double cos_delta = cos(heading - ref.heading);
    double param_k = (1 - ref.curvature * fre.l) / cos_delta;
    double param_1 = fre.d2l_s + (ref.dkappa * fre.l + ref.curvature * fre.dl_s) * tan(heading - ref.heading);
    double k = (param_1 * (cos_delta * cos_delta) / (1 - ref.curvature * fre.l) + ref.curvature) / param_k;

    // 计算加速度a
    double param_a = fre.dl_s * (k * param_k - ref.curvature) - (ref.dkappa * fre.l + ref.curvature * fre.dl_s);
    double a = fre.d2s_t * param_k + std::pow(fre.ds_t, 2) / cos_delta * param_a;

    carts_pt.x = x;
    carts_pt.y = y;
    carts_pt.heading = heading;
    carts_pt.v = v;
    carts_pt.curvature = k;
    carts_pt.a = a;
    carts_pt.s = fre.s;
    carts_pt.l = fre.l;
    return carts_pt;
}

Pose2d MyLatticePlanner::findMatchedPoint(const Pose2d &path_pt, const std::vector<Pose2d> &ref_ls)
{
    int min_index = 0;
    double min_dist = 1e9;
    // 1. 查找最近点
    for (size_t i = 0; i < ref_ls.size(); ++i)
    {
        auto &ref_pt = ref_ls.at(i);
        double dist = math_utils::distance(path_pt, ref_pt);
        if (dist > min_dist)
        {
            break;
        }
        // min_dist = dist < min_dist ? dist : min_dist;
        min_dist = dist;
        min_index = i;
    }

    // 2. 计算投影点
    int last_idx = min_index - 1;
    int next_idx = min_index + 1;
    if (min_index == 0)
    {
        last_idx = min_index;
    }
    if (min_index == ref_ls.size() - 1)
    {
        next_idx = min_index;
    }
    auto &last_pt = ref_ls.at(last_idx);
    auto &next_pt = ref_ls.at(next_idx);
    // 向量last->next
    double dx_l2n = next_pt.x - last_pt.x;
    double dy_l2n = next_pt.y - last_pt.y;
    // 向量last->p
    double dx_l2p = path_pt.x - last_pt.x;
    double dy_l2p = path_pt.y - last_pt.y;
    // 向量last->next的长度
    double len_l2n = sqrt(dx_l2n * dx_l2n + dy_l2n * dy_l2n);
    // 投影长度
    double t = (dx_l2p * dx_l2n + dy_l2p * dy_l2n) / len_l2n;
    Pose2d matched_cs_pt;

    matched_cs_pt.s = last_pt.s + t;
    // printf("min idx: %d  len_l_p:%f t: %f\n", min_index, len_l2n, t);

    // //判断是否垂直
    matched_cs_pt.x = last_pt.x + t * dx_l2n / len_l2n;
    matched_cs_pt.y = last_pt.y + t * dy_l2n / len_l2n;
    matched_cs_pt.heading = math_utils::computeHeading(last_pt, next_pt);
    matched_cs_pt.v = last_pt.v;
    matched_cs_pt.a = last_pt.a;
    // double dx_m2p = path_pt.x - matched_cs_pt.x;
    // double dy_m2p = path_pt.y - matched_cs_pt.y;
    // double dx_m_l = path_pt.x - last_pt.x;
    // double dy_m_l = path_pt.y - last_pt.y;
    // double dot = dx_m2p * dx_m2p + dy_m2p * dy_m2p;
    // printf("dot: %f\n", dot);

    // if (t < 0) // 投影在线段前
    // {
    //     // printf("投影前");
    //     matched_cs_pt = last_pt;
    // }
    // else if (t > len_l2n) // 投影在线段后
    // {
    //     // printf("投影后");
    //     matched_cs_pt = next_pt;
    // }
    // else // 投影到线段上
    // {
    //     // printf("投影上");
    //     matched_cs_pt.x = last_pt.x + t * dx_l2n / len_l2n;
    //     matched_cs_pt.y = last_pt.y + t * dy_l2n / len_l2n;
    //     matched_cs_pt.heading = computeHeading(last_pt, next_pt);
    // }

    return matched_cs_pt;
}

Pose2d MyLatticePlanner::findMatchedPoint(const double &s, const std::vector<Pose2d> &ref_ls)
{
    auto comp = [](const Pose2d &point, const double s)
    {
        return point.s < s;
    };

    auto it_lower = std::lower_bound(ref_ls.begin(), ref_ls.end(), s, comp);
    if (it_lower == ref_ls.begin())
    {
        return ref_ls.front();
    }
    auto p0 = *(it_lower - 1);
    auto p1 = *it_lower;
    double s0 = p0.s;
    double s1 = p1.s;

    Pose2d path_point;
    double weight = (s - s0) / (s1 - s0);
    double x = (1 - weight) * p0.x + weight * p1.x;
    double y = (1 - weight) * p0.y + weight * p1.y;
    double heading_d = p1.heading - p0.heading;
    double theta = p0.heading + weight * heading_d;
    double kappa = (1 - weight) * p0.curvature + weight * p1.curvature;
    double dkappa = (1 - weight) * p0.dkappa + weight * p1.dkappa;
    path_point.x = (x);
    path_point.y = (y);
    path_point.heading = math_utils::minusPi2Pi(theta);
    path_point.curvature = (kappa);
    path_point.dkappa = (dkappa);
    path_point.s = s;
    return path_point;
}

Pose2d MyLatticePlanner::findNearstRefPoint(const double &s, const std::vector<Pose2d> &ref_ls)
{
    Pose2d result_pt;
    int min_index = 0;
    double min_dist = 1e9;
    for (size_t i = 0; i < ref_ls.size(); ++i)
    {
        auto &ref_pt = ref_ls.at(i);
        double ref_s = ref_pt.s;
        double dist = fabs(s - ref_s);
        // if (dist > min_dist)
        // {
        //     break;
        // }
        if (dist < min_dist)
        {
            min_dist = dist;
            min_index = i;
        }
    }

    auto min_cs_pt = ref_ls.at(min_index);

    double s_distance = fabs(s - min_cs_pt.s);
    int front_index = s < min_cs_pt.s ? min_index - 1 : min_index;
    auto &front_cs_pt = ref_ls.at(front_index);
    result_pt = min_cs_pt;

    int to_reverse = (s < min_cs_pt.s) ? -1 : 1;
    // printf("reverse: %d\n", to_reverse);
    result_pt.x = min_cs_pt.x + to_reverse * s_distance * cos(min_cs_pt.heading);
    result_pt.y = min_cs_pt.y + to_reverse * s_distance * sin(min_cs_pt.heading);
    // printf("min index: %d,x:%f,y:%f, dist is: %f, s:%f, ref_s:%f, heading: %f\n", min_index, min_cs_pt.x, min_cs_pt.y, s_distance, s, min_cs_pt.s, min_cs_pt.heading);
    // printf("result x: %f, y:%f\n", result_pt.x, result_pt.y);
    result_pt.heading = min_cs_pt.heading;
    result_pt.s = s;

    return result_pt;
}

bool MyLatticePlanner::isCurvatureOk(const Pose2d &cs_pt)
{
    return fabs(cs_pt.curvature) < max_curvature_;
}

bool MyLatticePlanner::isCollision(const Pose2d &cs_pt, const std::vector<ObstaclePtr> &obbs)
{
    for (auto &obb : obbs)
    {
        bool result = collision_check_with_bbox_ptr_->isCollision(*obb, cs_pt, inflation_w_, inflation_l_);
        if (result)
        {
            return true;
        }
    }
    return false;
}

void MyLatticePlanner::writeToFile(const std::vector<SPolyParam> &spoly_vec, const std::vector<LPolyParam> &lpoly_vec)
{
    std::string filename = "/home/cqw/workspace/param.csv";
    std::ofstream file_log;
    file_log.open(filename, std::ios::out | std::ios::trunc);
    file_log << "a0,a1,a2,a3,a4,b0,b1,b2,b3,b4,b5\n";
    if (file_log.is_open())
    {
        for (auto &sdata : spoly_vec)
        {
            auto spoly = sdata.params;
            for (auto &ldata : lpoly_vec)
            {
                auto lpoly = ldata.params;
                file_log << spoly[0] << "," << spoly[1] << "," << spoly[2] << "," << spoly[3] << "," << spoly[4]
                         << "," << lpoly[0] << "," << lpoly[1] << "," << lpoly[2] << "," << lpoly[3] << "," << lpoly[4] << "," << lpoly[5] << std::endl;
            }
        }
    }
}

void MyLatticePlanner::drawSampledPath(std::vector<Path2d> &path_vec)
{
    visualization_msgs::MarkerArray ls_array;
    // printf("sample ve size: %d\n", path_vec.size());
    for (int i = 0; i < path_vec.size(); i++)
    {
        auto path = path_vec.at(i);
        visualization_msgs::Marker ls_marker;
        ls_marker.header.frame_id = "map";
        ls_marker.header.stamp = ros::Time::now();
        ls_marker.ns = "sampled_path";
        ls_marker.id = i;
        ls_marker.type = visualization_msgs::Marker::POINTS;
        ls_marker.action = visualization_msgs::Marker::ADD;
        ls_marker.scale.x = 0.05;
        ls_marker.scale.y = 0.05;
        ls_marker.color.b = 1.0;
        ls_marker.color.a = 1.0;
        int j = 0;
        // if (i == 0)
        // {
        //     printf("path size: %d\n", path.ls.size());
        // }
        for (auto &cs_pt : path.poses)
        {
            geometry_msgs::Point p;
            p.x = cs_pt.x;
            p.y = cs_pt.y;
            // if (i == 0)
            //     printf("i: %d, j:%d, x:%f, y:%f\n", i, j, p.x, p.y);
            ls_marker.points.push_back(p);
            j++;
        }
        // if (i == 2)
        ls_array.markers.push_back(ls_marker);
    }
    sample_path_pub_.publish(ls_array);
}

// void MyLatticePlanner::drawOptimalPath(Path &optimal_path)
// {
//     nav_msgs::Path path;
//     for (auto &pt : optimal_path.ls)
//     {
//         geometry_msgs::PoseStamped pose_stamped;
//         pose_stamped.pose.position.x = pt.x;
//         pose_stamped.pose.position.y = pt.y;
//         path.poses.push_back(pose_stamped);
//     }
//     path.header.frame_id = "map";
//     path.header.stamp = ros::Time::now();
//     optimal_path_pub_.publish(path);
// }
