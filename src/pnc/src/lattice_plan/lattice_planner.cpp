#include "lattice_planner.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>
#include <set>

std::vector<PathPoint> ToDiscretizedReferenceLine(
    const std::vector<OriginalInsData> &ref_points)
{
    double s = 0.0;
    std::vector<PathPoint> path_points;

    for (const auto &ref_point : ref_points)
    {
        PathPoint path_point;
        path_point.setX(ref_point.x);
        path_point.setY(ref_point.y);
        path_point.setTheta(ref_point.heading);
        path_point.setKappa(ref_point.kappa);
        path_point.setDkappa(ref_point.dkappa);

        if (!path_points.empty())
        {
            double dx = path_point.x() - path_points.back().x();
            double dy = path_point.y() - path_points.back().y();
            s += std::sqrt(dx * dx + dy * dy);
        }

        path_point.setS(s);
        path_points.push_back(std::move(path_point));
    }

    return path_points;
}

void LatticePlanner::ComputeInitFrenetState(const PathPoint &matched_point,
                                            const TrajectoryPoint &cartesian_state,
                                            std::array<double, 3> *ptr_s,
                                            std::array<double, 3> *ptr_d)
{
    cartesianfrenetconverter->cartesian_to_frenet(
        matched_point.s(), matched_point.x(), matched_point.y(),
        matched_point.theta(), matched_point.kappa(), matched_point.dkappa(),
        cartesian_state.path_point.x(), cartesian_state.path_point.y(),
        cartesian_state.v(), cartesian_state.a(),
        cartesian_state.path_point.theta(),
        cartesian_state.path_point.kappa(), ptr_s, ptr_d);
}

std::tuple<std::vector<TrajectoryPoint>, std::vector<std::vector<TrajectoryPoint>>, bool, int> LatticePlanner::GetMinCostPath(const std::vector<std::vector<TrajectoryPoint>> &trajectory, std::tuple<bool, bool, int> &path_status, int vehiclestatus)
{
    int mincost_idex = 0;
    int changepath_idex = 0;
    static int count_last = 5;
    bool micost_change = false;
    bool pathchange = true;
    int collision_number = 0;
    bool all_path_collision = false;

    for (int i = 0; i < trajectory.size(); ++i)
    {
        if (trajectory[i].back().safeproperty())
            collision_number += 1;
    }
    printf("collision_number:%d, trajectory_size:%d\n", collision_number, trajectory.size());
    if (collision_number >= trajectory.size())
    {
        changepath_idex = 5;
        all_path_collision = true;
    }
    else
    {
        mincost_idex = CalculateMinNumber(trajectory, path_status, count_last, micost_change);
        changepath_idex = CalculateMinNumber(trajectory, path_status, count_last, pathchange);
        count_last = mincost_idex;
    }

    auto path_final = std::forward_as_tuple(trajectory[changepath_idex], trajectory, all_path_collision, changepath_idex);

    return path_final;
}

std::tuple<std::vector<TrajectoryPoint>, std::vector<std::vector<TrajectoryPoint>>, bool, int> LatticePlanner::PlanOnReferenceLine(
    const TrajectoryPoint &planning_init_point,
    std::vector<OriginalInsData> &reference_line,
    std::vector<sCellMsg> lidarobjs_global,
    int vehiclestatus)
{
    // 1. obtain a reference line and transform it to the PathPoint format.
    auto ptr_reference_line =
        std::make_shared<std::vector<PathPoint>>(ToDiscretizedReferenceLine(reference_line));

    // 2. compute the matched point of the init planning point on the reference line.
    PathPoint matched_point = pathmatcher->MatchToPath(
        *ptr_reference_line, planning_init_point.path_point.x(), planning_init_point.path_point.y());

    // 3. according to the matched point, compute the init state in Frenet frame.
    std::array<double, 3> init_s;
    std::array<double, 3> init_d;
    ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);

    // 5. generate 1d trajectory bundle for longitudinal and lateral respectively.
    Trajectory1dGenerator trajectory1d_generator(init_s, init_d);
    std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
    std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;

    trajectory1d_generator.GenerateTrajectoryBundles(
        speed_limit, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle);

    // combine two 1d trajectories to one 2d trajectory
    combined_trajectory = trajectorycombiner->Combine(
        *ptr_reference_line, lon_trajectory1d_bundle, lat_trajectory1d_bundle);

    if (combined_trajectory[0].size() < 1)
    {
        std::vector<TrajectoryPoint> empty_path;
        std::vector<std::vector<TrajectoryPoint>> other_paths;
        empty_path.clear();
        other_paths.clear();

        return std::forward_as_tuple(empty_path, other_paths, true, 4);
    }

    CollisionCheck collisioncheck(combined_trajectory, vehicle_param, lidarobjs_global);
    checked_trajectory = collisioncheck.Collisionproperty();

    PathDecision pathDecision(checked_trajectory);
    int pathcost_number = pathDecision.GetMaxcostPath();
    auto path_status = pathDecision.PathSideJudge(pathcost_number);
    printf("lattice: check_traj_size:%d\n", checked_trajectory.size());
    auto final_path = GetMinCostPath(checked_trajectory, path_status, vehiclestatus);

    return final_path;
}

int LatticePlanner::CalculateMinNumber(std::vector<std::vector<TrajectoryPoint>> trajectory,
                                       std::tuple<bool, bool, int> &path_status,
                                       int idex_last, bool changepath)
{
    double path_cost_min = 2.1e18;
    double path_change_cost = 0.0;
    int path_idex = 0;
    int idex_begin = 0;
    int path_size = trajectory.size();
    bool left_side_pass = std::get<0>(path_status);
    bool right_side_pass = std::get<1>(path_status);
    int path_cost_max_idex = std::get<2>(path_status);
    // printf("left_side_pass:%d, ritht_side_pass:%d, path_cost_max_idex:%d\n",
    //        int(left_side_pass), int(right_side_pass), path_cost_max_idex);
    left_side_pass = true;
    right_side_pass = false;
    if (left_side_pass)
    {
        idex_begin = 0;
        path_size = path_cost_max_idex;
    }
    else if (right_side_pass)
    {
        idex_begin = path_cost_max_idex;
        path_size = trajectory.size();
    }

    for (int i = idex_begin; i < path_size; ++i)
    {
        if (trajectory[i].back().safeproperty())
            continue;

        if (changepath)
            path_change_cost = trajectorycost->PathChangeCost(idex_last, i);
        else
            path_change_cost = 0.0;

        double path_cost_ = trajectory[i].back().cost() + path_change_cost;
        //printf("i:%d, path_cost:%f, path_change_cost:%f\n", i, path_cost_, path_change_cost);

        if (path_cost_ < path_cost_min)
        {
            path_cost_min = path_cost_;
            path_idex = i;
        }
    }
    //printf("min_cost_idex:%d, min_cost:%f\n", path_idex, path_cost_min);

    return path_idex;
}
