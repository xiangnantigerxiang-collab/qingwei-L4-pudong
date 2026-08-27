#include "trajectory_combiner.h"

std::vector<std::vector<TrajectoryPoint>> TrajectoryCombiner::Combine(
    const std::vector<PathPoint>& reference_line, 
    const std::vector<std::shared_ptr<Curve1d>>& lon_trajectory_total,
    const std::vector<std::shared_ptr<Curve1d>>& lat_trajectory_total) 
{
    DpPolyPathConfig config;
    std::vector<std::vector<SDPoint>> sd_point;
    std::vector<SDPoint> sd_point_pre;

    for (const auto& lon_trajectory : lon_trajectory_total) {
        for (const auto& lat_trajectory : lat_trajectory_total) {
            double s0 = lon_trajectory->Evaluate(0, 0.0);
            double s_ref_max = reference_line.back().s();
            double accumulated_trajectory_s = 0.0;
            double last_s = -FLAGS_numerical_epsilon;
            double t_param = 0.0;

            while (t_param < FLAGS_trajectory_time_length) {
                SDPoint point_sample;
                double s = lon_trajectory->Evaluate(0, t_param);

                if (last_s > 0.0) s = std::max(last_s, s);

                last_s = s;

                double s_dot = std::max(FLAGS_numerical_epsilon, 
                                        lon_trajectory->Evaluate(1, t_param));
                double s_ddot = lon_trajectory->Evaluate(2, t_param);

                if (s > s_ref_max) break;

                double relative_s = s - s0;
                double d = lat_trajectory->Evaluate(0, relative_s);
                double d_prime = lat_trajectory->Evaluate(1, relative_s);
                double d_pprime = lat_trajectory->Evaluate(2, relative_s);

                point_sample.s = s;
                point_sample.s_dot = s_dot;
                point_sample.s_ddot = s_ddot;
                point_sample.d = d;
                point_sample.d_prime = d_prime;
                point_sample.d_pprime = d_pprime;

                sd_point_pre.emplace_back(point_sample);
                t_param = t_param + FLAGS_trajectory_time_resolution;
            }

            sd_point.emplace_back(sd_point_pre);
            sd_point_pre.clear();
        }
    }
  
    TrajectoryCost trajectory_cost(config);
  
    std::vector<std::vector<DPRoadGraphNode>> graph_nodes;
    std::vector<DPRoadGraphNode> graph_nodes_prev;
    size_t total_level = sd_point.size();

    for (std::size_t level = 0; level < sd_point.size(); ++level) {
        const auto &level_points = sd_point[level];
        double cost_ = 0;
        double cost_mun = 0;

        for (size_t i = 0; i < level_points.size(); ++i) {
            const auto &cur_point = level_points[i];
            cost_mun += cost_;
            DPRoadGraphNode node_;
            node_.sd_point = cur_point;
            node_.total_cost = cost_mun;
            graph_nodes_prev.emplace_back(node_);
            auto &cur_node = graph_nodes_prev.back();
            cost_ = UpdateNode(&trajectory_cost, &cur_node,reference_line);
        }

        graph_nodes.emplace_back(graph_nodes_prev);
        graph_nodes_prev.clear();
        cost_mun = 0;
    }

    auto cartesian_path = FrenetPathToCartesianPath(graph_nodes, reference_line);

    return cartesian_path;
}

double TrajectoryCombiner::UpdateNode(TrajectoryCost *trajectory_cost,
                                      DPRoadGraphNode *cur_node,
                                      const std::vector<PathPoint>& reference_line) 
{
    if(trajectory_cost == NULL) return 0.0;
    if(cur_node == NULL) return 0.0;

    const auto &cur_point = cur_node->sd_point;
    const auto cost = trajectory_cost->Calculate(cur_point, reference_line);
    return cost;
}

std::vector<std::vector<TrajectoryPoint>> TrajectoryCombiner::FrenetPathToCartesianPath(
    const std::vector<std::vector<DPRoadGraphNode>> &graph_nodes,
    const std::vector<PathPoint>& reference_line)
{
    std::vector<std::vector<TrajectoryPoint>> combined_trajectory;
    std::vector<TrajectoryPoint> combined_trajectory_pre;
    int nearest_id = 0;
    double accumulated_trajectory_s = 0.0;
    int path_num = reference_line.size();
    static int count = 0;

    PathPoint prev_trajectory_point;
    PathPoint matched_ref_point;
    int path_number = FLAGS_trajectory_number; 

    for(int i = 0;i<path_number;++i) {
        const auto &nodes_prev = graph_nodes[i];

        for (int j = 0;j< nodes_prev.size();++j) {
            const auto &nodes_ = nodes_prev[j];

            for(int k=0;k<path_num;k++) {
                if (reference_line[k].s()<nodes_.sd_point.s && reference_line[k+1].s()>=nodes_.sd_point.s) nearest_id = k;
            }

            matched_ref_point = reference_line[nearest_id]; // Cartesian
       
            double x = 0.0;
            double y = 0.0;
            double theta = 0.0;
            double kappa = 0.0;
            double v = 0.0;
            double a = 0.0;

            const double rs = matched_ref_point.s();
            const double rx = matched_ref_point.x();
            const double ry = matched_ref_point.y();
            const double rtheta = matched_ref_point.theta();
            const double rkappa = matched_ref_point.kappa();
            const double rdkappa = matched_ref_point.dkappa();

            std::array<double, 3> s_conditions = {nodes_.sd_point.s, nodes_.sd_point.s_dot, nodes_.sd_point.s_ddot};
            std::array<double, 3> d_conditions = {nodes_.sd_point.d, nodes_.sd_point.d_prime, nodes_.sd_point.d_pprime};
            cartesianfrenetconverter->frenet_to_cartesian(
                rs, rx, ry, rtheta, rkappa, rdkappa, s_conditions, d_conditions, 
                &x, &y, &theta, &kappa, &v, &a);
        
            static double  pre_point_x = x;
            static double  pre_point_y = y;
        
            double delta_x = x - pre_point_x;
            double delta_y = y - pre_point_y;
            double delta_s = std::hypot(delta_x, delta_y);
            accumulated_trajectory_s += delta_s;

            TrajectoryPoint trajectory_point;
            trajectory_point.path_point.setX(x);
            trajectory_point.path_point.setY(y);
            trajectory_point.path_point.setS(accumulated_trajectory_s);
            trajectory_point.path_point.setTheta(theta);
            trajectory_point.path_point.setKappa(kappa);
            trajectory_point.setCost(nodes_.total_cost);

            prev_trajectory_point = trajectory_point.path_point;
            pre_point_x = prev_trajectory_point.x();
            pre_point_y = prev_trajectory_point.y();

            combined_trajectory_pre.emplace_back(trajectory_point);
        }

        combined_trajectory.emplace_back(combined_trajectory_pre);
        combined_trajectory_pre.clear();
        accumulated_trajectory_s = 0.0;
    }

    return combined_trajectory;
}
