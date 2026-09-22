// 新感知避障的参数/输入/输出适配，保留其他业务的最终覆盖顺序。
bool PathPlanComply::ConfigurePerceptionSafety() {
    planning_perception::CONFIG_S config;
    const std::pair<const char *, double *> parameters[] = {
        {"empty_width", &config.empty_width}, {"loaded_width", &config.loaded_width}, {"empty_length", &config.empty_length}, {"loaded_length", &config.loaded_length}, {"empty_front", &config.empty_front}, {"loaded_front", &config.loaded_front}, {"lateral_margin", &config.lateral_margin}, {"stop_margin", &config.stop_margin}, {"emergency_margin", &config.emergency_margin}, {"reaction_time", &config.reaction_time}, {"comfort_deceleration", &config.comfort_deceleration}, {"emergency_deceleration", &config.emergency_deceleration}, {"jerk", &config.jerk}, {"release_acceleration", &config.release_acceleration}, {"confirmation_time", &config.confirmation_time}, {"confirmation_gap", &config.confirmation_gap}, {"coast_time", &config.coast_time}, {"input_timeout", &config.input_timeout}, {"clear_time", &config.clear_time}, {"prediction_time", &config.prediction_time}, {"prediction_margin_rate", &config.prediction_margin_rate}, {"low_score", &config.low_score}};
    const std::string prefix = "/robot/planning/perception_safety/";
    // 静态标定仅启动时读取，避免在 10 Hz 热循环增加参数服务器 RPC。
    for(std::size_t i = 0; i < sizeof(parameters) / sizeof(parameters[0]); ++i) {
        ros::param::get(prefix + parameters[i].first, *parameters[i].second);
    }
    ros::param::get(prefix + "static_obstacle_extra_time", config.static_obstacle_extra_time);
    ros::param::get(prefix + "static_stop_min_distance", config.static_stop_min_distance);
    ros::param::get(prefix + "front_no_confirmation_distance", config.front_no_confirmation_distance);
    ros::param::get(prefix + "confirmation_hits", config.confirmation_hits);
    if(!mPerceptionSafety.Configure(config)) {
        ROS_ERROR("Invalid /robot/planning/perception_safety configuration");
        return false;
    }
    mPerceptionSafetyPath.reserve(129);
    printf("perception safety cache: %zu bytes, widths %.2f/%.2f m, reaction %.2f s, static stop %.2f m, front immediate %.2f m\n",
           mPerceptionSafety.WorkingBytes(), config.empty_width, config.loaded_width, config.reaction_time,
           config.static_stop_min_distance, config.front_no_confirmation_distance);
    return true;
}

planning_perception::LaneMotionFilter PathPlanComply::BuildPerceptionLaneMotionFilter() const {
    // 定位未到或挡位不能确定行驶方向时保留目标，避免使用默认的北向航向误剔除。
    if(!mRcvGpsData || (mVehicleData.curGear != GEAR_D && mVehicleData.curGear != GEAR_R)) {
        return planning_perception::LaneMotionFilter();
    }
    return planning_perception::LaneMotionFilter(mNavData.heading + (mVehicleData.curGear == GEAR_R ? 180.0 : 0.0));
}

void PathPlanComply::SetPlanningPerceptionData(const robot::perception &perception_t) {
    // D 挡持续接收今日新增安全逻辑所需的真实框；其他挡位保留原 ACC 输入开关。
    if(AccSwitch && mVehicleData.curGear != GEAR_D) return;
    const bool was_active = mPerceptionSafety.HasInput();
    if(!mPerceptionSafety.Observe(perception_t, ros::Time::now().toSec(), BuildPerceptionLaneMotionFilter())) return;
    if(!was_active) {
        // 旧目标风险不得带进新链；此后这个历史窗只供独立前扫描规则使用。
        history_risk_vec.assign(4, std::vector<robot::object>());
        safety_check_counter_refer = 0;
    }
}

void PathPlanComply::BuildPerceptionSafetyPath(double tLookahead) {
    mPerceptionSafetyPath.clear();
    mPerceptionSafetyPath.emplace_back(mNavData.xAxis, mNavData.yAxis);
    const double yaw = (90.0 - mNavData.heading) * M_PI / 180;
    const double ux = std::cos(yaw), uy = std::sin(yaw);
    double distance = 0;
    // 现役参考点直接来自 mDrivingPath；读取更远的同一条路线以覆盖舒适制动距离。
    // 固定扫描/输出上限，每圈复用容量；不改实际发布的路径点和任务索引。
    const bool route = !mDrivingPath.empty() && mKeypoint >= 0 && mKeypoint < static_cast<int>(mDrivingPath.size());
    const std::size_t begin = route ? static_cast<std::size_t>(mKeypoint) : 0;
    const std::size_t count = route ? mDrivingPath.size() : std::min(mReferPath.x.size(), mReferPath.y.size());
    for(std::size_t i = begin; i < count && i - begin < 2048 && mPerceptionSafetyPath.size() < 129; ++i) {
        const double x = route ? mDrivingPath[i].x_axis : mReferPath.x[i];
        const double y = route ? mDrivingPath[i].y_axis : mReferPath.y[i];
        if(mPerceptionSafetyPath.size() == 1 && (x - mNavData.xAxis) * ux + (y - mNavData.yAxis) * uy < 0) continue;
        const double dx = x - mPerceptionSafetyPath.back().x, dy = y - mPerceptionSafetyPath.back().y;
        const double squared = dx * dx + dy * dy;
        if(squared < 0.09) continue;
        mPerceptionSafetyPath.emplace_back(x, y);
        distance += std::sqrt(squared);
        if(distance >= tLookahead) break;
    }
}

void PathPlanComply::CheckTrackedReferenceSafety() {
    const bool prior_safety = mReferPath.safety;
    const double prior_speed = mReferPath.desireSpeed;
    bool front_checked = false;
    // 前扫描没有跟踪 ID/真实四角，继续独立执行原有停车规则，不拿低置信度将它滤掉。
    // 单线程内 O(1) 交换数组，检查后恢复原 /perception 输入快照。
    mPerception.objs.swap(mPerceptionFrontScan.objs);
    if(!mPerception.objs.empty()) {
        std::vector<robot::object> risks = CollectReferenceRiskObjects();
        if(UpdateReferenceRiskHistory(risks)) {
            double dh = 0;
            const auto forward = FilterForwardRiskObjects(risks, dh);
            if(!forward.empty()) {
                ApplyReferenceCollisionSpeed(forward, dh);
                front_checked = true;
            }
        }
    }
    mPerception.objs.swap(mPerceptionFrontScan.objs);
    const bool front_safety = mReferPath.safety;
    mReferPath.safety = prior_safety || front_safety;
    mReferPath.desireSpeed = std::min<double>(prior_speed, mReferPath.desireSpeed);

    planning_perception::EGO_S ego;
    ego.x = mNavData.xAxis;
    ego.y = mNavData.yAxis;
    ego.heading = mNavData.heading;
    ego.speed = std::abs(mNavData.gpsSpeed);
    ego.loaded = mVehicleData.linkPallet;
    BuildPerceptionSafetyPath(mPerceptionSafety.Lookahead(std::max(ego.speed, static_cast<double>(mReferPath.desireSpeed))));
    const double now = ros::Time::now().toSec();
    mPerceptionSafetyResult = mPerceptionSafety.Evaluate(mPerceptionSafetyPath, ego, mReferPath.desireSpeed, now,
                                                       mVehicleData.curGear == GEAR_D);
    // 只叠加本模块限制，不能清掉前扫描或任何已有的 safety/零速。
    mReferPath.safety = mReferPath.safety || mPerceptionSafetyResult.emergency;
    mReferPath.desireSpeed = std::min<double>(mReferPath.desireSpeed, mPerceptionSafetyResult.speed_limit);
    if(mReferPath.safety) mReferPath.desireSpeed = 0;
    mPathPlanStatus.distance2Object = front_checked ? std::min<double>(mPathPlanStatus.distance2Object,
                                                                       mPerceptionSafetyResult.distance)
                                                    : mPerceptionSafetyResult.distance;
    int ultra_safe = 0;
    // ROS 本地缓存订阅参数更新，避免为独立超声停车条件每圈发一次 XML-RPC。
    ros::param::getCached("/ultra/status/safe", ultra_safe);
    if(ultra_safe == 1) mReferPath.desireSpeed = 0;

    const int reason = static_cast<int>(mPerceptionSafetyResult.reason);
    if(reason != mPerceptionSafetyLogReason || (reason != 0 && now - mPerceptionSafetyLogTime >= 1.0)) {
        printf("perception risk: reason=%d id=%d distance=%.2f ttc=%.2f limit=%.2f safety=%d front=%d prior=%d candidates=%zu axes=%zu ego=%.2f stop_distance=%.2f hits=%d immediate=%d\n",
               reason, mPerceptionSafetyResult.object_id, mPerceptionSafetyResult.distance,
               mPerceptionSafetyResult.ttc, mPerceptionSafetyResult.speed_limit, int(mReferPath.safety),
               int(front_safety), int(prior_safety),
               mPerceptionSafetyResult.candidates, mPerceptionSafetyResult.axis_tests, ego.speed,
               mPerceptionSafetyResult.emergency_distance, mPerceptionSafetyResult.emergency_hits,
               int(mPerceptionSafetyResult.immediate_confirmation));
        mPerceptionSafetyLogReason = reason;
        mPerceptionSafetyLogTime = now;
    }
}
