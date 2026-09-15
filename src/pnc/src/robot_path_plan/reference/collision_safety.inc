// 前向路径风险筛选、跨帧历史和碰撞限速。由 path_plan_reference.inc 包含。

void PathPlanComply::CheckForwardReferenceSafety() {
    // 空感知不推进历史窗；这与有感知但无风险的帧不同。
    if(mPerception.objs.size() == 0)
        return;

    std::vector<robot::object> risk_objs = CollectReferenceRiskObjects();
    if(!UpdateReferenceRiskHistory(risk_objs))
        return;

    double dh = 0.0;
    std::vector<robot::object> risk_objs_filter = FilterForwardRiskObjects(risk_objs, dh);
    if(risk_objs_filter.empty())
        return;

    ApplyReferenceCollisionSpeed(risk_objs_filter, dh);
}

std::vector<robot::object> PathPlanComply::CollectReferenceRiskObjects() {
    // 下面进行碰撞检测
    //  collistion check precise
    Point VehicleInfo;
    VehicleInfo.x = mNavData.xAxis;
    VehicleInfo.y = mNavData.yAxis;
    VehicleInfo.heading = mNavData.heading;
    VehicleInfo.length = 3.6;
    // VehicleInfo.width = 2.0; //3.4m
    VehicleInfo.width = 3.4;  // 3.4m

    //////////////////////////////////////////////////////////
    CollisionCheck ccheck;

    auto risk_objs = mPerception.objs;
    risk_objs.clear();
    // 每个目标与所有参考点的车辆矩形比较，保留最小间距；车宽按托盘 3.4 m 计算。
    for(int i = 0; i < mPerception.objs.size(); i++) {  // 障碍物与路径最近距离计算
        Point ObjInfo;
        ObjInfo.x = mPerception.objs[i].x;
        ObjInfo.y = mPerception.objs[i].y;
        ObjInfo.heading = mPerception.objs[i].heading;
        ObjInfo.length = mPerception.objs[i].dy;
        ObjInfo.width = mPerception.objs[i].dx;
        auto ObjPoints = ccheck.GetRect(ObjInfo);

        double dist = 10000.0;
        // 当前障碍物与路径进行碰撞检测
        for(int j = 0; j < mReferPath.x.size(); j++) {
            VehicleInfo.x = mReferPath.x[j];
            VehicleInfo.y = mReferPath.y[j];

            auto VehiclePoints = ccheck.GetRect(VehicleInfo);
            auto res = VehiclePoints;

            double d = ccheck.CheckRelation(VehiclePoints, ObjPoints, res);

            if(d < dist)
                dist = d;
        }
        // 现行阈值为 0.1 m；height 在此暂存路径间距，供后续速度规则使用。
        if(dist < 0.1) {
            risk_objs.push_back(mPerception.objs[i]);
            risk_objs.back().height = dist;  // 记录距离
        }
        // printf("obj i:%d, dist_to_ref:%.2f\n", i, dist);
    }
    return risk_objs;
}

bool PathPlanComply::UpdateReferenceRiskHistory(std::vector<robot::object> &risk_objs) {
    // 四帧中任一帧有风险就保留最近的非空结果，抑制单帧漏检带来的反复放行。
    history_risk_vec.erase(history_risk_vec.begin());
    history_risk_vec.emplace_back(risk_objs);
    int risk_num = 0;
    for(int i = 0; i < history_risk_vec.size(); i++) {
        if(history_risk_vec[i].size() > 0)
            risk_num++;
    }
    printf("risk vector: %d, %d, %d, %d\n", (int)history_risk_vec[0].size(), (int)history_risk_vec[1].size(),
           (int)history_risk_vec[2].size(), (int)history_risk_vec[3].size());
    if(risk_num == 0) {
        // 100+n 表示本帧有 n 个感知目标，但历史窗口内没有路径碰撞风险。
        mPathPlanStatus.distance2Object = 100 + mPerception.objs.size();
        return false;
    } else {
        for(int i = history_risk_vec.size() - 1; i >= 0; i--) {
            if(history_risk_vec[i].size() > 0) {
                risk_objs = history_risk_vec[i];
                break;
            }
        }
    }

    return true;
}

std::vector<robot::object> PathPlanComply::FilterForwardRiskObjects(const std::vector<robot::object> &risk_objs, double &dh) {
    Point VehicleInfo;
    VehicleInfo.heading = mNavData.heading;
    // 这里对车辆周围90度范围内的障碍物进行过滤，认为这些障碍物对车辆有碰撞风险
    auto risk_objs_filter = risk_objs;
    risk_objs_filter.clear();
    std::vector<std::vector<robot::object>> history_risk_vec_filter(4, std::vector<robot::object>());
    // 这里的 filter 窗口是每次重建的局部变量，不是上面的跨帧 history_risk_vec。
    for(auto i : risk_objs) {
        Point point;
        VehicleInfo.x = mNavData.xAxis;
        VehicleInfo.y = mNavData.yAxis;
        point.x = i.x;
        point.y = i.y;

        dh = PointDirectionToMe(point, VehicleInfo);

        if(fabs(dh) < 90.0)
            risk_objs_filter.push_back(i);
    }
    history_risk_vec_filter.erase(history_risk_vec_filter.begin());
    history_risk_vec_filter.emplace_back(risk_objs_filter);
    ////////////////////////记录历史碰撞
    int risk_num_filter = 0;
    for(int i = 0; i < history_risk_vec_filter.size(); i++) {
        if(history_risk_vec_filter[i].size() > 0)
            risk_num_filter++;
    }

    if(risk_num_filter == 0) {
        // 200+n 表示 n 个路径风险目标均被前向角度过滤；不推进安全去抖计数器。
        mPathPlanStatus.distance2Object = 200 + risk_objs.size();
        return risk_objs_filter;
    } else {
        for(int i = history_risk_vec_filter.size() - 1; i >= 0; i--) {
            if(history_risk_vec_filter[i].size() > 0) {
                risk_objs_filter = history_risk_vec_filter[i];
                break;
            }
        }
    }
    return risk_objs_filter;
}

void PathPlanComply::ApplyReferenceCollisionSpeed(const std::vector<robot::object> &risk_objs_filter, double dh) {
    ///////////////////////////////////////
    ////////////////////////////////////////////////////////////
    CollisionCheck ccheck;
    double dist = 10000.0;
    // 这里计算最近障碍物的距离和方向，进行速度调整和安全判断
    double base_to_front = 2.3;  // 车辆后轴中心到前雷达的距离
    int j = 0;
    double dist_to_refline = 10000;
    for(auto i : risk_objs_filter) {  // 障碍物与自车最近距离计算
        ////////////////////
        Point ObjInfo;
        ObjInfo.x = i.x;
        ObjInfo.y = i.y;
        ObjInfo.heading = i.heading;
        ObjInfo.length = i.dy;
        ObjInfo.width = i.dx;
        auto ObjPoints = ccheck.GetRect(ObjInfo);
        Point VehicleInfo;
        VehicleInfo.x = mNavData.xAxis;
        VehicleInfo.y = mNavData.yAxis;
        VehicleInfo.heading = mNavData.heading;
        VehicleInfo.length = 3.6;
        // VehicleInfo.width = 2.0;
        VehicleInfo.width = 3.4;  // 按照托盘尺寸计算
        // 计算车辆中心点
        double yaw = gis_utils::azimuthToYaw(VehicleInfo.heading);
        VehicleInfo.x = VehicleInfo.x + (base_to_front - VehicleInfo.length / 2) * cos(yaw);
        VehicleInfo.y = VehicleInfo.y + (base_to_front - VehicleInfo.length / 2) * sin(yaw);
        /////////////////////

        auto VehiclePoints = ccheck.GetRect(VehicleInfo);
        auto res = VehiclePoints;

        double ds = ccheck.CheckRelation(VehiclePoints, ObjPoints, res);
        double dertah = PointDirectionToMe(ObjInfo, VehicleInfo);
        printf("i obj:%f, ds:%f, dh:%f\n", j++, ds, dh);
        if(ds < dist) {
            dist = ds;
            dh = dertah;
            dist_to_refline = i.height;
        }
    }
    printf("Collision dist = %f,ref_dist:%f, collision heading = %f, spd = %f\n",
           dist, dist_to_refline, dh, mReferPath.desireSpeed);
    double min_speed = 0.0;
    double spd = mReferPath.desireSpeed;

    if(dist_to_refline > 0.2) {  // 车道外，不会碰撞
        min_speed = 0.5;
        // 根据与障碍物的距离计算速度
        mReferPath.desireSpeed = std::min<double>(spd, dist_to_refline) * 0.5 + mVehicleData.vehicleSpeed * 0.5;
    } else {  // 发生碰撞
        // 各距离条件依次覆盖，最后命中的分支生效，不能改成互斥 else-if。
        if((dist < 25.0) && (dist >= 10.0))
            mReferPath.desireSpeed = std::min<double>(spd, dist / 5.0) * 0.5 + mVehicleData.vehicleSpeed * 0.5;
        if(dist < 20.0)
            mReferPath.desireSpeed = std::min<double>(spd, dist / 6.0) * 0.5 + mVehicleData.vehicleSpeed * 0.5;
        if(dist < 15.0)
            mReferPath.desireSpeed = std::min<double>(spd, dist / 7.0) * 0.5 + mVehicleData.vehicleSpeed * 0.5;
        if(dist < 10.0)
            mReferPath.desireSpeed = std::min<double>(spd, dist / 8.0) * 0.5 + mVehicleData.vehicleSpeed * 0.5;
        if(dist < 6.5 && mVehicleData.vehicleSpeed > 1.5)
            mReferPath.safety = 1;
        if(dist < 4.5)
            mReferPath.safety = 1;
    }

    mPathPlanStatus.distance2Object = dist;

    // 连续三次有效碰撞检查才置 safety；早退帧不会进入这里，也不会清计数器。

    if(mReferPath.safety == 1) {
        safety_check_counter_refer += 1;

        if(safety_check_counter_refer < 3)
            mReferPath.safety = 0;
        else
            safety_check_counter_refer = 10;
    } else
        safety_check_counter_refer = 0;

    int ultra_safe = 0;
    // 保留当前读取位置：只有进入本阶段才消费 ultra 参数，前面的早退分支不读取。
    ros::param::get("/ultra/status/safe", ultra_safe);

    if(ultra_safe == 1) {
        mReferPath.desireSpeed = 0.0;
        ROS_INFO("ultra safe detect");
    }

    if(mReferPath.safety == 1) {
        mReferPath.desireSpeed = 0.0;
        // ROS_ERROR("Collision dist = %f, dh = %f", dist, dh);
    }  // else ROS_INFO("Collision dist = %f, dh = %f", dist, dh);
}
