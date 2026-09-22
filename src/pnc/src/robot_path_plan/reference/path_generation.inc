// 参考点采样、R 挡路径和前向路径生成。由 path_plan_reference.inc 包含。

void PathPlanComply::BuildReferencePathPoints(std::vector<float> &x, std::vector<float> &y,
                                              std::vector<float> &heading) {
    // D 挡保持原采样密度，向前取最多 60 点；其他挡位保留原 1.5 倍前视里程。
    // 剩余路线不足时只取到真实末端，不通过加密或外推凑齐点数。
    int size = mDrivingPath.size();
    if(FenceTaskRequired()) size = std::min(size, mStopIndex + 1);
    if(mKeypoint < 0 || mKeypoint >= size)
        return;

    const bool forward = mVehicleData.curGear == GEAR_D;
    const size_t forward_point_count = 60;
    if(forward) {
        // 三个数组各预留一次容量，避免延长轨迹时反复扩容和复制。
        x.reserve(forward_point_count);
        y.reserve(forward_point_count);
        heading.reserve(forward_point_count);
    }

    XYZ_COOR_S xyz_last = mDrivingPath.at(mKeypoint);
    x.push_back(xyz_last.x_axis);
    y.push_back(xyz_last.y_axis);
    heading.push_back(xyz_last.heading);

    double distance = 0.0;
    double lookahead = -1.0;
    bool reached_horizon = false;
    for(int i = mKeypoint + 1; i < size; ++i) {
        XYZ_COOR_S xyz_temp = mDrivingPath.at(i);
        if(!forward) {
            const auto &previous = mDrivingPath.at(i - 1);
            const double segment = hypot(xyz_temp.x_axis - previous.x_axis,
                                         xyz_temp.y_axis - previous.y_axis);
            if(lookahead > 0.0 && segment > 0.0 && distance + segment >= lookahead) {
                const double ratio = (lookahead - distance) / segment;
                x.push_back(previous.x_axis + ratio * (xyz_temp.x_axis - previous.x_axis));
                y.push_back(previous.y_axis + ratio * (xyz_temp.y_axis - previous.y_axis));
                heading.push_back(previous.heading);
                reached_horizon = true;
                break;
            }
            distance += segment;
        }
        if((i - mKeypoint) % 2 == 0)
            continue;

        double dx = xyz_temp.x_axis - xyz_last.x_axis;
        double dy = xyz_temp.y_axis - xyz_last.y_axis;
        double dist = hypot(dx, dy);

        if(dist < 0.3)
            continue;

        x.push_back(xyz_temp.x_axis);
        y.push_back(xyz_temp.y_axis);
        heading.push_back(xyz_temp.heading);

        xyz_last = xyz_temp;

        if(forward && x.size() >= forward_point_count) {
            reached_horizon = true;
            break;
        }
        if(!forward && x.size() == 20)
            lookahead = distance * 1.5;
    }

    const auto &endpoint = mDrivingPath[size - 1];
    if(!reached_horizon && hypot(endpoint.x_axis - x.back(), endpoint.y_axis - y.back()) > 1e-4) {
        x.push_back(endpoint.x_axis);
        y.push_back(endpoint.y_axis);
        heading.push_back(endpoint.heading);
    }

    // 控制侧至少接收 10 点；临近路线末端时补入路线内已走过的点供拟合，
    // 不再沿末端航向虚构延长线。最近点仍由控制器按车辆位置匹配。
    for(int i = mKeypoint - 1; i >= 0 && x.size() < 10; --i) {
        const auto &point = mDrivingPath.at(i);
        if(hypot(point.x_axis - x.front(), point.y_axis - y.front()) >= 0.3) {
            x.insert(x.begin(), point.x_axis);
            y.insert(y.begin(), point.y_axis);
            heading.insert(heading.begin(), point.heading);
        }
    }
}

void PathPlanComply::UpdateReverseReferencePath(const std::vector<float> &x, const std::vector<float> &y) {
    // 后向目标已转换到地图坐标，距离按后轴定位点计算；严格小于 2.5 m 才停车。
    mReferPath.x = x;
    mReferPath.y = y;
    mReferPath.Path_Id = 20;
    mReferPath.safety = 0;
    mReferPath.planspeed = 1.0;
    mReferPath.desireSpeed = mDesireSpeed;

    double backdist = 100000.0;
    for(auto i : mPerceptionBackScan.objs) {
        double backx = mNavData.xAxis;
        double backy = mNavData.yAxis;
        double d = hypot(i.x - backx, i.y - backy);

        if(d < backdist)
            backdist = d;
    }


    if(backdist < 2.5) {
        printf("R挡, 障碍物距离 < 2.5, 停车，障碍物距离: %f\n", backdist);
        mReferPath.safety = 1;
        mReferPath.planspeed = 0;
        mReferPath.desireSpeed = 0;
        ROS_ERROR("STOP");
    }
}

void PathPlanComply::UpdateForwardReferencePath(const std::vector<float> &x, const std::vector<float> &y,
                                                const std::vector<float> &heading) {
    // gear D
    lidarobjs_global_.clear();
    lidarobjs_global_ = Obj_Projecte_Map(mPerception, lidarobjs_global_);

    // path plan with lattice planner
    // printf("----plan lattice start\n");
    // path_final = LatticePlan(x, y, heading);
    // printf("----lattice planner size:%d\n", path_final.size());
    // if (path_final.empty())
    // {
    //     // mReferPath.x.clear();
    //     // mReferPath.y.clear();
    //     mReferPath.safety = 1;
    //     mReferPath.planspeed = 0;
    //     mReferPath.desireSpeed = 0;
    //     return;
    // }
    // path_final.erase(path_final.begin());

    (void)heading;  // 航向采样留给上方休眠的 lattice 调用，现役循迹只消费 x/y。
    ReusePreviousTrajectory();

    // speed plan
    // 这里只进行了参数传递，没有进行计算
    //Speed_Planning speed_planning(&path_final, mPathPlanStatus.curSpeed, lidarobjs_global_);

    // float planspeed = speed_planning.SpeedCalculateInGuidePth(&path_final);
    double planspeed = 8.0;

    if(!mPerceptionSafety.HasInput()) {
        double dist = Distance2PerceptionObjs(mNavData.xAxis, mNavData.yAxis);
        dist = std::max<double>(1.0, dist / 2.0);
        if(dist < 3 && mNavData.gpsSpeed > 3.0) {
            planspeed = std::min<double>(planspeed, dist);
        }
    }
    // 新链已按真实行驶区域/相对运动限速，不能再由历史目标的径向距离把路边车当成阻挡。

    std::vector<float> x_final;
    std::vector<float> y_final;

    for(auto i : path_final) {
        x_final.push_back(i.x);
        y_final.push_back(i.y);
    }

    mReferPath.x = x_final;
    mReferPath.y = y_final;
    mReferPath.Path_Id = 5;
    mReferPath.safety = 0;
    mReferPath.planspeed = planspeed;
    mReferPath.desireSpeed = std::min<double>(planspeed, mDesireSpeed);
    if(mTaskPlanData.task_id == 4) {  // 测试分段限速
        //double cur_lane_speed = mReferPath.desireSpeed;
        //double next_lane_speed = std::max(1.0,  mReferPath.desireSpeed - 1.5);
        //printf("cur_lane_speed = %f, next_lane_speed = %f\n", cur_lane_speed, next_lane_speed);
        double limit_speed = getLaneLimitSpeedTest(3.0, 1.5);
        mReferPath.desireSpeed = std::min(double(mReferPath.desireSpeed), limit_speed);
    }
    // 现役前向路径固定使用采样参考线；不再读取CSV变道标志。
    mReferPath.x = x;
    mReferPath.y = y;
}

void PathPlanComply::ReusePreviousTrajectory() {
    // 旧轨迹超过 20 点且无碰撞时，才可能从当前位姿裁剪复用。
    // 即便不复用，函数末尾也要更新 path_final_last，保持下一周期的比较基准。
    // check last trajectory collision
    int collision_check = 0;
    double x0 = 0.0;
    double y0 = 0.0;
    double h0 = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
    double h1 = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    double dist = 0.0;
    double path_dist = 0.0;

    int size = path_final_last.size();

    if(size > 20) {
        for(auto i : mPerception.objs) {
            path_dist = 100.0;
            // 计算障碍物与路径的最近距离
            for(auto j : path_final_last) {
                dx = i.x - j.x;
                dy = i.y - j.y;
                dist = hypot(dx, dy);

                if(dist < path_dist)
                    path_dist = dist;
            }

            dx = mNavData.xAxis - i.x;
            dy = mNavData.yAxis - i.y;
            dist = hypot(dx, dy);
            // 这里如果距离最近障碍物小于1m并且距离车base小于5m则认为有碰撞风险 车长3.45m，后轴中心到前雷达:2.41   车宽1.47m
            // if (path_dist < 1.0 && dist < 5.0)
            //     collision_check = 1;
            if(path_dist < 2.75)
                collision_check = 1;
        }

        dx = mNavData.xAxis - path_final_last[size - 1].x;
        dy = mNavData.yAxis - path_final_last[size - 1].y;
        dist = hypot(dx, dy);  // 自车与终点的距离

        if(collision_check == 0 && dist > 10.0) {  // 如果上一帧路径无碰撞风险，且距离>10m则继续使用上一帧路径,并从当前位置裁剪
            path_final = path_final_last;

            // get path cut from cur pose
            auto pathcut = path_final;
            path_dist = 10000.0;

            for(int i = 0; i < path_final.size(); i++) {
                dx = mNavData.xAxis - path_final[i].x;
                dy = mNavData.yAxis - path_final[i].y;
                dist = hypot(dx, dy);

                if(dist < path_dist) {
                    path_dist = dist;
                    size = i;
                }
            }

            pathcut.clear();
            for(int i = size; i < path_final.size(); i++) {
                pathcut.push_back(path_final[i]);
            }

            path_final = pathcut;
        }
    }

    path_final_last = path_final;
}
