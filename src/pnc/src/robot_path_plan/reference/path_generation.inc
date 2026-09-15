// 参考点采样、R 挡路径和前向路径生成。由 path_plan_reference.inc 包含。

void PathPlanComply::BuildReferencePathPoints(std::vector<float> &x, std::vector<float> &y,
                                              std::vector<float> &heading, int &LaneChangeSwitch) {
    // 从最近点向前每隔两点采样，点距不足 0.3 m 则跳过，最多保留 20 点。
    int size = mDrivingPath.size();
    std::vector<XYZ_COOR_S> extend_path;

    XYZ_COOR_S xyz_last = mDrivingPath.at(mKeypoint);
    x.push_back(xyz_last.x_axis);
    y.push_back(xyz_last.y_axis);
    heading.push_back(xyz_last.heading);

    LaneChangeSwitch = xyz_last.z_axis;

    // load path section from refernce path
    for(int i = mKeypoint + 1; i < size - 2; i += 2) {
        XYZ_COOR_S xyz_temp = mDrivingPath.at(i);

        double dx = xyz_temp.x_axis - xyz_last.x_axis;
        double dy = xyz_temp.y_axis - xyz_last.y_axis;
        double dist = hypot(dx, dy);

        if(dist < 0.3)
            continue;

        x.push_back(xyz_temp.x_axis);
        y.push_back(xyz_temp.y_axis);
        heading.push_back(xyz_temp.heading);

        xyz_last = xyz_temp;

        if(x.size() >= 20)
            break;
    }

    // generate extended path
    // 这里添加延长路径是为了精准停靠
    if(x.size() < 20) {
        XYZ_COOR_S xyz_temp = mDrivingPath.at(size - 2);
        // printf("last point: x:%f, y:%f, heading:%f\n", xyz_temp.x_axis, xyz_temp.y_axis, xyz_temp.heading);
        float head_extend = xyz_temp.heading;
        printf("xyz heading:%f\n", head_extend);
        if(mVehicleData.curGear == GEAR_R) {
            head_extend += 180.0;

            if(head_extend > 360.0)
                head_extend -= 360.0;
            // if (head_extend > 0.0)
            //     head_extend += 360.0;
        }
        printf("head extend is:%f\n", head_extend);
        extend_path = pubalgor.GenerateExtendedPath(
            xyz_temp.x_axis, xyz_temp.y_axis, head_extend, 30);

        size = extend_path.size();

        for(int i = 0; i < size; i += 6) {
            xyz_temp = extend_path.at(i);
            x.push_back(xyz_temp.x_axis);
            y.push_back(xyz_temp.y_axis);
            heading.push_back(head_extend);

            if(x.size() >= 20)
                break;
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

    printf("R挡检测: objs数量=%d, backdist=%.3f\n", (int)mPerceptionBackScan.objs.size(), backdist);

    if(backdist < 2.5) {
        printf("R挡, 障碍物距离 < 2.5, 停车，障碍物距离: %f\n", backdist);
        mReferPath.safety = 1;
        mReferPath.planspeed = 0;
        mReferPath.desireSpeed = 0;
        ROS_ERROR("STOP");
    }
}

void PathPlanComply::UpdateForwardReferencePath(const std::vector<float> &x, const std::vector<float> &y,
                                                const std::vector<float> &heading, int LaneChangeSwitch) {
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

    double dist = Distance2PerceptionObjs(mNavData.xAxis, mNavData.yAxis);
    dist = std::max<double>(1.0, dist / 2.0);
    if(dist < 3 && mVehicleData.vehicleSpeed > 3.0) {
        planspeed = std::min<double>(planspeed, dist);
    }

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
    printf("=======task_id:%d\n", mTaskPlanData.task_id);
    if(mTaskPlanData.task_id == 4) {  // 测试分段限速
        //double cur_lane_speed = mReferPath.desireSpeed;
        //double next_lane_speed = std::max(1.0,  mReferPath.desireSpeed - 1.5);
        //printf("cur_lane_speed = %f, next_lane_speed = %f\n", cur_lane_speed, next_lane_speed);
        double limit_speed = getLaneLimitSpeedTest(3.0, 1.5);
        mReferPath.desireSpeed = std::min(double(mReferPath.desireSpeed), limit_speed);
    }
    // printf("ref path id:%d, lane change switch:%d\n", mReferPath.Path_Id, LaneChangeSwitch);
    // LaneChangeSwitch = 1; // 绕障
    if(mReferPath.Path_Id == 5 || LaneChangeSwitch == 0) {
        mReferPath.x = x;
        mReferPath.y = y;
    }
}

void PathPlanComply::ReusePreviousTrajectory() {
    // 旧轨迹超过 20 点、起步检查已通过且无碰撞时，才可能从当前位姿裁剪复用。
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

    if(size > 20 && InitSafetyCheck == 1) {
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
