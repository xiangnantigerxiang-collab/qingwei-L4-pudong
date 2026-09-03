// 本文件由 path_plan_comply.cpp 原位包含，负责周期规划和参考路径生成。
// mReferPath 是规划内部的中间结果，最终发给 control 的是 mPlanPath。

void PathPlanComply::PathPlanProcess()
{
    // 10 Hz 周期主流程：
    // 1. 必要时生成作业路径；
    // 2. 在全局路径前向 80 点窗口内推进 mKeypoint；
    // 3. 根据停车点、任务类型和挡位计算本周期速度上限；
    // 4. 到达任务终点后清空任务路径。此函数本身不发布消息。
    printf("=====================================================\n");
    printf("任务状态: task execustatus: %d, tasktype: %d\n",
           mPathPlanStatus.taskExecuStatus, mTaskPlanData.taskType);
    // 1. 生成路径
    // printf("handle driving path\n");
    handleDrivingPath();
    // if (mDrivingPath.empty())
    // {
    //     printf("driving path is emppty\n");
    //     mKeypoint = 0;
    //     return;
    // }
    // 2.  更新最近点mKeyPoint索引

    mDesireSpeed = mTaskPlanData.desireSpeed;

    double x0 = mNavData.xAxis;
    double y0 = mNavData.yAxis;
    double h0 = mNavData.heading;
    double xt = mPathPlanStatus.stopX;
    double yt = mPathPlanStatus.stopY;
    double ht = mPathPlanStatus.stopAngle;
    if (!mDrivingPath.empty())
    {
        printf("update path info, path size: %d\n", mDrivingPath.size());
        UpdatePathInfo();
        if (mVehicleData.curGear == GEAR_D)
        {
            XYZ_COOR_S xyz = mDrivingPath.at(mKeypoint);
            x0 = xyz.x_axis;
            y0 = xyz.y_axis;
        }
    }
    // 根据与停止点的距离计算一个速度
    printf("计算limit速度, 更新完成状态\n");
    double speed = LimitSpeedByDistanceToStop(x0, y0, h0, xt, yt, ht);
    ////////////////计算一个限速/////////////////
    // 与任务要求速度进行比较
    printf("speed limit:%f, desire speed:%f\n", speed, mDesireSpeed);
    mDesireSpeed = std::min<double>(speed, mDesireSpeed);
    if (mVehicleData.curGear == GEAR_R) // 倒挡速度要求不大于0.5
        mDesireSpeed = std::min<double>(mDesireSpeed, 0.5);
    if (mVehicleData.curGear == GEAR_N)
    {
        printf("挡位: N, desire speed: 0\n");
        mDesireSpeed = 0.0;
    }

    // if (mPathPlanStatus.stopX == -1 && mPathPlanStatus.stopY == -1)
    // {
    //     printf("无停止点, desire speed: 0\n");
    //     mDesireSpeed = 0.0;
    // }

    if (mPathPlanStatus.taskExecuStatus == TASKFINISHED)
    {
        mDesireSpeed = 0.0;
        mKeypoint = 0;
        mStopIndex = 0;
        mTaskPlanData.pathList.clear();
        mTaskPlanData.taskType = NOTASK;
        // mPathPlanStatus.taskExecuStatus = NOTASK;
    }

    /////////////////////////////////////////////
    // 运行过程中挂钩异常，直接结束任务
    int hookstate = 0;
    ros::param::get("/canbus/hookstate", hookstate);
    // if (hookstate == 1 && mPathPlanStatus.curSpeed < 0.3)
    // {
    //     printf("hookstate == 1 and curspeed < 0.3 task finifshed\n");
    //     mDesireSpeed = 0.0;
    //     mPathPlanStatus.taskExecuStatus = TASKFINISHED;
    // }
}

void PathPlanComply::PublishPathPlanStatus(ros::Publisher &tPub)
{
    // if (mTaskPlanData.workMode != NOMALWORKINGMODE)
    //     mPathPlanStatus.taskExecuStatus = NOTASK;
    tPub.publish(mPathPlanStatus);
}

void PathPlanComply::PublishReferPath(ros::Publisher &tPub)
{
    // 从 mKeypoint 起抽取最多 20 个点；尾部不足时沿末端航向补点。
    // R 挡只做后向距离停车；D 挡依次执行旧轨迹复用、规划速度上限、
    // 路径包围盒碰撞筛选和三帧安全去抖。后段赋值可能覆盖前段速度，
    // 因而这里的执行顺序属于安全语义，不能随意调整。
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> heading;
    std::vector<XYZ_COOR_S> extend_path;
    std::vector<XYZ_COOR_S> xyz_list;
    //////////////////////
     if (mTaskPlanData.task_id == 4) // 测试分段限速
    {
        mPerception.objs.clear();
        mPerceptionFrontScan.objs.clear();
        mPerceptionBackScan.objs.clear();
    }
    /////////////////////
    if (mPathPlanStatus.taskExecuStatus == TASKFINISHED)
    {
        printf("任务结束\n");
        mReferPath.x.clear();
        mReferPath.y.clear();
        mReferPath.desireSpeed = 0.0;
        mReferPath.planspeed = 0.0;
        return;
    }

    int size = mDrivingPath.size(); // 全局路径，长度固定
    if (size < 20)
        return;

    XYZ_COOR_S xyz_last = mDrivingPath.at(mKeypoint);
    x.push_back(xyz_last.x_axis);
    y.push_back(xyz_last.y_axis);
    heading.push_back(xyz_last.heading);

    int LaneChangeSwitch = xyz_last.z_axis;

    // load path section from refernce path
    for (int i = mKeypoint + 1; i < size - 2; i += 2)
    {
        XYZ_COOR_S xyz_temp = mDrivingPath.at(i);

        double dx = xyz_temp.x_axis - xyz_last.x_axis;
        double dy = xyz_temp.y_axis - xyz_last.y_axis;
        double dist = hypot(dx, dy);

        if (dist < 0.3)
            continue;

        x.push_back(xyz_temp.x_axis);
        y.push_back(xyz_temp.y_axis);
        heading.push_back(xyz_temp.heading);

        xyz_last = xyz_temp;

        if (x.size() >= 20)
            break;
    }

    // generate extended path
    // 这里添加延长路径是为了精准停靠
    if (x.size() < 20)
    {
        XYZ_COOR_S xyz_temp = mDrivingPath.at(size - 2);
        // printf("last point: x:%f, y:%f, heading:%f\n", xyz_temp.x_axis, xyz_temp.y_axis, xyz_temp.heading);
        float head_extend = xyz_temp.heading;
         printf("xyz heading:%f\n", head_extend);
        if (mVehicleData.curGear == GEAR_R)
        {
            head_extend += 180.0;

            if (head_extend > 360.0)
                head_extend -= 360.0;
            // if (head_extend > 0.0)
            //     head_extend += 360.0;
        }
        printf("head extend is:%f\n", head_extend);
        extend_path = pubalgor.GenerateExtendedPath(
            xyz_temp.x_axis, xyz_temp.y_axis, head_extend, 30);

        size = extend_path.size();

        for (int i = 0; i < size; i += 6)
        {
            xyz_temp = extend_path.at(i);
            x.push_back(xyz_temp.x_axis);
            y.push_back(xyz_temp.y_axis);
            heading.push_back(head_extend);

            if (x.size() >= 20)
                break;
        }
    }

    // gear R
    if (mVehicleData.curGear == GEAR_R)
    {
        mReferPath.x = x;
        mReferPath.y = y;
        mReferPath.Path_Id = 20;
        mReferPath.safety = 0;
        mReferPath.planspeed = 1.0;
        mReferPath.desireSpeed = mDesireSpeed;

        double backdist = 100000.0;
        for (auto i : mPerceptionBackScan.objs)
        {
            double backx = mNavData.xAxis;
            double backy = mNavData.yAxis;
            double d = hypot(i.x - backx, i.y - backy);

            if (d < backdist)
                backdist = d;
        }
printf("R挡检测: objs数量=%d, backdist=%.3f\n", (int)mPerceptionBackScan.objs.size(), backdist);

        if (backdist < 2.5)
        {
            printf("R挡, 障碍物距离<0.2, 停车，障碍物距离: %f\n", backdist);
            mReferPath.safety = 1;
            mReferPath.planspeed = 0;
            mReferPath.desireSpeed = 0;
            ROS_ERROR("STOP");
        }

        tPub.publish(mReferPath);
        return;
    }

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
    double dh = 0.0;
    double dist = 0.0;
    double path_dist = 0.0;

    size = path_final_last.size();

    if (size > 20 && InitSafetyCheck == 1)
    {
        for (auto i : mPerception.objs)
        {
            path_dist = 100.0;
            // 计算障碍物与路径的最近距离
            for (auto j : path_final_last)
            {
                dx = i.x - j.x;
                dy = i.y - j.y;
                dist = hypot(dx, dy);

                if (dist < path_dist)
                    path_dist = dist;
            }

            dx = mNavData.xAxis - i.x;
            dy = mNavData.yAxis - i.y;
            dist = hypot(dx, dy);
            // 这里如果距离最近障碍物小于1m并且距离车base小于5m则认为有碰撞风险 车长3.45m，后轴中心到前雷达:2.41   车宽1.47m
            // if (path_dist < 1.0 && dist < 5.0)
            //     collision_check = 1;
            if (path_dist < 2.75)
                collision_check = 1;
        }

        dx = mNavData.xAxis - path_final_last[size - 1].x;
        dy = mNavData.yAxis - path_final_last[size - 1].y;
        dist = hypot(dx, dy); // 自车与终点的距离

        if (collision_check == 0 && dist > 10.0) // 如果上一帧路径无碰撞风险，且距离>10m则继续使用上一帧路径,并从当前位置裁剪
        {
            path_final = path_final_last;

            // get path cut from cur pose
            auto pathcut = path_final;
            path_dist = 10000.0;

            for (int i = 0; i < path_final.size(); i++)
            {
                dx = mNavData.xAxis - path_final[i].x;
                dy = mNavData.yAxis - path_final[i].y;
                dist = hypot(dx, dy);

                if (dist < path_dist)
                {
                    path_dist = dist;
                    size = i;
                }
            }

            pathcut.clear();
            for (int i = size; i < path_final.size(); i++)
            {
                pathcut.push_back(path_final[i]);
            }

            path_final = pathcut;
        }
    }

    path_final_last = path_final;

    // speed plan
    // 这里只进行了参数传递，没有进行计算
    //Speed_Planning speed_planning(&path_final, mPathPlanStatus.curSpeed, lidarobjs_global_);

    // float planspeed = speed_planning.SpeedCalculateInGuidePth(&path_final);
    double planspeed = 8.0;

    dist = Distance2PerceptionObjs(mNavData.xAxis, mNavData.yAxis);
    dist = std::max<double>(1.0, dist / 2.0);
    if (dist < 3 && mVehicleData.vehicleSpeed > 3.0)
    {
        planspeed = std::min<double>(planspeed, dist);
    }

    std::vector<float> x_final;
    std::vector<float> y_final;

    for (auto i : path_final)
    {
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
    if(mTaskPlanData.task_id == 4) // 测试分段限速
    {
        //double cur_lane_speed = mReferPath.desireSpeed;
        //double next_lane_speed = std::max(1.0,  mReferPath.desireSpeed - 1.5);
        //printf("cur_lane_speed = %f, next_lane_speed = %f\n", cur_lane_speed, next_lane_speed);
        double limit_speed = getLaneLimitSpeedTest(3.0, 1.5);
        mReferPath.desireSpeed = std::min(double(mReferPath.desireSpeed), limit_speed);
    }
    // printf("ref path id:%d, lane change switch:%d\n", mReferPath.Path_Id, LaneChangeSwitch);
    // LaneChangeSwitch = 1; // 绕障
    if (mReferPath.Path_Id == 5 || LaneChangeSwitch == 0)
    {
        mReferPath.x = x;
        mReferPath.y = y;
    }

    if (mPerception.objs.size() == 0)
    {
        tPub.publish(mReferPath);
        return;
    }
    // 下面进行碰撞检测
    //  collistion check precise
    Point VehicleInfo;
    VehicleInfo.x = mNavData.xAxis;
    VehicleInfo.y = mNavData.yAxis;
    VehicleInfo.heading = mNavData.heading;
    VehicleInfo.length = 3.6;
    // VehicleInfo.width = 2.0; //3.4m
    VehicleInfo.width = 3.4; // 3.4m

    //////////////////////////////////////////////////////////
    CollisionCheck ccheck;

    auto risk_objs = mPerception.objs;
    risk_objs.clear();
    // 对于每个障碍物与路径的每个点进行碰撞检测，找到最近的距离，如果距离小于0.5m则认为有碰撞风险
    for (int i = 0; i < mPerception.objs.size(); i++) // 障碍物与路径最近距离计算
    {
        Point ObjInfo;
        ObjInfo.x = mPerception.objs[i].x;
        ObjInfo.y = mPerception.objs[i].y;
        ObjInfo.heading = mPerception.objs[i].heading;
        ObjInfo.length = mPerception.objs[i].dy;
        ObjInfo.width = mPerception.objs[i].dx;
        auto ObjPoints = ccheck.GetRect(ObjInfo);

        dist = 10000.0;
        // 当前障碍物与路径进行碰撞检测
        for (int j = 0; j < mReferPath.x.size(); j++)
        {
            VehicleInfo.x = mReferPath.x[j];
            VehicleInfo.y = mReferPath.y[j];

            auto VehiclePoints = ccheck.GetRect(VehicleInfo);
            auto res = VehiclePoints;

            double d = ccheck.CheckRelation(VehiclePoints, ObjPoints, res);

            if (d < dist)
                dist = d;
        }
        // 将距离小于0.5m的障碍物认为有碰撞风险，加入风险列表
        if (dist < 0.1)
        {
            risk_objs.push_back(mPerception.objs[i]);
            risk_objs.back().height = dist; // 记录距离
        }
        // printf("obj i:%d, dist_to_ref:%.2f\n", i, dist);
    }
    history_risk_vec.erase(history_risk_vec.begin());
    history_risk_vec.emplace_back(risk_objs);
    int risk_num = 0;
    for (int i = 0; i < history_risk_vec.size(); i++)
    {
        if (history_risk_vec[i].size() > 0)
            risk_num++;
    }
    printf("risk vector: %d, %d, %d, %d\n", (int)history_risk_vec[0].size(), (int)history_risk_vec[1].size(),
           (int)history_risk_vec[2].size(), (int)history_risk_vec[3].size());
    if (risk_num == 0)
    {
        mPathPlanStatus.distance2Object = 100 + mPerception.objs.size();
        tPub.publish(mReferPath);
        return;
    }
    else
    {
        for (int i = history_risk_vec.size() - 1; i >= 0; i--)
        {
            if (history_risk_vec[i].size() > 0)
            {
                risk_objs = history_risk_vec[i];
                break;
            }
        }
    }

    // 这里对车辆周围90度范围内的障碍物进行过滤，认为这些障碍物对车辆有碰撞风险
    auto risk_objs_filter = risk_objs;
    risk_objs_filter.clear();
    std::vector<std::vector<robot::object>> history_risk_vec_filter(4, std::vector<robot::object>());
    for (auto i : risk_objs)
    {
        Point point;
        VehicleInfo.x = mNavData.xAxis;
        VehicleInfo.y = mNavData.yAxis;
        point.x = i.x;
        point.y = i.y;

        dh = PointDirectionToMe(point, VehicleInfo);

        if (fabs(dh) < 90.0)
            risk_objs_filter.push_back(i);
    }
    history_risk_vec_filter.erase(history_risk_vec_filter.begin());
    history_risk_vec_filter.emplace_back(risk_objs_filter);
    ////////////////////////记录历史碰撞
    int risk_num_filter = 0;
    for (int i = 0; i < history_risk_vec_filter.size(); i++)
    {
        if (history_risk_vec_filter[i].size() > 0)
            risk_num_filter++;
    }

    if (risk_num_filter == 0)
    {
        mPathPlanStatus.distance2Object = 200 + risk_objs.size();
        tPub.publish(mReferPath);
        return;
    }
    else
    {
        for (int i = history_risk_vec_filter.size() - 1; i >= 0; i--)
        {
            if (history_risk_vec_filter[i].size() > 0)
            {
                risk_objs_filter = history_risk_vec_filter[i];
                break;
            }
        }
    }
    ///////////////////////////////////////
    ////////////////////////////////////////////////////////////
    dist = 10000.0;
    // 这里计算最近障碍物的距离和方向，进行速度调整和安全判断
    double base_to_front = 2.3; // 车辆后轴中心到前雷达的距离
    int j = 0;
    double dist_to_refline = 10000;
    for (auto i : risk_objs_filter) // 障碍物与自车最近距离计算
    {
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
        VehicleInfo.width = 3.4; // 按照托盘尺寸计算
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
        if (ds < dist)
        {
            dist = ds;
            dh = dertah;
            dist_to_refline = i.height;
        }
    }
    printf("Collision dist = %f,ref_dist:%f, collision heading = %f, spd = %f\n",
           dist, dist_to_refline, dh, mReferPath.desireSpeed);
    double min_speed = 0.0;
    double spd = mReferPath.desireSpeed;
    
    if (dist_to_refline > 0.2) // 车道外，不会碰撞
    {
        min_speed = 0.5;
        // 根据与障碍物的距离计算速度
        mReferPath.desireSpeed = std::min<double>(spd, dist_to_refline) * 0.5 + mVehicleData.vehicleSpeed * 0.5;
    }
    else // 发生碰撞
    {
        // 根据与障碍物的距离计算速度
        if ((dist < 25.0) && (dist >= 10.0))
            mReferPath.desireSpeed = std::min<double>(spd, dist / 5.0) * 0.5 + mVehicleData.vehicleSpeed * 0.5;
        if (dist < 20.0)
            mReferPath.desireSpeed = std::min<double>(spd, dist / 6.0) * 0.5 + mVehicleData.vehicleSpeed * 0.5;
        if (dist < 15.0)
            mReferPath.desireSpeed = std::min<double>(spd, dist / 7.0) * 0.5 + mVehicleData.vehicleSpeed * 0.5;
        if (dist < 10.0)
            mReferPath.desireSpeed = std::min<double>(spd, dist / 8.0) * 0.5 + mVehicleData.vehicleSpeed * 0.5;
        if (dist < 6.5 && mVehicleData.vehicleSpeed > 1.5)
            mReferPath.safety = 1;
        if (dist < 4.5)
            mReferPath.safety = 1;
    }

    mPathPlanStatus.distance2Object = dist;

    // high frequncy safaty check filter

    if (mReferPath.safety == 1)
    {
        safety_check_counter_refer += 1;

        if (safety_check_counter_refer < 3)
            mReferPath.safety = 0;
        else
            safety_check_counter_refer = 10;
    }
    else
        safety_check_counter_refer = 0;

    if (mReferPath.safety == 1)
    {
        mReferPath.desireSpeed = 0.0;
        // ROS_ERROR("Collision dist = %f, dh = %f", dist, dh);
    } // else ROS_INFO("Collision dist = %f, dh = %f", dist, dh);

    tPub.publish(mReferPath);
}
