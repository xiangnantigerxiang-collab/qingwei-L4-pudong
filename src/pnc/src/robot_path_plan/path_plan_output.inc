// 本文件由 path_plan_comply.cpp 原位包含，负责最终输出和路径跟踪辅助。
// PublishPlanPath 中靠后的条件拥有更高覆盖优先级。

void PathPlanComply::PublishPlanPath(ros::Publisher &tPub1, ros::Publisher &tPub2)
{
    // 最终输出覆盖顺序：
    // 手动模式/短路径提前返回 -> 起步周边观察 -> 复制参考路径 ->
    // 加速平滑 -> 急停/断网 -> 声光 -> 等待区/T 路口 -> 云端暂停。
    // 修改任一停车条件时，必须继续检查它后面的 desireSpeed/safety 赋值。
    printf("desireSpeed in plan path begin: %f\n", mPlanPath.desireSpeed);
    // 启动时，检查四周是否有障碍物
    bool is_around_unsafe = checkAroundObstacle();
    bool is_T_unsafe = checkTJunctionObstacle();
    bool is_in_waiting = checkIsInWaiting();
    if (mVehicleData.controlPanelState == 0)
    {
        printf("==========人工驾驶中==========\n");
        mPlanPath.x.clear();
        mPlanPath.y.clear();
        mPlanPath.desireSpeed = 0.0;
        tPub1.publish(mPlanPath);
        InitSafetyCheck = 0;
        ros::param::set("/canbus/light", 0);
        // int around_horn_cmd = 0;
        //  ros::param::get("/canbus/light", around_horn_cmd);
        //  if (around_horn_cmd == 7)
        //  {
        //      ros::param::set("/canbus/light", 0);
        //  }
        return;
    }
    if (mReferPath.x.size() < 5)
    {
        ROS_ERROR("publish plan path size less 5.");
        mPlanPath.x.clear();
        mPlanPath.y.clear();
        mPlanPath.desireSpeed = 0.0;
        tPub1.publish(mPlanPath);
        InitSafetyCheck = 0;
        ros::param::set("/canbus/light", 0);
        return;
    }
    // 起步观察
    printf("IniSafetyCheck: %d\n", InitSafetyCheck);

    if (InitSafetyCheck == 0)
    {
        InitSafetyCheck = 1;
        mPlanPath.safety = 1;
        if (is_around_unsafe)
        {
            ROS_WARN("周围有障碍物.");
            // printf("四周不安全\n");
            InitSafetyCheck = 0;
        }
        // T路口观察
        if (is_T_unsafe)
        {
            ROS_WARN("T路口有障碍物");
            // printf("T路口不安全\n");
            InitSafetyCheck = 0;
        }
    }

    // 速度规划
    mPlanPath.x = mReferPath.x;
    mPlanPath.y = mReferPath.y;
    mPlanPath.Path_Id = mReferPath.Path_Id;
    mPlanPath.safety = mReferPath.safety;
    mPlanPath.desireSpeed = mReferPath.desireSpeed;
    mPlanPath.planspeed = mReferPath.planspeed;

    double spd = mVehicleData.vehicleSpeed;
    double acc = mPlanPath.desireSpeed - mVehicleData.vehicleSpeed;
    // 这里的代码是速度的加速跟踪
    if (acc > 0.5)
    // if (true)
    {
        if (mVehicleData.linkPallet && mVehicleData.hookStatus != ACTUATOR_UP_END) // 挂托盘，平滑过渡
        {
            mPlanPath.desireSpeed = 0.1 * mPlanPath.desireSpeed + 0.9 * mVehicleData.vehicleSpeed;
        }
        else
        {
            mPlanPath.desireSpeed = 0.3 * mPlanPath.desireSpeed + 0.7 * mVehicleData.vehicleSpeed;
        }
    }
    // emergency pause and recovery
    if (emergencyStop == 1)
    {
        mPlanPath.safety = 1;
        mPlanPath.desireSpeed = 0.0;
    }
    printf("desireSpeed in plan path end: %f\n", mPlanPath.desireSpeed);
    // perception & navigation alive check
    // double pGap = sysTime.now - sysTime.msgPerception;
    // double nGap = sysTime.now - sysTime.msgNavigation;

    // if(pGap > 1.0) {
    //     mPlanPath.safety = 1;
    //     mPlanPath.desireSpeed = 0.0;
    //     ROS_WARN("Percetion node offline !");
    // }

    // if(nGap > 1.0) {
    //     mPlanPath.safety = 1;
    //     mPlanPath.desireSpeed = 0.0;
    //     ROS_WARN("Localization node offline !");
    // }

    // location status check
    // 定位异常，则立即停车
    // if (mNavData.localization_status != 0)
    // {
    //     mPlanPath.desireSpeed = 0.0;
    //     ROS_WARN("GPS signal is weak !");
    // }

    // network check
    int network_status = 0;
    ros::param::get("/robot/planning/netcheck", network_status);
    // 网络异常，则立即停车
    if (network_status != 0)
    {
        mPlanPath.desireSpeed = 0.0;
        ROS_WARN("Network Disconnected !");
    }

    // sound & light command publish
    int HornCmd = 0;
    if (mPathPlanStatus.curSpeed > 0.1)
    {
        sysTime.vehicleStop = ros::Time::now().toSec();
    }
    else
        sysTime.vehicleRun = ros::Time::now().toSec();

    double stime = sysTime.now - sysTime.vehicleStop;
    double rtime = sysTime.now - sysTime.vehicleRun;
    double ttime = sysTime.now - sysTime.taskStart;
    // 鸣笛逻辑
    if (stime > 3.0) // 起步鸣笛
        HornFlag = 1;

    if (HornFlag == 1 && rtime < 1.0 && stime < 1.0)
        HornCmd = 1;
    if (ttime < 1.0 && stime < 1.0)
        HornCmd = 1;
    if (rtime > 1.0) // 停车>1s，不鸣笛
        HornFlag = 0;

    // 灯光逻辑
    int LightCmd = 0;
    if (mVehicleData.curGear == GEAR_D)
    {
        int n = std::min<int>(10, mReferPath.x.size() - 1);

        Point me;
        Point point;
        me.x = mNavData.xAxis;
        me.y = mNavData.yAxis;
        me.heading = mNavData.heading;
        point.x = mReferPath.x[n];
        point.y = mReferPath.y[n];

        mPathPlanStatus.turning = "none";

        double dh = PointDirectionToMe(point, me);
        if (dh > 6.0) // 目标点在右侧超过10度
        {
            LightCmd = 3; // 开启右转向灯
            mPathPlanStatus.turning = "left";
        }

        if (dh < -6.0) // 目标点在左侧超过10度
        {
            LightCmd = 4; // 开启左转向灯
            mPathPlanStatus.turning = "right";
        }
    }

    if (mVehicleData.curGear == GEAR_R)
    {
        LightCmd = 5; // 开启倒车灯
    }

    if (mPlanPath.safety == 1)
    {
        LightCmd = 8;
        // int sum = sumVec(history_unsafe);
    }
    if (InitSafetyCheck == 0)
    {
        LightCmd = 7; // 报警
        mPlanPath.desireSpeed = 0.0;
    }
    else
    {
        if (checkIsInWaiting() && is_T_unsafe)
        {
            printf("车辆在等待区, 检测周围障碍物\n");
            LightCmd = 7; // 报警
            mPlanPath.desireSpeed = 0.0;
        }
    }
    if (command_state == 1) // 暂停
    {
        mPlanPath.desireSpeed = 0.0;
    }
    printf("end speedlimit: %f\n", mPlanPath.desireSpeed);
    // param set & msg pub
    ros::param::set("/canbus/light", LightCmd); // 灯光和起步周边障碍物报警再canbus用到了
    ros::param::set("/canbus/horn", HornCmd);   // 鸣笛也没有用到
    tPub1.publish(mPlanPath);
    tPub2.publish(mSoundLightData); // 这个没有用到
}

void PathPlanComply::pubLatticeTrajs(ros::Publisher &tPub)
{
    // draw
    visualization_msgs::MarkerArray traj_cluster;
    for (int i = 0; i < lattice_trajs_.size(); i++)
    {
        auto traj = lattice_trajs_[i];
        visualization_msgs::Marker traj_marker;
        traj_marker.header.frame_id = "robot";
        traj_marker.header.stamp = ros::Time::now();
        traj_marker.ns = "lattice_traj";
        traj_marker.id = i;
        traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
        traj_marker.action = visualization_msgs::Marker::ADD;
        traj_marker.scale.x = 0.05;
        traj_marker.color.a = 1.0;
        traj_marker.color.r = 1.0 - 0.2 * i;
        traj_marker.color.g = 0.2 * i;
        traj_marker.color.b = 0.0;
        for (auto point : traj)
        {
            geometry_msgs::Point p;
            p.x = point.path_point.x();
            p.y = point.path_point.y();
            p.z = 0.0;
            traj_marker.points.push_back(p);
        }
        traj_cluster.markers.push_back(traj_marker);
    }
    tPub.publish(traj_cluster);
}

bool PathPlanComply::checkTJunctionObstacle()
{
    std::vector<Dot> vertexes;
    Dot vertex = {-4.83, 18.3};
    vertexes.push_back(vertex);

    vertex = {0.22, 20};
    vertexes.push_back(vertex);

    vertex = {12.3, -17.8};
    vertexes.push_back(vertex);

    vertex = {7.2, -19.5};
    vertexes.push_back(vertex);
    bool is_unsafe = false;
    for (auto i : mPerception.objs)
    {
        Dot objpoint = {i.x, i.y};
        if (pointInRectangle(vertexes, objpoint) != 0)
        {
            // ROS_WARN("20m Object detect in lane");
            is_unsafe = true;
            break;
        }
    }
    unsafe_vec.erase(unsafe_vec.begin());
    unsafe_vec.push_back(is_unsafe);
    int sum = 0;
    printf("T unsafe: ");
    for (int i = 0; i < unsafe_vec.size(); i++)
    {
        printf("%d ", int(unsafe_vec[i]));
        sum += unsafe_vec[i];
    }
    printf(" %d\n", sum);
    if (sum > 0) // 连续4帧有一帧有障碍物，则认为有危险
    {
        return true;
    }
    return false;
}

bool PathPlanComply::checkAroundObstacle()
{
    // 自车位置
    double x = mNavData.xAxis;
    double y = mNavData.yAxis;
    double h = mNavData.heading;

    auto point0 = local2global2(x, y, h, 2.3, 0.73, 0);   // 左前角
    auto point1 = local2global2(x, y, h, 2.3, -0.73, 0);  // 右前角
    auto point2 = local2global2(x, y, h, -0.8, 0.73, 0);  // 左后角
    auto point3 = local2global2(x, y, h, -0.8, -0.73, 0); // 右后角
    GeoPoint point0_geo(point0.x, point0.y);
    GeoPoint point1_geo(point1.x, point1.y);
    GeoPoint point2_geo(point2.x, point2.y);
    GeoPoint point3_geo(point3.x, point3.y);
    GeoLine line0(point0_geo, point1_geo); // 前
    GeoLine line1(point1_geo, point3_geo); // 右
    GeoLine line2(point2_geo, point3_geo); // 后
    GeoLine line3(point0_geo, point2_geo); // 左

    double dist0 = Distance2PerceptionObjs(line0);
    double dist1 = Distance2PerceptionObjs(line1);
    double dist2 = Distance2PerceptionObjs(line2);
    double dist3 = Distance2PerceptionObjs(line3);

    double safe_dist = 1.7 - 0.75;
    if (int(mVehicleData.linkPallet) == 1) // 连接托盘
    {
        safe_dist = 2.2 - 0.75;
    }

    bool unsafe = (dist0 < safe_dist || dist1 < safe_dist || dist2 < safe_dist || dist3 < safe_dist);

    history_unsafe.erase(history_unsafe.begin());
    history_unsafe.push_back(unsafe);
    int sum = 0;
    printf("safe_dist:%.2f, 前侧 = %.2f, 右侧 = %.2f, 后侧 = %.2f, 左侧 = %.2f\n", safe_dist, dist0, dist1, dist2, dist3);
    printf("Around_unsafe: ");
    for (int i = 0; i < history_unsafe.size(); i++)
    {
        printf("%d ", int(history_unsafe[i]));
        if (history_unsafe[i])
            sum += 1;
    }
    printf(" %d\n", sum);
    // if(history_unsafe.back() == true)
    // {
    //     return true;
    // }
    return sum > 5;
}

bool PathPlanComply::checkIsInWaiting()
{
    GeoPolygon waiting_polygon;
    waiting_polygon.outer().push_back(GeoPoint(1.0, 4.0));
    waiting_polygon.outer().push_back(GeoPoint(4.1, -4.7));
    waiting_polygon.outer().push_back(GeoPoint(-0.4, -6.13));
    waiting_polygon.outer().push_back(GeoPoint(-3.37, 2.45));
    GeoPoint cur_point(mNavData.xAxis, mNavData.yAxis);
    return bg::within(cur_point, waiting_polygon);
}

float PathPlanComply::distan2Line(GeoPoint point, GeoLine line)
{
    double d = bg::distance(point, line);
    return d;
}

std::vector<XYZ_COOR_S> PathPlanComply::LoadPathFile(string tPath)
{
    std::vector<XYZ_COOR_S> vector_list;
    std::vector<XYZ_COOR_S> incsv;
    float distance_temp = 0;
    std::string file_dir = "";
    ros::param::get("path_dir", file_dir);
    XYZ_COOR_S intp;

    FILE *fp;
    std::string path_dir = file_dir + tPath + ".csv";
    ROS_INFO("path_dir:%s", path_dir.c_str());
    fp = fopen(path_dir.c_str(), "r");
    if (fp == NULL)
    {
        printf("路径文件打开失败: %s\n", path_dir.c_str());
        return vector_list;
    }

    while (!feof(fp))
    {
        fscanf(fp, "%f,%f,%f,%f", &intp.x_axis, &intp.y_axis, &intp.heading, &intp.z_axis);
        incsv.push_back(intp);
    }
    fclose(fp);
    if (incsv.empty())
    {
        printf("incsv is empty\n");
        return vector_list;
    }
    XYZ_COOR_S intp_last = incsv.at(0);
    for (auto i : incsv)
    {
        XYZ_COOR_S xyz_temp;
        xyz_temp.heading = i.heading;
        xyz_temp.x_axis = i.x_axis;
        xyz_temp.y_axis = i.y_axis;
        xyz_temp.z_axis = i.z_axis;
        xyz_temp.velocity = 5;
        double dx = xyz_temp.x_axis - intp_last.x_axis;
        double dy = xyz_temp.y_axis - intp_last.y_axis;
        xyz_temp.p2pDistance = hypot(dx, dy);
        distance_temp += xyz_temp.p2pDistance;
        xyz_temp.dist_origin = distance_temp;
        intp_last = xyz_temp;
        vector_list.push_back(xyz_temp);
    }

    return vector_list;
}

float PathPlanComply::CalcuParkPointTangentDistance(
    float tCurX, float tCurY, float tTargetX, float tTargetY, float tTargetAngle)
{
    float angle_temp = tTargetAngle + 90.0;
    XYZ_COOR_S xyz_temp;

    if (angle_temp >= 360)
        angle_temp -= 360;

    xyz_temp = pubalgor.CaculateCrossPoint_P(tTargetX, tTargetY, angle_temp, tCurX, tCurY, tTargetAngle);

    double dx = xyz_temp.x_axis - tCurX;
    double dy = xyz_temp.y_axis - tCurY;

    return hypot(dx, dy);
}

float PathPlanComply::LimitSpeedByDistanceToStop(
    float tCurX, float tCurY, float tCurAngle,
    float tTargetX, float tTargetY, float tTargetAngle)
{
    // 速度上限按任务类型分层计算：
    // - DOACTION：根据挂钩反馈结束动作任务；
    // - ADAPTIVEPARK/ADAPTIVEHOOK：使用各自的停车距离与完成条件；
    // - 普通行驶：必要时先停靠占用中的飞机位，再处理最终停车点。
    // 返回值还会在 PathPlanProcess 中与任务期望速度取最小值。
    if (mTaskPlanData.taskType == DOACTION)
    {
        if (mTaskPlanData.hookCmd == HOOKOPERATION)
        {
            printf("正在hook中....\n");
            if (mVehicleData.hookStatus == ACTUATOR_UP_END) // canbus 状态机判销至顶端(带内持续1秒)
            {
                printf("hook到位, task finished\n");
                mPathPlanStatus.taskExecuStatus = TASKFINISHED;
            }
        }
        if (mTaskPlanData.hookCmd == DECOUPLING)
        {
            printf("正在decoupling中....\n");
            if (mVehicleData.hookStatus == ACTUATOR_DOWN_END) // canbus 状态机判销退到底(带内持续1秒)
            {
                printf("hook open, task finished\n");
                mPathPlanStatus.taskExecuStatus = TASKFINISHED;
            }
        }
        if (mTaskPlanData.stopX == -1 && mTaskPlanData.stopY == -1)
        {
            printf("无停止点, task finished\n");
            mPathPlanStatus.taskExecuStatus = TASKFINISHED;
        }
        return 0.0;
    }
    else
    {
        double dx = tTargetX - tCurX;
        double dy = tTargetY - tCurY;
        // remain_distance_ = hypot(dx, dy);
        if (mStopIndex < 0 || mStopIndex >= mDrivingPath.size() || mKeypoint < 0 || mKeypoint >= mDrivingPath.size())
        {
            printf("Invalid stop index or keypoint index. stopIndex: %d, keypoint: %d\n", mStopIndex, mKeypoint);
            return 0.0;
        }
        
        // 计算剩余距离
        remain_distance_ = mDrivingPath.at(mStopIndex).dist_origin - mDrivingPath.at(mKeypoint).dist_origin;
        printf("remain_distance_: %f, key_index:%d, stop_index:%d\n", remain_distance_, mKeypoint, mStopIndex);
        mPathPlanStatus.distance2Stop = remain_distance_;
        if (mTaskPlanData.taskType == ADAPTIVEPARK)
        {
            printf("倒车入库中...\n");
            Point point;
            Point VehicleInfo;

            VehicleInfo.x = tCurX;
            VehicleInfo.y = tCurY;
            VehicleInfo.heading = tCurAngle;
            point.x = tTargetX;
            point.y = tTargetY;

            double dh = PointDirectionToMe(point, VehicleInfo);

            if (fabs(dh) < 45.0)
            {
                printf("停车点在车前45度内\n");
                remain_distance_ = 0.0;
            }

            if (remain_distance_ < 0.2)
            {
                remain_distance_ = 0.0;
                printf("倒车入库结束 task finished\n");
                mPathPlanStatus.taskExecuStatus = TASKFINISHED;
            }
            return std::min<double>(remain_distance_, 0.6);
        }
        if (mTaskPlanData.taskType == ADAPTIVEHOOK)
        {
            printf("挂钩循迹中...\n");
            remain_distance_ = mHookPos.center_distance;
            printf("hook, remain_distance_ %f\n", remain_distance_);
    

            if (remain_distance_ < 1.5)
            {
            // 仅当车辆在指定矩形区域内时才检测托盘挂钩
//                std::vector<Dot> hook_rect;
//                hook_rect.push_back({-22.1, -17.9});
//                hook_rect.push_back({-22.1, -19.957});
//                hook_rect.push_back({-20.1, -19.957});
//                hook_rect.push_back({-20.1, -17.9});
//                Dot cur_pos = {mNavData.xAxis, mNavData.yAxis};
//                bool in_rect = (pointInRectangle(hook_rect, cur_pos) != 0);
//                printf("x:%f, y:%f\n", mNavData.xAxis, mNavData.yAxis);
//                printf("*******in_rect %d\n", in_rect);
//                if (in_rect && mVehicleData.linkPallet) // 在矩形框内且检测到托盘，切换到动作任务
            
                if (mVehicleData.linkPallet) // 检测到托盘，切换到动作任务
                {
                    printf("link hook到位, task finished\n");
                    mPathPlanStatus.taskExecuStatus = TASKFINISHED;
                    ROS_INFO("[Plan][info:][link pallet]");
                    return 0;
                }
                
		if(PalletType == 0) {
		    if(remain_distance_ < 0.3) backdist_flag = 1;
		    if(remain_distance_ > 0.5 && backdist_flag == 1) return 0;
                }

                return std::max(remain_distance_ / 2.0, 0.1);//6.0
            }else backdist_flag = 0;

            return 0.5;
        }
    }
    int stop_index = mStopIndex;
        if (mTaskPlanData.task_id == go_task_id_ || mTaskPlanData.task_id == back_task_id_)
        {
            auto port = findNextAirCraftParkingPort();
            printf("next port is:%d, parking:%d\n", port.id, port.parking);
            if (int(port.parking) == 1) // 入位中
            {
                stop_index =  port.go_stop_index == 0 ? port.back_stop_index : port.go_stop_index;
            }
        }
        remain_distance_ = mDrivingPath.at(stop_index).dist_origin - mDrivingPath.at(mKeypoint).dist_origin;
        printf("<<<<dest stop index: %d, airport stop index: %d>>>>>\n", mStopIndex, stop_index);
    if (mVehicleData.curGear == GEAR_N)
    {
        printf("挡位:N, 停车\n");
        return 0;
    }
    
    if (mVehicleData.curGear == GEAR_D)
    {
        printf("挡位:D, 路径跟踪\n");
        
        
         if (remain_distance_ < 0.5) // 距离终点还剩0.5开始刹停
        {
            if (stop_index == mStopIndex)
            {
             printf("离终点小于0.5, task_finished\n");
                remain_distance_ = 0.0;
                mPathPlanStatus.taskExecuStatus = TASKFINISHED;
                return 0.0;
                
            }
            else
            {
                printf("达到停止线停车点\n");
                return 0.0;
            }
        }
        else
        {
            if (remain_distance_ > 10.0) // 距离大于6m,按6m/s返回(也就是说会大于20km/h)
                return remain_distance_;
            else
                return std::max<double>(remain_distance_ / 6.0, 0.3);
        }
    }
    else
    {
        printf("挡位:%d\n", mVehicleData.curGear);
    }
    printf("return 0\n");
    return 0.0;
}

void PathPlanComply::UpdatePathInfo()
{
    // 只在当前关键点之后的 80 点窗口搜索最近点，保证索引单调向前，
    // 避免闭环或相邻道路上几何接近的旧点把车辆进度拉回。
    int keyPoint = mKeypoint;
    int size = mDrivingPath.size();

    XYZ_COOR_S xyz_temp;

    float min = 100.0;
    float distance_temp = 0.0;
    if (mDrivingPath.size() == 0)
        return;
    if (mKeypoint > (size - 1))
        return;
    // printf("------------keypoint:%d-----------\n", keyPoint);
    for (int k = keyPoint; k < keyPoint + 80; ++k)
    {
        if (k >= (size - 1))
            break;

        xyz_temp = mDrivingPath.at(k % size);

        double dx = xyz_temp.x_axis - mNavData.xAxis;
        double dy = xyz_temp.y_axis - mNavData.yAxis;

        distance_temp = hypot(dx, dy);
        if (min > distance_temp)
        {
            min = distance_temp;
            keyPoint = k;
        }
    }
    mKeypoint = keyPoint;
}

void PathPlanComply::pubObstacles(ros::Publisher &tPub)
{
    // draw
    visualization_msgs::MarkerArray marker_array;
    for(int i = 0; i < obstacles_.size(); i++)
    {
        visualization_msgs::Marker obstacle_marker;
        obstacle_marker.header.frame_id = "robot";
        obstacle_marker.header.stamp = ros::Time::now();
        obstacle_marker.ns = "obstacle_" + std::to_string(i);
        obstacle_marker.id = i;
        obstacle_marker.type = visualization_msgs::Marker::CUBE;
        obstacle_marker.action = visualization_msgs::Marker::ADD;
        obstacle_marker.pose.position.x = obstacles_[i]->x;
        obstacle_marker.pose.position.y = obstacles_[i]->y;
        obstacle_marker.pose.position.z = 0.1;
        obstacle_marker.pose.orientation = tf::createQuaternionMsgFromYaw(obstacles_[i]->heading);
        obstacle_marker.scale.x = obstacles_[i]->width;
        obstacle_marker.scale.y = obstacles_[i]->length;
        obstacle_marker.scale.z = 0.1;
        obstacle_marker.color.a = 0.5;
        obstacle_marker.color.r = 1.0;
        obstacle_marker.color.g = 0.0;
        obstacle_marker.color.b = 0.0;
        marker_array.markers.push_back(obstacle_marker);
    }
    tPub.publish(marker_array);
}

ObstaclePtr PathPlanComply::findObstacle(std::vector<ObstaclePtr> &obstacles, double x, double y)
{
    for (auto obstacle_ptr : obstacles)
    {
        double ox = obstacle_ptr->x;
        double oy = obstacle_ptr->y;
        double dist = hypot(ox - x, oy - y);
        if (dist < 0.5)
        {
            return obstacle_ptr;
        }
    }
    return nullptr;
}

double PathPlanComply::getLaneLimitSpeedTest(double acur_lane_speed, double anext_lane_speed)
{
    static double next_lane_speed = anext_lane_speed;
    static double current_lane_speed = acur_lane_speed;
    static double lane_limit_speed = current_lane_speed;
    double divid_point_x = 6.25;
    double divid_point_y = -65.5;

    double ego_x = mNavData.xAxis;
    double ego_y = mNavData.yAxis;
    double ego_heading = gis_utils::azimuthToYaw(mNavData.heading);

    double ego_array_x = cos(ego_heading);
    double ego_array_y = sin(ego_heading);

    double ego_to_divid_point_x = divid_point_x - ego_x;
    double ego_to_divid_point_y = divid_point_y - ego_y;

    double dot_product = ego_array_x * ego_to_divid_point_x + ego_array_y * ego_to_divid_point_y;
    if (dot_product > 0) // 分界点在前方
    {
        double distance = sqrt(pow(ego_x - divid_point_x, 2) + pow(ego_y - divid_point_y, 2));
        if (distance < 10)
        {
            //lane_limit_speed = next_lane_speed + (current_lane_speed - next_lane_speed) / 10 * distance;
            // lane_limit_speed = next_lane_speed + (current_lane_speed - next_lane_speed) / 10 * distance;
        }
        printf("distance: %f\n", distance);
    }else
    {
    lane_limit_speed = next_lane_speed;
    }
    printf("============lane limit speed: %f, dot:%f\n", lane_limit_speed, dot_product);
    return lane_limit_speed;
}
