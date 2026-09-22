// 本文件由 path_plan_comply.cpp 原位包含，负责输入快照和感知预处理。
// 最终是否停车不在这里决定，而是在参考路径和最终路径发布阶段分层覆盖。

void PathPlanComply::SetCanData(robot::can_msg can_msg_t) {
    mStartupObservation.SetVehicle(can_msg_t.controlPanelState != 0, can_msg_t.linkPallet != 0,
                                   ros::Time::now().toSec(), can_msg_t.curGear == GEAR_D);
    mVehicleData = can_msg_t;
    if(mVehicleData.emergencyStop)
        emergencyStop = 1;
}

void PathPlanComply::SetGantryState(bool active, bool gantry_open) {
    mGantryStop = active && !gantry_open;
}

void PathPlanComply::SetStartupPerceptionData(const robot::perception &perception_t) {
    mStartupObservation.Observe(perception_t, ros::Time::now().toSec(), mPerceptionSafety.GetConfig().input_timeout);
}

void PathPlanComply::SetPerceptionData(const robot::perception &perception_t) {
    // 侧向感知先过滤车辆后方/退化目标，再合并前向扫描框。
    sysTime.msgPerception = ros::Time::now().toSec();

    mPerception.objs.clear();
    if(++mPerceptionOutsideGeneration == 0) {
        std::fill(mPerceptionOutsideMarks.begin(), mPerceptionOutsideMarks.end(), 0);
        ++mPerceptionOutsideGeneration;
    }
    bool has_outside = false;
    const auto lane_filter = BuildPerceptionLaneMotionFilter();

    for(const auto &i : perception_t.objs) {
        // 忽略左二、左右车道外和左一动态对向目标；复用原 ID 标记清理风险历史。
        if(i.type == 2 || i.type == 3 || i.type == 4 || lane_filter.Ignore(i)) {
            if(i.id > 0) mPerceptionOutsideMarks[i.id] = mPerceptionOutsideGeneration;
            has_outside = true;
            continue;
        }
        // 两边都大于 1 m 且长宽比超过 3 时剔除；正尺寸用乘法比较，避免除法。
        if(i.dx > 1.0 && i.dy > 1.0 && (i.dx > 3.0 * i.dy || i.dy > 3.0 * i.dx)) {
            continue;
        }
        Point me;
        Point point;
        me.x = mNavData.xAxis;
        me.y = mNavData.yAxis;
        me.heading = mNavData.heading;
        point.x = i.x;
        point.y = i.y;

        double dh = PointDirectionToMe(point, me);
        double hCheck = 0;

        if(fabs(dh) > 90.0)
            hCheck = 1;
        if(hCheck == 0) {
            if(i.dx < 0.05 || i.dy < 0.05)
                continue;
            mPerception.objs.push_back(i);
        }
    }

    for(const auto &i : mPerceptionFrontScan.objs)
        mPerception.objs.push_back(i);

    if(has_outside && !mPerceptionSafety.HasInput()) {
        // 兼容链四帧历史同步删除明确剔除的 ID；用索引避免目标与历史两层匹配。
        bool has_history = false;
        for(auto &frame : history_risk_vec) {
            frame.erase(std::remove_if(frame.begin(), frame.end(), [this](const robot::object &object) {
                            return object.type == 2 || object.type == 3 || object.type == 4 ||
                                   (object.id > 0 && mPerceptionOutsideMarks[object.id] == mPerceptionOutsideGeneration);
                        }),
                        frame.end());
            has_history = has_history || !frame.empty();
        }
        if(!has_history) safety_check_counter_refer = 0;
    }

    // 新观测链启用后，/perception 快照保留给轨迹复用等消费，跳过未使用的限速计算。
    if(mPerceptionSafety.HasInput()) return;

    // collistion check precise
    Point VehicleInfo;
    VehicleInfo.x = mNavData.xAxis;
    VehicleInfo.y = mNavData.yAxis;
    VehicleInfo.heading = mNavData.heading;
    VehicleInfo.length = 3.6;
    VehicleInfo.width = 2.0;

    double dist = 10000.0;  // 距离车辆最近障碍物距离
    double sdh = 100.0;     // 距离车辆最近障碍物朝向

    for(auto i : mPerception.objs) {
        Point point;
        point.x = i.x;
        point.y = i.y;

        double dertah = PointDirectionToMe(point, VehicleInfo);
        double dx = VehicleInfo.x - point.x;
        double dy = VehicleInfo.y - point.y;
        double ds = hypot(dx, dy) - VehicleInfo.length / 2.0;

        if(ds < dist) {
            dist = ds;
            sdh = fabs(dertah) * M_PI / 180.0;
        }
    }
    // 下面根据障碍物距离和障碍物朝向判断安全，下面的desiredSpeed没有用到
    double safety = 0;
    double desireSpeed = mNavData.gpsSpeed + 1.0;
    double width = dist * sin(sdh);

    if(dist < 20.0 && width < 1.5)
        desireSpeed = std::min<double>(desireSpeed, 4.0 * dist / 20.0);
    if(dist < 5.0 && width < 1.5)
        safety = 1;

    // high frequncy safaty check filter

    if(safety == 1) {
        safety_check_counter_percep += 1;

        if(safety_check_counter_percep < 3)
            mReferPath.safety = 0;
        else
            safety_check_counter_percep = 10;
    } else
        safety_check_counter_percep = 0;

    if(safety == 1)
        desireSpeed = 0.0;
}

void PathPlanComply::SetFrontScanData(jsk_recognition_msgs::BoundingBoxArray msg) {
    // scan_bbox_result 使用车体局部坐标，这里按最新定位旋转、平移到地图坐标。
    // 前雷达相对后轴中心的纵向偏移沿用既有 -1.5 m 口径。
    mPerceptionFrontScan.objs.clear();

    if(!msg.boxes.size())
        return;

    double x = mNavData.xAxis;
    double y = mNavData.yAxis;
    double h = 90.0 - mNavData.heading;

    if(h > 360.0)
        h -= 360.0;
    if(h < 0.0)
        h += 360.0;

    double theta = h * M_PI / 180.0;

    for(auto i : msg.boxes) {
        robot::object obj;
        obj.x = x - 1.5 + i.pose.position.x * cos(theta) - i.pose.position.y * sin(theta);
        obj.y = y + i.pose.position.x * sin(theta) + i.pose.position.y * cos(theta);
        obj.dx = 0.5;
        obj.dy = 0.5;
        mPerceptionFrontScan.objs.push_back(obj);
    }
}

void PathPlanComply::SetPerceptionData2(robot::perception perception_t) {
    sysTime.msgPerception = ros::Time::now().toSec();
    // 侧向雷达
    std::vector<ObstaclePtr> new_obstacles;
    for(auto i : perception_t.objs) {
        if(fabs(i.dx) < 0.01 || fabs(i.dy) < 0.01)
            continue;
        // mPerception.objs.push_back(i);
        ObstaclePtr obstacle = std::make_shared<Obstacle>();
        obstacle->x = i.x;
        obstacle->y = i.y;
        obstacle->width = i.dx;
        obstacle->length = i.dy;
        obstacle->heading = gis_utils::azimuthToYaw(i.heading - 90);
        new_obstacles.push_back(obstacle);
    }
    // 前向雷达
    for(auto i : mPerceptionFrontScan.objs) {
        if(fabs(i.dx) < 0.01 || fabs(i.dy) < 0.01)
            continue;
        // mPerception.objs.push_back(i);
        ObstaclePtr obstacle = std::make_shared<Obstacle>();
        obstacle->x = i.x;
        obstacle->y = i.y;
        obstacle->width = i.dx;
        obstacle->length = i.dy;
        obstacle->heading = gis_utils::azimuthToYaw(i.heading - 90);
        new_obstacles.push_back(obstacle);
    }
    // 比较old and new obstacles
    // new find old
    for(int i = 0; i < new_obstacles.size(); i++) {
        auto o = new_obstacles[i];
        auto find_it = findObstacle(obstacles_, o->x, o->y);
        if(find_it) {
            find_it->history.erase(find_it->history.begin());
            find_it->x = o->x;
            find_it->y = o->y;
            find_it->length = o->length;
            find_it->width = o->width;
            find_it->heading = o->heading;
            find_it->history.push_back(1);
        } else {
            obstacles_.push_back(o);
        }
    }
    // old find new
    for(int i = 0; i < obstacles_.size(); i++) {
        auto o = obstacles_[i];
        auto find_it = findObstacle(new_obstacles, o->x, o->y);
        if(!find_it) {
            o->history.erase(o->history.begin());
            o->history.push_back(0);
        }
    }
    // 清理消失的障碍物
    for(auto it = obstacles_.begin(); it != obstacles_.end();) {
        auto o = *it;

        bool erase = o->historySum() < 1;
        if(erase) {
            it = obstacles_.erase(it);
        } else {
            it++;
        }
    }
    // 输出障碍物列表
    // for (int i = 0; i < obstacles_.size(); i++)
    // {
    //     auto o = obstacles_[i];
    //     printf("obstacle %d: (%.2f, %.2f), (%.2f, %.2f,%.2f), (%d,%d,%d,%d)\n", i, o->x, o->y, o->length, o->width, o->heading,
    //            o->history[0], o->history[1], o->history[2], o->history[3]);
    // }
}

void PathPlanComply::SetBackScanData(jsk_recognition_msgs::BoundingBoxArray msg) {
    // 后向框同样转换到地图坐标，仅供 R 挡 2.5 m 停车判断使用。
    mPerceptionBackScan.objs.clear();

    if(!msg.boxes.size())
        return;

    double x = mNavData.xAxis;
    double y = mNavData.yAxis;
    double h = 90.0 - mNavData.heading;

    if(h > 360.0)
        h -= 360.0;
    if(h < 0.0)
        h += 360.0;

    double theta = h * M_PI / 180.0;

    for(auto i : msg.boxes) {
        robot::object obj;
        obj.x = x - 0.9 - i.pose.position.x * cos(theta) + i.pose.position.y * sin(theta);
        obj.y = y - i.pose.position.x * sin(theta) - i.pose.position.y * cos(theta);
        obj.dx = 0.5;
        obj.dy = 0.5;
        mPerceptionBackScan.objs.push_back(obj);
    }

}

void PathPlanComply::SetPalletCoorData(robot::hook_position hook_pos_t) {
    mHookPos = hook_pos_t;
    // mHookPos.center_point_y += 0.1;
}

void PathPlanComply::SetLoadPosition(robot::palletpos load_pos_t) {
    mLoadPos = load_pos_t;
}

void PathPlanComply::SetNavigationData(robot::navigation_msg navigation_msg_t) {
    const double now = ros::Time::now().toSec();
    mStartupObservation.SetPose(navigation_msg_t.xAxis, navigation_msg_t.yAxis,
                                navigation_msg_t.heading, now);
    // 只由新的导航回调推进起步车速确认，CAN 重发不能重复计票。
    mStartupObservation.SetSpeed(navigation_msg_t.gpsSpeed, now);
    mNavData = navigation_msg_t;
    mRcvGpsData = true;
    mPathPlanStatus.curXAxis = mNavData.xAxis;
    mPathPlanStatus.curYAxis = mNavData.yAxis;

    mPathPlanStatus.curHead = mNavData.heading;
    mPathPlanStatus.curSpeed = mNavData.gpsSpeed;
    mPathPlanStatus.rtkState = mNavData.rtkState;
    // 计算车辆的包围盒 车长3.45m，后轴中心到前雷达:2.41   车宽1.47m,托盘宽3.4m
    //  double l = 4.82;
    //  double w = 3.4;
}

void PathPlanComply::SetButtonData(robot::rs232 button_data_t) {
    if(mVehicleData.emergencyStop)
        return;
    if(button_data_t.data[3])
        emergencyStop = 0;
}

double PathPlanComply::Distance2PerceptionObjs(double x, double y) {
    double dist = 10000.0;

    for(auto i : mPerception.objs) {
        double dx = fabs(x - i.x);
        double dy = fabs(y - i.y);
        double len = std::min<double>(i.dx / 2.0, i.dy / 2.0);
        double ds = hypot(dx, dy) - len - 0.5;
        double dl = dx - i.dx / 2.0;
        double dw = dy - i.dy / 2.0;

        if(dist > ds)
            dist = ds;
    }

    return dist;
}

void PathPlanComply::SetCommandData(robot::v2nCommandFeedback command_t) {
    // 云端指令先写入状态；停车再清任务，暂停/继续由最终输出阶段消费。
    SetCommandState(command_t.commandState);  // 0停车 1暂停 2继续
    if(command_t.commandState == 0) {
        resetTask();
    }
}

ObstaclePtr PathPlanComply::findObstacle(std::vector<ObstaclePtr> &obstacles, double x, double y) {
    for(auto obstacle_ptr : obstacles) {
        double ox = obstacle_ptr->x;
        double oy = obstacle_ptr->y;
        double dist = hypot(ox - x, oy - y);
        if(dist < 0.5) {
            return obstacle_ptr;
        }
    }
    return nullptr;
}

std::vector<sCellMsg> PathPlanComply::Obj_Projecte_Map(robot::perception local_object, std::vector<sCellMsg> ldobjs_global) {
    int obj_nums = local_object.objs.size();
    int subcon_num = 0;

    if(obj_nums < 1)
        return ldobjs_global;

    for(int i = 0; i < obj_nums; i++) {
        sCellMsg center_point;
        center_point.xg = local_object.objs[i].x;
        center_point.yg = local_object.objs[i].y;
        center_point.heading = local_object.objs[i].heading;
        ldobjs_global.emplace_back(center_point);
    }

    return ldobjs_global;
}
