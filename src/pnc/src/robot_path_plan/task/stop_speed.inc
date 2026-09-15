// 按任务类型计算停车限速和完成状态。由 path_plan_task.inc 包含。

float PathPlanComply::LimitSpeedByDistanceToStop(
    float tCurX, float tCurY, float tCurAngle,
    float tTargetX, float tTargetY, float tTargetAngle) {
    if(mTaskPlanData.taskType == DOACTION) {
        UpdateActionTaskStatus();
        return 0.0;
    }

    // 先记录到任务终点的距离；后续机位临停和作业距离只更新 remain_distance_。
    if(mStopIndex < 0 || mStopIndex >= mDrivingPath.size() || mKeypoint < 0 || mKeypoint >= mDrivingPath.size()) {
        printf("Invalid stop index or keypoint index. stopIndex: %d, keypoint: %d\n", mStopIndex, mKeypoint);
        return 0.0;
    }
    remain_distance_ = mDrivingPath.at(mStopIndex).dist_origin - mDrivingPath.at(mKeypoint).dist_origin;
    printf("remain_distance_: %f, key_index:%d, stop_index:%d\n", remain_distance_, mKeypoint, mStopIndex);
    mPathPlanStatus.distance2Stop = remain_distance_;

    if(mTaskPlanData.taskType == ADAPTIVEPARK)
        return LimitParkTaskSpeed(tCurX, tCurY, tCurAngle, tTargetX, tTargetY);
    if(mTaskPlanData.taskType == ADAPTIVEHOOK)
        return LimitHookTaskSpeed();
    return LimitDrivingTaskSpeed();
}

void PathPlanComply::UpdateActionTaskStatus() {
    // 动作任务始终返回零速；完成判据使用 canbus 去抖后的 hookStatus，不直接比较位置码值。
    if(mTaskPlanData.hookCmd == HOOKOPERATION) {
        printf("正在hook中....\n");
        if(mVehicleData.hookStatus == ACTUATOR_UP_END) {  // canbus 状态机判销至顶端(带内持续1秒)
            printf("hook到位, task finished\n");
            mPathPlanStatus.taskExecuStatus = TASKFINISHED;
        }
    }
    if(mTaskPlanData.hookCmd == DECOUPLING) {
        printf("正在decoupling中....\n");
        if(mVehicleData.hookStatus == ACTUATOR_DOWN_END) {  // canbus 状态机判销退到底(带内持续1秒)
            printf("hook open, task finished\n");
            mPathPlanStatus.taskExecuStatus = TASKFINISHED;
        }
    }
    if(mTaskPlanData.stopX == -1 && mTaskPlanData.stopY == -1) {
        printf("无停止点, task finished\n");
        mPathPlanStatus.taskExecuStatus = TASKFINISHED;
    }
}

float PathPlanComply::LimitParkTaskSpeed(float tCurX, float tCurY, float tCurAngle,
                                         float tTargetX, float tTargetY) {
    // 倒车入库时，停车点已落到车前 45 度内意味着已越过目标，也按到位处理。
    printf("倒车入库中...\n");
    Point point;
    Point VehicleInfo;

    VehicleInfo.x = tCurX;
    VehicleInfo.y = tCurY;
    VehicleInfo.heading = tCurAngle;
    point.x = tTargetX;
    point.y = tTargetY;

    double dh = PointDirectionToMe(point, VehicleInfo);

    if(fabs(dh) < 45.0) {
        printf("停车点在车前45度内\n");
        remain_distance_ = 0.0;
    }

    if(remain_distance_ < 0.2) {
        remain_distance_ = 0.0;
        printf("倒车入库结束 task finished\n");
        mPathPlanStatus.taskExecuStatus = TASKFINISHED;
    }
    return std::min<double>(remain_distance_, 0.6);
}

float PathPlanComply::LimitHookTaskSpeed() {
    // 挂钩阶段改用后向感知的 center_distance；连接反馈只在距离小于 1.5 m 时判定完成。
    printf("挂钩循迹中...\n");
    remain_distance_ = mHookPos.center_distance;
    printf("hook, remain_distance_ %f\n", remain_distance_);
    if(remain_distance_ < 1.5) {
        // 历史区域限定已注释；现役判据是下方的 linkPallet。
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

        if(mVehicleData.linkPallet) {  // 检测到托盘，切换到动作任务
            printf("link hook到位, task finished\n");
            mPathPlanStatus.taskExecuStatus = TASKFINISHED;
            ROS_INFO("[Plan][info:][link pallet]");
            return 0;
        }
        // 静态托盘在接近 0.3 m 后若距离反增超过 0.5 m，保持停车，避免继续倒退。
        if(PalletType == 0) {
            if(remain_distance_ < 0.3)
                backdist_flag = 1;
            if(remain_distance_ > 0.5 && backdist_flag == 1)
                return 0;
        }

        return std::max(remain_distance_ / 2.0, 0.1);  //6.0
    } else
        backdist_flag = 0;

    return 0.5;
}

float PathPlanComply::LimitDrivingTaskSpeed() {
    // 指定去/回程任务优先服从占用飞机位的临时停车线；其他任务直接驶向终点。
    int stop_index = mStopIndex;
    if(mTaskPlanData.task_id == go_task_id_ || mTaskPlanData.task_id == back_task_id_) {
        auto port = findNextAirCraftParkingPort();
        printf("next port is:%d, parking:%d\n", port.id, port.parking);
        if(int(port.parking) == 1) {  // 入位中
            stop_index = port.go_stop_index == 0 ? port.back_stop_index : port.go_stop_index;
        }
    }
    remain_distance_ = mDrivingPath.at(stop_index).dist_origin - mDrivingPath.at(mKeypoint).dist_origin;
    printf("<<<<dest stop index: %d, airport stop index: %d>>>>>\n", mStopIndex, stop_index);
    if(mVehicleData.curGear == GEAR_N) {
        printf("挡位:N, 停车\n");
        return 0;
    }
    if(mVehicleData.curGear == GEAR_D) {
        printf("挡位:D, 路径跟踪\n");

        if(remain_distance_ < 0.5) {  // 距离终点还剩0.5开始刹停
            // 临时停车只返回零速，不能把整个任务标成完成。
            if(stop_index == mStopIndex) {
                printf("离终点小于0.5, task_finished\n");
                remain_distance_ = 0.0;
                mPathPlanStatus.taskExecuStatus = TASKFINISHED;
                return 0.0;
            } else {
                printf("达到停止线停车点\n");
                return 0.0;
            }
        } else {
            if(remain_distance_ > 10.0)  // 远处返回距离值，外层再与任务期望速度取最小值。
                return remain_distance_;
            else
                return std::max<double>(remain_distance_ / 6.0, 0.3);
        }
    } else {
        printf("挡位:%d\n", mVehicleData.curGear);
    }
    printf("return 0\n");
    return 0.0;
}
