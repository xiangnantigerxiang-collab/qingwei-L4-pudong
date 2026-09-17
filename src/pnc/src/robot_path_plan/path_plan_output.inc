// 最终路径编排：观察、平滑、停车覆盖、声光输出和发布。由 path_plan_comply.cpp 包含。

void PathPlanComply::PublishPlanPath(ros::Publisher &tPub1, ros::Publisher &tPub2) {
    // 最终输出覆盖顺序：
    // 更新周边/T 路口观察窗 -> 手动模式/短路径提前返回 -> 起步安全判定 -> 复制参考路径 ->
    // 加速平滑 -> 急停/断网 -> 声光 -> 等待区/T 路口 -> 云端暂停 -> 单点区域限速 -> 发布时叠加闸机 safety。
    // 修改任一停车条件时，必须继续检查它后面的 desireSpeed/safety 赋值。
    // 启动时，检查四周是否有障碍物
    bool is_around_unsafe = checkAroundObstacle();
    bool is_T_unsafe = checkTJunctionObstacle();
    bool is_in_waiting = checkIsInWaiting();

    if(mVehicleData.controlPanelState == 0) {
        PublishStoppedPath(tPub1);
        // int around_horn_cmd = 0;
        //  ros::param::get("/canbus/light", around_horn_cmd);
        //  if (around_horn_cmd == 7)
        //  {
        //      ros::param::set("/canbus/light", 0);
        //  }
        return;
    }

    if(mReferPath.x.size() < 5) {
        ROS_ERROR("publish plan path size less 5.");
        PublishStoppedPath(tPub1);
        return;
    }

    UpdateStartupSafety(is_around_unsafe, is_T_unsafe);
    CopyReferencePath();
    SmoothPlanSpeed();
    ApplyEmergencyAndNetworkStop();

    int HornCmd = 0;
    int LightCmd = 0;
    UpdateSoundLightCommand(HornCmd, LightCmd);
    ApplyWaitingAreaStop(is_T_unsafe, LightCmd);
    if(command_state == 1) {  // 暂停
        mPlanPath.desireSpeed = 0.0;
    }
    ApplyPointSpeedLimit();
    printf("end speedlimit: %f\n", mPlanPath.desireSpeed);
    // param set & msg pub
    ros::param::set("/canbus/light", LightCmd);  // 灯光和起步周边障碍物报警再canbus用到了
    ros::param::set("/canbus/horn", HornCmd);    // 鸣笛也没有用到
    PublishFinalPlanPath(tPub1);
    tPub2.publish(mSoundLightData);  // 这个没有用到
}

void PathPlanComply::PublishFinalPlanPath(ros::Publisher &tPub) {
    // 闸机只覆盖本次发布的 safety；保留原值，避免影响声光或在退化路径中残留。
    bool safety = mPlanPath.safety;
    if(mGantryStop) {
        mPlanPath.safety = 1;
    }
    tPub.publish(mPlanPath);
    mPlanPath.safety = safety;
}

void PathPlanComply::ApplyPointSpeedLimit() {
    if(mPointSpeedLimits.empty()) return;
    const double x = mNavData.xAxis, y = mNavData.yAxis;
    if(!std::isfinite(x) || !std::isfinite(y)) return;
    for(const auto &point : mPointSpeedLimits) {
        if(point.speed_limit >= mPlanPath.desireSpeed) continue;
        const double dx = x - point.x;
        if(dx < -5.0 || dx > 5.0) continue;
        const double dy = y - point.y;
        // 含 5 m 边界；多个区域重叠时逐项取小，保持其他业务的零速和更低限速。
        if(dx * dx + dy * dy <= 25.0) {
            mPlanPath.desireSpeed = std::min<double>(mPlanPath.desireSpeed, point.speed_limit);
        }
    }
}

void PathPlanComply::PublishStoppedPath(ros::Publisher &tPub1) {
    // 退化路径只清坐标和期望速度，其余字段保留；先发布，再复位起步检查和灯光。
    mPlanPath.x.clear();
    mPlanPath.y.clear();
    mPlanPath.desireSpeed = 0.0;
    PublishFinalPlanPath(tPub1);
    InitSafetyCheck = 0;
    ros::param::set("/canbus/light", 0);
}

void PathPlanComply::UpdateStartupSafety(bool is_around_unsafe, bool is_T_unsafe) {
    // 起步观察
    printf("IniSafetyCheck: %d\n", InitSafetyCheck);

    if(InitSafetyCheck == 0) {
        InitSafetyCheck = 1;
        mPlanPath.safety = 1;
        if(is_around_unsafe) {
            ROS_WARN("周围有障碍物.");
            // printf("四周不安全\n");
            InitSafetyCheck = 0;
        }
        // T路口观察
        if(is_T_unsafe) {
            ROS_WARN("T路口有障碍物");
            // printf("T路口不安全\n");
            InitSafetyCheck = 0;
        }
    }
}

void PathPlanComply::CopyReferencePath() {
    // 只复制现役字段，不能整包赋值，以免改变其他输出字段的保留语义。
    // 此处会覆盖起步判定写入的 safety；起步未通过由后面的等待区覆盖阶段压零速。
    mPlanPath.x = mReferPath.x;
    mPlanPath.y = mReferPath.y;
    mPlanPath.Path_Id = mReferPath.Path_Id;
    mPlanPath.safety = mReferPath.safety;
    mPlanPath.desireSpeed = mReferPath.desireSpeed;
    mPlanPath.planspeed = mReferPath.planspeed;
}

void PathPlanComply::SmoothPlanSpeed() {
    double spd = mVehicleData.vehicleSpeed;
    double acc = mPlanPath.desireSpeed - mVehicleData.vehicleSpeed;
    // 只平滑加速超过 0.5 m/s 的指令；挂托盘且销未升到顶时使用更慢的 0.1 权重。
    if(acc > 0.5) {
        if(mVehicleData.linkPallet && mVehicleData.hookStatus != ACTUATOR_UP_END) {  // 挂托盘，平滑过渡
            mPlanPath.desireSpeed = 0.1 * mPlanPath.desireSpeed + 0.9 * mVehicleData.vehicleSpeed;
        } else {
            mPlanPath.desireSpeed = 0.3 * mPlanPath.desireSpeed + 0.7 * mVehicleData.vehicleSpeed;
        }
    }
}

void PathPlanComply::ApplyEmergencyAndNetworkStop() {
    // 急停同时置 safety 和零速，断网只压零速；两者字段语义保持区分。
    // emergency pause and recovery
    if(emergencyStop == 1) {
        mPlanPath.safety = 1;
        mPlanPath.desireSpeed = 0.0;
    }

    int network_status = 0;

    ros::param::get("/robot/planning/netcheck", network_status);
    // 网络异常，则立即停车
    if(network_status != 0) {
        mPlanPath.desireSpeed = 0.0;
        ROS_WARN("Network Disconnected !");
    }
}

void PathPlanComply::UpdateSoundLightCommand(int &HornCmd, int &LightCmd) {
    // 先根据运动/停车时间更新鸣笛，再生成挡位和转向灯；后续等待区报警可覆盖灯光。
    // sound & light command publish
    HornCmd = 0;
    if(mPathPlanStatus.curSpeed > 0.1) {
        sysTime.vehicleStop = ros::Time::now().toSec();
    } else
        sysTime.vehicleRun = ros::Time::now().toSec();

    double stime = sysTime.now - sysTime.vehicleStop;
    double rtime = sysTime.now - sysTime.vehicleRun;
    double ttime = sysTime.now - sysTime.taskStart;
    // 鸣笛逻辑
    if(stime > 3.0)  // 起步鸣笛
        HornFlag = 1;

    if(HornFlag == 1 && rtime < 1.0 && stime < 1.0)
        HornCmd = 1;
    if(ttime < 1.0 && stime < 1.0)
        HornCmd = 1;
    if(rtime > 1.0)  // 停车>1s，不鸣笛
        HornFlag = 0;

    // 灯光逻辑
    LightCmd = 0;
    if(mVehicleData.curGear == GEAR_D) {
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
        if(dh > 6.0) {     // 目标点在右侧超过10度
            LightCmd = 3;  // 开启右转向灯
            mPathPlanStatus.turning = "left";
        }

        if(dh < -6.0) {    // 目标点在左侧超过10度
            LightCmd = 4;  // 开启左转向灯
            mPathPlanStatus.turning = "right";
        }
    }

    if(mVehicleData.curGear == GEAR_R) {
        LightCmd = 5;  // 开启倒车灯
    }

    if(mPlanPath.safety == 1) {
        LightCmd = 8;
        // int sum = sumVec(history_unsafe);
    }
}

void PathPlanComply::ApplyWaitingAreaStop(bool is_T_unsafe, int &LightCmd) {
    // 起步检查未通过时持续报警；通过后仅等待区内的 T 路口风险继续限制通行。
    if(InitSafetyCheck == 0) {
        LightCmd = 7;  // 报警
        mPlanPath.desireSpeed = 0.0;
    } else {
        if(checkIsInWaiting() && is_T_unsafe) {
            printf("车辆在等待区, 检测周围障碍物\n");
            LightCmd = 7;  // 报警
            mPlanPath.desireSpeed = 0.0;
        }
    }
}

#include "safety/startup_observation.inc"
