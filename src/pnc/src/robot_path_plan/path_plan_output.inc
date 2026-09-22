// 最终路径编排：平滑、停车覆盖、声光输出和发布。由 path_plan_comply.cpp 包含。

void PathPlanComply::PublishPlanPath(ros::Publisher &tPub1, ros::Publisher &tPub2) {
    // 最终输出覆盖顺序：
    // 手动模式/短路径提前返回 -> 复制参考路径 -> 加速平滑 -> 急停/断网 -> 声光 ->
    // 云端暂停 -> 地图最高限速 -> 发布时叠加闸机/起步周边 safety。
    // 修改任一停车条件时，必须继续检查它后面的 desireSpeed/safety 赋值。
    if(mVehicleData.controlPanelState == 0) {
        PublishStoppedPath(tPub1);
        return;
    }

    // 控制器少于 10 点会拒绝更新路径，不能继续发布非零速度让它沿用旧轨迹。
    if(mReferPath.x.size() < 10) {
        ROS_ERROR("publish plan path size less 10.");
        PublishStoppedPath(tPub1);
        return;
    }

    CopyReferencePath();
    SmoothPlanSpeed();
    ApplyEmergencyAndNetworkStop();

    int HornCmd = 0;
    int LightCmd = 0;
    UpdateSoundLightCommand(HornCmd, LightCmd);
    if(command_state == 1) {  // 暂停
        mPlanPath.desireSpeed = 0.0;
    }
    // 最终出口再次取小，防止其他速度编排绕过当前路径点的地图限速。
    mPlanPath.desireSpeed = std::min<double>(mPlanPath.desireSpeed, GetMapSpeedLimit());
    // param set & msg pub
    ros::param::set("/canbus/light", LightCmd);  // canbus 消费灯光命令
    ros::param::set("/canbus/horn", HornCmd);    // 鸣笛也没有用到
    PublishFinalPlanPath(tPub1);
    tPub2.publish(mSoundLightData);  // 这个没有用到
}

bool PathPlanComply::IsGantryAheadOnReference() const {
    const auto &x = mReferPath.x;
    const auto &y = mReferPath.y;
    const double ego_x = mNavData.xAxis, ego_y = mNavData.yAxis;
    if(x.size() < 2 || x.size() != y.size() ||
       !std::isfinite(ego_x) || !std::isfinite(ego_y)) return false;

    // 地图坐标，单位 m；与参考路径使用相同的 float32 坐标精度。
    const float gantry_points[2][2] = {{96.14f, -405.63f}, {91.13f, -393.80f}};
    const double max_path_distance = 2.0;
    const double max_forward_distance = 8.0;
    double gantry_distance_squared[2] = {-1.0, -1.0};
    double gantry_s[2] = {0.0, 0.0};
    double ego_distance_squared = -1.0, ego_s = 0.0, path_s = 0.0;
    double ego_projection_s = 0.0, ego_projection_ratio = 0.0;
    size_t first_segment = x.size(), last_segment = x.size(), ego_segment = x.size();

    for(size_t i = 0; i < x.size(); ++i) {
        const double point_x = x[i], point_y = y[i];
        // 不跨过坏点拼接参考线，也不沿用上一周期的闸机位置匹配。
        if(!std::isfinite(point_x) || !std::isfinite(point_y)) return false;
        if(i > 0) {
            const double dx = point_x - x[i - 1], dy = point_y - y[i - 1];
            const double length = std::hypot(dx, dy);
            if(length > 0.0) {
                if(first_segment == x.size()) first_segment = i;
                last_segment = i;
                // 末端参考线可能包含车后点，自车起点取在线段上的最近投影。
                const double projection_ratio =
                    ((ego_x - x[i - 1]) * dx + (ego_y - y[i - 1]) * dy) / (length * length);
                const double ratio = std::max(0.0, std::min(1.0, projection_ratio));
                const double offset_x = ego_x - (x[i - 1] + ratio * dx);
                const double offset_y = ego_y - (y[i - 1] + ratio * dy);
                const double distance_squared = offset_x * offset_x + offset_y * offset_y;
                if(ego_distance_squared < 0.0 || distance_squared < ego_distance_squared) {
                    ego_distance_squared = distance_squared;
                    ego_s = path_s + ratio * length;
                    ego_projection_s = path_s + projection_ratio * length;
                    ego_projection_ratio = projection_ratio;
                    ego_segment = i;
                }
            }
            path_s += length;
        }
        for(size_t gate = 0; gate < 2; ++gate) {
            // 闸机匹配真实参考路径点，不以线段投影替代用户指定的最近点。
            const double dx = point_x - gantry_points[gate][0];
            const double dy = point_y - gantry_points[gate][1];
            const double distance_squared = dx * dx + dy * dy;
            if(gantry_distance_squared[gate] < 0.0 || distance_squared < gantry_distance_squared[gate]) {
                gantry_distance_squared[gate] = distance_squared;
                gantry_s[gate] = path_s;
            }
        }
    }
    if(ego_distance_squared < 0.0) return false;
    // 自车略在参考首点之前或末点之后时保留端部切向里程，不能钳到端点而少算距离。
    if((ego_segment == first_segment && ego_projection_ratio < 0.0) ||
       (ego_segment == last_segment && ego_projection_ratio > 1.0)) ego_s = ego_projection_s;
    for(size_t gate = 0; gate < 2; ++gate) {
        // 参考点序即当前路线的行驶方向，D/R 均按有符号里程排除已驶过的点。
        const double forward_distance = gantry_s[gate] - ego_s;
        if(gantry_distance_squared[gate] < max_path_distance * max_path_distance &&
           forward_distance > 0.0 && forward_distance < max_forward_distance) return true;
    }
    return false;
}

void PathPlanComply::PublishFinalPlanPath(ros::Publisher &tPub) {
    // 独立停车条件只叠加本次发布，恢复原值避免解除观察时残留或清掉其他 safety。
    bool safety = mPlanPath.safety;
    const float desire_speed = mPlanPath.desireSpeed;
    const double now = ros::Time::now().toSec();
    const auto startup = mVehicleData.curGear == GEAR_D
                             ? mStartupObservation.CheckStatus(mPerceptionSafety.GetConfig(), now)
                             : planning_perception::StartupObservation::STATUS_S();
    // 离开范围、已经驶过或参考线失效时清除旧关闭状态，重入必须收到新消息。
    if(mGantryStop && !IsGantryAheadOnReference()) mGantryStop = false;
    const bool gantry_stop = mGantryStop;
    const bool invalid_navigation_speed = !std::isfinite(mNavData.gpsSpeed);
    if(gantry_stop) {
        mPlanPath.safety = 1;
    }
    const bool fence_stop = FenceOutputStop();
    if(startup.stop || invalid_navigation_speed || fence_stop) {
        mPlanPath.safety = 1;
        mPlanPath.desireSpeed = 0.0;
    }
    // 同时列出其他 safety 来源，避免把闸机/常规停车误认作起步观察未解除。
    const int sources = int(safety) | (int(gantry_stop) << 1) | (int(startup.stop) << 2) |
                        (int(emergencyStop != 0) << 3) | (int(invalid_navigation_speed) << 4) | (int(fence_stop) << 5);
    if(sources != mFinalSafetyLogSources || std::strcmp(startup.reason, mStartupLogReason) != 0 ||
       (sources != 0 && (now - mFinalSafetyLogTime >= 1.0 || now < mFinalSafetyLogTime))) {
        printf("plan safety: startup=%s id=%d boxes=%zu ages(perception/pose/can)=%.3f/%.3f/%.3f base=%d gantry=%d emergency=%d published=%d invalid_navigation_speed=%d\n",
               startup.reason, startup.object_id, startup.objects, startup.perception_age, startup.pose_age,
               startup.can_age, int(safety), int(gantry_stop), int(emergencyStop != 0), int(mPlanPath.safety),
               int(invalid_navigation_speed));
        mFinalSafetyLogTime = now;
        mFinalSafetyLogSources = sources;
        mStartupLogReason = startup.reason;
    }
    tPub.publish(mPlanPath);
    mPlanPath.safety = safety;
    mPlanPath.desireSpeed = desire_speed;
}

void PathPlanComply::PublishStoppedPath(ros::Publisher &tPub1) {
    // 退化路径只清坐标和期望速度，其余字段保留；先发布，再复位灯光。
    mPlanPath.x.clear();
    mPlanPath.y.clear();
    mPlanPath.desireSpeed = 0.0;
    PublishFinalPlanPath(tPub1);
    ros::param::set("/canbus/light", 0);
}

void PathPlanComply::CopyReferencePath() {
    // 只复制现役字段，不能整包赋值，以免改变其他输出字段的保留语义。
    mPlanPath.x = mReferPath.x;
    mPlanPath.y = mReferPath.y;
    mPlanPath.Path_Id = mReferPath.Path_Id;
    mPlanPath.safety = mReferPath.safety;
    mPlanPath.desireSpeed = mReferPath.desireSpeed;
    mPlanPath.planspeed = mReferPath.planspeed;
}

void PathPlanComply::SmoothPlanSpeed() {
    double spd = mNavData.gpsSpeed;
    double acc = mPlanPath.desireSpeed - mNavData.gpsSpeed;
    // 只平滑加速超过 0.5 m/s 的指令；挂托盘且销未升到顶时使用更慢的 0.1 权重。
    if(acc > 0.5) {
        if(mVehicleData.linkPallet && mVehicleData.hookStatus != ACTUATOR_UP_END) {  // 挂托盘，平滑过渡
            mPlanPath.desireSpeed = 0.1 * mPlanPath.desireSpeed + 0.9 * mNavData.gpsSpeed;
        } else {
            mPlanPath.desireSpeed = 0.3 * mPlanPath.desireSpeed + 0.7 * mNavData.gpsSpeed;
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
    // 先根据运动/停车时间更新鸣笛，再生成挡位和转向灯。
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
        int raw_cmd = 0;
        double max_left = 0.0;
        double max_right = 0.0;
        double end_dh = 0.0;

        if(mReferPath.x.size() >= 2) {
            Point me;
            me.x = mNavData.xAxis;
            me.y = mNavData.yAxis;
            me.heading = mNavData.heading;

            size_t end_idx = 0;
            for(size_t i = 1; i < mReferPath.x.size(); ++i) {
                double dist = std::hypot(mReferPath.x[i] - me.x, mReferPath.y[i] - me.y);
                if(dist > 18.0) {
                    end_idx = i;
                    break;
                }
                end_idx = i;
                Point pt;
                pt.x = mReferPath.x[i];
                pt.y = mReferPath.y[i];
                double dh = PointDirectionToMe(pt, me);
                if(dh > max_left) max_left = dh;
                if(dh < max_right) max_right = dh;
            }

            Point end_pt;
            end_pt.x = mReferPath.x[end_idx];
            end_pt.y = mReferPath.y[end_idx];
            end_dh = PointDirectionToMe(end_pt, me);

            // 判定转向意图：
            // 左转/掉头：前瞻范围内最大左偏角 >= 35.0°，且末端维持左偏 >= 20.0°，且抑制 S 弯反向摆动 (max_right > -15.0°)
            if(max_left >= 35.0 && end_dh >= 20.0 && max_right > -15.0) {
                raw_cmd = 3;
            }
            // 右转/掉头：前瞻范围内最大右偏角 <= -35.0°，且末端维持右偏 <= -20.0°，且抑制 S 弯反向摆动 (max_left < 15.0°)
            else if(max_right <= -35.0 && end_dh <= -20.0 && max_left < 15.0) {
                raw_cmd = 4;
            }
        }

        // 1. 开启滤波防抖：候选命令连续满足 3 个周期（0.3s）才触发点亮
        if(raw_cmd != 0) {
            if(raw_cmd == mTurnLightCandidate) {
                mTurnLightCandidateCount++;
            } else {
                mTurnLightCandidate = raw_cmd;
                mTurnLightCandidateCount = 1;
            }

            if(mTurnLightCandidateCount >= 3) {
                if(mActiveTurnLight != mTurnLightCandidate) {
                    mActiveTurnLight = mTurnLightCandidate;
                    mTurnLightActiveTime = sysTime.now;
                    mActiveTurnLightSteps = 0;
                }
                mTurnLightOffCount = 0;
            }
        } else {
            mTurnLightCandidate = 0;
            mTurnLightCandidateCount = 0;
        }

        // 2. 保持与平稳熄灭：
        if(mActiveTurnLight != 0) {
            mActiveTurnLightSteps++;
            double active_duration = sysTime.now - mTurnLightActiveTime;
            // 满足最小保持时间 1.5 秒（或 15 个规划周期）
            if((mTurnLightActiveTime > 0.0 && active_duration >= 1.5) || mActiveTurnLightSteps >= 15) {
                // 转弯结束，车身与路径恢复对齐（左右偏角均收敛到 15 度以内）
                if(max_left < 15.0 && max_right > -15.0) {
                    mTurnLightOffCount++;
                    if(mTurnLightOffCount >= 5) { // 持续 0.5 秒平稳熄灭
                        mActiveTurnLight = 0;
                        mTurnLightOffCount = 0;
                        mActiveTurnLightSteps = 0;
                        mTurnLightActiveTime = -1.0;
                    }
                } else {
                    mTurnLightOffCount = 0;
                }
            }
        }

        LightCmd = mActiveTurnLight;
        if(LightCmd == 3) {
            mPathPlanStatus.turning = "left";
        } else if(LightCmd == 4) {
            mPathPlanStatus.turning = "right";
        } else {
            mPathPlanStatus.turning = "none";
        }
    } else {
        // 非 D 挡清空转向灯候选与保持状态
        mTurnLightCandidate = 0;
        mTurnLightCandidateCount = 0;
        mActiveTurnLight = 0;
        mTurnLightActiveTime = -1.0;
        mActiveTurnLightSteps = 0;
        mTurnLightOffCount = 0;
        mPathPlanStatus.turning = "none";
    }

    if(mVehicleData.curGear == GEAR_R) {
        LightCmd = 5;  // 开启倒车灯
    }

    if(mPlanPath.safety == 1) {
        LightCmd = 8;
    }
}
