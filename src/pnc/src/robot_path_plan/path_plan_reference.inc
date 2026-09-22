// 周期规划与参考路径发布入口。执行顺序决定本周期的速度和安全状态。

void PathPlanComply::PathPlanProcess() {
    if(mFenceEnabled) {
        TryLoadFenceTask();
        if(mFencePending || mFenceFailed || !FenceGuardFresh() || mFenceGuard.stop ||
           mFenceLoadedRevision != mFenceGuard.revision || mFenceTaskKey != mFenceGuard.task_key) {
            mDesireSpeed = 0;
            mReferPath.x.clear(); mReferPath.y.clear(); mReferPath.desireSpeed = 0;
            return;
        }
    }
    // 10 Hz 周期主流程：
    // 1. 必要时生成作业路径；
    // 2. 在全局路径上向前推进 mKeypoint（窗口随候选点移动，详见 UpdatePathInfo）；
    // 3. 根据停车点、任务类型和挡位计算本周期速度上限；
    // 4. 到达任务终点后清空任务路径。此函数本身不发布消息。
    // 1. 生成路径
    handleDrivingPath();
    // 2.  更新最近点mKeyPoint索引

    mDesireSpeed = mTaskPlanData.desireSpeed;

    double x0 = mNavData.xAxis;
    double y0 = mNavData.yAxis;
    double h0 = mNavData.heading;
    double xt = mPathPlanStatus.stopX;
    double yt = mPathPlanStatus.stopY;
    double ht = mPathPlanStatus.stopAngle;
    if(!mDrivingPath.empty()) {
        UpdatePathInfo();
        if(mVehicleData.curGear == GEAR_D) {
            XYZ_COOR_S xyz = mDrivingPath.at(mKeypoint);
            x0 = xyz.x_axis;
            y0 = xyz.y_axis;
        }
    }
    mDesireSpeed = std::min<double>(mDesireSpeed, GetMapSpeedLimit());
    // 根据与停止点的距离计算一个速度
    double speed = LimitSpeedByDistanceToStop(x0, y0, h0, xt, yt, ht);
    // 与任务要求速度进行比较
    mDesireSpeed = std::min<double>(speed, mDesireSpeed);
    if(mVehicleData.curGear == GEAR_R)  // 倒挡速度要求不大于0.5
        mDesireSpeed = std::min<double>(mDesireSpeed, 0.5);
    if(mVehicleData.curGear == GEAR_N) {
        printf("挡位: N, desire speed: 0\n");
        mDesireSpeed = 0.0;
    }

    if(mPathPlanStatus.taskExecuStatus == TASKFINISHED) {
        mDesireSpeed = 0.0;
        mKeypoint = 0;
        mStopIndex = 0;
        mTaskPlanData.pathList.clear();
        mTaskPlanData.taskType = NOTASK;
    }
}

void PathPlanComply::PublishPathPlanStatus(ros::Publisher &tPub) {
    tPub.publish(mPathPlanStatus);
}

void PathPlanComply::PublishReferPath(ros::Publisher &tPub) {
    // 任务完成或当前路线不足规划所需点数时清空旧参考，避免新任务沿用上一条路线。
    // 最终发布阶段会将空参考转为零速空路径，控制侧仍能收到停车指令。
    if(mPathPlanStatus.taskExecuStatus == TASKFINISHED || mDrivingPath.size() < 20) {
        mReferPath.x.clear();
        mReferPath.y.clear();
        mReferPath.desireSpeed = 0.0;
        mReferPath.planspeed = 0.0;
        return;
    }
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> heading;
    BuildReferencePathPoints(x, y, heading);

    // R 挡只使用后向扫描距离；其他挡位沿用前向生成与碰撞检查流程。
    if(mVehicleData.curGear == GEAR_R) {
        UpdateReverseReferencePath(x, y);
    } else {
        UpdateForwardReferencePath(x, y, heading);
        const double speed_limit = mReferPath.desireSpeed;
        CheckForwardReferenceSafety();
        // 兼容避障会混合当前车速；只能继续降速，不能抬高已经算出的终点/任务限速。
        mReferPath.desireSpeed = std::min<double>(mReferPath.desireSpeed, speed_limit);
    }
    // 碰撞检查内部的早退只结束检查阶段，仍须发布本周期参考路径。
    tPub.publish(mReferPath);
}

#include "reference/path_generation.inc"
#include "reference/collision_safety.inc"
