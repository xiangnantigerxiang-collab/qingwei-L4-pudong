// task_plan 拥有围栏策略；此适配层校验已批准任务与最终生成路径的交接。
bool PathPlanComply::FenceTaskRequired() const {
    return mFenceEnabled && !mFenceExempt;
}

bool PathPlanComply::FenceGuardFresh() const {
    const double now = ros::Time::now().toSec();
    const double age = now - mFenceReceived;
    const double source_age = now - mFenceGuard.header.stamp.toSec();
    return mFenceReceived >= 0 && std::isfinite(age) && age >= 0 && age <= 0.5 &&
           std::isfinite(source_age) && source_age >= 0 && source_age <= 0.5;
}

void PathPlanComply::SetFenceGuard(const robot::task_fence_guard &guard) {
    if(!mFenceEnabled || guard.revision < mFenceGuard.revision) return;
    if(guard.revision == mFenceGuard.revision && mFenceReceived >= 0 &&
       (guard.task_key != mFenceGuard.task_key || guard.fence_version != mFenceGuard.fence_version ||
        guard.route_version != mFenceGuard.route_version || guard.fence_file != mFenceGuard.fence_file ||
        guard.start_index != mFenceGuard.start_index || guard.stop_index != mFenceGuard.stop_index ||
        guard.truncated != mFenceGuard.truncated)) {
        FailFence("guard contract changed without new revision");
        return;
    }
    mFenceGuard = guard;
    mFenceReceived = ros::Time::now().toSec();
}

void PathPlanComply::FailFence(const std::string &reason) {
    mFenceFailed = true;
    mFenceFailure = reason;
    mDesireSpeed = 0;
}

bool PathPlanComply::CheckFencePath(const std::vector<float> &x, const std::vector<float> &y) const {
    if(x.size() != y.size() || x.size() > 200000) return false;
    for(size_t i = 0; i < x.size(); ++i) {
        const task_fence::Point point(x[i], y[i]);
        if(!mTaskFenceGeometry.Contains(point)) return false;
        if(i > 0 && !mTaskFenceGeometry.SegmentInside(task_fence::Point(x[i - 1], y[i - 1]), point, true)) return false;
    }
    return true;
}

void PathPlanComply::TryLoadFenceTask() {
    if(!mFencePending || !FenceGuardFresh() || mFenceGuard.stop ||
       mFenceGuard.task_key != mFenceTaskKey || mFenceGuard.revision <= mFenceLoadedRevision) return;
    if(FenceTaskRequired() && !mRcvGpsData) return;
    mFencePending = false;
    mFenceLoadedRevision = mFenceGuard.revision;
    if(!FenceTaskRequired()) {
        calcuGlobalPath(mTaskPlanData);
    } else {
        std::string error;
        if(!mTaskFenceGeometry.Load(mFenceGuard.fence_file, error) ||
           mTaskFenceGeometry.Version() != mFenceGuard.fence_version) {
            FailFence("fence file version mismatch: " + error); return;
        }
        LoadTaskPaths(mTaskPlanData);
        uint64_t hash = task_fence::HashStart();
        for(const auto &point : mDrivingPath)
            hash = task_fence::HashPose(hash, point.x_axis, point.y_axis, point.heading);
        hash = task_fence::HashWord(hash, mDrivingPath.size());
        if(hash != mFenceGuard.route_version || mDrivingPath.size() < 20 ||
           mFenceGuard.start_index > mFenceGuard.stop_index || mFenceGuard.stop_index >= mDrivingPath.size()) {
            FailFence("approved route version or index mismatch"); return;
        }
        if(mFenceGuard.truncated) {
            std::vector<XYZ_COOR_S> prefix(mDrivingPath.begin() + mFenceGuard.start_index,
                                         mDrivingPath.begin() + mFenceGuard.stop_index + 1);
            mDrivingPath.swap(prefix);
            if(mDrivingPath.size() < 20) { FailFence("approved prefix too short"); return; }
            // 只重算里程，不改变 CSV 航向和地图限速。
            const double origin = mDrivingPath.front().dist_origin;
            for(auto &point : mDrivingPath) point.dist_origin -= origin;
        }
        FindTaskStartPoint();
        ConnectVehicleToPath();
        if(mFenceGuard.truncated) {
            if(mDrivingPath.empty() || std::hypot(mDrivingPath.back().x_axis - mTaskPlanData.stopX,
                                                 mDrivingPath.back().y_axis - mTaskPlanData.stopY) > 0.05) {
                FailFence("connection changed approved stop"); return;
            }
            mStopIndex = mDrivingPath.size() - 1;
        } else FindTaskStopPoint(mTaskPlanData);
        if(mKeypoint < 0 || mStopIndex < mKeypoint || mStopIndex >= (int)mDrivingPath.size()) {
            FailFence("invalid generated route interval"); return;
        }
        std::vector<float> x, y;
        x.reserve(mStopIndex - mKeypoint + 2); y.reserve(mStopIndex - mKeypoint + 2);
        x.push_back(mNavData.xAxis); y.push_back(mNavData.yAxis);
        for(int i = mKeypoint; i <= mStopIndex; ++i) {
            x.push_back(mDrivingPath[i].x_axis); y.push_back(mDrivingPath[i].y_axis);
        }
        if(!CheckFencePath(x, y)) { FailFence("generated route crosses fence"); return; }
    }
    if(mTaskPlanData.task_id == go_task_id_) updateAirPortStopIndex(mDrivingPath, true);
    if(mTaskPlanData.task_id == back_task_id_) updateAirPortStopIndex(mDrivingPath, false);
}

bool PathPlanComply::FenceOutputStop() {
    if(!mFenceEnabled) return false;
    if(!FenceGuardFresh() || mFenceGuard.stop || mFencePending || mFenceFailed ||
       mFenceTaskKey != mFenceGuard.task_key || mFenceLoadedRevision != mFenceGuard.revision) return true;
    // 最终出口包括早退和动态生成路径，不能只检查静态 CSV。
    if(FenceTaskRequired() && !CheckFencePath(mPlanPath.x, mPlanPath.y)) {
        FailFence("published trajectory crosses fence"); return true;
    }
    return false;
}

void PathPlanComply::PublishFenceFeedback(ros::Publisher &publisher) {
    if(!mFenceEnabled || mFencePending || !FenceGuardFresh() ||
       mFenceTaskKey != mFenceGuard.task_key || mFenceLoadedRevision != mFenceGuard.revision) return;
    robot::task_fence_guard feedback = mFenceGuard;
    feedback.header.stamp = ros::Time::now();
    feedback.stop = mFenceFailed;
    feedback.reason = mFenceFailure;
    feedback.completed = !mFenceFailed && !mFenceGuard.stop && mPathPlanStatus.taskExecuStatus == TASKFINISHED;
    publisher.publish(feedback);
}
