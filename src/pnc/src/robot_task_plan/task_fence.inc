// 由 task_plan_core.cpp 包含。文件读取仅发生在任务装载/块切换，周期检查只用缓存。
bool TaskPlanCore::FenceDriving(uint8_t type) const {
    return type >= TRACKPATH && type <= ADAPTIVEPARK && type != DOACTION && type != ADAPTIVEHOOK;
}

void TaskPlanCore::SetPathDir(const std::string &path_dir) {
    std::string dir = path_dir;
    if(!dir.empty() && dir.back() != '/') dir += '/';
    if(dir == mPathDir) return;
    if(mFenceRuntime && !mTaskList.empty()) RejectFenceTask("path directory changed during task");
    mPathDir = dir;
    mFenceGeometry.Clear();
    mFenceList.clear();
    mFenceBBox = FenceBBox();
    mPathTrajectoryCache.clear();
    mCachedPointCount = 0;
}

bool TaskPlanCore::LoadFenceFile(const std::string &path_dir) {
    SetPathDir(path_dir);
    const uint64_t version = mFenceGeometry.Version();
    if(!mFenceGeometry.Load(mPathDir + "fence.csv", mFenceError)) {
        mFenceList.clear();
        mPathTrajectoryCache.clear();
        mCachedPointCount = 0;
        return false;
    }
    if(version != mFenceGeometry.Version()) {
        mPathTrajectoryCache.clear();
        mCachedPointCount = 0;
        mFenceList.clear();
        for(const auto &point : mFenceGeometry.Points()) {
            XYZ_COOR_S value;
            value.x_axis = point.x; value.y_axis = point.y; value.heading = 0;
            mFenceList.push_back(value);
        }
    }
    return true;
}

bool TaskPlanCore::IsPointInFence(double px, double py) const {
    return mFenceGeometry.Contains(task_fence::Point(px, py));
}

const std::vector<TaskPlanCore::TrajectoryPoint> &TaskPlanCore::GetCachedPathTrajectory(const std::string &path_name) {
    // 总缓存至多 16 文件/200000 点；失败不永久负缓存，原子替换/同名编辑都能失效。
    const std::string file = mPathDir + path_name + ".csv";
    const auto stamp = task_fence::FileStamp::Read(file);
    auto found = mPathTrajectoryCache.find(path_name);
    if(found != mPathTrajectoryCache.end()) {
        if(stamp == found->second.stamp) return found->second.points;
        mCachedPointCount -= found->second.points.size();
        mPathTrajectoryCache.erase(found);
    }
    CachedTrajectory entry;
    std::ifstream input(file.c_str());
    std::string line;
    bool valid = stamp.valid && stamp.size <= 32 * 1024 * 1024 && input.is_open();
    while(valid && std::getline(input, line)) {
        path_csv::ROW_S row;
        bool normalized = false;
        const auto status = path_csv::NormalizeMapRow(line, row, normalized);
        if(status == path_csv::SKIP_ROW) continue;
        if(status != path_csv::VALID_ROW || entry.points.size() >= 200000) { valid = false; break; }
        TrajectoryPoint point;
        point.x_axis = row.values[0]; point.y_axis = row.values[1]; point.heading = row.values[2];
        if(!std::isfinite(point.x_axis) || !std::isfinite(point.y_axis) || !std::isfinite(point.heading)) {
            valid = false; break;
        }
        point.in_fence = IsPointInFence(point.x_axis, point.y_axis);
        if(!entry.points.empty()) {
            point.first_in_file = false;
            const auto &previous = entry.points.back();
            point.segment_in_fence = previous.in_fence && point.in_fence && mFenceGeometry.SegmentInside(
                task_fence::Point(previous.x_axis, previous.y_axis), task_fence::Point(point.x_axis, point.y_axis), true);
        }
        entry.points.push_back(point);
    }
    if(!valid || input.bad() || !(stamp == task_fence::FileStamp::Read(file))) {
        entry.points.clear();
        mFenceError = "path file missing, invalid or changed: " + path_name;
    } else entry.stamp = stamp;
    if(mPathTrajectoryCache.size() >= 16 || mCachedPointCount + entry.points.size() > 200000) {
        mPathTrajectoryCache.clear();
        mCachedPointCount = 0;
    }
    mCachedPointCount += entry.points.size();
    return mPathTrajectoryCache.emplace(path_name, std::move(entry)).first->second.points;
}

bool TaskPlanCore::ValidateTaskPoolWithFence(std::vector<TASKINFO_S> &task_pool) {
    bool loaded = false;
    double start_x = mNavData.xAxis, start_y = mNavData.yAxis;
    bool have_start = mNavData.received_time >= 0 && std::isfinite(start_x) && std::isfinite(start_y);
    for(auto &block : task_pool) {
        if(!FenceDriving(block.tTaskType)) {
            // 挂钩路径由 planning 动态生成，保留原 control 的 ADAPTIVEHOOK 例外。
            if(block.tTaskType == ADAPTIVEHOOK) {
                start_x = block.tXAxis; start_y = block.tYAxis; have_start = true;
            }
            continue;
        }
        if(!loaded && !LoadFenceFile(mPathDir)) return false;
        loaded = true;
        if(!std::isfinite(block.tXAxis) || !std::isfinite(block.tYAxis) || !std::isfinite(block.tAngle)) {
            mFenceError = "non-finite task destination"; return false;
        }
        if(block.task_id == 1000 && !IsPointInFence(block.tXAxis, block.tYAxis)) {
            mFenceError = "remote target outside fence"; return false;
        }
        std::vector<TrajectoryPoint> points;
        for(const auto &name : block.tPathList) {
            const auto &part = GetCachedPathTrajectory(name);
            if(part.empty()) return false;
            if(points.size() + part.size() > 200000) { mFenceError = "route point limit exceeded"; return false; }
            points.insert(points.end(), part.begin(), part.end());
        }
        if(points.size() < 20) { mFenceError = "route too short for planning"; return false; }
        uint64_t hash = task_fence::HashStart();
        size_t start = 0, stop = 4;
        double near_start = 1e100, near_stop = 1e100;
        for(size_t i = 0; i < points.size(); ++i) {
            const auto &p = points[i];
            hash = task_fence::HashPose(hash, p.x_axis, p.y_axis, p.heading);
            // 与 planning 原始起终点搜索相同；截停后使用明确索引，不再重选终点。
            if(have_start && i + 10 < points.size()) {
                const double d = std::hypot(p.x_axis - start_x, p.y_axis - start_y);
                if(d < near_start) { near_start = d; start = i; }
            }
            if(i >= 4) {
                const double d = std::hypot(p.x_axis - block.tXAxis, p.y_axis - block.tYAxis);
                if(d < near_stop) { near_stop = d; stop = i; }
            }
        }
        if(block.task_id == 1000 && near_stop > 2.5) {
            mFenceError = "remote target too far from route"; return false;
        }
        block.fenceRouteVersion = task_fence::HashWord(hash, points.size());
        if(start > stop || !points[start].in_fence) {
            mFenceError = "route start outside fence or beyond destination"; return false;
        }
        if(have_start && !mFenceGeometry.SegmentInside(task_fence::Point(start_x, start_y),
                                                       task_fence::Point(points[start].x_axis, points[start].y_axis))) {
            mFenceError = "vehicle-to-route connection crosses fence"; return false;
        }
        size_t end = stop;
        for(size_t i = start; i <= stop; ++i) {
            const auto &p = points[i];
            const bool safe = p.in_fence && (i == start || (!p.first_in_file ? p.segment_in_fence : mFenceGeometry.SegmentInside(
                task_fence::Point(points[i - 1].x_axis, points[i - 1].y_axis),
                task_fence::Point(p.x_axis, p.y_axis), true)));
            if(!safe) {
                if(i == start) { mFenceError = "no reachable route inside fence"; return false; }
                end = i - 1;
                block.fenceTruncated = true;
                break;
            }
        }
        if(block.fenceTruncated) {
            if(block.tTaskType != TRACKPATH) {
                mFenceError = "dynamic operation route cannot use a fence-truncated destination"; return false;
            }
            // 保留前轴角点余量；可行前缀不足时拒绝，绝不改选围栏外的第4点。
            while(end > start && !mFenceGeometry.PoseInside(points[end].x_axis, points[end].y_axis, points[end].heading)) --end;
            if(end - start + 1 < 20 || !mFenceGeometry.PoseInside(points[end].x_axis, points[end].y_axis, points[end].heading)) {
                mFenceError = "safe prefix too short to stop and plan"; return false;
            }
            block.tXAxis = points[end].x_axis;
            block.tYAxis = points[end].y_axis;
            block.tAngle = points[end].heading;
        }
        block.fenceStartIndex = start;
        block.fenceStopIndex = end;
        start_x = block.tXAxis; start_y = block.tYAxis; have_start = true;
        // 后续块不能在安全截停之后继续执行，也无需用这个非业务终点校验后续连接。
        if(block.fenceTruncated) break;
    }
    return true;
}

void TaskPlanCore::RejectFenceTask(const std::string &reason) {
    clearTaskPool();
    mFenceRejected = true;
    mFenceAck = false;
    mFenceComplete = false;
    mFenceGuard.stop = true;
    mFenceGuard.reason = reason;
    mFenceAlarm = 1;
    mTaskStatus.procedure = 2;
    mTaskStatus.fail_code = 2;
    mTaskStatus.fail_reason = reason;
    recived_cloud_task = true;
}

void TaskPlanCore::PrepareFenceGuard() {
    ++mFenceGuard.revision;
    mFenceAck = false;
    mFenceComplete = false;
    mPathPlanStatus.taskExecuStatus = NOTASK;
    mFenceGuard.task_key = task_fence::TaskKey(mTaskPlanData);
    mFenceGuard.fence_file = mFenceGeometry.File();
    mFenceGuard.fence_version = mFenceGeometry.Version();
    mFenceGuard.route_version = 0;
    mFenceGuard.truncated = false;
    mFenceGuard.start_index = mFenceGuard.stop_index = 0;
    if(mCurTaskNum < (int)mTaskList.size()) {
        const auto &block = mTaskList[mCurTaskNum];
        mFenceGuard.route_version = block.fenceRouteVersion;
        mFenceGuard.start_index = block.fenceStartIndex;
        mFenceGuard.stop_index = block.fenceStopIndex;
        mFenceGuard.truncated = block.fenceTruncated;
    }
    mFenceGuard.stop = mFenceRejected || (FenceDriving(mTaskPlanData.taskType) &&
                      (!mFenceGeometry.Valid() || (mFenceRuntime && !mFenceNavReady)));
    if(!mFenceRejected) mFenceGuard.reason = mFenceGuard.stop ? "fence or navigation unavailable" : "";
}

void TaskPlanCore::TickFence(double now) {
    const double age = now - mNavData.received_time;
    mFenceNavReady = mNavData.received_time >= 0 && std::isfinite(age) && age >= 0 && age <= 0.5 &&
                     std::isfinite(mNavData.xAxis) && std::isfinite(mNavData.yAxis) && std::isfinite(mNavData.heading);
    const bool known = mFenceNavReady && mFenceGeometry.Valid();
    const bool inside = known && mFenceGeometry.PoseInside(mNavData.xAxis, mNavData.yAxis, mNavData.heading);
    const bool driving = mCurTaskNum < (int)mTaskList.size() && FenceDriving(mTaskList[mCurTaskNum].tTaskType);
    if(driving && !recived_cloud_task && mFenceGuard.task_key != 0 &&
       mFenceGuard.fence_version != mFenceGeometry.Version() && !mFenceRejected)
        RejectFenceTask("fence file changed during task");
    if(driving && known && !inside && !mFenceRejected) RejectFenceTask("vehicle front corners outside fence");
    mFenceAlarm = mFenceRejected ? 1 : (!known ? -1 : (inside ? 0 : 1));
    mFenceGuard.stop = mFenceRejected || (driving && !known);
    if(!mFenceRejected) mFenceGuard.reason = mFenceGuard.stop ? "fence or navigation unavailable" : "";
}

void TaskPlanCore::SetFenceFeedback(uint64_t revision, uint64_t task_key, bool stop, const std::string &reason, bool completed) {
    if(revision != mFenceGuard.revision || task_key != mFenceGuard.task_key || mFenceRejected) return;
    if(stop) RejectFenceTask("planning fence check: " + reason);
    else { mFenceAck = true; mFenceComplete = completed; }
}
