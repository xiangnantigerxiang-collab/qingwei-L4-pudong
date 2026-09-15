#include "task_plan_core.h"

// 远程指车 UTM 原点(场地硬编码, 与驱动侧局部化偏移是两套并存——已知债务)
static const double kRemoteUtmOriginX = 238162.61964;
static const double kRemoteUtmOriginY = 3539857.08307;

void TaskPlanCore::SetPathPlanStatus(const PlanStatusIn &path_plan_t) {
    mPathPlanStatus = path_plan_t;
}

void TaskPlanCore::SetRemoteSignal(const RemoteSignalIn &remote_signal) {
    mRemoteSignal = remote_signal;
}

void TaskPlanCore::SetCanData(const CanStateIn &can_data) {
    mCanData = can_data;

    if(mTaskPool.size() == 0)
        return;

    // 自动模式下每帧都从任务池重新武装任务列表(切回自动即恢复任务)
    if(can_data.controlPanelState == 1)
        mTaskList = mTaskPool;
}

void TaskPlanCore::SetPositionLimits(
    const PositionLimitsIn &position_limits) {
    mPositionLimits = position_limits;
}

void TaskPlanCore::SetNavigationData(const NavStateIn &navigation_msg_t) {
    mNavData = navigation_msg_t;
}

void TaskPlanCore::clearTaskPool() {
    mCurTaskNum = 0;
    mExecuteTaskNum = -1;
    mTaskList.clear();
    mTaskPool.clear();
    mTimerCount = 0;
}

void TaskPlanCore::SetTaskInfo(const TaskInfoIn &task_info,
                               const std::string &task_file_dir) {
    // 与当前任务比对(以 task_id 为准; task_id=1000 远程指车豁免——
    // 每条 running_msg 都携带新目标点, 恒视为"有变化")
    bool task_running = (int)mTaskList.size() > 0 &&
                        mCurTaskNum < (int)mTaskList.size();
    if(task_running) {
        if(task_info.task_id == mTaskInfoMsg.task_id &&
           task_info.task_id != 1000) {
            // 无变化且仍在执行: 忽略, 不打断当前任务
            printf("任务未变化且正在执行, 忽略: id:%lld\n",
                   (long long)task_info.task_id);
            return;
        }
        // 有变化: 立即切换, 切换前清任务容器防上一任务残留
        printf("任务变化, 立即切换: %lld -> %lld\n",
               (long long)mTaskInfoMsg.task_id,
               (long long)task_info.task_id);
        mCurTaskNum = 0;
        mExecuteTaskNum = -1;
        mTaskList.clear();
        mTaskPool.clear();
    }
    // 无任务/已完毕/有变化 均到此装载——已完毕时同号任务允许重跑(特例)
    {
        // 凡装载即统一初始化(切换/重跑一致):
        // 旧任务的 TASKFINISHED 残留会让块完成判定假推进
        mTimerCount = 0;
        mPathPlanStatus.taskExecuStatus = NOTASK;
        mTaskStatus.procedure = 1;
        mTaskStatus.fail_code = 0;
        mTaskStatus.fail_reason = "";
        mTaskInfoMsg = task_info;

        std::string cs_task_id = std::to_string(task_info.task_id);
        std::string file = "task" + cs_task_id + ".yaml";

        mTaskPool = ParseTaskFile(file, task_info.task_id, task_file_dir);
        if(mTaskPool.empty()) {
            // 任务文件缺失/损坏被拒: 上报失败终态, 不让云端无限等待
            printf("任务文件被拒, 上报失败: id:%lld\n",
                   (long long)task_info.task_id);
            mTaskStatus.procedure = 2;
            mTaskStatus.fail_code = 1;
            mTaskStatus.fail_reason = "task file load failed";
            mCurTaskNum = 0;
            mTaskList.clear();
            recived_cloud_task = true;  // 触发一次失败状态+终态停车指令发布
            mTaskStatus.vehicle_id = task_info.vehicle_id;
            mTaskStatus.task_info = task_info;
            return;
        }
        printf("mTaskPool.size() = %d\n", (int)mTaskPool.size());
        for(int i = 0; i < (int)mTaskPool.size(); i++) {
            TASKINFO_S task_info = mTaskPool[i];
            printf("i:%d, type:%d, id:%lld action:%d, stopX,:%.1f stopY:%.1f, desireSpeed:%.1f\n",
                   i, task_info.tTaskType,
                   (long long)task_info.task_id, task_info.tSubAction,
                   task_info.tXAxis, task_info.tYAxis, task_info.tSpeed);
        }

        mCurTaskNum = 0;

        recived_cloud_task = true;
        mTaskList = mTaskPool;
    }

    if(task_info.task_id == 1000) {  // follow cloud command
        mTaskStatus.procedure = 1;
        mTaskInfoMsg = task_info;

        std::string cs_task_id = std::to_string(task_info.task_id);
        std::string file = "task" + cs_task_id + ".yaml";

        mTaskPool = ParseTaskFile(file, task_info.task_id, task_file_dir);

        // 空任务文件守卫(文件缺失/坏 yaml 被拒后 mTaskPool 为空,
        // 原 mTaskPool[0] 直接越界写是 UB)
        if(mTaskPool.size() > 0) {
            mTaskPool[0].tXAxis = RunningtXAxis;
            mTaskPool[0].tYAxis = RunningtYAxis;
            mTaskPool[0].tAngle = RunningtAngle;
        }

        mCurTaskNum = 0;
        recived_cloud_task = true;
        mTaskList = mTaskPool;
    }

    mTaskStatus.vehicle_id = mTaskInfoMsg.vehicle_id;
    mTaskStatus.task_info = mTaskInfoMsg;
}

void TaskPlanCore::InitParameter(const char *config_file) {
    // 加载结果无读者, 但保留加载动作: 配置文件缺失/损坏时启动即失败(原行为)
    mConfigFile = YAML::LoadFile(config_file);

    mTimerCount = 0;
    mCurTaskNum = 0;
    mTaskList.clear();
    mTaskPool.clear();

    mTaskInfoMsg.task_id = 0;
    mTaskStatus.procedure = 1;

    mTaskPlanData.workMode = MANUALCONTROLMODE;
    mTaskPlanData.taskType = NOTHING;
    mTaskPlanData.pathList.clear();
    mTaskPlanData.pathX.clear();
    mTaskPlanData.pathY.clear();
    mTaskPlanData.desireSpeed = 0;
    mTaskPlanData.stopX = -1;
    mTaskPlanData.stopY = -1;
    mTaskPlanData.stopAngle = 0;

    mTaskPlanData.desireGear = GEAR_N;
    mTaskPlanData.hookCmd = 0;
}

void TaskPlanCore::TaskPlanProcess(TaskPlanSink sink) {
    if(mCanData.controlPanelState == 0) {
        mTaskPlanData.workMode = MANUALCONTROLMODE;
        // 手动模式不 return(原 return 被注释): 状态机在手动模式下
        // 照常推进——原行为
    } else {
        mTaskPlanData.workMode = NOMALWORKINGMODE;
    }

    TaskManage(mCurTaskNum, mTaskList, sink);
}

void TaskPlanCore::PublishTaskPlanMsg(float max_vehicle_speed,
                                      TaskPlanSink sink) {
    int size = (int)mTaskList.size();

    if(size != 0 && mCurTaskNum < size) {
        TASKINFO_S task_info = mTaskList.at(mCurTaskNum);
        mTaskPlanData.taskType = task_info.tTaskType;
        mTaskPlanData.pathList = task_info.tPathList;
        mTaskPlanData.pathX.clear();
        mTaskPlanData.pathY.clear();
        mTaskPlanData.pathAngle.clear();

        if(max_vehicle_speed > task_info.tSpeed) {
            mTaskPlanData.desireSpeed = task_info.tSpeed;
        } else {
            mTaskPlanData.desireSpeed = max_vehicle_speed;
        }

        mTaskPlanData.stopX = task_info.tXAxis;
        mTaskPlanData.stopY = task_info.tYAxis;
        mTaskPlanData.stopAngle = task_info.tAngle;
        mTaskPlanData.desireGear = task_info.tGear;
        mTaskPlanData.hookCmd = task_info.tSubAction;
        mTaskPlanData.task_id = task_info.task_id;
    } else {
        // 终态(全部完成/空任务池): 清干净再发布, 防 path_plan 把上一块
        // 残留字段当新任务重载(重读 CSV/置回执行中)
        mTaskPlanData.taskType = NOTHING;
        mTaskPlanData.pathList.clear();
        mTaskPlanData.stopX = -1;
        mTaskPlanData.stopY = -1;
        mTaskPlanData.stopAngle = 0;
        mTaskPlanData.hookCmd = 0;
        mTaskPlanData.task_id = 0;
        mTaskPlanData.desireSpeed = 0;
        mTaskPlanData.desireGear = GEAR_N;
    }
    if(mCurTaskNum >= size) {
        printf("-----当前任务块已完成------\n");
        mTaskPlanData.desireSpeed = 0;
        mTaskPlanData.desireGear = GEAR_N;
    } else {
        printf(">>>> task size:%d,cur_num:%d task type:%d, hookcmd:%d <<<<<<\n", size, mCurTaskNum,
               mTaskPlanData.taskType, mTaskPlanData.hookCmd);
    }
    // 完成时同样发布: 0 速/N 挡终态指令 + procedure=2 状态
    // (原实现在此 return, 云端永远收不到任务完成)
    emitPublish(sink, TP_PUBLISH_TASK_PLAN);
    emitPublish(sink, TP_PUBLISH_TASK_STATUS);
}

void TaskPlanCore::ClearTASKINFO_S(TASKINFO_S &tTaskInfo) {
    tTaskInfo.tTaskType = NOTHING;
    tTaskInfo.tPathList.clear();
    tTaskInfo.tPathX.clear();
    tTaskInfo.tPathY.clear();
    tTaskInfo.tSpeed = 0;
    tTaskInfo.tXAxis = 0;
    tTaskInfo.tYAxis = 0;
    tTaskInfo.tAngle = 0;
    tTaskInfo.tGear = NO_OPERATION;
    tTaskInfo.tSubAction = NO_OPERATION;
}

std::vector<TASKINFO_S> TaskPlanCore::ParseTaskFile(
    std::string file_name, int64_t task_id, const std::string &task_file_dir) {
    std::string task_file = "";
    YAML::Node task_node;
    TASKINFO_S task_info;
    std::vector<TASKINFO_S> rtn_task;
    int task_num = 0;

    task_file = task_file_dir;
    task_file += file_name;
    try {
        task_node = YAML::LoadFile(task_file.c_str());
        task_num = task_node["task_sum"].as<int>();

        for(int i = 0; i < task_num; ++i) {
            ClearTASKINFO_S(task_info);
            std::string key = "task" + std::to_string(i) + "_task_type";
            task_info.tTaskType = (uint8_t)task_node[key].as<int>();
            key = "task" + std::to_string(i) + "_path_list";
            task_info.tPathList =
                pubalgor.SplitString(task_node[key].as<std::string>(), ",");
            key = "task" + std::to_string(i) + "_desire_speed";
            task_info.tSpeed = task_node[key].as<float>();
            key = "task" + std::to_string(i) + "_stop_x";
            task_info.tXAxis = task_node[key].as<float>();
            key = "task" + std::to_string(i) + "_stop_y";
            task_info.tYAxis = task_node[key].as<float>();
            key = "task" + std::to_string(i) + "_stop_angle";
            task_info.tAngle = task_node[key].as<float>();
            key = "task" + std::to_string(i) + "_gear";
            task_info.tGear = (uint8_t)task_node[key].as<int>();
            key = "task" + std::to_string(i) + "_subaction";
            task_info.tSubAction = (uint8_t)task_node[key].as<int>();
            task_info.task_id = task_id;
            rtn_task.push_back(task_info);
        }
    } catch(const std::exception &e) {
        // 任务文件缺失/字段坏: 告警并拒绝该任务(返回空表), 不让节点崩溃
        printf("任务文件加载失败, 拒绝该任务: %s (%s)\n",
               task_file.c_str(), e.what());
        rtn_task.clear();
        return rtn_task;
    }

    return rtn_task;
}

bool TaskPlanCore::OperationStateJudge(TASKINFO_S tTask) {
    bool rtn_value = 0;

    switch(tTask.tSubAction) {
        case NO_OPERATION:
            rtn_value = 1;
            break;
        case HOOKOPERATION:
            // canbus 状态机判"销至顶端"(驱动入 min±10 带 1 秒)→任务完成
            if(mCanData.hookStatus == ACTUATOR_UP_END &&
               mPathPlanStatus.taskExecuStatus == TASKFINISHED)
                rtn_value = 1;
            break;
        case DECOUPLING:
            // canbus 状态机判"销至底端"(驱动入 max±10 带 1 秒)→任务完成
            if(mCanData.hookStatus == ACTUATOR_DOWN_END &&
               mPathPlanStatus.taskExecuStatus == TASKFINISHED)
                rtn_value = 1;
            // if (mCanData.linkPallet == 1)
            //     rtn_value = true;
            break;
        case UNLOAD_LOWER:
            rtn_value = 1;
            break;
        case WAITING:
            if(mRemoteSignal.isfull == 1)
                rtn_value = 1;
            break;
        default:
            break;
    }

    return rtn_value;
}

bool TaskPlanCore::JudgeArrivedDestination(TASKINFO_S tTask) {
    // printf("executestatus: %d\n", mPathPlanStatus.taskExecuStatus);
    if(mPathPlanStatus.taskExecuStatus != TASKFINISHED)
        return 0;

    return 1;
}

void TaskPlanCore::TaskManage(int &tCurTaskNum,
                              std::vector<TASKINFO_S> tTaskList,
                              TaskPlanSink sink) {
    int size = (int)tTaskList.size();
    uint8_t task_type = 0;
    TASKINFO_S task_info;

    if(mTimerCount > 500)
        mTimerCount = 500;

    if(0 == size || tCurTaskNum >= size) {  // 任务列表为空
        mTaskPlanData.desireSpeed = 0;
        return;
    }
    // 获取当前任务信息
    task_info = tTaskList.at(tCurTaskNum);
    task_type = task_info.tTaskType;
    mTaskPlanData.desireSpeed = task_info.tSpeed;

    // action name && action finished

    if(task_type == DOACTION) {  // 执行动作的任务
        int state = OperationStateJudge(tTaskList.at(tCurTaskNum));
        if(state == 1 && (mTimerCount++ > 50)) {
            mTimerCount = 0;
            tCurTaskNum++;
        }
    } else {
        // 123456任务
        //  TRACKPATH = 1,
        // ADAPTIVELOADPATH = 2,
        // ADAPTIVEUNLOADPATH = 3,
        // ADAPTIVEHOOK = 4,
        // ADAPTIVEBACK = 5,
        // DOACTION = 6,
        // ADAPTIVEPARK = 7,
        if(task_type >= TRACKPATH && task_type <= ADAPTIVEPARK) {
            int state = JudgeArrivedDestination(tTaskList.at(tCurTaskNum));  // 检查是否达到终点
            if(state == 1 && (mTimerCount++ > 20)) {
                mTimerCount = 0;
                tCurTaskNum++;
            }
        } else {
            tCurTaskNum++;
            mTimerCount = 0;
            std::string text = "[TaskPlan][Change gear [" +
                               std::to_string((int)task_info.tGear) + "]]";
            emitLogInfo(sink, text.c_str());
        }
    }

    // 全部任务块完成时才置"已完成"(原 == size-1 会在进入末块时就提前上报)
    if(tCurTaskNum >= size) {
        mTaskStatus.procedure = 2;
    }
}

void TaskPlanCore::OnRunningMsg(const RunningTargetIn &target,
                                const std::string &task_file_dir) {
    TaskInfoIn info = mTaskInfoMsg;

    info.task_id = 1000;

    double lon = target.lon * 3.141592654 / 180.0;
    double lat = target.lat * 3.141592654 / 180.0;

    UTMCoor xy;
    LatlonToUtmXY(lon, lat, &xy);

    lon = xy.x - kRemoteUtmOriginX;
    lat = xy.y - kRemoteUtmOriginY;

    RunningtXAxis = lon;
    RunningtYAxis = lat;
    RunningtAngle = target.heading;
    SetTaskInfo(info, task_file_dir);

    cloudFeedbackStatus = "RUNNING";

    printf("Task Receive cloud Running Msg x = %lf y = %lf\n", lon, lat);
}

void TaskPlanCore::OnCommandMsg(const CommandIn &cmd, TaskPlanSink sink) {
    cloudFeedbackStatus = "COMMAND:0";

    double speed = 100.0;

    mCommandFbData.ready = 1;
    mCommandFbData.commandState = cmd.commandState;  // 0停车 1暂停2继续
    mCommandFbData.commandID = cmd.commandID;
    mCommandFbData.limit = cmd.limit;
    mCommandFbData.speedCommand = cmd.speedCommand;

    emitPublish(sink, TP_PUBLISH_COMMAND_FB);

    // speed = msg->speedCommand;
    int commandState = cmd.commandState;
    if(commandState == 0) {  // 停车
        speed = 0.0;
        clearTaskPool();
    } else if(commandState == 1) {  // 暂停
        speed = 0.0;
    } else if(commandState == 2) {  // 继续
        // do noting;
    }

    emitParamDouble(sink, "/cloud/suggestspeed", speed);
}

void TaskPlanCore::RunHeartBeat(const HeartBeatInputs &in, TaskPlanSink sink) {
    int year = in.year;
    int month = in.month;
    int day = in.day;
    int hour = in.hour;
    int minute = in.minute;
    int second = in.second;

    hour += 8;   // set timezone to beijing
    hour %= 24;  // 回卷(仅保证 hour 字段合法, 跨日不进位日期)

    if(second != mHbSecondLast) {
        mHbCount = 0;
        mHbSecondLast = second;
    } else {
        mHbCount += 1;
    }

    std::string ts = std::to_string(year);

    if(month < 10)
        ts += "0";
    ts += std::to_string(month);

    if(day < 10)
        ts += "0";
    ts += std::to_string(day);

    if(hour < 10)
        ts += "0";
    ts += std::to_string(hour);

    if(minute < 10)
        ts += "0";
    ts += std::to_string(minute);

    if(second < 10)
        ts += "0";
    ts += std::to_string(second);

    ts = ts + "00" + std::to_string(mHbCount);

    double acc = mNavData.longitudinal_accelerate;

    HeartBeatOut HB;
    HB.ts = ts;
    HB.deviceId = "A03";
    HB.type = "heartBeat";
    HB.status = cloudFeedbackStatus;
    HB.text = "";
    HB.lat = mNavData.lat;
    HB.lon = mNavData.lon;
    HB.heading = mNavData.heading - 90;
    HB.turning = mPathPlanStatus.turning;
    HB.speed = (mPathPlanStatus.curSpeed * 3.6);
    HB.power = mCanData.batteryPower;
    KeyPointOut KP;
    KP.lat = mNavData.lat;
    KP.lon = mNavData.lon;
    KP.heading = mNavData.heading;
    HB.remainingKeyPoints.push_back(KP);
    HB.drivingState = 0;
    if(HB.speed > 0.1)
        HB.drivingState = 3;
    if(acc > 0.02)
        HB.drivingState = 2;
    if(acc < -0.02)
        HB.drivingState = 1;

    int sensorstate = in.sensorstate;
    HB.sensorsState = sensorstate & 0x01;
    HB.lidarState = (sensorstate & 0x02) >> 1;
    HB.cameraState = (sensorstate & 0x04) >> 2;
    HB.gnssState = (sensorstate & 0x08) >> 3;
    // can_msg.hookStatus 映射云端契约(v2nHeartBeatValue: 4-已挂钩/3-未挂钩/
    // 2-挂钩异常/1-脱钩告警): 4(up end=挂牢)/3(down end=脱开)直传,
    // 1(block=堵转)报 2, 其余 0(原 /canbus/hookstate param 已随状态机下线)
    int hookstate = 0;
    if(mCanData.hookStatus == ACTUATOR_UP_END ||
       mCanData.hookStatus == ACTUATOR_DOWN_END) {
        hookstate = mCanData.hookStatus;
    } else if(mCanData.hookStatus == ACTUATOR_BLOCK) {
        hookstate = 2;  // 堵转按"挂钩异常"上报
    }
    HB.vehicleState = 0;
    if(mCanData.faultCode.size() > 0) {
        HB.vehicleState = mCanData.faultCode[0];
    }

    HB.hookState = hookstate;
    HB.soc = mCanData.batteryPower;
    HB.chargingState = 0;
    mHeartBeatData = HB;

    emitPublish(sink, TP_PUBLISH_HEARTBEAT);

    if(sensorstate > 0)
        emitParamInt(sink, "/cloud/suggestspeed", 0);

    // running feedback
    RunningFbOut RF;
    RF.ts = ts;
    RF.deviceId = "拖A0002";
    RF.type = "runningFeedback";
    RF.vTs = ts;
    RF.vStatus = "SUCCEED";
    RF.vText = "";
    mRunningFbData = RF;

    emitPublish(sink, TP_PUBLISH_RUNNING_FB);

    // command feedback(只刷新时间戳, 不在此发布——发布仅发生在指令到达时)
    mCommandFbData.ts = ts;
    mCommandFbData.deviceId = "拖A0002";
    mCommandFbData.type = "commandFeedback";
    mCommandFbData.vTs = ts;
    mCommandFbData.vStatus = "SUCCEED";
    mCommandFbData.vText = "";
}

// ---- 事件发射 ----
void TaskPlanCore::emitPublish(TaskPlanSink sink, TaskPlanEventType type) {
    if(!sink)
        return;
    TaskPlanEvent ev;
    ev.type = type;
    sink(ev);
}

void TaskPlanCore::emitParamInt(TaskPlanSink sink, const char *key,
                                int value) {
    if(!sink)
        return;
    TaskPlanEvent ev;
    ev.type = TP_PARAM_INT;
    ev.paramKey = key;
    ev.paramInt = value;
    sink(ev);
}

void TaskPlanCore::emitParamDouble(TaskPlanSink sink, const char *key,
                                   double value) {
    if(!sink)
        return;
    TaskPlanEvent ev;
    ev.type = TP_PARAM_DOUBLE;
    ev.paramKey = key;
    ev.paramDouble = value;
    sink(ev);
}

void TaskPlanCore::emitLogInfo(TaskPlanSink sink, const char *text) {
    if(!sink)
        return;
    TaskPlanEvent ev;
    ev.type = TP_LOG_INFO;
    ev.text = text;
    sink(ev);
}
