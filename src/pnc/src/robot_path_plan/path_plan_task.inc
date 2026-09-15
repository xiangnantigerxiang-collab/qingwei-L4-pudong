// 任务生命周期：初始化、任务接收、跨任务状态复位。由 path_plan_comply.cpp 包含。

PathPlanComply::PathPlanComply() {
    // 跨帧历史窗口的初值与原函数内 static 声明保持一致
    history_risk_vec.assign(4, std::vector<robot::object>());
    unsafe_vec.assign(4, true);
    history_unsafe.assign(30, true);
}

void PathPlanComply::InitParameter() {
    mDesireSpeed = 0;
    mKeypoint = 0;
    InitSafetyCheck = 0;
    mDrivingPath.clear();
    mRcvGpsData = false;
    mVehicleData.curGear = GEAR_N;
    mTaskPlanData.workMode = MANUALCONTROLMODE;
    mTaskPlanData.desireSpeed = 0;
    mTaskPlanData.desireGear = GEAR_N;
    mPathPlanStatus.stopX = -1;
    mPathPlanStatus.stopY = -1;
    mPathPlanStatus.stopAngle = 0;
    mPathPlanStatus.taskExecuStatus = NOTASK;
    mHookPos.center_distance = 100;
    mHookPos.center_point_x = 0.0;
    mHookPos.center_point_y = 0.0;
    sysTime.init = ros::Time::now().toSec();
    this->loadAirCraftPorts();
}

void PathPlanComply::ResetParameter() {
    mDesireSpeed = 0;
    mKeypoint = 0;
    mDrivingPath.clear();
    mRcvGpsData = false;
    mTaskPlanData.desireSpeed = 0;
    mTaskPlanData.desireGear = GEAR_N;
    mPathPlanStatus.stopX = -1;
    mPathPlanStatus.stopY = -1;
    mPathPlanStatus.stopAngle = 0;
    mPathPlanStatus.taskExecuStatus = NOTASK;
}

void PathPlanComply::SetTaskPlanData(robot::task_plan_msg task_plan_t) {
    // task_plan_msg 是事件输入，不是每周期快照。每次收到消息都会重新加载
    // 机场配置和路径文件；必须先判断 task_id 是否切换，再覆盖旧任务数据。
    printf("========================设置任务: %d, id:%d===========================\n",
           int(task_plan_t.taskType), int(task_plan_t.task_id));
    printf("current task execustatus: %d, workmode: %d, tasktype: %d\n",
           mPathPlanStatus.taskExecuStatus, workMode, mTaskPlanData.taskType);
    bool is_take_new_task = true;
    if(int(mPathPlanStatus.taskExecuStatus) == 1) {
        if(!is_take_new_task) {
            printf("任务正在执行中,请勿设置新任务\n");
            return;
        } else {
            printf("舍弃当前未完成的任务，执行新任务\n");
        }
    }
    this->loadAirCraftPorts();  //test
    // if (JudgeTaskPlanMsgChanged(mTaskPlanData, task_plan_t))
    {
        printf("任务数据有更新, 重置关键点\n");
        sysTime.taskStart = ros::Time::now().toSec();
        mKeypoint = 0;
        mStopIndex = 0;
        // 任务切换判定须在覆盖 mTaskPlanData 之前(块推进消息 task_id 不变)
        bool task_switched = (task_plan_t.task_id != mTaskPlanData.task_id);
        mTaskPlanData = task_plan_t;
        workMode = task_plan_t.workMode;
        mPathPlanStatus.stopX = task_plan_t.stopX;
        mPathPlanStatus.stopY = task_plan_t.stopY;
        mPathPlanStatus.stopAngle = task_plan_t.stopAngle;
        InitSafetyCheck = 0;
        // 仅真实任务切换时复位跨帧任务状态（块推进保持滚动窗口）：
        // 否则同任务每个块边界都会重播观察窗（约 2.5 s 禁行）并抹掉云端暂停。
        if(task_switched) {
            ResetTaskHistory();
        }
        mDrivingPath.clear();
        mReferPath.desireSpeed = 0.0;
        mReferPath.x.clear();
        mReferPath.y.clear();
        if(task_plan_t.taskType == NOTHING) {
            mPathPlanStatus.taskExecuStatus = TASKFINISHED;
        } else {
            mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
        }

        printf("stopx:%f, stopy:%f, desired_speed:%f\n",
               mPathPlanStatus.stopX, mPathPlanStatus.stopY, mTaskPlanData.desireSpeed);
        calcuGlobalPath(task_plan_t);
        if(task_plan_t.task_id == go_task_id_) {  // 99去程,100回程
            updateAirPortStopIndex(mDrivingPath, true);
        }
        if(task_plan_t.task_id == back_task_id_) {  // 99去程,100回程
            updateAirPortStopIndex(mDrivingPath, false);
        }
    }
    // else
    // {
    //     printf("任务未改变\n");
    // }
    // handleDrivingPath();
}

void PathPlanComply::ResetTaskHistory() {
    // 仅用于 task_id 切换；不解除急停闩锁，也不用于同一任务的重复消息或块推进。
    history_risk_vec.assign(4, std::vector<robot::object>());
    safety_check_counter_percep = 0;
    safety_check_counter_refer = 0;
    HornFlag = 1;
    // 播种全 true = 保守方向: 新任务起步再观察(约 2.5s @10Hz)
    unsafe_vec.assign(4, true);
    history_unsafe.assign(30, true);
    backdist_flag = 0;
    mPerception.objs.clear();
    mPerceptionFrontScan.objs.clear();
    mPerceptionBackScan.objs.clear();
    command_state = 2;  // 上一任务的云端"暂停"不带入新任务
    if(emergencyStop == 1)
        printf("警告: 急停闩锁仍生效, 新任务将被压 0 速(需人工解除)\n");
    mReferPath.planspeed = 0.0;
    mReferPath.safety = 0;
}

void PathPlanComply::resetTask() {
    mPathPlanStatus.taskExecuStatus = NOTASK;
    mKeypoint = 0;
    mStopIndex = 0;
    InitSafetyCheck = 0;
    mDrivingPath.clear();
    mReferPath.desireSpeed = 0.0;
    mReferPath.planspeed = 0.0;
    mReferPath.x.clear();
    mReferPath.y.clear();
}

PathPlanComply::~PathPlanComply() {
}

#include "task/global_path.inc"
#include "task/operation_path.inc"
#include "task/airport_stop.inc"
#include "task/stop_speed.inc"
