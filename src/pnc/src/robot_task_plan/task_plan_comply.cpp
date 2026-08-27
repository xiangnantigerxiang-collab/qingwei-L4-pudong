#include "task_plan_comply.h"

void TaskPlanComply::SetPathPlanStatus(robot::path_plan_status path_plan_t)
{
    mPathPlanStatus = path_plan_t;
}

void TaskPlanComply::SetRemoteSignal(robot::RemoteSignal remote_signal)
{
    mRemoteSignal = remote_signal;
}

void TaskPlanComply::SetCanData(robot::can_msg can_data)
{
    mCanData = can_data;

    if (mTaskPool.size() == 0)
        return;

    if (can_data.controlPanelState == 1)
        mTaskList = mTaskPool;
}

void TaskPlanComply::SetNavigationData(robot::navigation_msg navigation_msg_t)
{
    mNavData = navigation_msg_t;
}

bool TaskPlanComply::IsChangeTask(
    robot::TaskInfo task_info_last, robot::TaskInfo task_info_new)
{
    printf("wait_task:%d, last_id:%d, new_id:%d\n", wait_task, task_info_last.task_id, task_info_new.task_id);
    if (wait_task == 0)
        return 0;

    auto A = task_info_last.task_id;
    auto B = task_info_new.task_id;

    if (A == 0 && B != 0)
        return 1;
    if (A != 0 && A != B)
        return 1;

    return 0;
}

void TaskPlanComply::clearTaskPool()
{
    mCurTaskNum = 0;
    mExecuteTaskNum = -1;
    mTaskList.clear();
    mTaskPool.clear();
    mTimerCount = 0;
}

void TaskPlanComply::SetTaskInfo(robot::TaskInfo task_info)
{
    bool is_take_new_task = true; // 如果有正在执行的任务，是否替换为新任务
    if (mCurTaskNum < mTaskList.size())
    {
        printf("存在未执行完的任务，替换新任务\n");
        if (!is_take_new_task) // 标志为false，则不允许替换
        {
            printf("存在未执行完的任务，不允许执行新任务\n");
            return;
        }
        else if (mCanData.vehicleSpeed > 0.5) // 速度>0.5不允许替换
        {
            printf("速度>0.5m/s, 不允许执行新任务\n");
            return;
        }
        else
        {
            printf("丢弃当前任务，执行新任务\n");
            mCurTaskNum = 0;
            mExecuteTaskNum = -1;
            mTaskList.clear();
            mTaskPool.clear();
            mTimerCount = 0;
        }
    }
    // if (IsChangeTask(mTaskInfoMsg, task_info))
    {
        mTaskStatus.procedure = 1;
        mTaskInfoMsg = task_info;

        std::string cs_task_id = std::to_string(task_info.task_id);
        std::string file = "task" + cs_task_id + ".yaml";

        mTaskPool = ParseTaskFile(file, task_info.task_id);
        printf("mTaskPool.size() = %d\n", mTaskPool.size());
        for (int i = 0; i < mTaskPool.size(); i++)
        {
            auto task_info = mTaskPool[i];
            printf("i:%d, type:%d, id:%d action:%d, stopX,:%.1f stopY:%.1f, desireSpeed:%.1f\n",
                   i, task_info.tTaskType, task_info.task_id, task_info.tSubAction,
                   task_info.tXAxis, task_info.tYAxis, task_info.tSpeed);
        }
        // for (auto task_info : mTaskPool)
        // {
        //     ROS_INFO("task:%u, %d, %.1f, %.3f, %.3f, %.3f, %u, %u",
        //              task_info.tTaskType,
        //              (int)task_info.tPathList.size(),
        //              task_info.tSpeed,
        //              task_info.tXAxis,
        //              task_info.tYAxis,
        //              task_info.tAngle,
        //              task_info.tGear,
        //              task_info.tSubAction);
        // }

        mCurTaskNum = 0;

        recived_cloud_task = true;
        mTaskList = mTaskPool;
    }
    // else
    // {
    //     printf("相同的任务, 跳过当前任务块\n");
    // }

    if (task_info.task_id == 1000)
    { // follow cloud command
        mTaskStatus.procedure = 1;
        mTaskInfoMsg = task_info;

        std::string cs_task_id = std::to_string(task_info.task_id);
        std::string file = "task" + cs_task_id + ".yaml";

        mTaskPool = ParseTaskFile(file, task_info.task_id);

        mTaskPool[0].tXAxis = RunningtXAxis;
        mTaskPool[0].tYAxis = RunningtYAxis;
        mTaskPool[0].tAngle = RunningtAngle;

        mCurTaskNum = 0;
        recived_cloud_task = true;
        mTaskList = mTaskPool;
    }

    mTaskStatus.vehicle_id = mTaskInfoMsg.vehicle_id;
    mTaskStatus.task_info = mTaskInfoMsg;
}

void TaskPlanComply::InitParameter()
{
    std::string config_file = "";

    ros::param::get("config_file", config_file);

    mConfigFile = YAML::LoadFile(config_file.c_str());

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

    wait_task = 1;
}

void TaskPlanComply::TaskPlanProcess()
{
    if (mCanData.controlPanelState == 0)
    {
        mTaskPlanData.workMode = MANUALCONTROLMODE;
        // ROS_INFO("Manual mode...");
        // return;
    }
    else
        mTaskPlanData.workMode = NOMALWORKINGMODE;

    TaskManage(mCurTaskNum, mTaskList);
}

void TaskPlanComply::PublishTaskPlanMsg(
    ros::Publisher tPub1, ros::Publisher tPub2)
{
    int size = mTaskList.size();

    float max_vehicle_speed = 0;
    ros::param::get("max_vehicle_speed", max_vehicle_speed);
    if (size != 0 && mCurTaskNum < size)
    {
        TASKINFO_S task_info = mTaskList.at(mCurTaskNum);
        mTaskPlanData.taskType = task_info.tTaskType;
        mTaskPlanData.pathList = task_info.tPathList;
        mTaskPlanData.pathX.clear();
        mTaskPlanData.pathY.clear();
        mTaskPlanData.pathAngle.clear();

        if (max_vehicle_speed > task_info.tSpeed)
        {
            mTaskPlanData.desireSpeed = task_info.tSpeed;
        }
        else
            mTaskPlanData.desireSpeed = max_vehicle_speed;

        mTaskPlanData.stopX = task_info.tXAxis;
        mTaskPlanData.stopY = task_info.tYAxis;
        mTaskPlanData.stopAngle = task_info.tAngle;
        mTaskPlanData.desireGear = task_info.tGear;
        mTaskPlanData.hookCmd = task_info.tSubAction;
        mTaskPlanData.task_id = task_info.task_id;
    }
    else
    {
        mTaskPlanData.desireSpeed = 0;
        mTaskPlanData.desireGear = GEAR_N;
    }
    if (mCurTaskNum >= size)
    {
        printf("-----当前任务块已完成------\n");
        mTaskPlanData.desireSpeed = 0;
        mTaskPlanData.desireGear = GEAR_N;
        return;
    }
    else
    {
        printf(">>>> task size:%d,cur_num:%d task type:%d, hookcmd:%d <<<<<<\n", size, mCurTaskNum,
               mTaskPlanData.taskType, mTaskPlanData.hookCmd);
    }
    tPub1.publish(mTaskPlanData);
    tPub2.publish(mTaskStatus);
}

void TaskPlanComply::ClearTASKINFO_S(TASKINFO_S &tTaskInfo)
{
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

std::vector<TASKINFO_S> TaskPlanComply::ParseTaskFile(std::string file_name, int task_id)
{
    std::string task_file = "";
    YAML::Node task_node;
    TASKINFO_S task_info;
    std::vector<TASKINFO_S> rtn_task;
    int task_num = 0;
    char buffer[100] = {0};

    ros::param::get("task_file", task_file);

    task_file += file_name;
    task_node = YAML::LoadFile(task_file.c_str());
    task_num = task_node["task_sum"].as<int>();

    // ROS_INFO("task_num:%d", task_num);

    for (int i = 0; i < task_num; ++i)
    {
        ClearTASKINFO_S(task_info);
        memset(buffer, 0, sizeof(buffer));
        sprintf(buffer, "task%d_task_type", i);
        task_info.tTaskType = (uint8_t)task_node[buffer].as<int>();
        memset(buffer, 0, sizeof(buffer));
        sprintf(buffer, "task%d_path_list", i);
        task_info.tPathList = pubalgor.SplitString(task_node[buffer].as<string>(), ",");
        memset(buffer, 0, sizeof(buffer));
        sprintf(buffer, "task%d_desire_speed", i);
        task_info.tSpeed = task_node[buffer].as<float>();
        memset(buffer, 0, sizeof(buffer));
        sprintf(buffer, "task%d_stop_x", i);
        task_info.tXAxis = task_node[buffer].as<float>();
        memset(buffer, 0, sizeof(buffer));
        sprintf(buffer, "task%d_stop_y", i);
        task_info.tYAxis = task_node[buffer].as<float>();
        memset(buffer, 0, sizeof(buffer));
        sprintf(buffer, "task%d_stop_angle", i);
        task_info.tAngle = task_node[buffer].as<float>();
        memset(buffer, 0, sizeof(buffer));
        sprintf(buffer, "task%d_gear", i);
        task_info.tGear = (uint8_t)task_node[buffer].as<int>();
        memset(buffer, 0, sizeof(buffer));
        sprintf(buffer, "task%d_subaction", i);
        task_info.tSubAction = (uint8_t)task_node[buffer].as<int>();
        task_info.task_id = task_id;
        rtn_task.push_back(task_info);
    }

    return rtn_task;
}

bool TaskPlanComply::OperationStateJudge(TASKINFO_S tTask)
{
    bool rtn_value = 0;

    switch (tTask.tSubAction)
    {
    case NO_OPERATION:
        rtn_value = 1;
        break;
    case HOOKOPERATION:
        if (mCanData.epsERR1 < 185 && mPathPlanStatus.taskExecuStatus == TASKFINISHED)
            rtn_value = 1;
        break;
    case DECOUPLING:
        if (mCanData.epsERR1 > 250 && mPathPlanStatus.taskExecuStatus == TASKFINISHED)
            rtn_value = 1;
        // if (mCanData.linkPallet == 1)
        //     rtn_value = true;
        break;
    case UNLOAD_LOWER:
        rtn_value = 1;
        break;
    case WAITING:
        if (mRemoteSignal.isfull == 1)
            rtn_value = 1;
        break;
    default:
        break;
    }

    return rtn_value;
}

bool TaskPlanComply::JudgeArrivedDestination(TASKINFO_S tTask)
{
    // printf("executestatus: %d\n", mPathPlanStatus.taskExecuStatus);
    if (mPathPlanStatus.taskExecuStatus != TASKFINISHED)
        return 0;

    RunningStatus = 1;
    return 1;
}

void TaskPlanComply::TaskManage(
    int &tCurTaskNum, std::vector<TASKINFO_S> tTaskList)
{
    int size = tTaskList.size();
    uint8_t sub_action = 0;
    uint8_t task_type = 0;
    TASKINFO_S task_info;

    if (mTimerCount > 500)
        mTimerCount = 500;

    if (0 == size || tCurTaskNum >= size) // 任务列表为空
    {
        mTaskPlanData.desireSpeed = 0;
        wait_task = true;
        return;
    }
    // 获取当前任务信息
    task_info = tTaskList.at(tCurTaskNum);
    task_type = task_info.tTaskType;
    mTaskPlanData.desireSpeed = task_info.tSpeed;

    // action name && action finished
    // ROS_INFO("task number :%d, task size :%d", tCurTaskNum, size);

    if (task_type == DOACTION) // 执行动作的任务
    {
        // printf("before task_num:%d\n",  tCurTaskNum);
        int state = OperationStateJudge(tTaskList.at(tCurTaskNum));
        // printf("do action state: %d, mTimerCount: %d, task_num: %d\n", state, mTimerCount, tCurTaskNum);
        if (state == 1 && (mTimerCount++ > 50))
        {
            mTimerCount = 0;
            tCurTaskNum++;
            // ROS_INFO("[TaskPlan][action is finished,change next task[%d][%d]]", tCurTaskNum, size);
        }
        // printf("action after state = %d, task_num:%d\n", state, tCurTaskNum);
    }
    else
    {
        // 123456任务
        //  TRACKPATH = 1,
        // ADAPTIVELOADPATH = 2,
        // ADAPTIVEUNLOADPATH = 3,
        // ADAPTIVEHOOK = 4,
        // ADAPTIVEBACK = 5,
        // DOACTION = 6,
        // ADAPTIVEPARK = 7,
        if (task_type >= TRACKPATH && task_type <= ADAPTIVEPARK)
        {
            int state = JudgeArrivedDestination(tTaskList.at(tCurTaskNum)); // 检查是否达到终点
                                                                            //    printf("before task_num:%d\n",  tCurTaskNum);
            if (state == 1 && (mTimerCount++ > 20))
            {
                mTimerCount = 0;
                tCurTaskNum++;
                // ROS_INFO("[TaskPlan][Arrive and change task[%d][%d]]", tCurTaskNum, size);
            }
            //  printf("path state = %d, task_num:%d\n", state, tCurTaskNum);
        }
        else
        {
            tCurTaskNum++;
            mTimerCount = 0;
            ROS_INFO("[TaskPlan][Change gear [%d]]", task_info.tGear);
        }
    }

    wait_task = false;
    // 检查是否已完成所有任务
    if (tCurTaskNum == (size - 1))
    {
        wait_task = true;
        mTaskStatus.procedure = 2;
    }
}
