#include "task_plan_comply.h"

TaskPlanComply taskPlanComply;

bool rcv_p_p_flag = false;
bool rcv_can_data = false;

ros::Publisher task_plan_pub;
ros::Publisher task_status_pub;
ros::Publisher v2nHeartBeat_pub;
ros::Publisher v2nCommandFeedback_pub;
ros::Publisher v2nRunningFeedback_pub;

robot::path_plan_status status_feedback;
robot::v2nCommandFeedback CF;

void PathPlanStatusCallBack(const robot::path_plan_status::ConstPtr &msg)
{
    rcv_p_p_flag = true;
    taskPlanComply.SetPathPlanStatus(*msg);
    status_feedback = *msg;
}

void CanMsgCallBack(const robot::can_msg::ConstPtr &msg)
{
    rcv_can_data = true;
    taskPlanComply.SetCanData(*msg);
}

void NavigationMsgCallBack(const robot::navigation_msg::ConstPtr &msg)
{
    taskPlanComply.SetNavigationData(*msg);
}

void TaskInfoMsgCallBack(const robot::TaskInfo::ConstPtr &msg)
{
    printf("=========received task info from cloud, task_id:%d============\n", msg->task_id);
    taskPlanComply.SetTaskInfo(*msg);
    // ROS_INFO("TaskInfoMsgCallBack ..........................,task id : %d", (int)msg->task_id);
}

void RemoteSignalMsgCallBack(const robot::RemoteSignal::ConstPtr &msg)
{
    taskPlanComply.SetRemoteSignal(*msg);
}

void RunningMsgCallBack(const robot::RunningMsg::ConstPtr &msg)
{
    auto info = taskPlanComply.mTaskInfoMsg;

    info.task_id = 1000;

    double lon = msg->values.toPoint.lon * 3.141592654 / 180.0;
    double lat = msg->values.toPoint.lat * 3.141592654 / 180.0;

    UTMCoor xy;
    LatlonToUtmXY(lon, lat, &xy);

    lon = xy.x - 238162.61964;
    lat = xy.y - 3539857.08307;

    taskPlanComply.RunningtXAxis = lon;
    taskPlanComply.RunningtYAxis = lat;
    taskPlanComply.RunningtAngle = msg->values.toPoint.heading;
    taskPlanComply.RunningFlag = 1;
    taskPlanComply.SetTaskInfo(info);

    taskPlanComply.cloudFeedbackStatus = "RUNNING";

    printf("Task Receive cloud Running Msg x = %lf y = %lf\n", lon, lat);
}

void CommandMsgCallBack(const robot::v2nCommandFeedback::ConstPtr &msg)
{
    taskPlanComply.CommandId = "0";
    taskPlanComply.cloudFeedbackStatus = "COMMAND:0";

    double speed = 100.0;

    CF.ready = 1;
    CF.commandState = msg->commandState; //0停车 1暂停2继续
    CF.commandID = msg->commandID;
    CF.limit = msg->limit;
    CF.speedCommand = msg->speedCommand;

    v2nCommandFeedback_pub.publish(CF);

    // speed = msg->speedCommand;
    int commandState = msg->commandState;
    if (commandState == 0) //停车
    {
        speed = 0.0;
        taskPlanComply.clearTaskPool();
    }else if(commandState == 1) //暂停
    {
        speed = 0.0;
    }else if(commandState == 2) //继续
    {
        //do noting;
    }
        

    ros::param::set("/cloud/suggestspeed", speed);
}

void WarningMsgCallBack(const robot::WarningMsg::ConstPtr &msg)
{
    return;
}

void NotifyMsgCallBack(const robot::NotifyMsg::ConstPtr &msg)
{
    return;
}

void T1Callback(const ros::TimerEvent &real)
{
    ros::WallTime wall_time = ros::WallTime::now();
    int year = wall_time.toBoost().date().year();
    int month = wall_time.toBoost().date().month();
    int day = wall_time.toBoost().date().day();
    int hour = wall_time.toBoost().time_of_day().hours();
    int minute = wall_time.toBoost().time_of_day().minutes();
    int second = wall_time.toBoost().time_of_day().seconds();

    hour += 8; // set timezone to beijing

    static int count = 0;
    static int second_last = 0;

    if (second != second_last)
    {
        count = 0;
        second_last = second;
    }
    else
        count += 1;

    std::string ts = std::to_string(year);

    if (month < 10)
        ts += "0";
    ts += std::to_string(month);

    if (day < 10)
        ts += "0";
    ts += std::to_string(day);

    if (hour < 10)
        ts += "0";
    ts += std::to_string(hour);

    if (minute < 10)
        ts += "0";
    ts += std::to_string(minute);

    if (second < 10)
        ts += "0";
    ts += std::to_string(second);

    ts = ts + "00" + std::to_string(count);

    std::string deviceId = "A03";
    std::string type = "heartBeat";
    std::string speedChange = "keep";

    double acc = taskPlanComply.mNavData.longitudinal_accelerate;
    if (acc > 0.02)
        speedChange = "accelerate";
    if (acc < -0.02)
        speedChange = "deaccelerate";
    robot::v2nHeartBeat HB;
    robot::v2nKeyPoint KP;
    HB.ts = ts;
    HB.deviceId = "A03";
    HB.type = "heartBeat";
    HB.values.status = taskPlanComply.cloudFeedbackStatus;
    HB.values.text = "";
    HB.values.lat = taskPlanComply.mNavData.lat;
    HB.values.lon = taskPlanComply.mNavData.lon;
    HB.values.heading = taskPlanComply.mNavData.heading - 90;
    HB.values.turning = status_feedback.turning;
    HB.values.speed = (status_feedback.curSpeed * 3.6);
    HB.values.power = taskPlanComply.mCanData.batteryPower;
    KP.lat = taskPlanComply.mNavData.lat;
    KP.lon = taskPlanComply.mNavData.lon;
    KP.heading = taskPlanComply.mNavData.heading;
    HB.values.remainingKeyPoints.push_back(KP);
    HB.values.drivingState = 0;
    if (HB.values.speed > 0.1)
        HB.values.drivingState = 3;
    if (acc > 0.02)
        HB.values.drivingState = 2;
    if (acc < -0.02)
        HB.values.drivingState = 1;

    int sensorstate = 0;
    ros::param::get("/planning/sensorstate", sensorstate);
    HB.values.sensorsState = sensorstate & 0x01;
    HB.values.lidarState = (sensorstate & 0x02) >> 1;
    HB.values.cameraState = (sensorstate & 0x04) >> 2;
    HB.values.gnssState = (sensorstate & 0x08) >> 3;
    int hookstate = 0;
    ros::param::get("/canbus/hookstate", hookstate);
    HB.values.vehicleState = 0;
    if (taskPlanComply.mCanData.faultCode.size() > 0)
    {
        HB.values.vehicleState = taskPlanComply.mCanData.faultCode[0];
    }

    HB.values.hookState = hookstate;
    HB.values.soc = taskPlanComply.mCanData.batteryPower;
    HB.values.chargingState = 0;
    v2nHeartBeat_pub.publish(HB);

    if (sensorstate > 0)
        ros::param::set("/cloud/suggestspeed", 0);
    // running feedback
    robot::v2nRunningFeedback RF;
    RF.ts = ts;
    RF.deviceId = "拖A0002";
    RF.type = "runningFeedback";
    RF.values.ts = ts;
    RF.values.status = "SUCCEED";
    RF.values.text = "";
    v2nRunningFeedback_pub.publish(RF);

    // command feedback
    CF.ts = ts;
    CF.deviceId = "拖A0002";
    CF.type = "commandFeedback";
    CF.values.ts = ts;
    CF.values.status = "SUCCEED";
    CF.values.text = "";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_plan_node");
    ros::NodeHandle nh;

    ros::Subscriber path_plan_sub = nh.subscribe("/path_plan_status", 1,
                                                 PathPlanStatusCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber can_msg_sub = nh.subscribe("/can_msg", 10,
                                               CanMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber navigation_sub = nh.subscribe("/navigation_msg", 1,
                                                  NavigationMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber task_info_sub = nh.subscribe("/cloud/task/task_info", 10,
                                                 TaskInfoMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber remote_signal_sub = nh.subscribe("/cloud/task/remote_signal", 10,
                                                     RemoteSignalMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber running_msg_sub = nh.subscribe("/cloud/msg/running_msg", 10,
                                                   RunningMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber command_msg_sub = nh.subscribe("/cloud/msg/command_msg", 10,
                                                   CommandMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber warning_msg_sub = nh.subscribe("/cloud/msg/warning_msg", 10,
                                                   WarningMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber notify_msg_sub = nh.subscribe("/cloud/msg/notify_msg", 10,
                                                  NotifyMsgCallBack, ros::TransportHints().tcpNoDelay());

    task_plan_pub = nh.advertise<robot::task_plan_msg>(
        "/task_plan_msg", 10);
    task_status_pub = nh.advertise<robot::TaskStatus>(
        "/cloud/task/task_status", 10);
    v2nHeartBeat_pub = nh.advertise<robot::v2nHeartBeat>(
        "/v2nHeartBeat", 10);
    v2nCommandFeedback_pub = nh.advertise<robot::v2nCommandFeedback>(
        "/v2nCommandFeedback", 10);
    v2nRunningFeedback_pub = nh.advertise<robot::v2nRunningFeedback>(
        "/v2nRunningFeedback", 10);

    ros::Timer T1 = nh.createTimer(ros::Duration(0.1), T1Callback);

    ros::Rate loop_rate(20);
    taskPlanComply.InitParameter();

    ros::param::set("/cloud/suggestspeed", 100.0);

    while (ros::ok())
    {
        ros::spinOnce();
        if (rcv_p_p_flag && rcv_can_data)
        {
            taskPlanComply.TaskPlanProcess();
        }

        // static int execute_task_num = -1;
        // printf("execute_task_num = %d, curTaskNum:%d\n", execute_task_num,taskPlanComply.mCurTaskNum);
        if (taskPlanComply.recived_cloud_task ||
            (taskPlanComply.mPathPlanStatus.taskExecuStatus == 2 &&
             taskPlanComply.mExecuteTaskNum != taskPlanComply.mCurTaskNum))
        {
            taskPlanComply.PublishTaskPlanMsg(task_plan_pub, task_status_pub);
            taskPlanComply.recived_cloud_task = false;
            taskPlanComply.mExecuteTaskNum = taskPlanComply.mCurTaskNum;
        }
        loop_rate.sleep();
    }

    return 0;
}
