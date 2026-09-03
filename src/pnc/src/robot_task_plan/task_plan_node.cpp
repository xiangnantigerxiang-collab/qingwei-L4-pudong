// ===========================================================================
// task_plan_node - TaskPlanCore 的 ROS 适配层。
//
// 本文件只放 ROS 粘合: 话题订阅/发布、参数服务器访问、定时器与时钟。
// 全部任务业务逻辑在 task_plan_core.cpp, 后者不依赖 ROS。
//
// 接线(与旧实现一致, 话题全部绝对名):
//   subscribe /path_plan_status       (下游块完成反馈)
//   subscribe /can_msg                (车辆 CAN 状态)
//   subscribe /navigation_msg         (定位)
//   subscribe /cloud/task/task_info   (云端任务派发)
//   subscribe /cloud/task/remote_signal(等待区信号)
//   subscribe /cloud/msg/running_msg  (远程指车)
//   subscribe /cloud/msg/command_msg  (云端指令 停/暂停/继续)
//   subscribe /cloud/msg/warning_msg / notify_msg(空回调, 仅占位)
//   publish   /task_plan_msg          (当前任务块 -> path_plan)
//   publish   /cloud/task/task_status (任务状态 -> 云端)
//   publish   /v2nHeartBeat /v2nCommandFeedback /v2nRunningFeedback
//   param in : task_file, config_file, max_vehicle_speed(热读),
//              /planning/sensorstate, /canbus/hookstate
//   param out: /cloud/suggestspeed
//
// 定时器 10Hz 心跳; 主循环 20Hz(发布门控与旧实现相同)。
// 全部回调运行在单线程 spinner, 无需加锁。
// ===========================================================================
#include <string>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <ros/ros.h>

#include "robot/TaskInfo.h"
#include "robot/TaskStatus.h"
#include "robot/can_msg.h"
#include "robot/path_plan_status.h"
#include "robot/task_plan_msg.h"
#include "robot/RemoteSignal.h"
#include "robot/navigation_msg.h"
#include "robot/v2nHeartBeat.h"
#include "robot/v2nCommandFeedback.h"
#include "robot/v2nRunningFeedback.h"
#include "robot/RunningMsg.h"
#include "robot/WarningMsg.h"
#include "robot/NotifyMsg.h"

#include "task_plan_core.h"

static TaskPlanCore g_core;

static bool g_rcv_p_p_flag = false;
static bool g_rcv_can_data = false;

static ros::Publisher g_task_plan_pub;
static ros::Publisher g_task_status_pub;
static ros::Publisher g_v2n_heartbeat_pub;
static ros::Publisher g_v2n_command_fb_pub;
static ros::Publisher g_v2n_running_fb_pub;

// ---- core 镜像 <-> ROS 消息, 逐字段 1:1 拷贝 ----
static void TaskInfoToCore(const robot::TaskInfo &m, TaskInfoIn *out) {
    out->timestamp_msec = m.timestamp_msec;
    out->vehicle_id = m.vehicle_id;
    out->task_from = m.task_from;
    out->task_id = m.task_id;
    out->task_type = m.task_type;
    out->task_op = m.task_op;
    out->target_info.area_type = m.target_info.area_type;
    out->target_info.dest_x = m.target_info.dest_x;
    out->target_info.dest_y = m.target_info.dest_y;
    out->target_info.heading = m.target_info.heading;
}

static void TaskInfoToMsg(const TaskInfoIn &in, robot::TaskInfo *m) {
    m->timestamp_msec = in.timestamp_msec;
    m->vehicle_id = in.vehicle_id;
    m->task_from = in.task_from;
    m->task_id = in.task_id;
    m->task_type = in.task_type;
    m->task_op = in.task_op;
    m->target_info.area_type = in.target_info.area_type;
    m->target_info.dest_x = in.target_info.dest_x;
    m->target_info.dest_y = in.target_info.dest_y;
    m->target_info.heading = in.target_info.heading;
}

static void TaskPlanMsgToMsg(const TaskPlanMsgOut &in,
                             robot::task_plan_msg *m) {
    m->workMode = in.workMode;
    m->taskType = in.taskType;
    m->pathList = in.pathList;
    m->pathX = in.pathX;
    m->pathY = in.pathY;
    m->pathAngle = in.pathAngle;
    m->desireSpeed = in.desireSpeed;
    m->stopX = in.stopX;
    m->stopY = in.stopY;
    m->stopAngle = in.stopAngle;
    m->desireGear = in.desireGear;
    m->hookCmd = in.hookCmd;
    m->task_id = in.task_id;
}

static void TaskStatusToMsg(const TaskStatusOut &in, robot::TaskStatus *m) {
    m->timestamp_msec = in.timestamp_msec;
    m->vehicle_id = in.vehicle_id;
    TaskInfoToMsg(in.task_info, &m->task_info);
    m->procedure = in.procedure;
    m->fail_code = in.fail_code;
    m->fail_reason = in.fail_reason;
}

static void HeartBeatToMsg(const HeartBeatOut &in, robot::v2nHeartBeat *m) {
    m->ts = in.ts;
    m->deviceId = in.deviceId;
    m->type = in.type;
    m->values.status = in.status;
    m->values.text = in.text;
    m->values.lat = in.lat;
    m->values.lon = in.lon;
    m->values.heading = in.heading;
    m->values.turning = in.turning;
    m->values.speed = in.speed;
    m->values.power = in.power;
    m->values.remainingKeyPoints.clear();
    for (int i = 0; i < (int)in.remainingKeyPoints.size(); i++) {
        robot::v2nKeyPoint kp;
        kp.lat = in.remainingKeyPoints[i].lat;
        kp.lon = in.remainingKeyPoints[i].lon;
        kp.heading = in.remainingKeyPoints[i].heading;
        m->values.remainingKeyPoints.push_back(kp);
    }
    m->values.drivingState = in.drivingState;
    m->values.sensorsState = in.sensorsState;
    m->values.lidarState = in.lidarState;
    m->values.cameraState = in.cameraState;
    m->values.gnssState = in.gnssState;
    m->values.vehicleState = in.vehicleState;
    m->values.errorContext = "";
    m->values.hookState = in.hookState;
    m->values.soc = in.soc;
    m->values.chargingState = in.chargingState;
}

static void CommandFbToMsg(const CommandFbOut &in,
                           robot::v2nCommandFeedback *m) {
    m->ts = in.ts;
    m->deviceId = in.deviceId;
    m->type = in.type;
    m->values.ts = in.vTs;
    m->values.status = in.vStatus;
    m->values.text = in.vText;
    m->ready = in.ready;
    m->commandState = in.commandState;
    m->commandID = in.commandID;
    m->limit = in.limit;
    m->speedCommand = in.speedCommand;
}

static void RunningFbToMsg(const RunningFbOut &in,
                           robot::v2nRunningFeedback *m) {
    m->ts = in.ts;
    m->deviceId = in.deviceId;
    m->type = in.type;
    m->values.ts = in.vTs;
    m->values.status = in.vStatus;
    m->values.text = in.vText;
}

// ---- core 事件 -> ROS 调用, 严格按 core 给出的顺序逐条执行 ----
static void CoreEventToRos(const TaskPlanEvent &ev) {
    switch (ev.type) {
    case TP_PUBLISH_TASK_PLAN: {
        robot::task_plan_msg m;
        TaskPlanMsgToMsg(g_core.GetTaskPlanMsg(), &m);
        g_task_plan_pub.publish(m);
        break;
    }
    case TP_PUBLISH_TASK_STATUS: {
        robot::TaskStatus m;
        TaskStatusToMsg(g_core.GetTaskStatus(), &m);
        g_task_status_pub.publish(m);
        break;
    }
    case TP_PUBLISH_HEARTBEAT: {
        robot::v2nHeartBeat m;
        HeartBeatToMsg(g_core.GetHeartBeat(), &m);
        g_v2n_heartbeat_pub.publish(m);
        break;
    }
    case TP_PUBLISH_RUNNING_FB: {
        robot::v2nRunningFeedback m;
        RunningFbToMsg(g_core.GetRunningFb(), &m);
        g_v2n_running_fb_pub.publish(m);
        break;
    }
    case TP_PUBLISH_COMMAND_FB: {
        robot::v2nCommandFeedback m;
        CommandFbToMsg(g_core.GetCommandFb(), &m);
        g_v2n_command_fb_pub.publish(m);
        break;
    }
    case TP_PARAM_INT:
        // 心跳传感器兜底用整型字面量(旧实现即 int 重载)
        ros::param::set(ev.paramKey, ev.paramInt);
        break;
    case TP_PARAM_DOUBLE:
        ros::param::set(ev.paramKey, ev.paramDouble);
        break;
    case TP_LOG_INFO:
        ROS_INFO("%s", ev.text.c_str());
        break;
    default:
        break;
    }
}

// ---- 订阅回调: 只做消息字段搬运 ----
static void PathPlanStatusCallBack(
    const robot::path_plan_status::ConstPtr &msg) {
    g_rcv_p_p_flag = true;
    PlanStatusIn s;
    s.taskExecuStatus = msg->taskExecuStatus;
    s.curSpeed = msg->curSpeed;
    s.turning = msg->turning;
    g_core.SetPathPlanStatus(s);
}

static void CanMsgCallBack(const robot::can_msg::ConstPtr &msg) {
    g_rcv_can_data = true;
    CanStateIn c;
    c.controlPanelState = msg->controlPanelState;
    c.vehicleSpeed = msg->vehicleSpeed;
    c.hookPos = msg->hookPos;
    c.hookStatus = msg->hookStatus;
    c.batteryPower = msg->batteryPower;
    c.faultCode = msg->faultCode;
    g_core.SetCanData(c);
}

static void NavigationMsgCallBack(const robot::navigation_msg::ConstPtr &msg) {
    NavStateIn n;
    n.lat = msg->lat;
    n.lon = msg->lon;
    n.heading = msg->heading;
    n.longitudinal_accelerate = msg->longitudinal_accelerate;
    g_core.SetNavigationData(n);
}

static void TaskInfoMsgCallBack(const robot::TaskInfo::ConstPtr &msg) {
    printf("=========received task info from cloud, task_id:%d============\n", (int)msg->task_id);
    std::string task_file = "";
    ros::param::get("task_file", task_file);
    TaskInfoIn info;
    TaskInfoToCore(*msg, &info);
    g_core.SetTaskInfo(info, task_file);
}

static void RemoteSignalMsgCallBack(const robot::RemoteSignal::ConstPtr &msg) {
    RemoteSignalIn r;
    r.isfull = msg->isfull;
    g_core.SetRemoteSignal(r);
}

static void RunningMsgCallBack(const robot::RunningMsg::ConstPtr &msg) {
    RunningTargetIn t;
    t.lon = msg->values.toPoint.lon;
    t.lat = msg->values.toPoint.lat;
    t.heading = msg->values.toPoint.heading;
    std::string task_file = "";
    ros::param::get("task_file", task_file);
    g_core.OnRunningMsg(t, task_file);
}

static void CommandMsgCallBack(const robot::v2nCommandFeedback::ConstPtr &msg) {
    CommandIn c;
    c.commandState = msg->commandState;
    c.commandID = msg->commandID;
    c.limit = msg->limit;
    c.speedCommand = msg->speedCommand;
    g_core.OnCommandMsg(c, CoreEventToRos);
}

static void WarningMsgCallBack(const robot::WarningMsg::ConstPtr &msg) {
    (void)msg;
    return;
}

static void NotifyMsgCallBack(const robot::NotifyMsg::ConstPtr &msg) {
    (void)msg;
    return;
}

// ---- 10Hz 心跳定时器: 墙钟 + 参数快照 -> core ----
static void T1Callback(const ros::TimerEvent &real) {
    (void)real;
    ros::WallTime wall_time = ros::WallTime::now();

    HeartBeatInputs in;
    in.year = wall_time.toBoost().date().year();
    in.month = wall_time.toBoost().date().month();
    in.day = wall_time.toBoost().date().day();
    in.hour = wall_time.toBoost().time_of_day().hours();
    in.minute = wall_time.toBoost().time_of_day().minutes();
    in.second = wall_time.toBoost().time_of_day().seconds();
    in.sensorstate = 0;
    in.hookstate = 0;
    ros::param::get("/planning/sensorstate", in.sensorstate);
    ros::param::get("/canbus/hookstate", in.hookstate);
    g_core.RunHeartBeat(in, CoreEventToRos);
}

int main(int argc, char **argv) {
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

    g_task_plan_pub = nh.advertise<robot::task_plan_msg>(
        "/task_plan_msg", 10);
    g_task_status_pub = nh.advertise<robot::TaskStatus>(
        "/cloud/task/task_status", 10);
    g_v2n_heartbeat_pub = nh.advertise<robot::v2nHeartBeat>(
        "/v2nHeartBeat", 10);
    g_v2n_command_fb_pub = nh.advertise<robot::v2nCommandFeedback>(
        "/v2nCommandFeedback", 10);
    g_v2n_running_fb_pub = nh.advertise<robot::v2nRunningFeedback>(
        "/v2nRunningFeedback", 10);

    ros::Timer T1 = nh.createTimer(ros::Duration(0.1), T1Callback);

    ros::Rate loop_rate(20);

    std::string config_file = "";
    ros::param::get("config_file", config_file);
    g_core.InitParameter(config_file.c_str());

    ros::param::set("/cloud/suggestspeed", 100.0);

    while (ros::ok()) {
        ros::spinOnce();

        // 挂钩/托盘位置限值: canbus 启动时发布, 此处每圈热读; 读失败时
        // position_limits 保持 NSDMI 初值(=config.cfg 默认值), 不覆盖镜像
        PositionLimitsIn position_limits;
        ros::param::get("/canbus/hookposition/min",
                        position_limits.hookPosMin);
        ros::param::get("/canbus/hookposition/max",
                        position_limits.hookPosMax);
        ros::param::get("/canbus/palletposition/min",
                        position_limits.palletPosMin);
        ros::param::get("/canbus/palletposition/max",
                        position_limits.palletPosMax);
        g_core.SetPositionLimits(position_limits);

        if (g_rcv_p_p_flag && g_rcv_can_data) {
            g_core.TaskPlanProcess(CoreEventToRos);
        }

        if (g_core.recived_cloud_task ||
            (g_core.mPathPlanStatus.taskExecuStatus == 2 &&
             g_core.mExecuteTaskNum != g_core.mCurTaskNum)) {
            float max_vehicle_speed = 0;
            ros::param::get("max_vehicle_speed", max_vehicle_speed);
            g_core.PublishTaskPlanMsg(max_vehicle_speed, CoreEventToRos);
            g_core.recived_cloud_task = false;
            g_core.mExecuteTaskNum = g_core.mCurTaskNum;
        }
        loop_rate.sleep();
    }

    return 0;
}
