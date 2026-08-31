// ===========================================================================
// canbus_node - ROS adapter for CanbusCore.
//
// Only ROS specific glue lives in this file: topic subscribe/publish,
// parameter server access, timers and clocks. Every piece of protocol or
// safety logic is in canbus_core.cpp, which knows nothing about ROS.
//
// Wiring (unchanged against the previous implementation; all names are
// absolute so a namespace in the launch file can never silently remap
// them - relative names like "can_comm_msg" used to resolve to the same
// globals only because the node ran without a namespace):
//   subscribe /can_recv        (socketcan_bridge -> vehicle feedback frames)
//   subscribe /can_comm_msg    (pnc control command, 20 Hz)
//   subscribe /path_plan_status(planning heartbeat)
//   subscribe /task_plan_msg   (current task type)
//   publish   /can_msg         (parsed vehicle state, after each frame)
//   publish   /can_send        (outgoing frames -> socketcan_bridge)
//   param     in:  /planning/alive, /planning/sensorstate,
//                 /robot/planning/netcheck, /alarmcmd, /canbus/light,
//                 /canbus/hookstate, ~config_file (private)
//   param     out: /planning/alive, /canbus/hookstate, /canbus/time
//
// Timers: 5 Hz safety check + 5 Hz control sender, main loop 50 Hz (clock).
// All callbacks run in the single threaded spinner, no locks needed.
// ===========================================================================
#include <ctime>

#include <ros/ros.h>
#include <can_msgs/Frame.h>

#include "canbus/can_comm_msg.h"
#include "canbus/can_msg.h"
#include "robot/path_plan_status.h"
#include "robot/task_plan_msg.h"

#include "canbus_core.h"

static CanbusCore g_core;
static ros::Publisher g_state_pub;   // "can_msg": parsed vehicle state
static ros::Publisher g_frame_pub;   // "/can_send": frames to the CAN bus

// ---- core state -> ROS message, plain 1:1 field copy ----
static void StateToMsg(const VehicleState &s, canbus::can_msg *m) {
    m->throttlePercent = s.throttlePercent;
    m->brakePercent = s.brakePercent;
    m->wheelAngle = s.wheelAngle;
    m->vehicleSpeed = s.vehicleSpeed;
    m->curGear = s.curGear;
    m->hookState = s.hookState;
    m->linkPallet = s.linkPallet;
    m->controlPanelState = s.controlPanelState;
    m->eabPanelState = s.eabPanelState;
    m->emergencyStop = s.emergencyStop;
    m->batteryPower = s.batteryPower;
    m->hookButton = s.hookButton;
    m->linkButton = s.linkButton;
    m->faultCode = s.faultCode;
    m->epsCMD = s.epsCMD;
    m->wheelAngleCMD = s.wheelAngleCMD;
    m->epsMode = s.epsMode;
    m->epsCurrent = s.epsCurrent;
    m->epsCentring = s.epsCentring;
    m->epsERR1 = s.epsErr1;
    m->epsERR2 = s.epsErr2;
    m->rawcommand = s.rawcommand;
    m->rawfeedback = s.rawfeedback;
    /* can_msg.msg 无 desireSpeed/desireAcc 字段(它们属于 can_comm_msg
     * 控制指令消息)——曾误加两行赋值致车端编译失败,08-28 车端实测发现 */
}

static void PublishState() {
    canbus::can_msg m;
    StateToMsg(g_core.mState, &m);
    g_state_pub.publish(m);
}

// ---- core event -> ROS call, in exactly the order the core emits them ----
static void CoreEventToRos(const CoreEvent &ev) {
    switch (ev.type) {
    case CORE_FRAME: {
        can_msgs::Frame f;
        f.id = ev.frame.id;
        f.is_rtr = 0;
        f.is_extended = 0;
        f.is_error = 0;
        f.dlc = 8;
        for (int i = 0; i < 8; i++)
            f.data[i] = ev.frame.data[i];
        g_frame_pub.publish(f);
        break;
    }
    case CORE_PARAM:
        ros::param::set(ev.paramKey, ev.paramValue);
        break;
    case CORE_PUBLISH_STATE:
        PublishState();
        break;
    case CORE_LOG_ERROR:
        ROS_ERROR("%s", ev.text.c_str());
        break;
    case CORE_LOG_INFO:
        ROS_INFO("%s", ev.text.c_str());
        break;
    default:
        break;
    }
}

// ---- subscriptions --------------------------------------------------------
static void CanFrameCallback(const can_msgs::Frame &msg) {
    uint8_t d[8];
    for (int i = 0; i < 8; i++)
        d[i] = msg.data[i];

    g_core.OnCanFrame(msg.id, d, CoreEventToRos);
    // publish the parsed state right after each feedback frame
    PublishState();
    g_core.MarkCanRx();
}

static void PlanStatusCallback(const robot::path_plan_status &msg) {
    (void)msg;
    g_core.OnPlanningHeartbeat();
}

static void ControlCmdCallback(const canbus::can_comm_msg &msg) {
    ControlCommand c;
    c.gear = msg.desireGear;
    c.throttle = msg.throttlePercent;
    c.brake = msg.brakePercent;
    c.wheelAngle = msg.wheelAngle;
    c.hookCmd = msg.hookCmd;
    g_core.OnControlCommand(c);
}

static void TaskMsgCallback(const robot::task_plan_msg &msg) {
    g_core.OnTaskType(msg.taskType);
}

// ---- timers, 5 Hz each -----------------------------------------------------
static void SafetyCheckTimerCallback(const ros::TimerEvent &ev) {
    (void)ev;
    SafetyInputs in;
    ros::param::get("/planning/sensorstate", in.sensorState);
    ros::param::get("/robot/planning/netcheck", in.networkDown);
    ros::param::get("/planning/alive", in.planningAlive);
    ros::param::get("/alarmcmd", in.fenceAlarm);
    ros::param::get("/canbus/light", in.lightCmd);
    g_core.RunSafetyCheck(in, CoreEventToRos);
}

// Local clock hour, needed for the automatic head lamp.
static int LocalHourNow() {
    ros::Time t = ros::Time::now();
    std::time_t sec = t.sec;
    std::tm *local = std::localtime(&sec);
    return local->tm_hour;
}

static void ControlSendTimerCallback(const ros::TimerEvent &ev) {
    (void)ev;
    ControlInputs in;
    in.hourOfDay = LocalHourNow();
    // "/canbus/hookstate" and "planning/alive" may have been written by the
    // safety check earlier in this same tick - reads happen after it ran,
    // matching the previous single-process behaviour
    ros::param::get("/planning/alive", in.planningAlive);
    ros::param::get("/canbus/hookstate", in.hookState);
    ros::param::get("/planning/sensorstate", in.sensorState);
    ros::param::get("/alarmcmd", in.fenceAlarm);
    ros::param::get("/canbus/light", in.lightCmd);
    g_core.RunControlCycle(in, CoreEventToRos);
    g_core.RunCameraPoll(CoreEventToRos);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "can_node");
    ros::NodeHandle nh;

    // Load the hook/pallet position limits once at startup. Path can be
    // overridden with the private param ~config_file (launch: <param
    // name="config_file" .../> inside the <node> block). Any problem only
    // warns: the core then keeps the built-in defaults (equal to the
    // previously hard coded thresholds), the node must always come up.
    {
        std::string cfg_path =
            "/home/nvidia/qingwei-L4-No2/src/canbus/config.cfg";
        ros::NodeHandle pnh("~");
        pnh.param<std::string>("config_file", cfg_path, cfg_path);
        int warns = g_core.LoadPositionConfig(cfg_path.c_str());
        if (warns < 0)
            ROS_WARN("config.cfg not loaded (%s), using built-in defaults",
                     cfg_path.c_str());
        else if (warns > 0)
            ROS_WARN("config.cfg loaded with %d warning line(s), "
                     "see console output", warns);
        else
            ROS_INFO("config.cfg loaded: hook [%d..%d], pallet [%d..%d]",
                     g_core.mHookPosMin, g_core.mHookPosMax,
                     g_core.mPalletPosMin, g_core.mPalletPosMax);
    }

    ros::Subscriber comm_sub = nh.subscribe(
        "/can_comm_msg", 1, ControlCmdCallback,
        ros::TransportHints().tcpNoDelay());
    ros::Subscriber can_sub = nh.subscribe(
        "/can_recv", 100, CanFrameCallback,
        ros::TransportHints().tcpNoDelay());
    ros::Subscriber plan_sub = nh.subscribe(
        "/path_plan_status", 1, PlanStatusCallback,
        ros::TransportHints().tcpNoDelay());
    ros::Subscriber task_sub = nh.subscribe(
        "/task_plan_msg", 1, TaskMsgCallback);

    g_state_pub = nh.advertise<canbus::can_msg>("/can_msg", 1);
    g_frame_pub = nh.advertise<can_msgs::Frame>("/can_send", 1);

    ros::Rate loop_rate(50);
    ros::Timer safety_timer =
        nh.createTimer(ros::Duration(0.2), SafetyCheckTimerCallback);
    ros::Timer control_timer =
        nh.createTimer(ros::Duration(0.2), ControlSendTimerCallback);

    g_core.mInitSec = ros::Time::now().toSec();
    g_core.mNowSec = g_core.mInitSec;

    while (ros::ok()) {
        g_core.mNowSec = ros::Time::now().toSec();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
