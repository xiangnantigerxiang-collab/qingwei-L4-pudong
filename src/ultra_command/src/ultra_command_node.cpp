// ===========================================================================
// ultra_command_node - ultra_command 的 ROS 薄壳(业务逻辑在 ultra_command_comply)。
//
// 规格:
//   task_plan 执行 pudong_air/312_316_01 或 pudong_air/312_cargo_01 时,
//     监测 /perception 障碍物是否落入矩形 pudong_air/left1 或 pudong_air/left2;
//   task_plan 执行 pudong_air/312_charge_01 时,
//     监测 /perception 障碍物是否落入矩形 pudong_air/right。
//   落入 -> rosparam::set("/ultra/status/safe", 1); 无 -> 0。
//
//   初始化监控(2026-09-11 新增): 上述三路径的 _01_01 后缀变体
//   (312_316_01_01 / 312_cargo_01_01 -> left1+left2, 312_charge_01_01 -> right):
//     起步前(车速<0.5m/s)同上监测区内障碍物 -> safe 1/0;
//     首次车速超过 0.5m/s 后, 无论区内是否有障碍物 safe 恒 0(单向, 停车不复位)。
//   车速源 = /can_msg.vehicleSpeed(m/s, canbus 20Hz, 全仓标准车速)。
//
// 接线(话题全部绝对名, 惯例对齐 pnc):
//   subscribe /task_plan_msg (队列1, 事件驱动; 取 pathList 精确匹配任务路径)
//   subscribe /perception    (队列10; 障碍物中心点, 地图系, 与矩形 csv 同系)
//   subscribe /can_msg       (队列10; 仅取 vehicleSpeed 作起步判据)
//   subscribe /cloud/msg/command_msg (队列1; 云端停止指令即退监控模式,
//              兼容未发布空任务的旧 task_plan 节点)
//   param in : path_dir(全局, robot_path_plan.launch 设置; 每周期热读, 失败重试)
//   param out: /ultra/status/safe(0/1; 每周期重发=ZOH; 非目标任务写 0;
//              感知/矩形/初始化车速不可用写 1)
//
// 主循环 10Hz; 单线程 spinOnce, 回调免锁。
// ===========================================================================
#include <csignal>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "robot/can_msg.h"
#include "robot/perception.h"
#include "robot/task_plan_msg.h"
#include "robot/v2nCommandFeedback.h"

#include "ultra_command_comply.h"

static UltraCommandComply g_comply;

static bool g_rcv_task_plan_msg = false;  // 收到过 /task_plan_msg(失明告警用)
static bool g_task_state_known = false;   // latch 空任务也表示已确认待命
static bool g_rcv_perception = false;     // 收到过 /perception
static bool g_warned_blind = false;       // 失明告警只打一次
static volatile std::sig_atomic_t g_shutdown_requested = 0;

static void ShutdownSignalHandler(int)
{
    // 信号处理函数中只置标志，ROS API 由主线程调用。
    g_shutdown_requested = 1;
}

static void TaskPlanMsgCallBack(const robot::task_plan_msg::ConstPtr &msg)
{
    g_rcv_task_plan_msg = true;
    g_task_state_known = true;
    g_comply.SetTaskPlanPaths(msg->pathList, msg->task_id);
}

static void CommandMsgCallBack(const robot::v2nCommandFeedback::ConstPtr &msg)
{
    // 云端停止指令(0-停止 1-暂停 2-运行): 新 task_plan 会同步发布空任务;
    // 此处保留直达清空以兼容旧版并缩短退出延迟。暂停不清模式
    if (msg->commandState == 0)
    {
        printf("====ultra_command==== 收到云端停止指令, 退出监控模式\n");
        g_task_state_known = true;
        g_comply.SetTaskPlanPaths(std::vector<std::string>(), 0);
    }
}

static void PerceptionCallBack(const robot::perception::ConstPtr &msg)
{
    g_rcv_perception = true;
    std::vector<UltraObstacle> objs_temp;
    objs_temp.reserve(msg->objs.size());
    for (size_t i = 0; i < msg->objs.size(); i++)
    {
        UltraObstacle obj_temp;
        obj_temp.x = msg->objs[i].x;
        obj_temp.y = msg->objs[i].y;
        objs_temp.push_back(obj_temp);
    }
    g_comply.SetObstacles(objs_temp, ros::Time::now().toSec());
}

static void CanMsgCallBack(const robot::can_msg::ConstPtr &msg)
{
    g_comply.SetVehicleSpeed(msg->vehicleSpeed);
}

static void ShutdownCallBack()
{
    // 正常退出期间先置故障态; launch respawn 后由新进程重新确认
    ros::param::set("/ultra/status/safe", 1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ultra_command_node",
              ros::init_options::NoSigintHandler);
    std::signal(SIGINT, ShutdownSignalHandler);
    std::signal(SIGTERM, ShutdownSignalHandler);
    ros::NodeHandle nh;

    ros::Subscriber task_plan_sub = nh.subscribe(
        "/task_plan_msg", 1, TaskPlanMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber perception_sub = nh.subscribe(
        "/perception", 10, PerceptionCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber can_msg_sub = nh.subscribe(
        "/can_msg", 10, CanMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber command_msg_sub = nh.subscribe(
        "/cloud/msg/command_msg", 1, CommandMsgCallBack, ros::TransportHints().tcpNoDelay());

    // 启动时尚未确认任务/感知/矩形, 先写故障态
    ros::param::set("/ultra/status/safe", 1);

    ros::Rate loop_rate(10);
    std::string path_dir = "";

    while (ros::ok() && !g_shutdown_requested)
    {
        ros::spinOnce();

        // 正常接线下 /task_plan_msg 为 latch; 若上游版本不带 latch, 打印一次告警
        if (g_rcv_perception && !g_rcv_task_plan_msg && !g_warned_blind)
        {
            g_warned_blind = true;
            printf("====ultra_command==== 警告: 已收到 /perception 但未收到过 "
                   "/task_plan_msg, 请检查 task_plan 节点及 latch 配置\n");
        }

        // path_dir 热读(全局参数, robot_path_plan.launch 下发);
        // 先于 pnc 启动时参数缺失, 加载失败每周期自动重试
        ros::param::get("path_dir", path_dir);
        if (!path_dir.empty())
            g_comply.LoadZones(path_dir);

        // 每周期覆盖参数: 任务状态未知保持故障态; 已知非目标任务清 0;
        // 目标任务由 comply 做故障安全判定
        int safe_status = 1;
        if (g_task_state_known)
            safe_status = g_comply.JudgeSafeStatus(ros::Time::now().toSec());
        ros::param::set("/ultra/status/safe", safe_status);

        loop_rate.sleep();
    }

    // roscpp 没有 rospy::on_shutdown 风格的退出回调。先发布故障态，再关闭 ROS。
    ShutdownCallBack();
    ros::shutdown();
    return 0;
}
