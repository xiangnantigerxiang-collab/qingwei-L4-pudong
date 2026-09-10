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
// 接线(话题全部绝对名, 惯例对齐 pnc):
//   subscribe /task_plan_msg (队列1, 事件驱动; 取 pathList 精确匹配任务路径)
//   subscribe /perception    (队列10; 障碍物中心点, 地图系, 与矩形 csv 同系)
//   subscribe /cloud/msg/command_msg (队列1; 云端停止指令即退监控模式,
//              因 task_plan 停车分支只清任务池、终态空 pathList 的
//              /task_plan_msg 因发布门控不会发出, 不订阅会闩死在 LEFT/RIGHT)
//   param in : path_dir(全局, robot_path_plan.launch 设置; 每周期热读, 失败重试)
//   param out: /ultra/status/safe(0/1; 监控激活期间每周期重发=ZOH,
//              非目标任务/待命/停止指令后保持最后值不动)
//
// 主循环 10Hz; 单线程 spinOnce, 回调免锁。
// ===========================================================================
#include <string>
#include <vector>

#include <ros/ros.h>

#include "robot/perception.h"
#include "robot/task_plan_msg.h"
#include "robot/v2nCommandFeedback.h"

#include "ultra_command_comply.h"

static UltraCommandComply g_comply;

static bool g_rcv_task_plan_msg = false;  // 收到过 /task_plan_msg(失明告警用)
static bool g_rcv_perception = false;     // 收到过 /perception
static bool g_warned_blind = false;       // 失明告警只打一次

static void TaskPlanMsgCallBack(const robot::task_plan_msg::ConstPtr &msg)
{
    g_rcv_task_plan_msg = true;
    g_comply.SetTaskPlanPaths(msg->pathList);
}

static void CommandMsgCallBack(const robot::v2nCommandFeedback::ConstPtr &msg)
{
    // 云端停止指令(0-停止 1-暂停 2-运行): task_plan 侧 clearTaskPool 后
    // 不发终态 /task_plan_msg(上游已知行为), 此处主动清空模式防闩死;
    // 暂停不清(任务仍在执行, 保持监控)
    if (msg->commandState == 0)
    {
        printf("====ultra_command==== 收到云端停止指令, 退出监控模式\n");
        g_comply.SetTaskPlanPaths(std::vector<std::string>());
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ultra_command_node");
    ros::NodeHandle nh;

    ros::Subscriber task_plan_sub = nh.subscribe(
        "/task_plan_msg", 1, TaskPlanMsgCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber perception_sub = nh.subscribe(
        "/perception", 10, PerceptionCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber command_msg_sub = nh.subscribe(
        "/cloud/msg/command_msg", 1, CommandMsgCallBack, ros::TransportHints().tcpNoDelay());

    // 启动即初始化一次, 保证消费者总能读到该参数
    // (对齐 task_plan 启动写 /cloud/suggestspeed 的做法)
    ros::param::set("/ultra/status/safe", 0);

    ros::Rate loop_rate(10);
    std::string path_dir = "";

    while (ros::ok())
    {
        ros::spinOnce();

        // /task_plan_msg 非 latch 事件驱动: 晚于任务下发启动时收不到当前块,
        // 监控失明至下次块推进(窗口=当前块剩余执行时长, 分钟级), 打印一次告警
        if (g_rcv_perception && !g_rcv_task_plan_msg && !g_warned_blind)
        {
            g_warned_blind = true;
            printf("====ultra_command==== 警告: 已收到 /perception 但未收到过 "
                   "/task_plan_msg(非 latch), 若任务已在执行则当前块内监控失明, "
                   "应随 pnc 栈一起启动\n");
        }

        // path_dir 热读(全局参数, robot_path_plan.launch 下发);
        // 先于 pnc 启动时参数缺失, 加载失败每周期自动重试
        ros::param::get("path_dir", path_dir);
        if (!path_dir.empty())
            g_comply.LoadZones(path_dir);

        // 仅在三个目标任务激活期间写参数(ZOH 每周期重发);
        // 其余任务/待命保持最后值不动
        if (g_comply.GetMode() != UltraCommandComply::ZONE_MODE_NONE)
            ros::param::set("/ultra/status/safe",
                            g_comply.JudgeSafeStatus(ros::Time::now().toSec()));

        loop_rate.sleep();
    }

    return 0;
}
