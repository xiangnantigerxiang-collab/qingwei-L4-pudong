#include "canbus_comply.h"
#include "robot/path_plan_status.h"

ros::Publisher can_pub;
ros::Publisher can_msg_pub;
CanbusComply canbusComply;
double hook_duration = 0.0;
double alive_time = 0.0;

double lidar_warning_time = 0.0;
double camera_warning_time = 0.0;
double gnss_warning_time = 0.0;
double hook_warning_time = 0.0;
double software_warning_time = 0.0;
double fence_warning_time = 0.0;
double net_warning_time = 0.0;
double can_receive_time = 0.0;
void CanbusCallBack(const can_msgs::Frame &msg)
{
    canbusComply.RecvCan0Data(msg);
    canbusComply.PublishVehicleStateParaMsg(can_pub);
    can_receive_time = canbusComply.sysTime.now;
}

void pathPlanStatusCallback(const robot::path_plan_status &msg)
{
    alive_time = canbusComply.sysTime.now;
}

void T1Callback(const ros::TimerEvent &real)
{
    // 传感器状态标志
    int sensorstate = 0;
    ros::param::get("/planning/sensorstate", sensorstate);
    // 周边障碍物标志
    int around_obstacle_cmd = 0;
    ros::param::get("/canbus/light", around_obstacle_cmd);
    // 网络中断标志
    int network_down = 0;
    ros::param::get("/robot/planning/netcheck", network_down);
    // 规划节点状态
    // canbus的启动时间
    double canbus_init_gap = canbusComply.sysTime.now - canbusComply.sysTime.init;
    // 检测规划节点状态
    double not_alive_time = canbusComply.sysTime.now - alive_time;
    if (not_alive_time >= 0.5 && canbus_init_gap > 10.0)
    {
        ros::param::set("/planning/alive", 0);
    }
   //can time
    double can_gap_time = canbusComply.sysTime.now - can_receive_time;
    if(can_gap_time > 0.5)
    {
        ROS_ERROR("can cant recived now\n");
        canbusComply.mCanMsg.faultCode.resize(1);
        canbusComply.mCanMsg.faultCode[0] = 1;
        can_pub.publish(canbusComply.mCanMsg);
        can_pub.publish(canbusComply.mCanMsg);
    }else
    {
        canbusComply.mCanMsg.faultCode.clear();
    }
    static int hookState = 0;
    static int hookState_last = 0;
    if (!canbusComply.mCanMsg.controlPanelState) // 人工模式下不报警
    {
        hookState = 0;
        ros::param::set("/canbus/hookstate", 0); // set hookstate to heartbeat
        return;
    }
    uint8_t can[8];
    can[0] = 0x0E;
    can[1] = 0x00;
    can[2] = 0x05;
    can[3] = 0x00;
    can[4] = 0x00;
    can[5] = 0x00;
    can[6] = 0x00;
    can[7] = 0x00;
    // 网络故障
    if (network_down == 1)
    {
        if ((canbusComply.sysTime.now - net_warning_time > 3) && canbus_init_gap > 10)
        {
            can[0] = 0x09;
            canbusComply.SendVehicleControlCmd(0x201, can);
            net_warning_time = canbusComply.sysTime.now;
        }
    }
    // 检测传感器状态
    if ((sensorstate & 0x02) == 2)
    {
        if ((canbusComply.sysTime.now - lidar_warning_time > 3) && canbus_init_gap > 10)
        {
            can[0] = 0x06; // lidar
            canbusComply.SendVehicleControlCmd(0x201, can);
            lidar_warning_time = canbusComply.sysTime.now;
        }
    }

    if ((sensorstate & 0x04) == 4)
    {
        if ((canbusComply.sysTime.now - camera_warning_time > 3) && canbus_init_gap > 10)
        {
            can[0] = 0x08; // camera
//            canbusComply.SendVehicleControlCmd(0x201, can);
            camera_warning_time = canbusComply.sysTime.now;
        }
    }

    if ((sensorstate & 0x08) == 8)
    {
        if ((canbusComply.sysTime.now - gnss_warning_time > 3) && canbus_init_gap > 10)
        {
            can[0] = 0x04; // gnss
            canbusComply.SendVehicleControlCmd(0x201, can);
            gnss_warning_time = canbusComply.sysTime.now;
        }
    }

    // 软件故障
    int planning_alive = 0;
    ros::param::get("planning/alive", planning_alive);
    printf("planning alive:%d\n", planning_alive);
    if (planning_alive == 0)
    {
        if ((canbusComply.sysTime.now - software_warning_time > 3) && canbus_init_gap > 10)
        {
            printf("软件故障\n");
            can[0] = 0x07; // software dead
//            canbusComply.SendVehicleControlCmd(0x201, can);
        }
    }
    else
    {
        printf("软件isok\n");
    }

    // 车辆启动成功
    static bool ready_flag = false;
    if (canbus_init_gap >= 10.0 && sensorstate == 0 && !ready_flag && not_alive_time < 0.5)
    {
        can[0] = 0x01; // ready
        canbusComply.SendVehicleControlCmd(0x201, can);
        ready_flag = true;
    }

    // 车辆周边障碍物
    if (around_obstacle_cmd == 7)
    {
        uint8_t can201[8] = {0x02, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
        // can201[0] = 0x0E;
        // can201[1] = 0x00;
        // can201[2] = 0x05;
        // can201[3] = 0x00;
        // can201[4] = 0x00;
        // can201[5] = 0x00;
        // can201[6] = 0x00;
        // can201[7] = 0x00;
//        canbusComply.SendVehicleControlCmd(0x201, can201);
        // if (LightCmd == 7) // 周边障碍物的报警
        // {
        //     can201[0] = 0x02; // around obstacle
        //     SendVehicleControlCmd(0x201, can201);
        // }
    }

    double spd = canbusComply.mCanMsg.vehicleSpeed;
    // 牵引座位置    //牵引栓  eps1     //已连接状态- 座在最上，栓在最下面  //已释放 - 座在最下面，栓在最上面
    int hookpos = canbusComply.mCanMsg.epsERR1; // 牵引销 esp1 //181-254 从下到上
    int linkpos = canbusComply.mCanMsg.epsERR2; // 牵引座 esp2  // 122-254 从下到上
    int hookDownPos = 240;                      // 已释放的点
    int hookedPos = 190;                        // 未到位的点,到位是180
    int linkUpPos = 240;                        // 暂时不用

    // 脱落位置
    int hookLimitPos = 180;

    printf("hook pos: %d, link pos: %d\n", hookpos, linkpos);
    static std::vector<int> link_vec(4, 0);
    link_vec.erase(link_vec.begin());
    link_vec.push_back(canbusComply.mCanMsg.linkPallet);
    int link_sum = 0;
    static int last_camera_cmd = 0; // 0-默认 1 伸出， 2 缩回
    for (int i = 0; i < link_vec.size(); i++)
    {
        link_sum += link_vec[i];
    }
    if (hookpos < 185 && linkpos > 240) // 已连接
    {
        hookState = 4;
        if (last_camera_cmd != 1)
        {
            canbusComply.forwardCamera();
            last_camera_cmd = 1;
        }
    }
    else if (hookpos > hookDownPos && (canbusComply.can_comm_cmd.hookCmd == DECOUPLING)) // 挂钩已释放
    {
        hookState = 3; // hook release
    }
    else if (hook_duration > 6.5 &&
             (canbusComply.can_comm_cmd.hookCmd == HOOKOPERATION) &&
             (hookedPos > hookedPos || !canbusComply.mCanMsg.linkPallet)) // 未到位
    {
        printf("hook duration: %f\n", hook_duration);
        hookState = 2;
    }else
    {
        hookState = 0;
    }
    if (link_sum < 4 &&
        canbusComply.mCanMsg.curGear == 4 && hookState_last == 4) // 挂钩脱落
    {
        hookState = 1; // hook warning
    }

    if (hookpos > hookDownPos)
    {
        if (last_camera_cmd != 2)
        {
            canbusComply.backwardCambera();
            last_camera_cmd = 2;
        }
    }
    printf("hookcmd:%d, hook_state:%d, last_hook_state:%d, link_sum:%d\n",
           canbusComply.can_comm_cmd.hookCmd, hookState, hookState_last, link_sum);

    if (hookState == 1) // 挂钩脱落
    {
        can[0] = 0x0A; // hook warning
//        canbusComply.SendVehicleControlCmd(0x201, can);
    }
    else
    {
        if (hookState != hookState_last)
        {
            if (hookState == 3)
                can[0] = 0x0C; // hook release
            if (hookState == 4)
                can[0] = 0x0B; // hook link
            if (hookState == 2)
                can[0] = 0x05; // hook 未到位
//            canbusComply.SendVehicleControlCmd(0x201, can);
            hookState_last = hookState;
        }
    }
    if (canbusComply.mCanMsg.controlPanelState == 1)
    {
        if(hookState == 0)
        {
            hookState = 3;
        }
        ros::param::set("/canbus/hookstate", hookState); // set hookstate to heartbeat
    }
    // 围栏报警
    int fencealarm = 0;
    ros::param::get("alarmcmd", fencealarm);
    if (fencealarm == 1)
    {
        if (canbusComply.sysTime.now - fence_warning_time > 3)
        {
            can[0] = 0x0D; // fence alarm
            canbusComply.SendVehicleControlCmd(0x201, can);
        }

        // ros::param::set("alarmcmd", 0);
    }
}

void TaskPlanMsgCallBack(const robot::task_plan_msg &msg)
{
    canbusComply.task_msg = msg;
}

void T2Callback(const ros::TimerEvent &real)
{
    canbusComply.VehicleComm();
    canbusComply.checkCamera();
}

void CanCommCallBack(const canbus::can_comm_msg &msg)
{
    canbusComply.can_comm_cmd = msg;
    static double last_time = canbusComply.sysTime.now;
    static int last_hook_cmd = msg.hookCmd;
    if (last_hook_cmd != msg.hookCmd)
    {
        last_time = canbusComply.sysTime.now;
    }
    else
    {
        hook_duration = canbusComply.sysTime.now - last_time;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_node");
    ros::NodeHandle nh;

    ros::Subscriber can_comm_sub = nh.subscribe(
        "can_comm_msg", 1, CanCommCallBack,
        ros::TransportHints().tcpNoDelay());
    ros::Subscriber can_msg_sub = nh.subscribe(
        "/can_recv", 100, CanbusCallBack,
        ros::TransportHints().tcpNoDelay());
    ros::Subscriber plan_sub = nh.subscribe("/path_plan_status", 1, pathPlanStatusCallback,
                                            ros::TransportHints().tcpNoDelay());
    ros::Subscriber task_plan_sub = nh.subscribe("task_plan_msg", 1, TaskPlanMsgCallBack);

    can_pub = nh.advertise<canbus::can_msg>("can_msg", 1);
    can_msg_pub = nh.advertise<can_msgs::Frame>("/can_send", 1);

    ros::Rate loop_rate(50);
    ros::Timer T1 = nh.createTimer(ros::Duration(0.2), T1Callback);
    ros::Timer T2 = nh.createTimer(ros::Duration(0.2), T2Callback);

    canbusComply.sysTime.init = ros::Time::now().toSec();

    while (ros::ok())
    {
        canbusComply.sysTime.now = ros::Time::now().toSec();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
