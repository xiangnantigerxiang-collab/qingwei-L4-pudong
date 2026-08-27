#include "canbus_comply.h"

extern ros::Publisher can_msg_pub;

void CanbusComply::VehicleComm()
{
    // can284 byte1 : bit0 left light
    // can284 byte1 : bit1 right light
    // can284 byte1 : bit3 whistle
    // can284 byte2 : bit01 pallet control 1down 2up
    // can284 byte2 : bit23 hook control 1up 2down

    // state init
    int WheelAngle = 0;

    uint8_t Throttle = 0;
    uint8_t Brake = 0;
    uint8_t Gear = 0;

    uint8_t can184[8];
    uint8_t can284[8];

    if (mCanMsg.controlPanelState == 0) // 非自动模式
    {                                   // manual state
        can184[0] = 0xFF;
        can184[1] = 0x00;
        can184[2] = 0x00;
        can184[3] = 0x00;
        can184[4] = 0x00;
        can184[5] = 0x00;
        can184[6] = 0x00;
        can184[7] = 0x00;
        // 点火/挡位/EPB刹车使能
        SendVehicleControlCmd(0x184, can184);

        can284[0] = 0xFF;
        can284[1] = 0x00;
        can284[2] = 0x00;
        can284[3] = 0x00;
        can284[4] = 0x00;
        can284[5] = 0x00;
        can284[6] = 0x00;
        can284[7] = 0x00;
        // 声光使能
        SendVehicleControlCmd(0x284, can284);
        return;
    }
    // 自动模式
    //  command
    Gear = can_comm_cmd.desireGear;
    Throttle = can_comm_cmd.throttlePercent;
    Brake = can_comm_cmd.brakePercent;
    WheelAngle = (int)(can_comm_cmd.wheelAngle);

    // restrain
    if (mCanMsg.emergencyStop == 1)
        Throttle = 0;
    if (mCanMsg.emergencyStop == 1)
        WheelAngle = 0;
    if (mCanMsg.curGear == GEAR_N)
        WheelAngle = 0;
    if (WheelAngle > 700)
        WheelAngle = 700;
    if (WheelAngle < -700)
        WheelAngle = -700;
    // 软件故障
    int planning_alive = 0;
    ros::param::get("planning/alive", planning_alive);
    if (planning_alive == 0)
    {
        Throttle = 0;
        Brake = 70;
    }

    // send
    can184[0] = 0xFF;
    can184[1] = Throttle;
    can184[2] = Brake;
    can184[3] = (unsigned char)((WheelAngle * 10 + 16384) % 256);
    can184[4] = (unsigned char)((WheelAngle * 10 + 16384) / 256);
    can184[5] = 10;
    int hook_state = 0;
    int sensor_state = 0;
    int fence_alarmed = 0;

    ros::param::get("/canbus/hookstate", hook_state);
    ros::param::get("/planning/sensorstate", sensor_state);
    ros::param::get("alarmcmd", fence_alarmed);
    if (mCanMsg.vehicleSpeed < 0.1 && Throttle == 0)
    {
        static std::vector<int> alarm_vec(3, 0);
        alarm_vec.erase(alarm_vec.begin());
        alarm_vec.push_back(fence_alarmed);
        int sum = alarm_vec[0] + alarm_vec[1] + alarm_vec[2];
        printf("hookstate:%d, sensorstate:%d, alarm_sum:%d\n", hook_state, sensor_state, sum);
        if ((sensor_state & 0x02) == 2 ||
            (sensor_state & 0x04) == 4 ||
            // hook_state == 1 ||
            sum > 0 ||
            (sensor_state & 0x08) == 8 ||
            planning_alive == 0)
        {
            printf("异常，hookstate:%d, sensorstate:%d, alarm_sum:%d 挂N档\n", hook_state, sensor_state, sum);
            can184[1] = 0;
            Gear = GEAR_N;
            // 拉起电子手刹
            can184[6] = 0x02 << 4 | Gear;
        }
    }
    if (hook_state == 1)
    {
        can184[1] = 0;
        printf("异常，hookstate:%d, sensorstate:%d挂N档\n", hook_state, sensor_state);
    }
    // else
    // {
    //     // 松开电子手刹
    //     can184[6] = 0x01 << 4 | Gear;
    // }
    if (Gear == GEAR_D || Gear == GEAR_R)
    {
        // 松开电子手刹
        can184[6] = 0x01 << 4 | Gear;
    }
    else if (Gear == GEAR_N)
    {
        // 拉起电子手刹
        can184[6] = 0x02 << 4 | Gear;
    }
    can184[7] = 0;

    SendVehicleControlCmd(0x184, can184);

    can284[0] = 0xFF;
    can284[1] = 0x00;
    can284[2] = 0x00;
    can284[3] = 0x00;
    can284[4] = 0x00;
    can284[5] = 0x00;
    can284[6] = 0x00;
    can284[7] = 0x00;

    if (can_comm_cmd.hookCmd == HOOKOPERATION) // 挂钩
    {
        if (hook_state != 4)
        {
            // static double mHookTime = 0;
            if (mCanMsg.epsERR1 > 184)
            {
                // printf("销子下移\n");
                can284[2] = 0x04; // 销子下移
                                  // mHookTime = sysTime.now;
            }
            else // 销子已连接，继续放牵引座
            {
                // double linkTime = sysTime.now;
                // if (linkTime - mHookTime > 1.0)
                // {
                // printf("牵引座上移\n");
                can284[2] = 0x01;
                // }
            }
        }
    }
    else if (can_comm_cmd.hookCmd == DECOUPLING) // 座下移，钩上移，脱掉底盘
    {
        can284[2] = 0x0A;
    }
    else if (task_msg.taskType == ADAPTIVEHOOK)
    {
        if (mCanMsg.epsERR1 < 245 || mCanMsg.epsERR2 > 130)
        {
            can284[2] = 0x0A;
        }
    }

    int LightCmd = 0;
    ros::param::get("/canbus/light", LightCmd);

    if (LightCmd == 3)
        can284[1] = 0x01; // left
    if (LightCmd == 4)
        can284[1] = 0x02; // right

    ros::Time timenow = ros::Time::now();
    std::time_t now_time = timenow.sec;
    std::tm *local_time = std::localtime(&now_time);

    if (local_time->tm_hour >= 18 || local_time->tm_hour <= 6)
        can284[1] += 4;
    ros::param::set("/canbus/time", local_time->tm_hour);

    SendVehicleControlCmd(0x284, can284);
}

void CanbusComply::forwardCamera()
{
    uint8_t can608[8] = {0x2b, 0x40, 0x40, 0x00, 0xe7, 0x03, 0x00, 0x00};
    uint8_t can606[8] = {0x2b, 0x40, 0x40, 0x00, 0xe7, 0x03, 0x00, 0x00};
    SendVehicleControlCmd(0x608, can608);
    usleep(10000);
    SendVehicleControlCmd(0x606, can606);
}

void CanbusComply::backwardCambera()
{
    uint8_t can608[8] = {0x2b, 0x40, 0x40, 0x00, 0x19, 0xfc, 0x00, 0x00};
    uint8_t can606[8] = {0x2b, 0x40, 0x40, 0x00, 0x19, 0xfc, 0x00, 0x00};
    SendVehicleControlCmd(0x608, can608);
    usleep(10000);
    SendVehicleControlCmd(0x606, can606);
}

void CanbusComply::checkCamera()
{
    uint8_t can608[8] = {0x40, 0x00, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t can606[8] = {0x40, 0x00, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00};
    SendVehicleControlCmd(0x608, can608);
    usleep(50000);
    SendVehicleControlCmd(0x606, can606);
}

void CanbusComply::PublishVehicleStateParaMsg(ros::Publisher tPub)
{
    tPub.publish(mCanMsg);
}

void CanbusComply::SendVehicleControlCmd(uint32_t id, unsigned char *data)
{
    can_msgs::Frame msg;
    msg.id = id;
    msg.is_rtr = 0;
    msg.is_extended = 0;
    msg.is_error = 0;
    msg.dlc = 8;
    for (int i = 0; i < 8; i++)
        msg.data[i] = data[i];

    can_msg_pub.publish(msg);
}

void CanbusComply::RecvCan0Data(const can_msgs::Frame &msg)
{
    switch (msg.id)
    {
    case 0x185:
    {
        float temp_f = 0;
        uint8_t temp_u8 = 0;
        int16_t temp_int16 = 0; // add
        // 方向盘转角与协议计算不一致，且temp_f没有用到????????
        temp_f = ((double)((msg.data[1] << 8) | msg.data[0])) - 16384;

        mCanMsg.brakePercent = (uint8_t)(msg.data[2] & 0xff);
        // 解析速度，高低压与协议不一致????????????
        temp_int16 = (int16_t)((msg.data[7] << 8) | msg.data[6]);
        temp_f = temp_int16 * 1.0;
        // 低压版本
        mCanMsg.vehicleSpeed = temp_f * 3 / 24.2 / 60 * M_PI * 0.7 / 3.6; // m/s add	->计算为公里/小时
        // 高压版本?
        // mCanMsg.vehicleSpeed = temp_f*3/24.2/60*M_PI*0.66/3.6; // m/s add
        // 解析挡位
        temp_u8 = msg.data[3] & 0x0f;
        if (temp_u8 == GEAR_R)
            mCanMsg.curGear = GEAR_R;
        if (temp_u8 == GEAR_N)
            mCanMsg.curGear = GEAR_N;
        if (temp_u8 == GEAR_D)
            mCanMsg.curGear = GEAR_D;

        if (temp_u8 == 0x01)
            ROS_INFO("Gear P");
        // 解析自动/手动模式切换 1 自动 0 手动
        if (msg.data[3] & 0x40)
            mCanMsg.controlPanelState = 1;
        else
            mCanMsg.controlPanelState = 0;
        // 解析急停，但是协议里没有????????????
        if (msg.data[3] & 0x80)
            mCanMsg.emergencyStop = 1;
        else
            mCanMsg.emergencyStop = 0;
        mCanMsg.rawcommand.clear();
        for (auto i : msg.data)
            mCanMsg.rawcommand.push_back(i);
    }
    break;

    case 0x285:
    {
        // 解析挂钩状态 1 挂载托盘，2 未挂载托盘
        if (msg.data[0] & 0x01)
            mCanMsg.hookState = 1;
        else
            mCanMsg.hookState = 0;
        // 解析挂钩故障状态缺失??????????????
        // todo
        // 解析挂钩限位开关
        if (msg.data[0] & 0x04)
            mCanMsg.linkPallet = 1;
        else
            mCanMsg.linkPallet = 0;
        // 解析EAB面板状态-但是协议里并没有 ???????
        if (msg.data[0] & 0x08)
            mCanMsg.eabPanelState = 1;
        else
            mCanMsg.eabPanelState = 0;
        // 协议里没有
        LinkPos = msg.data[6];
        HookPos = msg.data[4];
        mCanMsg.epsERR1 = HookPos;
        mCanMsg.epsERR2 = LinkPos;
        // 解析电池电量
        mCanMsg.batteryPower = msg.data[1];

        uint8_t FaultCode[2] = {0, 0};
        mCanMsg.faultCode.clear();
        mCanMsg.faultCode.push_back(FaultCode[0]);
        //???????????
        mCanMsg.hookButton = (msg.data[0] & 0x30) >> 4; // 0pause 1up 2down
        mCanMsg.linkButton = (msg.data[0] & 0xC0) >> 6; // 0pause 1up 2down
    }
    break;

    case 0x401:
    {
        float temp_eps = 0;
        mCanMsg.epsMode = msg.data[0];
        mCanMsg.epsCurrent = (double)(msg.data[1] - 126) / 4.2;
        temp_eps = ((double)((msg.data[3] << 8) | msg.data[2])) - 16384;
        mCanMsg.epsCentring = msg.data[4];
        mCanMsg.epsERR1 = msg.data[5];
        mCanMsg.epsERR2 = msg.data[6];

        mCanMsg.wheelAngle = temp_eps / 10.0;

        mCanMsg.rawfeedback.clear();
        for (auto i : msg.data)
            mCanMsg.rawfeedback.push_back(i);
    }
    break;

    case 0x0c02a0a2:
    {
        int data = (msg.data[1] << 8) + msg.data[0] - 15750;
        mCanMsg.wheelAngle = (double)data / 10.0;

        data = (msg.data[3] << 8) + msg.data[2] - 15750;
        mCanMsg.epsCurrent = (double)data * 0.000152 - 5.0;

        mCanMsg.epsMode = msg.data[6];

        mCanMsg.rawfeedback.clear();
        for (auto i : msg.data)
            mCanMsg.rawfeedback.push_back(i);
    }
    break;
    case 588:
    {
        // left
        double value = msg.data[4] * 0.01;
        if (value > 1.0)
        {
            printf("left camera error\n");
        }
        printf("left camera current is: %f\n", value);
        break;
    }
    case 586:
    {
        // right
        double value = msg.data[4];
        if (value > 1.0)
        {
            printf("right camera error\n");
        }
        printf("right camera current is: %f\n", value);
        break;
    }
    default:
        break;
    }
}
