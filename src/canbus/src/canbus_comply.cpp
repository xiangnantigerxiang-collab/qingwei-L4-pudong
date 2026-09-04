#include "canbus_comply.h"

extern ros::Publisher can_send_pub;

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

    if(mCanMsg.controlPanelState == 0) {//manual state                                   
        can184[0] = 0xFF;
        can184[1] = 0x00;
        can184[2] = 0x00;
        can184[3] = 0x00;
        can184[4] = 0x00;
        can184[5] = 0x00;
        can184[6] = 0x00;
        can184[7] = 0x00;
        
        SendVehicleControlCmd(0x184, can184);

        can284[0] = 0xFF;
        can284[1] = 0x00;
        can284[2] = 0x00;
        can284[3] = 0x00;
        can284[4] = 0x00;
        can284[5] = 0x00;
        can284[6] = 0x00;
        can284[7] = 0x00;
        
        SendVehicleControlCmd(0x284, can284);

        return;
    }

    //autonomous mode
    Gear = can_comm_cmd.desireGear;
    Throttle = can_comm_cmd.throttlePercent;
    Brake = can_comm_cmd.brakePercent;
    WheelAngle = (int)(can_comm_cmd.wheelAngle);

    if(EmergencyStop == 1 || mCanMsg.emergencyStop == 1) Brake = 80;

    //restrain & protection
    if(Brake > 0) Throttle = 0;
    if(Brake > 0) WheelAngle = 0;
    if(mCanMsg.curGear == GEAR_N) WheelAngle = 0;
    if(WheelAngle > 700) WheelAngle = 700;
    if(WheelAngle < -700) WheelAngle = -700;
    if(Gear == GEAR_N) Throttle = 0;
    if(Gear == GEAR_N && Brake > 20) Brake = 20;

    // send
    can184[0] = 0xFF;
    can184[1] = Throttle;
    can184[2] = Brake;
    can184[3] = (unsigned char)((WheelAngle * 10 + 16384) % 256);
    can184[4] = (unsigned char)((WheelAngle * 10 + 16384) / 256);
    can184[5] = 10;
    can184[7] = 0;

    if(Gear == GEAR_D) can184[6] = 0x01 << 4 | Gear; //EPB OFF
    if(Gear == GEAR_R) can184[6] = 0x01 << 4 | Gear; //EPB OFF
    if(Gear == GEAR_N) can184[6] = 0x02 << 4 | Gear; //EPB ON

    SendVehicleControlCmd(0x184, can184);

    can284[0] = 0xFF;
    can284[1] = 0x00;
    can284[2] = 0x00;
    can284[3] = 0x00;
    can284[4] = 0x00;
    can284[5] = 0x00;
    can284[6] = 0x00;
    can284[7] = 0x00;

    if(can_comm_cmd.hookCmd == HOOKOPERATION) {
        int status = mCanMsg.hookStatus;
        if(status != 1 && status != 4) can284[2] = 0x05; //1 = block
    }

    if(can_comm_cmd.hookCmd == DECOUPLING) {
        int status = mCanMsg.hookStatus;
        if(status != 3) can284[2] = 0x0A;
    }

    int LightCmd = 0;
    ros::param::get("/canbus/light", LightCmd);

    if(LightCmd == 3) can284[1] = 0x01; //left
    if(LightCmd == 4) can284[1] = 0x02; //right

    ros::Time timenow = ros::Time::now();
    std::time_t now_time = timenow.sec;
    std::tm *local_time = std::localtime(&now_time);

    if(local_time->tm_hour >= 18 || local_time->tm_hour <= 6) can284[1] += 4;
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

void CanbusComply::SendVehicleControlCmd(uint32_t id, unsigned char *data)
{
    can_msgs::Frame msg;
    msg.id = id;
    msg.is_rtr = 0;
    msg.is_extended = 0;
    msg.is_error = 0;
    msg.dlc = 8;
    for (int i = 0; i < 8; i++) msg.data[i] = data[i];

    can_send_pub.publish(msg);
}

void CanbusComply::RecvCanData(const can_msgs::Frame &msg)
{
    sysTime.canbus = sysTime.now;

    switch (msg.id) {
    case 0x185: {
        mCanMsg.brakePercent = (uint8_t)(msg.data[2] & 0xFF);
        mCanMsg.curGear = msg.data[3] & 0x0F;
        mCanMsg.controlPanelState = (msg.data[3] & 0x40) >> 6;
        mCanMsg.emergencyStop = (msg.data[3] & 0x80) >> 7;

        double spd = (double)(msg.data[7] * 256 + msg.data[6]);
        mCanMsg.vehicleSpeed = spd * 3/25.8/60 * M_PI * 0.66/3.6;
	
        mCanMsg.rawcommand.clear();

        for (auto i : msg.data) mCanMsg.rawcommand.push_back(i);
    } break;

    case 0x285: {
        mCanMsg.hookState = msg.data[0] & 0x01;
        mCanMsg.linkPallet = (msg.data[0] & 0x04) >> 2;
        mCanMsg.eabPanelState = (msg.data[0] & 0x08) >> 3;
        mCanMsg.hookButton = (msg.data[0] & 0x30) >> 4;// 0-pause 1-up 2-down
        mCanMsg.linkButton = (msg.data[0] & 0xC0) >> 6;// 0-pause 1-up 2-down
        mCanMsg.batteryPower = msg.data[1];
        mCanMsg.hookPos = msg.data[4];	
        mCanMsg.palletPos = msg.data[6];

        uint8_t FaultCode[2] = {0, 0};
        mCanMsg.faultCode.clear();
        mCanMsg.faultCode.push_back(FaultCode[0]);
    } break;

    case 0x0c02a0a2: {
        double angle = (double)(msg.data[1] * 256 + msg.data[0] - 15750);
        mCanMsg.wheelAngle = angle / 10.0;

        double current = (double)(msg.data[3] * 256 + msg.data[2] - 15750);
        mCanMsg.epsCurrent = current * 0.000152 - 5.0;

        mCanMsg.epsMode = msg.data[6];

        mCanMsg.rawfeedback.clear();
        for (auto i : msg.data) mCanMsg.rawfeedback.push_back(i);
    } break;

    case 0x588: {
        double value = msg.data[4] * 0.01;
    } break;

    case 0x586: {
        double value = msg.data[4];
    } break;

    default: break;
    }
}
