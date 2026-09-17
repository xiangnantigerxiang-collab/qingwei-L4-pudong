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

    if(mCanMsg.hookStatus == 4 && mCanMsg.palletStatus != 3) can284[2] = 0x01;
    if(mCanMsg.hookStatus == 3 && mCanMsg.palletStatus != 4) can284[2] = 0x02;

    int LightCmd = 0;
    ros::param::get("/canbus/light", LightCmd);

    if(LightCmd == 3) can284[1] = 0x01; //left
    if(LightCmd == 4) can284[1] = 0x02; //right

    ros::Time timenow = ros::Time::now();
    std::time_t now_time = timenow.sec;
    std::tm *local_time = std::localtime(&now_time);

    if(local_time->tm_hour >= 18 || local_time->tm_hour <= 6) can284[1] += 4;

    SendVehicleControlCmd(0x284, can284);

    //set camera handle
    if(mCanMsg.hookStatus == 4) forwardCamera();
    else backwardCamera();
}

void CanbusComply::forwardCamera()
{
    uint8_t can608[8] = {0x2b, 0x40, 0x40, 0x00, 0xe7, 0x03, 0x00, 0x00};
    uint8_t can606[8] = {0x2b, 0x40, 0x40, 0x00, 0xe7, 0x03, 0x00, 0x00};
    SendVehicleControlCmd(0x608, can608);
    usleep(10000);
    SendVehicleControlCmd(0x606, can606);
}

void CanbusComply::backwardCamera()
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

    case 0x08FB670E: {  // EHB 帧1(PGN 0xFB67,EHB_STORAGE 蓄能系统),J1939 Intel 位序(位1=最低位)
        mEHBMsg.EHB_STO_StorageSystemStatus             = msg.data[0] & 0x03;          //1.1-1.2 蓄能系统工作状态
        mEHBMsg.EHB_STO_StorageSystemFault              = (msg.data[0] >> 2) & 0x03;   //1.3-1.4 蓄能系统故障状态
        mEHBMsg.EHB_STO_StorageSystemLowPressureWarn    = (msg.data[0] >> 4) & 0x03;   //1.5-1.6 蓄能系统低压警示
        mEHBMsg.EHB_STO_BrakeFluidPosition              = (msg.data[0] >> 6) & 0x03;   //1.7-1.8 制动液位信号状态
        mEHBMsg.EHB_STO_PressureSensor1RawValue         = msg.data[1];                 //2.1-2.8 液压传感器1原始压力值
        mEHBMsg.EHB_STO_PressureSensor2RawValue         = msg.data[2];                 //3.1-3.8 液压传感器2原始压力值
        mEHBMsg.EHB_STO_PressureSensor3RawValue         = msg.data[3];                 //4.1-4.8 液压传感器3原始压力值
        mEHBMsg.EHB_STO_HighPressureAccumulatorA        = msg.data[4];                 //5.1-5.8 高压蓄能器压力A
        mEHBMsg.EHB_STO_HighPressureAccumulatorB        = msg.data[5];                 //6.1-6.8 高压蓄能器压力B
        mEHBMsg.EHB_STO_FailureNum                      = msg.data[6] & 0x0F;          //7.1-7.4 蓄能系统当前故障数量
        mEHBMsg.EHB_STO_FAU_PowerSupplyVoltageHigh      = (msg.data[6] >> 4) & 0x01;   //7.5 电源电压过高
        mEHBMsg.EHB_STO_FAU_PowerSupplyVoltageLow       = (msg.data[6] >> 5) & 0x01;   //7.6 电源电压过低
        mEHBMsg.EHB_STO_FAU_SensorSupplyError           = (msg.data[6] >> 6) & 0x01;   //7.7 液压传感器电源异常
        mEHBMsg.EHB_STO_FAU_Sensor1Error                = (msg.data[6] >> 7) & 0x01;   //7.8 1号液压传感器信号异常
        mEHBMsg.EHB_STO_FAU_Sensor2Error                = msg.data[7] & 0x01;          //8.1 2号液压传感器信号异常
        mEHBMsg.EHB_STO_FAU_Sensor3Error                = (msg.data[7] >> 1) & 0x01;   //8.2 3号液压传感器信号异常
        mEHBMsg.EHB_STO_FAU_SensorJudgmentFailure       = (msg.data[7] >> 2) & 0x01;   //8.3 液压传感器信号无法解析
        mEHBMsg.EHB_STO_FAU_MotorOpenLoad               = (msg.data[7] >> 3) & 0x01;   //8.4 蓄能电机断路
        mEHBMsg.EHB_STO_FAU_MotorCannotStop             = (msg.data[7] >> 4) & 0x01;   //8.5 蓄能电机无法关闭
        mEHBMsg.EHB_STO_FAU_MotorWorkTimeout            = (msg.data[7] >> 5) & 0x01;   //8.6 蓄能电机加压超时
    } break;

    case 0x08FB680E: {  // EHB 帧2(PGN 0xFB68,EHB_AUTOBRAKE 主动制动),2.3-2.4 未分配
        mEHBMsg.EHB_ATB_ActualBrakePressure        = msg.data[0];                //1.1-1.8 主动制动系统实际压力
        mEHBMsg.EHB_ATB_AutoBrakeSystemStatus      = msg.data[1] & 0x03;         //2.1-2.2 主动制动系统当前状态
        mEHBMsg.EHB_ATB_FailureNum                 = (msg.data[1] >> 4) & 0x0F;  //2.5-2.8 主动制动系统当前故障数量
        mEHBMsg.EHB_ATB_FAU_PowerSupplyVoltageHigh = msg.data[2] & 0x01;         //3.1 电源电压过高
        mEHBMsg.EHB_ATB_FAU_PowerSupplyVoltageLow  = (msg.data[2] >> 1) & 0x01;  //3.2 电源电压过低
        mEHBMsg.EHB_ATB_FAU_CanBusOff              = (msg.data[2] >> 2) & 0x01;  //3.3 CAN BUS OFF
        mEHBMsg.EHB_ATB_FAU_SensorSupplyError      = (msg.data[2] >> 3) & 0x01;  //3.4 传感器电源故障
        mEHBMsg.EHB_ATB_FAU_MotorDriverError       = (msg.data[2] >> 4) & 0x01;  //3.5 电机驱动故障
        mEHBMsg.EHB_ATB_FAU_BrakePressSensorError  = (msg.data[2] >> 5) & 0x01;  //3.6 制动液压信号异常
        mEHBMsg.EHB_ATB_FAU_LosOfEHBSTOCom         = (msg.data[2] >> 6) & 0x01;  //3.7 EHB蓄能器通讯丢失
        mEHBMsg.EHB_ATB_FAU_MotorCommuteFail       = (msg.data[2] >> 7) & 0x01;  //3.8 电机校准失败
        mEHBMsg.EHB_ATB_FAU_MotorOpenLoad          = msg.data[3] & 0x01;         //4.1 电机开路
        mEHBMsg.EHB_ATB_FAU_MotorShort             = (msg.data[3] >> 1) & 0x01;  //4.2 电机短路
        mEHBMsg.EHB_ATB_FAU_MotorStall             = (msg.data[3] >> 2) & 0x01;  //4.3 电机堵转
        mEHBMsg.EHB_ATB_FAU_ValveBodyError         = (msg.data[3] >> 3) & 0x01;  //4.4 阀体故障
        mEHBMsg.EHB_ATB_FAU_LosOfBrakeReqSignal    = (msg.data[3] >> 4) & 0x01;  //4.5 制动请求信号丢失
        mEHBMsg.EHB_ATB_FAU_BrakeReqRcError        = (msg.data[3] >> 5) & 0x01;  //4.6 制动请求rollcounter错误
        mEHBMsg.EHB_ATB_FAU_BrakeReqCsError        = (msg.data[3] >> 6) & 0x01;  //4.7 制动请求checksum错误
    } break;

    case 0x08FB690E: {  // EHB 帧3(PGN 0xFB69,EHB_EPB 电子驻车)
        mEHBMsg.EHB_EPB_SystemStatus                = msg.data[0] & 0x03;         //1.1-1.2 驻车系统工作状态
        mEHBMsg.EHB_EPB_ParkingStatus               = (msg.data[0] >> 2) & 0x07;  //1.3-1.5 驻车系统驻车状态
        mEHBMsg.EHB_EPB_ParkingPressure             = msg.data[1];                //2.1-2.8 驻车系统液压
        mEHBMsg.EHB_EPB_FailureNum                  = msg.data[2] & 0x0F;         //3.1-3.4 驻车系统当前故障数
        mEHBMsg.EHB_EPB_FAU_PowerSupplyVoltageHigh  = msg.data[3] & 0x01;         //4.1 电源电压过高
        mEHBMsg.EHB_EPB_FAU_PowerSupplyVoltageLow   = (msg.data[3] >> 1) & 0x01;  //4.2 电源电压过低
        mEHBMsg.EHB_EPB_FAU_NoValueError            = (msg.data[3] >> 2) & 0x01;  //4.3 常开电磁阀异常
        mEHBMsg.EHB_EPB_FAU_NcValueError            = (msg.data[3] >> 3) & 0x01;  //4.4 常闭电磁阀异常
        mEHBMsg.EHB_EPB_FAU_SensorSupplyError       = (msg.data[3] >> 4) & 0x01;  //4.5 传感器电源异常
        mEHBMsg.EHB_EPB_FAU_ParkingPresSensorError  = (msg.data[3] >> 5) & 0x01;  //4.6 驻车系统液压信号异常
        mEHBMsg.EHB_EPB_FAU_ParkingPressureLow      = (msg.data[3] >> 6) & 0x01;  //4.7 驻车液压过低
        mEHBMsg.EHB_EPB_FAU_LosOfParkingReqSignal   = (msg.data[3] >> 7) & 0x01;  //4.8 驻车请求信号丢失
        mEHBMsg.EHB_EPB_FAU_ParkingReqRcError       = msg.data[4] & 0x01;         //5.1 驻车请求rollcounter错误
        mEHBMsg.EHB_EPB_FAU_ParkingReqCsError       = (msg.data[4] >> 1) & 0x01;  //5.2 驻车请求checksum错误
    } break;

    case 0x08FB1458: {  // EHB 接收方向帧4(VCU SA=0x58→EHB,Vehicle_Brake_Request 制动请求 20ms),canbus 旁听解析
        mEHBMsg.VHL_ATB_BrakePressRequest     = msg.data[0];                //1.1-1.8 请求制动力大小(raw×0.04 MPa)
        mEHBMsg.VHL_ATB_BrakePressRequestFlag = msg.data[1] & 0x03;         //2.1-2.2 制动请求标志
        mEHBMsg.VHL_ATB_RollingCounter        = (msg.data[6] >> 4) & 0x0F;  //7.5-7.8 滚动计数器(声明8bit/位跨4bit冲突I02,按位置4bit)
        mEHBMsg.VHL_ATB_CheckSum              = msg.data[7];                //8.1-8.8 校验和(求和宽度未定义I06,不校验)
    } break;

    case 0x08FB1558: {  // EHB 接收方向帧5(VCU SA=0x58→EHB,Vehicle_Parking_Request 驻车请求 100ms),canbus 旁听解析
        mEHBMsg.VHL_EPB_ParkingRequest  = msg.data[0] & 0x03;         //1.1-1.2 驻车请求
        mEHBMsg.VHL_EPB_RollingCounter  = (msg.data[6] >> 4) & 0x0F;  //7.5-7.8 滚动计数器(I02,按位置4bit)
        mEHBMsg.VHL_EPB_CheckSum        = msg.data[7];                //8.1-8.8 校验和(I06,不校验)
    } break;

    case 0x0CFD0358: {  // EHB 接收补充项(VCU SA=0x58→EHB):车辆车速(原表中文信号名)
        mEHBMsg.VHL_VehicleSpeed = (uint16_t)(msg.data[4] | (msg.data[5] << 8));  //5.1-6.8 ×0.1 km/h,0xFFFF 忽略
    } break;

    case 0x0CFD0058: {  // EHB 接收补充项:高压上电状态(原表「点火钥匙状态(改为:高压上电状态)」)
        mEHBMsg.VHL_HvPowerState = msg.data[2];                     //3.1-3.8 0x00断电/0x01上电/0xFF忽略
    } break;

    case 0x0CFD0158: {  // EHB 接收补充项:制动踏板状态
        mEHBMsg.VHL_BrakePedalStatus = msg.data[3];                 //4.1-4.8 0-100%,0xFF 忽略
    } break;

    case 0x08FD0258: {  // EHB 接收补充项:车辆当前档位(ASCII R,N,D,P;两字节摆放未定义 I09,存原始16bit)
        mEHBMsg.VHL_VehicleGear = (uint16_t)(msg.data[4] | (msg.data[5] << 8));  //5.1-6.8 小端拼接
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
