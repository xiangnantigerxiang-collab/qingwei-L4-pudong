#ifndef CANBUS_COMPLY_H
#define CANBUS_COMPLY_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <pthread.h>

#include "ros/ros.h"
#include "ros/time.h"
#include <ctime>
#include <iostream>
#include <iostream>
#include <socketcan_interface/socketcan.h>
#include <serial/serial.h>
#include <can_msgs/Frame.h>
#include "struct_type.h"
#include "canbus/can_comm_msg.h"
#include "canbus/can_msg.h"
#include "canbus/control_msg.h"
#include "canbus/sound_light_msg.h"
#include "canbus/task_plan_msg.h"
#include "canbus/can_comm_msg.h"
#include "robot/task_plan_msg.h"

class CanbusComply
{
public:
    CanbusComply() {
        can_comm_cmd.desireGear = GEAR_N;
        can_comm_cmd.hookCmd = 0;
        can_comm_cmd.sweepCmd = 0;
        can_comm_cmd.sprayCmd = 0;
        can_comm_cmd.unloadGarbage = 0;
        can_comm_cmd.leftLightEnable = 0;
        can_comm_cmd.rightLightEnable = 0;
        can_comm_cmd.doubleLightEnable = 0;
        can_comm_cmd.whistleEnable = 0;
        can_comm_cmd.rgbLightEnable = 0;

	mCanMsg.throttlePercent = 0;
        mCanMsg.brakePercent = 0;
        mCanMsg.wheelAngle = 0;
        mCanMsg.curGear = GEAR_N;
        mCanMsg.hookState = 0;
        mCanMsg.linkPallet = 0;
        mCanMsg.controlPanelState = 0;
        mCanMsg.emergencyStop = 0;
        mCanMsg.batteryPower = 80;
        mCanMsg.vehicleSpeed = 0;
        mCanMsg.eabPanelState = 0;

        mBuzzerCount = 0;
        mCount = 0;

        CalibrateCmd = 0;
    }

    ~CanbusComply(){};

    void VehicleComm();
    void PublishVehicleStateParaMsg(ros::Publisher tPub);

    void forwardCamera();
    void backwardCambera();
    void checkCamera();

    int handle;
    uint8_t CalibrateCmd = 0;

    int brakecmd = 0;
    int HookWarning = 0;
    int HookState = 0;
    int LinkState = 0;
    int LockState = 0;
    int HookPos = 0;
    int LinkPos = 0;
    int ReleaseEnable = 0;
    int HookEnable = 0;

    double aeb_speed = 0;

    struct SysTime {
        double now;
        double init;
        double planningStop;
        double taskStart;
        double vehicleStop;
        double vehicleRun;
        double idPass;
        double msgPerception;
        double msgNavigation;
        double msgLidar;
        double msgCamera;
    }sysTime;

    canbus::can_comm_msg can_comm_cmd;
    canbus::can_msg mCanMsg;
    robot::task_plan_msg task_msg;

public:
    void SendVehicleControlCmd(uint32_t id,unsigned char *data);
    void RecvCan0Data(const can_msgs::Frame &msg);

    uint8_t mBuzzerCount;//add
    uint8_t mCurGear;
    uint8_t mCount;
};

#endif



