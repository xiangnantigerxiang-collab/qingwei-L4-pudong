#include "robot_can_comm/can_comm_comply.h"

CanCommComply::CanCommComply()
{
    CanCommMsg.desireGear = GEAR_N;
    CanCommMsg.hookCmd = 0;
    CanCommMsg.sweepCmd = 0;
    CanCommMsg.sprayCmd = 0;
    CanCommMsg.unloadGarbage = 0;
    CanCommMsg.leftLightEnable = 0;
    CanCommMsg.rightLightEnable = 0;
    CanCommMsg.doubleLightEnable = 0;
    CanCommMsg.whistleEnable = 0;
}

CanCommComply::~CanCommComply()
{
    //nop
}

void CanCommComply::SetTaskPlanData(robot::task_plan_msg task_plan_t)
{
    CanCommMsg.hookCmd = task_plan_t.hookCmd;
    CanCommMsg.desireGear = task_plan_t.desireGear;
}

void CanCommComply::SetControlData(robot::control_msg control_t)
{
    ControlMsg = control_t;
    CanCommMsg.bypassProcessing = ControlMsg.bypassProcessing;
    CanCommMsg.vehicleSpeed = ControlMsg.vehicleSpeed;
}

void CanCommComply::SetSoundLightData(robot::sound_light_msg sound_light_t)
{
    CanCommMsg.leftLightEnable = sound_light_t.leftLightEnable;
    CanCommMsg.rightLightEnable = sound_light_t.rightLightEnable;
    CanCommMsg.doubleLightEnable = sound_light_t.doubleLightEnable;
    CanCommMsg.whistleEnable = sound_light_t.whistleEnable;
}

void CanCommComply::CanCommProcess()
{
    float ratio = 0.0;
    ros::param::get("vehicle_wheel_co", ratio);

    CanCommMsg.desireSpeed = ControlMsg.desireSpeed;
    CanCommMsg.desireAcc = ControlMsg.desireAcc;
    CanCommMsg.wheelAngle = ControlMsg.wheelAngle * ratio;
    CanCommMsg.brakePercent = ControlMsg.brakePercent;
    CanCommMsg.throttlePercent = ControlMsg.throttlePercent;
    /////////////////////////test/////////////////////////////////
    // float throttle_cmd = 0.0;
    // ros::param::get("throttle_cmd", throttle_cmd);
    // printf("throttle_cmd: %f\n", throttle_cmd);
    // CanCommMsg.throttlePercent = throttle_cmd;
    // if(throttle_cmd > 0)
    // {
    //     CanCommMsg.throttlePercent = throttle_cmd;
    // }
    ///////////////////////////////////////////////////////////////
    if(ControlMsg.throttlePercent - CanCommMsg.throttlePercent > 1.0) {
        CanCommMsg.throttlePercent += 1;
    }else CanCommMsg.throttlePercent = ControlMsg.throttlePercent;

    // if(CanCommMsg.desireSpeed == 0) {
    //    if(CanCommMsg.vehicleSpeed < 0.1) CanCommMsg.brakePercent = 0;
    //    else CanCommMsg.brakePercent = 70;
    // }
}

