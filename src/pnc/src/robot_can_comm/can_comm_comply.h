#ifndef CAN_COMM_COMPLY
#define CAN_COMM_COMPLY

#include "../common/pubalgor/pubalgor.h"
#include "ros/ros.h"
#include "robot/control_msg.h"
#include "robot/sound_light_msg.h"
#include "robot/task_plan_msg.h"
#include "robot/can_comm_msg.h"

class CanCommComply
{
public:
    CanCommComply();
    ~CanCommComply();

    void SetTaskPlanData(robot::task_plan_msg task_plan_t);
    void SetControlData(robot::control_msg control_t);
    void SetSoundLightData(robot::sound_light_msg sound_light_t);
    void CanCommProcess();

    robot::can_comm_msg CanCommMsg;
    robot::control_msg ControlMsg;
};

#endif
