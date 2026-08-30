#include "can_comm_comply.h"

CanCommComply canCommComply;

void TaskPlanMsgCallBack(const robot::task_plan_msg &msg)
{
    canCommComply.SetTaskPlanData(msg);
}

void ControlMsgCallBack(const robot::control_msg &msg)
{
    canCommComply.SetControlData(msg);
}

void SoundLightMsgCallBack(const robot::sound_light_msg &msg)
{
    canCommComply.SetSoundLightData(msg);
}

int main(int argc,char **argv){
    ros::init(argc,argv,"can_comm_node");
    ros::NodeHandle nh;

    ros::Subscriber task_plan_sub = nh.subscribe(
        "/task_plan_msg", 1, TaskPlanMsgCallBack);
    ros::Subscriber control_sub = nh.subscribe(
        "/control_msg", 1 , ControlMsgCallBack);
    ros::Subscriber sound_light_sub = nh.subscribe(
        "/sound_light_msg", 1, SoundLightMsgCallBack);
	
    ros::Publisher can_comm_pub = nh.advertise<robot::can_comm_msg>(
        "/can_comm_msg", 1);

    ros::Rate loop_rate(20);

    while(ros::ok()){
        ros::spinOnce();
        canCommComply.CanCommProcess();
        can_comm_pub.publish(canCommComply.CanCommMsg);
    }

    return 0;
}


