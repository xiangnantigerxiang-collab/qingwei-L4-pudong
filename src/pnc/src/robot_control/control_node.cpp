// 控制节点 ROS 入口：订阅 navigation_msg / plan_path_msg / path_plan_status /
// can_msg / /camera/tl_status / task_plan_msg / perception，20Hz 主循环调用
// ControlComply 并发布 control_msg 与 acc
#include "control_comply.h"

ControlComply controlComply;

void NavigationMsgCallBack(const robot::navigation_msg::ConstPtr& msg) {
  controlComply.SetNavigationData(*msg);
}

void PathPlanMsgCallBack(const robot::path_plan_msg::ConstPtr& msg) {
  controlComply.SetPathPlanData(*msg);
}

void CanMsgCallBack(const robot::can_msg::ConstPtr& msg) {
  controlComply.SetCanData(*msg);
}

void TlStatusCoorCallback(const robot::TLStatus::ConstPtr& msg) {
  controlComply.SetTlStatusData(*msg);
  ROS_INFO("/camera/tl_status:%d", static_cast<int>(msg->light_status));
}

void PerceptionMsgCallBack(const robot::perception& msg) {  // zhangyu 20220213
  controlComply.LidarObject = msg;
}

void pathStatusCallback(const robot::path_plan_statusConstPtr& msg) {
  controlComply.SetPathStatusData(*msg);
}

void TaskMsgCallBack(const robot::task_plan_msg& msg) {
  controlComply.setTaskPlanData(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "control_node");
  ros::NodeHandle nh;

  ros::Subscriber navigation_sub =
      nh.subscribe("/navigation_msg", 1, NavigationMsgCallBack,
                   ros::TransportHints().tcpNoDelay());
  ros::Subscriber path_plan_sub =
      nh.subscribe("/plan_path_msg", 1, PathPlanMsgCallBack,
                   ros::TransportHints().tcpNoDelay());
  ros::Subscriber path_status_sub =
      nh.subscribe("/path_plan_status", 1, pathStatusCallback,
                   ros::TransportHints().tcpNoDelay());
  ros::Subscriber can_msg_sub =
      nh.subscribe("/can_msg", 10, CanMsgCallBack,
                   ros::TransportHints().tcpNoDelay());
  ros::Subscriber tl_status_sub =
      nh.subscribe("/camera/tl_status", 1, TlStatusCoorCallback,
                   ros::TransportHints().tcpNoDelay());
  ros::Subscriber task_sub =
      nh.subscribe("/task_plan_msg", 1, TaskMsgCallBack,
                   ros::TransportHints().tcpNoDelay());

  ros::Subscriber perception_sub =
      nh.subscribe("/perception", 1, PerceptionMsgCallBack,
                   ros::TransportHints().tcpNoDelay());

  ros::Publisher control_pub =
      nh.advertise<robot::control_msg>("/control_msg", 10);

  ros::Publisher acc_pub = nh.advertise<robot::acc>("/acc", 10);
  controlComply.LoadPathFile("fence");

  ros::Rate loop_rate(20);
  ros::param::set("alarmcmd", 0);
  while (ros::ok()) {
    nh.setParam("/sound/play", controlComply.SoundPlayCommand);

    ros::spinOnce();

    controlComply.VehicleControl();
    controlComply.FenceAlarm();  // 电子围栏检测，设置标志位
    controlComply.PublishMessage(control_pub);

    acc_pub.publish(controlComply.acc_msg);

    loop_rate.sleep();
  }
  return 0;
}
