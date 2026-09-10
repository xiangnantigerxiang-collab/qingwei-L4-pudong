/**
 * @class CenterPoint 
 * @brief 
 * @Author Yan Haixu 
 */

#include "./centerpoint_ros.h"

int main(int argc, char **argv)
{
  ros::init(argc , argv, "centerpoint");
  CenterPointRos app;
  app.CreateRosPubSub();
  ros::spin();
  return 0;
}
