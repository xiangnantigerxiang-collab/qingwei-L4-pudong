#include <ros/ros.h>

#include "gantry_detect/gantry_detector.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gantry_detect_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  GantryDetector detector(nh, pnh);
  ros::spin();
  return 0;
}
