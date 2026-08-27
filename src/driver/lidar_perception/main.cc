#include "lidar_perception.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_perception");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  Segmenter seg(nh, private_nh);

  ros::spin();
  return 0;
}