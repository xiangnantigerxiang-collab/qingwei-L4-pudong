cmake_minimum_required(VERSION 2.8.3)
project(lidar_perception)

SET(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  std_msgs
  sensor_msgs
  jsk_recognition_msgs
  message_generation
  laser_geometry
  pcl_conversions
  rospy
  tf
)
find_package(PCL 1.7 REQUIRED)
add_message_files(
  FILES
  perception_area.msg
  perception_area_array.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #CATKIN_DEPENDS roscpp sensor_msgs pcl_ros std_msgs jsk_recognition_msgs message_runtime
)


find_package(PCL REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})

include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_scan_node src/lidar_perception.cc main.cc)
target_link_libraries(${PROJECT_NAME}_scan_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
