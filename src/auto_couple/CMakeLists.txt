cmake_minimum_required(VERSION 2.8.3)
project(auto_couple)

add_compile_options(-std=c++14)

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
  center_position.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros std_msgs jsk_recognition_msgs message_runtime
)

include_directories(
 include
 ${PCL_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node src/auto_couple_node.cpp src/auto_couple_core.cpp) 
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)




