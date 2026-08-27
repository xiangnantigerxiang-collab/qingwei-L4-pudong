cmake_minimum_required(VERSION 2.8.3)
project(view)
                  
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  tf
  message_generation
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
)

add_message_files(
  FILES	
  bypass_msg.msg
  can_comm_msg.msg
  can_msg.msg
  control_msg.msg
  decision_msg.msg
  download_msg.msg
  fault_msg.msg
  fms_msg.msg
  hook_position.msg
  keypoints.msg
  navigation_msg.msg
  object1.msg
  obstac.msg
  path_plan_msg.msg
  path_plan_status.msg
  perception_msg.msg
  platform_msg.msg
  self_check_msg.msg
  Sick_msg.msg
  sound_light_msg.msg
  state_machine_msg.msg
  task_plan_msg.msg
  task_plan_status.msg
  TLStatus.msg
  vehicle_task_status.msg
  visionObjects.msg
  palletpos.msg

  ivmsglocpos.msg
  ivsensorgps.msg
  ivsensorimu.msg
  ivsensorodom.msg
  ndt_status.msg

  object.msg
  perception.msg
  pose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include 
  CATKIN_DEPENDS roscpp rospy message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(simviewer
  src/simviewer.cpp
  src/draw.cpp
  src/thread.cpp
)
add_dependencies(simviewer
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(simviewer
  ${catkin_LIBRARIES}
)

