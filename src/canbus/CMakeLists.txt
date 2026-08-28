cmake_minimum_required(VERSION 2.8.3)
project(canbus)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  socketcan_bridge
  socketcan_interface
  can_msgs
  geometry_msgs
  serial
)

add_message_files(
  FILES
  can_comm_msg.msg
  can_msg.msg
  control_msg.msg
  navigation_msg.msg
  path_plan_msg.msg
  path_plan_status.msg
  platform_msg.msg
  sound_light_msg.msg
  task_plan_msg.msg
  object.msg
  perception.msg
  hook_position.msg
  TLStatus.msg
  perception_area.msg
  #fms task msg
  Point.msg 
  TargetInfo.msg 
  TaskInfo.msg
  TaskStatus.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs  
  std_msgs  
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIB_DIRS}
)

set (TARGET_NAME ${CMAKE_PROJECT_NAME}${SUFFIX_STR})

include_directories (
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(aeb_can_node
  src/canbus_node.cpp
  src/canbus_core.cpp
)
add_dependencies(aeb_can_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(aeb_can_node
  ${catkin_LIBRARIES} 
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
