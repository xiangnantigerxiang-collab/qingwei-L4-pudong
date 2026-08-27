cmake_minimum_required(VERSION 2.8.3)
project(ivlocmsg)

#set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs	
  sensor_msgs
  message_generation
  geometry_msgs
)

####include(/opt/ros/kinetic/share/catkin/cmake/env/CMakeLists.txt)


add_message_files(
    FILES
    ndt_status.msg
    ivmsglocpos.msg
    ivsensorgps.msg
    ivsensorimu.msg
    ivsensorodom.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs
   message_runtime
   geometry_msgs
)

include_directories(include 
   ${catkin_INCLUDE_DIRS}
)  
