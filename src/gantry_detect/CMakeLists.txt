cmake_minimum_required(VERSION 2.8.3)
project(gantry_detect)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  visualization_msgs
  message_generation
  pcl_ros
  tf
)

find_package(PCL REQUIRED)
find_package(ivlocmsg REQUIRED)

add_message_files(
  FILES
  gantry_state.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs visualization_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ivlocmsg_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}_node
  src/gantry_detect_node.cpp
  src/gantry_detector.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
