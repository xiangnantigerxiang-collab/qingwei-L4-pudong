cmake_minimum_required(VERSION 3.0)
project(health_monitor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  topic_tools
  diagnostic_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(health_monitor_node
  src/health_monitor_node.cpp
  src/health_monitor.cpp
)
target_link_libraries(health_monitor_node ${catkin_LIBRARIES})

# 纯逻辑回归(不依赖 ROS 运行时,车载可直接跑)
add_executable(hz_meter_test test/hz_meter_test.cpp)
add_executable(state_classify_test test/state_classify_test.cpp)
