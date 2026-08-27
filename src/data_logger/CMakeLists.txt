cmake_minimum_required(VERSION 3.0.2)
project(data_logger)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  canbus
  robot
  roslib
  # log_util
  # project_config
)

find_package(yaml-cpp REQUIRED)

catkin_package(CATKIN_DEPENDS roslib project_config)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${project_config_INCLUDE_DIRS}/project_config/include
  # ${CMAKE_CURRENT_SOURCE_DIR}/../log_util/include
  # ${CMAKE_CURRENT_SOURCE_DIR}/../log_util/include/log_util
)

message("catkin_INCLUDE_DIRS : ${catkin_INCLUDE_DIRS}")
message("project_config_INCLUDE_DIRS/project_config/include : ${project_config_INCLUDE_DIRS}/project_config/include")

add_executable(data_logger_node src/data_logger_node.cpp src/data_logger.cpp src/csv_logger.cpp)
# add_executable(log_test src/test.cpp)

target_link_libraries(data_logger_node
  ${catkin_LIBRARIES}
  yaml-cpp
  atomic
)

target_link_libraries(data_logger_node
${catkin_LIBRARIES}
)

# target_link_libraries(log_test ${catkin_LIBRARIES})

#############
## Install ##
#############

# install(TARGETS data_logger_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(TARGETS data_logger_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
