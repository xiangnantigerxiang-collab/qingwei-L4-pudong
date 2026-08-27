cmake_minimum_required(VERSION 3.0.2)
project(lakibeam1)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(include${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_scan_node src/lakibeam1_scan.cpp src/remote.cpp)
target_link_libraries(${PROJECT_NAME}_scan_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} curl)

add_executable(${PROJECT_NAME}_pcd_node src/lakibeam1_pcd.cpp src/remote.cpp)
target_link_libraries(${PROJECT_NAME}_pcd_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} curl)

install(TARGETS ${PROJECT_NAME}_scan_node ${PROJECT_NAME}_pcd_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
