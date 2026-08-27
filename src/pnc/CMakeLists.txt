cmake_minimum_required(VERSION 2.8.3)
project(robot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  jsk_recognition_msgs
)

# ÅÐ¶ÏÊÇ·ñÎª ARM ¼Ü¹¹
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|ARM|aarch64|AARCH64")
    message(STATUS "Building for ARM architecture")
    add_definitions(-DARCH_ARM)
    set(IS_ARM TRUE)
    set(IS_X86 FALSE)
else()
    message(STATUS "Building for x86 architecture")
    add_definitions(-DARCH_X86)
    set(IS_ARM FALSE)
    set(IS_X86 TRUE)
endif()

if(IS_ARM)
    message("arm architecture detected, using ARM-specific libraries")
    set(ENV_QPOASES_INCLUDE  "${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/arm/qpoases/include")
    file(GLOB ENV_QPOASES_LIB ${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/arm/qpoases/lib/Debug/*.a)
    set(ENV_PROJ4_INCLUDE  "${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/arm/proj4/include")
    file(GLOB ENV_PROJ4_LIB ${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/arm/proj4/lib/Debug/libproj4_linux_x64.a)
    set(ENV_OSQP_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/arm/osqp/include")
    file(GLOB ENV_OSQP_LIB ${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/arm/osqp/lib/*.so)
else()
    message("x86 architecture detected")
    set(ENV_QPOASES_INCLUDE  "${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/qpoases/include")
    file(GLOB ENV_QPOASES_LIB ${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/qpoases/lib/Debug/*.a)
    set(ENV_PROJ4_INCLUDE  "${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/proj4/include")
    file(GLOB ENV_PROJ4_LIB ${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/proj4/lib/Debug/libproj4_linux_x64.a)
    set(ENV_OSQP_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/osqp/include")
    file(GLOB ENV_OSQP_LIB ${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/osqp/lib/*.so)

endif()


include_directories(${ENV_QPOASES_INCLUDE})
link_libraries(${ENV_QPOASES_LIB})


include_directories(${ENV_PROJ4_INCLUDE})
link_libraries(${ENV_PROJ4_LIB})


include_directories(${ENV_OSQP_INCLUDE})
link_libraries(${ENV_OSQP_LIB})

add_message_files(
  FILES
  hook_position.msg
  navigation_msg.msg
  can_comm_msg.msg
  can_msg.msg
  control_msg.msg
  path_plan_msg.msg
  path_plan_status.msg
  sound_light_msg.msg
  task_plan_msg.msg
  task_plan_status.msg
  vehicle_task_status.msg
  object.msg
  perception.msg
  perception_msg.msg
  hook_position.msg
  TLStatus.msg
  acc.msg
  rs232.msg

  RemoteSignal.msg
  TargetInfo.msg 
  TaskInfo.msg
  TaskStatus.msg
  palletpos.msg

  CommandMsg.msg
  CommandValue.msg
  HeadingPoint.msg
  NotifyMsg.msg
  NotifyObject.msg
  NotifyRsu.msg
  NotifyValue.msg
  PointLon.msg
  Point.msg
  RoutingPath.msg
  RunningMsg.msg
  RunningValue.msg
  WarningMsg.msg
  WarningValue.msg
  v2nHeartBeat.msg
  v2nHeartBeatValue.msg
  v2nKeyPoint.msg
  v2nRunningFeedback.msg
  v2nCommandFeedback.msg
  v2nFeedbackValue.msg
  AirCraftParkingPort.msg
#  DiagnosticFault.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs  
  std_msgs  
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIB_DIRS} 
  lib 
  /usr/local/lib/		
)

set (TARGET_NAME ${CMAKE_PROJECT_NAME}${SUFFIX_STR})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

file(GLOB_RECURSE SRC_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_library(pubalgor 
  src/common/pubalgor/pubalgor.cpp
)
target_link_libraries(pubalgor
  ${catkin_LIBRARIES}
)

add_library(spline
  src/common/spline/Spline.cpp
)
target_link_libraries(spline
  ${catkin_LIBRARIES}
)

add_library(task_plan_comply 
  src/robot_task_plan/task_plan_comply.cpp
)
target_link_libraries(task_plan_comply 
  pubalgor 
  jsoncpp
  ${catkin_LIBRARIES} 
)

add_library(path_plan_comply 
  src/robot_path_plan/path_plan_comply.cpp 
  src/lattice_plan/lattice_planner.cpp 
  src/lattice_plan/end_condition_sampler.cpp
  src/lattice_plan/lattice_trajectory1d.cpp
  src/lattice_plan/trajectory1d_generator.cpp
  src/lattice_plan/trajectory_combiner.cpp
  src/lattice_plan/trajectory_cost.cpp
  src/collisioncheck/collision_detection.cpp
  src/pathdecision/lanechange_decision.cpp
  src/pathdecision/path_decision.cpp
  src/pathmatch/pathmatcher.cpp
  src/reference_line/qp_spline_corridor_reference_line_generation.cpp
  src/reference_line/reference_line.cpp
  src/reference_line/reference_line_provider.cpp
  src/reference_line/reference_point.cpp
  src/common/curve1d/cubic_polynomial_curve1d.cpp
  src/common/curve1d/quartic_polynomial_curve1d.cpp
  src/common/curve1d/quintic_polynomial_curve1d.cpp
  src/common/line/line_segment2d.cpp
  src/common/line/linear_interpolation.cpp
  src/common/math/cartesian_frenet_conversion.cpp
  src/common/math/math_utils.cpp
  src/common/math/search.cpp
  src/common/path/discretized_path.cpp
  src/common/path/frenet_frame_path.cpp
  src/common/path/sl_boundary.cpp
  src/common/polynomial_xd/curve_math.cpp
  src/common/polynomial_xd/polynomial_xd.cpp
  src/common/smooth_line/qp_spline_corridor/qp_spline_corridor.cpp
  src/common/surface/aabox2d.cpp
  src/common/surface/box2d.cpp
  src/common/surface/polygon2d.cpp
  src/common/surface/vec2d.cpp
  src/trans/trans_data.cpp
  src/speedplan/speed_planning.cpp
  src/robot_path_plan/adaptiveHook.cpp
  src/robot_path_plan/lattice/collision_check_with_bbox.cpp
  src/robot_path_plan/lattice/my_lattice_planner.cpp
)
target_link_libraries(path_plan_comply 
  pubalgor 
  jsoncpp
  ${catkin_LIBRARIES} 
)

add_library(control_comply 
  src/robot_control/control_comply.cpp
  src/robot_control/lat_controller/math_utils.h
  src/robot_control/lat_controller/pose2d.h
  src/robot_control/lat_controller/lat_controller.cpp

)
target_link_libraries(control_comply 
  pubalgor
  ${catkin_LIBRARIES} 
)

add_library(can_comm_comply 
  src/robot_can_comm/can_comm_comply.cpp
)
target_link_libraries(can_comm_comply 
  pubalgor
  ${catkin_LIBRARIES}
)

add_executable(navigation_node 
  src/robot_navigation/navigation_node.cpp
)
add_dependencies(navigation_node 
  ${PROJECT_NAME}_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(navigation_node
  ${catkin_LIBRARIES} 
  pubalgor
)

add_executable(task_plan_node 
  src/robot_task_plan/task_plan_node.cpp
  src/robot_task_plan/frame_transform.cc
)
add_dependencies(task_plan_node
  ${PROJECT_NAME}_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(task_plan_node
  ${catkin_LIBRARIES} 
  task_plan_comply 
  yaml-cpp 
)

add_executable(path_plan_node 
  src/robot_path_plan/path_plan_node.cpp
)
add_dependencies(path_plan_node 
  ${PROJECT_NAME}_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(path_plan_node
  ${catkin_LIBRARIES} 
  path_plan_comply 
  yaml-cpp
  spline 
)

add_executable(control_node 
  src/robot_control/control_node.cpp 
  src/robot_control/geometric_control.cpp
  src/robot_control/acc_control.cpp
  src/robot_control/lateral_control.cpp
  src/robot_control/speed_control.cpp
)
add_dependencies(control_node
  ${PROJECT_NAME}_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(control_node
  ${catkin_LIBRARIES} 
  control_comply 
  yaml-cpp 
  spline 
)

add_executable(can_comm_node 
  src/robot_can_comm/can_comm_node.cpp
)
add_dependencies(can_comm_node
  ${PROJECT_NAME}_generate_messages_cpp	
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(can_comm_node
  ${catkin_LIBRARIES} 
  can_comm_comply
)

add_executable(perception_msg_convert
  src/robot_perception_convert/perception_msg_convert.cpp
  ${proto_sources}
)
add_dependencies(perception_msg_convert
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(perception_msg_convert
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
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
