cmake_minimum_required(VERSION 2.8.3)
project(robot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  tf
  jsk_recognition_msgs
  roslib
)

# hdmap 是独立包，既支持同工作空间 catkin 构建，也支持已安装 SDK 的导出目标。
find_package(hdmap REQUIRED CONFIG)

# 预编译三方库按架构选择（x86 开发机 / aarch64 车载 Orin）
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|ARM|aarch64|AARCH64")
    message(STATUS "Building for ARM architecture")
    set(THIRDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/arm)
else()
    message(STATUS "Building for x86 architecture")
    set(THIRDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rd-party)
endif()

# OSQP：已编译的 qp_spline_corridor.cpp 与参考线走廊生成（休眠代码，运行期未被
#        调用）引用其符号，链接必须保留——与下方 path_plan 段"休眠"口径一致
# proj4：仅 trans_data.cpp 包含其头文件，无符号调用，不链接
# qpoases：头文件被 spline_2d_solver.h（经 reference_line 头传递）包含，编译需要；
#          但已编译代码不实例化其求解器，无符号引用，故不链接库
include_directories(${THIRDPARTY_DIR}/osqp/include)
file(GLOB OSQP_LIBS ${THIRDPARTY_DIR}/osqp/lib/*.so)
include_directories(${THIRDPARTY_DIR}/proj4/include)
include_directories(${THIRDPARTY_DIR}/qpoases/include)

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
  task_fence_guard.msg
  task_plan_status.msg
  vehicle_task_status.msg
  object.msg
  perception.msg
  perception_msg.msg
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
  ${CMAKE_CURRENT_SOURCE_DIR}/include              # Eigen 与 common/(config|pnc_point|struct_type...)
  ${CMAKE_CURRENT_SOURCE_DIR}/src                  # 跨节点引用根：robot_control/... robot_path_plan/...
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_path_plan  # 规划算法库引用根：common/... reference_line/... speedplan/...
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIB_DIRS}
  /usr/local/lib
)

# ============================================================
# 公共算法库（多节点共用，位于 robot_path_plan/common/）
# ============================================================
add_library(pubalgor
  src/robot_path_plan/common/pubalgor/pubalgor.cpp
)
target_link_libraries(pubalgor
  ${catkin_LIBRARIES}
)

add_library(spline
  src/robot_path_plan/common/spline/Spline.cpp
)
target_link_libraries(spline
  ${catkin_LIBRARIES}
)

# ============================================================
# 任务规划节点：/cloud/task/task_info 等 9 订阅 → /task_plan_msg、
# /cloud/task/task_status 及 3 个 v2n 回执话题（共 5 发布）
# ROS/业务解耦（2026-08-30，同 canbus_core 模式）：
# task_plan_core 零 ROS 业务库 + task_plan_node 薄适配
# ============================================================
add_library(task_plan_core
  src/robot_task_plan/task_plan_core.cpp
  src/robot_task_plan/frame_transform.cc
)
target_link_libraries(task_plan_core
  pubalgor
  yaml-cpp
  ${catkin_LIBRARIES}
)

add_executable(task_plan_node
  src/robot_task_plan/task_plan_node.cpp
)
target_link_libraries(task_plan_node
  ${catkin_LIBRARIES}
  task_plan_core
)

# ============================================================
# 路径规划节点：task_plan_msg → plan_path_msg/path_plan_status
# 含休眠的 lattice/参考线/速度规划算法族（编译进库、运行期未调用）
# ============================================================
add_library(path_plan_comply
  # 主链
  # path_plan_comply.cpp 原位包含几何辅助与五个业务入口分片；
  # task/reference/safety 子目录由对应入口继续包含，所有 .inc 均非独立翻译单元。
  # 不要在此重复列出 .inc，以免破坏静态状态和产生重复定义。
  src/robot_path_plan/path_plan_comply.cpp
  src/robot_path_plan/adaptiveHook.cpp
  src/robot_path_plan/collisioncheck/collision_detection.cpp
  # 自研 lattice（休眠）
  src/robot_path_plan/lattice/collision_check_with_bbox.cpp
  src/robot_path_plan/lattice/my_lattice_planner.cpp
  # Apollo lattice 规划器（休眠）
  src/robot_path_plan/lattice_plan/lattice_planner.cpp
  src/robot_path_plan/lattice_plan/end_condition_sampler.cpp
  src/robot_path_plan/lattice_plan/lattice_trajectory1d.cpp
  src/robot_path_plan/lattice_plan/trajectory1d_generator.cpp
  src/robot_path_plan/lattice_plan/trajectory_combiner.cpp
  src/robot_path_plan/lattice_plan/trajectory_cost.cpp
  # 路径决策与匹配（休眠）
  src/robot_path_plan/pathdecision/lanechange_decision.cpp
  src/robot_path_plan/pathdecision/path_decision.cpp
  src/robot_path_plan/pathmatch/pathmatcher.cpp
  # 参考线生成与平滑（休眠，OSQP）
  src/robot_path_plan/reference_line/reference_line.cpp
  src/robot_path_plan/reference_line/reference_line_provider.cpp
  src/robot_path_plan/reference_line/reference_point.cpp
  src/robot_path_plan/reference_line/qp_spline_corridor_reference_line_generation.cpp
  # 速度规划（休眠）
  src/robot_path_plan/speedplan/speed_planning.cpp
  # 路径数据转换（休眠）
  src/robot_path_plan/trans/trans_data.cpp
  # 规划公共数学库
  src/robot_path_plan/common/curve1d/cubic_polynomial_curve1d.cpp
  src/robot_path_plan/common/curve1d/quartic_polynomial_curve1d.cpp
  src/robot_path_plan/common/curve1d/quintic_polynomial_curve1d.cpp
  src/robot_path_plan/common/line/line_segment2d.cpp
  src/robot_path_plan/common/line/linear_interpolation.cpp
  src/robot_path_plan/common/math/cartesian_frenet_conversion.cpp
  src/robot_path_plan/common/math/math_utils.cpp
  src/robot_path_plan/common/math/search.cpp
  src/robot_path_plan/common/path/discretized_path.cpp
  src/robot_path_plan/common/path/frenet_frame_path.cpp
  src/robot_path_plan/common/path/sl_boundary.cpp
  src/robot_path_plan/common/polynomial_xd/curve_math.cpp
  src/robot_path_plan/common/polynomial_xd/polynomial_xd.cpp
  src/robot_path_plan/common/smooth_line/qp_spline_corridor/qp_spline_corridor.cpp
  src/robot_path_plan/common/surface/aabox2d.cpp
  src/robot_path_plan/common/surface/box2d.cpp
  src/robot_path_plan/common/surface/polygon2d.cpp
  src/robot_path_plan/common/surface/vec2d.cpp
)
target_link_libraries(path_plan_comply
  pubalgor
  ${OSQP_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(path_plan_node
  src/robot_path_plan/path_plan_node.cpp
)
# 仅规划节点使用闸机消息；不把检测算法或 PCL 引入其他 PNC 目标。
find_package(gantry_detect REQUIRED CONFIG)
target_include_directories(path_plan_node PRIVATE ${gantry_detect_INCLUDE_DIRS})
if(gantry_detect_EXPORTED_TARGETS)
  add_dependencies(path_plan_node ${gantry_detect_EXPORTED_TARGETS})
endif()
target_link_libraries(path_plan_node
  ${catkin_LIBRARIES}
  path_plan_comply
  pubalgor
  ${OSQP_LIBS}
  yaml-cpp
  spline
)

# ============================================================
# 控制节点：plan_path_msg → control_msg
# ============================================================
add_library(control_comply
  src/robot_control/control_comply.cpp
  src/robot_control/stanley_controller/stanley_controller.cpp
)
target_link_libraries(control_comply
  pubalgor
  ${catkin_LIBRARIES}
)

add_executable(control_node
  src/robot_control/control_node.cpp
  src/robot_control/lateral_reverse_control.cpp
  src/robot_control/longitudinal_speed_control.cpp
)
target_link_libraries(control_node
  ${catkin_LIBRARIES}
  control_comply
  spline
)

# ============================================================
# CAN 通信节点：task/control/声光 → can_comm_msg
# ============================================================
add_library(can_comm_comply
  src/robot_can_comm/can_comm_comply.cpp
)
target_link_libraries(can_comm_comply
  pubalgor
  ${catkin_LIBRARIES}
)

add_executable(can_comm_node
  src/robot_can_comm/can_comm_node.cpp
)
target_link_libraries(can_comm_node
  ${catkin_LIBRARIES}
  can_comm_comply
)

# ============================================================
# 定位导航节点：ivlocmsg → navigation_msg
# ============================================================
add_executable(navigation_node
  src/robot_navigation/navigation_node.cpp
)
target_link_libraries(navigation_node
  ${catkin_LIBRARIES}
  pubalgor
)

# ============================================================
# 感知转换节点：/box → /perception
# ============================================================
add_executable(perception_msg_convert
  src/robot_perception_convert/perception_msg_convert.cpp
)
if(TARGET hdmap::hdmap_server)
  set(ROBOT_HDMAP_LIBRARY hdmap::hdmap_server)
else()
  # 隔离构建时，catkin 的包配置提供传统 include/library 变量。
  target_include_directories(perception_msg_convert PRIVATE ${hdmap_INCLUDE_DIRS})
  set(ROBOT_HDMAP_LIBRARY ${hdmap_LIBRARIES})
endif()
target_link_libraries(perception_msg_convert
  ${catkin_LIBRARIES}
  ${ROBOT_HDMAP_LIBRARY}
)

# 消息生成依赖：所有编译 robot/*.h 的目标都需等待消息头生成，避免并行编译竞态
set(ROBOT_MESSAGE_DEPS
  ${PROJECT_NAME}_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
foreach(_target
    task_plan_core
    path_plan_comply
    control_comply
    can_comm_comply
    task_plan_node
    path_plan_node
    control_node
    can_comm_node
    navigation_node
    perception_msg_convert)
  add_dependencies(${_target} ${ROBOT_MESSAGE_DEPS})
endforeach()

install(TARGETS
    task_plan_node
    path_plan_node
    control_node
    can_comm_node
    navigation_node
    perception_msg_convert
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config param path
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
