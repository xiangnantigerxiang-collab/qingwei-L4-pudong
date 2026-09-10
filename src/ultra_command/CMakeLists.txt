cmake_minimum_required(VERSION 3.0)
project(ultra_command)

add_compile_options(-std=c++11)

# 消费 robot(pnc) 包消息(/perception, /task_plan_msg, /cloud/msg/command_msg);
# 与 auto_couple 消费 ivlocmsg 同款跨包消息模式(catkin_make 合并 devel 下
# devel/include 全局可见)。std_msgs/geometry_msgs 为 robot 生成头的传递依赖
# (perception.msg 的 std_msgs/Header, object.msg 的 geometry_msgs/Point[])。
# 注意: 干净构建时不可单独 catkin_make --pkg ultra_command
# (robot 消息未先生成), 走 rebuild_all.sh 全量段。
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  robot
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ultra_command_node
  src/ultra_command_node.cpp
  src/ultra_command_comply.cpp
)
# 干净工作区并行构建时保证 robot 消息头先生成(auto_couple 同款)
add_dependencies(ultra_command_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ultra_command_node ${catkin_LIBRARIES})
