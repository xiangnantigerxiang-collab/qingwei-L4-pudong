#include "path_plan_comply.h"
#include "robot/can_msg.h"
#include <cmath>
#include <fstream>
#include <visualization_msgs/MarkerArray.h>
#include "lattice/math_utils.h"
#include "lattice/gis_utils.h"
#include <yaml-cpp/yaml.h>

// 这些分片共享本翻译单元；不可单独加入 CMake。
// 原有五个业务分片仍按任务、感知、参考路径、输出、实验算法的顺序包含。
#include "path_plan_geometry.inc"
#include "path_plan_task.inc"
#include "path_plan_perception.inc"
#include "path_plan_reference.inc"
#include "path_plan_output.inc"
#include "path_plan_experimental.inc"
