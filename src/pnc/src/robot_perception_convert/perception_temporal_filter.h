#ifndef PERCEPTION_TEMPORAL_FILTER_H
#define PERCEPTION_TEMPORAL_FILTER_H

#include <vector>
#include <utility>
#include "robot/object.h"
#include "robot/perception.h"

// 输入为 HDMap 分类后的原始帧和采集时间（秒）；只读取，不修改调用方保存的历史。
// 丢弃距今 >=2 秒、未来或非有限时间戳；同时间戳只使用输入中最后一帧，空帧也参与归一化。
// 每帧每个目标最多累计一次 exp(-age/1s)，confidence = 目标权重和 / 有效帧权重和。
// 输出保留每个目标最新观测的全部字段，仅重算 confidence；未设置置信度删除阈值。
// 按用户约定，仅用 x/y/dx/dy 构造地图轴对齐矩形；非法框抛出 std::invalid_argument。
robot::perception FilterPerceptionHistory(const std::vector<std::pair<robot::perception, double>>& tHistory);

#endif  // PERCEPTION_TEMPORAL_FILTER_H
