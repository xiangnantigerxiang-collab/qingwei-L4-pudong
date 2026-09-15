#ifndef PERCEPTION_CONFIDENCE_FILTER_H
#define PERCEPTION_CONFIDENCE_FILTER_H

#include "robot/perception.h"

// 按给定阈值筛选：confidence < 阈值的对象剔除，相等时保留，不修改输入消息。
// dx 为横向宽，dy 为纵向长；参数依次为宽度闭区间、长度闭区间，单位米，任一越界就剔除。
// 下限须为有限非负数，上限须 >= 下限；上限允许正无穷，表示不设上限，NaN 禁止使用。
// 对象尺寸仍须为有限正数；下限等于上限时，只保留尺寸恰等于该值的对象。
// 返回结果保留 header、对象顺序及对象原有全部字段，不重新计算或归一化 confidence。
// 非有限 confidence 不参与输出；非法阈值抛出 std::invalid_argument，阈值不自动截断。
robot::perception FilterPerceptionByConfidence(const robot::perception& tPerception, double tConfidenceThreshold,
    double tMinWidth, double tMaxWidth, double tMinLength, double tMaxLength);

// 兼容原来的两参数/四参数调用：原参数仍为最小宽度、最小长度，两项上限均不限制。
robot::perception FilterPerceptionByConfidence(const robot::perception& tPerception, double tConfidenceThreshold,
    double tMinWidth = 0.0, double tMinLength = 0.0);

#endif  // PERCEPTION_CONFIDENCE_FILTER_H
