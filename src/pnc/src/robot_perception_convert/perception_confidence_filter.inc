// 由 perception_msg_convert.cpp 单独包含，沿用现有编译入口，不修改目录外 CMake。
#include "perception_confidence_filter.h"
#include <cmath>
#include <stdexcept>
#include <limits>

robot::perception FilterPerceptionByConfidence(const robot::perception& tPerception, double tConfidenceThreshold,
    double tMinWidth, double tMaxWidth, double tMinLength, double tMaxLength) {
    if(!std::isfinite(tConfidenceThreshold)) {
        throw std::invalid_argument("FilterPerceptionByConfidence requires a finite threshold");
    }
    if(!std::isfinite(tMinWidth) || tMinWidth < 0 || !std::isfinite(tMinLength) || tMinLength < 0 ||
       std::isnan(tMaxWidth) || tMaxWidth < tMinWidth || std::isnan(tMaxLength) || tMaxLength < tMinLength) {
        // 不自动交换上下限，避免配置错误静默变成错误的障碍物输出；正无穷上限允许通过。
        throw std::invalid_argument("FilterPerceptionByConfidence requires valid nonnegative size intervals");
    }

    robot::perception result;
    result.header = tPerception.header;
    result.objs.reserve(tPerception.objs.size());  // 一次分配，避免筛选过程中反复扩容。
    for(std::size_t i = 0; i < tPerception.objs.size(); ++i) {
        const auto& object = tPerception.objs[i];
        // NaN 无法参与大小比较，非有限置信度不能作为有效目标通过阈值筛选。
        if(!std::isfinite(object.confidence) || object.confidence < tConfidenceThreshold) {
            continue;
        }
        // dx 为横向宽，dy 为纵向长，直接比较原始尺寸，不根据 heading 交换长宽。
        if(!std::isfinite(object.dx) || !std::isfinite(object.dy) || object.dx <= 0 || object.dy <= 0 ||
           object.dx < tMinWidth || object.dx > tMaxWidth || object.dy < tMinLength || object.dy > tMaxLength) {
            continue;
        }
        result.objs.push_back(object);  // 整体复制，保留 HDMap type、几何和原始置信度。
    }
    return result;
}

robot::perception FilterPerceptionByConfidence(const robot::perception& tPerception, double tConfidenceThreshold,
    double tMinWidth, double tMinLength) {
    // 旧调用保持两个尺寸下限的含义，统一进入区间实现，避免两套筛选逻辑不一致。
    const double max_size = std::numeric_limits<double>::infinity();
    return FilterPerceptionByConfidence(tPerception, tConfidenceThreshold, tMinWidth, max_size, tMinLength, max_size);
}
