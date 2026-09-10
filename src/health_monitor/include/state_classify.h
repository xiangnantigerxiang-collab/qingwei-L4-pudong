// state_classify.h —— 话题状态分类纯函数,无 ROS 依赖,供单测直接覆盖
// 评估顺序(spec 4.5):event/sampling 判定优先于 stale。
#pragma once
#include <cstdint>

enum TopicStateE {
    TOPIC_OK = 0,        // 窗口有数据且新鲜
    TOPIC_STALE = 1,     // 曾有数据但超过 stale_s(仅非 event 话题)
    TOPIC_EVENT = 2,     // 事件驱动话题,合法静默
    TOPIC_SAMPLING = 3,  // 重载荷轮转窗口外(含等待首采)
    TOPIC_NO_DATA = 4,   // 发现后长期 0 帧 / 末窗 0 帧
    TOPIC_NEW = 5        // 刚发现,信息不足
};

struct ClassifyInput {
    bool is_event;
    bool is_heavy;
    bool window_open;       // 重载荷当前在采样窗内(窗内按新鲜度正常判)
    bool ever_sampled;      // 重载荷是否完成过至少一个采样窗
    uint64_t now_ns;
    uint64_t first_seen_ns;
    uint64_t last_msg_ns;   // 0 = 从未收到消息
    int window_msgs;        // 当前窗口帧数
    double stale_s;
    double no_data_grace_s;
    double discover_period_s;
};

inline double SecBetween(uint64_t a_ns, uint64_t b_ns)
{
    if (a_ns <= b_ns) return 0.0;
    return (double)(a_ns - b_ns) / 1e9;
}

inline TopicStateE ClassifyTopicState(const ClassifyInput &in)
{
    bool has_msgs = in.last_msg_ns != 0;
    double age_first = SecBetween(in.now_ns, in.first_seen_ns);

    if (!has_msgs) {
        if (in.is_event) {
            if (age_first < 2.0 * in.discover_period_s) return TOPIC_NEW;
            return TOPIC_EVENT;   // 事件话题合法静默,不算 no_data
        }
        if (age_first >= in.no_data_grace_s) return TOPIC_NO_DATA;
        return TOPIC_NEW;
    }
    if (in.is_event) return TOPIC_EVENT;

    if (in.is_heavy && !in.window_open) {
        if (!in.ever_sampled) return TOPIC_SAMPLING;   // 等待首次轮转
        if (in.window_msgs == 0) return TOPIC_NO_DATA; // 末窗 0 帧(历史消息仍在)
        return TOPIC_SAMPLING;                          // 保留末窗快照
    }
    // 新鲜度判定(light 常订,或 heavy 窗口内)
    if (SecBetween(in.now_ns, in.last_msg_ns) < in.stale_s) return TOPIC_OK;
    return TOPIC_STALE;
}

inline const char *StateName(TopicStateE st)
{
    switch (st) {
    case TOPIC_OK: return "ok";
    case TOPIC_STALE: return "stale";
    case TOPIC_EVENT: return "event";
    case TOPIC_SAMPLING: return "sampling";
    case TOPIC_NO_DATA: return "no_data";
    case TOPIC_NEW: return "new";
    }
    return "unknown";
}

// diagnostic_msgs::DiagnosticStatus 级别:OK=0, WARN=1
inline int LevelOf(TopicStateE st)
{
    if (st == TOPIC_STALE || st == TOPIC_NO_DATA || st == TOPIC_NEW) return 1;
    return 0;
}
