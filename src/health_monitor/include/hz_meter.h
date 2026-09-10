// hz_meter.h —— 话题频率/新鲜度/流量量测,纯逻辑无 ROS 依赖
// 时间戳用 uint64 纳秒由调用方传入;窗口双上限截断,时间与字节成对回退。
#pragma once
#include <deque>
#include <utility>
#include <cstdint>
#include <cstddef>

class HzMeter
{
public:
    HzMeter() : last_msg_ns_(0) {}

    void Tick(uint64_t ns, uint32_t bytes)
    {
        window_.push_back(std::make_pair(ns, bytes));
        last_msg_ns_ = ns;
        Trim();
    }

    // 清空窗口状态;保留 last_msg_ns_(历史新鲜度仍可判,重载荷轮转依赖此语义)
    void Reset()
    {
        window_.clear();
    }

    // 平均频率 = (n-1)/(t_last - t_first);不足 2 帧或跨度 0 返回 0
    double Hz() const
    {
        if (window_.size() < 2) return 0.0;
        double span = SpanS();
        if (span <= 0.0) return 0.0;
        return (double)(window_.size() - 1) / span;
    }

    int WindowMsgs() const { return (int)window_.size(); }

    uint64_t LastMsgNs() const { return last_msg_ns_; }

    // 无消息返回 -1(报告侧原样展示,表示"从未有数据")
    double StaleAgeS(uint64_t now_ns) const
    {
        if (last_msg_ns_ == 0) return -1.0;
        if (now_ns < last_msg_ns_) return 0.0;
        return (double)(now_ns - last_msg_ns_) / 1e9;
    }

    // 窗口字节和 / 窗口跨度;与截断后窗口严格一致(成对回退保证)
    double TrafficBps() const
    {
        if (window_.size() < 2) return 0.0;
        uint64_t bytes = 0;
        for (std::size_t i = 0; i < window_.size(); i++) bytes += window_[i].second;
        double span = SpanS();
        if (span <= 0.0) return 0.0;
        return (double)bytes / span;
    }

    double SpanS() const
    {
        if (window_.size() < 2) return 0.0;
        return (double)(window_.back().first - window_.front().first) / 1e9;
    }

private:
    // 双上限截断:>100 条 或 跨度 >64s 时从头部弹出,至少保留 1 条
    void Trim()
    {
        while (window_.size() > 1) {
            bool over_cnt = window_.size() > 100;
            bool over_span = (window_.back().first - window_.front().first) > (uint64_t)(64.0 * 1e9);
            if (!over_cnt && !over_span) break;
            window_.pop_front();
        }
    }

    std::deque<std::pair<uint64_t, uint32_t> > window_;   // (纳秒时间戳, 字节数)
    uint64_t last_msg_ns_;                                 // 0 = 从未收到消息
};
