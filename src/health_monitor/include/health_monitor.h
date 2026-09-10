#pragma once
// HealthMonitor:话题健康监测
// 发现(ros::master::getTopics 发布者表口径) -> 订阅分类(ShapeShifter) ->
// 重载荷轮流采样 -> 1Hz 评估发布 /diagnostics。单线程 Timer,回调免锁。
#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include "ros/ros.h"
#include "ros/master.h"
#include "topic_tools/shape_shifter.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "hz_meter.h"
#include "state_classify.h"

class HealthMonitor
{
public:
    HealthMonitor();
    void Run();   // 100Hz spinOnce 主循环(canbus main 风格)

private:
    struct TopicRec {
        std::string datatype;
        bool is_event;       // 事件驱动话题:不判 stale
        bool is_heavy;       // 重载荷:轮流采样,不常订
        bool ever_sampled;   // 已完成过至少一个采样窗
        bool window_open;    // 当前处于采样窗内
        int miss_count;      // 连续缺席发现周期数(>=2 移除)
        uint64_t first_seen_ns;
        HzMeter meter;
        ros::Subscriber sub;

        TopicRec()
            : is_event(false), is_heavy(false), ever_sampled(false),
              window_open(false), miss_count(0), first_seen_ns(0) {}
    };

    void LoadParams();
    void Subscribe(const std::string &name, struct TopicRec &rec);
    void DiscoverCallBack(const ros::TimerEvent &event);
    void SampleCallBack(const ros::TimerEvent &event);
    void ReportCallBack(const ros::TimerEvent &event);
    void ShapeCallBack(const topic_tools::ShapeShifter::ConstPtr &msg,
                       const std::string &name);

    ros::NodeHandle nh_;
    ros::Publisher diag_pub_;
    ros::Timer discover_timer_;
    ros::Timer sample_timer_;
    ros::Timer report_timer_;

    double discover_period_s_;
    double report_period_s_;
    double stale_s_;
    double no_data_grace_s_;
    double sample_period_s_;
    double sample_on_s_;
    int max_topics_;
    std::vector<std::string> exclude_topics_;
    std::vector<std::string> event_topics_;
    std::vector<std::string> heavy_types_;

    std::map<std::string, TopicRec> topics_;   // map 字典序 = 轮转序
    uint64_t next_rotate_ns_;
    uint64_t window_close_ns_;
    std::string rot_cur_;   // 上一轮采样的话题名(下轮取字典序严格大于者)
};
