// health_monitor.cpp —— 发现/订阅/轮转/评估发布
// 发现口径:ros::master::getTopics 只含已 advertise 的话题(发布者表),
// 本节点自身订阅不会把话题钉死在 master;话题连续 2 个发现周期缺席才移除。
#include "health_monitor.h"
#include <cstdio>
#include <cstdarg>
#include <algorithm>
#include "boost/bind/bind.hpp"

// ---- 内置默认参数(rosparam 读取失败时的回退值,与 README 表一致) ----
static const double kDiscoverPeriodS = 5.0;    // 发现轮询周期 s
static const double kReportPeriodS   = 1.0;    // /diagnostics 发布周期 s
static const double kStaleS          = 5.0;    // 新鲜度阈值 s(超过判 stale)
static const double kNoDataGraceS    = 30.0;   // 发现后 0 帧宽限 s
static const double kSamplePeriodS   = 10.0;   // 重载荷轮转步进 s
static const double kSampleOnS       = 2.0;    // 重载荷采样窗宽 s
static const int    kMaxTopics       = 128;    // 订阅数上限(实车约 50,留余量)
static const uint32_t kSubQueue      = 3;      // 常订队列:1 在 >=spinOnce 频率话题上会丢帧
static const double kSampleTickS     = 0.5;    // 轮转状态机节拍 s(内部固定)

static const char *kDefaultExclude[] = {"/rosout", "/rosout_agg", "/diagnostics", "/clock"};
// 事件/条件驱动话题:合法静默,不判 stale(本图实证,依据 spec 4.5)
static const char *kDefaultEvent[] = {
    "/task_plan_msg",           // 仅云端下发/任务块切换时发布
    "/cloud/task/task_status",  // 与 task_plan 同门事件发布
    "/v2nCommandFeedback",      // 仅指令到达时发布
    "/cloud/task/task_info",    // fms MQTT 脚本话题,云端不下发即静默
    "/cloud/task/remote_signal",
    "/cloud/msg/command_msg",
    "/cam0/status",             // latch+状态翻转才发
    "/cam7/status"};
// 重载荷类型集合:进轮流采样池,不常订(本车=5 雷达点云+2 路 1080p20 JPEG)
static const char *kDefaultHeavy[] = {
    "sensor_msgs/PointCloud2",
    "sensor_msgs/Image",
    "sensor_msgs/CompressedImage"};

static uint64_t SecsToNs(double s) { return (uint64_t)(s * 1e9); }

static void PushKv(std::vector<diagnostic_msgs::KeyValue> &out,
                   const char *key, const char *fmt, ...)
{
    char buf[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    diagnostic_msgs::KeyValue kv;
    kv.key = key;
    kv.value = buf;
    out.push_back(kv);
}

static bool ListHas(const std::vector<std::string> &list, const std::string &s)
{
    return std::find(list.begin(), list.end(), s) != list.end();
}

HealthMonitor::HealthMonitor()
    : discover_period_s_(kDiscoverPeriodS),
      report_period_s_(kReportPeriodS),
      stale_s_(kStaleS),
      no_data_grace_s_(kNoDataGraceS),
      sample_period_s_(kSamplePeriodS),
      sample_on_s_(kSampleOnS),
      max_topics_(kMaxTopics),
      next_rotate_ns_(0),
      window_close_ns_(0)
{
    LoadParams();

    exclude_topics_.assign(kDefaultExclude, kDefaultExclude + 4);
    event_topics_.assign(kDefaultEvent, kDefaultEvent + 8);
    heavy_types_.assign(kDefaultHeavy, kDefaultHeavy + 3);
    ros::param::get("/health_monitor/exclude_topics", exclude_topics_);
    ros::param::get("/health_monitor/event_topics", event_topics_);
    ros::param::get("/health_monitor/heavy_types", heavy_types_);
    // /diagnostics 恒排除(防自环),参数不得放开
    if (!ListHas(exclude_topics_, "/diagnostics"))
        exclude_topics_.push_back("/diagnostics");

    diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);

    discover_timer_ = nh_.createTimer(ros::Duration(discover_period_s_),
                                      &HealthMonitor::DiscoverCallBack, this);
    sample_timer_ = nh_.createTimer(ros::Duration(kSampleTickS),
                                    &HealthMonitor::SampleCallBack, this);
    report_timer_ = nh_.createTimer(ros::Duration(report_period_s_),
                                    &HealthMonitor::ReportCallBack, this);

    printf("[health_monitor] 启动:discover=%.0fs report=%.0fs stale=%.0fs "
           "sample=%.0f/%.0fs max_topics=%d\n",
           discover_period_s_, report_period_s_, stale_s_,
           sample_period_s_, sample_on_s_, max_topics_);
}

void HealthMonitor::LoadParams()
{
    struct PairD { const char *key; double *v; };
    PairD ds[] = {
        {"/health_monitor/discover_period_s", &discover_period_s_},
        {"/health_monitor/report_period_s",   &report_period_s_},
        {"/health_monitor/stale_s",           &stale_s_},
        {"/health_monitor/no_data_grace_s",   &no_data_grace_s_},
        {"/health_monitor/sample_period_s",   &sample_period_s_},
        {"/health_monitor/sample_on_s",       &sample_on_s_},
    };
    for (size_t i = 0; i < sizeof(ds) / sizeof(ds[0]); i++) {
        if (!ros::param::get(ds[i].key, *ds[i].v) && ros::param::has(ds[i].key))
            printf("[health_monitor] 参数 %s 存在但读取失败,沿用内置默认\n", ds[i].key);
    }
    if (!ros::param::get("/health_monitor/max_topics", max_topics_) &&
        ros::param::has("/health_monitor/max_topics"))
        printf("[health_monitor] 参数 max_topics 读取失败,沿用内置默认\n");
    if (sample_on_s_ <= 0.0) {
        printf("[health_monitor] sample_on_s(%.1f) <= 0,钳制为 0.5s(负值会让轮转窗停滞)\n",
               sample_on_s_);
        sample_on_s_ = 0.5;
    }
    if (sample_on_s_ > sample_period_s_) {
        printf("[health_monitor] sample_on_s(%.1f) > sample_period_s(%.1f),钳制为相等\n",
               sample_on_s_, sample_period_s_);
        sample_on_s_ = sample_period_s_;
    }
}

void HealthMonitor::Subscribe(const std::string &name, TopicRec &rec)
{
    rec.sub = nh_.subscribe<topic_tools::ShapeShifter>(
        name, kSubQueue,
        boost::bind(&HealthMonitor::ShapeCallBack, this,
                    boost::placeholders::_1, name),
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
}

void HealthMonitor::ShapeCallBack(const topic_tools::ShapeShifter::ConstPtr &msg,
                                  const std::string &name)
{
    // 单线程回调内做统计,免锁(rospy statistics 同款做法)
    std::map<std::string, TopicRec>::iterator it = topics_.find(name);
    if (it == topics_.end()) return;   // 已移除但订阅未拆净的尾帧
    it->second.meter.Tick(ros::Time::now().toNSec(), msg->size());
}

void HealthMonitor::DiscoverCallBack(const ros::TimerEvent &)
{
    std::vector<ros::master::TopicInfo> infos;
    if (!ros::master::getTopics(infos)) {
        printf("[health_monitor] getTopics 失败(master 未起或重启),本轮跳过\n");
        return;
    }

    std::map<std::string, std::string> live;   // name -> datatype(重名取首个)
    for (size_t i = 0; i < infos.size(); i++) {
        const std::string &name = infos[i].name;
        if (ListHas(exclude_topics_, name)) continue;
        if (live.find(name) == live.end()) live[name] = infos[i].datatype;
    }

    static bool overflow_printed = false;
    for (std::map<std::string, std::string>::iterator it = live.begin();
         it != live.end(); ++it) {
        std::map<std::string, TopicRec>::iterator old = topics_.find(it->first);
        if (old != topics_.end()) {
            old->second.miss_count = 0;
            continue;
        }
        if ((int)topics_.size() >= max_topics_) {
            if (!overflow_printed) {
                printf("[health_monitor] 话题数达上限 %d,跳过新增 %s\n",
                       max_topics_, it->first.c_str());
                overflow_printed = true;
            }
            break;
        }
        TopicRec rec;
        rec.datatype = it->second;
        rec.is_event = ListHas(event_topics_, it->first);
        rec.is_heavy = ListHas(heavy_types_, it->second);
        rec.first_seen_ns = ros::Time::now().toNSec();
        if (!rec.is_heavy)
            Subscribe(it->first, rec);   // 常订;重载荷等轮转,从不首发全量订阅
        topics_[it->first] = rec;
        printf("[health_monitor] 发现话题 %s (%s)%s%s\n", it->first.c_str(),
               rec.datatype.c_str(),
               rec.is_event ? " [event]" : "",
               rec.is_heavy ? " [heavy]" : "");
    }

    // 连续 2 个发现周期缺席才移除(防 master 抖动);窗内被移除则立即关窗
    for (std::map<std::string, TopicRec>::iterator it = topics_.begin();
         it != topics_.end();) {
        if (live.find(it->first) == live.end()) {
            it->second.miss_count++;
            if (it->second.miss_count >= 2) {
                if (it->second.window_open) {
                    it->second.sub.shutdown();
                    it->second.window_open = false;
                    it->second.ever_sampled = true;
                } else {
                    it->second.sub.shutdown();
                }
                printf("[health_monitor] 移除话题 %s(发布者表连续缺席)\n", it->first.c_str());
                it = topics_.erase(it);
                continue;
            }
        }
        ++it;
    }
}

void HealthMonitor::SampleCallBack(const ros::TimerEvent &)
{
    uint64_t now = ros::Time::now().toNSec();

    // 关到期的采样窗(末窗快照保留在 meter,reset 只在开新窗时做)
    if (now >= window_close_ns_ && window_close_ns_ != 0) {
        for (std::map<std::string, TopicRec>::iterator it = topics_.begin();
             it != topics_.end(); ++it) {
            if (!it->second.window_open) continue;
            it->second.sub.shutdown();
            it->second.window_open = false;
            it->second.ever_sampled = true;
        }
        window_close_ns_ = 0;
    }
    if (now < next_rotate_ns_) return;

    // 字典序严格大于 rot_cur_ 的首个重载荷话题;到尾回绕取首个
    std::string pick = "";
    for (std::map<std::string, TopicRec>::iterator it = topics_.begin();
         it != topics_.end(); ++it) {
        if (!it->second.is_heavy) continue;
        if (it->first > rot_cur_) { pick = it->first; break; }
    }
    if (pick.empty())
        for (std::map<std::string, TopicRec>::iterator it = topics_.begin();
             it != topics_.end(); ++it)
            if (it->second.is_heavy) { pick = it->first; break; }

    if (pick.empty()) {   // 池空:空转一个周期再查
        rot_cur_ = "";
        next_rotate_ns_ = now + SecsToNs(sample_period_s_);
        return;
    }

    TopicRec &rec = topics_[pick];
    rec.meter.Reset();          // 开新窗清旧窗(保留 last_msg 历史)
    Subscribe(pick, rec);
    rec.window_open = true;
    rot_cur_ = pick;
    window_close_ns_ = now + SecsToNs(sample_on_s_);
    next_rotate_ns_ = window_close_ns_ + SecsToNs(sample_period_s_ - sample_on_s_);
    printf("[health_monitor] 采样窗 %s 开 %.0fs\n", pick.c_str(), sample_on_s_);
}

void HealthMonitor::ReportCallBack(const ros::TimerEvent &)
{
    diagnostic_msgs::DiagnosticArray arr;
    arr.header.stamp = ros::Time::now();
    uint64_t now = ros::Time::now().toNSec();
    int n_all = 0, n_sub = 0, n_heavy = 0;
    std::string sampling_now = "";

    for (std::map<std::string, TopicRec>::iterator it = topics_.begin();
         it != topics_.end(); ++it) {
        TopicRec &rec = it->second;

        ClassifyInput in;
        in.is_event = rec.is_event;
        in.is_heavy = rec.is_heavy;
        in.window_open = rec.window_open;
        in.ever_sampled = rec.ever_sampled;
        in.now_ns = now;
        in.first_seen_ns = rec.first_seen_ns;
        in.last_msg_ns = rec.meter.LastMsgNs();
        in.window_msgs = rec.meter.WindowMsgs();
        in.stale_s = stale_s_;
        in.no_data_grace_s = no_data_grace_s_;
        in.discover_period_s = discover_period_s_;
        TopicStateE st = ClassifyTopicState(in);

        diagnostic_msgs::DiagnosticStatus s;
        s.name = it->first;
        s.hardware_id = rec.datatype;
        s.level = (uint8_t)LevelOf(st);
        char buf[160];
        snprintf(buf, sizeof(buf), "state=%s hz=%.2f stale_age=%.1fs",
                 StateName(st), rec.meter.Hz(), rec.meter.StaleAgeS(now));
        s.message = buf;
        PushKv(s.values, "hz", "%.3f", rec.meter.Hz());
        PushKv(s.values, "window_msgs", "%d", rec.meter.WindowMsgs());
        PushKv(s.values, "stale_age_s", "%.2f", rec.meter.StaleAgeS(now));
        PushKv(s.values, "traffic_bps", "%.1f", rec.meter.TrafficBps());
        PushKv(s.values, "datatype", "%s", rec.datatype.c_str());
        PushKv(s.values, "state", "%s", StateName(st));
        arr.status.push_back(s);

        n_all++;
        if (rec.is_heavy) n_heavy++; else n_sub++;
        if (rec.window_open) sampling_now = it->first;
    }

    diagnostic_msgs::DiagnosticStatus sum;
    sum.name = "/health_monitor/summary";
    sum.hardware_id = "health_monitor";
    sum.level = diagnostic_msgs::DiagnosticStatus::OK;
    char buf[128];
    snprintf(buf, sizeof(buf), "topics=%d subscribed=%d heavy=%d sampling=%s",
             n_all, n_sub, n_heavy,
             sampling_now.empty() ? "-" : sampling_now.c_str());
    sum.message = buf;
    arr.status.push_back(sum);

    diag_pub_.publish(arr);
}

void HealthMonitor::Run()
{
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}
