# health_monitor 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新建 catkin 包 `src/health_monitor`：自动发现全图话题，量测频率/新鲜度/流量/类型，1Hz 发布 `/diagnostics`。

**Architecture:** 单线程 roscpp 节点（canbus main 风格：100Hz spinOnce + Timer）。发现用 `ros::master::getTopics()`（发布者表+datatype）；小话题 ShapeShifter 常订（queue=3）；重载荷（点云/图像/压缩图像）按字典序轮流开 2s 采样窗；纯逻辑（HzMeter、状态分类器）独立头文件，g++ 单测不依赖 ROS。

**Tech Stack:** C++11 / roscpp / topic_tools(ShapeShifter) / diagnostic_msgs。

**Spec:** `docs/superpowers/specs/2026-09-08-health-monitor-design.md`（先读 spec 再动手；本计划已吸收其对抗校验全部 9 项修正）。

## Global Constraints

- 工程根：`/home/mothotob/work/projects/claude-code/qingwei-L4-No2-pudong`（下称 `$ROOT`，非 git 工程——**无 commit 步骤**，每任务以测试/桩编译通过为完成门槛）
- 风格：`src/canbus/VIBE_CODING_GUIDE.md`——C++11、4 空格、类/函数左括号换行、if/for 同行、方法 PascalCase、回调 `XxxCallBack`、话题绝对名、成员显式初始化、注释中文业务+英文短标签
- **不改任何现有业务文件**（例外：spec §3 文件清单小补、workflow.md 日志条目）
- 本机 Ubuntu 24.04 + ROS2 jazzy（**无 ROS1**）：ROS 相关代码只做桩编译；纯逻辑用 g++ 直接测
- 本机 boost 较新：桩编译用 `-std=c++14`（车载 noetic 默认即可，无碍）

---

### Task 1: HzMeter 纯逻辑（TDD）

**Files:**
- Create: `src/health_monitor/include/hz_meter.h`
- Test: `src/health_monitor/test/hz_meter_test.cpp`

**Interfaces:**
- Consumes: 无
- Produces: `class HzMeter`——`void Tick(uint64_t ns, uint32_t bytes)`、`void Reset()`（清窗口、**保留** last_msg）、`double Hz() const`、`int WindowMsgs() const`、`uint64_t LastMsgNs() const`、`double StaleAgeS(uint64_t now_ns) const`（无消息返回 -1）、`double TrafficBps() const`、`double SpanS() const`

- [ ] **Step 1: 写失败测试**

创建 `src/health_monitor/test/hz_meter_test.cpp`：

```cpp
// hz_meter_test.cpp —— HzMeter 纯数学单测,不依赖 ROS,g++ 直接编跑
#include "hz_meter.h"
#include <cstdio>

static int g_fail = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); g_fail++; } } while (0)

static uint64_t NS(double s) { return (uint64_t)(s * 1e9); }   // 秒 -> 纳秒

int main()
{
    {   // 已知间隔 10Hz -> hz 与 stale_age
        HzMeter m;
        for (int i = 0; i < 11; i++) m.Tick(NS(i * 0.1), 10);
        CHECK(m.Hz() > 9.5 && m.Hz() < 10.5);
        CHECK(m.WindowMsgs() == 11);
        CHECK(m.StaleAgeS(NS(10.15)) > 9.0 && m.StaleAgeS(NS(10.15)) < 9.2);
        CHECK(m.SpanS() > 0.95 && m.SpanS() < 1.05);
    }
    {   // 不足 2 帧
        HzMeter m;
        m.Tick(NS(1.0), 5);
        CHECK(m.Hz() == 0.0);
        CHECK(m.StaleAgeS(NS(1.0)) == 0.0);
        CHECK(m.TrafficBps() == 0.0);
    }
    {   // 从未收到消息
        HzMeter m;
        CHECK(m.StaleAgeS(NS(5.0)) < 0.0);
        CHECK(m.LastMsgNs() == 0);
    }
    {   // >100 条截断,hz 仍正确
        HzMeter m;
        for (int i = 0; i < 300; i++) m.Tick(NS(i * 0.01), 1);   // 100Hz
        CHECK(m.WindowMsgs() <= 100);
        CHECK(m.Hz() > 90.0 && m.Hz() < 110.0);
    }
    {   // 跨度 >64s 截断
        HzMeter m;
        m.Tick(NS(0.0), 1);
        for (int i = 0; i < 40; i++) m.Tick(NS(10.0 + i * 2.0), 1);   // 总跨度 86s
        CHECK(m.SpanS() <= 64.0 + 2.0);
        CHECK(m.WindowMsgs() >= 2);
    }
    {   // traffic 截断成对回退:字节必须随时间戳一起被弹出
        HzMeter m;
        for (int i = 0; i < 150; i++) m.Tick(NS(i * 0.01), 1000);   // 1.5s 窗口截到 ~100 条
        CHECK(m.WindowMsgs() <= 100);
        // 100 条×1000B/约 1s 跨度 ≈ 100KB/s;若字节不回退会测出 ~150KB/s
        CHECK(m.TrafficBps() > 80.0 * 1000 && m.TrafficBps() < 110.0 * 1000);
    }
    {   // reset 清窗口但保留 last_msg(历史新鲜度仍可判)
        HzMeter m;
        for (int i = 0; i < 10; i++) m.Tick(NS(i * 0.5), 100);
        m.Reset();
        CHECK(m.WindowMsgs() == 0);
        CHECK(m.Hz() == 0.0);
        CHECK(m.TrafficBps() == 0.0);
        CHECK(m.LastMsgNs() == NS(4.5));
        CHECK(m.StaleAgeS(NS(6.0)) > 1.4 && m.StaleAgeS(NS(6.0)) < 1.6);
    }

    if (g_fail == 0) printf("hz_meter_test: ALL PASS\n");
    else printf("hz_meter_test: %d FAILED\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
```

- [ ] **Step 2: 跑测试确认失败（头文件不存在）**

```bash
cd /home/mothotob/work/projects/claude-code/qingwei-L4-No2-pudong
g++ -std=c++11 -I src/health_monitor/include src/health_monitor/test/hz_meter_test.cpp -o /tmp/hm_hz
```
Expected: 编译错误 `hz_meter.h: No such file or directory`

- [ ] **Step 3: 实现 hz_meter.h**

```cpp
// hz_meter.h —— 话题频率/新鲜度/流量量测,纯逻辑无 ROS 依赖
// 时间戳用 uint64 纳秒由调用方传入;窗口双上限截断,时间与字节成对回退。
#pragma once
#include <deque>
#include <utility>
#include <cstdint>

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
        for (size_t i = 0; i < window_.size(); i++) bytes += window_[i].second;
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
```

- [ ] **Step 4: 跑测试确认通过**

```bash
g++ -std=c++11 -I src/health_monitor/include src/health_monitor/test/hz_meter_test.cpp -o /tmp/hm_hz && /tmp/hm_hz
```
Expected: `hz_meter_test: ALL PASS`（退出码 0）

---

### Task 2: 状态分类器（TDD）

**Files:**
- Create: `src/health_monitor/include/state_classify.h`
- Test: `src/health_monitor/test/state_classify_test.cpp`
- Modify: `docs/superpowers/specs/2026-09-08-health-monitor-design.md` §3 文件清单（补上两文件）

**Interfaces:**
- Consumes: 无
- Produces: `enum TopicStateE {TOPIC_OK,TOPIC_STALE,TOPIC_EVENT,TOPIC_SAMPLING,TOPIC_NO_DATA,TOPIC_NEW}`、`struct ClassifyInput`（字段见下）、`TopicStateE ClassifyTopicState(const ClassifyInput &in)`、`const char *StateName(TopicStateE)`、`int LevelOf(TopicStateE)`（0=OK,1=WARN）

- [ ] **Step 1: 写失败测试**

创建 `src/health_monitor/test/state_classify_test.cpp`：

```cpp
// state_classify_test.cpp —— 话题状态分类纯函数单测,不依赖 ROS
#include "state_classify.h"
#include <cstdio>

static int g_fail = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); g_fail++; } } while (0)

static uint64_t NS(double s) { return (uint64_t)(s * 1e9); }

static ClassifyInput Base()
{
    ClassifyInput in;
    in.is_event = false;
    in.is_heavy = false;
    in.window_open = false;
    in.ever_sampled = false;
    in.now_ns = NS(1000.0);
    in.first_seen_ns = NS(0.0);
    in.last_msg_ns = NS(999.0);      // 1s 前有消息
    in.window_msgs = 10;
    in.stale_s = 5.0;
    in.no_data_grace_s = 30.0;
    in.discover_period_s = 5.0;
    return in;
}

int main()
{
    {   // light 常订新鲜 -> OK;超时 -> STALE
        ClassifyInput in = Base();
        CHECK(ClassifyTopicState(in) == TOPIC_OK);
        in.last_msg_ns = NS(980.0);   // 20s 前
        CHECK(ClassifyTopicState(in) == TOPIC_STALE);
    }
    {   // event 优先于 stale:有消息且超时 -> EVENT 非 STALE
        ClassifyInput in = Base();
        in.is_event = true;
        in.last_msg_ns = NS(0.0);     // 1000s 前
        CHECK(ClassifyTopicState(in) == TOPIC_EVENT);
        CHECK(LevelOf(ClassifyTopicState(in)) == 0);
    }
    {   // event 无消息:合法静默 -> EVENT;刚发现 -> NEW
        ClassifyInput in = Base();
        in.is_event = true;
        in.last_msg_ns = 0;
        in.first_seen_ns = NS(998.0);   // 发现 2s(< 2 个发现周期)
        CHECK(ClassifyTopicState(in) == TOPIC_NEW);
        in.first_seen_ns = NS(0.0);     // 发现 1000s
        CHECK(ClassifyTopicState(in) == TOPIC_EVENT);
    }
    {   // 非 event 无消息:宽限内 NEW,超过 -> NO_DATA
        ClassifyInput in = Base();
        in.last_msg_ns = 0;
        in.first_seen_ns = NS(990.0);   // 10s < 30s 宽限
        CHECK(ClassifyTopicState(in) == TOPIC_NEW);
        in.first_seen_ns = NS(0.0);     // 1000s > 30s
        CHECK(ClassifyTopicState(in) == TOPIC_NO_DATA);
        CHECK(LevelOf(ClassifyTopicState(in)) == 1);
    }
    {   // heavy:未首采 -> SAMPLING;末窗 0 帧(曾有历史消息) -> NO_DATA;有快照 -> SAMPLING
        ClassifyInput in = Base();
        in.is_heavy = true;
        in.ever_sampled = false;
        in.window_msgs = 0;
        CHECK(ClassifyTopicState(in) == TOPIC_SAMPLING);

        in.ever_sampled = true;
        in.window_msgs = 0;             // 末窗 0 帧,last_msg 来自更早窗口
        CHECK(ClassifyTopicState(in) == TOPIC_NO_DATA);

        in.window_msgs = 20;
        CHECK(ClassifyTopicState(in) == TOPIC_SAMPLING);
    }
    {   // heavy 采样窗内:按新鲜度正常判 OK/STALE
        ClassifyInput in = Base();
        in.is_heavy = true;
        in.window_open = true;
        in.ever_sampled = true;
        CHECK(ClassifyTopicState(in) == TOPIC_OK);
        in.last_msg_ns = NS(900.0);     // 100s 前(上一窗遗留)
        CHECK(ClassifyTopicState(in) == TOPIC_STALE);
    }
    {   // StateName / LevelOf 映射
        CHECK(std::string(StateName(TOPIC_OK)) == "ok");
        CHECK(std::string(StateName(TOPIC_NO_DATA)) == "no_data");
        CHECK(LevelOf(TOPIC_STALE) == 1);
        CHECK(LevelOf(TOPIC_NEW) == 1);
        CHECK(LevelOf(TOPIC_EVENT) == 0);
        CHECK(LevelOf(TOPIC_SAMPLING) == 0);
    }

    if (g_fail == 0) printf("state_classify_test: ALL PASS\n");
    else printf("state_classify_test: %d FAILED\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
```

（`#include <string>` 需补在文件头。）

- [ ] **Step 2: 跑测试确认失败**

```bash
g++ -std=c++11 -I src/health_monitor/include src/health_monitor/test/state_classify_test.cpp -o /tmp/hm_sc
```
Expected: `state_classify.h: No such file or directory`

- [ ] **Step 3: 实现 state_classify.h**

```cpp
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
```

- [ ] **Step 4: 跑测试确认通过**

```bash
g++ -std=c++11 -I src/health_monitor/include src/health_monitor/test/state_classify_test.cpp -o /tmp/hm_sc && /tmp/hm_sc
```
Expected: `state_classify_test: ALL PASS`

- [ ] **Step 5: 同步 spec §3 文件清单**

在 `docs/superpowers/specs/2026-09-08-health-monitor-design.md` §3 的包结构中，
`include/hz_meter.h` 行后加一行 `├── include/state_classify.h    # 状态分类纯函数（可单测）`，
`test/hz_meter_test.cpp` 行后加 `└── test/state_classify_test.cpp`（原 `└──` 改 `├──`）。

---

### Task 3: ROS 桩环境（本机编译验证设施）

**Files:**
- Create: `/tmp/hm_stub/ros/ros.h`、`/tmp/hm_stub/ros/master.h`、`/tmp/hm_stub/topic_tools/shape_shifter.h`、`/tmp/hm_stub/diagnostic_msgs/DiagnosticArray.h`（重启即失，属验证设施非交付物）

**Interfaces:**
- Produces: 供 Task 4/5 桩编译的假 ROS 头（签名接受真实调用形式）

- [ ] **Step 1: 写桩头文件**

`/tmp/hm_stub/ros/ros.h`：

```cpp
#pragma once
// 本机桩:仅用于编译验证 health_monitor,非交付物(本机无 ROS1)
#include <cstdint>
#include <string>
#include <vector>
#include <memory>

namespace ros {

class Duration { public: double t; Duration() : t(0) {} explicit Duration(double s) : t(s) {} };
class Time {
public:
    uint64_t ns;
    Time() : ns(0) {}
    double toSec() const { return (double)ns / 1e9; }
    uint64_t toNSec() const { return ns; }
    static Time now() { return Time(); }
};
class Rate { public: explicit Rate(double) {} void sleep() {} };
struct TransportHints { TransportHints tcpNoDelay() const { return TransportHints(); } };
struct TimerEvent {};
class Timer { public: Timer() {} };
class Subscriber { public: Subscriber() {} void shutdown() {} };
class Publisher {
public:
    Publisher() {}
    template <class M> void publish(const M &) const {}
};
typedef std::shared_ptr<const void> VoidConstPtr;

inline void init(int &, char **, const std::string &) {}
inline bool ok() { return true; }
inline void spinOnce() {}

class NodeHandle {
public:
    template <class M, class... A>
    Subscriber subscribe(A &&...) { return Subscriber(); }
    template <class M>
    Publisher advertise(const std::string &, uint32_t) { return Publisher(); }
    template <class... A>
    Timer createTimer(Duration, A &&...) { return Timer(); }
};

namespace param {
template <class T> bool get(const std::string &, T &) { return false; }
inline bool has(const std::string &) { return false; }
} // namespace param

} // namespace ros
```

`/tmp/hm_stub/ros/master.h`：

```cpp
#pragma once
#include <string>
#include <vector>

namespace ros {
namespace master {

struct TopicInfo {
    std::string name;
    std::string datatype;
};

inline bool getTopics(std::vector<TopicInfo> &) { return false; }

} // namespace master
} // namespace ros
```

`/tmp/hm_stub/topic_tools/shape_shifter.h`：

```cpp
#pragma once
#include <string>
#include <cstdint>
#include <memory>

namespace topic_tools {

class ShapeShifter {
public:
    typedef std::shared_ptr<const ShapeShifter> ConstPtr;
    std::string getDataType() const { return ""; }
    uint32_t size() const { return 0; }
};

} // namespace topic_tools
```

`/tmp/hm_stub/diagnostic_msgs/DiagnosticArray.h`：

```cpp
#pragma once
#include <string>
#include <vector>
#include "ros/ros.h"

namespace std_msgs {
struct Header { ros::Time stamp; };
}

namespace diagnostic_msgs {

struct KeyValue { std::string key; std::string value; };

struct DiagnosticStatus {
    uint8_t level;
    std::string name;
    std::string hardware_id;
    std::string message;
    std::vector<KeyValue> values;
    static const uint8_t OK = 0;
    static const uint8_t WARN = 1;
    static const uint8_t ERROR = 2;
};

struct DiagnosticArray {
    std_msgs::Header header;
    std::vector<DiagnosticStatus> status;
};

} // namespace diagnostic_msgs
```

- [ ] **Step 2: 探针 TU 验证桩可用**

```bash
cat > /tmp/hm_stub/probe.cpp <<'EOF'
#include "ros/ros.h"
#include "ros/master.h"
#include "topic_tools/shape_shifter.h"
#include "diagnostic_msgs/DiagnosticArray.h"
int main() {
    ros::NodeHandle nh;
    diagnostic_msgs::DiagnosticArray a;
    a.header.stamp = ros::Time::now();
    (void)nh; (void)a;
    std::vector<ros::master::TopicInfo> v;
    ros::master::getTopics(v);
    return 0;
}
EOF
g++ -std=c++14 -I /tmp/hm_stub /tmp/hm_stub/probe.cpp -o /tmp/hm_stub/probe && echo STUB_OK
```
Expected: `STUB_OK`

---

### Task 4: 包脚手架 + 最小节点（桩编译通）

**Files:**
- Create: `src/health_monitor/CMakeLists.txt`、`src/health_monitor/package.xml`、`src/health_monitor/launch/health_monitor.launch`、`src/health_monitor/include/health_monitor.h`（骨架）、`src/health_monitor/src/health_monitor.cpp`（骨架）、`src/health_monitor/src/health_monitor_node.cpp`

**Interfaces:**
- Consumes: Task 1/2 的 hz_meter.h、state_classify.h；Task 3 桩
- Produces: `class HealthMonitor`（本任务只含构造/Run 骨架，Task 5 填逻辑）

- [ ] **Step 1: package.xml**

```xml
<?xml version="1.0"?>
<package format="2">
  <name>health_monitor</name>
  <version>0.1.0</version>
  <description>话题健康监测:自动发现全图话题,量测频率/新鲜度/流量,1Hz 发布 /diagnostics</description>
  <maintainer email="nvidia@todo.todo">nvidia</maintainer>
  <license>BSD</license>

  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>topic_tools</depend>
  <depend>diagnostic_msgs</depend>
</package>
```

- [ ] **Step 2: CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.0)
project(health_monitor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  topic_tools
  diagnostic_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(health_monitor_node
  src/health_monitor_node.cpp
  src/health_monitor.cpp
)
target_link_libraries(health_monitor_node ${catkin_LIBRARIES})

# 纯逻辑回归(不依赖 ROS 运行时,车载可直接跑)
add_executable(hz_meter_test test/hz_meter_test.cpp)
add_executable(state_classify_test test/state_classify_test.cpp)
```

- [ ] **Step 3: launch/health_monitor.launch**

```xml
<launch>
  <!-- 参数默认值已内置在代码中;需要覆盖时取消注释改值
  <param name="/health_monitor/stale_s" value="5.0"/>
  <param name="/health_monitor/sample_period_s" value="10.0"/>
  -->
  <node pkg="health_monitor" type="health_monitor_node" name="health_monitor_node" output="screen"/>
</launch>
```

- [ ] **Step 4: health_monitor.h 骨架（完整版在 Task 5，本步先建结构）**

`src/health_monitor/include/health_monitor.h`：

```cpp
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
    void LoadParams();
    void Subscribe(const std::string &name, struct TopicRec &rec);
    void DiscoverCallBack(const ros::TimerEvent &event);
    void SampleCallBack(const ros::TimerEvent &event);
    void ReportCallBack(const ros::TimerEvent &event);
    void ShapeCallBack(const topic_tools::ShapeShifter::ConstPtr &msg,
                       const std::string &name);

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
```

- [ ] **Step 5: health_monitor.cpp 骨架 + node 入口**

`src/health_monitor/src/health_monitor.cpp`：

```cpp
// health_monitor.cpp —— 发现/订阅/轮转/评估发布(逻辑实现)
#include "health_monitor.h"
#include <cstdio>

HealthMonitor::HealthMonitor()
    : discover_period_s_(5.0),
      report_period_s_(1.0),
      stale_s_(5.0),
      no_data_grace_s_(30.0),
      sample_period_s_(10.0),
      sample_on_s_(2.0),
      max_topics_(128),
      next_rotate_ns_(0),
      window_close_ns_(0)
{
    // Task 5 填充:LoadParams/列表默认值/发布器/三个 Timer
}

void HealthMonitor::LoadParams() {}
void HealthMonitor::Subscribe(const std::string &, TopicRec &) {}
void HealthMonitor::DiscoverCallBack(const ros::TimerEvent &) {}
void HealthMonitor::SampleCallBack(const ros::TimerEvent &) {}
void HealthMonitor::ReportCallBack(const ros::TimerEvent &) {}
void HealthMonitor::ShapeCallBack(const topic_tools::ShapeShifter::ConstPtr &,
                                  const std::string &) {}

void HealthMonitor::Run()
{
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}
```

`src/health_monitor/src/health_monitor_node.cpp`：

```cpp
// health_monitor_node.cpp —— ROS 接线:初始化与主循环
#include "ros/ros.h"
#include "health_monitor.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "health_monitor_node");
    HealthMonitor monitor;
    monitor.Run();
    return 0;
}
```

注意：`Run()` 里 `ros::Rate rate(100);` 用的是实参 100——骨架里 `HealthMonitor()` 初始化列表后的成员 `rot_cur_` 由 std::string 默认构造，无需列出。

- [ ] **Step 6: 桩编译验证**

```bash
cd /home/mothotob/work/projects/claude-code/qingwei-L4-No2-pudong
g++ -std=c++14 -include cstdint \
    -I /tmp/hm_stub -I src/health_monitor/include \
    src/health_monitor/src/health_monitor_node.cpp src/health_monitor/src/health_monitor.cpp \
    -o /tmp/hm_stub/health_monitor_node && echo SCAFFOLD_OK
```
Expected: `SCAFFOLD_OK`

---

### Task 5: HealthMonitor 完整逻辑

**Files:**
- Modify: `src/health_monitor/src/health_monitor.cpp`（整文件替换为下述实现）

**Interfaces:**
- Consumes: Task 1-4 全部产物
- Produces: 可上车的完整节点

- [ ] **Step 1: 写完整实现**

`src/health_monitor/src/health_monitor.cpp` 整文件替换为：

```cpp
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
```

- [ ] **Step 2: 桩编译验证（编译+链接）**

```bash
cd /home/mothotob/work/projects/claude-code/qingwei-L4-No2-pudong
g++ -std=c++14 -include cstdint \
    -I /tmp/hm_stub -I src/health_monitor/include \
    src/health_monitor/src/health_monitor_node.cpp src/health_monitor/src/health_monitor.cpp \
    -o /tmp/hm_stub/health_monitor_node && echo IMPL_OK
```
Expected: `IMPL_OK`（0 error 0 warning 为目标；boost bind 若有弃用告警可接受）

- [ ] **Step 3: 回归纯逻辑测试**

```bash
/tmp/hm_hz && /tmp/hm_sc
```
Expected: 两行 `ALL PASS`

---

### Task 6: README、spec/工作记录同步、收尾核对

**Files:**
- Create: `src/health_monitor/README.md`
- Modify: `workflow.md`（日志条目）

- [ ] **Step 1: 写 README**

```markdown
# health_monitor —— 话题健康监测节点

自动发现 ROS 图中全部话题（ros::master::getTopics 发布者表口径），量测每个话题的
频率/新鲜度/流量/消息类型，1Hz 发布 `diagnostic_msgs/DiagnosticArray` 到
`/diagnostics`。旁路观测节点，不接任何控制链；健康判定留给消费方。

## 机制

- 小/中话题：topic_tools::ShapeShifter 常订（queue=3，跳过反序列化，回调只记
  时间戳与字节数）
- 重载荷（sensor_msgs/{PointCloud2,Image,CompressedImage}，本车=5 雷达+2 相机）：
  按字典序轮流开 2s 采样窗，从不常订
- 状态分类（详见代码 state_classify.h）：ok / stale / event / sampling / no_data / new；
  event 话题（事件驱动，合法静默）不判 stale
- 单线程 Timer，回调免锁

## 参数（绝对键，均有内置默认值）

| 键 | 默认 | 说明 |
|---|---|---|
| /health_monitor/discover_period_s | 5.0 | 发现轮询周期 |
| /health_monitor/report_period_s | 1.0 | /diagnostics 发布周期 |
| /health_monitor/stale_s | 5.0 | 新鲜度阈值（稳态话题） |
| /health_monitor/no_data_grace_s | 30.0 | 发现后 0 帧宽限 |
| /health_monitor/sample_period_s | 10.0 | 重载荷轮转步进 |
| /health_monitor/sample_on_s | 2.0 | 采样窗宽 |
| /health_monitor/exclude_topics | [/rosout,/rosout_agg,/diagnostics,/clock] | 排除表 |
| /health_monitor/heavy_types | [PointCloud2,Image,CompressedImage] | 重载荷类型 |
| /health_monitor/event_topics | 见代码（8 个本图实证话题） | 事件驱动话题 |
| /health_monitor/max_topics | 128 | 订阅数上限 |

## 启动

roslaunch health_monitor health_monitor.launch

## 本机回归（无 ROS 依赖）

catkin_make --pkg health_monitor 后直接运行（或 g++ 编译 test/*.cpp）：
- hz_meter_test / state_classify_test：ALL PASS

## 实车验证清单

- rostopic echo /diagnostics 与 rostopic hz <话题> 抽查对照（≥60s 长窗，±10%）
- kill <pid>（SIGTERM 干净注销）→ 条目 2 个发现周期内从报告移除；
  kill -9（注册残留）→ state=no_data/stale
- 事件话题（如 /task_plan_msg）待命时 state=event 且 level=OK
- 重载荷轮转日志（5 雷达+2 相机约 70s 一圈）与 sampling 态
- top 对比起停前后 CPU 差（预期 <3% 单核）
- 话题数对照：与发布者表口径一致（rostopic list 为并集，多出纯订阅话题属预期）
```

- [ ] **Step 2: workflow.md 日志条目**

在「九、日志」顶部追加（结论级，一整条）：

```markdown
- **09-08 晚 health_monitor 新包（话题健康监测,C++/单线程/零侵入）**：需求=监控全部话题+点云占空比采样+1Hz 出 /diagnostics,消费方用户后续自建。**前置 spike 推翻 /statistics 路线**（roscpp 未实现该特性,rospy 为订阅端语义,ros_comm noetic 源码验证）→ ShapeShifter 主动订阅方案。设计 spec 经 ultracode 17 代理五视角对抗校验（27 发现→9 确认全吸收）,关键修正：发现用 ros::master::getTopics 发布者表口径（getSystemState 并集会被自身订阅钉死+roscpp 无此封装）;重载荷池=PointCloud2+Image+CompressedImage（否则 2 路 1080p20 JPEG 被常订,预算破产）;event 话题 8 个实证入默认表不判 stale（/task_plan_msg 等,待命不误报）;HzMeter (ns,bytes) 成对存储防 traffic 单调发散;常订 queue=3 防 100Hz 丢帧;kill/kill -9 两种死亡路径分测。交付：src/health_monitor 新包（node/逻辑分层,纯逻辑 hz_meter+state_classify 可独立单测）;本机验证=桩编译(/tmp/hm_stub)+纯逻辑单测全绿,**实车清单见包 README 未执行**。车载部署：同步 src/health_monitor 整目录,rebuild_all 全量段自动编入,roslaunch 即起
```

- [ ] **Step 3: 收尾核对（对照 Vibe guide 13 节）**

```bash
cd /home/mothotob/work/projects/claude-code/qingwei-L4-No2-pudong
# 1.话题绝对名(应只有 / 开头字符串)
grep -n '"' src/health_monitor/src/health_monitor.cpp | grep -E 'topic|rosout|diagnostics' 
# 2.无未初始化成员:核对 health_monitor.h 构造初始化列表覆盖全部标量成员
# 3.包文件清单
find src/health_monitor -type f | sort
# 4.最终回归
/tmp/hm_hz && /tmp/hm_sc && ls -la /tmp/hm_stub/health_monitor_node
```
Expected: 单测双绿、可执行存在、9 个文件（含 README）

---

## Self-Review 记录

- **Spec 覆盖**：§4.1→Task5 DiscoverCallBack；§4.2→Subscribe/ShapeCallBack；§4.3→SampleCallBack；§4.4→Task1；§4.5→Task2+ReportCallBack；§4.6→Run；§5→LoadParams+常量；§7→各文件；§8→桩编译+单测+README 实车清单；§9→Task6 日志。无缺口
- **占位符扫描**：无 TBD/TODO；骨架文件在 Task4 明示"Task5 填充"属任务边界非占位
- **类型一致性**：`TopicRec` 前置声明（`struct TopicRec &`）与定义一致；`ClassifyInput` 字段与 Task5 使用处逐一对齐；HzMeter 方法名与测试一致（含 `SpanS`）
