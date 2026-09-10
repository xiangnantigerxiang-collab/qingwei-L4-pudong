// state_classify_test.cpp —— 话题状态分类纯函数单测,不依赖 ROS
#include "state_classify.h"
#include <cstdio>
#include <string>

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
        in.last_msg_ns = NS(980.0);   // 20s 前(超过 stale_s=5s)
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
