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
