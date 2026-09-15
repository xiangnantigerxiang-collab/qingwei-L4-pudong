// PNC 消费侧示例：实际编译使用仓内 XYZ_COOR_S 定义。
#include <cstdint>
#include "common/struct_type.h"
#include "hdmap/hdmap_server.h"
#include "hdmap/pnc_adapter.h"
#include <cstdio>

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::fprintf(stderr, "用法: %s <SDK 生成的 CSV 文件>\n", argv[0]);
        return 2;
    }
    hdmap::HdMapServer server;
    hdmap::MapPointList points;
    hdmap::DIRECTION_E direction = hdmap::DIRECTION_AUTO;
    hdmap::STATUS_S status = server.LoadProcessedTrajectory(argv[1], points, direction);
    if (!status.IsOk())
    {
        std::fprintf(stderr, "%s\n", status.message.c_str());
        return 1;
    }
    std::vector<XYZ_COOR_S> path_list;
    status = hdmap::ConvertToPncPath(points, path_list);
    if (!status.IsOk())
    {
        std::fprintf(stderr, "%s\n", status.message.c_str());
        return 1;
    }
    // 生产接入点：将 path_list 交给 PNC 路径管理层；挡位仍由任务配置决定。
    std::printf("PNC 路径 %zu 点，方向 %s，首点 heading %.6f，末点里程 %.6f 米\n",
                path_list.size(), direction == hdmap::DIRECTION_REVERSE ? "R" : "D",
                path_list.front().heading, path_list.back().dist_origin);
    return 0;
}
