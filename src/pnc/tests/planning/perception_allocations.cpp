#include <cstdlib>
#include <cstdio>
#include <new>
#include <vector>
#include "../../src/robot_path_plan/safety/perception_safety.inc"

namespace {
    bool watching = false;
    std::size_t allocations = 0;
}

void* operator new(std::size_t size) {
    if(watching) ++allocations;
    void* memory = std::malloc(size ? size : 1);
    if(!memory) throw std::bad_alloc();
    return memory;
}
void* operator new[](std::size_t size) {
    return ::operator new(size);
}
void operator delete(void* memory) noexcept {
    std::free(memory);
}
void operator delete[](void* memory) noexcept {
    ::operator delete(memory);
}

int main() {
    planning_perception::PerceptionSafety safety;
    planning_perception::EGO_S ego;
    ego.heading = 90;
    ego.speed = 1;
    std::vector<planning_perception::PATH_POINT_S> path;
    for(int i = 0; i < 50; ++i) path.emplace_back(i * 0.5, 0);
    robot::perception input;
    for(int i = 0; i < 300; ++i) {
        robot::object object;
        object.id = i + 1;
        object.dx = object.dy = 1;
        object.x = 12 + i % 20;
        object.y = (i / 20 - 7) * 3;
        object.polygons.resize(4);
        const double dx[] = {-0.5, 0.5, 0.5, -0.5}, dy[] = {-0.5, -0.5, 0.5, 0.5};
        for(int j = 0; j < 4; ++j) {
            object.polygons[j].x = object.x + dx[j];
            object.polygons[j].y = object.y + dy[j];
        }
        input.objs.push_back(object);
    }
    const planning_perception::LaneMotionFilter lane_filter(ego.heading);
    for(int tick = 0; tick < 1005; ++tick) {
        const double now = 100 + tick * 0.1;
        input.header.stamp = ros::Time(now);
        if(tick == 5) watching = true;
        // 持续轮换静止/同向/对向，覆盖剔除旧轨迹与同 ID 再进入的热路径。
        for(std::size_t i = 0; i < input.objs.size(); ++i) {
            input.objs[i].type = i % 3;
            input.objs[i].vx = static_cast<int>((i + tick) % 3) - 1;
        }
        if(!safety.Observe(input, now, lane_filter)) return 2;
        const auto result = safety.Evaluate(path, ego, 3, now, true);
        // 保持在普通减速场景，不能因全程急停早退而漏测新速度曲线的热路径分配。
        if(result.emergency || result.speed_limit >= 3) return 3;
    }
    watching = false;
    std::printf("300 objects, 1000 warmed Observe+Evaluate calls: allocations=%zu cache_bytes=%zu\n",
                allocations, safety.WorkingBytes());
    return allocations == 0 ? 0 : 1;
}
