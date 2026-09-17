#include "test_access.h"
#include <chrono>
#include <cstdio>
#include <iomanip>
#include <new>

int main() {
    if(!std::freopen("/dev/null", "w", stdout)) return 2;
    std::cerr << std::fixed << std::setprecision(6);
    for(int count : {20, 100, 300, 1000}) {
        for(int dense : {0, 1}) {
            void* memory = ::operator new(sizeof(PathPlanComply));
            std::memset(memory, 0, sizeof(PathPlanComply));
            PathPlanComply* plan = new(memory) PathPlanComply;
            plan->mNavData.xAxis = 100;
            plan->mNavData.yAxis = 100;
            plan->mNavData.heading = 90;
            plan->mVehicleData.vehicleSpeed = 1;
            for(int i = 0; i < 20; ++i) {
                plan->mReferPath.x.push_back(100 + i * 0.5);
                plan->mReferPath.y.push_back(100);
            }
            robot::perception message, legacy;
            for(int i = 0; i < count; ++i) {
                robot::object object;
                object.id = i + 1;
                object.x = dense ? 104 + (i % 10) * 0.3 : 104 + (i % 25) * 1.5;
                object.y = dense ? 99 + (i % 5) * 0.5 : 100 + (i / 25 - count / 50) * 4;
                object.dx = 1;
                object.dy = 1.2;
                object.heading = 90;
                object.confidence = 0.8;
                legacy.objs.push_back(object);
                object.polygons.resize(4);
                const double dx[] = {0.6, 0.6, -0.6, -0.6}, dy[] = {-0.5, 0.5, 0.5, -0.5};
                for(int j = 0; j < 4; ++j) {
                    object.polygons[j].x = object.x + dx[j];
                    object.polygons[j].y = object.y + dy[j];
                }
                message.objs.push_back(object);
            }
            std::vector<double> times;
            times.reserve(400);
            for(int frame = 0; frame < 420; ++frame) {
                ros::testTime() = 100 + frame * 0.1;
                message.header.stamp = ros::Time(ros::testTime());
                legacy.header = message.header;
                ros::param::events().clear();
                plan->mReferPath.safety = false;
                plan->mReferPath.desireSpeed = 3;
                const auto start = std::chrono::steady_clock::now();
#ifdef WITH_TRACKED_PERCEPTION
                plan->SetPlanningPerceptionData(message);
#endif
                plan->SetPerceptionData(legacy);
                plan->CheckForwardReferenceSafety();
                const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
                if(frame >= 20) times.push_back(ms);
            }
            std::sort(times.begin(), times.end());
            std::cerr << "objects=" << count << " dense=" << dense << " p50_ms=" << times[200]
                      << " p99_ms=" << times[396] << " max_ms=" << times.back();
#ifdef WITH_TRACKED_PERCEPTION
            std::cerr << " cache_bytes=" << plan->mPerceptionSafety.WorkingBytes();
#endif
            std::cerr << '\n';
            plan->~PathPlanComply();
            ::operator delete(plan);
        }
    }
}
