#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <limits>
#define main UltraNodeMain
#include "../../../ultra_command/src/ultra_command_node.cpp"
#undef main

int checks = 0;
void Check(bool condition, const char* name) {
    if(!condition) {
        std::cerr << "FAIL: " << name << '\n';
        std::exit(1);
    }
    ++checks;
}

void Speed(double speed) {
    boost::shared_ptr<robot::navigation_msg> msg(new robot::navigation_msg);
    msg->gpsSpeed = speed;
    CanMsgCallBack(msg);
}

int main(int argc, char** argv) {
    if(argc != 2) return 2;
    UltraNodeMain(argc, argv);
    const auto& topics = ros::testSubscriptions();
    Check(std::count(topics.begin(), topics.end(), "/navigation_msg") == 1, "node subscribes navigation speed");
    Check(std::count(topics.begin(), topics.end(), "/can_msg") == 0, "node no longer subscribes CAN speed");
    for(const char* path : {"pudong_air/312_316_01_01", "pudong_air/312_cargo_01_01", "pudong_air/312_charge_01_01"}) {
        g_comply = UltraCommandComply();
        Check(g_comply.LoadZones(argv[1]), "test zones loaded");
        boost::shared_ptr<robot::task_plan_msg> task(new robot::task_plan_msg);
        task->pathList = {path};
        task->task_id = 1;
        TaskPlanMsgCallBack(task);
        g_comply.SetObstacles({}, 100);
        Check(g_comply.JudgeSafeStatus(100) == 1, "missing navigation does not release initialization");
        Speed(std::numeric_limits<double>::quiet_NaN());
        Check(g_comply.JudgeSafeStatus(100) == 1, "invalid navigation does not become valid feedback");
        Speed(0);
        Check(g_comply.JudgeSafeStatus(100) == 0, "valid stopped navigation permits clear surroundings");
        g_comply.SetObstacles({UltraObstacle{5, 5}}, 100);
        Check(g_comply.JudgeSafeStatus(100) == 1, "obstacle blocks before navigation confirms motion");
        Speed(0.5);
        Check(g_comply.JudgeSafeStatus(100) == 1, "exact threshold does not exit initialization");
        Speed(0.5001);
        Check(g_comply.JudgeSafeStatus(100) == 0, "navigation exceeding threshold exits initialization");
        Speed(0);
        Check(g_comply.JudgeSafeStatus(102) == 0, "motion latch retains existing one-way behavior");
        task->task_id = 2;
        TaskPlanMsgCallBack(task);
        Check(g_comply.JudgeSafeStatus(102) == 1, "new task at rest rearms initialization");
    }
    for(const char* path : {"pudong_air/312_316", "pudong_air/312_cargo"}) {
        g_comply = UltraCommandComply();
        boost::shared_ptr<robot::task_plan_msg> task(new robot::task_plan_msg);
        task->pathList = {path};
        TaskPlanMsgCallBack(task);
        Check(g_comply.GetMode() == UltraCommandComply::ZONE_MODE_LEFT, "target left path activates LEFT mode");
    }
    {
        g_comply = UltraCommandComply();
        boost::shared_ptr<robot::task_plan_msg> task(new robot::task_plan_msg);
        task->pathList = {"pudong_air/312_charge"};
        TaskPlanMsgCallBack(task);
        Check(g_comply.GetMode() == UltraCommandComply::ZONE_MODE_RIGHT, "target charge path activates RIGHT mode");
    }
    for(const char* path : {"pudong_air/312_316_01", "pudong_air/312_cargo_01", "pudong_air/312_charge_01"}) {
        g_comply = UltraCommandComply();
        boost::shared_ptr<robot::task_plan_msg> task(new robot::task_plan_msg);
        task->pathList = {path};
        TaskPlanMsgCallBack(task);
        Check(g_comply.GetMode() == UltraCommandComply::ZONE_MODE_NONE, "old _01 path no longer activates monitor mode");
    }
    std::cout << "PASS navigation ultra: " << checks << " checks\n";
}
