// 同一组业务输入分别驱动重构前后实现；输出完整消息、参数事件及跨帧状态供逐字节比较。
#include "test_access.h"
#include "navigation_feedback.h"
#include "test_trace.h"
#include <cassert>
#include <fstream>
#include <new>
#include <random>

std::ofstream trace;
std::string pnc_dir;
std::string fixture_dir;

struct Fixture {
    PathPlanComply *plan;
    ros::Publisher refer_pub;
    ros::Publisher plan_pub;
    ros::Publisher sound_pub;
    ros::Publisher status_pub;

    Fixture() {
        // 生产节点的全局 Comply 在构造前经过静态零初始化；测试按同样的初态构造。
        void *storage = ::operator new(sizeof(PathPlanComply));
        std::memset(storage, 0, sizeof(PathPlanComply));
        plan = new(storage) PathPlanComply;
        ros::param::values().clear();
        ros::param::events().clear();
        ros::testTime() = 100.0;
        ros::param::set("task_file", pnc_dir + "/param/");
        ros::param::set("path_dir", pnc_dir + "/path/");
        plan->InitParameter();
        plan->sysTime.now = 100.0;
        plan->sysTime.taskStart = 90.0;
        plan->sysTime.vehicleStop = 90.0;
        plan->sysTime.vehicleRun = 90.0;
        plan->mTaskPlanData.task_id = 12;
        plan->mTaskPlanData.taskType = TRACKPATH;
        plan->mTaskPlanData.desireSpeed = 3.0;
        plan->mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
        plan->mDesireSpeed = 3.0;
        robot::navigation_msg nav;
        nav.xAxis = 100.0;
        nav.yAxis = 100.0;
        nav.heading = 90.0;
        nav.rtkState = "fixed";
        plan->SetNavigationData(nav);
        robot::can_msg can;
        can.curGear = GEAR_D;
        can.controlPanelState = 1;
        // 旧业务差分从已完成起步观察的自动会话开始，专门的手动切自动另有测试。
        can.vehicleSpeed = 0.3;
        ros::testTime() = 99.8;
        SetVehicleFeedback(*plan, can);
        ros::testTime() = 99.9;
        SetVehicleFeedback(*plan, can);
        can.vehicleSpeed = 0;
        ros::testTime() = 100.0;
        SetVehicleFeedback(*plan, can);
        for(int i = 0; i < 160; i++) {
            XYZ_COOR_S point{};
            point.x_axis = 100.0 + i * 0.2;
            point.y_axis = 100.0;
            point.heading = 90.0;
            point.dist_origin = i * 0.2;
            point.p2pDistance = 0.2;
            point.velocity = 5.0;
            plan->mDrivingPath.push_back(point);
        }
        plan->mStopIndex = 150;
        plan->mPathPlanStatus.stopX = plan->mDrivingPath[150].x_axis;
        plan->mPathPlanStatus.stopY = 100.0;
        refer_pub.name = "refer";
        plan_pub.name = "plan";
        sound_pub.name = "sound";
        status_pub.name = "status";
        auto capture_path = [](const std::type_info &, const void *data) {
            trace << "PATH ";
            TraceValue(trace, *static_cast<const robot::path_plan_msg *>(data));
            trace << '\n';
        };
        refer_pub.capture = capture_path;
        plan_pub.capture = capture_path;
        sound_pub.capture = [](const std::type_info &, const void *data) {
            trace << "SOUND ";
            TraceValue(trace, *static_cast<const robot::sound_light_msg *>(data));
            trace << '\n';
        };
        status_pub.capture = [](const std::type_info &, const void *data) {
            trace << "STATUS ";
            TraceValue(trace, *static_cast<const robot::path_plan_status *>(data));
            trace << '\n';
        };
    }

    ~Fixture() {
        plan->~PathPlanComply();
        ::operator delete(plan);
    }

    void Snapshot(const std::string &name) {
        trace << "STATE " << name << '\n';
        TraceValue(trace, plan->mReferPath);
        TraceValue(trace, plan->mPlanPath);
        TraceValue(trace, plan->mPathPlanStatus);
        TraceValue(trace, plan->mTaskPlanData);
        TraceValue(trace, plan->mPerception);
        TraceValue(trace, plan->mPerceptionFrontScan);
        TraceValue(trace, plan->mPerceptionBackScan);
        trace << '\n';
        for(double value : {double(plan->mKeypoint), double(plan->mStopIndex), double(plan->mDesireSpeed),
                            double(plan->remain_distance_), double(plan->command_state),
                            double(plan->emergencyStop), double(plan->HornFlag), double(plan->backdist_flag),
                            double(plan->safety_check_counter_refer), double(plan->safety_check_counter_percep),
                            plan->sysTime.vehicleStop, plan->sysTime.vehicleRun, plan->sysTime.taskStart})
            TraceValue(trace, value);
        TraceValue(trace, plan->history_risk_vec);
        trace << '\n';
        for(auto point : plan->mDrivingPath) {
            TraceValue(trace, point.x_axis);
            TraceValue(trace, point.y_axis);
            TraceValue(trace, point.heading);
            TraceValue(trace, point.dist_origin);
        }
        trace << '\n';
        for(const auto &path : {plan->path_final, plan->path_final_last}) {
            for(auto point : path) {
                TraceValue(trace, point.x);
                TraceValue(trace, point.y);
            }
            trace << '\n';
        }
        for(const auto &event : ros::param::events())
            trace << event << '\n';
        ros::param::events().clear();
    }

    void Tick(const std::string &name, bool process = true) {
        ros::testTime() += 0.1;
        plan->sysTime.now = ros::testTime();
        if(process)
            plan->PathPlanProcess();
        plan->PublishReferPath(refer_pub);
        plan->PublishPlanPath(plan_pub, sound_pub);
        plan->PublishPathPlanStatus(status_pub);
        Snapshot(name);
    }
};

robot::object ObstacleAt(double x, double y) {
    robot::object obj;
    obj.x = x;
    obj.y = y;
    obj.dx = 0.6;
    obj.dy = 0.8;
    obj.heading = 90.0;
    return obj;
}

void TestStartupAndStops() {
    Fixture f;
    for(int i = 0; i < 35; i++) {
        f.Tick("startup-" + std::to_string(i));
        assert(f.plan->mPlanPath.desireSpeed > 0.0);
    }
    for(int reason = 0; reason < 4; reason++) {
        f.plan->emergencyStop = reason == 0;
        f.plan->command_state = reason == 1 ? 1 : 2;
        ros::param::set("/robot/planning/netcheck", reason == 2 ? 1 : 0);
        f.plan->mVehicleData.controlPanelState = reason == 3 ? 0 : 1;
        f.Tick("stop-" + std::to_string(reason));
        assert(f.plan->mPlanPath.desireSpeed == 0.0);
    }
    f.plan->mVehicleData.controlPanelState = 1;
    f.plan->mDrivingPath.resize(19);
    f.plan->mReferPath.x.resize(4);
    f.Tick("short-path", false);
    assert(f.plan->mPlanPath.x.empty());
    f.plan->mPathPlanStatus.taskExecuStatus = TASKFINISHED;
    f.Tick("finished", false);
    assert(f.plan->mReferPath.x.empty());
}

void TestCollisionAndReverse() {
    Fixture f;
    f.plan->mPerception.objs = {ObstacleAt(103, 100)};
    for(int i = 0; i < 3; i++) {
        f.Tick("collision-" + std::to_string(i), false);
        assert(f.plan->mReferPath.safety == (i == 2));
    }
    f.plan->mPerception.objs.clear();
    f.Tick("empty-perception", false);
    assert(f.plan->safety_check_counter_refer == 10);
    f.plan->mPerception.objs = {ObstacleAt(180, 140)};
    for(int i = 0; i < 5; i++)
        f.Tick("risk-history-" + std::to_string(i), false);
    assert(f.plan->mPathPlanStatus.distance2Object == 101);

    // 覆盖前向角度过滤后的 200+n 哨兵，以及 ultra 参数的既有早退边界。
    f.plan->mNavData.heading = 270.0;
    f.plan->mPerception.objs = {ObstacleAt(103, 100)};
    f.Tick("behind-risk", false);
    assert(f.plan->mPathPlanStatus.distance2Object == 201);
    f.plan->mNavData.heading = 90.0;
    f.plan->mPerception.objs = {ObstacleAt(112, 100)};
    ros::param::set("/ultra/status/safe", 1);
    f.Tick("ultra-risk", false);
    assert(f.plan->mReferPath.desireSpeed == 0.0);
    f.plan->mPerception.objs.clear();
    f.Tick("ultra-empty-early-return", false);
    assert(f.plan->mReferPath.desireSpeed > 0.0);

    f.plan->mVehicleData.curGear = GEAR_R;
    f.plan->mKeypoint = 155;  // 触发末端轨迹适配，仍满足控制侧至少 10 点的要求。
    for(double distance : {2.49, 2.5, 2.51, 8.0}) {
        f.plan->mPerceptionBackScan.objs = {ObstacleAt(100 + distance, 100)};
        f.Tick("reverse-" + std::to_string(distance), false);
        assert(f.plan->mReferPath.x.size() >= 10);
        assert(f.plan->mReferPath.safety == (distance < 2.5));
    }
}

void TestTaskSwitchAndPaths() {
    Fixture f;
    auto task = f.plan->mTaskPlanData;
    task.pathList = {"pudong_air/312_316_01"};
    task.stopX = -200;
    task.stopY = 500;
    task.workMode = 1;
    f.plan->mNavData.xAxis = 0;
    f.plan->mNavData.yAxis = 0;
    f.plan->command_state = 1;
    f.plan->SetTaskPlanData(task);
    f.Snapshot("same-task");
    assert(f.plan->command_state == 1);
    f.plan->emergencyStop = 1;
    task.task_id++;
    f.plan->SetTaskPlanData(task);
    f.Snapshot("new-task");
    assert(f.plan->command_state == 2);
    assert(f.plan->emergencyStop == 1);
    assert(f.plan->mDrivingPath.size() > 20);
    f.plan->SetTaskPlanData(task);
    f.Snapshot("repeat-message");

    // 真实 CSV 的装载与当前位姿到全局路径的样条连接。
    f.plan->mNavData.xAxis = f.plan->mDrivingPath[20].x_axis + 1.8;
    f.plan->mNavData.yAxis = f.plan->mDrivingPath[20].y_axis;
    f.plan->mNavData.heading = f.plan->mDrivingPath[20].heading;
    f.plan->SetTaskPlanData(task);
    f.Snapshot("connect-real-path");
    task.pathList = {"pudong_air/312_316_01", "pudong_air/312_cargo_01"};
    f.plan->SetTaskPlanData(task);
    f.Snapshot("multiple-csv");
    // 使用横向偏差已知的直线确保连接分支确实执行，避免真实路径沿线偏移未触发。
    ros::param::set("path_dir", fixture_dir + "/");
    task.pathList = {"straight"};
    task.stopX = 130;
    task.stopY = 100;
    f.plan->mNavData.xAxis = 100;
    f.plan->mNavData.yAxis = 101.8;
    f.plan->mNavData.heading = 90;
    f.plan->SetTaskPlanData(task);
    f.Snapshot("connect-synthetic-path");
    assert(f.plan->mDrivingPath.size() > 200);
    assert(f.plan->mDrivingPath.front().y_axis > 101.0);
    task.pathList = {"missing-regression-path"};
    f.plan->SetTaskPlanData(task);
    f.Snapshot("missing-csv");
    assert(f.plan->mDrivingPath.empty());
    task.taskType = NOTHING;
    f.plan->SetTaskPlanData(task);
    f.Tick("no-task");
    for(int command : {1, 2, 0}) {
#ifdef REFACTORED
        robot::v2nCommandFeedback msg;
        msg.commandState = command;
        f.plan->SetCommandData(msg);
#else
        f.plan->SetCommandState(command);
        if(command == 0)
            f.plan->resetTask();
#endif
        f.Snapshot("command-" + std::to_string(command));
    }
}

void TestTaskSpeedRules() {
    for(int type : {TRACKPATH, DOACTION, ADAPTIVEPARK, ADAPTIVEHOOK}) {
        Fixture f;
        f.plan->mTaskPlanData.taskType = type;
        for(double distance : {0.1, 0.2, 0.29, 0.3, 0.49, 0.5, 0.51, 1.49, 1.5, 6.0, 10.0, 10.1}) {
            f.plan->mDrivingPath[f.plan->mStopIndex].dist_origin = distance;
            f.plan->mHookPos.center_distance = distance;
            f.plan->mTaskPlanData.hookCmd = HOOKOPERATION;
            f.plan->mVehicleData.hookStatus = distance < 1 ? ACTUATOR_UP_END : ACTUATOR_DOWN_END;
            f.plan->mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
            float speed = f.plan->LimitSpeedByDistanceToStop(100, 100, 270, 101, 100, 0);
            trace << "SPEED ";
            TraceValue(trace, speed);
            f.Snapshot("task-speed-" + std::to_string(type) + "-" + std::to_string(distance));
        }
        if(type == ADAPTIVEHOOK) {
            f.plan->mHookPos.center_distance = 0.8;
            f.plan->mVehicleData.linkPallet = true;
            assert(f.plan->LimitSpeedByDistanceToStop(100, 100, 270, 101, 100, 0) == 0);
            assert(f.plan->mPathPlanStatus.taskExecuStatus == TASKFINISHED);
            f.Snapshot("hook-linked");
        }
        if(type == DOACTION) {
            f.plan->mTaskPlanData.hookCmd = DECOUPLING;
            f.plan->mVehicleData.hookStatus = ACTUATOR_DOWN_END;
            f.plan->LimitSpeedByDistanceToStop(0, 0, 0, 0, 0, 0);
            assert(f.plan->mPathPlanStatus.taskExecuStatus == TASKFINISHED);
            f.Snapshot("decoupling");
        }
    }
    Fixture f;
    f.plan->go_task_id_ = 12;
    AirCraftParkingPort port;
    port.id = 1;
    port.parking = 1;
    port.go_stop_index = 30;
    f.plan->aircraft_parking_ports_ = {port};
    for(int keypoint : {0, 28, 30, 150}) {
        f.plan->mKeypoint = keypoint;
        f.Tick("airport-stop-" + std::to_string(keypoint), false);
        float speed = f.plan->LimitSpeedByDistanceToStop(100, 100, 90, 130, 100, 0);
        TraceValue(trace, speed);
        f.Snapshot("airport-speed-" + std::to_string(keypoint));
        if(keypoint == 28 || keypoint == 30) {
            assert(speed == 0.0);
            assert(f.plan->mPathPlanStatus.taskExecuStatus != TASKFINISHED);
        }
    }
    f.plan->mStopIndex = -1;
    assert(f.plan->LimitSpeedByDistanceToStop(0, 0, 0, 0, 0, 0) == 0);
    f.Snapshot("invalid-stop-index");
}

void TestWaitingAreaAndProgress() {
    Fixture f;
    f.plan->mNavData.xAxis = 0;
    f.plan->mNavData.yAxis = 0;
    // 原固定等待区不再叠加区域停车；路径碰撞仍由参考路径阶段判断。
    f.plan->mPerception.objs = {ObstacleAt(5, 0)};
    f.Tick("waiting-area", false);
    assert(f.plan->mPlanPath.desireSpeed > 0);
    assert(ros::param::values()["/canbus/light"] != "7");

    f.plan->mNavData.xAxis = 100;
    f.plan->mNavData.yAxis = 100;
    f.plan->mKeypoint = 40;
    f.plan->UpdatePathInfo();
    assert(f.plan->mKeypoint >= 40);
    f.Snapshot("progress-no-backtrack");
    f.plan->mKeypoint = 0;
    f.plan->mNavData.xAxis = 124;
    f.plan->UpdatePathInfo();
    // 原循环以不断更新的 keyPoint 作为上界，实际可前进超过初始的 80 点。
    assert(f.plan->mKeypoint == 120);
    f.Snapshot("progress-moving-window");
}

void TestInputAndTrajectoryReuse() {
    Fixture f;
    robot::perception perception;
    perception.objs = {ObstacleAt(103, 100), ObstacleAt(90, 100)};
    for(int i = 0; i < 4; i++) {
        f.plan->SetPerceptionData(perception);
        f.Tick("input-filter-" + std::to_string(i), false);
    }
    jsk_recognition_msgs::BoundingBoxArray scan;
    scan.boxes.resize(1);
    scan.boxes[0].pose.position.x = 2.0;
    f.plan->SetFrontScanData(scan);
    f.plan->SetBackScanData(scan);
    f.plan->SetPerceptionData(perception);
    f.Snapshot("scan-input");
    f.plan->mPerception.objs.clear();
    for(int i = 0; i < 40; i++) {
        OriginalInsData point{};
        point.x = 95 + i;
        point.y = 100;
        f.plan->path_final_last.push_back(point);
    }
    f.Tick("reuse-last-trajectory", false);
    assert(f.plan->path_final.front().x == 100);
    f.plan->mPerception.objs = {ObstacleAt(105, 100)};
    f.Tick("reuse-collision", false);
}

void TestSeededScenarios() {
    // 固定种子覆盖障碍物位置、挡位、暂停/急停和加速平滑组合，并保留连续帧历史。
    std::mt19937 random(20260912);
    Fixture f;
    for(int i = 0; i < 250; i++) {
        f.plan->mVehicleData.curGear = i % 11 == 0 ? GEAR_R : GEAR_D;
        f.plan->mNavData.gpsSpeed = (random() % 45) / 10.0;
        // 改前/改后都使用同一实测速度；来源冲突由 navigation_speed 专项单独覆盖。
        f.plan->mVehicleData.vehicleSpeed = f.plan->mNavData.gpsSpeed;
        f.plan->mPathPlanStatus.curSpeed = f.plan->mNavData.gpsSpeed;
        f.plan->mVehicleData.linkPallet = i % 3 == 0;
        f.plan->mVehicleData.hookStatus = i % 2 ? ACTUATOR_UP_END : ACTUATOR_DOWN_END;
        f.plan->command_state = i % 17 == 0 ? 1 : 2;
        f.plan->emergencyStop = i % 23 == 0;
        f.plan->mPerception.objs.clear();
        for(int j = 0; j < i % 5; j++)
            f.plan->mPerception.objs.push_back(ObstacleAt(98 + (random() % 220) / 10.0, 97 + (random() % 60) / 10.0));
        f.Tick("seeded-" + std::to_string(i), false);
    }
}

int main(int argc, char **argv) {
    assert(argc == 4);
    pnc_dir = argv[1];
    fixture_dir = argv[3];
    trace.open(argv[2]);
    assert(trace.good());
    trace << std::setprecision(17);
    TestStartupAndStops();
    TestCollisionAndReverse();
    TestTaskSwitchAndPaths();
    TestTaskSpeedRules();
    TestWaitingAreaAndProgress();
    TestInputAndTrajectoryReuse();
    TestSeededScenarios();
    trace.close();
    return 0;
}
