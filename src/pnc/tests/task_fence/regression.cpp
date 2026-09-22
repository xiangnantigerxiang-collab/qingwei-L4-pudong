#include "test_access.h"
#include "test_trace.h"
#include "robot_task_plan/task_plan_core.h"
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <sys/stat.h>

static int checks = 0;
static void Check(bool ok, const char *reason) {
    ++checks;
    if(!ok) { std::cerr << "FAIL: " << reason << std::endl; std::exit(1); }
}
static void Write(const std::string &file, const std::string &text) { std::ofstream out(file); out << text; }
static std::string Route(double first, double last, double step = 0.5, double y = 20) {
    std::ostringstream out;
    for(double x = first; x <= last + 1e-6; x += step) out << x << ',' << y << ",90,2\n";
    return out.str();
}
static std::string Task(double x, bool action = false, int type = TRACKPATH, const std::string &path = "route") {
    std::ostringstream out;
    out << "task_sum: " << (action ? 2 : 1) << "\n";
    out << "task0_task_type: " << type << "\ntask0_path_list: '" << path << "'\n"
        << "task0_desire_speed: 2\ntask0_stop_x: " << x
        << "\ntask0_stop_y: 20\ntask0_stop_angle: 90\ntask0_gear: 4\ntask0_subaction: 0\n";
    if(action) out << "task1_task_type: 6\ntask1_path_list: 'missing'\ntask1_desire_speed: 0\n"
        << "task1_stop_x: -1\ntask1_stop_y: -1\ntask1_stop_angle: 0\ntask1_gear: 2\ntask1_subaction: 6\n";
    return out.str();
}
static NavStateIn Pose(double x = 5, double y = 20, double heading = 90) {
    NavStateIn pose; pose.xAxis = x; pose.yAxis = y; pose.heading = heading; pose.received_time = 100;
    return pose;
}
static void Init(TaskPlanCore &core, const std::string &dir, bool runtime = false) {
    core.InitParameter((dir + "/config.yaml").c_str());
    core.SetPathDir(dir);
    core.LoadFenceFile(dir);
    if(runtime) core.EnableFenceRuntime(1000);
    core.SetNavigationData(Pose());
    core.TickFence(100);
    CanStateIn can; can.controlPanelState = 1; core.SetCanData(can);
}
static void Dispatch(TaskPlanCore &core, const std::string &dir, int id = 1) {
    TaskInfoIn task; task.task_id = id;
    core.SetTaskInfo(task, dir + "/");
    core.TickFence(100);
    core.TaskPlanProcess(TaskPlanSink());
    core.PublishTaskPlanMsg(5, TaskPlanSink());
    core.recived_cloud_task = false;
}
static robot::task_plan_msg Message(const TaskPlanCore &core) {
    const auto &in = core.GetTaskPlanMsg(); robot::task_plan_msg msg;
    msg.workMode = in.workMode; msg.taskType = in.taskType; msg.task_id = in.task_id;
    msg.pathList = in.pathList; msg.desireSpeed = in.desireSpeed; msg.desireGear = in.desireGear;
    msg.stopX = in.stopX; msg.stopY = in.stopY; msg.stopAngle = in.stopAngle; msg.hookCmd = in.hookCmd;
    return msg;
}
static robot::task_fence_guard Guard(const TaskPlanCore &core) {
    const auto &in = core.GetFenceGuard(); robot::task_fence_guard msg;
    msg.header.stamp = ros::Time::now(); msg.revision = in.revision; msg.task_key = in.task_key;
    msg.fence_version = in.fence_version; msg.route_version = in.route_version; msg.fence_file = in.fence_file;
    msg.start_index = in.start_index; msg.stop_index = in.stop_index; msg.truncated = in.truncated;
    msg.stop = in.stop; msg.reason = in.reason; return msg;
}
static void PlannerInit(PathPlanComply &plan, const std::string &dir) {
    ros::param::set("path_dir", dir); ros::param::set("config_file", dir + "/config.yaml");
    plan.InitParameter(); plan.EnableFenceGuard();
    robot::navigation_msg pose; pose.xAxis = 5; pose.yAxis = 20; pose.heading = 90;
    plan.SetNavigationData(pose);
    robot::can_msg can; can.controlPanelState = 1; can.curGear = GEAR_D; plan.SetCanData(can);
}

int main(int argc, char **argv) {
    Check(argc == 3, "arguments");
    const std::string dir = argv[1], pnc = argv[2];
    const std::string square = "0,0,0,10\n100,0,0,10\n100,100,0,10\n0,100,0,10\n0,0,0,10\n";
    Write(dir + "/config.yaml", "{}\n"); Write(dir + "/fence.csv", square);
    Write(dir + "/route.csv", Route(5, 120)); Write(dir + "/task1.yaml", Task(120, true));
    task_fence::Geometry geometry; std::string error;
    Check(geometry.Load(dir + "/fence.csv", error), "valid closed polygon");
    Check(!geometry.Contains({100, 20}) && geometry.Contains({99, 20}), "strict boundary");
    Check(!geometry.PoseInside(98, 20, 90) && geometry.PoseInside(97, 20, 90), "old front corner geometry");
    Write(dir + "/bad.csv", "1,1,0,10\n"); Check(!geometry.Load(dir + "/bad.csv", error), "single vertex rejected");
    Write(dir + "/bad.csv", "0,0,0,10\n10,10,0,10\n0,10,0,10\n10,0,0,10\n");
    Check(!geometry.Load(dir + "/bad.csv", error), "self intersection rejected");
    Write(dir + "/bad.csv", "0,0,0,10\n20,0,0,10\n20,20,0,10\n15,20,0,10\n15,5,0,10\n5,5,0,10\n5,20,0,10\n0,20,0,10\n");
    Check(geometry.Load(dir + "/bad.csv", error), "concave polygon valid");
    Check(geometry.Contains({2, 15}) && geometry.Contains({18, 15}) &&
          !geometry.SegmentInside({2, 15}, {18, 15}), "inside endpoints cannot cross concavity");

    TaskPlanCore core; Init(core, dir, true); Dispatch(core, dir);
    Check(core.GetTaskPlanMsg().taskType == TRACKPATH && core.GetFenceGuard().truncated, "truncated driving dispatch");
    Check(core.GetTaskPlanMsg().stopX <= 97.5 && core.GetTaskStatus().fail_code == 0, "front clearance at stop");
    const auto limited_task = Message(core); const auto limited_guard = Guard(core);
    PlanStatusIn finished; finished.taskExecuStatus = TASKFINISHED;
    core.SetPathPlanStatus(finished);
    core.SetFenceFeedback(limited_guard.revision - 1, limited_guard.task_key, false, "", true);
    for(int i = 0; i < 30; ++i) core.TaskPlanProcess(TaskPlanSink());
    Check(core.mCurTaskNum == 0 && core.GetTaskStatus().fail_code == 0, "old generation completion ignored");
    core.SetFenceFeedback(limited_guard.revision, limited_guard.task_key, false, "", false);
    for(int i = 0; i < 30; ++i) core.TaskPlanProcess(TaskPlanSink());
    Check(core.GetTaskStatus().fail_code == 0, "geometry ack alone cannot complete task");
    core.SetFenceFeedback(limited_guard.revision, limited_guard.task_key, false, "", true);
    for(int i = 0; i < 30; ++i) core.TaskPlanProcess(TaskPlanSink());
    core.PublishTaskPlanMsg(5, TaskPlanSink());
    Check(core.GetTaskStatus().fail_code == 2 && core.GetTaskPlanMsg().taskType == NOTHING &&
          core.GetTaskPlanMsg().hookCmd == 0 && core.GetTaskPlanMsg().desireSpeed == 0, "truncation cancels decoupling");
    CommandIn command; command.commandState = 2; core.OnCommandMsg(command, TaskPlanSink());
    Check(core.GetFenceGuard().stop, "continue cannot clear fence abort");

    PathPlanComply plan; PlannerInit(plan, dir);
    plan.SetTaskPlanData(limited_task); plan.TryLoadFenceTask();
    Check(plan.mFencePending && plan.FenceOutputStop(), "task before guard holds");
    plan.SetFenceGuard(limited_guard); plan.TryLoadFenceTask();
    Check(!plan.mFenceFailed && !plan.mFencePending && plan.mStopIndex == (int)plan.mDrivingPath.size() - 1,
          "approved prefix loads exact final stop");
    Check(std::abs(plan.mDrivingPath.back().x_axis - limited_task.stopX) < 0.01, "planner stop matches task");
    Check(plan.GetMapSpeedLimit() == 2, "CSV fourth column retained");
    std::vector<float> x, y, heading; plan.BuildReferencePathPoints(x, y, heading);
    Check(!x.empty() && *std::max_element(x.begin(), x.end()) <= limited_task.stopX, "preview never extends past stop");
    plan.mPlanPath.x = x; plan.mPlanPath.y = y; plan.mPlanPath.desireSpeed = 2;
    Check(!plan.FenceOutputStop(), "valid preview allowed");
    ros::testTime() = 100.6; Check(plan.FenceOutputStop(), "guard timeout holds");
    ros::testTime() = 99; Check(plan.FenceOutputStop(), "clock rewind holds"); ros::testTime() = 100;
    plan.mPlanPath.safety = true; ros::Publisher publisher;
    bool captured = false; publisher.capture = [&](const std::type_info &type, const void *value) {
        if(type == typeid(robot::path_plan_msg)) { Check(static_cast<const robot::path_plan_msg *>(value)->safety, "other safety retained"); captured = true; }
    };
    plan.PublishFinalPlanPath(publisher); Check(captured && plan.mPlanPath.safety, "safety saved and restored");
    plan.mPlanPath.x = {5, 101}; plan.mPlanPath.y = {20, 20};
    Check(plan.FenceOutputStop() && plan.mFenceFailed, "final dynamic path crossing rejected");
    publisher.capture = [&](const std::type_info &type, const void *value) {
        if(type == typeid(robot::task_fence_guard)) {
            const auto &fb = *static_cast<const robot::task_fence_guard *>(value);
            Check(fb.stop && fb.revision == limited_guard.revision && !fb.completed, "failure feedback identity");
        }
    };
    plan.PublishFenceFeedback(publisher);
    PathPlanComply guard_first; PlannerInit(guard_first, dir);
    guard_first.SetFenceGuard(limited_guard); guard_first.SetTaskPlanData(limited_task); guard_first.TryLoadFenceTask();
    Check(!guard_first.mFencePending && !guard_first.mFenceFailed, "guard before task works");
    auto newer = limited_guard; newer.revision++;
    guard_first.SetFenceGuard(newer); Check(guard_first.FenceOutputStop(), "next generation waits task event");
    guard_first.SetTaskPlanData(limited_task); guard_first.TryLoadFenceTask();
    Check(!guard_first.mFenceFailed && guard_first.mFenceLoadedRevision == newer.revision, "same task retry uses new generation");
    guard_first.SetFenceGuard(limited_guard); Check(guard_first.mFenceGuard.revision == newer.revision, "old guard ignored");

    Write(dir + "/route.csv", Route(110, 130)); Write(dir + "/task1.yaml", Task(130));
    TaskPlanCore outside; Init(outside, dir); Dispatch(outside, dir);
    Check(outside.GetTaskStatus().fail_code == 2 && outside.GetTaskPlanMsg().desireSpeed == 0, "all outside fails closed");
    Write(dir + "/route.csv", Route(99, 120));
    TaskPlanCore early; Init(early, dir); early.SetNavigationData(Pose(99)); Dispatch(early, dir);
    Check(early.GetTaskStatus().fail_code == 2, "early unsafe stop cannot map to index four");
    Write(dir + "/task1.yaml", Task(50, false, TRACKPATH, "missing"));
    TaskPlanCore missing; Init(missing, dir); Dispatch(missing, dir);
    Check(missing.GetTaskStatus().fail_code == 2, "missing path rejected");
    Write(dir + "/task1.yaml", Task(50)); Write(dir + "/route.csv", Route(5, 50));
    Write(dir + "/fence.csv", "1,1,0,10\n"); TaskPlanCore invalid; Init(invalid, dir); Dispatch(invalid, dir);
    Check(invalid.GetTaskStatus().fail_code == 2, "invalid fence rejected");
    Write(dir + "/fence.csv", square); TaskPlanCore cache; Init(cache, dir); Dispatch(cache, dir);
    Check(cache.GetTaskStatus().fail_code == 0, "valid route accepted");
    Write(dir + "/route.csv", Route(110, 130)); Write(dir + "/task2.yaml", Task(130)); Dispatch(cache, dir, 2);
    Check(cache.GetTaskStatus().fail_code == 2, "same-name path cache invalidated");
    Write(dir + "/fence.csv", "100,100,0,10\n200,100,0,10\n200,200,0,10\n100,200,0,10\n");
    Check(cache.LoadFenceFile(dir) && cache.IsPointInFence(150, 150) && !cache.IsPointInFence(5, 20), "same-name fence invalidated");
    const std::string other = dir + "/other"; ::mkdir(other.c_str(), 0700); Write(other + "/fence.csv", square);
    cache.SetPathDir(other); Check(cache.LoadFenceFile(other) && cache.IsPointInFence(5, 20) &&
                                 !cache.IsPointInFence(150, 150), "directory invalidates geometry");

    Write(dir + "/fence.csv", square); Write(dir + "/route.csv", Route(5, 50)); Write(dir + "/task1.yaml", Task(50));
    TaskPlanCore runtime; Init(runtime, dir, true); Dispatch(runtime, dir);
    runtime.TickFence(100.6); Check(runtime.GetFenceGuard().stop && runtime.GetFenceAlarm() == -1, "stale navigation holds unknown");
    runtime.TickFence(99); Check(runtime.GetFenceGuard().stop, "task clock rewind holds");
    runtime.TickFence(100); Check(!runtime.GetFenceGuard().stop && runtime.GetFenceAlarm() == 0, "fresh navigation recovers temporary hold");
    runtime.SetNavigationData(Pose(98)); runtime.TickFence(100);
    Check(runtime.GetTaskStatus().fail_code == 2 && runtime.GetFenceGuard().stop, "actual front outside aborts task");
    TaskPlanCore hook; Init(hook, dir, true); Write(dir + "/task1.yaml", Task(50, false, ADAPTIVEHOOK, "missing"));
    Dispatch(hook, dir); hook.SetNavigationData(Pose(101)); hook.TickFence(100);
    Check(!hook.GetFenceGuard().stop && hook.GetTaskStatus().fail_code == 0, "adaptive hook exemption retained");
    TaskPlanCore action; Init(action, dir, true); Write(dir + "/task1.yaml", Task(-1, false, DOACTION, "missing"));
    Dispatch(action, dir); Check(action.GetTaskStatus().fail_code == 0, "action placeholder path allowed");
    TaskPlanCore actual; actual.InitParameter((dir + "/config.yaml").c_str()); actual.SetPathDir(pnc + "/path/");
    TaskInfoIn real; real.task_id = 1; actual.SetTaskInfo(real, pnc + "/param/"); actual.PublishTaskPlanMsg(5, TaskPlanSink());
    Check(actual.GetTaskStatus().fail_code == 2 && actual.GetTaskPlanMsg().desireSpeed == 0, "real task1 all-outside repro rejected");
    TaskPlanCore remote; Init(remote, dir); Write(dir + "/task1000.yaml", Task(50));
    remote.RunningtXAxis = 120; remote.RunningtYAxis = 20; remote.RunningtAngle = 90; Dispatch(remote, dir, 1000);
    Check(remote.GetTaskStatus().fail_code == 2, "outside remote destination rejected");

    Write(dir + "/task1.yaml", Task(50)); TaskPlanCore version; Init(version, dir, true); Dispatch(version, dir);
    auto old_task = Message(version); auto old_guard = Guard(version);
    Write(dir + "/route.csv", Route(5, 51)); PathPlanComply changed; PlannerInit(changed, dir);
    changed.SetTaskPlanData(old_task); changed.SetFenceGuard(old_guard); changed.TryLoadFenceTask();
    Check(changed.mFenceFailed, "task/planning file version mismatch rejected");
    Write(dir + "/route.csv", Route(5, 50));
    // 正常 D/R 路径使用新许可链，与同版原有规划入口逐字段比较最终消息。
    for(int gear : {GEAR_D, GEAR_R}) {
        TaskPlanCore normal; Init(normal, dir, true); Dispatch(normal, dir);
        auto task = Message(normal); auto guard = Guard(normal);
        task.desireGear = gear; guard.task_key = task_fence::TaskKey(task);
        PathPlanComply enabled, baseline; PlannerInit(enabled, dir); PlannerInit(baseline, dir);
        baseline.mFenceEnabled = false;
        robot::can_msg can; can.controlPanelState = 1; can.curGear = gear;
        enabled.SetCanData(can); baseline.SetCanData(can);
        enabled.SetFenceGuard(guard); enabled.SetTaskPlanData(task); baseline.SetTaskPlanData(task);
        ros::Publisher output, sound, reference;
        std::string before, after;
        output.capture = [&](const std::type_info &type, const void *value) {
            if(type == typeid(robot::path_plan_msg)) {
                std::ostringstream stream; TraceValue(stream, *static_cast<const robot::path_plan_msg *>(value));
                before = stream.str();
            }
        };
        baseline.PathPlanProcess(); baseline.PublishReferPath(reference); baseline.PublishPlanPath(output, sound);
        output.capture = [&](const std::type_info &type, const void *value) {
            if(type == typeid(robot::path_plan_msg)) {
                std::ostringstream stream; TraceValue(stream, *static_cast<const robot::path_plan_msg *>(value));
                after = stream.str();
            }
        };
        enabled.PathPlanProcess(); enabled.PublishReferPath(reference); enabled.PublishPlanPath(output, sound);
        Check(!enabled.mFenceFailed && !enabled.mFencePending, "normal task admitted with fence enabled");
        Check(!before.empty() && before == after, "normal D/R full planning output unchanged");
    }
    // 已经走过的围栏外前缀不应使当前安全区间被错误截断。
    Write(dir + "/route.csv", Route(-20, 50));
    TaskPlanCore prefix; Init(prefix, dir, true); Dispatch(prefix, dir);
    Check(prefix.GetTaskStatus().fail_code == 0 && !prefix.GetFenceGuard().truncated &&
          prefix.GetFenceGuard().start_index == 50, "actual start skips passed outside prefix");
    prefix.SetNavigationData(Pose(-10)); Write(dir + "/task2.yaml", Task(50)); Dispatch(prefix, dir, 2);
    Check(prefix.GetTaskStatus().fail_code == 2, "outside first point cannot skip ahead to inside island");
    Write(dir + "/route.csv", Route(5, 50));
    TaskPlanCore transient; Init(transient, dir); Write(dir + "/task1.yaml", Task(50, false, TRACKPATH, "recovered"));
    Dispatch(transient, dir); Check(transient.GetTaskStatus().fail_code == 2, "missing file initially fails");
    Write(dir + "/recovered.csv", Route(5, 50)); Dispatch(transient, dir);
    Check(transient.GetTaskStatus().fail_code == 0, "missing file cache retries when repaired");
    Write(dir + "/task1.yaml", Task(50));
    TaskPlanCore stopped; Init(stopped, dir, true); Dispatch(stopped, dir);
    command.commandState = 0; stopped.OnCommandMsg(command, TaskPlanSink());
    Check(stopped.recived_cloud_task, "cloud stop schedules terminal task dispatch");
    stopped.PublishTaskPlanMsg(5, TaskPlanSink());
    Check(stopped.GetTaskPlanMsg().taskType == NOTHING, "cloud stop clears planner task contract");
    TaskPlanCore nonfinite; Init(nonfinite, dir, true); Dispatch(nonfinite, dir);
    NavStateIn bad_pose = Pose(); bad_pose.heading = std::numeric_limits<float>::quiet_NaN();
    nonfinite.SetNavigationData(bad_pose); nonfinite.TickFence(100);
    Check(nonfinite.GetFenceGuard().stop && nonfinite.GetFenceAlarm() == -1, "NaN pose cannot release motion");
    // 场地真实多边形与热循环计时；只报告主机数据，不当作 Orin 时限。
    TaskPlanCore real_fence; real_fence.LoadFenceFile(pnc + "/path/"); real_fence.SetNavigationData(Pose(0, 0, 250));
    const auto real_begin = std::chrono::steady_clock::now();
    for(int i = 0; i < 10000; ++i) real_fence.TickFence(100);
    const double real_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - real_begin).count() / 10000;
    Check(real_fence.GetFenceList().size() >= 100, "real fence geometry loaded");
    std::cout << "Runtime real fence host mean us=" << real_us << std::endl;
    const auto begin = std::chrono::steady_clock::now();
    for(int i = 0; i < 10000; ++i) version.TickFence(100);
    const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - begin).count() / 10000;
    std::cout << "Runtime square fence host mean us=" << us << std::endl;
    std::cout << "PASS: " << checks << " fence regression checks" << std::endl;
}
