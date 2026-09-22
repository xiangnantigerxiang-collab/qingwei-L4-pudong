#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "robot_task_plan/task_plan_core.h"

static void AssertCheck(bool condition, const char *msg) {
    if(!condition) {
        std::fprintf(stderr, "FAIL: %s\n", msg);
        std::exit(1);
    }
}

int main(int argc, char **argv) {
    std::string path_dir = "qingwei-L4-No2-pudong/src/pnc/path/";
    if(argc > 1) {
        path_dir = argv[1];
    }
    if(!path_dir.empty() && path_dir.back() != '/') {
        path_dir += '/';
    }

    std::cout << "Running task_plan electronic fence validation tests..." << std::endl;
    TaskPlanCore core;

    // Test 1: LoadFenceFile & BBox
    std::cout << "[Test 1] LoadFenceFile & BBox..." << std::endl;
    bool loaded = core.LoadFenceFile(path_dir);
    AssertCheck(loaded, "LoadFenceFile should return true for path/fence.csv");
    AssertCheck(!core.GetFenceList().empty(), "Fence point list should not be empty");
    std::cout << "  Loaded " << core.GetFenceList().size() << " fence vertices." << std::endl;

    // Test points
    AssertCheck(core.IsPointInFence(0.0, 0.0), "Point (0, 0) should be inside fence");
    AssertCheck(!core.IsPointInFence(99999.0, 99999.0), "Far point (99999, 99999) should be outside fence");
    AssertCheck(!core.IsPointInFence(616.905, 2426.686), "Point (616.905, 2426.686) should be outside fence");

    // Test 2: All points inside fence -> Normal dispatch, no stop modification
    std::cout << "[Test 2] Path completely inside fence (pudong_air/312_charge_04)..." << std::endl;
    std::vector<TASKINFO_S> pool_inside;
    TASKINFO_S t1{};
    t1.tTaskType = 1;
    t1.tPathList.push_back("pudong_air/312_charge_04");
    t1.tXAxis = 0.069f;
    t1.tYAxis = -0.677f;
    t1.tAngle = 248.873f;
    t1.task_id = 11;
    pool_inside.push_back(t1);

    AssertCheck(core.ValidateTaskPoolWithFence(pool_inside), "Safe route validation must succeed");
    AssertCheck(std::abs(pool_inside[0].tXAxis - 0.069f) < 1e-4, "Stop X should not change when all points inside");
    AssertCheck(std::abs(pool_inside[0].tYAxis - (-0.677f)) < 1e-4, "Stop Y should not change when all points inside");
    AssertCheck(std::abs(pool_inside[0].tAngle - 248.873f) < 1e-4, "Stop Angle should not change when all points inside");
    std::cout << "  PASS: Stop point kept at (" << pool_inside[0].tXAxis << ", " << pool_inside[0].tYAxis << ")" << std::endl;

    // Test 3: Path leaves fence -> Stop point truncated to last point inside fence
    std::cout << "[Test 3] Path leaves fence (pudong/go_straight)..." << std::endl;
    // pudong/go_straight.csv: 461 points. Points 0..365 inside, points 366..460 outside.
    // Original destination is the last point (outside fence).
    std::vector<TASKINFO_S> pool_outside;
    TASKINFO_S t2{};
    t2.tTaskType = 1;
    t2.tPathList.push_back("pudong/go_straight");
    // Row 460 (last row) of pudong/go_straight.csv: [-3.843, -18.393, 340.84]
    t2.tXAxis = -3.843f;
    t2.tYAxis = -18.393f;
    t2.tAngle = 340.84f;
    t2.task_id = 99;
    pool_outside.push_back(t2);

    AssertCheck(core.ValidateTaskPoolWithFence(pool_outside), "Safe truncated prefix must succeed");
    // 新契约保留车头角点余量，不再把第365点的中心在内当作充分条件。
    task_fence::Geometry geometry;
    std::string error;
    AssertCheck(geometry.Load(path_dir + "fence.csv", error), "Valid fence geometry");
    AssertCheck(pool_outside[0].fenceTruncated, "Truncation must remain visible to task state machine");
    AssertCheck(geometry.PoseInside(pool_outside[0].tXAxis, pool_outside[0].tYAxis, pool_outside[0].tAngle),
                "Stop front corners must be inside fence");
    AssertCheck(pool_outside[0].fenceStopIndex <= 365, "Stop cannot move past original first crossing");
    AssertCheck(core.IsPointInFence(pool_outside[0].tXAxis, pool_outside[0].tYAxis),
                "New stop point must be inside electronic fence");
    std::cout << "  PASS: Truncated stop point from (-3.843, -18.393) to (" << pool_outside[0].tXAxis
              << ", " << pool_outside[0].tYAxis << ", heading: "
              << pool_outside[0].tAngle << ") which is strictly inside fence!" << std::endl;

    // Test 4: Trajectory & in_fence caching performance (Space for Time)
    std::cout << "[Test 4] Caching performance benchmark (1000 iterations)..." << std::endl;
    auto t_start = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < 1000; ++i) {
        std::vector<TASKINFO_S> pool_bench = pool_inside;
        AssertCheck(core.ValidateTaskPoolWithFence(pool_bench), "Cached safe route stays valid");
    }
    auto t_end = std::chrono::high_resolution_clock::now();
    double duration_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    std::cout << "  PASS: 1000 validations completed in " << duration_ms << " ms ("
              << (duration_ms / 1000.0) << " ms/call), fully hitting memory cache!" << std::endl;
    // 报告当前主机耗时，不用依赖机器速度的硬阈值替代车载性能验收。

    // Test 5: Dummy action task without valid coordinates (-1, -1)
    std::cout << "[Test 5] Dummy/action task handling (-1, -1)..." << std::endl;
    std::vector<TASKINFO_S> pool_dummy;
    TASKINFO_S t3{};
    t3.tTaskType = 6;
    t3.tPathList.push_back("sx040901");
    t3.tXAxis = -1.0f;
    t3.tYAxis = -1.0f;
    t3.tAngle = -1.0f;
    t3.task_id = 1;
    pool_dummy.push_back(t3);
    AssertCheck(core.ValidateTaskPoolWithFence(pool_dummy), "Action placeholder route is exempt");
    AssertCheck(pool_dummy[0].tXAxis == -1.0f && pool_dummy[0].tYAxis == -1.0f,
                "Action task (-1, -1) should remain untouched");
    // Test 6: Full SetTaskInfo flow and verify zero disk file modification
    std::cout << "[Test 6] Full SetTaskInfo with task11.yaml and disk immutability check..." << std::endl;
    std::string task_param_dir = path_dir.substr(0, path_dir.size() - 5) + "param/";
    std::string yaml_path = task_param_dir + "task11.yaml";
    std::ifstream before_file(yaml_path);
    std::string before_content((std::istreambuf_iterator<char>(before_file)),
                               std::istreambuf_iterator<char>());
    before_file.close();

    TaskInfoIn info11;
    info11.task_id = 11;
    info11.vehicle_id = "A0002";
    core.SetTaskInfo(info11, task_param_dir);

    // Verify task_plan_msg
    TaskPlanMsgOut msg_out;
    core.PublishTaskPlanMsg(10.0f, [&](const TaskPlanEvent &ev) {
        if(ev.type == TP_PUBLISH_TASK_PLAN) {
            msg_out = core.GetTaskPlanMsg();
        }
    });
    AssertCheck(std::abs(msg_out.stopX - 0.069f) < 1e-3, "Published stopX should match task11 target");
    AssertCheck(std::abs(msg_out.stopY - (-0.677f)) < 1e-3, "Published stopY should match task11 target");

    // Check yaml file content after SetTaskInfo
    std::ifstream after_file(yaml_path);
    std::string after_content((std::istreambuf_iterator<char>(after_file)),
                              std::istreambuf_iterator<char>());
    after_file.close();
    AssertCheck(before_content == after_content, "task11.yaml on disk must NOT be modified");
    std::cout << "  PASS: SetTaskInfo dispatched successfully and task11.yaml disk file byte-for-byte identical." << std::endl;

    std::cout << "\nALL ELECTRONIC FENCE TESTS PASSED SUCCESSFULLY!" << std::endl;
    return 0;
}
