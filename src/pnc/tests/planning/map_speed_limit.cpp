// CSV逐行读取、逐点限速、接入路径保留限速及最终安全来源隔离。
#define main ExistingGantryTestsMain
#include "gantry_safety.cpp"
#undef main
#include <fstream>
#include <sys/stat.h>
#include <unistd.h>

void WritePathCsv(const std::string &directory, const std::string &text) {
    std::ofstream output(directory + "/map_limit.csv", std::ios::binary);
    output << text;
    output.close();
    Check(output.good(), "write isolated path CSV");
}

void TestPathCsvLoading(const std::string &pnc, const std::string &directory) {
    Fixture fixture(pnc, "forward");
    auto &plan = *fixture.plan;
    ros::param::set("path_dir", directory);
    WritePathCsv(directory, "\xEF\xBB\xBF" "100,100,90,10\r\n\r\n# comment\n"
                            "101,100,90,2.5\n102,100,90,0\n103,100,90,10");
    const auto path = plan.LoadPathFile("map_limit");
    Check(path.size() == 4, "four columns, BOM, CRLF, blank/comment, final row without newline");
    const float speeds[] = {10, 2.5, 0, 10};
    for(size_t i = 0; i < path.size(); ++i) {
        Check(path[i].x_axis == 100 + i && path[i].y_axis == 100 && path[i].heading == 90,
              "CSV columns never spill into next row geometry");
        Check(path[i].map_speed_limit == speeds[i] && path[i].dist_origin == i,
              "cache fourth-column m/s with continuous actual arc length");
        Check(path[i].z_axis == 0, "CSV speed never enters the independent geometry height field");
    }
    for(const char *bad : {"100,100,90,-1", "100,100,90,nan", "100,100,90,inf",
                           "100,100,90,1e999", "100,,90,10", "100,100,90,",
                           "100,100,90,,0", "header", "100,100"}) {
        WritePathCsv(directory, std::string("99,100,90,10\n") + bad + "\n101,100,90,10\n");
        Check(plan.LoadPathFile("map_limit").empty(), "malformed route rejects whole file without partial geometry");
    }
    WritePathCsv(directory, "");
    Check(plan.LoadPathFile("map_limit").empty(), "empty CSV returns no phantom point");
    Check(plan.LoadPathFile("missing_map_limit").empty(), "missing CSV does not dereference FILE pointer");
    WritePathCsv(directory, "100,100,90,10\n101,100,90,1\n");
    robot::task_plan_msg task;
    task.pathList = {"map_limit", "missing_map_limit", "map_limit"};
    plan.LoadTaskPaths(task);
    Check(plan.mDrivingPath.empty(), "missing segment never stitches remaining CSVs across a gap");
    task.pathList = {"map_limit", "map_limit"};
    plan.LoadTaskPaths(task);
    Check(plan.mDrivingPath.size() == 4 && plan.mDrivingPath[3].dist_origin == 3,
          "CSV join keeps exact row count and cumulative distance");
    Check(plan.mDrivingPath[1].map_speed_limit == 1 && plan.mDrivingPath[3].map_speed_limit == 1,
          "multiple CSV limits remain attached to their points");
    task.pathList = {"missing_map_limit"};
    plan.UpdateStopPoint(task);
    Check(plan.mDrivingPath.empty(), "missing operation CSV does not index an empty vector");
}

std::string ReadPathCsv(const std::string &path) {
    std::ifstream input(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

void TestSavedColumnUpgrade(const std::string &pnc, const std::string &directory) {
    Fixture fixture(pnc, "forward");
    auto &plan = *fixture.plan;
    ros::param::set("path_dir", directory);
    const std::string path = directory + "/map_limit.csv";
    struct CASE_S { const char *before; const char *after; float first_speed; };
    const CASE_S cases[] = {
        {"100,100,90\n101,100,90\n", "100,100,90,10\n101,100,90,10\n", 10},
        {"\xEF\xBB\xBF" "100,100,90\r\n\r\n# x,y,heading\r\n101,100,90",
         "\xEF\xBB\xBF" "100,100,90,10\r\n\r\n# x,y,heading\r\n101,100,90,10", 10},
        {"100,100,90,0.25\n101,100,90", "100,100,90,0.25\n101,100,90,10", 0.25f},
        {"100,100,90,0,1\r\n101,100,90,2,unused,more",
         "100,100,90,0\r\n101,100,90,2", 0},
        {"100,100,90,10,\n101,100,90,10", "100,100,90,10\n101,100,90,10", 10},
    };
    for(const auto &item : cases) {
        WritePathCsv(directory, item.before);
        Check(::chmod(path.c_str(), 0640) == 0, "set original map permissions");
        const auto loaded = plan.LoadPathFile("map_limit");
        Check(loaded.size() == 2 && loaded[0].map_speed_limit == item.first_speed,
              "actual planner loads default or existing fourth-column speed after normalization");
        Check(ReadPathCsv(path) == item.after, "planner persists exact four-column content into original file");
        struct stat saved, reloaded;
        Check(::stat(path.c_str(), &saved) == 0 && (saved.st_mode & 0777) == 0640,
              "replacement preserves original permission bits");
        Check(plan.LoadPathFile("map_limit").size() == 2, "saved four-column map can be loaded again");
        Check(::stat(path.c_str(), &reloaded) == 0 && saved.st_ino == reloaded.st_ino &&
                  saved.st_mtim.tv_sec == reloaded.st_mtim.tv_sec && saved.st_mtim.tv_nsec == reloaded.st_mtim.tv_nsec,
              "already-four-column file is not rewritten on repeated task loads");
    }
    const std::string malformed = "100,100,90\n101,,90\n";
    WritePathCsv(directory, malformed);
    Check(plan.LoadPathFile("map_limit").empty() && ReadPathCsv(path) == malformed,
          "a later invalid row prevents partial upgrade of earlier valid rows");
    const std::string legacy = "100,100,90\n101,100,90\n";
    WritePathCsv(directory, legacy);
    Check(::chmod(path.c_str(), 0440) == 0, "make legacy fixture read-only");
    const auto failed = plan.LoadPathFile("map_limit");
    Check(::chmod(path.c_str(), 0640) == 0, "restore fixture permissions");
    Check(failed.empty() && ReadPathCsv(path) == legacy,
          "write failure preserves original map and rejects unsaved driving path");
}

void TestMapSpeedPublications(const std::string &pnc) {
    for(const std::string &scenario : {"forward", "reverse", "manual", "short", "empty",
                                      "reference_unsafe", "emergency", "network", "pause"}) {
        for(float cap : {10.0f, 0.7f, 0.0f}) {
            for(bool closed : {false, true}) {
                Fixture reference(pnc, scenario), subject(pnc, scenario);
                for(PathPlanComply *plan : {reference.plan, subject.plan}) {
                    SetReferenceNearGantry(*plan);
                    plan->SetGantryState(true, !closed);
                    plan->mDrivingPath.resize(12);
                    plan->mKeypoint = 0;
                }
                subject.plan->mDrivingPath[0].map_speed_limit = cap;
                const auto expected = reference.Publish();
                auto actual = subject.Publish();
                Check(actual.path.desireSpeed == std::min(expected.path.desireSpeed, cap),
                      "map cap clamps final desired speed, including zero, without raising lower stops");
                actual.path.desireSpeed = expected.path.desireSpeed;
                Check(Trace(actual.path) == Trace(expected.path), "all other final fields and safety sources preserved");
                Check(actual.sound == expected.sound && actual.events == expected.events,
                      "map cap adds no ROS parameter access and preserves sound/light output");
                Check(Trace(subject.plan->mReferPath) == Trace(reference.plan->mReferPath),
                      "final clamp does not corrupt reference or independent safety state");
            }
        }
    }
}

void TestRetiredAreaFileIgnored(const std::string &pnc, const std::string &directory) {
    std::ifstream removed(pnc + "/path/speed_limit.csv");
    Check(!removed.is_open(), "retired area CSV is absent from production path directory");
    const std::string legacy_file = directory + "/speed_limit.csv";
    for(const char *contents : {"96.14,-405.63,0,10\n91.13,-393.80,0,10\n", "malformed\n"}) {
        std::ofstream output(legacy_file);
        output << contents;
        output.close();
        Check(output.good(), "write obsolete-file fixture outside production data");
        for(const std::string &scenario : {"forward", "reverse", "manual", "short", "empty",
                                          "reference_unsafe", "emergency", "network", "pause"}) {
            for(float cap : {10.0f, 0.25f}) {
                for(const auto &gate : {std::pair<float, float>(96.14f, -405.63f),
                                       std::pair<float, float>(91.13f, -393.80f)}) {
                    Fixture reference(pnc, scenario), subject(pnc, scenario);
                    ros::param::set("path_dir", directory);
                    const auto gear = subject.plan->mVehicleData.curGear;
                    ros::param::events().clear();
                    subject.plan->InitParameter();
                    subject.plan->mVehicleData.curGear = gear;
                    for(const auto &event : ros::param::events())
                        Check(event != "get:path_dir", "initialization no longer requests an area-file directory");
                    for(auto plan : {reference.plan, subject.plan}) {
                        plan->mNavData.xAxis = gate.first;
                        plan->mNavData.yAxis = gate.second;
                        plan->mDrivingPath.resize(1);
                        plan->mDrivingPath[0].map_speed_limit = cap;
                    }
                    const auto expected = reference.Publish();
                    const auto actual = subject.Publish();
                    Check(Trace(expected.path) == Trace(actual.path) && expected.sound == actual.sound &&
                              expected.events == actual.events,
                          "obsolete zero/invalid area file has no effect on map speed or independent safety");
                    Check(actual.path.desireSpeed <= cap, "map cap still applies at both former area centres");
                    if((scenario == "forward" || scenario == "reverse") && cap == 10)
                        Check(actual.path.desireSpeed > 0.5f, "former region no longer adds its 0.5 m/s cap");
                }
            }
        }
    }
    Check(std::remove(legacy_file.c_str()) == 0, "remove isolated retired-area fixture");
}

void TestMapProgress(const std::string &pnc, const std::string &directory) {
    Fixture fixture(pnc, "forward");
    auto &plan = *fixture.plan;
    plan.mDrivingPath.resize(100);
    for(size_t i = 0; i < plan.mDrivingPath.size(); ++i) {
        auto &point = plan.mDrivingPath[i];
        point.x_axis = 100 + i;
        point.y_axis = 100;
        point.heading = 90;
        point.dist_origin = i;
        point.map_speed_limit = 10;
    }
    plan.mDrivingPath[10].map_speed_limit = 0.7f;
    plan.mDrivingPath[11].map_speed_limit = 0;
    plan.mDrivingPath[12].map_speed_limit = 2;
    plan.mTaskPlanData.taskType = TRACKPATH;
    plan.mTaskPlanData.desireSpeed = 5;
    plan.mTaskPlanData.desireGear = GEAR_D;
    plan.mPathPlanStatus.taskExecuStatus = TASKINPROGRESS;
    plan.mPathPlanStatus.stopX = 199;
    plan.mPathPlanStatus.stopY = 100;
    plan.mStopIndex = 99;
    plan.mKeypoint = 0;
    ros::Publisher reference;
    for(int index : {9, 10, 11, 12, 13}) {
        plan.mNavData.xAxis = 100 + index;
        plan.mNavData.yAxis = 100;
        plan.PathPlanProcess();
        const float cap = plan.mDrivingPath[index].map_speed_limit;
        Check(plan.mKeypoint == index && plan.GetMapSpeedLimit() == cap, "reuse advancing nearest-path index");
        Check(plan.mDesireSpeed == std::min(5.0f, cap), "map cap reaches planning before reference speed generation");
        plan.PublishReferPath(reference);
        Check(plan.mReferPath.desireSpeed <= cap && fixture.Publish().path.desireSpeed <= cap,
              "reference and final commands respect current CSV point");
    }
    ros::param::set("path_dir", directory);
    WritePathCsv(directory, "100,100,90,0.25\n101,100,90,0.25\n");
    robot::task_plan_msg task;
    task.pathList = {"map_limit"};
    plan.LoadTaskPaths(task);
    plan.mKeypoint = 0;
    std::remove((directory + "/map_limit.csv").c_str());
    ros::param::events().clear();
    Check(plan.GetMapSpeedLimit() == 0.25 && ros::param::events().empty(),
          "cycle lookup uses cached point only, without file or parameter reads");
    plan.ResetParameter();
    Check(std::isinf(plan.GetMapSpeedLimit()), "task reset leaves no stale limit");
    plan.mDrivingPath.resize(20);
    Check(std::isinf(plan.GetMapSpeedLimit()), "new generated path does not inherit prior CSV limit");
    plan.mDrivingPath[0].map_speed_limit = std::numeric_limits<float>::quiet_NaN();
    Check(plan.GetMapSpeedLimit() == 0, "invalid cached limit cannot release speed");
    plan.mKeypoint = -1;
    Check(std::isinf(plan.GetMapSpeedLimit()), "invalid index is not dereferenced");
}

void TestConnectionLimits(const std::string &pnc, size_t zero_index) {
    Fixture reference(pnc, "forward"), subject(pnc, "forward");
    for(PathPlanComply *plan : {reference.plan, subject.plan}) {
        for(int i = 0; i < 60; ++i) {
            XYZ_COOR_S point;
            point.x_axis = 100 + i * 0.25f;
            point.y_axis = 100;
            point.heading = 90;
            plan->mDrivingPath.push_back(point);
        }
        plan->mNavData.xAxis = 100;
        plan->mNavData.yAxis = 101.8f;
        plan->mNavData.heading = 90;
        plan->mKeypoint = 0;
    }
    for(auto &point : subject.plan->mDrivingPath) point.map_speed_limit = 10;
    subject.plan->mDrivingPath[zero_index].map_speed_limit = 0;
    subject.plan->mDrivingPath.back().map_speed_limit = 2.5;
    reference.plan->ConnectVehicleToPath();
    subject.plan->ConnectVehicleToPath();
    Check(reference.plan->mDrivingPath.size() == subject.plan->mDrivingPath.size(), "limit metadata preserves spline point count");
    bool zero_retained = false;
    for(size_t i = 0; i < subject.plan->mDrivingPath.size(); ++i) {
        const auto &a = reference.plan->mDrivingPath[i];
        const auto &b = subject.plan->mDrivingPath[i];
        Check(a.x_axis == b.x_axis && a.y_axis == b.y_axis && a.heading == b.heading,
              "limit propagation does not alter generated connection geometry");
        Check(b.map_speed_limit >= 0 && b.map_speed_limit <= 10, "resampled points retain bounded source limits");
        zero_retained = zero_retained || b.map_speed_limit == 0;
    }
    Check(zero_retained, "connection and decimation preserve a one-point zero-speed restriction");
    Check(subject.plan->mDrivingPath.back().map_speed_limit == 2.5, "true endpoint retains its own limit");
}

int main(int argc, char **argv) {
    Check(argc == 3, "pnc and isolated fixture directories supplied");
    TestPathCsvLoading(argv[1], argv[2]);
    TestSavedColumnUpgrade(argv[1], argv[2]);
    TestMapSpeedPublications(argv[1]);
    TestRetiredAreaFileIgnored(argv[1], argv[2]);
    TestMapProgress(argv[1], argv[2]);
    TestConnectionLimits(argv[1], 2);
    TestConnectionLimits(argv[1], 7);
    std::cout << "PASS map speed integration: " << checks << " checks\n";
}
