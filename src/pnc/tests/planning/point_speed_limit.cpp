// 覆盖 CSV 加载、圆形边界和最终发布，复用真实规划输出夹具。
#define main ExistingGantryTestsMain
#include "gantry_safety.cpp"
#undef main
#include <fstream>
#include <limits>

void WriteLimits(const std::string &directory, const std::string &contents) {
    std::ofstream output(directory + "/speed_limit.csv", std::ios::binary);
    output << contents;
    output.close();
    Check(output.good(), "write isolated test CSV");
}

void LoadLimits(PathPlanComply &plan, const std::string &directory, const std::string &contents) {
    WriteLimits(directory, contents);
    ros::param::set("path_dir", directory);
    plan.LoadPointSpeedLimits();
}

void TestLoading(const std::string &pnc, const std::string &directory) {
    Fixture fixture(pnc, "forward");
    PathPlanComply &plan = *fixture.plan;
    WriteLimits(directory, "96.14,-405.63,0.5\r\n91.13,-393.80,0.5\r\n");
    ros::param::set("path_dir", directory);
    plan.InitParameter();
    Check(plan.mPointSpeedLimits.size() == 2, "initialization loads CRLF rows without trailing path separator");
    Check(plan.mPointSpeedLimits[0].x == 96.14 && plan.mPointSpeedLimits[0].y == -405.63 &&
              plan.mPointSpeedLimits[0].speed_limit == 0.5,
          "CSV preserves navigation coordinates and m/s");
    ros::param::set("path_dir", directory + "/");
    plan.LoadPointSpeedLimits();
    Check(plan.mPointSpeedLimits.size() == 2, "path with trailing separator loads equally");

    LoadLimits(plan, directory,
               "\r\n \t\r\n100,100,0.5\n1 2 3\n1,2,-1\n1,2,nan\nnan,2,1\n"
               "1,inf,1\n1,2,inf\n1,2,1e999\n1,2\n1,2,3,4\n1,2,3junk\n"
               " 101 , 102 , 0 \r\n103,104,1.25");
    Check(plan.mPointSpeedLimits.size() == 3, "skip blank, malformed, nonfinite and negative-speed rows");
    Check(plan.mPointSpeedLimits[1].speed_limit == 0 && plan.mPointSpeedLimits[2].speed_limit == 1.25,
          "zero, surrounding spaces and final row without newline are valid");

    LoadLimits(plan, directory, "100,100,0.5\n");
    WriteLimits(directory, "");
    plan.mNavData.xAxis = plan.mNavData.yAxis = 100;
    plan.mPlanPath.desireSpeed = 3;
    ros::param::events().clear();
    plan.ApplyPointSpeedLimit();
    Check(plan.mPlanPath.desireSpeed == 0.5 && ros::param::events().empty(),
          "running clamp uses cached file and does not access ROS parameters");
    plan.LoadPointSpeedLimits();
    Check(plan.mPointSpeedLimits.empty(), "loading an empty file clears previous configuration");
    plan.mPlanPath.desireSpeed = 3;
    plan.ApplyPointSpeedLimit();
    Check(plan.mPlanPath.desireSpeed == 3, "empty file adds no speed constraint");
    LoadLimits(plan, directory, "100,100,0.5\n");
    Check(std::remove((directory + "/speed_limit.csv").c_str()) == 0, "remove isolated test CSV");
    plan.LoadPointSpeedLimits();
    Check(plan.mPointSpeedLimits.empty(), "missing optional file adds no constraint");
    ros::param::values().erase("path_dir");
    plan.LoadPointSpeedLimits();
    Check(plan.mPointSpeedLimits.empty(), "missing path parameter adds no constraint");
}

void TestRadius(const std::string &pnc, const std::string &directory) {
    Fixture fixture(pnc, "forward");
    PathPlanComply &plan = *fixture.plan;
    LoadLimits(plan, directory, "100,100,0.5\n");
    struct CASE_S {
        float x, y, expected;
    };
    const CASE_S cases[] = {{100, 100, 0.5}, {105, 100, 0.5}, {95, 100, 0.5}, {100, 105, 0.5}, {100, 95, 0.5}, {103, 104, 0.5}, {97, 96, 0.5}, {104.999f, 100, 0.5}, {105.001f, 100, 3}, {100, 105.001f, 3}, {105, 105, 3}};
    for(const auto &item : cases) {
        plan.mNavData.xAxis = item.x;
        plan.mNavData.yAxis = item.y;
        plan.mPlanPath.desireSpeed = 3;
        plan.ApplyPointSpeedLimit();
        Check(plan.mPlanPath.desireSpeed == item.expected, "inclusive 5 m circle, not square");
    }
    for(const std::string &rows : {"100,100,0.8\n100,103,0.25\n100,120,0\n100,100,4\n",
                                   "100,100,4\n100,120,0\n100,103,0.25\n100,100,0.8\n"}) {
        LoadLimits(plan, directory, rows);
        plan.mNavData.xAxis = plan.mNavData.yAxis = 100;
        for(float speed : {3.0f, 0.125f, 0.0f}) {
            plan.mPlanPath.desireSpeed = speed;
            plan.ApplyPointSpeedLimit();
            Check(plan.mPlanPath.desireSpeed == std::min(speed, 0.25f),
                  "overlapping zones take minimum independent of order and preserve lower or zero target");
        }
    }
    plan.mNavData.xAxis = std::numeric_limits<float>::quiet_NaN();
    plan.mPlanPath.desireSpeed = 3;
    plan.ApplyPointSpeedLimit();
    Check(plan.mPlanPath.desireSpeed == 3, "invalid navigation does not match a zone");
}

void TestPublications(const std::string &pnc, const std::string &directory) {
    for(const std::string &scenario : {"forward", "reverse", "manual", "short", "empty", "startup",
                                       "reference_unsafe", "emergency", "network", "pause", "waiting",
                                       "manual_retain_unsafe", "short_retain_unsafe"}) {
        for(float limit : {0.5f, 0.0f}) {
            for(bool closed : {false, true}) {
                Fixture reference(pnc, scenario);
                Fixture subject(pnc, scenario);
                reference.plan->mPointSpeedLimits.clear();
                LoadLimits(*subject.plan, directory, "100,100," + std::to_string(limit) + "\n0,0," + std::to_string(limit) + "\n");
                for(PathPlanComply *plan : {reference.plan, subject.plan}) {
                    plan->SetGantryState(true, !closed);
                    if(scenario == "waiting") plan->mNavData.xAxis = plan->mNavData.yAxis = 0;
                }
                Publication expected = reference.Publish();
                Publication actual = subject.Publish();
                expected.path.desireSpeed = std::min(expected.path.desireSpeed, limit);
                const std::string name = scenario + (closed ? " closed" : " open") + std::to_string(limit);
                Check(Trace(actual.path) == Trace(expected.path), name + ": final message only clamps desireSpeed");
                Check(actual.path_count == 1 && actual.sound_count == expected.sound_count &&
                          actual.sound == expected.sound && actual.events == expected.events,
                      name + ": other publications and parameter events unchanged");
                Check(Trace(subject.plan->mReferPath) == Trace(reference.plan->mReferPath) &&
                          Trace(subject.plan->mPathPlanStatus) == Trace(reference.plan->mPathPlanStatus) &&
                          subject.plan->mPlanPath.safety == reference.plan->mPlanPath.safety,
                      name + ": reference, task status and internal safety unchanged");
                if(scenario == "emergency" || scenario == "network" || scenario == "pause" ||
                   scenario == "waiting" || scenario == "startup" || scenario == "manual" ||
                   scenario == "short" || scenario == "empty") {
                    Check(actual.path.desireSpeed == 0, name + ": existing stop remains zero");
                }
            }
        }
    }

    Fixture fixture(pnc, "forward");
    LoadLimits(*fixture.plan, directory, "100,100,0.5\n");
    Check(fixture.Publish().path.desireSpeed == 0.5, "cap is applied after acceleration smoothing");
    fixture.plan->mReferPath.desireSpeed = 0.25;
    Check(fixture.Publish().path.desireSpeed == 0.25, "existing lower planning target wins");
    fixture.plan->mReferPath.desireSpeed = 3;
    fixture.plan->mNavData.xAxis = 106;
    const float outside = fixture.Publish().path.desireSpeed;
    Check(outside > 0.5, "leaving zone restores normal speed calculation without a latched cap");
    LoadLimits(*fixture.plan, directory, "");
    fixture.plan->mNavData.xAxis = 100;
    Check(fixture.Publish().path.desireSpeed == outside, "empty CSV leaves normal final speed unchanged");
}

int main(int argc, char **argv) {
    Check(argc == 3, "pnc directory and isolated test directory supplied");
    TestLoading(argv[1], argv[2]);
    TestRadius(argv[1], argv[2]);
    TestPublications(argv[1], argv[2]);
    std::cout << "PASS point speed limit: " << checks << " checks\n";
}
