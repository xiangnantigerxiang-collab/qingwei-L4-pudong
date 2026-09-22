#!/usr/bin/env python3
"""复用刚完成的规划全量桩构建，独立验证 CAN/导航矛盾输入和导航断流。

先运行 tests/planning/verify_map_speed_limit.py；--planning-build 指向其输出目录。
所有生成文件和测试矩形写入 --output，业务 CSV 和参数保持只读。
"""
import argparse
import json
import math
import re
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PNC = HERE.parents[1]
sys.path.insert(0, str(HERE.parent / "planning"))
from verify import sources


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planning-build", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    work = args.output.resolve()
    work.mkdir(parents=True, exist_ok=True)
    planning = args.planning_build.resolve()
    build = planning / "build"
    matrix = json.loads((build / "result.json").read_text())
    objects = matrix["objects"]
    # 防止类布局变更后误用旧 .o；要求全量构建新于所有当前生产源码/头文件。
    inputs = [path for folder in (PNC / "src", PNC / "include", PNC / "msg")
              for path in folder.rglob("*") if path.suffix in (".cpp", ".cc", ".h", ".hpp", ".inc", ".msg")]
    newest = max(path.stat().st_mtime for path in inputs)
    if any(Path(path).stat().st_mtime < newest for path in objects.values()):
        raise RuntimeError("生产源码晚于公共对象，请先重新运行规划全量桩构建")

    def run(command, name):
        result = subprocess.run(list(map(str, command)), text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        (work / (name + ".log")).write_text(result.stdout)
        if result.returncode:
            raise RuntimeError(f"{name} failed; see {work / (name + '.log')}")
        return result.stdout

    includes = [build / "stubs", work, planning, HERE.parent / "planning", PNC / "include", PNC / "src",
                PNC / "src/robot_path_plan", PNC / "src/robot_control"]
    includes += [PNC / "3rd-party" / name / "include" for name in ("osqp", "qpoases", "proj4")]
    flags = ["g++", "-std=c++14", "-O0", "-g", "-Wall", "-Wextra", "-include", "cstdint", "-include", "array"]
    flags += ["-I" + str(path) for path in includes]
    libraries = ["-lyaml-cpp", str(PNC / "3rd-party/osqp/lib/libosqp.so"),
                 "-Wl,-rpath," + str(PNC / "3rd-party/osqp/lib")]
    units = sources(PNC, "path_plan_comply") + sources(PNC, "pubalgor") + sources(PNC, "spline")
    run(flags + [HERE / "planning.cpp", planning / "gantry_node.o"] + [objects[unit] for unit in units] +
        libraries + ["-o", work / "planning"], "planning_compile")
    results = {"planning": run([work / "planning", PNC], "planning_result").splitlines()[-1]}

    header = (PNC / "src/robot_control/control_comply.h").read_text()
    (work / "control_access.h").write_text("\n".join(re.findall(r"^#include.*$", header, re.M)) +
        '\n#define private public\n#include "control_comply.h"\n#undef private\n')
    units = [unit for unit in objects if unit.startswith("src/robot_control/") and not unit.endswith("control_node.cpp")]
    units += sources(PNC, "pubalgor") + sources(PNC, "spline") + ["src/robot_can_comm/can_comm_comply.cpp"]
    run(flags + [HERE / "control.cpp"] + [objects[unit] for unit in units] +
        libraries + ["-o", work / "control"], "control_compile")

    def trace(mode, can, nav):
        output = run([work / "control", mode, can, nav], f"control_{mode}_{can}_{nav}")
        return [[float(value) for value in line.split()[1:]] for line in output.splitlines() if line.startswith("TRACE ")]

    def same(a, b):
        return len(a) == len(b) and all(len(x) == len(y) and all(
            v == w or (math.isnan(v) and math.isnan(w)) for v, w in zip(x, y)) for x, y in zip(a, b))

    comparisons = 0
    for mode, nav in (("forward", 1), ("reverse", 1), ("brake", 1.6)):
        expected = trace(mode, nav, nav)
        assert len(expected) == 12
        for can in (-100, 0, 0.2, 99, "nan", "inf"):
            assert same(expected, trace(mode, can, nav)), (mode, can, "CAN affected full control output")
            comparisons += 1
    brake = trace("brake", 99, 1.6)
    assert all(row[0:2] == [15, 0] for row in brake[:4]) and brake[4][0:2] == [0, 30]
    assert all(row[0:2] == [15, 0] for row in trace("brake", 99, 1)), "navigation must govern braking"
    results["control"] = {"contradictory_CAN_full_trace_comparisons": comparisons,
                          "published_messages_compared": comparisons * 12,
                          "navigation_brake_expectations": 2}
    results["control_freshness"] = run([work / "control", "freshness", 99, 1], "control_freshness").splitlines()[-1]
    results["invalid_navigation_control"] = {}
    for mode in ("invalid_forward", "invalid_reverse"):
        for speed in ("nan", "inf", "-inf"):
            name = mode + "_" + speed
            results["invalid_navigation_control"][name] = run(
                [work / "control", mode, 99, speed], name).splitlines()[-1]

    stubs = work / "ultra_stubs"
    shutil.copytree(build / "stubs", stubs, dirs_exist_ok=True)
    ros_header = stubs / "ros/ros.h"
    source = ros_header.read_text().replace("struct NodeHandle {", """inline std::vector<std::string>& testSubscriptions() {
    static std::vector<std::string> topics; return topics;
}
struct NodeHandle {""").replace(
        "template<class... T> Subscriber subscribe(T&&...) { return Subscriber(); }",
        "template<class... T> Subscriber subscribe(const std::string& topic, T&&...) { testSubscriptions().push_back(topic); return Subscriber(); }")
    source += "\nnamespace ros { namespace init_options { const unsigned NoSigintHandler = 1; }\ninline void init(int, char**, const char*, unsigned) {}\ninline void shutdown() {}\n}\n"
    ros_header.write_text(source)
    ultra = PNC.parent / "ultra_command/src"
    run(["g++", "-std=c++11", "-Wall", "-Wextra", "-I" + str(stubs), "-I" + str(ultra),
         HERE / "ultra.cpp", ultra / "ultra_command_comply.cpp", "-o", work / "ultra"], "ultra_compile")
    fixtures = work / "fixtures/pudong_air"
    fixtures.mkdir(parents=True, exist_ok=True)
    for name in ("left1", "left2", "right"):
        (fixtures / (name + ".csv")).write_text("0,0,1\n10,0,1\n10,10,1\n0,10,1\n")
    results["ultra"] = run([work / "ultra", fixtures.parent], "ultra_result").splitlines()[-1]
    (work / "result.json").write_text(json.dumps(results, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(results, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
