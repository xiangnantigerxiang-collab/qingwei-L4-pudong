#!/usr/bin/env python3
"""Compile planning with ROS stubs and compare observable traces with a baseline."""

import argparse
import concurrent.futures
import hashlib
import json
import re
import subprocess
import tempfile
from pathlib import Path

from generate_stubs import generate
from check_structure import tokens


def run(command, log):
    with log.open("w") as output:
        subprocess.run(command, stdout=output, stderr=subprocess.STDOUT, check=True)


def sources(pnc, target):
    cmake = (pnc / "CMakeLists.txt").read_text()
    block = re.search(r"add_(?:library|executable)\(" + target + r"\s+(.*?)\)", cmake, re.S)[1]
    block = re.sub(r"#[^\n]*", "", block)
    return re.findall(r"src/\S+\.cpp", block)


def trace_header(pnc, output):
    # Expose only the class under test, after its dependencies have been included.
    header = (pnc / "src/robot_path_plan/path_plan_comply.h").read_text()
    includes = re.findall(r"^#include[^\n]*", header, re.M)
    access = '\n'.join(includes) + '\n#define private public\n#include "path_plan_comply.h"\n#undef private\n'
    (output / "test_access.h").write_text(access)
    names = ["path_plan_msg", "path_plan_status", "task_plan_msg", "sound_light_msg", "object", "perception"]
    lines = ['#pragma once', '#include <iomanip>', '#include <ostream>']
    lines += ['#include "robot/' + name + '.h"' for name in names]
    lines += ['template<class T> void TraceValue(std::ostream &out, const T &value) { out << +value << ","; }',
              'inline void TraceValue(std::ostream &out, const std::string &value) { out << std::quoted(value) << ","; }',
              'inline void TraceValue(std::ostream &out, const geometry_msgs::Point &value) { TraceValue(out, value.x); TraceValue(out, value.y); TraceValue(out, value.z); }']
    lines += ['void TraceValue(std::ostream &, const robot::' + name + ' &);' for name in names]
    lines += ['template<class T> void TraceValue(std::ostream &out, const std::vector<T> &values) {',
              '    out << "["; for (const auto &value : values) TraceValue(out, value); out << "]";', '}']
    for name in names:
        lines += ['inline void TraceValue(std::ostream &out, const robot::' + name + ' &value) {']
        for raw in (pnc / "msg" / (name + ".msg")).read_text().splitlines():
            field = raw.split("#", 1)[0].strip()
            if not field or "=" in field:
                continue
            field_type, field_name = field.split()
            if field_type in ("Header", "std_msgs/Header"):
                lines += ['    TraceValue(out, value.' + field_name + '.seq);',
                          '    TraceValue(out, value.' + field_name + '.stamp.toSec());',
                          '    TraceValue(out, value.' + field_name + '.frame_id);']
            else:
                lines += ['    TraceValue(out, value.' + field_name + ');']
        lines += ['}']
    (output / "test_trace.h").write_text('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True, help="Path to the original pnc directory")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    test_dir = Path(__file__).resolve().parent
    pnc = test_dir.parents[1]
    work = args.output or Path(tempfile.mkdtemp(prefix="planning-verify-"))
    work.mkdir(parents=True, exist_ok=True)
    generate(pnc, work / "stubs")
    fixtures = work / "fixtures"
    fixtures.mkdir(exist_ok=True)
    (fixtures / "straight.csv").write_text(''.join(f"{100 + i * 0.2:.1f},100,90,0\n" for i in range(200)))
    common_sources = sources(pnc, "path_plan_comply")[1:] + sources(pnc, "pubalgor") + sources(pnc, "spline")
    # 格式统一允许空白变化；有效 token 不同仍拒绝共享对象，不能掩盖算法改动。
    for source in common_sources:
        if tokens((pnc / source).read_text()) != tokens((args.baseline / source).read_text()):
            raise RuntimeError("Algorithm changed: " + source)
    for folder in ("include", "src/robot_path_plan/common", "src/robot_path_plan/lattice",
                   "src/robot_path_plan/lattice_plan", "src/robot_path_plan/reference_line",
                   "src/robot_path_plan/speedplan", "src/robot_path_plan/trans", "src/robot_path_plan/collisioncheck"):
        for path in (pnc / folder).rglob("*.h"):
            relative = path.relative_to(pnc)
            if tokens(path.read_text()) != tokens((args.baseline / relative).read_text()):
                raise RuntimeError("Algorithm header changed: " + str(relative))

    def flags(tree):
        paths = [work / "stubs", tree / "include", tree / "src", tree / "src/robot_path_plan"]
        paths += [pnc / "3rd-party" / lib / "include" for lib in ("osqp", "qpoases", "proj4")]
        return ["g++", "-std=c++14", "-O0", "-g", "-Wall", "-Wextra", "-include", "cstdint",
                "-include", "array", "-ffunction-sections", "-fdata-sections"] + ["-I" + str(path) for path in paths]

    def compile_common(source):
        name = source.replace("/", "_")
        target = work / (name + ".o")
        run(flags(pnc) + ["-c", str(pnc / source), "-o", str(target)], work / (name + ".log"))
        return target

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        common_objects = list(pool.map(compile_common, common_sources))
    print("Compiled", len(common_objects), "shared planning/geometry algorithm units", flush=True)
    link_flags = ["-lyaml-cpp", str(pnc / "3rd-party/osqp/lib/libosqp.so"),
                  "-Wl,-rpath," + str(pnc / "3rd-party/osqp/lib")]
    reports = []
    for name, tree in (("before", args.baseline.resolve()), ("after", pnc)):
        output = work / name
        output.mkdir(exist_ok=True)
        trace_header(tree, output)
        objects = []
        for unit in ("path_plan_comply", "path_plan_node"):
            target = output / (unit + ".o")
            run(flags(tree) + ["-c", str(tree / "src/robot_path_plan" / (unit + ".cpp")), "-o", str(target)],
                output / (unit + ".log"))
            objects.append(target)
        run(["g++", *map(str, objects + common_objects), *link_flags, "-o", str(output / "path_plan_node")], output / "link_node.log")
        test_flags = ["-DREFACTORED"] if "SetCommandData" in (tree / "src/robot_path_plan/path_plan_comply.h").read_text() else []
        run(flags(tree) + test_flags + ["-I" + str(output), "-c", str(test_dir / "regression.cpp"), "-o", str(output / "regression.o")], output / "compile_test.log")
        run(["g++", str(output / "regression.o"), str(objects[0]), *map(str, common_objects), *link_flags,
             "-o", str(output / "regression")], output / "link_test.log")
        trace = output / "trace.txt"
        run([str(output / "regression"), str(pnc), str(trace), str(fixtures)], output / "run.log")
        reports.append(trace.read_bytes())
        print(name, "node linked and regression passed", flush=True)
    if reports[0] != reports[1]:
        import difflib
        diff = '\n'.join(difflib.unified_diff(reports[0].decode().splitlines(), reports[1].decode().splitlines(), fromfile="before", tofile="after"))
        (work / "trace.diff").write_text(diff)
        raise RuntimeError("Behavior differs; inspect " + str(work / "trace.diff"))
    report = {"snapshots": reports[0].count(b"STATE "), "trace_sha256": hashlib.sha256(reports[0]).hexdigest(),
              "common_translation_units": len(common_objects), "baseline": str(args.baseline.resolve())}
    if "SetGantryState" in (pnc / "src/robot_path_plan/path_plan_comply.h").read_text():
        output = work / "after"
        # 重命名节点入口以调用真实 ROS 回调；消息仍由 gantry_detect/msg 生成。
        node = output / "gantry_node.o"
        test = output / "gantry_safety.o"
        run(flags(pnc) + ["-Dmain=PlanningNodeMain", "-c",
                         str(pnc / "src/robot_path_plan/path_plan_node.cpp"), "-o", str(node)],
            output / "compile_gantry_node.log")
        run(flags(pnc) + ["-I" + str(output), "-c", str(test_dir / "gantry_safety.cpp"),
                         "-o", str(test)], output / "compile_gantry_test.log")
        binary = output / "gantry_safety"
        run(["g++", str(test), str(node), str(output / "path_plan_comply.o"),
             *map(str, common_objects), *link_flags, "-o", str(binary)], output / "link_gantry_test.log")
        run([str(binary), str(pnc)], output / "run_gantry_test.log")
        report["gantry_checks"] = int(re.search(r"PASS: (\d+) gantry checks",
                                               (output / "run_gantry_test.log").read_text())[1])
    (work / "result.json").write_text(json.dumps(report, indent=2) + '\n')
    print("PASS: identical published messages, parameter events and state snapshots:", report, flush=True)
    print("Logs:", work)


if __name__ == "__main__":
    main()
