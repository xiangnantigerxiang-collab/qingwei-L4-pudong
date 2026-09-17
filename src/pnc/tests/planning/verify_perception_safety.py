#!/usr/bin/env python3
"""验证真实观测链、连续碰撞、其他停车来源，并对比实际旧/新规划热路径。"""
import argparse
import concurrent.futures
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml

from generate_stubs import generate
from verify import sources, trace_header
from check_structure import tokens


def run(command, log):
    with log.open("w") as output:
        completed = subprocess.run(command, stdout=output, stderr=subprocess.STDOUT)
    if completed.returncode:
        raise RuntimeError(str(log) + "\n" + log.read_text()[-7000:])
    return log.read_text()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--legacy-build", type=Path, help="复用已核对相同的非热路径公共算法对象")
    args = parser.parse_args()
    tests = Path(__file__).resolve().parent
    pnc = tests.parents[1]
    work = args.output.resolve()
    work.mkdir(parents=True, exist_ok=True)
    generate(pnc, work / "stubs")
    baseline = args.baseline.resolve()
    launch = ET.parse(pnc / "launch/robot_path_plan.launch")
    configuration = "$(find robot)/param/perception_safety.yaml"
    if not any(item.get("file") == configuration for item in launch.findall("rosparam")):
        raise RuntimeError("Planning launch does not load perception safety configuration")
    parameters = yaml.safe_load((pnc / "param/perception_safety.yaml").read_text())
    parameters = parameters["/robot/planning/perception_safety"]
    if len(parameters) != 23 or any(not isinstance(value, (int, float)) for value in parameters.values()):
        raise RuntimeError("Incomplete or nonnumeric perception safety configuration")
    source_hashes = {str(path.relative_to(pnc)): hashlib.sha256(path.read_bytes()).hexdigest()
                     for folder in ("src/robot_path_plan", "src/robot_perception_convert")
                     for path in (pnc / folder).rglob("*")
                     if path.is_file() and path.suffix in (".cpp", ".h", ".inc")}

    def flags(tree):
        paths = [work / "stubs", tree / "include", tree / "src", tree / "src/robot_path_plan"]
        paths += [pnc / "3rd-party" / lib / "include" for lib in ("osqp", "qpoases", "proj4")]
        return ["g++", "-std=c++14", "-O2", "-g", "-include", "cstdint", "-include", "array"] + ["-I" + str(p) for p in paths]

    core_flags = ["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-I" + str(work / "stubs")]
    run(core_flags + ["-O2", str(tests / "perception_safety.cpp"), "-o", str(work / "core")], work / "compile_core.log")
    core_result = run([str(work / "core")], work / "core.log").strip()
    print(core_result, flush=True)
    run(core_flags + ["-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                      str(tests / "perception_safety.cpp"), "-o", str(work / "core_sanitize")], work / "compile_sanitize.log")
    sanitized = run([str(work / "core_sanitize")], work / "sanitize.log").strip()
    print("ASan/UBSan:", sanitized, flush=True)
    run(core_flags + ["-O2", "-Wno-mismatched-new-delete", str(tests / "perception_allocations.cpp"),
                      "-o", str(work / "allocations")], work / "compile_allocations.log")
    allocations = run([str(work / "allocations")], work / "allocations.log").strip()
    print(allocations, flush=True)

    common_sources = sources(pnc, "path_plan_comply")[1:] + sources(pnc, "pubalgor") + sources(pnc, "spline")
    def common_object(source):
        if tokens((pnc / source).read_text()) != tokens((baseline / source).read_text()):
            raise RuntimeError("Shared algorithm changed: " + source)
        name = source.replace("/", "_")
        if args.legacy_build and "collision_detection.cpp" not in source:
            target = args.legacy_build / (name + ".o")
            if not target.exists():
                raise RuntimeError("Missing shared object: " + str(target))
            return target
        target = work / (name + ".o")
        run(flags(pnc) + ["-c", str(pnc / source), "-o", str(target)], work / (name + ".log"))
        return target
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        common = list(pool.map(common_object, common_sources))
    libraries = ["-lyaml-cpp", str(pnc / "3rd-party/osqp/lib/libosqp.so"),
                 "-Wl,-rpath," + str(pnc / "3rd-party/osqp/lib")]
    for name, tree in (("before", baseline), ("after", pnc)):
        folder = work / name
        folder.mkdir(exist_ok=True)
        trace_header(tree, folder)
        comply = folder / "path_plan_comply.o"
        run(flags(tree) + ["-c", str(tree / "src/robot_path_plan/path_plan_comply.cpp"), "-o", str(comply)], folder / "compile.log")
        extra = ["-DWITH_TRACKED_PERCEPTION"] if name == "after" else []
        run(flags(tree) + ["-I" + str(folder), *extra, "-c", str(tests / "perception_benchmark.cpp"),
                          "-o", str(folder / "benchmark.o")], folder / "compile_benchmark.log")
        run(["g++", str(folder / "benchmark.o"), str(comply), *map(str, common), *libraries,
             "-o", str(folder / "benchmark")], folder / "link_benchmark.log")

    after = work / "after"
    node = after / "node.o"
    run(flags(pnc) + ["-Dmain=PlanningNodeMain", "-c", str(pnc / "src/robot_path_plan/path_plan_node.cpp"),
                      "-o", str(node)], after / "compile_node.log")
    run(flags(pnc) + ["-I" + str(after), "-c", str(tests / "perception_safety_integration.cpp"),
                      "-o", str(after / "integration.o")], after / "compile_integration.log")
    run(["g++", str(after / "integration.o"), str(after / "path_plan_comply.o"), str(node),
         *map(str, common), *libraries, "-o", str(after / "integration")], after / "link_integration.log")
    integration = run([str(after / "integration"), str(pnc)], after / "integration.log").splitlines()[-1]
    print(integration, flush=True)
    legacy = {}
    if args.legacy_build:
        # 使用最终 comply/node 对象重跑原业务及闸机检查；复用同一 fixture 路径以便逐字节比较。
        run(flags(pnc) + ["-DREFACTORED", "-I" + str(after), "-c", str(tests / "regression.cpp"),
                          "-o", str(after / "regression.o")], after / "compile_regression.log")
        run(["g++", str(after / "regression.o"), str(after / "path_plan_comply.o"),
             *map(str, common), *libraries, "-o", str(after / "regression")], after / "link_regression.log")
        trace = after / "trace.txt"
        run([str(after / "regression"), str(pnc), str(trace), str(args.legacy_build / "fixtures")],
            after / "regression.log")
        original = (args.legacy_build / "before/trace.txt").read_bytes()
        if trace.read_bytes() != original:
            raise RuntimeError("Legacy behavior differs from baseline: " + str(trace))
        run(flags(pnc) + ["-I" + str(after), "-c", str(tests / "gantry_safety.cpp"),
                          "-o", str(after / "gantry.o")], after / "compile_gantry.log")
        run(["g++", str(after / "gantry.o"), str(after / "path_plan_comply.o"), str(node),
             *map(str, common), *libraries, "-o", str(after / "gantry")], after / "link_gantry.log")
        gantry = run([str(after / "gantry"), str(pnc)], after / "gantry.log").splitlines()[-1]
        legacy = {"snapshots": original.count(b"STATE "), "trace_sha256": hashlib.sha256(original).hexdigest(),
                  "gantry": gantry}
        print("Legacy:", legacy, flush=True)
    benchmarks = {}
    # 顺序交替跑，避免基准相互争用 CPU。
    for repeat in range(3):
        for name in ("before", "after"):
            result = run([str(work / name / "benchmark")], work / name / ("benchmark_" + str(repeat) + ".log"))
            benchmarks.setdefault(name, []).append(result.strip().splitlines())
    after_hashes = {name: hashlib.sha256((pnc / name).read_bytes()).hexdigest() for name in source_hashes}
    if source_hashes != after_hashes:
        raise RuntimeError("Production source changed during verification; rerun affected checks")
    report = {"core": core_result, "sanitizers": sanitized, "integration": integration, "allocations": allocations,
              "core_language": "C++11", "planning_language": "C++14 (host Boost requirement)",
              "transport": "generated ROS stubs", "parameters": parameters, "legacy": legacy,
              "benchmarks": benchmarks, "source_sha256": source_hashes}
    (work / "result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print("PASS: " + str(work / "result.json"), flush=True)


if __name__ == "__main__":
    main()
