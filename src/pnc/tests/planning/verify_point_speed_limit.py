#!/usr/bin/env python3
"""用真实规划源码和消息桩验证单点区域限速及既有安全覆盖。"""
import argparse
import concurrent.futures
import hashlib
import json
from pathlib import Path

from generate_stubs import generate
from verify import run, sources, trace_header


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    tests = Path(__file__).resolve().parent
    pnc = tests.parents[1]
    work = args.output.resolve()
    work.mkdir(parents=True, exist_ok=True)
    generate(pnc, work / "stubs")
    trace_header(pnc, work)
    fixtures = work / "fixtures"
    fixtures.mkdir(exist_ok=True)
    paths = [work / "stubs", work, pnc / "include", pnc / "src", pnc / "src/robot_path_plan"]
    paths += [pnc / "3rd-party" / lib / "include" for lib in ("osqp", "qpoases", "proj4")]
    flags = ["g++", "-std=c++14", "-O0", "-g", "-include", "cstdint", "-include", "array"]
    flags += ["-I" + str(path) for path in paths]
    libraries = ["-lyaml-cpp", str(pnc / "3rd-party/osqp/lib/libosqp.so"),
                 "-Wl,-rpath," + str(pnc / "3rd-party/osqp/lib")]
    units = sources(pnc, "path_plan_comply") + sources(pnc, "pubalgor") + sources(pnc, "spline")
    units.append("src/robot_path_plan/path_plan_node.cpp")

    def compile_unit(source):
        name = source.replace("/", "_")
        target = work / (name + ".o")
        extra = ["-Dmain=PlanningNodeMain"] if source.endswith("path_plan_node.cpp") else []
        run(flags + extra + ["-c", str(pnc / source), "-o", str(target)], work / (name + ".log"))
        return target

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        objects = list(pool.map(compile_unit, units))
    print("Compiled", len(objects), "planning and shared algorithm units", flush=True)
    results = {}
    for name in ("point_speed_limit", "gantry_safety", "perception_safety_integration"):
        target = work / (name + ".o")
        run(flags + ["-c", str(tests / (name + ".cpp")), "-o", str(target)], work / (name + "_compile.log"))
        binary = work / name
        run(["g++", str(target), *map(str, objects), *libraries, "-o", str(binary)], work / (name + "_link.log"))
        extra = [str(fixtures)] if name == "point_speed_limit" else []
        log = work / (name + ".log")
        run([str(binary), str(pnc), *extra], log)
        result = log.read_text().splitlines()[-1]
        print(result, flush=True)
        results[name] = result
    results["transport"] = "generated ROS stubs"
    results["language"] = "C++14 (host Boost requirement); production remains C++11"
    results["source_sha256"] = {
        str(path.relative_to(pnc)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in (pnc / "src/robot_path_plan").rglob("*")
        if path.suffix in (".h", ".cpp", ".inc")
    }
    (work / "result.json").write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n")
    print("PASS:", work / "result.json", flush=True)


if __name__ == "__main__":
    main()
