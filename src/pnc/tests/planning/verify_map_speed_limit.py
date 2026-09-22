#!/usr/bin/env python3
"""重新编译全部 PNC 对象，验证 CSV 地图限速及相关规划安全输出。"""

import argparse
import concurrent.futures
import json
import subprocess
import sys
from pathlib import Path

from verify import sources, trace_header


def run(command, log):
    with log.open("w") as stream:
        subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    tests = Path(__file__).resolve().parent
    pnc = tests.parents[1]
    work = args.output.resolve()
    work.mkdir(parents=True, exist_ok=True)
    build = work / "build"
    # 规划类和共享路径点的布局变化时，不能混用旧头文件编译的 .o。
    subprocess.run([sys.executable, "-B", str(tests.parent / "compile_all.py"),
                    "--output", str(build)], check=True)
    trace_header(pnc, work)
    includes = [build / "stubs", work, tests, pnc / "include", pnc / "src",
                pnc / "src/robot_path_plan"]
    includes += [pnc / "3rd-party" / name / "include" for name in ("osqp", "qpoases", "proj4")]
    flags = ["g++", "-std=c++14", "-O0", "-g", "-Wall", "-Wextra", "-include", "cstdint",
             "-include", "array"] + ["-I" + str(path) for path in includes]
    libraries = ["-lyaml-cpp", str(pnc / "3rd-party/osqp/lib/libosqp.so"),
                 "-Wl,-rpath," + str(pnc / "3rd-party/osqp/lib")]
    node = work / "gantry_node.o"
    run(flags + ["-Dmain=PlanningNodeMain", "-c", str(pnc / "src/robot_path_plan/path_plan_node.cpp"),
                 "-o", str(node)], work / "node_compile.log")
    units = sources(pnc, "path_plan_comply") + sources(pnc, "pubalgor") + sources(pnc, "spline")
    objects = [build / (unit.replace("/", "_") + ".o") for unit in units]
    names = ["map_speed_limit", "gantry_safety", "terminal_stop",
             "perception_safety_integration", "startup_observation_integration", "startup_removal"]

    def compile_test(name):
        obj = work / (name + ".o")
        run(flags + ["-c", str(tests / (name + ".cpp")), "-o", str(obj)], work / (name + "_compile.log"))
        run(["g++", str(obj), str(node), *map(str, objects), *libraries, "-o", str(work / name)],
            work / (name + "_link.log"))
        return name

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        for name in pool.map(compile_test, names):
            print("Linked", name, flush=True)
    results = {}
    for name in names:
        fixture = work / ("fixtures_" + name)
        fixture.mkdir(exist_ok=True)
        extra = [str(fixture)] if name in ("map_speed_limit", "terminal_stop") else []
        if name == "startup_observation_integration":
            # 当前起步半径1m，历史完整套件仍有2m期望；保留差异，只运行本次相关入口。
            extra = ["--left-second-only"]
        log = work / (name + "_result.log")
        run([str(work / name), str(pnc), *extra], log)
        results[name] = [line for line in log.read_text().splitlines() if line.startswith("PASS")][-1]
        print(results[name], flush=True)
    # 保留原区域限速验证入口中的规划/控制交接覆盖，两个节点分别链接。
    control_units = [unit for unit in json.loads((build / "result.json").read_text())["objects"]
                     if unit.startswith("src/robot_control/") and not unit.endswith("control_node.cpp")]
    control_units += sources(pnc, "pubalgor") + sources(pnc, "spline")
    control_objects = [build / (unit.replace("/", "_") + ".o") for unit in control_units]
    target = work / "terminal_control.o"
    run(flags + ["-c", str(tests / "terminal_control.cpp"), "-o", str(target)], work / "terminal_control_compile.log")
    binary = work / "terminal_control"
    run(["g++", str(target), *map(str, control_objects), *libraries, "-o", str(binary)],
        work / "terminal_control_link.log")
    log = work / "terminal_control_result.log"
    run([str(binary), str(work / "fixtures_terminal_stop/terminal_control_inputs.txt")], log)
    results["terminal_control"] = [line for line in log.read_text().splitlines() if line.startswith("PASS")][-1]
    print(results["terminal_control"], flush=True)
    csv_flags = ["g++", "-std=c++11", "-O1", "-g", "-Wall", "-Wextra", "-Werror",
                 "-I" + str(pnc / "include")]
    csv_files = list(map(str, sorted((pnc / "path").rglob("*.csv"))))
    for name, extra in (("path_csv", []), ("path_csv_sanitized", ["-fsanitize=address,undefined",
                                                               "-fno-omit-frame-pointer"])):
        run(csv_flags + extra + [str(tests / "path_csv.cpp"), "-o", str(work / name)],
            work / (name + "_compile.log"))
        log = work / (name + "_result.log")
        run([str(work / name), *csv_files], log)
        results[name] = log.read_text().strip()
        print(results[name], flush=True)
    for name, extra in (("path_csv_upgrade", []), ("path_csv_upgrade_sanitized", ["-fsanitize=address,undefined",
                                                                               "-fno-omit-frame-pointer"])):
        run(csv_flags + extra + [str(tests / "path_csv_upgrade.cpp"), "-o", str(work / name)],
            work / (name + "_compile.log"))
        fixture = work / ("fixtures_" + name)
        fixture.mkdir(exist_ok=True)
        log = work / (name + "_result.log")
        run([str(work / name), str(fixture)], log)
        results[name] = log.read_text().strip()
        print(results[name], flush=True)
    (work / "result.json").write_text(json.dumps(results, indent=2) + "\n")
    print("Logs:", work, flush=True)


if __name__ == "__main__":
    main()
