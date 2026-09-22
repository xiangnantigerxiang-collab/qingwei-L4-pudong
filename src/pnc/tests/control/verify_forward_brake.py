#!/usr/bin/env python3
"""编译真实 C++11 control 节点，并运行 D 挡液压执行反馈专项。"""

import argparse
import concurrent.futures
import hashlib
import json
from pathlib import Path
import subprocess
import sys


HERE = Path(__file__).resolve().parent
PNC = HERE.parents[1]
sys.path.insert(0, str(HERE.parent / "planning"))
from generate_stubs import generate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    work = args.output.resolve()
    work.mkdir(parents=True, exist_ok=True)
    generate(PNC, work / "stubs")
    control = PNC / "src/robot_control"
    include_paths = [work / "stubs", PNC / "include", PNC / "src",
                     PNC / "src/robot_path_plan", control]
    flags = ["g++", "-std=c++11", "-O0", "-Wall", "-Wextra", "-include", "cstdint", "-include", "array"]
    flags += ["-I" + str(path) for path in include_paths]
    names = ["robot_control/stanley_controller/stanley_controller.cpp",
             "robot_control/lateral_reverse_control.cpp",
             "robot_control/longitudinal_speed_control.cpp",
             "robot_path_plan/common/pubalgor/pubalgor.cpp",
             "robot_path_plan/common/spline/Spline.cpp"]
    commands = []

    def compile_one(source, name):
        obj = work / (name + ".o")
        command = flags + ["-c", str(source), "-o", str(obj)]
        commands.append(command)
        with (work / (name + ".compile.log")).open("w") as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
        return obj

    tracked = [control / "control_comply.cpp", control / "control_comply.h",
               control / "forward_brake_control.inc", HERE / "forward_brake_feedback.cpp"]
    before = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in tracked}
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        jobs = [pool.submit(compile_one, PNC / "src" / name, Path(name).stem) for name in names]
        common = [job.result() for job in jobs]
    comply = compile_one(control / "control_comply.cpp", "control_comply")
    test = compile_one(HERE / "forward_brake_feedback.cpp", "forward_brake_feedback")
    node = compile_one(control / "control_node.cpp", "control_node")
    for main_object, name in [(test, "forward_brake_feedback"), (node, "control_node")]:
        command = ["g++", str(main_object), str(comply)] + [str(obj) for obj in common]
        command += ["-o", str(work / name)]
        commands.append(command)
        subprocess.run(command, check=True)
    (work / "compile_commands.json").write_text(json.dumps(commands, indent=2) + "\n")
    print("C++11 control_node: compiled and linked", flush=True)
    log_path = work / "forward_brake_feedback.log"
    with log_path.open("w") as log:
        result = subprocess.run([str(work / "forward_brake_feedback")],
                                stdout=log, stderr=subprocess.STDOUT)
    after = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in tracked}
    lines = log_path.read_text().splitlines()
    summary = {"exit_code": result.returncode, "source_unchanged_during_build": before == after,
               "sources": after, "result": lines[-1] if lines else "no output"}
    (work / "result.json").write_text(json.dumps(summary, indent=2) + "\n")
    if result.returncode != 0:
        print("\n".join(lines[-20:]), file=sys.stderr)
        raise SystemExit(result.returncode)
    if before != after:
        raise RuntimeError("Source changed during build; rerun to verify a consistent version")
    print(summary["result"], flush=True)


if __name__ == "__main__":
    main()
