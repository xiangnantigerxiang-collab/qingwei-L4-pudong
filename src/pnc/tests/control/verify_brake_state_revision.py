#!/usr/bin/env python3
"""以真实C++11源码和消息桩验收D制动业务状态；可指定冻结基线重现旧失败。"""
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

CASES = ["manual_n_hold", "manual_n_empty", "manual_n_moving_hold", "auto_n_unanswered",
         "auto_n_delayed_response", "clock_rewind", "unknown_cancelled", "unknown_late_response",
         "unknown_safety", "failed_deceleration", "cancelled_still_overspeed",
         "observation_navigation_gap", "failure_new_navigation", "small_unanswered",
         "cancelled_still_overspeed_late_response", "saturation_new_navigation",
         "repeated_unanswered_safety", "ordinary_brief_input_gap",
         "reverse_ordinary_unknown", "reverse_ordinary_failed_deceleration"]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--control-dir", type=Path, default=PNC / "src/robot_control")
    parser.add_argument("--cases", nargs="+", choices=CASES, default=CASES)
    parser.add_argument("--allow-failures", action="store_true", help="仅用于保存旧版反例；JSON仍完整报告失败")
    args = parser.parse_args()
    work = args.output.resolve()
    control = args.control_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)
    generate(PNC, work / "stubs")
    flags = ["g++", "-std=c++11", "-O0", "-Wall", "-Wextra", "-include", "cstdint", "-include", "array"]
    flags += ["-I" + str(path) for path in (work / "stubs", PNC / "include", PNC / "src", PNC / "src/robot_path_plan", control)]
    names = ["robot_control/stanley_controller/stanley_controller.cpp", "robot_control/lateral_reverse_control.cpp",
             "robot_control/longitudinal_speed_control.cpp", "robot_path_plan/common/pubalgor/pubalgor.cpp",
             "robot_path_plan/common/spline/Spline.cpp"]
    sources = [PNC / "src" / name for name in names]
    tracked = sorted(control.rglob("*.cpp")) + sorted(control.rglob("*.h")) + sorted(control.rglob("*.inc")) + sources + [HERE / "brake_state_revision.cpp"]
    def hashes():
        return {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in tracked}
    before = hashes()
    commands = []
    def compile_one(source):
        obj = work / (source.stem + ".o")
        command = flags + ["-c", str(source), "-o", str(obj)]
        commands.append(command)
        with obj.with_suffix(".compile.log").open("w") as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
        return str(obj)
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        common = list(pool.map(compile_one, sources))
    comply = compile_one(control / "control_comply.cpp")
    test = compile_one(HERE / "brake_state_revision.cpp")
    binary = work / "brake_state_revision"
    command = ["g++", test, comply] + common + ["-o", str(binary)]
    commands.append(command)
    subprocess.run(command, check=True)
    cases = []
    for case in args.cases:
        run = subprocess.run([str(binary), case], text=True, capture_output=True)
        (work / (case + ".log")).write_text(run.stdout + run.stderr)
        trace = [line[6:] for line in run.stdout.splitlines() if line.startswith("TRACE,")]
        (work / (case + ".csv")).write_text("\n".join(trace) + "\n")
        summary = [line for line in run.stdout.splitlines() if line.startswith("RESULT ")]
        failures = [line for line in run.stderr.splitlines() if line.startswith("FAIL ")]
        cases.append({"case": case, "exit_code": run.returncode, "summary": summary,
                      "failed_assertions": len(failures), "first_failures": failures[:8]})
        print(case + ": " + (summary[-1] if summary else "missing result"), flush=True)
    after = hashes()
    result = {"source_unchanged_during_test": before == after, "sources": after,
              "control_dir": str(control), "cases": cases,
              "failed_cases": sum(case["exit_code"] != 0 for case in cases)}
    (work / "compile_commands.json").write_text(json.dumps(commands, indent=2) + "\n")
    (work / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    if before != after:
        raise RuntimeError("Sources changed during build/test; rerun before reporting")
    if result["failed_cases"] and not args.allow_failures:
        raise SystemExit(1)

if __name__ == "__main__":
    main()
