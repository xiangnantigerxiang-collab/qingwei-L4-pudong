#!/usr/bin/env python3
"""独立保存旧R/N基线，逐字段对照D制动改动后的非D输出；不改生产源码或旧测试。"""

import argparse
import concurrent.futures
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PNC = HERE.parents[1]
sys.path.insert(0, str(HERE.parent / "planning"))
from generate_stubs import generate


FIELDS = ["biaDistance", "biaAngle", "preCurve", "preAngleDev", "desireSpeed", "desireAcc",
          "throttlePercent", "brakePercent", "wheelAngle", "vehicleSpeed", "remoteEnable",
          "bypassProcessing", "faultCode"]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def scenarios():
    result = []
    # 常规标定、正负目标、旧超速阈值、左右曲线和零速边界。
    for gear in (3, 2):
        for curve in (-0.01, 0.0, 0.01):
            for desired, actual in ((-1, 0.3), (-0.05, 0.3), (0, 0), (0.05, 0.1),
                                    (0.2, 0), (1, 1), (1, 2.5), (1, 2.5001),
                                    (1, 4), (4.5, 3), (6, 3), (1, -0.2)):
                result.append(["normal", gear, desired, actual, curve, "none"])
        for scene in ("manual", "mode_cycle", "can_estop", "speed_ramp", "lateral",
                      "navigation_nan", "navigation_inf", "stale_can", "stale_navigation", "gnss",
                      "sensor_slow", "acc", "terminal", "safety", "safety_zero", "safety_path5",
                      "target_cycle", "task_change", "path_change", "traffic", "empty", "gear_cycle",
                      "duplicate", "clockback", "gap"):
            for actual in (0.2, 3.0):
                result.append([scene, gear, 1.0, actual, 0.01, "none"])
        # D输出不比较；D历史不能通过随后非D的完整消息泄漏新行为。
        for prefix in ("drive", "brake", "safety", "terminal"):
            for scene in ("normal", "terminal", "safety_zero", "target_cycle", "empty", "gear_only_transition"):
                result.append([scene, gear, 1.0, 2.0, -0.01, prefix])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-control", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--baseline-only", action="store_true")
    mode.add_argument("--compare-after", action="store_true",
                      help="复用已保存的前版结果，只编译当前版并比较")
    args = parser.parse_args()
    work = args.output.resolve()
    work.mkdir(parents=True, exist_ok=True)
    actual_fields = [line.split("#", 1)[0].split()[1]
                     for line in (PNC / "msg/control_msg.msg").read_text().splitlines()
                     if line.split("#", 1)[0].strip()]
    assert actual_fields == FIELDS, "control_msg字段变化，必须同步完整序列化，不能漏比较"
    cases = scenarios()
    manifest = {"fields": FIELDS, "scenarios": cases,
                "driver_sha256": digest(HERE / "non_d_brake_compat.cpp"),
                "baseline_control": str(args.baseline_control.resolve()),
                "desireAcc": "global zero initialization makes historical m_acc_last=0 in both builds; field is compared",
                "float_comparison": "exact float32 bits; NaN payload normalized, finite/NaN changes rejected"}
    baseline_manifest = work / "baseline_manifest.json"
    if args.compare_after:
        assert json.loads(baseline_manifest.read_text()) == manifest, "基线用例或驱动变化，应重新建立基线"
    else:
        baseline_manifest.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    generate(PNC, work / "stubs")
    common_files = [PNC / "src" / name for name in (
        "robot_control/stanley_controller/stanley_controller.cpp",
        "robot_control/lateral_reverse_control.cpp", "robot_control/longitudinal_speed_control.cpp",
        "robot_path_plan/common/pubalgor/pubalgor.cpp", "robot_path_plan/common/spline/Spline.cpp")]
    common_hashes = {str(path): digest(path) for path in common_files}
    common_manifest = work / "common_sources.json"
    if args.compare_after:
        assert json.loads(common_manifest.read_text()) == common_hashes, "非目标公共算法变化，不能复用旧对象"
    else:
        common_manifest.write_text(json.dumps(common_hashes, indent=2) + "\n")
    flags = ["g++", "-std=c++11", "-O0", "-Wall", "-Wextra", "-include", "cstdint", "-include", "array"]
    flags += ["-I" + str(path) for path in (work / "stubs", PNC / "include", PNC / "src", PNC / "src/robot_path_plan")]
    commands = []

    def compile_common(path):
        output = work / (path.stem + ".o")
        command = flags + ["-c", str(path), "-o", str(output)]
        if not args.compare_after:
            with output.with_suffix(".compile.log").open("w") as log:
                subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
        else:
            assert output.is_file(), "公共构建对象缺失"
        return output, command

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        objects_and_commands = list(pool.map(compile_common, common_files))
    common = [str(item[0]) for item in objects_and_commands]
    commands.extend(item[1] for item in objects_and_commands)
    versions = []
    if not args.compare_after:
        versions.append(("before", args.baseline_control.resolve()))
    if not args.baseline_only:
        versions.append(("after", PNC / "src/robot_control"))
    for version, folder in versions:
        output = work / version
        output.mkdir(exist_ok=True)
        includes = re.findall(r"^#include.*$", (folder / "control_comply.h").read_text(), re.M)
        access = "\n".join(includes) + '\n#define private public\n#include "control_comply.h"\n#undef private\n'
        (output / "non_d_brake_access.h").write_text(access)
        binary = output / "non_d_brake_compat"
        command = flags + ["-I" + str(output), "-I" + str(folder),
                           str(HERE / "non_d_brake_compat.cpp"), str(folder / "control_comply.cpp")]
        command += common + ["-o", str(binary)]
        commands.append(command)
        with (output / "compile.log").open("w") as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
        collected = []
        for index, case in enumerate(cases):
            run = subprocess.run([str(binary)] + [str(value) for value in case],
                                 text=True, capture_output=True, check=True)
            (output / ("case_%03d.log" % index)).write_text(run.stdout + run.stderr)
            records = [line.split()[1:] for line in run.stdout.splitlines() if line.startswith("MESSAGE ")]
            counts = [line.split()[1:] for line in run.stdout.splitlines() if line.startswith("COUNT ")]
            assert len(counts) == 36 and records, (version, case, "缺少真实发布输出")
            assert sum(int(row[1]) for row in counts) == len(records)
            assert all(len(row) == 15 + int(row[14]) for row in records), "消息字段数未完整序列化"
            collected.append({"case": case, "messages": records, "counts": counts})
        (work / (version + "_trace.json")).write_text(json.dumps(collected, ensure_ascii=False, indent=2) + "\n")
        print("Saved %s: %d scenarios, %d non-D messages" %
              (version, len(collected), sum(len(item["messages"]) for item in collected)), flush=True)
    (work / "compile_commands.json").write_text(json.dumps(commands, indent=2) + "\n")
    if args.baseline_only:
        return
    before = json.loads((work / "before_trace.json").read_text())
    after = json.loads((work / "after_trace.json").read_text())
    mismatches = []
    assert len(before) == len(after)
    for index, (old, new) in enumerate(zip(before, after)):
        if old != new:
            mismatch = {"index": index, "case": old["case"],
                        "old_count": len(old["messages"]), "new_count": len(new["messages"])}
            for row, (left, right) in enumerate(zip(old["messages"], new["messages"])):
                if left != right:
                    mismatch["first_message"] = row
                    mismatch["before"] = left
                    mismatch["after"] = right
                    break
            mismatches.append(mismatch)
    result = {"scenarios": len(cases), "messages": sum(len(item["messages"]) for item in after),
              "reverse_messages": sum(len(item["messages"]) for item in after if item["case"][1] == 3),
              "neutral_messages": sum(len(item["messages"]) for item in after if item["case"][1] == 2),
              "transition_scenarios": sum(item["case"][5] != "none" for item in after),
              "mismatches": mismatches, "fields": FIELDS}
    (work / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    if mismatches:
        print(json.dumps(mismatches[:8], ensure_ascii=False, indent=2))
        raise SystemExit("FAIL: %d non-D scenarios differ" % len(mismatches))
    print("PASS: %d scenarios / %d complete non-D messages; %d D-to-non-D scenarios" %
          (result["scenarios"], result["messages"], result["transition_scenarios"]))


if __name__ == "__main__":
    main()
