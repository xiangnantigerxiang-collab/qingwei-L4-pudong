#!/usr/bin/env python3
"""验证D挡4秒动态曲率预瞄/标定/历史释放、改前R对照及同输入性能。先运行verify.py生成公共对象。"""

import argparse
import json
import math
import os
import re
import statistics
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
PNC = HERE.parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--control-build", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--baseline-control", type=Path)
    parser.add_argument("--upper-baseline-control", type=Path,
                        help="可选：参考弯道1.8m/s版本，与1.0m/s改前版共同检查新曲线的中值")
    parser.add_argument("--benchmark", action="store_true")
    args = parser.parse_args()
    work = args.output.resolve()
    build = args.control_build.resolve()
    work.mkdir(parents=True, exist_ok=True)
    common = [build / (name + ".o") for name in (
        "stanley_controller", "lateral_reverse_control", "longitudinal_speed_control", "pubalgor", "Spline")]
    if not all(path.is_file() for path in common):
        raise RuntimeError("先运行tests/control/verify.py，--control-build指向其输出目录")
    binaries = {}
    commands = []
    for version, folder in [("after", PNC / "src/robot_control"), ("before", args.baseline_control),
                            ("upper", args.upper_baseline_control)]:
        if folder is None:
            continue
        folder = folder.resolve()
        output = work / version
        output.mkdir(exist_ok=True)
        header = (folder / "control_comply.h").read_text()
        includes = re.findall(r"^#include.*$", header, re.M)
        access = "\n".join(includes) + '\n#define private public\n#include "control_comply.h"\n#undef private\n'
        (output / "curve_history_access.h").write_text(access)
        flags = ["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-include", "cstdint", "-include", "array"]
        flags += ["-I" + str(path) for path in (
            output, folder, build / "stubs", PNC / "include", PNC / "src", PNC / "src/robot_path_plan")]
        if version != "after":
            flags.append("-DCURVE_HISTORY_BASELINE")
            if "ForwardCurveLimitSpeed" not in header:
                flags.append("-DCURVE_HISTORY_LEGACY_BASELINE")
        command = flags + [str(HERE / "curve_history.cpp"), str(folder / "control_comply.cpp")]
        binary = output / "curve_history"
        command += [str(path) for path in common] + ["-o", str(binary)]
        commands.append(command)
        with (output / "compile.log").open("w") as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
        binaries[version] = binary
    (work / "compile_commands.json").write_text(json.dumps(commands, indent=2) + "\n")

    def run(version, mode):
        proc = subprocess.run([str(binaries[version]), mode], text=True, capture_output=True, check=True)
        (work / version / (mode + ".log")).write_text(proc.stdout + proc.stderr)
        return proc.stdout

    checked = run("after", "check")
    count = int(re.search(r"PASS curve history: (\d+) checks", checked)[1])
    result = {"history_checks": count}
    after_history = run("after", "history_trace")
    pattern = r"^curvelimitspeed:([^,]+),"
    after_limits = [float(value) for value in re.findall(r"^HISTORY_LIMIT (.+)$", after_history, re.M)]
    assert len(after_limits) == 90 and after_limits[0] < 3
    assert all(value == 600 for value in after_limits[29:]), after_limits
    result["D_straight_release"] = {"first_limit": after_limits[0], "limit_after_30": after_limits[29]}
    if "before" in binaries:
        before_history = run("before", "history_trace")
        before_limits = [float(value) for value in re.findall(r"^HISTORY_LIMIT (.+)$", before_history, re.M)]
        if not before_limits:
            before_limits = [float(value) for value in re.findall(pattern, before_history, re.M)]
        assert len(before_limits) == 90
        result["baseline_D_straight_limits"] = {"first": before_limits[0], "last": before_limits[-1]}
        before_trace = run("before", "reverse_trace")
        after_trace = run("after", "reverse_trace")

        def trace(text):
            return [[float(value) for value in line.split()[1:]] for line in text.splitlines()
                    if line.startswith("TRACE ")]

        before, after = trace(before_trace), trace(after_trace)
        assert before and len(before) == len(after)
        assert all(len(a) == len(b) and all(x == y or (math.isnan(x) and math.isnan(y))
                   for x, y in zip(a, b)) for a, b in zip(before, after)), "R完整消息出现变化"
        result["R_identical_messages"] = len(after)
        before = trace(run("before", "mixed_reverse_trace"))
        after = trace(run("after", "mixed_reverse_trace"))
        assert before and len(before) == len(after)
        assert all(len(a) == len(b) and all(x == y or (math.isnan(x) and math.isnan(y))
                   for x, y in zip(a, b)) for a, b in zip(before, after)), "D转R后完整消息出现变化"
        result["R_after_D_identical_messages"] = len(after)
        calibration = {}
        for version in binaries:
            calibration[version] = [tuple(map(float, line.split()[1:]))
                                    for line in run(version, "calibration_trace").splitlines()
                                    if line.startswith("CALIBRATION ")]
        assert len(calibration["before"]) == len(calibration["after"]) == 4901
        reference_before = min(calibration["before"], key=lambda row: abs(row[0] - 0.04))[1]
        if abs(reference_before - 1.4) < 1e-5:
            assert calibration["before"] == calibration["after"], "动态预瞄不能改变同一曲率的速度标定"
            result["identical_calibration_points"] = len(calibration["after"])
            preview = {}
            for version in ("before", "after"):
                preview[version] = [tuple(map(float, line.split()[1:]))
                                    for line in run(version, "preview_trace").splitlines()
                                    if line.startswith("PREVIEW ")]
            assert len(preview["before"]) == len(preview["after"]) == 96
            equal_windows = 0
            for old, new in zip(preview["before"], preview["after"]):
                assert old[:3] == new[:3]
                speed = new[2]
                if speed == 1.5:
                    assert old[3] == new[3], "1.5m/s时应与原6m窗口完全一致"
                    equal_windows += 1
                elif speed < 1.5:
                    assert new[3] >= old[3], "缩短新窗口不能在同一无历史输入下增加峰值曲率"
                else:
                    assert new[3] <= old[3], "扩大新窗口不能漏掉原6m窗口内的曲率"
            result["dynamic_preview_comparison"] = {
                "total_inputs": len(preview["after"]), "identical_6m_inputs": equal_windows
            }
        if abs(reference_before - 1.0) < 1e-5 or abs(reference_before - 1.8) < 1e-5:
            straight = 0
            for (k_old, old), (k_new, new) in zip(calibration["before"], calibration["after"]):
                assert k_old == k_new
                if reference_before < 1.4:
                    assert new >= old - 1e-12, "折中曲线不能低于1.0m/s版"
                else:
                    assert new <= old + 1e-12, "折中曲线不能高于1.8m/s版"
                if k_new <= 0.01:
                    assert new == old, "已通过实车的直线区间必须逐值保持"
                    straight += 1
                if k_new >= 0.04:
                    assert abs(new / old - 1.4 / reference_before) < 3e-7
            result["calibration_comparison"] = {
                "total_points": 4901, "identical_straight_points": straight,
                "reference_before_mps": reference_before,
                "reference_after_mps": min(calibration["after"], key=lambda row: abs(row[0] - 0.04))[1]
            }
        if "upper" in calibration:
            assert abs(reference_before - 1.0) < 1e-5, "下界应使用参考1.0m/s版本"
            reference_upper = min(calibration["upper"], key=lambda row: abs(row[0] - 0.04))[1]
            assert abs(reference_upper - 1.8) < 1e-5 and len(calibration["upper"]) == 4901
            midpoint_errors = []
            for (k_low, low), (k_new, new), (k_high, high) in zip(
                    calibration["before"], calibration["after"], calibration["upper"]):
                assert k_low == k_new == k_high
                assert low <= new <= high, "每个曲率点的限速都应落在两版之间"
                error = abs(new - (low + high) * 0.5)
                assert error <= 2e-7 * max(1.0, high), "限速应为两版逐点算术平均，仅容许float舍入误差"
                if k_new <= 0.01:
                    assert low == new == high, "三版直线保护区必须逐值相同"
                midpoint_errors.append(error)
            result["midpoint_comparison"] = {
                "total_points": len(midpoint_errors), "max_rounding_error_mps": max(midpoint_errors),
                "upper_reference_mps": reference_upper
            }
        if args.benchmark:
            if hasattr(os, "sched_getaffinity"):
                os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
            rows = []
            for points in (128, 1024):
                for speed in (0.0, 1.5, 3.0, 5.0):
                    for trial in range(5):
                        versions = ("before", "after") if trial % 2 == 0 else ("after", "before")
                        for version in versions:
                            proc = subprocess.run([str(binaries[version]), "benchmark", str(points), str(speed)],
                                                  text=True, capture_output=True, check=True)
                            row = json.loads(next(line[6:] for line in proc.stderr.splitlines() if line.startswith("BENCH ")))
                            row.update(version=version, trial=trial)
                            if version == "after":
                                assert row["cpp_allocations"] == 0
                            rows.append(row)
            (work / "benchmark.json").write_text(json.dumps(rows, indent=2) + "\n")
            summary = []
            for points in (128, 1024):
                for speed in (0.0, 1.5, 3.0, 5.0):
                    for version in ("before", "after"):
                        group = [row for row in rows if row["points"] == points and row["speed_mps"] == speed
                                 and row["version"] == version]
                        record = {"points": points, "speed_mps": speed, "version": version}
                        record.update({field: statistics.median(row[field] for row in group) for field in (
                            "p50_us", "p99_us", "max_us", "cpp_allocations", "controller_bytes", "rss_kib")})
                        summary.append(record)
            result["benchmark_summary"] = summary
            if "CalcuForwardPathCurve" in (PNC / "src/robot_control/control_comply.h").read_text():
                geometry_rows = []
                for points in (128, 1024):
                    for trial in range(5):
                        proc = subprocess.run([str(binaries["after"]), "geometry_benchmark", str(points)],
                                              text=True, capture_output=True, check=True)
                        row = json.loads(next(line.split(" ", 1)[1] for line in proc.stderr.splitlines()
                                              if line.startswith("GEOMETRY_BENCH ")))
                        row["trial"] = trial
                        assert row["cpp_allocations"] == 0
                        geometry_rows.append(row)
                (work / "geometry_benchmark.json").write_text(json.dumps(geometry_rows, indent=2) + "\n")
                result["geometry_benchmark_summary"] = [dict(points=points, **{
                    field: statistics.median(row[field] for row in geometry_rows if row["points"] == points)
                    for field in ("p50_us", "p99_us", "max_us", "cpp_allocations", "cache_bytes", "rss_kib")
                }) for points in (128, 1024)]
    (work / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
