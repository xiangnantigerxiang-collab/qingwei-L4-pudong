#!/usr/bin/env python3
"""验证障碍物 ID、Kalman 速度、navigation 航向、跨帧关联及按 ID 累计置信度。"""

import argparse
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
from verify import run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    pnc = Path(__file__).resolve().parents[3]
    work = args.output.resolve()
    work.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(pnc / "tests/planning"))
    from generate_stubs import generate
    generate(pnc, work / "stubs")
    source = Path(__file__).with_name("tracking_test.cpp")
    flags = ["g++", "-std=c++11", "-Wall", "-Wextra", "-Wpedantic", "-Werror", "-I" + str(work / "stubs")]
    binary = work / "tracking_test"
    run([*flags, "-O2", str(source), "-o", str(binary)], work / "build.log")
    result = run([str(binary)], work / "test.log").strip()
    print(result, flush=True)
    benchmark = run([str(binary), "--benchmark"], work / "benchmark.log").splitlines()
    print("\n".join(benchmark), flush=True)
    sanitized = work / "tracking_sanitize"
    run([*flags, "-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", str(source),
         "-o", str(sanitized)], work / "sanitize_build.log")
    sanitized_result = run([str(sanitized)], work / "sanitize.log").strip()
    print("ASan/UBSan:", sanitized_result, flush=True)
    (work / "result.json").write_text(json.dumps({
        "language": "C++11", "transport": "stubs generated from actual ROS messages", "test": result,
        "sanitizers": sanitized_result, "benchmark": benchmark,
    }, ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    main()
