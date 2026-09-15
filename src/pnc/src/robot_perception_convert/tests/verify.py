#!/usr/bin/env python3
"""验证时序滤波的边界、置信度、空间索引及本机耗时；ROS 消息由真实 .msg 生成。"""

import argparse
import json
from pathlib import Path
import platform
import subprocess
import sys


def run(command, log):
    result = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    log.write_text(result.stdout)
    if result.returncode:
        raise RuntimeError(f"失败：{log}\n{result.stdout[-5000:]}")
    return result.stdout


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
    source = Path(__file__).with_name("temporal_filter_test.cpp")
    flags = ["g++", "-std=c++11", "-Wall", "-Wextra", "-Wpedantic", "-Werror", "-I" + str(work / "stubs")]
    binary = work / "temporal_filter_test"
    run([*flags, "-O2", str(source), "-o", str(binary)], work / "build.log")
    result = run([str(binary)], work / "test.log")
    print(result.strip(), flush=True)
    benchmark = run([str(binary), "--benchmark"], work / "benchmark.log")
    print(benchmark.strip(), flush=True)
    sanitized = work / "temporal_filter_sanitize"
    run([*flags, "-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
         str(source), "-o", str(sanitized)], work / "sanitize_build.log")
    sanitize_result = run([str(sanitized)], work / "sanitize.log")
    print("ASan/UBSan:", sanitize_result.strip(), flush=True)
    summary = {"language": "C++11", "transport": "generated ROS message stubs", "test": result.strip(),
               "sanitizers": sanitize_result.strip(), "benchmark": benchmark.splitlines(),
               "machine": platform.machine(), "python": platform.python_version()}
    (work / "result.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    main()
