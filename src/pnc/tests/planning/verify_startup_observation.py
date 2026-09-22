#!/usr/bin/env python3
"""验证手动切自动的起步周边观察、最终发布及既有规划/控制回归。"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from generate_stubs import generate
from verify import run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    tests = Path(__file__).resolve().parent
    pnc = tests.parents[1]
    work = args.output.resolve()
    work.mkdir(parents=True, exist_ok=True)
    generate(pnc, work / "stubs")
    flags = ["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-I" + str(work / "stubs")]
    results = {}
    for name, options in (("core", ["-O2"]),
                          ("sanitized", ["-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer"])):
        binary = work / name
        run(flags + options + [str(tests / "startup_observation.cpp"), "-o", str(binary)],
            work / (name + "_compile.log"))
        run([str(binary)], work / (name + ".log"))
        results[name] = (work / (name + ".log")).read_text().strip()
        print(name + ": " + results[name], flush=True)
    subprocess.run([sys.executable, "-B", str(tests / "verify_map_speed_limit.py"),
                    "--output", str(work / "integration")], check=True)
    results["integration"] = json.loads((work / "integration/result.json").read_text())
    results["source_sha256"] = {
        str(path.relative_to(pnc)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in (pnc / "src/robot_path_plan").rglob("*")
        if path.suffix in (".h", ".cpp", ".inc")
    }
    (work / "result.json").write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n")
    print("PASS:", work / "result.json", flush=True)


if __name__ == "__main__":
    main()
