#!/usr/bin/env python3
"""从真实 CMake 目标表编译 PNC 全部翻译单元并链接六个节点；ROS 使用编译桩。"""

import argparse
import concurrent.futures
import json
import re
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "planning"))
from generate_stubs import generate


def run(command, log):
    with log.open("w") as stream:
        result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError("编译或链接失败，见 " + str(log))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pnc", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--hdmap-sdk", type=Path, help="本机架构的已安装 HDMap SDK，默认 ../hdmap/sdk")
    args = parser.parse_args()
    pnc, work = args.pnc.resolve(), args.output.resolve()
    hdmap_sdk = (args.hdmap_sdk or (pnc.parent / "hdmap/sdk")).resolve()
    hdmap_library = hdmap_sdk / "lib/libhdmap_server.so"
    if not hdmap_library.is_file():
        raise RuntimeError("请先编译安装本机 HDMap SDK：" + str(hdmap_library))
    work.mkdir(parents=True, exist_ok=True)
    generate(pnc, work / "stubs")
    cmake = re.sub(r"#[^\n]*", "", (pnc / "CMakeLists.txt").read_text())
    targets = {}
    for match in re.finditer(r"add_(library|executable)\((\w+)\s+(.*?)\)", cmake, re.S):
        targets[match[2]] = {"kind": match[1], "sources": re.findall(r"src/\S+\.(?:cpp|cc|c)", match[3])}
    dependencies = {match[1]: match[2].split() for match in re.finditer(
        r"target_link_libraries\((\w+)\s+(.*?)\)", cmake, re.S)}
    sources = sorted({source for target in targets.values() for source in target["sources"]})
    includes = [work / "stubs", pnc / "include", pnc / "src", pnc / "src/robot_path_plan"]
    includes += [pnc / "3rd-party" / name / "include" for name in ("osqp", "qpoases", "proj4")]
    flags = ["g++", "-std=c++14", "-O0", "-Wall", "-Wextra", "-include", "cstdint", "-include", "array"]
    flags += ["-I" + str(path) for path in includes]

    def compile_source(source):
        name = source.replace("/", "_")
        target = work / (name + ".o")
        extra = ["-I" + str(hdmap_sdk / "include")] if "robot_perception_convert/" in source else []
        run(flags + extra + ["-c", str(pnc / source), "-o", str(target)], work / (name + ".log"))
        return source, str(target)

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        objects = dict(pool.map(compile_source, sources))
    print("Compiled", len(objects), "PNC translation units", flush=True)

    def collect_sources(name, visited):
        if name in visited:
            return []
        visited.add(name)
        result = list(targets[name]["sources"])
        for dependency in dependencies.get(name, []):
            if dependency in targets:
                result += collect_sources(dependency, visited)
        return list(dict.fromkeys(result))

    executables = []
    for name, target in targets.items():
        if target["kind"] != "executable":
            continue
        unit_sources = collect_sources(name, set())
        libraries = ["-lyaml-cpp", str(pnc / "3rd-party/osqp/lib/libosqp.so"),
                     "-Wl,-rpath," + str(pnc / "3rd-party/osqp/lib")]
        if name == "perception_msg_convert":
            libraries += [str(hdmap_library), "-Wl,-rpath," + str(hdmap_library.parent)]
        run(["g++", *[objects[source] for source in unit_sources], *libraries,
             "-o", str(work / name)], work / (name + "_link.log"))
        executables.append(name)
    result = {"translation_units": len(objects), "executables": executables, "objects": objects}
    (work / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print("PASS: linked", len(executables), "nodes:", ", ".join(executables), flush=True)


if __name__ == "__main__":
    main()
