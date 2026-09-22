#!/usr/bin/env python3
"""真实任务/规划源码联调；借用全量桩构建的公共算法对象，重编被测业务单元。"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
PNC = HERE.parents[1]
sys.path.insert(0, str(HERE.parent / "planning"))
from generate_stubs import generate
from verify import sources, trace_header


def run(cmd, log):
    with log.open("w") as stream:
        subprocess.run(cmd, stdout=stream, stderr=subprocess.STDOUT, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    work = args.output.resolve()
    work.mkdir(parents=True, exist_ok=True)
    generate(PNC, work / "stubs")
    trace_header(PNC, work)
    includes = [work, work / "stubs", PNC / "include", PNC / "src", PNC / "src/robot_path_plan"]
    includes += [PNC / "3rd-party" / name / "include" for name in ("osqp", "qpoases", "proj4")]
    flags = ["g++", "-std=c++14", "-O1", "-g", "-include", "cstdint", "-include", "array"]
    flags += ["-I" + str(p) for p in includes]
    objects = []
    # 编译核心与节点，节点不放进测试程序，避免 main 重复。
    for name in ("robot_task_plan/task_plan_core.cpp", "robot_path_plan/path_plan_comply.cpp",
                 "robot_task_plan/task_plan_node.cpp", "robot_path_plan/path_plan_node.cpp"):
        obj = work / (Path(name).stem + ".o")
        run(flags + ["-c", str(PNC / "src" / name), "-o", str(obj)], obj.with_suffix(".log"))
        if "_node" not in name:
            objects.append(str(obj))
    manifest = json.loads((args.build / "result.json").read_text())["objects"]
    common = sources(PNC, "path_plan_comply")[1:] + sources(PNC, "pubalgor") + sources(PNC, "spline")
    common += ["src/robot_task_plan/frame_transform.cc"]
    objects += [manifest[name] for name in dict.fromkeys(common)]
    binary = work / "regression"
    run(flags + [str(HERE / "regression.cpp"), *objects, "-lyaml-cpp",
                 str(PNC / "3rd-party/osqp/lib/libosqp.so"),
                 "-Wl,-rpath," + str(PNC / "3rd-party/osqp/lib"), "-o", str(binary)], work / "link.log")
    fixtures = work / "fixtures"
    fixtures.mkdir(exist_ok=True)
    run([str(binary), str(fixtures), str(PNC)], work / "run.log")
    print((work / "run.log").read_text().splitlines()[-1])
    # 纯 core 单独按生产 C++11 标准编译，并运行更新了截停契约的原专项。
    core = work / "core_cxx11.o"
    cxx11 = ["-std=c++11" if arg == "-std=c++14" else arg for arg in flags]
    run(cxx11 + ["-c", str(PNC / "src/robot_task_plan/task_plan_core.cpp"), "-o", str(core)], work / "core_cxx11.log")
    legacy = work / "task_fence_legacy"
    run(cxx11 + [str(HERE.parent / "test_task_plan_fence.cpp"), str(core),
                 manifest["src/robot_task_plan/frame_transform.cc"],
                 manifest["src/robot_path_plan/common/pubalgor/pubalgor.cpp"], "-lyaml-cpp", "-o", str(legacy)], work / "legacy_compile.log")
    run([str(legacy), str(PNC / "path")], work / "legacy.log")
    print((work / "legacy.log").read_text().splitlines()[-1])


if __name__ == "__main__":
    main()
