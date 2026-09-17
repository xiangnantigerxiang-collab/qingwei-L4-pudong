#!/usr/bin/env python3
"""从真实消息生成 ROS 桩,编译控制节点并验证起步斜坡与标定比例。"""

import argparse
import concurrent.futures
import json
import math
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PNC = HERE.parents[1]
sys.path.insert(0, str(HERE.parent / "planning"))
from generate_stubs import generate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--baseline-control", type=Path,
                        help="可选:回滚基准 control_comply.cpp/control_comply.h 所在目录")
    args = parser.parse_args()
    work = args.output.resolve()
    work.mkdir(parents=True, exist_ok=True)
    generate(PNC, work / "stubs")
    fixtures = work / "fixtures"
    fixtures.mkdir(exist_ok=True)
    (fixtures / "inside.csv").write_text("-10,-10,0,0\n50,-10,0,0\n50,10,0,0\n-10,10,0,0\n")
    (fixtures / "outside.csv").write_text("-10,-10,0,0\n1,-10,0,0\n1,10,0,0\n-10,10,0,0\n")
    includes = [work / "stubs", PNC / "include", PNC / "src", PNC / "src/robot_path_plan"]
    flags = ["g++", "-std=c++11", "-O0", "-Wall", "-Wextra", "-include", "cstdint", "-include", "array"]
    flags += ["-I" + str(path) for path in includes]

    def compile_one(source, name, control_dir=None):
        obj = work / (name + ".o")
        extra = ["-I" + str(control_dir), "-I" + str(PNC / "src/robot_control")] if control_dir else []
        with (work / (name + ".compile.log")).open("w") as log:
            subprocess.run(flags + extra + ["-c", str(source), "-o", str(obj)],
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        return obj

    names = ["robot_control/stanley_controller/stanley_controller.cpp",
             "robot_control/lateral_reverse_control.cpp",
             "robot_control/longitudinal_speed_control.cpp",
             "robot_path_plan/common/pubalgor/pubalgor.cpp",
             "robot_path_plan/common/spline/Spline.cpp"]
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        jobs = [pool.submit(compile_one, PNC / "src" / name, Path(name).stem) for name in names]
        common = [job.result() for job in jobs]
    binaries = {}
    for version, folder in [("after", PNC / "src/robot_control"),
                            ("before", args.baseline_control)]:
        if folder is None:
            continue
        folder = folder.resolve()
        comply = compile_one(folder / "control_comply.cpp", version + "_comply", folder)
        test = compile_one(HERE / "launch_speed.cpp", version + "_test", folder)
        binary = work / (version + "_test")
        subprocess.run(["g++", str(test), str(comply)] + [str(obj) for obj in common] +
                       ["-o", str(binary)], check=True)
        binaries[version] = binary
        if version == "after":
            node = compile_one(PNC / "src/robot_control/control_node.cpp", "control_node", folder)
            subprocess.run(["g++", str(node), str(comply)] + [str(obj) for obj in common] +
                           ["-o", str(work / "control_node")], check=True)
    print("C++11: control_node 编译和链接通过", flush=True)
    passed = []

    def run(version, scenario, value=0.3):
        proc = subprocess.run([str(binaries[version]), scenario, str(value), str(fixtures)],
                              text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=True)
        (work / (version + "_" + scenario + "_" + str(value) + ".log")).write_text(proc.stdout)
        lines = [line.split()[1:] for line in proc.stdout.splitlines() if line.startswith("TRACE ")]
        return [[float(value) for value in line] for line in lines]

    def check(name, condition):
        if not condition:
            raise AssertionError(name)
        passed.append(name)
        print("PASS:", name, flush=True)

    def same_trace(left, right):
        # 原横向算法直线场景可能输出 NaN;同形态保留比较,不掩盖有限值变化。
        return len(left) == len(right) and all(
            a == b or (math.isnan(a) and math.isnan(b))
            for lrow, rrow in zip(left, right) for a, b in zip(lrow, rrow))

    default = run("after", "default")
    check("默认斜率保留小数累积,首四帧 0/0/0/1", [row[0] for row in default[:4]] == [0, 0, 0, 1])
    check("默认 0.3m/s^2:1s 输出 5%,最终到原目标 9%", default[19][0] == 5 and default[-1][0] == 9)
    check("旧三个参数不再影响控制", same_trace(run("after", "old_params"), default))
    slow = run("after", "slope", 0.1)
    check("较小斜率 0.1 仍能起步且更柔和", slow[19][0] == 1 and slow[-1][0] == 3)
    for value in (0, -1, "nan", "inf"):
        check("非法斜率回退默认:" + str(value), same_trace(run("after", "slope", value), default))
    feedback = run("after", "feedback")
    check("速度环跟随时平缓到达 2m/s 对应的 36%", feedback[-1][0] == 36)
    check("整个起步过程每帧油门增量不超过量化的 1%",
          all(0 <= feedback[i][0] - feedback[i - 1][0] <= 1 for i in range(1, len(feedback))))
    check("速度给定 1m/s 约 3.35s 达到", feedback[65][0] < 18 and feedback[66][0] == 18)

    stop_cases = {"neutral": 80, "path_safety": 100, "safety_zero": 100,
                  "task_stop": 80, "gnss": 100, "sensor_slow": 0,
                  "lateral": 70, "fence": 70, "zero": 0,
                  "traffic1": 30, "traffic2": 50}
    for scenario, brake in stop_cases.items():
        trace = run("after", scenario)
        check(scenario + ":停车立即清零并保留刹车", trace[40][0] == 0 and trace[40][1] == brake)
        resumed = [row[0] for row in trace[41:] if row[1] == 0]
        check(scenario + ":恢复起步不继承旧油门", resumed[:4] == [0, 0, 0, 1])
    empty = run("after", "empty")
    check("空路径停车后恢复软起步", empty[0][0:2] == [0, 50] and
          [row[0] for row in empty[-4:]] == [0, 0, 0, 1])
    check("目标下降立即生效", run("after", "decrease")[-1][0] == 0)
    check("控制实例之间不共享斜坡状态",
          [row[0] for row in run("after", "instances")[-4:]] == [0, 0, 0, 1])
    reverse = run("after", "gear_reverse")
    check("R 挡按原标定直出,回 D 挡重新起步", reverse[40][0] == 20 and
          [row[0] for row in reverse[-4:]] == [0, 0, 0, 1])
    for speed in (0.1, 0.25, 0.5, 1.0, 2.0, 3.0, 4.0, 4.5, 6.0):
        for scenario in ("steady", "reverse"):
            after = run("after", scenario, speed)
            if "before" in binaries:
                check(scenario + ":标定/转向/消息与回滚基准一致:" + str(speed),
                      same_trace(after, run("before", scenario, speed)))
    low = run("after", "low_target", 0.2)
    check("原 5% 最小目标保留,经过斜坡而非首帧跳变", low[0][0] == 0 and low[-1][0] == 5)
    if "before" in binaries:
        check("低速稳定输出与基准一致", same_trace(low[-1:], run("before", "low_target", 0.2)[-1:]))
    (work / "result.json").write_text(json.dumps({"passed": len(passed), "checks": passed},
                                               ensure_ascii=False, indent=2))
    print("PASS:", len(passed), "项检查", flush=True)


if __name__ == "__main__":
    main()
