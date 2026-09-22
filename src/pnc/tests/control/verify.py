#!/usr/bin/env python3
"""从真实消息生成 ROS 桩,编译控制节点并验证规划速度跟随、积分制动与标定比例。"""

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
    parser.add_argument("--curve-preview", action="store_true",
                        help="构建节点并运行规划制动与动态曲率预瞄专项，不执行历史主控制断言")
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
            request = compile_one(HERE / "brake_request.cpp", "brake_request", folder)
            subprocess.run(["g++", str(request), str(comply)] + [str(obj) for obj in common] +
                           ["-o", str(work / "brake_request")], check=True)
    print("C++11: control_node 编译和链接通过", flush=True)
    with (work / "brake_request.log").open("w") as log:
        subprocess.run([str(work / "brake_request")], stdout=log, stderr=subprocess.STDOUT, check=True)
    print((work / "brake_request.log").read_text().splitlines()[-1], flush=True)
    if args.curve_preview:
        # 普通补刹力度调整时，独立核对触发时机、积分上限及与改前的逐周期积分一致性。
        integral_scenes = [("integral_saturation", 0.3), ("integral_period", 0.01),
                           ("integral_period", 0.05), ("integral_period", 0.1)]
        integral_scenes += [("integral_" + name, 0.3) for name in (
            "irregular", "duplicate", "gap", "clockback", "stale_pending", "stale_active")]
        integral_scenes += [("integral_threshold", value) for value in (0.05, 0.1, 0.2, 0, -1, 11, "nan", "inf")]
        integral_results = []
        for scene, value in integral_scenes:
            observed = {}
            for version, binary in binaries.items():
                proc = subprocess.run([str(binary), scene, str(value), str(fixtures)],
                                      text=True, capture_output=True, check=True)
                (work / (version + "_" + scene + "_" + str(value) + ".log")).write_text(proc.stdout + proc.stderr)
                integrals = [float(line.split("brake_integral: ")[1].split(",")[0])
                             for line in proc.stdout.splitlines() if "brake_integral: " in line]
                brakes = [int(line.split()[2]) for line in proc.stdout.splitlines() if line.startswith("TRACE ")]
                assert integrals and len(integrals) == len(brakes)
                threshold = float(value) if scene == "integral_threshold" else 0.1
                if not math.isfinite(threshold) or threshold <= 0 or threshold > 10:
                    threshold = 0.1
                assert min(integrals) >= 0 and max(integrals) <= 2 * threshold + 1e-9
                if version == "after":
                    assert all(brake in (0, 15) for brake in brakes)
                    if scene == "integral_saturation" or (scene == "integral_threshold" and threshold <= 0.2):
                        assert abs(max(integrals) - 2 * threshold) < 1e-9
                observed[version] = (integrals, [brake > 0 for brake in brakes])
            if "before" in observed:
                assert observed["after"] == observed["before"], "积分累计、介入和释放时机不应随力度系数改变"
            integral_results.append({"scenario": scene, "value": value,
                                     "max_integral_m": max(observed["after"][0])})
        (work / "brake_integral_result.json").write_text(json.dumps(integral_results, indent=2) + "\n")
        print("PASS brake integral timing/saturation: " + str(len(integral_results)) + " scenarios", flush=True)
        command = [sys.executable, "-B", str(HERE / "verify_curve_history.py"),
                   "--control-build", str(work), "--output", str(work / "curve_preview")]
        if args.baseline_control:
            command += ["--baseline-control", str(args.baseline_control.resolve())]
        subprocess.run(command, check=True)
        return
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
        return len(left) == len(right) and all(len(lrow) == len(rrow) for lrow, rrow in zip(left, right)) and all(
            a == b or (math.isnan(a) and math.isnan(b))
            for lrow, rrow in zip(left, right) for a, b in zip(lrow, rrow))

    # 独立按时间/面积检查实际发布，不能只验证积分函数的私有状态。
    for dec, brake in ((-0.2, 0), (0, 0), (0.03, 0), (0.05, 1), (0.49, 7), (0.5, 8),
                       (0.5001, 8), (0.6, 9), (1, 15), (2, 30), (3.4, 51), (10, 100)):
        trace = run("after", "deceleration", dec)
        check("D挡先用电机速度环,首帧无补刹:" + str(dec),
              len(trace) == 60 and trace[0][0] > 0 and trace[0][1] == 0)
        check("持续减速按积分接入和比例限幅:" + str(dec), trace[-1][1] == brake)
        braking = [row for row in trace if row[1] > 0]
        check("补刹发布无驱动且比例不逐帧累加:" + str(dec),
              all(row[0] == 0 and row[1] == brake for row in braking))
        if dec == 0.5:
            check("速度差0.5也积分,面积等于0.1仍用电机环,超过才补刹",
                  all(row[1] == 0 for row in trace[:5]) and trace[5][1] == 8)
        if dec == 0.6:
            check("0.6m/s误差在20Hz下0.2秒接入9%补刹",
                  all(row[0:2] == [15, 0] for row in trace[:4]) and trace[4][0:2] == [0, 9])
    for dec in (-0.2, 0, 0.49, 0.5, 0.5001, 0.6, 1, 1.5, 1.5001, 2, 3.4, 10):
        trace = run("after", "deceleration_reverse", dec)
        check("R挡保留今日修改前速度环及原超速制动:" + str(dec),
              (len(trace) == 5 and all(row[0:2] == [10, 0] for row in trace)) if dec <= 1.5 else
              (len(trace) == 10 and all(row[0:2] == [10, 0] for row in trace[::2]) and
               all(row[0] == 0 and 0 < row[1] <= 5 for row in trace[1::2])))
        if "before" in binaries:
            check("R挡完整发布与指定基准一致:" + str(dec),
                  same_trace(trace, run("before", "deceleration_reverse", dec)))
    if "before" in binaries:
        for scene in ("normal", "launch", "terminal", "safety", "safety_zero", "safety_path5",
                      "gnss", "sensor_slow", "lateral", "empty", "traffic1", "traffic2", "manual", "acc"):
            for value in (0.2, 0.6, 2.0, 6.0):
                scenario = "reverse_compat_" + scene
                check("R挡全消息/发布顺序对照:" + scene + ":" + str(value),
                      same_trace(run("after", scenario, value), run("before", scenario, value)))
    for scenario, brake in (("deceleration_safety", 100), ("deceleration_safety_reverse", 100),
                             ("deceleration_gnss", 100), ("deceleration_lateral", 70)):
        trace = run("after", scenario, 0.6)
        check(scenario + ":独立安全制动不等待积分",
              len(trace) == 5 and all(row[0] == 0 and row[1] == brake for row in trace))
    trace = run("after", "deceleration_fractional")
    check("导航速度1.0、目标0.4先速度环,积分后浮点差正确量化为9%刹车",
          len(trace) == 60 and all(row[1] == 0 for row in trace[:4]) and
          all(row[0:2] == [0, 9] for row in trace[4:]))
    trace = run("after", "deceleration_release", 0.6)
    check("误差消失即释放,下一次减速必须重新积累",
          trace[5][0:2] == [15, 0] and all(row[1] == 0 for row in trace[6:10]) and trace[10][0:2] == [0, 9])
    check("积分补刹切入safety立即100%,解除后直接执行受限目标",
          trace[11][0:2] == [0, 100] and all(row[0:2] == [7, 0] for row in trace[12:]))
    for period in (0.01, 0.025, 0.05, 0.1):
        trace = run("after", "integral_period", period)
        check("按真实时间而非帧数积分,周期:" + str(period),
              all(row[1] == (15 if i * period > 0.100001 else 0) for i, row in enumerate(trace)))
    trace = run("after", "integral_irregular")
    check("不等间隔0/0.02/0.07/0.1/0.11秒,只在面积超过0.1后接入",
          [row[1] for row in trace] == [0, 0, 0, 0, 15])
    default_integral = run("after", "integral_threshold", 0.1)
    for threshold in (0.05, 0.2, 0.5):
        trace = run("after", "integral_threshold", threshold)
        check("积分门槛可调:" + str(threshold),
              all(row[1] == (15 if i * 0.05 > threshold + 0.000001 else 0) for i, row in enumerate(trace)))
    for threshold in (0, -1, 11, "nan", "inf"):
        check("非法积分门槛回退0.1:" + str(threshold),
              same_trace(run("after", "integral_threshold", threshold), default_integral))
    for scene in ("gap", "clockback"):
        trace = run("after", "integral_" + scene)
        check(scene + ":未观测时间不计入积分,未接入状态重新计时",
              [row[1] for row in trace] == [0, 0, 0, 0, 0, 15])
    trace = run("after", "integral_duplicate")
    check("重复同一时间执行不能累积误差面积",
          len(trace) == 34 and all(row[1] == 0 for row in trace[:-1]) and trace[-1][1] == 15)
    trace = run("after", "integral_small")
    check("0.01m/s小误差也积分,接入后整数刹车最小1%",
          all(row[1] == 0 for row in trace[:200]) and all(row[0:2] == [0, 1] for row in trace[202:]))
    trace = run("after", "integral_pulses")
    check("短暂误差不跨次累计,后续持续误差仍可触发",
          all(row[1] == 0 for row in trace[:9]) and all(row[1] == 15 for row in trace[9:]))
    trace = run("after", "integral_command_repeat")
    check("相同规划/任务消息重复到达不能清掉持续减速积分",
          all(row[1] == 0 for row in trace[:3]) and all(row[1] == 15 for row in trace[3:]))
    for reason in ("safety", "gnss", "lateral", "neutral", "manual", "reverse", "task", "equal", "accelerate"):
        trace = run("after", "integral_reset_" + reason)
        first = 7 if reason == "task" else 8
        check(reason + ":已建立的积分在状态变化后清零,恢复后重新积累",
              trace[3][1] == 15 and all(row[1] == 0 for row in trace[5:first]) and
              all(row[1] == 15 for row in trace[first:]))
        if reason in ("safety", "gnss"):
            check(reason + ":积分已接入时仍立即100%急刹", trace[4][0:2] == [0, 100])
        if reason == "reverse":
            check("D挡补刹接入后换R立即恢复原速度环,不携带积分制动", trace[4][0:2] == [10, 0])
    trace = run("after", "integral_stale_pending")
    check("过期反馈不能凭历史车速新建补刹", all(row[1] == 0 for row in trace))
    trace = run("after", "integral_stale_active")
    check("反馈断流不累计未知误差,也不单凭断流释放已接入的刹车",
          all(row[1] == 15 for row in trace[3:]))
    trace = run("after", "integral_instances")
    check("控制实例之间不共享积分", trace[-2][1] == 15 and trace[-1][1] == 0)
    trace = run("after", "integral_saturation")
    integrals = [float(line.split("brake_integral: ")[1].split(",")[0])
                 for line in (work / "after_integral_saturation_0.3.log").read_text().splitlines()
                 if "brake_integral: " in line]
    check("持续误差积分有界,解除后没有残余补刹",
          max(integrals) <= 0.2 and all(row[1] == 15 for row in trace[3:200]) and
          all(row[1] == 0 for row in trace[200:204]) and trace[204][1] == 15)

    default = run("after", "default")
    launch_prefix = [7] * 8
    check("D挡首帧按现有车速+0.5上限直出,不再等待起步斜坡",
          len(default) == 40 and all(row[0:2] == [7, 0] and row[2] == 2 for row in default))
    check("旧三个参数不再影响控制", same_trace(run("after", "old_params"), default))
    for value in (0.01, 0.1, 0.15, 1, 100, 0, -1, "nan", "inf"):
        check("已删除的起步斜率参数不再影响输出:" + str(value),
              same_trace(run("after", "slope", value), default))
    acceleration = run("after", "planning_acceleration")
    check("规划逐次提高目标时当周期按D乘15执行,保留5%低速保底",
          [row[0] for row in acceleration] == [0, 5, 7, 15, 22, 30, 37, 45, 52] and
          all(math.isclose(row[2], desired, abs_tol=1e-6) for row, desired in
              zip(acceleration, [0, 0.2, 0.5, 1, 1.5, 2, 2.5, 3, 3.5])) and
          all(row[1] == 0 for row in acceleration))
    feedback = run("after", "feedback")
    check("理想速度环跟随仅受原速度差上限影响,五帧内达到2m/s的30%给定",
          feedback[0][0:2] == [7, 0] and all(row[0:2] == [30, 0] for row in feedback[4:]))
    check("加速中不新增补刹或油门回退",
          all(row[1] == 0 for row in feedback) and
          all(feedback[i][0] >= feedback[i - 1][0] for i in range(1, len(feedback))))

    stop_cases = {"neutral": 80, "path_safety": 100, "safety_zero": 100,
                  "task_stop": 80, "gnss": 100, "sensor_slow": 0,
                  "lateral": 70, "zero": 0,
                  "traffic1": 30, "traffic2": 50}
    for scenario, brake in stop_cases.items():
        trace = run("after", scenario)
        check(scenario + ":停车立即清零并保留刹车", trace[40][0] == 0 and trace[40][1] == brake)
        resumed = [row[0] for row in trace[41:] if row[1] == 0]
        check(scenario + ":恢复后直接执行当前受限目标", resumed[:8] == launch_prefix)
    empty = run("after", "empty")
    check("空路径停车后直接恢复当前目标", empty[0][0:2] == [0, 50] and
          [row[0] for row in empty[-8:]] == launch_prefix)
    check("目标下降立即生效", run("after", "decrease")[-1][0] == 0)
    check("新控制实例同样直接执行当前受限目标",
          [row[0] for row in run("after", "instances")[-8:]] == launch_prefix)
    reverse = run("after", "gear_reverse")
    check("R 挡按原标定直出,回 D 挡直接执行当前受限目标", reverse[40][0] == 20 and
          [row[0] for row in reverse[-8:]] == launch_prefix)
    terminal = run("after", "terminal_normal")
    check("D 挡普通终点低速刹车从 5% 起逐步建立至 80%",
          [row[1] for row in terminal[:16]] == list(range(5, 81, 5)))
    check("终点缓刹全程无驱动且停车后保持 80%",
          all(row[0] == 0 and row[2] == 0 for row in terminal[:20]) and
          all(row[1] == 80 for row in terminal[15:20]))
    check("终点停车后再次起步直接执行当前受限目标", [row[0] for row in terminal[-8:]] == launch_prefix)
    for scenario in ("terminal_fast", "terminal_reverse", "terminal_other_task"):
        trace = run("after", scenario)
        check(scenario + ":保留原 80% 停车", all(row[1] == 80 for row in trace[:20]))
    for scenario in ("terminal_stopped", "terminal_sensor", "terminal_network", "terminal_dead", "terminal_can_estop"):
        trace = run("after", scenario)
        check(scenario + ":静止或故障立即切换 80% 保持",
              trace[0][1] == 5 and all(row[1] == 80 for row in trace[4:20]))
    trace = run("after", "terminal_safety")
    check("终点缓刹途中 safety 立即覆盖为 100%", trace[0][1] == 5 and
          all(row[0] == 0 and row[1] == 100 for row in trace[4:20]))
    trace = run("after", "terminal_stale")
    check("反馈断流不能继续低速缓刹", trace[0][1] == 5 and
          all(row[1] == 80 for row in trace[5:20]))
    for value, expected in ((0, 80), (0.05, 80), (0.35, 5), (0.35001, 80), (-0.1, 80), ("nan", 80), ("inf", 80)):
        trace = run("after", "terminal_value", value)
        check("终点缓刹实车反馈边界:" + str(value), trace[0][1] == expected)
    trace = run("after", "terminal_gap")
    check("控制周期卡顿不会延长建压时长", trace[4][1] == 75 and trace[5][1] == 80)
    trace = run("after", "terminal_clockback")
    check("时钟回退立即保持原制动", all(row[1] == 80 for row in trace[4:20]))
    trace = run("after", "terminal_hold")
    check("已有安全制动不会因进入终点而重新从 5% 建压",
          trace[0][1] == 100 and all(row[1] == 80 for row in trace[1:20]))
    for speed in (0.1, 0.25, 0.5, 1.0, 2.0, 3.0, 4.0, 4.5, 6.0):
        for scenario in ("steady", "reverse"):
            after = run("after", scenario, speed)
            if "before" in binaries:
                check(scenario + ":标定/转向/消息与回滚基准一致:" + str(speed),
                      same_trace(after, run("before", scenario, speed)))
    low = run("after", "low_target", 0.2)
    check("原 5% 低速保底保留并在首帧生效", all(row[0:2] == [5, 0] for row in low))
    if "before" in binaries:
        check("低速稳定输出与基准一致", same_trace(low[-1:], run("before", "low_target", 0.2)[-1:]))
    (work / "result.json").write_text(json.dumps({"passed": len(passed), "checks": passed},
                                               ensure_ascii=False, indent=2))
    print("PASS:", len(passed), "项检查", flush=True)


if __name__ == "__main__":
    main()
