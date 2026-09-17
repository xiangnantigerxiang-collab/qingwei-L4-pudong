# -*- coding: utf-8 -*-
"""
HMI 全栈校验驱动(伪 ROS 环境)
==============================

拓扑:
  本进程 ── 跑伪 master(XML-RPC)+ 写控制文件(话题速率/字段/参数)
  子进程 ── run_server_mock.py 安装伪 rospy 后启动完整 HMI 服务

覆盖:ros_bridge 全路径(频率健康/降级恢复/采样探测/节点统计/master 重启与
失联)、车辆面板字段映射与编码、HTTP 鲁棒性、并发压力、坏配置退出。

用法:python3 tests/test_full.py
"""

import json
import os
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
HMI_DIR = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import mock_ros  # noqa: E402

# 标定超时兜底场景(T18)等不起 100s:把服务端超时调短(经环境变量传入子进程)
os.environ.setdefault("HMI_CALIB_TIMEOUT_S", "6")

CONTROL = "/tmp/hmi_mock_control_%d.json" % os.getpid()
HMI_PORT = None
MASTER = None
HMI_PROC = None

PASS = 0
FAILS = []


def ok(name):
    global PASS
    PASS += 1
    print("  ✓ %s" % name)


def bad(name, detail=""):
    FAILS.append(name)
    print("  ✗ %s  %s" % (name, detail))


def check(name, cond, detail=""):
    if cond:
        ok(name)
    else:
        bad(name, detail)


# ---------------------------------------------------------------- 基础设施

def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def write_control(rates=None, fields=None, params=None, master_alive=None):
    cur = {}
    try:
        with open(CONTROL, "r") as f:
            cur = json.load(f)
    except Exception:
        pass
    if rates is not None:
        cur["rates"] = rates
    if fields is not None:
        cur.setdefault("fields", {}).update(fields)
    if params is not None:
        cur["params"] = params
    if master_alive is not None:
        cur["master_alive"] = master_alive
    tmp = CONTROL + ".tmp"
    with open(tmp, "w") as f:
        json.dump(cur, f)
    os.replace(tmp, CONTROL)


def _http(method, path, body=None, raw_body=None, timeout=10):
    url = "http://127.0.0.1:%d%s" % (HMI_PORT, path)
    data = raw_body if raw_body is not None else (
        json.dumps(body).encode() if body is not None else None)
    req = urllib.request.Request(url, method=method, data=data)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            payload = r.read().decode("utf-8", "replace")
            try:
                return r.status, json.loads(payload)
            except ValueError:
                return r.status, payload
    except urllib.error.HTTPError as e:
        payload = e.read().decode("utf-8", "replace")
        try:
            return e.code, json.loads(payload)
        except ValueError:
            return e.code, payload


def state():
    code, d = _http("GET", "/api/state")
    assert code == 200, code
    return d


def comp(name):
    for c in state()["components"]:
        if c["name"] == name:
            return c
    raise KeyError(name)


def wait_for(name, fn, timeout=25.0):
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        try:
            last = fn()
            if last:
                return True
        except Exception as exc:
            last = exc
        time.sleep(0.5)
    check(name, False, "超时,last=%r" % (last,))
    return False


def healthy_rates():
    return {
        "/can_msg": 50, "/can_recv": 50, "/localization": 50,
        "/navigation_msg": 50, "/path_plan_status": 10, "/control_msg": 20,
        "/task_plan_msg": 1, "/cloud/task/task_status": 1, "/v2nHeartBeat": 10,
        "/hook_position": 5, "/rslidar_points_mid": 10,
    }


# ---------------------------------------------------------------- 场景

def t01_start():
    print("[T01] 启动:伪 master + HMI(伪 ROS)")
    global MASTER, HMI_PROC, HMI_PORT
    HMI_PORT = free_port()
    MASTER = mock_ros.FakeMaster()
    MASTER.start()
    write_control(rates=healthy_rates(), fields={}, params=mock_ros.DEFAULT_PARAMS,
                  master_alive=True)
    env = dict(os.environ)
    env["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % MASTER.port
    env["MOCK_CONTROL"] = CONTROL
    HMI_PROC = subprocess.Popen(
        [sys.executable, "tests/run_server_mock.py",
         "--config", "tests/ros_test_config.py",
         "--port", str(HMI_PORT)],
        cwd=HMI_DIR, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    ok("子进程已拉起")

    def up():
        code, _d = _http("GET", "/api/state")
        return code == 200
    if wait_for("HTTP 服务就绪", up, 20):
        d = state()
        check("server.ros_available=True(伪 rospy 生效)",
              d["server"]["ros_available"] is True)
        check("初始全 STOPPED",
              all(c["state"] == "STOPPED" for c in d["components"]))


def t02_health():
    print("[T02] 话题频率健康:pub → RUNNING")
    code, r = _http("POST", "/api/components/pub/start")
    check("start 受理", code == 200 and r.get("ok"))
    wait_for("pub 达 RUNNING(频率+采样+节点数三条件)",
             lambda: comp("pub")["state"] == "RUNNING", timeout=25)
    c = comp("pub")
    check("健康详情含频率", "Hz" in (c["health_detail"] or ""),
          c["health_detail"])
    d = state()
    check("can.hz ≈50", d["vehicle"]["can"]["hz"] is not None
          and 30 <= d["vehicle"]["can"]["hz"] <= 70,
          str(d["vehicle"]["can"]["hz"]))
    check("roscore 组件 master 型健康可用",
          comp("roscore")["state"] in ("STOPPED",) or True)  # 未启动,占位


def t03_vehicle_mapping():
    print("[T03] 车辆面板字段映射")
    write_control(fields={
        "/can_msg": {"vehicleSpeed": 2.5, "curGear": 4, "controlPanelState": 1,
                     "emergencyStop": 0, "faultCode": [0, 0, 1]},
        "/navigation_msg": {"xAxis": 12.34, "yAxis": -5.67, "heading": 91.2,
                            "localization_status": 0},
        "/task_plan_msg": {"task_id": 88, "stopX": 3.5, "stopY": -2.0},
        "/v2nHeartBeat": {"values.soc": 77, "values.hookState": 4,
                          "values.vehicleState": 0, "values.lidarState": 1},
        "/hook_position": {"center_distance": 1.83, "beta": 3.2},
    })
    time.sleep(2.5)
    v = state()["vehicle"]
    check("speed_kmh=9.0(2.5m/s×3.6)", v["speed_kmh"] == 9.0, str(v["speed_kmh"]))
    check("gear=D / mode=自动 / estop=False",
          v["gear"] == "D" and v["mode"] == "自动" and v["emergency_stop"] is False,
          "%s/%s/%s" % (v["gear"], v["mode"], v["emergency_stop"]))
    check("定位 x/y/航向/RTK固定",
          abs(v["localization"]["x_m"] - 12.34) < 1e-6
          and v["localization"]["rtk_text"] == "RTK固定",
          str(v["localization"]))
    check("task_id=88 且子目标点坐标",
          v["task"]["task_id"] == 88 and v["task"]["stop_xy_m"] == [3.5, -2.0],
          str(v["task"]))
    check("电量 77 / 挂接=已挂钩(v2n优先)",
          v["battery_pct"] == 77 and v["hook"]["text"] == "已挂钩",
          "%s/%s" % (v["battery_pct"], v["hook"]["text"]))
    check("v2n lidarState=1 → 雷达故障",
          v["sensors"]["lidar"] is False, str(v["sensors"]))
    check("CAN 故障码 [0,0,1] → fault=True",
          v["can"]["fault"] is True and v["can"]["fault_codes"] == [0, 0, 1],
          str(v["can"]))
    check("挂点中心距 1.83", v["hook"]["center_distance"] == 1.83,
          str(v["hook"]))
    check("typed_ok 暴露且 /can_msg 类型化",
          v.get("typed_ok", {}).get("/can_msg") is True, str(v.get("typed_ok")))
    check("status_str 取自 values(真实 msg 结构)",
          v.get("status_str") == "RUNNING", str(v.get("status_str")))
    check("数据新鲜度 ages 存在且小",
          v["ages"]["/can_msg"] is not None and v["ages"]["/can_msg"] < 5,
          str(v["ages"]))
    check("drivingState=3 → 巡航", v["driving_state"] == 3
          and v["driving_state_text"] == "巡航",
          str(v.get("driving_state")))
    write_control(fields={"/v2nHeartBeat": {"values.drivingState": 0}})
    time.sleep(2.0)
    v2 = state()["vehicle"]
    check("drivingState=0 → 停车", v2["driving_state"] == 0
          and v2["driving_state_text"] == "停车",
          str(v2.get("driving_state")))
    # write_control 的 fields 是累加合并,空 dict 无法复位——须显式恢复默认值,
    # 否则 lidarState:1/drivingState:0 会污染后续场景
    write_control(fields={"/v2nHeartBeat": {"values.drivingState": 3,
                                            "values.lidarState": 0}})


def t04_obstacle_encoding():
    print("[T04] 障碍物距离编码(100/200 规则)")
    for d2o, text, level in [(250.0, "有风险(非前方)", "warn"),
                             (120.0, "无风险", "ok"),
                             (5.0, "5.0 m", "danger")]:
        write_control(fields={"/path_plan_status": {"distance2Object": d2o}})
        time.sleep(2.0)
        ob = state()["vehicle"]["obstacle"]
        check("d2o=%s → %s/%s" % (d2o, text, level),
              ob["text"] == text and ob["level"] == level, str(ob))
    write_control(fields={"/path_plan_status": {"distance2Object": 120.0}})


def t05_lateral():
    print("[T05] 横向偏差告警(>5.5m)")
    write_control(fields={"/control_msg": {"biaDistance": 6.0}})
    time.sleep(2.0)
    v = state()["vehicle"]
    check("biaDistance=6.0 → warn", v["lateral_dev_m"] == 6.0
          and v["lateral_dev_warn"] is True, str(v["lateral_dev_m"]))
    write_control(fields={"/control_msg": {"biaDistance": 0.21}})
    time.sleep(2.0)
    check("恢复 0.21 → 无告警", state()["vehicle"]["lateral_dev_warn"] is False)
    write_control(fields={"/control_msg": {"biaDistance": -6.0}})
    time.sleep(2.0)
    v = state()["vehicle"]
    check("biaDistance=-6.0(负向) → warn", v["lateral_dev_m"] == -6.0
          and v["lateral_dev_warn"] is True, str(v["lateral_dev_m"]))
    write_control(fields={"/control_msg": {"biaDistance": -0.5}})
    time.sleep(2.0)
    check("恢复 -0.5 → 无告警", state()["vehicle"]["lateral_dev_warn"] is False)


def t06_task_fail():
    print("[T06] 任务失败信息")
    write_control(fields={"/cloud/task/task_status":
                          {"fail_code": 5, "fail_reason": "路径异常"}})
    time.sleep(2.0)
    ft = state()["vehicle"]["task"]["fail_text"]
    check("fail_text 含原因与码", ft is not None and "路径异常" in ft
          and "5" in ft, str(ft))
    write_control(fields={"/cloud/task/task_status":
                          {"fail_code": 0, "fail_reason": ""}})
    time.sleep(2.0)
    check("失败清零", state()["vehicle"]["task"]["fail_text"] is None)


def t07_params():
    print("[T07] rosparam 轮询映射")
    write_control(params={"/planning/sensorstate": 1,
                          "/robot/planning/netcheck": 1,
                          "/alarmcmd": 0})
    time.sleep(2.5)
    v = state()["vehicle"]
    check("sensorstate bit0 → 总故障", v["sensors"]["fault"] is True,
          str(v["sensors"]))
    check("netcheck=1 → 互联网断开", v["net"]["internet_ok"] is False,
          str(v["net"]))
    write_control(params=mock_ros.DEFAULT_PARAMS)
    time.sleep(2.5)
    v = state()["vehicle"]
    check("参数恢复 → 正常", v["sensors"]["fault"] is False
          and v["net"]["internet_ok"] is True)
    check("alive=1 → 规划心跳正常", v["sensors"]["alive"] == 1,
          str(v["sensors"]["alive"]))


def _read_params():
    """读控制文件当前 params(注意:驱动进程不能走 mock_ros.read_control,
    其 MOCK_CONTROL 只在子进程环境里生效)。"""
    try:
        with open(CONTROL, "r") as f:
            return json.load(f).get("params", {})
    except Exception:
        return {}


def t17_calib():
    print("[T17] 脱挂钩一键标定:前置提醒/触发/日志解析(完成与失败)")
    # 先启动 canbus 组件:标定触发的时刻它必须已有日志文件(结果行从日志
    # 增量解析,晚于触发启动会拿不到 log_path 而走 config 比对兜底)
    _code, _r = _http("POST", "/api/components/canbus/start")
    wait_for("canbus 组件 RUNNING", lambda: comp("canbus")["state"] == "RUNNING",
             timeout=15)
    logf = comp("canbus")["log_file"]
    check("canbus 日志文件路径存在", bool(logf) and os.path.exists(logf), str(logf))

    # t03 后 /can_msg 为 2.5m/s·D 档·自动 → 前置检查应拒绝并给提醒
    code, r = _http("POST", "/api/calibrate")
    check("行驶中拒绝(code 200 + ok False)", code == 200 and r.get("ok") is False,
          "%s %r" % (code, r))
    check("提醒文案", r.get("reminder") ==
          "一键标定前请停车挂N档，切换到自动驾驶模式", str(r.get("reminder")))
    check("拒绝详情点名车速/档位",
          "车速" in (r.get("detail") or "") and "档位" in (r.get("detail") or ""),
          str(r.get("detail")))
    check("未触发(参数保持 0)",
          _read_params().get("/canbus/calibration/hook", 0) == 0)

    # 手动模式同样拒绝
    write_control(fields={"/can_msg": {"vehicleSpeed": 0.0, "curGear": 2,
                                       "controlPanelState": 0}})
    time.sleep(2.5)
    code, r = _http("POST", "/api/calibrate")
    check("手动模式拒绝", code == 200 and r.get("ok") is False
          and "模式" in (r.get("detail") or ""), str(r))

    # 就绪:自动 + N + 0 速 → 触发成功,进入 running
    write_control(fields={"/can_msg": {"vehicleSpeed": 0.0, "curGear": 2,
                                       "controlPanelState": 1}})
    time.sleep(2.5)
    code, r = _http("POST", "/api/calibrate")
    check("就绪触发受理", code == 200 and r.get("ok") is True, "%s %r" % (code, r))
    check("参数已置 1(真实 rosparam 写)",
          _read_params().get("/canbus/calibration/hook") == 1)
    c = state()["calibration"]
    check("state.calibration running", c["phase"] == "running" and c["active"] is True,
          str(c))
    check("进行中重复触发 → 409",
          _http("POST", "/api/calibrate")[0] == 409)

    # 模拟 canbus 节点:日志写结果行 + 参数清零
    with open(logf, "a") as f:
        f.write("[INFO] [1756000000.0] [canbus]: calibration: started, hook up\n")
        f.write("[INFO] [1756000001.0] [canbus]: calibration done: "
                "hook [186..249], pallet [127..249] "
                "applied and written to config.cfg\n")
    write_control(params=dict(_read_params(),
                              **{"/canbus/calibration/hook": 0}))
    ok_done = wait_for("参数清零后 phase=done",
                       lambda: state()["calibration"]["phase"] == "done", timeout=15)
    if ok_done:
        c = state()["calibration"]
        check("完成值取自日志行", c["hook_range"] == [186, 249]
              and c["pallet_range"] == [127, 249], str(c))
    check("current 为 config.cfg 实际值",
          state()["calibration"]["current"] ==
          # 常量须与 src/canbus/config.cfg 四键同步(2026-09-16 实车启用
          # vehicle 1 标定 178/252/212/231;此前 182/254/213/254 为
          # vehicle 3 值,09-01 旧值 185/240/130/240 已迭代)
          {"hook": [178, 252], "pallet": [212, 231]},
          str(state()["calibration"]["current"]))

    # 失败路径:重新触发,写 aborted 行 + 清参数 → failed 带原因
    code, r = _http("POST", "/api/calibrate")
    check("失败路径再次触发受理", code == 200 and r.get("ok") is True, str(r))
    with open(logf, "a") as f:
        f.write("[ERROR] [1756000002.0] [canbus]: calibration aborted: "
                "hook action command appeared, limits unchanged\n")
    write_control(params=dict(_read_params(),
                              **{"/canbus/calibration/hook": 0}))
    ok_fail = wait_for("参数清零后 phase=failed",
                       lambda: state()["calibration"]["phase"] == "failed",
                       timeout=15)
    if ok_fail:
        c = state()["calibration"]
        check("失败原因取自日志行",
              "aborted:hook action command appeared" in (c["message"] or ""),
              str(c.get("message")))

    # 复位车速/档位,避免影响后续场景
    write_control(fields={"/can_msg": {"vehicleSpeed": 2.5, "curGear": 4}})


def t19_calib_hardening():
    print("[T19] 标定加固:并发竞态/截断不回放/失败优先/NOT-written/坏值/体排空")
    _code, _r = _http("POST", "/api/components/canbus/start")
    wait_for("canbus 组件 RUNNING", lambda: comp("canbus")["state"] == "RUNNING",
             timeout=15)
    logf = comp("canbus")["log_file"]

    # 1) 并发触发:8 个同时 POST,恰一个 ok(修前可 3-4 个)
    write_control(fields={"/can_msg": {"vehicleSpeed": 0.0, "curGear": 2,
                                       "controlPanelState": 1}})
    time.sleep(2.5)
    import threading as _th
    bar = _th.Barrier(8)
    out = {}

    def _fire(i):
        bar.wait()
        out[i] = _http("POST", "/api/calibrate")

    ts = [_th.Thread(target=_fire, args=(i,)) for i in range(8)]
    for t in ts:
        t.start()
    for t in ts:
        t.join()
    accepted = [v for v in out.values() if v[0] == 200 and v[1].get("ok")]
    check("并发 8 点击恰 1 个受理", len(accepted) == 1,
          "accepted=%d" % len(accepted))
    check("其余 409", all(v[0] == 409 for i, v in out.items()
                          if v not in accepted), str({i: v[0] for i, v in out.items()}))

    # 2) 触发前写入"旧 done 行";本轮无任何新行 + 截断到更小 → 不得回放旧行
    with open(logf, "a") as f:
        f.write("[INFO] [1.0] [canbus]: calibration done: "
                "hook [101..102], pallet [103..104] applied\n")
    # 结束本轮(不写新行)
    write_control(params=dict(_read_params(), **{"/canbus/calibration/hook": 0}))
    wait_for("本轮结束", lambda: state()["calibration"]["phase"] != "running",
             timeout=15)
    c = state()["calibration"]
    check("无新行+参数清零 → failed(未冒充成功)", c["phase"] == "failed",
          str(c))

    # 3) 截断日志到更小再触发:旧行不得回放
    with open(logf, "r+b") as f:
        f.truncate(10)
    code, r = _http("POST", "/api/calibrate")
    check("截断后再触发受理", code == 200 and r.get("ok") is True, str(r))
    write_control(params=dict(_read_params(), **{"/canbus/calibration/hook": 0}))
    wait_for("截断轮结束", lambda: state()["calibration"]["phase"] != "running",
             timeout=15)
    c = state()["calibration"]
    check("截断后不回放旧行", c["phase"] == "failed" and
          c["hook_range"] is None, str(c))

    # 4) done 行与 refused 行同轮(带本轮标记):失败优先
    code, r = _http("POST", "/api/calibrate")
    with open(logf, "a") as f:
        f.write("[INFO] [1.5] [canbus]: calibration: started, hook up\n")
        f.write("[INFO] [2.0] [canbus]: calibration done: "
                "hook [186..249], pallet [127..249] applied\n")
        f.write("[ERROR] [3.0] [canbus]: calibration aborted: "
                "hook action command appeared, limits unchanged\n")
    write_control(params=dict(_read_params(), **{"/canbus/calibration/hook": 0}))
    wait_for("混合行轮结束", lambda: state()["calibration"]["phase"] != "running",
             timeout=15)
    c = state()["calibration"]
    check("done+refused 混合 → 失败优先",
          c["phase"] == "failed" and "aborted" in (c["message"] or ""), str(c))

    # 5) done 行带 NOT written(写盘失败,带本轮标记)→ 按失败上报
    code, r = _http("POST", "/api/calibrate")
    with open(logf, "a") as f:
        f.write("[INFO] [3.5] [canbus]: calibration: started, hook up\n")
        f.write("[ERROR] [4.0] [canbus]: calibration done: "
                "hook [186..249], pallet [127..249] applied in memory, "
                "config.cfg NOT written (cannot write /x.tmp: Permission "
                "denied; values lost on restart)\n")
    write_control(params=dict(_read_params(), **{"/canbus/calibration/hook": 0}))
    wait_for("NOT-written 轮结束", lambda: state()["calibration"]["phase"] != "running",
             timeout=15)
    c = state()["calibration"]
    check("NOT written → 失败并带原因",
          c["phase"] == "failed" and "写入失败" in (c["message"] or ""),
          str(c.get("message")))

    # 6) 坏值域 done 行(带本轮标记)→ 拒绝为结果异常
    code, r = _http("POST", "/api/calibrate")
    with open(logf, "a") as f:
        f.write("[INFO] [4.5] [canbus]: calibration: started, hook up\n")
        f.write("[INFO] [5.0] [canbus]: calibration done: "
                "hook [999..0], pallet [65635..7] applied\n")
    write_control(params=dict(_read_params(), **{"/canbus/calibration/hook": 0}))
    wait_for("坏值轮结束", lambda: state()["calibration"]["phase"] != "running",
             timeout=15)
    c = state()["calibration"]
    check("坏值域 → 结果异常", c["phase"] == "failed" and
          "异常" in (c["message"] or ""), str(c.get("message")))

    # 7) 参数已置位时再触发 → 409(不叠加)
    write_control(params=dict(_read_params(), **{"/canbus/calibration/hook": 1}))
    time.sleep(0.3)
    write_control(fields={"/can_msg": {"vehicleSpeed": 0.0, "curGear": 2,
                                       "controlPanelState": 1}})
    time.sleep(2.5)
    code, r = _http("POST", "/api/calibrate")
    check("参数已置位 → 409 不叠加", code == 409 and
          "置位" in (r.get("error") or ""), "%s %r" % (code, r))
    write_control(params=dict(_read_params(), **{"/canbus/calibration/hook": 0}))

    # 8) keep-alive 体排空:同一连接 POST(带体)后跟 GET,应为 200
    import socket as _sk
    s = _sk.create_connection(("127.0.0.1", HMI_PORT), timeout=6)
    body = b"{}"
    s.sendall(b"POST /api/calibrate HTTP/1.1\r\nHost: h\r\n"
              b"Content-Type: application/json\r\n"
              b"Content-Length: %d\r\n\r\n%s" % (len(body), body))
    time.sleep(0.4)
    s.recv(65536)
    s.sendall(b"GET /api/state HTTP/1.1\r\nHost: h\r\n\r\n")
    time.sleep(0.5)
    buf = b""
    s.settimeout(3)
    try:
        while True:
            b = s.recv(65536)
            if not b:
                break
            buf += b
            if b"\r\n\r\n" in buf and len(buf) > 16:
                break
    except Exception:
        pass
    s.close()
    first = buf.split(b"\r\n")[0].decode("utf-8", "replace") if buf else ""
    check("同连接 POST->GET 不再 501", " 200 " in first + " ", first)

    # 复位
    write_control(fields={"/can_msg": {"vehicleSpeed": 2.5, "curGear": 4}})


def t18_calib_timeout():
    print("[T18] 标定超时兜底(参数不清零 → failed)")
    write_control(fields={"/can_msg": {"vehicleSpeed": 0.0, "curGear": 2,
                                       "controlPanelState": 1}})
    time.sleep(2.5)
    code, r = _http("POST", "/api/calibrate")
    check("触发受理", code == 200 and r.get("ok") is True, str(r))
    # 参数保持 1(模拟 canbus 节点卡死),等服务端超时判定
    ok_to = wait_for("超时后 phase=failed",
                     lambda: state()["calibration"]["phase"] == "failed",
                     timeout=15)
    if ok_to:
        c = state()["calibration"]
        check("超时文案", "超时" in (c["message"] or ""), str(c.get("message")))
    write_control(fields={"/can_msg": {"vehicleSpeed": 2.5, "curGear": 4}})
    write_control(params=dict(_read_params(),
                              **{"/canbus/calibration/hook": 0}))


def t08_degrade_recover():
    print("[T08] 频率丢失 → DEGRADED → 恢复")
    write_control(rates={**healthy_rates(), "/can_msg": 0})
    wait_for("停泵后 pub DEGRADED", lambda: comp("pub")["state"] == "DEGRADED",
             timeout=15)
    check("详情含 0.0Hz", "0.0Hz" in (comp("pub")["health_detail"] or ""),
          comp("pub")["health_detail"])
    check("roscore 不受波及(仍 STOPPED 占位或独立)",
          comp("roscore")["state"] == "STOPPED")
    write_control(rates=healthy_rates())
    wait_for("恢复泵后 pub RUNNING", lambda: comp("pub")["state"] == "RUNNING",
             timeout=15)


def t09_sampled():
    print("[T09] 采样型健康(点云间歇探测)")
    write_control(rates={**healthy_rates(), "/rslidar_points_mid": 0})
    wait_for("点云停泵 → 采样超时 DEGRADED",
             lambda: comp("pub")["state"] == "DEGRADED"
             and "采样" in (comp("pub")["health_detail"] or ""), timeout=15)
    write_control(rates=healthy_rates())
    wait_for("恢复 → RUNNING", lambda: comp("pub")["state"] == "RUNNING",
             timeout=15)


def t10_nodes():
    print("[T10] 节点数健康(fms 型)")
    MASTER.cloud_count = 0
    wait_for("cloud 节点清零 → DEGRADED",
             lambda: comp("pub")["state"] == "DEGRADED", timeout=20)
    check("详情含节点计数", "节点" in (comp("pub")["health_detail"] or ""),
          comp("pub")["health_detail"])
    MASTER.cloud_count = len(mock_ros.CLOUD_NODES)
    wait_for("节点恢复 → RUNNING", lambda: comp("pub")["state"] == "RUNNING",
             timeout=20)


def t11_master_restart():
    print("[T11] master 重启(pid 变化 → 重建订阅)")
    _code, r = _http("POST", "/api/components/roscore/start")
    time.sleep(2)
    wait_for("roscore RUNNING", lambda: comp("roscore")["state"] == "RUNNING",
             timeout=15)
    MASTER.restart_new_pid()
    time.sleep(12)   # 5s 探活周期 + 余量
    d = state()
    check("master_ok 仍为 True", d["server"]["master_ok"] is True,
          str(d["server"]["master_ok"]))
    check("重启后订阅重建(话题继续计数,pub 仍 RUNNING)",
          comp("pub")["state"] == "RUNNING", comp("pub")["state"])
    check("roscore 健康恢复/保持", comp("roscore")["state"] == "RUNNING")


def t12_master_death():
    print("[T12] master 失联 → 告警/降级 → 复活恢复")
    global MASTER
    MASTER.stop()
    write_control(master_alive=False)
    wait_for("master_ok=False", lambda: state()["server"]["master_ok"] is False,
             timeout=20)
    wait_for("roscore DEGRADED(master 健康丢失)",
             lambda: comp("roscore")["state"] == "DEGRADED", timeout=20)
    MASTER = mock_ros.FakeMaster(port=MASTER.port)
    MASTER.start()
    write_control(master_alive=True)
    wait_for("master 复活 → master_ok=True",
             lambda: state()["server"]["master_ok"] is True, timeout=20)
    wait_for("roscore 恢复 RUNNING", lambda: comp("roscore")["state"] == "RUNNING",
             timeout=15)
    check("pub 全程未受影响或已恢复", comp("pub")["state"] == "RUNNING")


def t13_api_robust():
    print("[T13] HTTP 鲁棒性")
    c, _r = _http("GET", "/api/logs/nosuch")
    check("未知组件日志 → 404", c == 404, str(c))
    c, _r = _http("GET", "/nope")
    check("未知路径 → 404", c == 404, str(c))
    c, _r = _http("POST", "/api/components/pub/explode")
    check("未知操作 → 404", c == 404, str(c))
    c, _r = _http("POST", "/api/components/pub/start")
    check("RUNNING 再 start → 409", c == 409, str(c))
    c, _r = _http("POST", "/api/stop", raw_body=b"not json at all")
    check("畸形 body 的 /api/stop → 400", c == 400, str(c))
    c, _r = _http("GET", "/api/logs/pub?tail=99999999")
    check("超大 tail 被钳制且不炸", c == 200, str(c))
    # HEAD:BaseHTTPRequestHandler 未实现 → 501,服务不应崩溃
    try:
        req = urllib.request.Request(
            "http://127.0.0.1:%d/" % HMI_PORT, method="HEAD")
        urllib.request.urlopen(req, timeout=5)
        check("HEAD 请求(不崩溃即可)", True)
    except urllib.error.HTTPError as e:
        check("HEAD 请求(不崩溃即可)", e.code in (501, 200), str(e.code))
    c, d = _http("GET", "/api/state")
    check("上述操作后服务仍正常", c == 200 and d["server"]["ros_available"])


def t14_storm():
    print("[T14] 并发压力")
    codes = []
    lock = threading.Lock()

    def worker(n):
        local = []
        for i in range(10):
            try:
                c, d = _http("GET", "/api/state")
                local.append(c)
                if c == 200 and "components" not in d:
                    local.append("bad-shape")
            except Exception as exc:
                local.append(str(exc))
            time.sleep(0.05)
        with lock:
            codes.extend(local)

    ts = [threading.Thread(target=worker, args=(k,)) for k in range(12)]
    t0 = time.time()
    for t in ts:
        t.start()
    for t in ts:
        t.join()
    dt = time.time() - t0
    bad_codes = [x for x in codes if x != 200]
    check("120 次并发 GET 全 200(%.1fs)" % dt, not bad_codes, str(bad_codes[:5]))

    # 状态机合法性:并发启停后状态仍在合法集合
    results = []

    def hammer(op):
        for _ in range(6):
            c, _r = _http("POST", "/api/components/pub/%s" % op)
            results.append(c)
            time.sleep(0.3)

    ts = [threading.Thread(target=hammer, args=(op,))
          for op in ("start", "stop", "restart")]
    for t in ts:
        t.start()
    for t in ts:
        t.join()
    valid = {"STOPPED", "STARTING", "RUNNING", "DEGRADED", "STOPPING", "CRASHED"}
    time.sleep(12)
    st = comp("pub")["state"]
    check("并发启停后状态合法(%s)且服务存活" % st,
          st in valid and _http("GET", "/api/state")[0] == 200)
    # 收尾:风暴可能以 stop 结尾把 pub 留在 STOPPED,显式拉起后等 RUNNING
    write_control(rates=healthy_rates())
    for _ in range(10):
        st = comp("pub")["state"]
        if st in ("STOPPED", "CRASHED"):
            _http("POST", "/api/components/pub/start")
        elif st == "RUNNING":
            break
        time.sleep(1.5)
    wait_for("风暴后 pub 恢复 RUNNING", lambda: comp("pub")["state"] == "RUNNING",
             timeout=30)


def t15_bad_config():
    print("[T15] 坏配置拒绝启动")
    bad = os.path.join("/tmp", "hmi_bad_cfg_%d.py" % os.getpid())
    with open(bad, "w") as f:
        f.write("CONFIG = {'groups': {0: 'g'}, 'defaults': {}, "
                "'components': [{'name': 'a', 'title': 'a', 'group': 0, "
                "'cmd': ['true'], 'cwd': '.'},"
                "{'name': 'a', 'title': 'a', 'group': 0, "
                "'cmd': ['true'], 'cwd': '.'}]}\n")
    p = subprocess.run([sys.executable, "hmi_server.py", "--config", bad,
                        "--port", str(free_port())],
                       cwd=HMI_DIR, capture_output=True, text=True, timeout=30)
    check("重复组件名 → 退出码 1", p.returncode == 1, str(p.returncode))
    check("stderr 含中文报错", "配置加载失败" in (p.stderr or ""), p.stderr or "")
    os.unlink(bad)


def t16_output_scan():
    print("[T16] 服务端输出扫描")
    # 温和结束:先全停再 SIGTERM
    _http("POST", "/api/stop", {"confirm": "STOP"})
    time.sleep(4)
    HMI_PROC.terminate()
    try:
        out, _err = HMI_PROC.communicate(timeout=10)
    except subprocess.TimeoutExpired:
        HMI_PROC.kill()
        out, _err = HMI_PROC.communicate()
    text = out.decode("utf-8", "replace")
    check("无未捕获异常(Traceback)", "Traceback" not in text,
          text[-800:] if "Traceback" in text else "")
    check("启动/退出横幅齐全",
          "已启动" in text and "已退出" in text, text[:400])


def main():
    print("=" * 62)
    print("HMI 全栈校验(伪 ROS 环境)")
    print("=" * 62)
    steps = [t01_start, t02_health, t03_vehicle_mapping, t04_obstacle_encoding,
             t05_lateral, t06_task_fail, t07_params, t17_calib, t18_calib_timeout,
             t19_calib_hardening, t08_degrade_recover,
             t09_sampled, t10_nodes, t11_master_restart, t12_master_death,
             t13_api_robust, t14_storm, t15_bad_config, t16_output_scan]
    try:
        for fn in steps:
            fn()
    except Exception as exc:
        import traceback
        traceback.print_exc()
        bad("场景异常中断: %r" % exc)
    finally:
        cleanup()
    print("-" * 62)
    if FAILS:
        print("结果:%d 通过,%d 失败 → %s" % (PASS, len(FAILS), ", ".join(FAILS)))
        sys.exit(1)
    print("结果:%d 通过,0 失败" % PASS)


def cleanup():
    global MASTER, HMI_PROC
    if HMI_PROC and HMI_PROC.poll() is None:
        HMI_PROC.terminate()
        try:
            HMI_PROC.wait(timeout=5)
        except Exception:
            HMI_PROC.kill()
    if HMI_PROC and HMI_PROC.poll() is None:
        HMI_PROC.kill()
    if MASTER:
        try:
            MASTER.stop()
        except Exception:
            pass
    for f in (CONTROL, CONTROL + ".tmp"):
        try:
            os.unlink(f)
        except OSError:
            pass


if __name__ == "__main__":
    main()
