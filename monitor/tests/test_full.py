# -*- coding: utf-8 -*-
"""
monitor 全栈测试(伪 ROS,开发机用)——对标 hmi/tests/test_full.py 风格
=====================================================================

用法: cd monitor && python3 tests/test_full.py

覆盖:
  t01 静态端点与地图三线/图层默认态
  t02 换算锁定(yaw=90-heading / dx-dy 互换 / 路径丢帧 / stopAngle / origin)
  t03 scan.bin(极坐标->笛卡尔->外参旋转,inf/0/负滤除)
  t04 cloud 解析变体(大端/uint16 intensity/organized/NaN/stride/上限)
  t05 bin 字节锁(与 frontend_test.js 共用 fixture)
  t06 数据龄(停泵后增长)
  t07 并发混合请求
  t08 RSS 有界
  t09 py3.8 兼容(ast feature_version)
"""

import ast
import json
import math
import os
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
MON = os.path.dirname(HERE)
CONTROL = "/tmp/monitor_mock_control.json"
PORT = 18081
BASE = "http://127.0.0.1:%d" % PORT

PASS = 0
FAILS = []


def check(name, cond, detail=""):
    global PASS
    if cond:
        PASS += 1
        print("  ok %s" % name)
    else:
        FAILS.append(name)
        print("  FAIL %s  %s" % (name, detail))


def write_control(rates=None, fields=None, master_alive=True,
                  master_pid=None):
    ctl = {"master_alive": master_alive, "rates": rates or {},
           "fields": fields or {}}
    if master_pid is not None:
        ctl["master_pid"] = master_pid      # FakeMaster 读它模拟重启
    tmp = CONTROL + ".tmp"
    with open(tmp, "w") as f:
        json.dump(ctl, f)
    os.replace(tmp, CONTROL)


def http(path, timeout=10):
    try:
        r = urllib.request.urlopen(BASE + path, timeout=timeout)
        return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


def snap():
    _, b = http("/api/snapshot")
    return json.loads(b.decode("utf-8"))


def decode_bin(blob):
    """服务端同款解码(供测试断言)。"""
    if len(blob) < 20:
        return None
    magic, ver, nchan, total, ts = struct.unpack_from("<4sHHIQ", blob, 0)
    if magic != b"QWMC":
        return None
    off = 20
    channels = []
    for _ in range(nchan):
        cid, flags, rsv, cnt = struct.unpack_from("<BBHI", blob, off)
        off += 8
        pts = []
        for _ in range(cnt):
            x, y, z, i, _pad = struct.unpack_from("<iihBB", blob, off)
            off += 12
            pts.append((x / 1000.0, y / 1000.0, z / 1000.0, i))
        channels.append({"id": cid, "count": cnt, "pts": pts})
    return {"ts": ts, "total": total, "channels": channels}


def wait_for(name, fn, timeout=25.0):
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        try:
            last = fn()
            if last:
                return last
        except Exception as exc:
            last = exc
        time.sleep(0.5)
    check(name, False, "超时 last=%r" % (last,))
    return None


def rss_kb(pid):
    try:
        with open("/proc/%d/status" % pid) as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1])
    except OSError:
        pass
    return None


# ------------------------------------------------------------------ 场景

def t01_static():
    print("== t01 静态端点 ==")
    code, body = http("/")
    check("页面 200", code == 200 and len(body) > 10000, str(code))
    code, body = http("/three.min.js")
    check("three.min.js 200", code == 200 and len(body) > 500000, str(code))
    code, _ = http("/nope")
    check("未知路径 404", code == 404)
    _, body = http("/api/map")
    m = json.loads(body.decode("utf-8"))
    check("地图目录模式: 2 张按名有序",
          m.get("n") == 2 and [x["name"] for x in m["maps"]] ==
          ["m1.csv", "m2.csv"], str(m.get("n")))
    check("地图中心线点数 3+2",
          len(m["maps"][0]["center"]) == 3 and
          len(m["maps"][1]["center"]) == 2)
    check("三线等长", len(m["maps"][0]["left"]) ==
          len(m["maps"][0]["center"]) == len(m["maps"][0]["right"]))
    check("合并 bbox(目录模式)", m["bbox"] == [0.0, 0.0, 110.0, 100.0],
          str(m["bbox"]))
    check("图层默认态照 rviz",
          m["layers"]["map"] is False and m["layers"]["planning"] is False
          and m["layers"]["grid"] is False and m["layers"]["vehicle"] is True)
    d = snap()
    check("ros_available=True(伪 rospy 生效)", d["ros_available"] is True)
    check("origin=地图中心",
          abs(d["origin"][0] - (m["bbox"][0] + m["bbox"][2]) / 2) < 0.01)


def t02_conversions():
    print("== t02 换算锁定 ==")
    write_control(rates={"/navigation_msg": 10, "/perception": 5,
                         "/plan_path_msg": 5, "/refer_path_msg": 5,
                         "/path_plan_status": 5, "/palletpos": 5})
    d = wait_for("navigation 数据到位",
                 lambda: snap()["vehicle"] and snap())
    if not d:
        return
    v = d["vehicle"]
    check("yaw=(90-heading)rad",
          abs(v["yaw"] - math.radians(90 - 274.59)) < 1e-3, str(v["yaw"]))
    check("x/y 减 origin",
          abs(v["x"] - (55.94 - d["origin"][0])) < 0.02 and
          abs(v["y"] - (-12.58 - d["origin"][1])) < 0.02, str(v))
    check("speed 透传", v["speed"] == 2.5)

    o = d["obstacles"][0]
    check("障碍 dx/dy 互换(l=dy=4,w=dx=1)",
          o["l"] == 4.0 and o["w"] == 1.0, str(o))
    check("障碍 yaw=(90-heading)rad",
          abs(o["yaw"] - math.radians(-10.0)) < 1e-3, str(o["yaw"]))
    check("障碍坐标减 origin",
          abs(o["x"] - (58.0 - d["origin"][0])) < 0.02)
    check("障碍 type 透传(供前端着色)", o["type"] == 0, str(o))
    check("障碍 confidence 透传", o["conf"] == 0.87, str(o))
    check("障碍标签 heading 保留原始角度", o["heading"] == 100.0, str(o))
    check("障碍标签 id/vx/vy 透传",
          o["id"] == 1 and o["vx"] == 0.0 and o["vy"] == 0.0, str(o))

    check("refer 路径 2 点", d["paths"]["refer"] is not None and
          len(d["paths"]["refer"]) == 2)

    st = d["stop"]
    check("stop yaw=stopAngle 换算",
          abs(st["yaw"] - math.radians(0.0)) < 1e-3, str(st))

    check("pallet 减 origin",
          abs(d["pallet"]["x"] - (59.0 - d["origin"][0])) < 0.02)

    # 路径长度不等 -> 整帧丢弃(对标 thread.cpp:72)
    write_control(rates={"/navigation_msg": 10, "/perception": 5,
                         "/plan_path_msg": 5, "/refer_path_msg": 5,
                         "/path_plan_status": 5, "/palletpos": 5},
                  fields={"/plan_path_msg": {"x": [1.0, 2.0], "y": [1.0]}})
    time.sleep(0.8)
    d = snap()
    check("路径 x/y 不等长丢帧", d["paths"]["plan"] is None,
          str(d["paths"]["plan"]))

    # stopAngle=0 -> identity(对标 rviz C++ 忽略 stopAngle 的行为)
    write_control(rates={"/path_plan_status": 5},
                  fields={"/path_plan_status": {"stopX": 60.0,
                                                "stopY": -11.0,
                                                "stopAngle": 0.0}})
    time.sleep(0.8)
    check("stopAngle=0 -> yaw=0(identity)", snap()["stop"]["yaw"] == 0.0)


def t02b_statusbar():
    """右侧状态栏四段:task/plan/control/can 快照映射。"""
    print("== t02b 状态栏四段 ==")
    write_control(rates={"/task_plan_msg": 5, "/path_plan_status": 5,
                         "/cloud/task/task_status": 5, "/plan_path_msg": 5,
                         "/control_msg": 10, "/can_msg": 20})
    wait_for("task 段就位(真实数据而非 exec 兜底)",
             lambda: snap()["task"] and snap()["task"].get("id") == 88
             and snap()["task"].get("cloud_proc") is not None)
    d = snap()
    t_ = d["task"]
    check("任务: id/type/workMode", t_["id"] == 88 and t_["type"] == 1
          and t_["work_mode"] == 1, str(t_))
    check("任务: exec(0合法值不被or吞)", t_["exec"] == 1, str(t_["exec"]))
    check("任务: 云端 procedure/嵌套 task_id",
          t_["cloud_proc"] == 1 and t_["fail_code"] == 0, str(t_))
    check("规划: desireSpeed/planspeed/safety 精确锁定",
          d["plan"]["desire_speed"] == 1.8 and
          d["plan"]["planspeed"] == 1.6 and
          d["plan"]["safety"] is False, str(d["plan"]))
    co = d["control"]
    check("控制: 前轮转角/制动/油门", co["steer"] == -11.0 and
          co["brake"] == 0 and co["throttle"] == 18, str(co))
    ca = d["can"]
    check("CAN: 挡位/模式/电量/挂接(0-1域)/故障",
          ca["gear"] == 4 and ca["mode"] == 1 and ca["battery"] == 77
          and ca["hook"] == 1 and ca["fault"] == [0], str(ca))
    check("CAN: 方向盘转角(22倍于前轮基准)", ca["steer_fb"] == -220.0,
          str(ca["steer_fb"]))
    check("CAN: 扩展段(车速/制动反馈/限位/按钮/EPS/位置码)",
          ca["speed"] == 1.2 and ca["brake_fb"] == 30 and
          ca["link_pallet"] == 1 and ca["eab"] == 1 and
          ca["hook_btn"] == 1 and ca["link_btn"] == 2 and
          ca["eps_mode"] == 3 and ca["eps_current"] == -4.75 and
          ca["pin_pos"] == 185 and ca["seat_pos"] == 130, str(ca))
    check("数据龄含新话题", "/can_msg" in d["ages"] and
          "/control_msg" in d["ages"], str(list(d["ages"])[:12]))
    # 执行状态 0(无任务)与 procedure 0 不被 or 兜底吞
    write_control(rates={"/path_plan_status": 5, "/cloud/task/task_status": 5},
                  fields={"/path_plan_status": {"taskExecuStatus": 0},
                          "/cloud/task/task_status": {"procedure": 0}})
    time.sleep(0.8)
    d0 = snap()
    check("exec=0 透传(无任务)", d0["task"]["exec"] == 0)
    check("procedure=0 透传(不被吞成-1)",
          d0["task"]["cloud_proc"] == 0, str(d0["task"].get("cloud_proc")))
    write_control(rates={})


def t02c_gantry():
    """闸机口:三态快照映射(active/open)与 bool 化。"""
    print("== t02c 闸机口 ==")
    write_control(rates={"/gantry_state": 10})
    ok = wait_for("gantry 数据到位",
                  lambda: snap().get("gantry") is not None)
    if not ok:
        return
    d = snap()
    g = d["gantry"]
    # 默认字段 active=True/open=False -> 原样透传且为 bool
    check("关闭态: active/open 透传", g["active"] is True
          and g["open"] is False, str(g))
    check("数据龄含 /gantry_state", "/gantry_state" in d["ages"],
          str([k for k in d["ages"] if "gantry" in k]))
    # 开启态
    write_control(rates={"/gantry_state": 10},
                  fields={"/gantry_state": {"active": True,
                                            "gantry_open": True}})
    wait_for("开启态到位", lambda: snap()["gantry"]["open"] is True)
    check("开启态: open 透传", snap()["gantry"]["open"] is True)
    # 无效态(active=false)。注:mock 的 build_msg 把 DEFAULT_FIELDS 合并
    # 在 override 之下,无法构造"属性真缺失"的消息——getattr 默认值分支
    # 不在此覆盖(真实 catkin 类恒有全字段,该分支仅防御旧定义混部)
    write_control(rates={"/gantry_state": 10},
                  fields={"/gantry_state": {"active": False}})
    wait_for("无效态到位", lambda: snap()["gantry"]["active"] is False)
    d2 = snap()
    check("无效态: active=False 且 open=False 透传",
          d2["gantry"]["active"] is False and
          d2["gantry"]["open"] is False, str(d2["gantry"]))
    write_control(rates={})


def t03_scan():
    print("== t03 scan.bin ==")
    write_control(rates={"/back_left_scan": 5, "/back_right_scan": 5})
    wait_for("scan 数据到位",
             lambda: len(http("/api/scan.bin")[1]) > 20 and 1)
    _, blob = http("/api/scan.bin")
    d = decode_bin(blob)
    check("scan 帧 magic/2 通道", d is not None and len(d["channels"]) == 2,
          repr(blob[:8]))
    # 默认 ranges [2,2,inf,0,-1,2,2,2] -> 5 有效点(滤 inf/0/负)
    left = [c for c in d["channels"] if c["id"] == 1]
    check("左通道 5 点(inf/0/负滤除)", left and left[0]["count"] == 5,
          str(left and left[0]["count"]))
    if left and left[0]["count"] == 5:
        p0 = left[0]["pts"][0]     # angle=-pi: 传感器系 (-2, 0)
        # 外参 x=+1, yaw=90°: 旋转 90° 后 (0,-2) 再平移 (1,-2)
        check("外参旋转+平移", abs(p0[0] - 1.0) < 1e-6 and
              abs(p0[1] + 2.0) < 1e-6, str(p0))
        p1 = left[0]["pts"][1]     # angle=-3pi/4: (-1.414,-1.414)
        # 旋转90°: (1.414,-1.414) 平移: (2.414,-1.414)
        check("第二点外参", abs(p1[0] - (math.sqrt(2) + 1)) < 1e-3 and
              abs(p1[1] + math.sqrt(2)) < 1e-3, str(p1))
        check("intensity 截断填充(仅前2束有值)",
              left[0]["pts"][0][3] == 10 and left[0]["pts"][2][3] == 0)
    right = [c for c in d["channels"] if c["id"] == 2]
    check("右通道外参=0 直通", right and right[0]["count"] == 8 and
          abs(right[0]["pts"][0][0] + 2.0) < 1e-6,
          str(right and right[0]["count"]))


def t04_cloud_variants():
    print("== t04 cloud 解析变体 ==")
    base_pts = [[1.0, 2.0, 0.5, 100], [1.05, 2.0, 0.5, 90],
                [3.0, 4.0, 0.6, 80]]

    def run_variant(tag, opt, pts=None, expect_n=None, cond=None):
        """写变体 -> 等**帧切换**(ts 变化)且内容满足期望才断言。

        曾有假绿:写控制后立即轮询,拿到的还是上一变体的陈旧缓存,
        期望值恰与旧帧一致时直接 break,organized 的真实输出
        (4 点含幻影)从未被验证。现在必须等到缓存 ts 变化(即 worker
        完成了一次新解析)且 count/cond 满足,双条件防陈旧帧。
        """
        write_control(rates={"/rslidar_points_mid": 5},
                      fields={"/rslidar_points_mid": {
                          "__pc2__": pts or base_pts,
                          "__pc2_opt__": opt}})
        _, blob0 = http("/api/cloud.bin")
        d0 = decode_bin(blob0)
        ts0 = d0["ts"] if d0 else -1
        deadline = time.time() + 8
        got = None
        while time.time() < deadline:
            http("/api/cloud.bin")          # 保持活动(门控)
            _, blob = http("/api/cloud.bin")
            d = decode_bin(blob)
            if d and d["channels"] and d["ts"] != ts0:
                cand = d["channels"][0]
                ok = (expect_n is None or cand["count"] == expect_n)
                if ok and cond:
                    try:
                        ok = bool(cond(cand))
                    except Exception:
                        ok = False
                if ok:
                    got = cand
                    break
                got = cand
            time.sleep(0.4)
        check("%s(%d 点)" % (tag, got["count"] if got else -1),
              got is not None and (expect_n is None or
                                   got["count"] == expect_n))
        if cond and got:
            try:
                ok = bool(cond(got))
            except Exception:
                ok = False
            check(tag + " 数值", ok)

    def near(a, b, tol=1e-3):
        return abs(a - b) <= tol

    run_variant("小端 packed 快路径", {}, expect_n=3,
                cond=lambda c: near(c["pts"][0][0], 1.0) and
                c["pts"][0][3] == 100)
    run_variant("大端", {"bigendian": True}, expect_n=3,
                cond=lambda c: near(c["pts"][2][0], 3.0) and
                near(c["pts"][2][2], 0.6))
    run_variant("uint16 intensity(dtype=4)", {"dtype": {"intensity": 4}},
                pts=[[1.0, 2.0, 0.5, 300], [3.0, 4.0, 0.6, 80]],
                expect_n=2,
                cond=lambda c: c["pts"][0][3] == 255,
                )
    run_variant("非 packed offset", {"point_step": 32,
                                     "offsets": {"x": 4, "y": 8, "z": 12,
                                                 "intensity": 28}},
                expect_n=3,
                cond=lambda c: near(c["pts"][0][0], 1.0))
    run_variant("organized(height=2)", {"organized": 2}, expect_n=3)
    run_variant("NaN 过滤", {}, pts=[[float("nan"), 1.0, 0.5, 10],
                                      [1.0, 2.0, 0.5, 20]],
                expect_n=1)
    run_variant("z 越界过滤", {}, pts=[[1.0, 1.0, 5.0, 10],
                                        [1.0, 2.0, 0.5, 20]],
                expect_n=1)
    run_variant("上限截断(max=4)", {}, pts=[[float(i) * 0.5, 0.0, 0.1, 5]
                                             for i in range(10)],
                expect_n=4)
    # 恢复默认
    write_control(rates={"/rslidar_points_mid": 5})


def t05_byte_lock():
    print("== t05 bin 字节锁 ==")
    # fixture: 单点 (1.5, -2.25, 0.5, 200) —— 与 frontend_test.js 共用
    write_control(rates={"/rslidar_points_mid": 5},
                  fields={"/rslidar_points_mid": {
                      "__pc2__": [[1.5, -2.25, 0.5, 200]], "__pc2_opt__": {}}})
    deadline = time.time() + 6
    blob = b""
    while time.time() < deadline:
        http("/api/cloud.bin")
        _, blob = http("/api/cloud.bin")
        if len(blob) >= 20 + 8 + 12:
            d = decode_bin(blob)
            if d and d["channels"] and d["channels"][0]["count"] == 1:
                break
        time.sleep(0.4)
    check("字节锁:头", blob[:4] == b"QWMC" and blob[4:6] == b"\x01\x00"
          and blob[6:8] == b"\x01\x00" and blob[8:12] == b"\x01\x00\x00\x00",
          blob[:20].hex())
    point_bytes = blob[28:40]
    check("字节锁:点",
          point_bytes == struct.pack("<iihBB", 1500, -2250, 500, 200, 0),
          point_bytes.hex())
    write_control(rates={})


def t06_ages():
    print("== t06 数据龄 ==")
    # 先全停让旧数据老化,再开泵等 ages 回落(避免上一场景残留干扰)
    write_control(rates={})
    time.sleep(1.0)
    write_control(rates={"/navigation_msg": 10})
    wait_for("ages 回落",
             lambda: snap()["ages"].get("/navigation_msg") is not None
             and snap()["ages"]["/navigation_msg"] < 2 and 1, timeout=15)
    check("活动时 ages < 2", snap()["ages"]["/navigation_msg"] < 2)
    write_control(rates={})          # 全停
    time.sleep(6.5)
    a = snap()["ages"]["/navigation_msg"]
    check("停泵后 ages > 5(灰化线)", a is not None and a > 5, str(a))


def t07_concurrent():
    print("== t07 并发 ==")
    write_control(rates={"/navigation_msg": 20, "/back_left_scan": 10,
                         "/rslidar_points_mid": 10})
    errors = []

    def worker(path):
        for _ in range(20):
            try:
                code, _ = http(path, timeout=8)
                if code != 200:
                    errors.append((path, code))
            except Exception as exc:
                errors.append((path, str(exc)))

    ths = [threading.Thread(target=worker, args=(p,)) for p in
           ["/api/snapshot", "/api/map", "/api/scan.bin", "/api/cloud.bin"] *
           2]
    for t in ths:
        t.start()
    for t in ths:
        t.join()
    check("8 线程 x 20 次混合请求零错误", not errors, str(errors[:3]))


def t08_rss():
    print("== t08 RSS 有界 ==")
    big = [[float(i % 50) * 0.25, float(i % 37) * 0.25, 0.1, 50]
           for i in range(300)]
    write_control(rates={"/rslidar_points_mid": 50},
                  fields={"/rslidar_points_mid": {"__pc2__": big,
                                                   "__pc2_opt__": {}}})
    rss0 = None
    deadline = time.time() + 5
    while time.time() < deadline:
        http("/api/cloud.bin")
        d = decode_bin(http("/api/cloud.bin")[1])
        if d and d["channels"] and d["channels"][0]["count"] == 4:
            break
        time.sleep(0.3)
    rss0 = rss_kb(SRV_PID)
    for _ in range(60):
        http("/api/cloud.bin")
        http("/api/snapshot")
    rss1 = rss_kb(SRV_PID)
    check("RSS 有界(<15MB 增长)", rss0 and rss1 and rss1 - rss0 < 15360,
          "%s -> %s KB" % (rss0, rss1))


def t10_master_restart():
    """master pid 变化 -> 三类订阅(typed/scan/cloud)重建,数据恢复。"""
    print("== t10 master 重启重建 ==")
    write_control(rates={"/navigation_msg": 10, "/back_left_scan": 5,
                         "/rslidar_points_mid": 5},
                  fields={"/navigation_msg": {"xAxis": 55.94}})
    wait_for("数据就位", lambda: snap()["vehicle"] and 1)
    # 改 pid 模拟 master 重启;5s 探活周期后应触发重建并恢复
    write_control(rates={"/navigation_msg": 10, "/back_left_scan": 5,
                         "/rslidar_points_mid": 5},
                  master_pid=9999)
    time.sleep(7.5)                       # 探活周期 5s + 余量
    d = snap()
    check("master 重启后 master_ok=true", d["master_ok"] is True)
    check("重启后 typed 数据恢复", d["vehicle"] is not None)
    # scan 通道恢复
    _, blob = http("/api/scan.bin")
    dd = decode_bin(blob)
    check("重启后 scan 数据恢复",
          dd is not None and len(dd["channels"]) == 2 and
          dd["channels"][0]["count"] > 0,
          str(dd and [(c["id"], c["count"]) for c in dd["channels"]]))
    # master 失联 -> master_ok=false(数据冻结但服务不崩)
    write_control(rates={"/navigation_msg": 10, "/back_left_scan": 5,
                         "/rslidar_points_mid": 5},
                  master_alive=False)
    time.sleep(7.5)
    d = snap()
    check("master 失联 -> master_ok=false", d["master_ok"] is False)
    check("失联期间服务仍响应", d["ros_available"] is True)
    write_control(rates={"/navigation_msg": 10, "/back_left_scan": 5,
                         "/rslidar_points_mid": 5})


def t11_nan_safety():
    """NaN 注入 -> snapshot 不得输出非法 JSON(Python 端能解析但浏览器
    不能;曾致整页瘫痪)。回归 H1。"""
    print("== t11 NaN 安全 ==")
    write_control(rates={"/navigation_msg": 10, "/perception": 5},
                  fields={"/navigation_msg": {"xAxis": float("nan"),
                                              "heading": float("inf"),
                                              "gpsSpeed": float("nan")},
                          "/perception": {"objs": [
                              {"id": 1, "x": float("nan"), "y": 2.0,
                               "dx": 1.0, "dy": 4.0, "heading": float("nan"),
                               "height": 1.8}]}})
    # 等 NaN 帧真正生效:特征是 xAxis 归 0(旧帧是 6.71)
    wait_for("NaN 帧生效(x 归 0)",
             lambda: snap()["vehicle"] and
             snap()["vehicle"]["x"] == 0.0 and 1, timeout=15)
    code, body = http("/api/snapshot")
    raw = body.decode("utf-8", "replace")
    check("响应体不含裸 NaN/Infinity", ("NaN" not in raw) and
          ("Infinity" not in raw), raw[:200])
    d = json.loads(raw)                    # 标准 JSON 可解析
    check("NaN 字段归 0", d["vehicle"]["x"] == 0.0 and
          d["vehicle"]["speed"] == 0.0, str(d["vehicle"]))
    check("障碍 NaN 归 0", d["obstacles"][0]["x"] == 0.0 and
          d["obstacles"][0]["yaw"] == 0.0 and
          d["obstacles"][0]["heading"] == 0.0)
    # scan intensity NaN/Inf 不崩
    write_control(rates={"/back_left_scan": 5},
                  fields={"/back_left_scan": {"__scan__": {
                      "amin": -1.0, "ainc": 0.5, "rmin": 0.1, "rmax": 100,
                      "ranges": [2.0, 2.0], "intensities":
                          [float("nan"), float("inf")]}}})
    time.sleep(1.0)
    code, blob = http("/api/scan.bin")
    check("scan NaN/Inf intensity 不崩", code == 200 and
          decode_bin(blob) is not None and
          decode_bin(blob)["channels"][0]["count"] == 2)


def t09_py38():
    print("== t09 py3.8 兼容 ==")
    names = ["monitor_server.py", "ros_visualizer.py", "monitor_config.py",
             "tests/mock_ros.py", "tests/run_server_mock.py",
             "tests/test_full.py", "tests/ros_test_config.py"]
    for n in names:
        try:
            with open(os.path.join(MON, n)) as f:
                ast.parse(f.read(), feature_version=(3, 8))
            check("ast(3,8) %s" % n, True)
        except SyntaxError as exc:
            check("ast(3,8) %s" % n, False, str(exc))


# ------------------------------------------------------------------ 主流程

SRV_PID = None


def main():
    global SRV_PID
    sys.path.insert(0, HERE)
    import mock_ros
    master = mock_ros.FakeMaster(port=21111)
    master.start()
    write_control(rates={})
    env = dict(os.environ)
    env["MOCK_CONTROL"] = CONTROL
    env["ROS_MASTER_URI"] = "http://127.0.0.1:21111"
    env["DASHBOARD_PORT"] = "18082"       # 避让本机可能在跑的 8082
    srv = subprocess.Popen(
        [sys.executable, os.path.join(HERE, "run_server_mock.py"),
         "--port", str(PORT), "--config",
         os.path.join(HERE, "ros_test_config.py")],
        cwd=HERE, env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    SRV_PID = srv.pid
    try:
        up = wait_for("服务就绪", lambda: snap()["ros_available"] is True,
                      timeout=15)
        if up:
            for fn in (t01_static, t02_conversions, t02b_statusbar,
                       t02c_gantry, t03_scan,
                       t04_cloud_variants, t05_byte_lock, t06_ages,
                       t07_concurrent, t08_rss, t10_master_restart,
                       t11_nan_safety, t09_py38):
                try:
                    fn()
                except Exception as exc:
                    check(fn.__name__ + " 异常", False, repr(exc))
        print("\n==== %d passed, %d failed ====" % (PASS, len(FAILS)))
        if FAILS:
            for f in FAILS:
                print("  FAILED:", f)
    finally:
        srv.terminate()
        try:
            srv.wait(timeout=5)
        except subprocess.TimeoutExpired:
            srv.kill()
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
