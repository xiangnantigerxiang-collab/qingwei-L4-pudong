# -*- coding: utf-8 -*-
"""
monitor 混沌/浸泡测试(开发机用)——对标 hmi/tests/chaos_test.py 思路
=====================================================================

与 test_full.py(确定性场景)不同,本套是**随机动态压力**:
  c1 随机数据流浸泡(60s):rates/fields 每 1s 随机翻转(NaN/Inf/不等长/
     坏类型注入),3 并发客户端持续拉取,全程不变量断言
  c2 服务 SIGKILL 暴毙重启:客户端侧恢复
  c3 master 风暴(20s):pid 每秒跳变 + 随机失联,服务不崩
  c4 控制文件写风暴(10s):50ms 间隔重写(泵重载放大)
  c5 收尾:RSS/fd/存活断言

不变量(每轮拉取都查):
  I1 /api/snapshot 恒 200 且响应体不含裸 NaN/Infinity(非法 JSON)
  I2 bin 帧可解码,magic 正确,total == 各通道点数之和
  I3 全程服务进程存活,退出码未变
  I4 浸泡结束 RSS 增长 < 20MB,fd 增长 < 40

用法: cd monitor && python3 tests/chaos_test.py   (约 2 分钟)
"""

import json
import math
import os
import random
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
MON = os.path.dirname(HERE)
CONTROL = "/tmp/monitor_chaos_control.json"
PORT = 18085
BASE = "http://127.0.0.1:%d" % PORT

sys.path.insert(0, HERE)
import mock_ros          # noqa: E402

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


def http(path, timeout=8):
    try:
        r = urllib.request.urlopen(BASE + path, timeout=timeout)
        return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()
    except Exception as e:
        return -1, str(e).encode()


def write_control(obj):
    tmp = CONTROL + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f)
    os.replace(tmp, CONTROL)


def decode_bin(blob):
    if len(blob) < 20:
        return None
    magic, ver, nchan, total, ts = struct.unpack_from("<4sHHIQ", blob, 0)
    if magic != b"QWMC":
        return None
    off = 20
    total_seen = 0
    for _ in range(nchan):
        if off + 8 > len(blob):
            return None
        _, _, _, cnt = struct.unpack_from("<BBHI", blob, off)
        off += 8 + cnt * 12
        total_seen += cnt
    if total_seen != total:
        return None
    return {"total": total, "nchan": nchan, "ts": ts}


def rss_and_fds(pid):
    rss = None
    try:
        with open("/proc/%d/status" % pid) as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    rss = int(line.split()[1])
    except OSError:
        pass
    try:
        fds = len(os.listdir("/proc/%d/fd" % pid))
    except OSError:
        fds = None
    return rss, fds


# ---------------------------------------------------------------- 随机数据

def rand_fields(rng):
    """构造一轮随机 fields:覆盖所有话题,随机注入脏值。"""
    f = {}
    if rng.random() < 0.8:
        nav = {"xAxis": rng.uniform(0, 120), "yAxis": rng.uniform(-160, 0),
               "heading": rng.uniform(-720, 1440),
               "gpsSpeed": rng.uniform(-2, 15)}
        for k in list(nav):                       # 10% 概率单字段脏值
            if rng.random() < 0.10:
                nav[k] = rng.choice([float("nan"), float("inf"),
                                     float("-inf"), "abc", None])
        f["/navigation_msg"] = nav
    if rng.random() < 0.7:
        n = rng.randint(0, 8)
        objs = []
        for i in range(n):
            o = {"id": i, "x": rng.uniform(0, 120), "y": rng.uniform(-160, 0),
                 "dx": rng.uniform(0, 5), "dy": rng.uniform(0, 5),
                 "heading": rng.uniform(-720, 1440),
                 "height": rng.uniform(0, 3), "vx": 0.0, "vy": 0.0}
            if rng.random() < 0.10:
                o[rng.choice(["x", "y", "dx", "dy", "heading"])] = \
                    rng.choice([float("nan"), float("inf"), "x"])
            objs.append(o)
        f["/perception"] = {"objs": objs}
    for topic in ("/plan_path_msg", "/refer_path_msg"):
        if rng.random() < 0.6:
            n = rng.randint(0, 50)
            xs = [rng.uniform(0, 120) for _ in range(n)]
            ys = [rng.uniform(-160, 0) for _ in range(n)]
            if rng.random() < 0.2:               # 不等长 -> 丢帧路径
                ys = ys[:max(0, n - rng.randint(1, 3))]
            if rng.random() < 0.05:              # 字符串混入
                xs = xs + ["bad"] if xs else ["bad"]
            f[topic] = {"x": xs, "y": ys}
    if rng.random() < 0.5:
        rmin, rmax = 0.1, 100.0
        ranges = []
        for _ in range(rng.randint(0, 60)):
            pick = rng.random()
            if pick < 0.15:
                ranges.append(rng.choice([float("inf"), float("nan"),
                                          0.0, -1.0, 1e9]))
            else:
                ranges.append(rng.uniform(rmin, 50))
        f["/back_left_scan"] = {"__scan__": {
            "amin": rng.uniform(-math.pi, 0),
            "ainc": math.pi / rng.choice([2, 4, 8, 16]),
            "rmin": rmin, "rmax": rmax, "ranges": ranges,
            "intensities": [rng.choice([0, rng.uniform(0, 300), 1e9,
                                        float("nan")])
                            for _ in ranges]}}
    if rng.random() < 0.7:
        npts = rng.randint(0, 200)
        pts = [[rng.uniform(-80, 80), rng.uniform(-80, 80),
                rng.uniform(-3, 5),
                rng.choice([0, rng.uniform(0, 255), 1e12, float("nan")])]
               for _ in range(npts)]
        opt = {}
        if rng.random() < 0.2:
            opt["bigendian"] = True
        if rng.random() < 0.2:
            opt["dtype"] = {"intensity": rng.choice([4, 6, 8])}
        if rng.random() < 0.2:
            opt["point_step"] = rng.choice([16, 32])
            if opt["point_step"] == 32:
                opt["offsets"] = {"x": 4, "y": 8, "z": 12, "intensity": 28}
        if rng.random() < 0.15:
            opt["organized"] = rng.randint(2, 4)
        f["/rslidar_points_mid"] = {"__pc2__": pts, "__pc2_opt__": opt}
    return f


def rand_rates(rng):
    rates = {}
    for t in ("/navigation_msg", "/perception", "/plan_path_msg",
              "/refer_path_msg", "/path_plan_status", "/palletpos",
              "/back_left_scan", "/back_right_scan", "/rslidar_points_mid"):
        pick = rng.random()
        if pick < 0.25:
            rates[t] = 0                # 随机停流(测 ages/冻结)
        elif pick < 0.85:
            rates[t] = rng.uniform(1, 50)
        else:
            rates[t] = rng.uniform(50, 120)   # 超高频
    return rates


# ---------------------------------------------------------------- 客户端

class Client(threading.Thread):
    """并发客户端:循环拉取并做不变量断言(错误收集后统一报)。"""

    def __init__(self, name, mix):
        threading.Thread.__init__(self, daemon=True)
        self.name = name
        self.mix = mix        # 轮换路径列表
        self.errors = []
        self.count = 0
        self.stop = threading.Event()

    def run(self):
        i = 0
        while not self.stop.is_set():
            path = self.mix[i % len(self.mix)]
            i += 1
            code, body = http(path)
            self.count += 1
            if code != 200:
                self.errors.append("%s %s -> %s %r" %
                                   (self.name, path, code, body[:80]))
                continue
            if path == "/api/snapshot":
                raw = body.decode("utf-8", "replace")
                if "NaN" in raw or "Infinity" in raw:
                    self.errors.append("%s snapshot 含裸 NaN/Inf" % self.name)
                else:
                    try:
                        json.loads(raw)
                    except ValueError as e:
                        self.errors.append("%s snapshot 不可解析 %s" %
                                           (self.name, e))
            elif path.endswith(".bin"):
                if decode_bin(body) is None:
                    self.errors.append("%s %s bin 帧损坏(len=%d)" %
                                       (self.name, path, len(body)))
            time.sleep(0.05)


# ---------------------------------------------------------------- 场景

def spawn_server(env):
    return subprocess.Popen(
        [sys.executable, os.path.join(HERE, "run_server_mock.py"),
         "--port", str(PORT), "--config",
         os.path.join(HERE, "ros_test_config.py")],
        cwd=HERE, env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)


def main():
    rng = random.Random(20260828)
    master = mock_ros.FakeMaster(port=11311)
    master.start()
    write_control({"master_alive": True, "rates": {}, "fields": {}})
    env = dict(os.environ)
    env["MOCK_CONTROL"] = CONTROL
    env["ROS_MASTER_URI"] = "http://127.0.0.1:11311"
    srv = spawn_server(env)

    up = False
    for _ in range(30):
        code, _ = http("/api/snapshot")
        if code == 200:
            up = True
            break
        time.sleep(0.4)
    check("服务启动", up)
    if not up:
        return 1

    # ---- c1 随机浸泡 60s ----
    print("== c1 随机数据流浸泡 60s ==")
    clients = [Client("c%d" % i, mix) for i, mix in enumerate([
        ["/api/snapshot"],
        ["/api/snapshot", "/api/scan.bin"],
        ["/api/snapshot", "/api/cloud.bin", "/api/map"],
    ])]
    for c in clients:
        c.start()
    rss0, fds0 = rss_and_fds(srv.pid)
    t0 = time.time()
    flips = 0
    while time.time() - t0 < 60:
        write_control({"master_alive": True, "rates": rand_rates(rng),
                       "fields": rand_fields(rng)})
        flips += 1
        if srv.poll() is not None:
            check("浸泡期间服务存活", False,
                  "exit=%s at %.0fs" % (srv.returncode, time.time() - t0))
            break
        time.sleep(1.0)
    for c in clients:
        c.stop.set()
    for c in clients:
        c.join(timeout=5)
    check("浸泡期间服务存活", srv.poll() is None)
    errs = [e for c in clients for e in c.errors]
    check("3 客户端 x 60s 零不变量违例(共 %d 次拉取)" %
          sum(c.count for c in clients), not errs, "; ".join(errs[:5]))
    rss1, fds1 = rss_and_fds(srv.pid)
    check("RSS 增长 < 20MB", rss0 is not None and rss1 is not None and
          rss1 - rss0 < 20480, "%s -> %s KB" % (rss0, rss1))
    check("fd 增长 < 40", fds0 is not None and fds1 is not None and
          fds1 - fds0 < 40, "%s -> %s" % (fds0, fds1))
    print("     (配置翻转 %d 轮,客户端拉取 %d 次)" %
          (flips, sum(c.count for c in clients)))

    # ---- c3 master 风暴 20s ----
    print("== c3 master 重启/失联风暴 20s ==")
    write_control({"master_alive": True, "rates": rand_rates(rng),
                   "fields": rand_fields(rng)})
    t0 = time.time()
    seq = 0
    while time.time() - t0 < 20:
        seq += 1
        if seq % 2:
            write_control({"master_alive": True, "master_pid": 10000 + seq,
                           "rates": rand_rates(rng),
                           "fields": rand_fields(rng)})
        else:
            write_control({"master_alive": False,
                           "rates": rand_rates(rng),
                           "fields": rand_fields(rng)})
        time.sleep(1.0)
    write_control({"master_alive": True, "master_pid": 31337,
                   "rates": rand_rates(rng), "fields": rand_fields(rng)})
    time.sleep(8)                       # 等 5s 探活周期 + 重建
    code, body = http("/api/snapshot")
    d = json.loads(body.decode("utf-8"))
    check("风暴后服务存活且响应", code == 200 and srv.poll() is None)
    check("风暴后 master_ok 恢复 true", d["master_ok"] is True,
          str(d["master_ok"]))

    # ---- c4 控制文件写风暴 10s ----
    print("== c4 控制文件写风暴 10s ==")
    t0 = time.time()
    writes = 0
    while time.time() - t0 < 10:
        write_control({"master_alive": True, "rates": rand_rates(rng),
                       "fields": rand_fields(rng)})
        writes += 1
        time.sleep(0.05)
    code, _ = http("/api/snapshot")
    check("写风暴后服务存活(%d 次写)" % writes,
          code == 200 and srv.poll() is None)

    # ---- c2 SIGKILL 暴毙重启 ----
    print("== c2 服务 SIGKILL 暴毙重启 ==")
    srv.kill()
    srv.wait(timeout=5)
    for _ in range(20):
        code, _ = http("/api/snapshot")
        if code == -1:
            break
        time.sleep(0.2)
    check("暴毙后端口释放", http("/api/snapshot")[0] == -1)
    srv = spawn_server(env)
    up = False
    for _ in range(30):
        code, _ = http("/api/snapshot")
        if code == 200:
            up = True
            break
        time.sleep(0.4)
    check("重启后服务恢复", up)
    # 数据链路恢复(等订阅重建+泵)
    time.sleep(3)
    code, body = http("/api/snapshot")
    d = json.loads(body.decode("utf-8"))
    check("重启后数据链路恢复", d["ros_available"] is True and
          d["master_ok"] is True, str(d.get("master_ok")))

    srv.terminate()
    try:
        srv.wait(timeout=5)
    except subprocess.TimeoutExpired:
        srv.kill()

    print("\n==== %d passed, %d failed ====" % (PASS, len(FAILS)))
    for f in FAILS:
        print("  FAILED:", f)
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
