# -*- coding: utf-8 -*-
"""
monitor 协议双解码对账 fuzz(开发机用)
=====================================

test_full t05 只锁过单点 fixture;本套对**随机数据**验证打包->两种解码器
的一致性(服务端 struct 打包,Python/JS 各自独立解码实现):

  随机点集(负坐标/高强度/z 钳位边界) x 50 轮
    -> mock PC2 -> 服务 /api/cloud.bin
    -> Python decode_bin(测试侧第三实现) == JS MM.decodeBin(页面真实现)

JS 侧从 static/index.html 切片提取 MonitorMath 段(到 window.MM 赋值为止)
在 node 里 eval 后直调,与浏览器/前端测试同一份代码。

用法: cd monitor && python3 tests/fuzz_proto.py   (约 1 分钟)
"""

import json
import math
import os
import random
import struct
import subprocess
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
MON = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import mock_ros          # noqa: E402

CONTROL = "/tmp/monitor_fuzz_control.json"
PORT = 18086
BASE = "http://127.0.0.1:%d" % PORT

PASS = 0
FAILS = []


def check(name, cond, detail=""):
    global PASS
    if cond:
        PASS += 1
    else:
        FAILS.append(name)
        print("  FAIL %s  %s" % (name, detail))


# JS 侧:提取 MM 段并构造单次解码 runner(每次调用起 node 太慢,改为
# 一次性起常驻 node 子进程,stdin 逐行喂 base64,stdout 回 JSON)
JS_RUNNER = r"""
"use strict";
const fs = require("fs");
const html = fs.readFileSync(%r, "utf8");
const m = html.match(/<script>\s*([\s\S]*?)<\/script>/);
// MonitorMath 段:从脚本开头到 window.MM 赋值(不含场景装配/IO)
const full = m[1];
const cut = full.indexOf("window.MM = MM;");
const mmSrc = full.slice(0, cut) + "\nwindow.MM = MM;";
global.window = {};
eval(mmSrc);
const MM = window.MM;
let buf = "";
process.stdin.on("data", (d) => {
  buf += d.toString();
  let idx;
  while ((idx = buf.indexOf("\n")) >= 0) {
    const line = buf.slice(0, idx); buf = buf.slice(idx + 1);
    if (!line.trim()) continue;
    const ab = Buffer.from(line, "base64");
    const ab2 = ab.buffer.slice(ab.byteOffset, ab.byteOffset + ab.byteLength);
    const d2 = MM.decodeBin(ab2);
    if (!d2) { console.log(JSON.stringify(null)); continue; }
    console.log(JSON.stringify({
      total: d2.total,
      chans: d2.channels.map((c) => ({ id: c.id, n: c.count,
        pts: Array.from(c.pts).map((v) => Math.round(v * 1e6) / 1e6) }))
    }));
  }
});
"""


def py_decode(blob):
    """Python 侧独立解码(与 test_full 同款)。"""
    if len(blob) < 20:
        return None
    magic, ver, nchan, total, ts = struct.unpack_from("<4sHHIQ", blob, 0)
    if magic != b"QWMC":
        return None
    off = 20
    chans = []
    for _ in range(nchan):
        cid, _, _, cnt = struct.unpack_from("<BBHI", blob, off)
        off += 8
        pts = []
        for _ in range(cnt):
            x, y, z, i, _ = struct.unpack_from("<iihBB", blob, off)
            off += 12
            pts.append((round(x / 1000.0, 6), round(z / 1000.0, 6),
                        round(-y / 1000.0, 6)))
        chans.append({"id": cid, "n": cnt, "pts": pts})
    return {"total": total, "chans": chans, "ts": ts}


def main():
    rng = random.Random(424242)
    master = mock_ros.FakeMaster(port=11311)
    master.start()
    with open(CONTROL, "w") as f:
        json.dump({"master_alive": True, "rates": {}, "fields": {}}, f)
    env = dict(os.environ)
    env["MOCK_CONTROL"] = CONTROL
    env["ROS_MASTER_URI"] = "http://127.0.0.1:11311"
    srv = subprocess.Popen(
        [sys.executable, os.path.join(HERE, "run_server_mock.py"),
         "--port", str(PORT), "--config",
         os.path.join(HERE, "ros_test_config.py")],
        cwd=HERE, env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    # 常驻 node 解码器
    runner_path = "/tmp/monitor_fuzz_runner.js"
    with open(runner_path, "w") as f:
        f.write(JS_RUNNER % os.path.join(MON, "static", "index.html"))
    node = subprocess.Popen(["node", runner_path],
                            stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

    def js_decode(blob_b64):
        node.stdin.write(blob_b64.encode() + b"\n")
        node.stdin.flush()
        line = node.stdout.readline().decode()
        return json.loads(line) if line.strip() else None

    rounds = 50
    mismatches = 0
    checked_pts = 0
    try:
        up = False
        for _ in range(30):
            try:
                urllib.request.urlopen(BASE + "/api/snapshot", timeout=3)
                up = True
                break
            except Exception:
                time.sleep(0.4)
        check("服务启动", up)
        if not up:
            return 1

        import base64
        for r in range(rounds):
            npts = rng.randint(0, 60)
            pts = [[rng.uniform(-90, 90), rng.uniform(-90, 90),
                    rng.uniform(-4, 6),
                    rng.choice([0, rng.uniform(0, 255), rng.uniform(255, 1e4),
                                -5.0])]
                   for _ in range(npts)]
            opt = {}
            if rng.random() < 0.3:
                opt["bigendian"] = True
            if rng.random() < 0.3:
                opt["dtype"] = {"intensity": 4}
            if rng.random() < 0.2 and "bigendian" not in opt:
                opt["point_step"] = 32
                opt["offsets"] = {"x": 4, "y": 8, "z": 12, "intensity": 28}
            ctl = {"master_alive": True,
                   "rates": {"/rslidar_points_mid": 5},
                   "fields": {"/rslidar_points_mid": {"__pc2__": pts,
                                                      "__pc2_opt__": opt}}}
            tmp = CONTROL + ".tmp"
            with open(tmp, "w") as f:
                json.dump(ctl, f)
            os.replace(tmp, CONTROL)

            # 等帧切换(参照 test_full run_variant 的防陈旧缓存做法):
            # 记录当前 ts,轮询到 ts 变化
            def fetch():
                return urllib.request.urlopen(BASE + "/api/cloud.bin",
                                              timeout=5).read()
            fetch()          # 保持活动
            d0 = py_decode(fetch())
            ts0 = d0["ts"] if d0 else -1
            blob = b""
            deadline = time.time() + 6
            while time.time() < deadline:
                fetch()
                blob = fetch()
                d = py_decode(blob)
                if d and d["ts"] != ts0 and d["chans"]:
                    break
                time.sleep(0.3)

            dpy = py_decode(blob)
            djs = js_decode(base64.b64encode(blob).decode())
            ok = (dpy is not None and djs is not None and
                  dpy["total"] == djs["total"] and
                  len(dpy["chans"]) == len(djs["chans"]))
            if ok:
                for cp, cj in zip(dpy["chans"], djs["chans"]):
                    # js pts 是平面数组 [x,y,z,...](rosToThree 后),
                    # py 侧同序 (x, z, -y)。容差 5e-6(0.005mm):覆盖两侧
                    # 输出前舍入方式不同(py round(v,6) vs js
                    # Math.round(v*1e6)/1e6)产生的 ±1e-6 量化差,
                    # 协议语义(字节->坐标)本身逐位一致
                    if cp["n"] != cj["n"] or cp["id"] != cj["id"]:
                        ok = False
                        break
                    flat = cj["pts"]
                    for k in range(cp["n"]):
                        if (abs(cp["pts"][k][0] - flat[k * 3]) > 5e-6 or
                                abs(cp["pts"][k][1] - flat[k * 3 + 1]) > 5e-6 or
                                abs(cp["pts"][k][2] - flat[k * 3 + 2]) > 5e-6):
                            ok = False
                            break
                    if not ok:
                        break
                    checked_pts += cp["n"]
            if not ok:
                mismatches += 1
                if mismatches <= 3:
                    print("  第 %d 轮不一致: py=%r js=%r" %
                          (r, dpy and dpy["total"], djs and djs["total"]))
        check("50 轮随机数据双解码零不一致(对账 %d 点)" % checked_pts,
              mismatches == 0, "mismatches=%d" % mismatches)
    finally:
        node.kill()
        srv.terminate()
        try:
            srv.wait(timeout=5)
        except subprocess.TimeoutExpired:
            srv.kill()
        os.path.exists(runner_path) and os.unlink(runner_path)

    print("\n==== %d passed, %d failed ====" % (PASS, len(FAILS)))
    for f in FAILS:
        print("  FAILED:", f)
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
