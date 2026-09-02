# -*- coding: utf-8 -*-
"""
混沌 / 浸泡 / 故障注入校验(无 ROS,纯进程层)
==============================================

场景:
  A  坏命令门控:Popen 失败 → CRASHED → 一键启动中止,后续组不动
  B  忽略 SIGTERM:停止链升级 SIGKILL,限时完成,无残留
  C  同组孙进程:killpg 收割完整性
  D  逃离进程组:残留检测 stop_failed=True(设计行为)
  E  日志洪水:轮转生效 + 并发日志读取不炸
  F  服务器 SIGKILL 后重启:外部进程标记 foreign;全部停止可清理外部进程
  G  快速启停翻转 ×15:状态机合法、无僵尸、fd 不泄漏
  H  停止中再操作:STOPPING 期间 restart → 409
  I  浸泡 60s:4 线程持续拉状态 + 周期重启,RSS/fd 增长受控
  J  磁盘水位(进程内 monkeypatch):<500MB 删轮转副本并降上限

用法:python3 tests/chaos_test.py
"""

import json
import os
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
HMI_DIR = os.path.dirname(HERE)
sys.path.insert(0, HMI_DIR)

PORT = None
PROC = None
PASS = 0
FAILS = []


def ok(name):
    global PASS
    PASS += 1
    print("  ✓ %s" % name)


def check(name, cond, detail=""):
    if cond:
        ok(name)
    else:
        FAILS.append(name)
        print("  ✗ %s  %s" % (name, detail))


def free_port():
    import socket
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def http(method, path, body=None, timeout=15):
    url = "http://127.0.0.1:%d%s" % (PORT, path)
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, method=method, data=data)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode() or "{}")
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read().decode() or "{}")
        except ValueError:
            return e.code, {}


def state():
    c, d = http("GET", "/api/state")
    assert c == 200, c
    return d


def comp(name):
    for c in state()["components"]:
        if c["name"] == name:
            return c
    raise KeyError(name)


def wait_for(name, fn, timeout=30.0):
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        try:
            last = fn()
            if last:
                return True
        except Exception as exc:
            last = exc
        time.sleep(0.4)
    check(name, False, "超时 last=%r" % (last,))
    return False


def pgrep(pat):
    r = subprocess.run(["pgrep", "-f", pat], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
    return r.returncode == 0


def server_pid():
    return PROC.pid


def fd_count():
    return len(os.listdir("/proc/%d/fd" % server_pid()))


def rss_mb():
    with open("/proc/%d/status" % server_pid()) as f:
        for line in f:
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024.0
    return None


def defunct_children():
    r = subprocess.run(["ps", "-o", "stat=", "--ppid", str(server_pid())],
                       capture_output=True, text=True)
    return sum(1 for ln in r.stdout.splitlines() if "Z" in ln)


def start_server():
    global PORT, PROC
    PORT = free_port()
    PROC = subprocess.Popen(
        [sys.executable, "hmi_server.py",
         "--config", "tests/chaos_config.py", "--port", str(PORT)],
        cwd=HMI_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    wait_for("服务就绪", lambda: http("GET", "/api/state")[0] == 200, 20)


def stop_server(sigkill=False):
    PROC.kill() if sigkill else PROC.terminate()
    try:
        PROC.wait(timeout=8)
    except Exception:
        PROC.kill()
        PROC.wait()


# ---------------------------------------------------------------- 场景

def t_a_gate():
    print("[A] 坏命令门控 → 一键启动中止")
    http("POST", "/api/start")
    wait_for("gate CRASHED", lambda: comp("gate")["state"] == "CRASHED", 15)
    g = comp("gate")
    check("Popen 失败有错误信息", "启动失败" in (g["last_error"] or ""),
          str(g["last_error"]))
    wait_for("编排结束(failed 含 gate)",
             lambda: state()["sequence"]["failed"] == ["gate"], 15)
    check("中止提示", "中止" in (state()["sequence"]["note"] or ""),
          str(state()["sequence"]["note"]))
    check("后续组未启动", comp("stubborn")["state"] == "STOPPED"
          and comp("flapper")["state"] == "STOPPED")


def t_b_stubborn():
    print("[B] 忽略 SIGTERM → SIGKILL 升级")
    http("POST", "/api/components/stubborn/start")
    wait_for("stubborn RUNNING", lambda: comp("stubborn")["state"] == "RUNNING", 15)
    t0 = time.time()
    code, _r = http("POST", "/api/components/stubborn/stop")
    check("stop 受理", code == 200)
    wait_for("限时内 STOPPED", lambda: comp("stubborn")["state"] == "STOPPED",
             timeout=30)
    dt = time.time() - t0
    check("停止耗时 < 20s(实际 %.1fs)" % dt, dt < 20)
    time.sleep(1)
    check("进程无残留", not pgrep("trap '' TERM"))


def t_c_breeder():
    print("[C] 同组孙进程 → killpg 完整收割")
    http("POST", "/api/components/breeder/start")
    wait_for("breeder RUNNING", lambda: comp("breeder")["state"] == "RUNNING", 15)
    http("POST", "/api/components/breeder/stop")
    wait_for("breeder STOPPED", lambda: comp("breeder")["state"] == "STOPPED", 25)
    time.sleep(1)
    check("孙进程(sleep 310/311)无残留", not pgrep("sleep 31"))


def t_d_daemon_escape():
    print("[D] 逃离进程组 → stop_failed 残留标记")
    http("POST", "/api/components/daemon/start")
    wait_for("daemon RUNNING", lambda: comp("daemon")["state"] == "RUNNING", 15)
    http("POST", "/api/components/daemon/stop")
    wait_for("daemon STOPPED", lambda: comp("daemon")["state"] == "STOPPED", 25)
    d = comp("daemon")
    check("stop_failed=True", d["stop_failed"] is True, str(d["stop_failed"]))
    check("错误信息含残留", "残留" in (d["last_error"] or ""),
          str(d["last_error"]))
    check("脱组进程仍在(pgrep 命中)", pgrep("sleep 600"))
    subprocess.run(["pkill", "-f", "sleep 600"])   # 人工清理,不留给后续场景
    time.sleep(1)


def t_e_grower():
    print("[E] 日志洪水 → 轮转 + 并发读取")
    http("POST", "/api/components/grower/start")
    wait_for("grower RUNNING", lambda: comp("grower")["state"] == "RUNNING", 15)
    errs = []

    def hammer():
        t_end = time.time() + 40
        while time.time() < t_end:
            c, d = http("GET", "/api/logs/grower?tail=50", timeout=10)
            if c != 200 or not isinstance(d.get("lines"), list):
                errs.append((c, str(d)[:80]))
            time.sleep(0.5)

    th = threading.Thread(target=hammer)
    th.start()
    th.join()
    logdir = os.path.join(HMI_DIR, "logs", "grower")
    wait_for("轮转副本出现", lambda: any(f.endswith(".1") for f in
             os.listdir(logdir)), timeout=45)
    cur = [f for f in os.listdir(logdir) if f.endswith(".log")]
    cur_size = os.path.getsize(os.path.join(logdir, cur[0])) if cur else 0
    check("40s 内并发日志读取零错误", not errs, str(errs[:2]))
    check("当前日志受控(<15MB,实际 %.1fMB)" % (cur_size / 1e6), cur_size < 15e6)
    http("POST", "/api/components/grower/stop")
    wait_for("grower STOPPED", lambda: comp("grower")["state"] == "STOPPED", 25)


def t_f_server_kill_foreign():
    print("[F] 服务器 SIGKILL → 重启 → foreign 标记与清理")
    http("POST", "/api/components/flapper/start")
    wait_for("flapper RUNNING", lambda: comp("flapper")["state"] == "RUNNING", 15)
    stop_server(sigkill=True)
    time.sleep(1)
    check("组件仍存活(HMI 暴毙不连带)", pgrep("sleep 900"))
    start_server()
    f = comp("flapper")
    check("重启后 foreign=True 标记", f["foreign"] is True, str(f["foreign"]))
    check("状态仍为 STOPPED(未收养)", f["state"] == "STOPPED")
    code, _ = http("POST", "/api/components/flapper/stop")
    check("外部组件单卡停止受理", code == 200, str(code))
    wait_for("单卡清理完成",
             lambda: comp("flapper")["foreign"] is False, 30)
    time.sleep(2)
    check("单卡停止清掉了外部进程(foreign 清理)",
          not pgrep("sleep 900"))
    check("foreign 标记已清除", comp("flapper")["foreign"] is False)


def t_g_flapping():
    print("[G] 快速启停翻转 ×15")
    fd0 = fd_count()
    for i in range(15):
        http("POST", "/api/components/flapper/start")
        time.sleep(0.8)
        http("POST", "/api/components/flapper/stop")
        time.sleep(1.6)
    time.sleep(2)
    st = comp("flapper")["state"]
    check("最终状态合法(%s)" % st, st in ("STOPPED", "CRASHED"))
    check("无僵尸子进程", defunct_children() == 0, "Z×%d" % defunct_children())
    fd_growth = fd_count() - fd0
    check("fd 增长受控(<15,实际 %+d)" % fd_growth, fd_growth < 15)
    check("无残留进程", not pgrep("sleep 900"))


def t_h_stop_race():
    print("[H] 停止进行中再操作 → 409")
    http("POST", "/api/components/stubborn/start")
    wait_for("stubborn RUNNING", lambda: comp("stubborn")["state"] == "RUNNING", 15)
    http("POST", "/api/components/stubborn/stop")
    time.sleep(0.3)
    c, _r = http("POST", "/api/components/stubborn/restart")
    check("STOPPING 期间 restart → 409", c == 409, str(c))
    wait_for("stubborn STOPPED", lambda: comp("stubborn")["state"] == "STOPPED", 30)


def t_i_soak():
    print("[I] 浸泡 60s:4 线程拉状态 + 周期重启")
    rss0, fd0 = rss_mb(), fd_count()
    stop_flag = {"v": False}
    errs = []
    n_req = [0]

    def poller():
        while not stop_flag["v"]:
            try:
                c, d = http("GET", "/api/state", timeout=10)
                n_req[0] += 1
                if c != 200 or "components" not in d:
                    errs.append(c)
            except Exception as exc:
                errs.append(str(exc))
            time.sleep(0.12)

    ths = [threading.Thread(target=poller) for _ in range(4)]
    for t in ths:
        t.start()
    t0 = time.time()
    restarts = 0
    while time.time() - t0 < 60:
        http("POST", "/api/components/flapper/start")
        time.sleep(2.5)
        http("POST", "/api/components/flapper/stop")
        time.sleep(2.0)
        restarts += 1
    stop_flag["v"] = True
    for t in ths:
        t.join()
    rss1, fd1 = rss_mb(), fd_count()
    check("%d 次状态请求零错误" % n_req[0], not errs, str(errs[:3]))
    check("RSS 增长 < 60MB(实际 %+.1fMB)" % (rss1 - rss0), rss1 - rss0 < 60)
    check("fd 增长 < 40(实际 %+d)" % (fd1 - fd0), fd1 - fd0 < 40)
    check("周期重启 %d 次完成" % restarts, comp("flapper")["state"] in
          ("STOPPED", "CRASHED", "RUNNING"))


def t_j_disk_watermark():
    print("[J] 磁盘水位(进程内 monkeypatch)")
    import importlib
    import shutil as _shutil
    import process_manager as pm_mod
    importlib.reload(pm_mod)   # 独立实例,不与运行中的服务共享类状态
    tmp = os.path.join("/tmp", "hmi_disk_test_%d" % os.getpid())
    os.makedirs(tmp, exist_ok=True)
    cfg = {"groups": {0: "g"}, "defaults": {"health": [], "start_timeout": 5,
          "health_lost_s": 3, "max_log_mb": 50, "keep_logs": 3, "ready_s": 1,
          "sample_period": 30},
          "components": [{"name": "x", "title": "x", "group": 0,
                          "cmd": ["sleep", "5"], "cwd": tmp,
                          "max_log_bytes": 100}]}
    pm = pm_mod.ProcessManager(cfg, tmp, pm_mod.HealthProvider())
    r = pm.runners["x"]
    assert r.do_start()
    r.logf.write(b"z" * 200)
    r.logf.flush()
    r.rotate_if_needed()
    d = os.path.join(tmp, "hmi", "logs", "x")
    # 手工造 .1/.2 副本
    for i in (1, 2):
        with open(os.path.join(d, "20260101-1.log.%d" % i), "w") as f:
            f.write("old" * i)

    real_usage = pm_mod.shutil.disk_usage
    pm_mod.shutil.disk_usage = lambda p: type("U", (), {"free": 400 * 1024 ** 2,
                                                        "used": 1, "total": 2})()
    pm._housekeeping()
    pm_mod.shutil.disk_usage = real_usage
    files = os.listdir(d)
    check("free<500MB 删除轮转副本", not any(f.endswith(".1") or f.endswith(".2")
          for f in files), str(files))
    check("紧急上限降为 0.25", pm._limit_scale == 0.25, str(pm._limit_scale))
    r.stop()
    import shutil
    shutil.rmtree(tmp, ignore_errors=True)


def main():
    global PROC
    print("=" * 62)
    print("HMI 混沌 / 浸泡校验(进程层)")
    print("=" * 62)
    steps = [t_a_gate, t_b_stubborn, t_c_breeder, t_d_daemon_escape,
             t_e_grower, t_f_server_kill_foreign, t_g_flapping, t_h_stop_race,
             t_i_soak, t_j_disk_watermark]
    try:
        start_server()
        for fn in steps:
            if fn is t_j_disk_watermark:
                stop_server()
            if fn in (t_j_disk_watermark,):
                fn()
                continue
            fn()
    except Exception as exc:
        import traceback
        traceback.print_exc()
        FAILS.append("场景异常: %r" % exc)
    finally:
        if PROC and PROC.poll() is None:
            PROC.terminate()
            try:
                out, _e = PROC.communicate(timeout=8)
            except Exception:
                PROC.kill()
                out, _e = PROC.communicate()
            check("服务端输出无 Traceback", b"Traceback" not in out)
        # 兜底清理
        for pat in ("trap '' TERM", "sleep 31", "sleep 600", "head -c 65536",
                    "sleep 900"):
            subprocess.run(["pkill", "-f", pat], stderr=subprocess.DEVNULL)
        import shutil
        shutil.rmtree(os.path.join(HMI_DIR, "logs"), ignore_errors=True)
        shutil.rmtree(os.path.join(HMI_DIR, "var"), ignore_errors=True)
    print("-" * 62)
    if FAILS:
        print("结果:%d 通过,%d 失败 → %s" % (PASS, len(FAILS), ", ".join(FAILS)))
        sys.exit(1)
    print("结果:%d 通过,0 失败" % PASS)


if __name__ == "__main__":
    main()
