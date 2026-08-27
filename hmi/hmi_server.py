# -*- coding: utf-8 -*-
"""
HMI 服务入口:配置加载、系统资源监控、HTTP API、启动编排。

用法:
  车载:  bash hmi/hmi.sh                (默认 0.0.0.0:8080,加载 hmi_config.py)
  本机:  python3 hmi_server.py --config test_config.py --port 18080

API:
  GET  /                     前端页面(no-store)
  GET  /api/state            全量快照(server/sequence/components/vehicle/system)
  GET  /api/logs/<name>      日志尾部 ?tail=N(默认200,上限2000);?download=1 全文下载
  POST /api/start            一键启动(分组并行+健康门控,后台执行)
  POST /api/stop             全部停止(逆序);body 必须含 {"confirm":"STOP"}
  POST /api/components/<name>/{start,stop,restart}
"""

import argparse
import importlib.util
import json
import os
import shutil
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs, unquote

import process_manager
from process_manager import ProcessManager

# rospy 可用性探测(独立于 RosBridge 实例化;本机无 ROS 时为 False)
try:
    import rospy  # noqa: F401
    ROS_OK = True
except ImportError:
    ROS_OK = False

VERSION = "1.0.0"
HMI_DIR = Path(__file__).resolve().parent
ROOT = HMI_DIR.parent
LOG_TAIL_MAX = 2000
LOG_READ_BYTES = 256 * 1024

# ---------------------------------------------------------------- 配置加载

def load_config(path):
    """加载配置模块并做校验与占位符替换($ROOT/$VARDIR)。"""
    spec = importlib.util.spec_from_file_location("hmi_cfg", str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    cfg = mod.CONFIG

    vardir = HMI_DIR / "var"
    subs = {"$ROOT": str(ROOT), "$VARDIR": str(vardir)}

    def repl(s):
        for k, v in subs.items():
            s = s.replace(k, v)
        return s

    def repl_item(x):
        if isinstance(x, str):
            return repl(x)
        if isinstance(x, list):
            return [repl_item(i) for i in x]
        if isinstance(x, dict):
            return {k: repl_item(v) for k, v in x.items()}
        return x

    names = set()
    for c in cfg["components"]:
        for key in ("name", "title", "group", "cmd", "cwd"):
            if key not in c:
                raise ValueError("组件缺少字段 %s: %r" % (key, c.get("name")))
        if c["name"] in names:
            raise ValueError("组件名重复: %s" % c["name"])
        names.add(c["name"])
        if c["group"] not in cfg["groups"]:
            raise ValueError("组件 %s 的 group %r 不在 groups 定义中" % (c["name"], c["group"]))
    cfg["components"] = [repl_item(c) for c in cfg["components"]]
    return cfg, names


# ---------------------------------------------------------------- 系统资源

class SystemMonitor(object):

    def __init__(self):
        self._lk = threading.Lock()
        self._last_stat = self._read_stat()
        self._last_t = time.monotonic()

    @staticmethod
    def _read_stat():
        with open("/proc/stat", "r") as f:
            parts = f.readline().split()[1:]
        return [int(x) for x in parts]

    def snapshot(self):
        with self._lk:
            st, t = self._read_stat(), time.monotonic()
            d_total = sum(st) - sum(self._last_stat)
            d_idle = st[3] - self._last_stat[3] + (st[4] - self._last_stat[4] if len(st) > 4 else 0)
            cpu = None if d_total <= 0 else round(100.0 * (1 - d_idle / d_total), 1)
            self._last_stat, self._last_t = st, t
        out = {"cpu_pct": cpu, "mem_pct": None, "mem_total_gb": None,
               "cpu_temp_c": None, "disk_pct": None, "disk_free_gb": None,
               "disk_warn": False, "load1": None, "sys_uptime_s": None}
        try:
            with open("/proc/meminfo", "r") as f:
                mi = {}
                for line in f:
                    k, v = line.split(":", 1)
                    mi[k] = int(v.strip().split()[0])
            total, avail = mi.get("MemTotal", 0), mi.get("MemAvailable", 0)
            if total:
                out["mem_pct"] = round(100.0 * (1 - avail / total), 1)
                out["mem_total_gb"] = round(total / 1048576.0, 1)
        except (OSError, ValueError):
            pass
        try:
            temps = []
            for z in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
                try:
                    temps.append(int(z.read_text().strip()) / 1000.0)
                except (OSError, ValueError):
                    continue
            if temps:
                out["cpu_temp_c"] = round(max(temps), 1)
        except OSError:
            pass
        try:
            du = shutil.disk_usage(str(HMI_DIR))
            out["disk_pct"] = round(100.0 * du.used / du.total, 1)
            out["disk_free_gb"] = round(du.free / (1000 ** 3), 1)
            out["disk_warn"] = du.free < 1024 ** 3
        except OSError:
            pass
        try:
            with open("/proc/loadavg", "r") as f:
                out["load1"] = float(f.read().split()[0])
        except (OSError, ValueError):
            pass
        try:
            with open("/proc/uptime", "r") as f:
                out["sys_uptime_s"] = int(float(f.read().split()[0]))
        except (OSError, ValueError):
            pass
        return out


# ---------------------------------------------------------------- HTTP 服务

class HmiApp(object):
    """聚合各模块,供 handler 访问。"""

    def __init__(self, cfg, names):
        self.names = names
        self.started_at = time.time()
        self.sm = SystemMonitor()

        # ROS 可用则用 RosBridge 做健康判定,否则退化为仅进程存活
        self.bridge = None
        provider = process_manager.HealthProvider()
        if ROS_OK:
            import ros_bridge
            health_specs = []
            for c in cfg["components"]:
                health_specs.extend(c.get("health") or [])
            self.bridge = ros_bridge.RosBridge(
                health_specs, sample_period=cfg["defaults"].get("sample_period", 30))
            provider = self.bridge
        self.provider = provider
        self.pm = ProcessManager(cfg, ROOT, provider)
        self.pm.detect_foreign()
        threading.Thread(target=self.pm.monitor_loop, daemon=True).start()
        if self.bridge is not None:
            self.bridge.start()

    def state(self):
        if self.bridge is not None:
            vehicle = self.bridge.vehicle_state_final()
            master_ok = self.bridge.master_ok
            errs = self.bridge.msg_import_errors()
        else:
            vehicle = {"ros_available": False,
                       "reason": "无 ROS 环境(rospy 导入失败),仅进程管理可用"}
            master_ok = None
            errs = {}
        st = self.pm.state()
        return {
            "server": {"version": VERSION, "started_at": self.started_at,
                       "ros_available": ROS_OK, "master_ok": master_ok,
                       "msg_import_errors": errs},
            "sequence": st["sequence"],
            "components": st["components"],
            "vehicle": vehicle,
            "system": self.sm.snapshot(),
        }

    # ---- 组件操作(供 handler 调;stop/restart 走后台线程) ----

    def comp_start(self, name):
        if self.pm.seq_active():
            return 409, {"ok": False, "error": "一键编排进行中,请稍候"}
        ok, st = self.pm.start_component(name)
        if not ok:
            return 409, {"ok": False, "error": "当前状态 %s 不允许启动" % st}
        return 200, {"ok": True, "state": st}

    def comp_stop(self, name):
        if self.pm.seq_active():
            return 409, {"ok": False, "error": "一键编排进行中,请稍候"}
        r = self.pm.runners.get(name)
        if r is None:
            return 404, {"ok": False, "error": "未知组件"}
        with r._lk:
            if r.state == "STOPPED":
                return 200, {"ok": True, "note": "already stopped"}
            if r.state == "STOPPING":
                return 409, {"ok": False, "error": "正在停止中"}
        threading.Thread(target=self.pm.stop_component, args=(name,),
                         daemon=True).start()
        return 200, {"ok": True, "state": "STOPPING"}

    def comp_restart(self, name):
        if self.pm.seq_active():
            return 409, {"ok": False, "error": "一键编排进行中,请稍候"}
        r = self.pm.runners.get(name)
        if r is None:
            return 404, {"ok": False, "error": "未知组件"}
        with r._lk:
            if r.state in ("STARTING", "STOPPING"):
                return 409, {"ok": False, "error": "当前状态 %s 不允许重启" % r.state}
        threading.Thread(target=self.pm.restart_component, args=(name,),
                         daemon=True).start()
        return 200, {"ok": True, "state": "RESTARTING"}

    # ---- 日志 ----

    def logs(self, name, tail, download):
        if name not in self.names:
            return 404, {"ok": False, "error": "未知组件"}
        r = self.pm.runners[name]
        with r._lk:
            path = r.log_path
        if path is None or not path.exists():
            return 200, {"name": name, "lines": [], "size": 0, "note": "暂无日志"}
        size = path.stat().st_size
        if download:
            return 200, ("file", path, size)
        with open(str(path), "rb") as f:
            if size > LOG_READ_BYTES:
                f.seek(size - LOG_READ_BYTES)
                f.readline()   # 丢弃半行
            data = f.read()
        text = data.decode("utf-8", "replace")
        lines = text.splitlines()[-tail:]
        return 200, {"name": name, "lines": lines, "size": size}


def make_handler(app):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "qingweiHMI/" + VERSION

        # ---- 基础 ----

        def log_message(self, fmt, *args):
            pass   # 静默访问日志,避免刷屏

        def _send_json(self, code, obj):
            body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_page(self):
            page = HMI_DIR / "static" / "index.html"
            try:
                body = page.read_bytes()
            except FileNotFoundError:
                self._send_json(404, {"ok": False, "error": "缺少 static/index.html"})
                return
            try:
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _body_json(self):
            n = int(self.headers.get("Content-Length") or 0)
            if n <= 0:
                return {}
            try:
                return json.loads(self.rfile.read(n).decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                return {}

        def _guard_wfile(fn):
            def wrapped(self, *a, **kw):
                try:
                    return fn(self, *a, **kw)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            return wrapped

        # ---- GET ----

        @_guard_wfile
        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/" or u.path == "/index.html":
                self._send_page()
                return
            if u.path == "/api/state":
                self._send_json(200, app.state())
                return
            if u.path.startswith("/api/logs/"):
                name = unquote(u.path[len("/api/logs/"):])
                qs = parse_qs(u.query)
                try:
                    tail = min(int(qs.get("tail", ["200"])[0]), LOG_TAIL_MAX)
                except ValueError:
                    tail = 200
                tail = max(1, tail)
                download = qs.get("download", ["0"])[0] == "1"
                code, res = app.logs(name, tail, download)
                if code == 200 and isinstance(res, tuple):
                    _, path, size = res
                    self.send_response(200)
                    self.send_header("Content-Type",
                                     "text/plain; charset=utf-8")
                    self.send_header("Content-Disposition",
                                     'attachment; filename="%s.log"' % name)
                    self.send_header("Content-Length", str(size))
                    self.end_headers()
                    with open(str(path), "rb") as f:
                        while True:
                            chunk = f.read(65536)
                            if not chunk:
                                break
                            self.wfile.write(chunk)
                    return
                self._send_json(code, res)
                return
            if u.path == "/favicon.ico":
                self.send_response(204)
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            self._send_json(404, {"ok": False, "error": "not found: %s" % u.path})

        # ---- POST ----

        @_guard_wfile
        def do_POST(self):
            u = urlparse(self.path)
            parts = [p for p in u.path.split("/") if p]
            # /api/start | /api/stop | /api/components/<name>/<op>
            if u.path == "/api/start":
                if not app.pm.start_all_async():
                    return self._send_json(409, {"ok": False, "error": "编排已在进行中"})
                return self._send_json(200, {"ok": True, "state": "STARTING"})
            if u.path == "/api/stop":
                body = self._body_json()
                if body.get("confirm") != "STOP":
                    return self._send_json(
                        400, {"ok": False, "error": '需要 body {"confirm":"STOP"} 确认'})
                if not app.pm.stop_all_async():
                    return self._send_json(409, {"ok": False, "error": "编排已在进行中"})
                return self._send_json(200, {"ok": True, "state": "STOPPING"})
            if len(parts) == 4 and parts[:2] == ["api", "components"]:
                name, op = unquote(parts[2]), parts[3]
                if name not in app.names:
                    return self._send_json(404, {"ok": False, "error": "未知组件"})
                if op == "start":
                    code, res = app.comp_start(name)
                elif op == "stop":
                    code, res = app.comp_stop(name)
                elif op == "restart":
                    code, res = app.comp_restart(name)
                else:
                    code, res = 404, {"ok": False, "error": "未知操作 %s" % op}
                return self._send_json(code, res)
            self._send_json(404, {"ok": False, "error": "not found: %s" % u.path})

    return Handler


# ---------------------------------------------------------------- 入口

def main():
    ap = argparse.ArgumentParser(description="qingwei L4 Web HMI")
    ap.add_argument("--config", default=str(HMI_DIR / "hmi_config.py"))
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=int(os.environ.get("HMI_PORT", "8080")))
    args = ap.parse_args()

    cfg_path = Path(args.config)
    if not cfg_path.is_absolute():
        cfg_path = HMI_DIR / cfg_path
    try:
        cfg, names = load_config(cfg_path)
    except Exception as exc:
        print("[HMI] 配置加载失败(%s): %s" % (cfg_path, exc), file=sys.stderr)
        sys.exit(1)

    app = HmiApp(cfg, names)
    try:
        httpd = ThreadingHTTPServer((args.host, args.port), make_handler(app))
    except OSError as exc:
        print("[HMI] 端口绑定失败 %s:%d(%s)" % (args.host, args.port, exc),
              file=sys.stderr)
        sys.exit(1)
    httpd.daemon_threads = True

    def _shutdown(signum, _frame):
        print("[HMI] 收到信号 %d,退出(不停止组件;组件保持运行)" % signum)
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    print("[HMI] 已启动: http://%s:%d  (ROS=%s, 组件数=%d, 配置=%s)" % (
        args.host, args.port, "可用" if ROS_OK else "不可用", len(names), cfg_path.name))
    try:
        httpd.serve_forever()
    finally:
        httpd.server_close()
        print("[HMI] 已退出")


if __name__ == "__main__":
    main()
