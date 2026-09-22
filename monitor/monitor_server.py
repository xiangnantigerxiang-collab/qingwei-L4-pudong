# -*- coding: utf-8 -*-
"""
monitor_server - 自动驾驶可视化 Web 服务(对标 hmi/hmi_server.py 骨架)
=======================================================================

纯 Python stdlib + rospy(可选)。无进程编排职责(那是 hmi 的事),
本服务只读话题、出数据:

  GET /                页面(static/index.html,no-store)
  GET /three.min.js    vendored Three.js r147(长缓存)
  GET /OrbitControls.js
  GET /api/snapshot    JSON 快照(前端 2Hz 轮询)
  GET /api/map         地图三线(一次性)
  GET /api/scan.bin    2D 补盲激光二进制帧
  GET /api/cloud.bin   3D 感知点云二进制帧(拉取即标记活动,门控订阅)

用法: python3 monitor_server.py [--config monitor_config.py]
                              [--host 0.0.0.0] [--port 8081]
"""

import argparse
import importlib.util
import json
import os
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

VERSION = "1.0.1"

MON_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, MON_DIR)

import ros_visualizer  # noqa: E402


# ---------------------------------------------------------------------------
# 配置加载(对标 hmi_server.load_config,校验从简:纯数据模块)
# ---------------------------------------------------------------------------

def load_config(path):
    spec = importlib.util.spec_from_file_location("monitor_config", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    cfg = getattr(mod, "CONFIG", None)
    if not isinstance(cfg, dict):
        raise ValueError("配置文件缺少 CONFIG 字典")
    return cfg


# ---------------------------------------------------------------------------
# 应用聚合(对标 hmi 的 HmiApp,但只有可视化数据)
# ---------------------------------------------------------------------------

class MonitorApp(object):

    def __init__(self, cfg):
        self.cfg = cfg
        self.viz = ros_visualizer.RosVisualizer(cfg)
        self.viz.start()

    def snapshot(self):
        return self.viz.snapshot()

    def map_payload(self):
        return self.viz.map_payload()

    def fence_payload(self):
        return self.viz.fence_payload()

    def scan_bin(self):
        return self.viz.scan_bin()

    def cloud_bin(self):
        self.viz.mark_cloud_pull()      # 门控:拉取即活动
        blob = self.viz.cloud_bin()
        if not blob:                    # 尚无数据也回 20B 空头,前端统一处理
            blob = ros_visualizer.pack_bin(
                [], int(time.time() * 1000) & 0xFFFFFFFFFFFFFFFF)
        return blob

    def shutdown(self):
        self.viz.stop()


# ---------------------------------------------------------------------------
# HTTP 层(对标 hmi_server.make_handler)
# ---------------------------------------------------------------------------

_CACHE_JS = {"three.min.js": "three", "OrbitControls.js": "orbit"}


def make_handler(app):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "qingweiMON/" + VERSION
        timeout = 30    # keep-alive 空闲连接兜底,防 FD/线程滞留

        def log_message(self, fmt, *args):
            pass    # 静默访问日志

        # ---- 响应原语 ----

        def _json(self, code, obj):
            try:
                # allow_nan=False:nan/inf 若漏进任何字段,宁可知情报错
                # 也不向浏览器输出非法 JSON(其 r.json() 会解析失败)
                body = json.dumps(obj, ensure_ascii=False,
                                  allow_nan=False).encode("utf-8")
            except ValueError:
                body = json.dumps({"ok": False,
                                   "error": "non-finite number in payload"},
                                  ensure_ascii=False).encode("utf-8")
                code = 200     # 仍回 200:前端按字段缺失降级而非断连
            self.send_response(code)
            self.send_header("Content-Type",
                             "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _bytes(self, body, ctype, cache):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", cache)
            self.end_headers()
            self.wfile.write(body)

        # ---- 路由 ----

        def do_GET(self):
            try:
                self._do_get()
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _do_get(self):
            u = urlparse(self.path)
            p = u.path

            if p == "/" or p == "/index.html":
                page = os.path.join(MON_DIR, "static", "index.html")
                try:
                    with open(page, "rb") as f:
                        body = f.read()
                except OSError:
                    self._json(404, {"ok": False,
                                     "error": "缺少 static/index.html"})
                    return
                self._bytes(body, "text/html; charset=utf-8", "no-store")
                return

            if p.lstrip("/") in _CACHE_JS:
                name = os.path.join(MON_DIR, "static", p.lstrip("/"))
                try:
                    with open(name, "rb") as f:
                        body = f.read()
                except OSError:
                    self._json(404, {"ok": False, "error": "缺 " + p})
                    return
                # 大文件长缓存(带版本号路径);html 仍 no-store
                self._bytes(body,
                            "application/javascript; charset=utf-8",
                            "public, max-age=86400")
                return

            if p == "/api/snapshot":
                self._json(200, app.snapshot())
                return

            if p == "/api/map":
                payload = dict(app.map_payload())   # 浅拷贝,勿污染缓存
                payload["layers"] = app.cfg.get("LAYERS", {})
                # 电子围栏静态数据随地图一次性下发(同为懒加载缓存)
                payload["fences"] = app.fence_payload().get("fences", [])
                self._json(200, payload)
                return

            if p == "/api/scan.bin":
                self._bytes(app.scan_bin(),
                            "application/octet-stream", "no-store")
                return

            if p == "/api/cloud.bin":
                self._bytes(app.cloud_bin(),
                            "application/octet-stream", "no-store")
                return

            if p == "/favicon.ico":
                self.send_response(204)
                self.end_headers()
                return

            self._json(404, {"ok": False, "error": "unknown path " + p})

    return Handler


# ---------------------------------------------------------------------------
# 入口(对标 hmi_server.main)
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(MON_DIR,
                                                     "monitor_config.py"))
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int,
                    default=int(os.environ.get("MONITOR_PORT", "8081")))
    args = ap.parse_args()

    cfg_path = args.config
    if not os.path.isabs(cfg_path):
        cfg_path = os.path.join(MON_DIR, cfg_path)
    try:
        cfg = load_config(cfg_path)
    except Exception as exc:
        print("[MONITOR] 配置加载失败: %s: %s" % (cfg_path, exc),
              file=sys.stderr)
        sys.exit(1)
    # 端口优先级:命令行 > 环境变量 MONITOR_PORT > 配置 PORT > 默认
    if args.port == 8081 and os.environ.get("MONITOR_PORT"):
        args.port = int(os.environ["MONITOR_PORT"])
    elif args.port == 8081 and cfg.get("PORT"):
        args.port = int(cfg["PORT"])

    app = MonitorApp(cfg)

    # dashboard 仪表板:独立只读端口(默认 8082),与主服务同进程同一
    # rospy 节点,复用订阅;任何启动失败只告警不影响主服务。
    # 端口语义:环境变量未设/空串 -> 配置值(空串不遮蔽配置);
    # 两者皆未设 -> 8082;值 0 -> 关闭;非法/越界 -> stderr 告警并关闭
    raw_port = os.environ.get("DASHBOARD_PORT")
    if raw_port is None or raw_port == "":
        raw_port = cfg.get("DASHBOARD_PORT", 8082)
    try:
        dash_port = int(raw_port or 0)
        if not 0 <= dash_port <= 65535:
            raise ValueError("端口越界 %r" % (raw_port,))
    except (TypeError, ValueError) as exc:
        print("[MONITOR] DASHBOARD_PORT 非法(%s),dashboard 关闭(主服务继续)"
              % exc, file=sys.stderr)
        dash_port = 0
    dash_httpd = None
    if dash_port:
        try:
            import dashboard
            dash_app = dashboard.DashboardApp(app.viz)
            dash_httpd = ThreadingHTTPServer(
                (args.host, dash_port),
                dashboard.make_dashboard_handler(dash_app))
            dash_httpd.daemon_threads = True
            threading.Thread(target=dash_httpd.serve_forever,
                             name="mon-dash", daemon=True).start()
            print("[MONITOR] dashboard 已启动: http://%s:%d" % (
                args.host if args.host != "0.0.0.0" else "本机IP", dash_port))
        except Exception as exc:   # 含端口占用/Overflow/.msg 解析异常:永不拖死主服务
            print("[MONITOR] dashboard 启动失败(主服务继续): %s" % exc,
                  file=sys.stderr)
            dash_httpd = None

    try:
        httpd = ThreadingHTTPServer((args.host, args.port),
                                    make_handler(app))
    except OSError as exc:
        print("[MONITOR] 端口绑定失败 %s:%s: %s" % (args.host, args.port, exc),
              file=sys.stderr)
        app.shutdown()
        sys.exit(1)
    httpd.daemon_threads = True
    print("[MONITOR] 已启动: http://%s:%d  (ROS: %s)" % (
        args.host if args.host != "0.0.0.0" else "本机IP", args.port,
        "可用" if ros_visualizer.ROS_AVAILABLE else "不可用(仅静态/空数据)"))

    def _shutdown(signum, frame):
        import threading
        # 不能在服务线程内同步调 shutdown,另起线程
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        httpd.serve_forever()
    finally:
        httpd.server_close()
        if dash_httpd is not None:
            dash_httpd.shutdown()
            dash_httpd.server_close()
        app.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
