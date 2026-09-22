# -*- coding: utf-8 -*-
"""log_online 状态页:只读展示,不做控制。内网使用,严禁公网映射。"""
import sys
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


def _sanitize(value):
    """递归把 float NaN/inf 换成 None:前端 JSON.parse 不认这三个字面量。
    与 recorder._sanitize 各持一份小函数——本模块保持仅标准库、不反向依赖
    recorder(recorder 顶部已 import 本模块,反向引用即循环依赖)。"""
    if isinstance(value, float) and (value != value or value in (float("inf"), float("-inf"))):
        return None
    if isinstance(value, dict):
        return {k: _sanitize(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_sanitize(v) for v in value]
    return value


class StatusServer(object):
    def __init__(self, status_fn, events_fn, static_dir, host, port):
        self._status_fn = status_fn
        self._events_fn = events_fn
        self._static_dir = Path(static_dir)
        self._host = host
        self._port = int(port)
        self._server = None
        self._thread = None

    @property
    def port(self):
        return self._server.server_address[1] if self._server else self._port

    def start(self):
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path == "/":
                    outer._serve_file(self, "index.html", "text/html; charset=utf-8")
                elif parsed.path == "/api/status":
                    outer._serve_json(self, outer._status_fn())
                elif parsed.path == "/api/events":
                    query = parse_qs(parsed.query)
                    limit = 20
                    if "limit" in query:
                        try:
                            limit = max(1, min(200, int(query["limit"][0])))
                        except ValueError:
                            pass
                    outer._serve_json(self, {"events": outer._events_fn(limit)})
                else:
                    self.send_error(404)

            def log_message(self, fmt, *args):
                pass  # 轮询噪声不刷日志

        try:
            self._server = ThreadingHTTPServer((self._host, self._port), Handler)
        except OSError as exc:
            print("[log_online] 状态页绑定失败(%s:%s): %s -- 记录不受影响"
                  % (self._host, self._port, exc), file=sys.stderr)
            return False
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        daemon=True, name="log_online_http")
        self._thread.start()
        return True

    def stop(self):
        if self._server is not None:
            self._server.shutdown()
            self._server = None

    def _serve_json(self, handler, payload):
        # 序列化前消毒 NaN/inf -> null,响应体保持严格合法 JSON
        body = json.dumps(_sanitize(payload), ensure_ascii=False).encode("utf-8")
        handler.send_response(200)
        handler.send_header("Content-Type", "application/json; charset=utf-8")
        handler.send_header("Content-Length", str(len(body)))
        handler.end_headers()
        handler.wfile.write(body)

    def _serve_file(self, handler, name, content_type):
        path = self._static_dir / name
        if not path.is_file():
            handler.send_error(404)
            return
        body = path.read_bytes()
        handler.send_response(200)
        handler.send_header("Content-Type", content_type)
        handler.send_header("Content-Length", str(len(body)))
        handler.end_headers()
        handler.wfile.write(body)
