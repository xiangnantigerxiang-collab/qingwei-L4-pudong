# -*- coding: utf-8 -*-
import json
import sys
import tempfile
import threading
import unittest
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from status_server import StatusServer  # noqa: E402


class TestStatusServer(unittest.TestCase):
    def test_endpoints(self):
        static_dir = Path(__file__).resolve().parents[1] / "static"
        fake_status = {"ts": 1.0, "vehicle_id": "A03", "snapshot": {}, "health": {},
                       "trigger": {"conditions": [], "state": "IDLE"},
                       "ring": {"count": 0}, "disk": {"used_bytes": 0}}
        fake_events = [{"dir": "20260916_000000_emergencystop", "complete": True}]

        srv = StatusServer(lambda: fake_status, lambda limit: fake_events[:limit],
                           static_dir, "127.0.0.1", 0)
        self.assertTrue(srv.start())
        self.addCleanup(srv.stop)
        port = srv.port

        with urllib.request.urlopen("http://127.0.0.1:%d/api/status" % port, timeout=3) as r:
            self.assertEqual(json.loads(r.read().decode("utf-8"))["vehicle_id"], "A03")
        with urllib.request.urlopen(
                "http://127.0.0.1:%d/api/events?limit=5" % port, timeout=3) as r:
            data = json.loads(r.read().decode("utf-8"))
            self.assertEqual(len(data["events"]), 1)
        with urllib.request.urlopen("http://127.0.0.1:%d/" % port, timeout=3) as r:
            body = r.read().decode("utf-8")
            self.assertIn("log_online", body)
            for anchor in ("id=\"live\"", "id=\"health\"", "id=\"events\""):
                self.assertIn(anchor, body)

    def test_port_conflict_returns_false(self):
        import socket
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        s.listen(1)
        port = s.getsockname()[1]
        srv = StatusServer(lambda: {}, lambda limit: [], Path("."), "127.0.0.1", port)
        self.assertFalse(srv.start())
        srv.stop()
        s.close()

    def test_status_json_sanitizes_nan(self):
        # status_fn 返回含 NaN/inf -> 响应体不得出现 NaN/Infinity 字面量
        # (python json.loads 默认容忍 NaN,须直接查字符串),值为 null
        fake_status = {"ts": float("nan"),
                       "snapshot": {"motion": {"speed": float("inf")}},
                       "nested": [float("-inf"), 1.0]}
        srv = StatusServer(lambda: fake_status, lambda limit: [],
                           Path("."), "127.0.0.1", 0)
        self.assertTrue(srv.start())
        self.addCleanup(srv.stop)
        with urllib.request.urlopen(
                "http://127.0.0.1:%d/api/status" % srv.port, timeout=3) as r:
            body = r.read().decode("utf-8")
        self.assertNotIn("NaN", body)
        self.assertNotIn("Infinity", body)
        data = json.loads(body)
        self.assertIsNone(data["ts"])
        self.assertIsNone(data["snapshot"]["motion"]["speed"])
        self.assertIsNone(data["nested"][0])
        self.assertEqual(data["nested"][1], 1.0)


if __name__ == "__main__":
    unittest.main()
