# -*- coding: utf-8 -*-
"""端到端:200s 模拟时间线,t=100 急停 10s;验证事件目录/前后窗/complete/状态数据。"""
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import log_online_config as cfg  # noqa: E402
import mock_ros  # noqa: E402

ros = mock_ros.install()
from robot.msg import can_msg as can_mod, navigation_msg as nav_mod  # noqa: E402
import recorder  # noqa: E402


class TestLifecycle(unittest.TestCase):
    def test_event_window_and_metadata(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        data_dir = Path(tmp.name) / "events"
        rec = recorder.Recorder(ros, cfg, data_dir)
        rec.wire()

        t = 0.0
        while t < 210.0 - 1e-9:  # 210s:清除(~110s)+90s 后窗落在时间线内,事件能关闭
            es = 1 if 100.0 <= t < 110.0 else 0
            rec._on_can(can_mod.can_msg(vehicleSpeed=1.0, curGear=4,
                                        controlPanelState=1, emergencyStop=es))
            rec._on_navigation(nav_mod.navigation_msg(xAxis=1.0, yAxis=2.0))
            rec.tick(t)
            t += 0.1

        events = [d for d in data_dir.iterdir() if d.is_dir()]
        self.assertEqual(len(events), 1)
        ev = events[0]
        meta = json.loads((ev / "metadata.json").read_text("utf-8"))
        self.assertTrue(meta["complete"])
        self.assertEqual(len(meta["trigger_segments"]), 1)
        self.assertAlmostEqual(meta["trigger_segments"][0]["start"], 100.5, delta=0.2)
        records = [json.loads(line) for line in
                   (ev / "records.jsonl").read_text("utf-8").strip().split("\n")]
        first, last = records[0]["ts"], records[-1]["ts"]
        trigger_start = meta["trigger_segments"][0]["start"]
        trigger_clear = meta["trigger_segments"][0]["end"]
        # 需求:事发前至少 90s,清除后至少 90s
        self.assertLessEqual(first, trigger_start - 90.0 + 1e-6)
        self.assertGreaterEqual(last, trigger_clear + 90.0 - 1e-6)
        # 七类字段在记录里齐
        for key in ("control", "position", "motion", "perception", "response",
                    "lights", "video", "fault"):
            self.assertIn(key, records[0])
        # 关闭后状态机回 IDLE
        self.assertEqual(rec.get_status()["state"], "IDLE")

    def test_two_events_isolated(self):
        """同一进程先后两次独立触发(100-110 / 250-260):2 个目录,段信息各归各。"""
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        data_dir = Path(tmp.name) / "events"
        rec = recorder.Recorder(ros, cfg, data_dir)
        rec.wire()

        t = 0.0
        while t < 360.0 - 1e-9:  # 360s:第二事件清除(260s)+90s 后窗落在时间线内
            es = 1 if (100.0 <= t < 110.0 or 250.0 <= t < 260.0) else 0
            rec._on_can(can_mod.can_msg(vehicleSpeed=1.0, curGear=4,
                                        controlPanelState=1, emergencyStop=es))
            rec._on_navigation(nav_mod.navigation_msg(xAxis=1.0, yAxis=2.0))
            rec.tick(t)
            t += 0.1

        events = sorted(d for d in data_dir.iterdir() if d.is_dir())
        self.assertEqual(len(events), 2)
        starts = []
        for ev in events:
            meta = json.loads((ev / "metadata.json").read_text("utf-8"))
            self.assertTrue(meta["complete"])
            self.assertEqual(len(meta["trigger_segments"]), 1)  # 各自恰 1 段,无跨事件泄漏
            starts.append(meta["trigger_segments"][0]["start"])
        # 时间戳各归各:事件一段落在首触发展开处(≈100.5),事件二段落在 ≈250.5
        self.assertAlmostEqual(starts[0], 100.5, delta=0.2)
        self.assertAlmostEqual(starts[1], 250.5, delta=0.2)
        self.assertEqual(rec.get_status()["state"], "IDLE")

    def test_status_shape(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        rec = recorder.Recorder(ros, cfg, Path(tmp.name) / "events")
        rec.wire()
        rec._on_can(can_mod.can_msg(vehicleSpeed=0.5, curGear=4))
        rec.tick(500.0)
        st = rec.get_status()
        for key in ("ts", "vehicle_id", "snapshot", "health", "trigger", "ring", "disk"):
            self.assertIn(key, st)
        self.assertIn("can", st["health"])
        self.assertEqual(st["vehicle_id"], "A03")


if __name__ == "__main__":
    unittest.main()
