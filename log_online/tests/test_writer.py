# -*- coding: utf-8 -*-
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import log_online_config as cfg  # noqa: E402
import mock_ros  # noqa: E402

mock_ros.install()
import recorder  # noqa: E402


def rec(ts):
    return {"ts": ts, "vehicle_id": "A03"}


class TestEventWriter(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.data_dir = Path(self.tmp.name) / "events"

    def tearDown(self):
        self.tmp.cleanup()

    def test_open_append_close(self):
        w = recorder.EventWriter(self.data_dir, "A03", cfg)
        name = w.open("emergencystop", 100.0, [rec(10.0), rec(10.1)],
                      {"can_msg": 0.02})
        self.assertEqual(name, time_name(100.0) + "_emergencystop")
        w.append(rec(100.0))
        w.append(rec(100.1))
        w.maybe_flush(100.2)
        info = w.close([{"start": 100.0, "end": 100.1, "types": ["emergencystop"]}], 200.0)
        self.assertEqual(info["records"], 4)
        meta = json.loads((self.data_dir / name / "metadata.json").read_text("utf-8"))
        self.assertTrue(meta["complete"])
        self.assertEqual(meta["record_start"], 10.0)
        self.assertEqual(meta["record_end"], 200.0)
        self.assertEqual(meta["vehicle_id"], "A03")
        self.assertEqual(meta["health_at_trigger"], {"can_msg": 0.02})
        lines = (self.data_dir / name / "records.jsonl").read_text("utf-8").strip().split("\n")
        self.assertEqual(len(lines), 4)
        self.assertEqual(json.loads(lines[0])["ts"], 10.0)

    def test_name_collision_suffix(self):
        w1 = recorder.EventWriter(self.data_dir, "A03", cfg)
        n1 = w1.open("emergencystop", 100.0, [], {})
        w1.close([], 100.5)
        w2 = recorder.EventWriter(self.data_dir, "A03", cfg)
        n2 = w2.open("emergencystop", 100.0, [], {})
        w2.close([], 100.5)
        self.assertNotEqual(n1, n2)
        self.assertTrue(n2.endswith("_1"))

    def test_metadata_while_open_incomplete(self):
        w = recorder.EventWriter(self.data_dir, "A03", cfg)
        w.open("faultcode", 300.0, [rec(210.0)], {})
        meta = json.loads((self.data_dir / w._dir_name / "metadata.json").read_text("utf-8"))
        self.assertFalse(meta["complete"])
        self.assertIsNone(meta["record_end"])
        # 不 close 直接丢引用 = 模拟进程被 kill,metadata 保持 incomplete

    def test_sanitize_replaces_nonfinite(self):
        # NaN/inf(含嵌套 dict/list)落盘须换成 null:严格 JSON 解析器才认
        snap = {"ts": 100.0, "vehicle_id": "A03",
                "motion": {"speed": float("nan"), "accel": float("inf"), "gear": 4},
                "nested": [{"v": float("-inf")}, {"v": 1.5}]}
        w = recorder.EventWriter(self.data_dir, "A03", cfg)
        name = w.open("emergencystop", 100.0, [snap], {})
        w.close([], 100.5)
        line = (self.data_dir / name / "records.jsonl").read_text("utf-8").strip()

        def reject(token):
            raise ValueError("非标准 JSON 字面量: %s" % token)

        record = json.loads(line, parse_constant=reject)  # 严格模式解析成功
        self.assertIsNone(record["motion"]["speed"])
        self.assertIsNone(record["motion"]["accel"])
        self.assertIsNone(record["nested"][0]["v"])
        self.assertEqual(record["nested"][1]["v"], 1.5)


class TestRotator(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.data_dir = Path(self.tmp.name) / "events"
        self.data_dir.mkdir(parents=True)

    def tearDown(self):
        self.tmp.cleanup()

    def make_event(self, name, size_bytes):
        d = self.data_dir / name
        d.mkdir()
        (d / "records.jsonl").write_bytes(b"x" * size_bytes)

    def test_evict_oldest_until_under_target(self):
        self.make_event("a_1", 600)
        self.make_event("b_2", 600)
        self.make_event("c_3", 600)
        rot = recorder.Rotator(self.data_dir)
        self.assertEqual(rot.scan(), 1800)
        removed = rot.evict_if_over(1000, 0.9)  # 目标 900
        self.assertEqual(removed, ["a_1", "b_2"])
        self.assertFalse((self.data_dir / "a_1").exists())
        self.assertFalse((self.data_dir / "b_2").exists())
        self.assertTrue((self.data_dir / "c_3").exists())
        self.assertEqual(rot.scan(), 600)

    def test_under_cap_no_evict(self):
        self.make_event("a_1", 100)
        rot = recorder.Rotator(self.data_dir)
        self.assertEqual(rot.evict_if_over(1000, 0.9), [])

    def test_evict_failure_not_counted(self):
        # rmtree 无声失败(ignore_errors 目录仍在):须告警且不虚报水位
        # —— 不进 removed 列表、total 不扣减,目录保留待下轮重试
        self.make_event("a_1", 600)
        self.make_event("b_2", 600)
        rot = recorder.Rotator(self.data_dir)
        with mock.patch("shutil.rmtree", lambda path, ignore_errors=False: None), \
                mock.patch("sys.stderr"):
            self.assertEqual(rot.evict_if_over(1000, 0.9), [])
        self.assertEqual(rot.scan(), 1200)
        self.assertTrue((self.data_dir / "a_1").exists())
        self.assertTrue((self.data_dir / "b_2").exists())


def time_name(ts):
    import time as _t
    return _t.strftime("%Y%m%d_%H%M%S", _t.localtime(ts))


if __name__ == "__main__":
    unittest.main()
