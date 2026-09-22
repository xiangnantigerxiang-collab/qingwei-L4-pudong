# -*- coding: utf-8 -*-
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mock_ros  # noqa: E402

mock_ros.install()
import recorder  # noqa: E402


def snap(**over):
    base = {
        "ts": 0.0,
        "response": {"sensorstate": 0, "safety": 0},
        "fault": {"emergencyStop": 0, "netcheck": 0, "faultCode_hex": ""},
    }
    base.update(over)
    return base


class TestRingBuffer(unittest.TestCase):
    def test_trim_keeps_window(self):
        ring = recorder.RingBuffer(120.0)
        for i in range(1500):
            ring.append({"ts": float(i) * 0.1})
            ring.trim(float(i) * 0.1)
        self.assertEqual(ring.span(149.9), 149.9 - ring.tail(0.0)[0]["ts"])
        self.assertLessEqual(ring.span(149.9), 120.0 + 0.1)
        self.assertGreaterEqual(ring.span(149.9), 119.0)
        tail = ring.tail(60.0)
        self.assertAlmostEqual(tail[0]["ts"], 60.0, places=5)   # 0.1 步进有浮点尾差
        self.assertAlmostEqual(tail[-1]["ts"], 149.9, places=5)


class TestEvaluateTrigger(unittest.TestCase):
    def test_none_triggers_nothing(self):
        self.assertEqual(recorder.evaluate_trigger(
            snap(fault={"emergencyStop": None, "netcheck": 0, "faultCode_hex": None},
                 response={"sensorstate": None})), [])

    def test_each_condition(self):
        self.assertEqual(recorder.evaluate_trigger(
            snap(fault={"emergencyStop": 1, "netcheck": 0, "faultCode_hex": ""})),
            ["emergencystop"])
        self.assertEqual(recorder.evaluate_trigger(
            snap(response={"sensorstate": 3})), ["sensorstate"])
        self.assertEqual(recorder.evaluate_trigger(
            snap(fault={"emergencyStop": 0, "netcheck": 1, "faultCode_hex": ""})),
            ["netcheck"])
        self.assertEqual(recorder.evaluate_trigger(
            snap(fault={"emergencyStop": 0, "netcheck": 0, "faultCode_hex": "01"})),
            ["faultcode"])

    def test_multiple_order(self):
        self.assertEqual(recorder.evaluate_trigger(
            snap(fault={"emergencyStop": 1, "netcheck": 1, "faultCode_hex": "aa"},
                 response={"sensorstate": 2})),
            ["emergencystop", "sensorstate", "netcheck", "faultcode"])


class TestDebouncer(unittest.TestCase):
    def test_fires_exactly_once_at_nth(self):
        deb = recorder.Debouncer(5)
        results = [deb.feed(True) for _ in range(7)]
        self.assertEqual(results, [False, False, False, False, True, False, False])

    def test_reset_on_inactive(self):
        deb = recorder.Debouncer(5)
        for _ in range(4):
            deb.feed(True)
        self.assertFalse(deb.feed(False))
        self.assertFalse(deb.feed(True))
        self.assertFalse(deb.feed(True))
        self.assertFalse(deb.feed(True))
        self.assertFalse(deb.feed(True))
        self.assertTrue(deb.feed(True))


if __name__ == "__main__":
    unittest.main()
