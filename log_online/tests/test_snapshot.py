# -*- coding: utf-8 -*-
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import log_online_config as cfg  # noqa: E402
import mock_ros  # noqa: E402

mock_ros.install()
from robot.msg import can_msg as can_mod, navigation_msg as nav_mod, object as obj_mod, \
    perception as perc_mod, path_plan_msg as plan_mod, control_msg as ctrl_mod  # noqa: E402
import recorder  # noqa: E402

NOW = 1000.0


def fresh_latest(**over):
    latest = {
        "can": can_mod.can_msg(vehicleSpeed=1.2, curGear=4, controlPanelState=1,
                               emergencyStop=0, faultCode=b"\x01\x02"),
        "navigation": nav_mod.navigation_msg(lat=31.1, lon=121.8, xAxis=3.0, yAxis=4.0,
                                             heading=90.0, rtkState="fixed",
                                             longitudinal_accelerate=0.0),
        "perception": perc_mod.perception(objs=[
            obj_mod.object(id=1, type=0, x=3.0, y=4.0, dx=4.0, dy=2.0, confidence=0.9)]),
        "plan": plan_mod.path_plan_msg(desireSpeed=1.5, safety=False),
        "control": ctrl_mod.control_msg(throttlePercent=9, brakePercent=0, wheelAngle=12.5),
        "camera": object(),
        "sensorstate": 0, "netcheck": 0, "ultra_safe": 0, "light": 3, "horn": 0,
        "camera_fps": 15.0,
    }
    latest.update(over)
    return latest


def fresh_seen(**over):
    seen = {"can": NOW - 0.05, "navigation": NOW - 0.02, "perception": NOW - 0.01,
            "plan": NOW - 0.1, "control": NOW - 0.05, "camera": NOW - 0.1}
    seen.update(over)
    return seen


class TestFaultCodeHex(unittest.TestCase):
    def test_bytes_and_list(self):
        self.assertEqual(recorder.fault_code_hex(b"\x01\xaa"), "01aa")
        self.assertEqual(recorder.fault_code_hex([1, 170]), "01aa")
        self.assertEqual(recorder.fault_code_hex(b""), "")
        self.assertEqual(recorder.fault_code_hex(None), "")


class TestAccelEstimator(unittest.TestCase):
    def test_constant_accel_converges(self):
        est = recorder.AccelEstimator(0.5)
        t, v, out = 0.0, 0.0, 0.0
        for i in range(200):
            t = i * 0.1
            v = 0.5 * t
            out = est.update(v, t)
        self.assertGreater(out, 0.45)
        self.assertLess(out, 0.55)

    def test_none_speed_keeps_value(self):
        est = recorder.AccelEstimator(0.5)
        est.update(0.0, 0.0)
        est.update(1.0, 1.0)
        val = est.update(None, 2.0)
        val2 = est.update(None, 3.0)
        self.assertEqual(val, val2)


class TestBuildSnapshot(unittest.TestCase):
    def test_schema_complete(self):
        snap = recorder.build_snapshot(fresh_latest(), fresh_seen(), 0.1, NOW, cfg)
        for key in ("ts", "vehicle_id", "control", "position", "motion", "perception",
                    "response", "lights", "video", "fault"):
            self.assertIn(key, snap)
        self.assertEqual(snap["vehicle_id"], "A03")
        self.assertEqual(snap["control"]["controlPanelState"], 1)
        self.assertEqual(snap["position"]["lat"], 31.1)
        self.assertEqual(snap["motion"]["gear"], 4)
        self.assertEqual(snap["motion"]["speed"], 1.2)
        self.assertEqual(snap["motion"]["accel"], 0.1)
        self.assertEqual(snap["motion"]["accel_raw"], 0.0)
        self.assertEqual(snap["perception"]["obj_count"], 1)
        self.assertEqual(snap["perception"]["nearest_dist"], 0.0)  # 障碍(3,4)与自车(3,4)重合
        self.assertEqual(snap["response"]["throttle"], 9)
        self.assertEqual(snap["lights"]["light"], 3)
        self.assertTrue(snap["video"]["online"])
        self.assertEqual(snap["fault"]["faultCode_hex"], "0102")

    def test_missing_msg_gives_null_and_stale(self):
        snap = recorder.build_snapshot(fresh_latest(can=None), fresh_seen(), 0.0, NOW, cfg)
        self.assertIsNone(snap["motion"]["speed"])
        self.assertTrue(snap["motion"]["stale"])
        self.assertTrue(snap["control"]["stale"])

    def test_stale_by_age(self):
        snap = recorder.build_snapshot(fresh_latest(), fresh_seen(can=NOW - 3.0), 0.0, NOW, cfg)
        self.assertTrue(snap["motion"]["stale"])

    def test_object_truncation(self):
        objs = [obj_mod.object(id=i, x=10.0 + i, y=0.0) for i in range(100)]
        snap = recorder.build_snapshot(
            fresh_latest(perception=perc_mod.perception(objs=objs)), fresh_seen(), 0.0, NOW, cfg)
        self.assertEqual(snap["perception"]["obj_count"], 100)
        self.assertEqual(len(snap["perception"]["objs"]), cfg.MAX_OBJECTS)

    def test_nearest_dist_uses_ego_position(self):
        objs = [obj_mod.object(id=1, x=6.0, y=8.0)]
        snap = recorder.build_snapshot(
            fresh_latest(perception=perc_mod.perception(objs=objs)), fresh_seen(), 0.0, NOW, cfg)
        self.assertEqual(snap["perception"]["nearest_dist"], 5.0)  # hypot(6-3, 8-4)

    def test_camera_disabled_when_topic_empty(self):
        class NoCamCfg(object):
            STALE_AFTER_S = cfg.STALE_AFTER_S
            MAX_OBJECTS = cfg.MAX_OBJECTS
            TOPIC_CAMERA = ""
        snap = recorder.build_snapshot(fresh_latest(), fresh_seen(), 0.0, NOW, NoCamCfg)
        self.assertIsNone(snap["video"]["online"])

    def test_camera_stale_offline(self):
        snap = recorder.build_snapshot(fresh_latest(), fresh_seen(camera=NOW - 3.0), 0.0, NOW, cfg)
        self.assertFalse(snap["video"]["online"])


if __name__ == "__main__":
    unittest.main()
