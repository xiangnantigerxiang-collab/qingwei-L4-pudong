# -*- coding: utf-8 -*-
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import log_online_config as cfg  # noqa: E402
import mock_ros  # noqa: E402


class TestConfig(unittest.TestCase):
    def test_defaults(self):
        self.assertEqual(cfg.VEHICLE_ID, "A03")
        self.assertEqual(cfg.TOPIC_CAN, "/can_msg")
        self.assertEqual(cfg.SAMPLE_HZ, 10)
        self.assertEqual(cfg.RING_SECONDS, 120.0)
        self.assertEqual(cfg.PRE_WINDOW_S, 90.0)
        self.assertEqual(cfg.POST_WINDOW_S, 90.0)
        self.assertEqual(cfg.DEBOUNCE_TICKS, 5)
        self.assertEqual(cfg.DISK_CAP_BYTES, 1073741824)
        self.assertEqual(cfg.ROTATE_KEEP_RATIO, 0.9)
        self.assertEqual(cfg.MAX_OBJECTS, 64)
        self.assertEqual(cfg.STALE_AFTER_S, 2.0)
        self.assertEqual(cfg.HTTP_PORT, 8083)
        self.assertEqual(cfg.TOPIC_CAMERA, "/cam0/compressed")


class TestMockRos(unittest.TestCase):
    def test_install_and_subscriber(self):
        mock_ros.install()
        import rospy  # noqa: F401
        from robot.msg import can_msg as can_msg_mod  # noqa: F401

        sub = rospy.Subscriber("/can_msg", can_msg_mod.can_msg, lambda m: None, queue_size=1)
        self.assertEqual(sub.topic, "/can_msg")
        msg = can_msg_mod.can_msg(vehicleSpeed=1.5, curGear=4, emergencyStop=1)
        self.assertEqual(msg.vehicleSpeed, 1.5)
        self.assertEqual(msg.emergencyStop, 1)

    def test_get_param_default(self):
        mock_ros.install()
        import rospy
        self.assertEqual(rospy.get_param("/planning/sensorstate", 0), 0)


if __name__ == "__main__":
    unittest.main()
