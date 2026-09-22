"""围栏心跳与三态显示；不启动 ROS、HTTP 服务或进程管理器。"""
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros_bridge import RosBridge


class FenceAlarmTest(unittest.TestCase):
    def test_freshness_and_unknown(self):
        bridge = RosBridge([])
        bridge._veh["params"] = {"alarm": 0}  # 旧参数不能冒充当前正常状态。
        self.assertIsNone(bridge.vehicle_state_final()["net"]["alarm"])
        with patch("ros_bridge.time.monotonic", return_value=100):
            bridge._on_fence_guard(SimpleNamespace(alarm=0))
            self.assertEqual(bridge.vehicle_state_final()["net"]["alarm"], 0)
            bridge._on_fence_guard(SimpleNamespace(alarm=1))
            self.assertEqual(bridge.vehicle_state_final()["net"]["alarm"], 1)
            bridge._on_fence_guard(SimpleNamespace(alarm=-1))
            self.assertIsNone(bridge.vehicle_state_final()["net"]["alarm"])
            bridge._on_fence_guard(SimpleNamespace(alarm=0))
        with patch("ros_bridge.time.monotonic", return_value=101.01):
            self.assertIsNone(bridge.vehicle_state_final()["net"]["alarm"])
        with patch("ros_bridge.time.monotonic", return_value=99):
            self.assertIsNone(bridge.vehicle_state_final()["net"]["alarm"])
        with patch("ros_bridge.time.monotonic", return_value=102):
            bridge._on_fence_guard(SimpleNamespace(alarm=0))
            self.assertEqual(bridge.vehicle_state_final()["net"]["alarm"], 0)


if __name__ == "__main__":
    unittest.main()
