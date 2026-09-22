# -*- coding: utf-8 -*-
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mock_ros  # noqa: E402

mock_ros.install()
import recorder  # noqa: E402

T_NONE, T_ES = [], ["emergencystop"]


class TestStateMachine(unittest.TestCase):
    def test_full_lifecycle(self):
        m = recorder.EventStateMachine(90.0, 90.0)
        # t=100 去抖通过开事件
        self.assertEqual(m.feed(100.0, True, T_ES), [("open", "emergencystop")])
        self.assertEqual(m.state, "RECORDING")
        self.assertEqual(m.segments, [{"start": 100.0, "end": None, "types": T_ES}])
        # 持续激活:无动作(调用方照 state 追加)
        self.assertEqual(m.feed(100.1, False, T_NONE), [])
        self.assertEqual(m.state, "POST")  # 触发清除 -> POST
        self.assertEqual(m.segments[-1]["end"], 100.1)
        # post 未满:无动作
        self.assertEqual(m.feed(190.0, False, T_NONE), [])
        # post 满:close
        self.assertEqual(m.feed(190.1, False, T_NONE), [("close",)])
        self.assertEqual(m.state, "IDLE")

    def test_post_retrigger_extends(self):
        m = recorder.EventStateMachine(90.0, 90.0)
        m.feed(100.0, True, T_ES)
        m.feed(100.1, False, T_NONE)
        # post 期间(第 50s)再次触发(无需去抖)
        self.assertEqual(m.feed(150.0, False, ["sensorstate"]), [("extend", ["sensorstate"])])
        self.assertEqual(m.state, "RECORDING")
        self.assertEqual(len(m.segments), 2)
        self.assertEqual(m.segments[1]["types"], ["sensorstate"])
        # 再清除后重新计满 90s 才关
        m.feed(150.1, False, T_NONE)
        self.assertEqual(m.feed(240.0, False, T_NONE), [])
        self.assertEqual(m.feed(240.2, False, T_NONE), [("close",)])

    def test_segments_reset_between_events(self):
        m = recorder.EventStateMachine(90.0, 90.0)
        # 事件一:post 窗内扩展出第 2 段,直到 close
        m.feed(100.0, True, T_ES)
        m.feed(100.1, False, T_NONE)
        self.assertEqual(m.feed(150.0, False, ["sensorstate"]),
                         [("extend", ["sensorstate"])])
        m.feed(150.1, False, T_NONE)
        self.assertEqual(m.feed(240.2, False, T_NONE), [("close",)])
        self.assertEqual(len(m.segments), 2)
        # 事件二:open 时段表重建,只含新事件首段(上一事件的段不得混入)
        self.assertEqual(m.feed(1000.0, True, T_ES), [("open", "emergencystop")])
        self.assertEqual(m.segments,
                         [{"start": 1000.0, "end": None, "types": T_ES}])

    def test_debounce_only_from_idle(self):
        m = recorder.EventStateMachine(90.0, 90.0)
        # 去抖未通过
        self.assertEqual(m.feed(100.0, False, T_ES), [])
        self.assertEqual(m.state, "IDLE")


if __name__ == "__main__":
    unittest.main()
