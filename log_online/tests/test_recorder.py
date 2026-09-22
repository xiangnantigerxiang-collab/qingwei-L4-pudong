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

ros = mock_ros.install()
from robot.msg import can_msg as can_mod, navigation_msg as nav_mod  # noqa: E402
import recorder  # noqa: E402


class RecorderHarness(object):
    """假时钟驱动 Recorder,消息时间戳由 tick 直接置新鲜。"""

    def __init__(self, tmpdir):
        self.tmp = tmpdir
        self.ros = mock_ros.install()
        self.rec = recorder.Recorder(self.ros, cfg, Path(tmpdir) / "events")
        self.rec.wire()

    def feed_ok(self, now, **over):
        can = can_mod.can_msg(vehicleSpeed=1.0, curGear=4, controlPanelState=1,
                              emergencyStop=over.get("emergencyStop", 0),
                              faultCode=over.get("faultCode", b""))
        self.rec._on_can(can)
        self.rec._on_navigation(nav_mod.navigation_msg(xAxis=1.0, yAxis=2.0))
        # 其余链路缺省 -> stale 但不触发
        for key, value in (("sensorstate", over.get("sensorstate", 0)),):
            self.ros.params[cfg.PARAM_SENSORSTATE] = value
        self.ros.params[cfg.PARAM_NETCHECK] = over.get("netcheck", 0)

    def drive(self, now, **over):
        self.feed_ok(now, **over)
        self.rec.tick(now)


class TestRecorderStartup(unittest.TestCase):
    def test_run_schedules_sampling(self):
        for sample_hz in (10, 20):
            with self.subTest(sample_hz=sample_hz), tempfile.TemporaryDirectory() as tmp:
                ros = mock_ros._FakeRospy()
                rec = recorder.Recorder(ros, cfg, Path(tmp) / "events")
                with mock.patch.object(cfg, "SAMPLE_HZ", sample_hz), \
                        mock.patch.object(ros, "Timer", wraps=ros.Timer) as timer, \
                        mock.patch.object(ros, "spin") as spin:
                    rec.run()
                timer.assert_called_once()
                spin.assert_called_once_with()
                period, callback = timer.call_args[0]
                self.assertAlmostEqual(period.to_sec(), 1.0 / sample_hz)
                # 驱动实际注册的回调,确认启动路径接到了快照采样。
                for now in (1000.0, 1000.0 + period.to_sec()):
                    with mock.patch("recorder.time.time", return_value=now):
                        callback(object())
                self.assertEqual(len(rec._ring), 2)
                self.assertAlmostEqual(rec._ring.span(now), period.to_sec())


class TestRecorderBasics(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.h = RecorderHarness(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def test_ring_and_idle(self):
        for i in range(50):
            self.h.drive(1000.0 + i * 0.1)
        st = self.rec_status()
        self.assertEqual(st["state"], "IDLE")
        self.assertEqual(len(self.h.rec._ring), 50)

    def test_debounce_then_event(self):
        for i in range(5):
            self.h.drive(2000.0 + i * 0.1, emergencyStop=1)
        self.assertEqual(self.rec_status()["state"], "RECORDING")

    def test_no_trigger_when_can_missing(self):
        # 只有 tick,无任何消息
        for i in range(10):
            self.h.rec.tick(3000.0 + i * 0.1)
        self.assertEqual(self.rec_status()["state"], "IDLE")

    def test_get_status_segments_is_copy(self):
        for i in range(5):
            self.h.drive(2000.0 + i * 0.1, emergencyStop=1)
        self.assertEqual(self.h.rec._machine.state, "RECORDING")
        st = self.h.rec.get_status()
        segs = st["trigger"]["segments"]
        self.assertEqual(len(segs), 1)
        # 篡改返回值不得影响状态机内部(Task 8 HTTP 线程序列化活引用会撕裂)
        segs.append({"start": 0.0, "end": 1.0, "types": ["fake"]})
        segs[0]["end"] = 123.4
        st2 = self.h.rec.get_status()
        self.assertEqual(len(st2["trigger"]["segments"]), 1)
        self.assertIsNone(st2["trigger"]["segments"][0]["end"])
        self.assertEqual(len(self.h.rec._machine.segments), 1)
        self.assertIsNone(self.h.rec._machine.segments[0]["end"])

    def test_rotator_scan_tolerates_vanishing_files(self):
        data_dir = Path(self._tmp.name) / "events2"
        (data_dir / "ev_1").mkdir(parents=True)
        (data_dir / "ev_1" / "records.jsonl").write_text("x")
        rot = recorder.Rotator(data_dir)
        with mock.patch("pathlib.Path.rglob", side_effect=OSError("vanished")):
            self.assertEqual(rot.scan(), 0)  # 统计途中消失:不抛,不计入

    def test_startup_rotation_evicts_oldest(self):
        # 构造 Recorder 即执行一次启动轮转(DESIGN §9):1800B > cap 1000 -> 删最旧至 ≤900B
        data_dir = Path(self._tmp.name) / "events_startup"
        for name in ("a_1", "b_2", "c_3"):
            (data_dir / name).mkdir(parents=True)
            (data_dir / name / "records.jsonl").write_bytes(b"x" * 600)
        with mock.patch.object(cfg, "DISK_CAP_BYTES", 1000):
            recorder.Recorder(mock_ros._FakeRospy(), cfg, data_dir)
        self.assertFalse((data_dir / "a_1").exists())
        self.assertFalse((data_dir / "b_2").exists())
        self.assertTrue((data_dir / "c_3").exists())
        total = sum(f.stat().st_size for f in data_dir.rglob("*") if f.is_file())
        self.assertEqual(total, 600)
        self.assertLessEqual(total, 1000 * cfg.ROTATE_KEEP_RATIO)

    def test_disk_write_failure_drops_event_not_tick(self):
        # 磁盘写失败(open 抛 OSError):tick 不抛、丢弃本事件(writer=None)、
        # 轮转兜底仍执行;恢复后第二个事件正常落盘,机制不卡死
        with mock.patch.object(recorder.EventWriter, "open",
                               side_effect=OSError("disk full")), \
                mock.patch.object(self.h.rec._rotator, "evict_if_over",
                                  wraps=self.h.rec._rotator.evict_if_over) as evict:
            for i in range(10):  # 第 5 拍去抖通过 -> open 失败
                self.h.drive(4000.0 + i * 0.1, emergencyStop=1)
        self.assertIsNone(self.h.rec._writer)
        self.assertEqual(self.h.rec._machine.state, "RECORDING")
        self.assertGreaterEqual(evict.call_count, 1)
        # 触发清除 + 走完 post 窗:writer 为 None 各拍自然跳过,状态机能回 IDLE
        t = 4010.0
        while t < 4105.0 - 1e-9:
            self.h.drive(t)
            t += 0.5
        self.assertEqual(self.h.rec._machine.state, "IDLE")
        # open 恢复正常后第二个事件正常落盘
        for i in range(5):
            self.h.drive(4110.0 + i * 0.1, emergencyStop=1)
        self.assertIsNotNone(self.h.rec._writer)
        self.assertEqual(self.h.rec._machine.state, "RECORDING")
        events = [d for d in self.h.rec.data_dir.iterdir() if d.is_dir()]
        self.assertEqual(len(events), 1)
        self.assertTrue((events[0] / "records.jsonl").exists())

    def test_extend_refresh_failure_drops_event(self):
        # 事件开启 -> 清除进 POST -> post 窗内再触发(extend 拍)时
        # refresh_metadata 抛 OSError:tick 不抛、丢事件 + 轮转兜底一次,
        # 状态机照常推进最终回 IDLE(与 open/close 失败同款善后)
        for i in range(5):
            self.h.drive(5000.0 + i * 0.1, emergencyStop=1)
        self.assertIsNotNone(self.h.rec._writer)
        self.h.drive(5001.0)  # 清除 -> POST
        self.assertEqual(self.h.rec._machine.state, "POST")
        with mock.patch.object(recorder.EventWriter, "refresh_metadata",
                               side_effect=OSError(28, "No space left on device")), \
                mock.patch.object(self.h.rec._rotator, "evict_if_over",
                                  wraps=self.h.rec._rotator.evict_if_over) as evict:
            self.h.drive(5050.0, emergencyStop=1)  # extend 拍
        self.assertIsNone(self.h.rec._writer)
        self.assertEqual(self.h.rec._machine.state, "RECORDING")
        self.assertGreaterEqual(evict.call_count, 1)
        # 清除 + 走完 post 窗:writer 为 None 各拍自然跳过,回 IDLE
        t = 5060.0
        while t < 5155.0 - 1e-9:
            self.h.drive(t)
            t += 0.5
        self.assertEqual(self.h.rec._machine.state, "IDLE")

    def rec_status(self):
        return self.h.rec.get_status()


class TestResolvePort(unittest.TestCase):
    def test_missing_valid_and_invalid(self):
        # 缺省:环境变量未设 -> 配置默认端口
        self.assertEqual(recorder._resolve_port(lambda _key: None, 8083), 8083)
        # 合法数字:按环境变量
        self.assertEqual(recorder._resolve_port(lambda _key: "8090", 8083), 8090)
        # 非法值:stderr 告警并回落默认
        with mock.patch("sys.stderr") as fake_err:
            self.assertEqual(recorder._resolve_port(lambda _key: "808x", 8083), 8083)
        self.assertTrue(fake_err.write.called)


if __name__ == "__main__":
    unittest.main()
