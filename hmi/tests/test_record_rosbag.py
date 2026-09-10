#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import importlib.util
import os
import re
import tempfile
import unittest
from datetime import datetime
from pathlib import Path
from unittest import mock


MODULE_PATH = Path(__file__).resolve().parents[1] / "record_rosbag.py"
SPEC = importlib.util.spec_from_file_location(
    "record_rosbag", str(MODULE_PATH)
)
record_rosbag = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(record_rosbag)


class RecordRosbagTest(unittest.TestCase):
    def test_production_config_uses_two_manual_recorders(self):
        import hmi_config

        components = {
            component["name"]: component
            for component in hmi_config.CONFIG["components"]
        }
        self.assertNotIn("bags", components)
        # data_logger 已整包删除（09-08），锁死不再回归
        self.assertNotIn("data_logger", components)

        perception = components["perception_bags"]
        self.assertEqual(perception["title"], "感知数据录制")
        self.assertEqual(perception["group"], 2)
        self.assertFalse(perception["enabled"])
        self.assertEqual(perception["cmd"][-1], "perception")
        perception_pat = re.compile(perception["stop_pat"])
        self.assertIsNotNone(perception_pat.search(
            "python3 /project/hmi/record_rosbag.py perception"
        ))
        self.assertIsNotNone(perception_pat.search(
            "/opt/ros/noetic/bin/rosbag record -O "
            "/project/data/bags/perception/20260901.bag /box"
        ))

        pnc = components["pnc_bags"]
        self.assertEqual(pnc["title"], "规控数据录制")
        self.assertEqual(pnc["group"], 3)
        self.assertFalse(pnc["enabled"])
        self.assertEqual(pnc["cmd"][-1], "pnc")
        pnc_pat = re.compile(pnc["stop_pat"])
        self.assertIsNotNone(pnc_pat.search(
            "python3 /project/hmi/record_rosbag.py pnc"
        ))
        self.assertIsNotNone(pnc_pat.search(
            "/opt/ros/noetic/bin/rosbag record -O "
            "/project/data/bags/pnc/20260901.bag /control_msg"
        ))
        self.assertNotEqual(perception["stop_pat"], pnc["stop_pat"])

        unrelated_commands = [
            "scp /project/data/bags/perception/a.bag vehicle:/archive/",
            "rsync /project/data/bags/pnc/ /archive/",
            "rosbag info /project/data/bags/perception/a.bag",
            "tar -cf bags.tar /project/data/bags/pnc/",
        ]
        for command in unrelated_commands:
            with self.subTest(command=command):
                self.assertIsNone(perception_pat.search(command))
                self.assertIsNone(pnc_pat.search(command))

    def test_project_config_parses(self):
        # 出厂配置是操作员会话级开关(随时改 0/1),不断言具体取值;
        # 只要求两个分组格式合法，且各自没有重复 topic。
        groups = record_rosbag.read_topic_groups(record_rosbag.TOPIC_CONFIG)
        self.assertEqual(set(groups), {"perception", "pnc"})
        for enabled in groups.values():
            self.assertEqual(len(enabled), len(set(enabled)))

    def test_recording_config_monitor_reloads_and_exposes_error(self):
        import hmi_server

        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "topics.md"
            config.write_text(
                "## 感知数据录制\n- /shared: 1\n"
                "## 规控数据录制\n- /shared: 0\n",
                encoding="utf-8",
            )
            monitor = hmi_server.RecordingConfigMonitor(True, config)
            self.assertEqual(
                monitor.snapshot(),
                {"enabled": True, "ok": True, "error": None},
            )

            config.write_text(
                "## 感知数据录制\n- /only_perception: 1\n"
                "## 规控数据录制\n- /only_pnc: 1\n",
                encoding="utf-8",
            )
            status = monitor.snapshot()
            self.assertFalse(status["ok"])
            self.assertIn("topic 清单不一致", status["error"])

    def test_disabled_recording_config_monitor_ignores_missing_file(self):
        import hmi_server

        monitor = hmi_server.RecordingConfigMonitor(
            False, Path("/definitely/missing/topics.md")
        )
        self.assertEqual(
            monitor.snapshot(),
            {"enabled": False, "ok": True, "error": None},
        )

    def test_read_enabled_topics(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "topics.md"
            config.write_text(
                "# topics\n"
                "## 感知数据录制\n"
                "- /disabled: 0\n- /enabled: 1\n- /shared: 1\n"
                "## 规控数据录制\n"
                "- /pnc: 1\n- /shared: 0\n",
                encoding="utf-8",
            )
            self.assertEqual(
                record_rosbag.read_enabled_topics(config, "perception"),
                ["/enabled", "/shared"],
            )
            self.assertEqual(
                record_rosbag.read_enabled_topics(config, "pnc"), ["/pnc"]
            )

    def test_rejects_duplicate_topic(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "topics.md"
            config.write_text(
                "## 感知数据录制\n- /same: 0\n- /same: 1\n"
                "## 规控数据录制\n- /same: 0\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(RuntimeError, "topic 重复"):
                record_rosbag.read_topic_groups(config)

    def test_rejects_invalid_switch(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "topics.md"
            config.write_text(
                "## 感知数据录制\n- /invalid: 2\n"
                "## 规控数据录制\n- /valid: 0\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(RuntimeError, "配置格式错误"):
                record_rosbag.read_topic_groups(config)

    def test_selected_group_isolated_from_invalid_other_group(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "topics.md"
            config.write_text(
                "## 感知数据录制\n- /valid: 1\n"
                "## 规控数据录制\n- /invalid: 2\n",
                encoding="utf-8",
            )
            self.assertEqual(
                record_rosbag.read_enabled_topics(config, "perception"),
                ["/valid"],
            )

    def test_rejects_topic_without_absolute_name(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "topics.md"
            config.write_text(
                "## 感知数据录制\n- relative_topic: 1\n- /valid: 1\n"
                "## 规控数据录制\n- /valid: 1\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(RuntimeError, "配置格式错误"):
                record_rosbag.read_enabled_topics(config, "perception")

    def test_rejects_every_malformed_list_item_in_selected_group(self):
        malformed_lines = (
            "-/missing_space: 1",
            "- topic name: 1",
            "- /missing_switch:",
            "- /invalid_switch: 2",
        )
        for malformed in malformed_lines:
            with self.subTest(line=malformed), \
                    tempfile.TemporaryDirectory() as tmp:
                config = Path(tmp) / "topics.md"
                config.write_text(
                    "## 感知数据录制\n%s\n"
                    "## 规控数据录制\n- /valid: 1\n" % malformed,
                    encoding="utf-8",
                )
                with self.assertRaisesRegex(RuntimeError, "配置格式错误"):
                    record_rosbag.read_enabled_topics(config, "perception")

    def test_rejects_duplicate_recording_section(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "topics.md"
            config.write_text(
                "## 感知数据录制\n- /one: 1\n"
                "## 感知数据录制\n- /two: 0\n"
                "## 规控数据录制\n- /one: 1\n- /two: 0\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(RuntimeError, "topic 分组重复"):
                record_rosbag.read_topic_groups(config)

    def test_rejects_mismatched_topic_inventories(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "topics.md"
            config.write_text(
                "## 感知数据录制\n- /shared: 1\n- /only_perception: 0\n"
                "## 规控数据录制\n- /shared: 1\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(RuntimeError, "topic 清单不一致"):
                record_rosbag.read_topic_groups(config)

    def test_rejects_missing_recording_group(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "topics.md"
            config.write_text(
                "## 感知数据录制\n- /one: 1\n", encoding="utf-8"
            )
            with self.assertRaisesRegex(RuntimeError, "缺少 topic 分组"):
                record_rosbag.read_topic_groups(config)

    def test_rejects_topic_outside_recording_group(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "topics.md"
            config.write_text(
                "# topics\n- /orphan: 1\n"
                "## 感知数据录制\n- /one: 1\n"
                "## 规控数据录制\n- /two: 1\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(RuntimeError, "topic 未归入"):
                record_rosbag.read_topic_groups(config)

    def test_cleanup_deletes_completed_bag_but_keeps_active_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag_dir = Path(tmp)
            (bag_dir / "old.bag").write_bytes(b"b" * 64)
            (bag_dir / "interrupted.bag.active").write_bytes(b"a" * 64)
            (bag_dir / "keep.txt").write_text("keep", encoding="utf-8")

            record_rosbag.cleanup_bag_directory(bag_dir, limit_bytes=70)

            self.assertFalse((bag_dir / "old.bag").exists())
            self.assertTrue((bag_dir / "interrupted.bag.active").exists())
            self.assertTrue((bag_dir / "keep.txt").exists())

    def test_cleanup_deletes_only_oldest_files_needed(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag_dir = Path(tmp)
            oldest = bag_dir / "oldest.bag"
            middle = bag_dir / "middle.bag"
            newest = bag_dir / "newest.bag"
            for bag in (oldest, middle, newest):
                bag.write_bytes(b"x" * 40)
            os.utime(str(oldest), (1, 1))
            os.utime(str(middle), (2, 2))
            os.utime(str(newest), (3, 3))

            record_rosbag.cleanup_bag_directory(bag_dir, limit_bytes=85)

            self.assertFalse(oldest.exists())
            self.assertTrue(middle.exists())
            self.assertTrue(newest.exists())

    def test_cleanup_keeps_files_within_limit(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "small.bag"
            bag.write_bytes(b"small")

            record_rosbag.cleanup_bag_directory(
                Path(tmp), limit_bytes=1024
            )

            self.assertTrue(bag.exists())

    def test_cleanup_rejects_oversized_non_bag_content(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag_dir = Path(tmp)
            (bag_dir / "keep.txt").write_bytes(b"x" * 64)

            with self.assertRaisesRegex(RuntimeError, "仍超过限制"):
                record_rosbag.cleanup_bag_directory(
                    bag_dir, limit_bytes=32
                )

    def test_cleanup_rejects_oversized_active_bag_without_deleting_it(self):
        with tempfile.TemporaryDirectory() as tmp:
            active = Path(tmp) / "recording.bag.active"
            active.write_bytes(b"x" * 64)

            with self.assertRaisesRegex(RuntimeError, "bag.active"):
                record_rosbag.cleanup_bag_directory(
                    Path(tmp), limit_bytes=32
                )

            self.assertTrue(active.exists())

    def test_timestamped_bag_path(self):
        path = record_rosbag.timestamped_bag_path(
            Path("/data/bags"),
            datetime(2026, 9, 1, 12, 34, 56, 789123),
        )
        self.assertEqual(path.name, "20260901_123456_789.bag")

    def test_main_execs_rosbag_with_enabled_topics_and_output_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag_dir = Path(tmp) / "data" / "bags"
            config = Path(tmp) / "topics.md"
            config.write_text(
                "## 感知数据录制\n- /perception: 1\n"
                "## 规控数据录制\n- /one: 1\n- /two: 1\n",
                encoding="utf-8",
            )

            with mock.patch.object(record_rosbag, "BAG_DIR", bag_dir), \
                    mock.patch.object(record_rosbag, "TOPIC_CONFIG", config), \
                    mock.patch.object(
                        record_rosbag.shutil,
                        "which",
                        return_value="/opt/ros/noetic/bin/rosbag",
                    ), \
                    mock.patch.object(record_rosbag.os, "execv") as execv:
                self.assertEqual(record_rosbag.main(["pnc"]), 0)

            executable, argv = execv.call_args.args
            self.assertEqual(executable, "/opt/ros/noetic/bin/rosbag")
            self.assertEqual(argv[0:2], [executable, "record"])
            self.assertEqual(argv[2], "-O")
            self.assertEqual(Path(argv[3]).parent, bag_dir / "pnc")
            self.assertRegex(Path(argv[3]).name, r"^\d{8}_\d{6}_\d{3}\.bag$")
            self.assertEqual(argv[4:], ["/one", "/two"])

    def test_main_rejects_empty_topics_before_cleanup(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "topics.md"
            config.write_text(
                "## 感知数据录制\n- /disabled: 0\n"
                "## 规控数据录制\n- /enabled: 1\n",
                encoding="utf-8",
            )
            with mock.patch.object(record_rosbag, "TOPIC_CONFIG", config), \
                    mock.patch.object(
                        record_rosbag, "cleanup_bag_directory"
                    ) as cleanup:
                self.assertEqual(record_rosbag.main(["perception"]), 2)
            cleanup.assert_not_called()

    def test_main_checks_rosbag_command_before_cleanup(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "topics.md"
            config.write_text(
                "## 感知数据录制\n- /enabled: 1\n"
                "## 规控数据录制\n- /enabled: 1\n",
                encoding="utf-8",
            )
            with mock.patch.object(record_rosbag, "TOPIC_CONFIG", config), \
                    mock.patch.object(
                        record_rosbag.shutil, "which", return_value=None
                    ), \
                    mock.patch.object(
                        record_rosbag, "cleanup_bag_directory"
                    ) as cleanup:
                self.assertEqual(record_rosbag.main(["perception"]), 2)
            cleanup.assert_not_called()

    def test_process_manager_refuses_starting_foreign_component(self):
        import process_manager

        with tempfile.TemporaryDirectory() as tmp:
            config = {
                "groups": {0: "test"},
                "defaults": {
                    "start_timeout": 1,
                    "health": [],
                    "health_lost_s": 1,
                    "max_log_mb": 1,
                    "keep_logs": 1,
                    "ready_s": 0,
                },
                "components": [{
                    "name": "manual",
                    "title": "手动组件",
                    "group": 0,
                    "cmd": ["true"],
                    "cwd": tmp,
                }],
            }
            manager = process_manager.ProcessManager(
                config,
                Path(tmp),
                process_manager.HealthProvider(),
            )
            runner = manager.runners["manual"]
            runner.foreign = True

            self.assertEqual(
                manager.start_component("manual"), (False, "FOREIGN")
            )
            self.assertEqual(
                manager.restart_component("manual"), (False, "FOREIGN")
            )

            with mock.patch.object(process_manager.time, "sleep"), \
                    mock.patch.object(
                        process_manager, "_pgrep", return_value=False
                    ):
                self.assertEqual(
                    manager.stop_component("manual"), (True, "STOPPED")
                )
            self.assertFalse(runner.foreign)
            self.assertIsNone(runner.proc)


if __name__ == "__main__":
    unittest.main()
