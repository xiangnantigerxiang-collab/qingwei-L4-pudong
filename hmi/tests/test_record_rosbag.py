#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import importlib.util
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
    def test_project_config_parses(self):
        # 出厂配置是操作员会话级开关(随时改 0/1),不断言具体取值;
        # 只要求格式合法:能被解析且无重复 topic。
        enabled = record_rosbag.read_enabled_topics(
            record_rosbag.TOPIC_CONFIG
        )
        self.assertEqual(len(enabled), len(set(enabled)))

    def test_read_enabled_topics(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "topics.md"
            config.write_text(
                "# topics\n- /disabled: 0\n- /enabled: 1\n",
                encoding="utf-8",
            )
            self.assertEqual(
                record_rosbag.read_enabled_topics(config), ["/enabled"]
            )

    def test_rejects_duplicate_topic(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "topics.md"
            config.write_text(
                "- /same: 0\n- /same: 1\n", encoding="utf-8"
            )
            with self.assertRaisesRegex(RuntimeError, "topic 重复"):
                record_rosbag.read_enabled_topics(config)

    def test_rejects_invalid_switch(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "topics.md"
            config.write_text("- /invalid: 2\n", encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "配置格式错误"):
                record_rosbag.read_enabled_topics(config)

    def test_cleanup_deletes_only_rosbag_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag_dir = Path(tmp)
            (bag_dir / "old.bag").write_bytes(b"b" * 64)
            (bag_dir / "interrupted.bag.active").write_bytes(b"a" * 64)
            (bag_dir / "keep.txt").write_text("keep", encoding="utf-8")

            record_rosbag.cleanup_bag_directory(bag_dir, limit_bytes=32)

            self.assertFalse((bag_dir / "old.bag").exists())
            self.assertFalse((bag_dir / "interrupted.bag.active").exists())
            self.assertTrue((bag_dir / "keep.txt").exists())

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
            config.write_text("- /one: 1\n- /two: 1\n", encoding="utf-8")

            with mock.patch.object(record_rosbag, "BAG_DIR", bag_dir), \
                    mock.patch.object(record_rosbag, "TOPIC_CONFIG", config), \
                    mock.patch.object(
                        record_rosbag.shutil,
                        "which",
                        return_value="/opt/ros/noetic/bin/rosbag",
                    ), \
                    mock.patch.object(record_rosbag.os, "execv") as execv:
                self.assertEqual(record_rosbag.main(), 0)

            executable, argv = execv.call_args.args
            self.assertEqual(executable, "/opt/ros/noetic/bin/rosbag")
            self.assertEqual(argv[0:2], [executable, "record"])
            self.assertEqual(argv[2], "-O")
            self.assertEqual(Path(argv[3]).parent, bag_dir)
            self.assertRegex(Path(argv[3]).name, r"^\d{8}_\d{6}_\d{3}\.bag$")
            self.assertEqual(argv[4:], ["/one", "/two"])


if __name__ == "__main__":
    unittest.main()
