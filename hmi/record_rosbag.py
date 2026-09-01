#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HMI rosbag 录制启动器。

每次启动依次完成：
1. 创建工程根目录 data/bags；
2. 目录超过 2 GiB 时清空其中的 .bag/.bag.active 文件；
3. 读取 hmi/record_rostopic_list.md 中开关为 1 的 topic；
4. 使用当前时间戳命名并 exec rosbag record。
"""

import os
import re
import shutil
import sys
from datetime import datetime
from pathlib import Path


HMI_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = HMI_DIR.parent
BAG_DIR = PROJECT_ROOT / "data" / "bags"
TOPIC_CONFIG = HMI_DIR / "record_rostopic_list.md"
BAG_SIZE_LIMIT_BYTES = 2 * 1024 * 1024 * 1024

_TOPIC_LINE = re.compile(
    r"^\s*-\s+(/[A-Za-z0-9_./-]+)\s*:\s*([01])\s*(?:#.*)?$"
)


def directory_size_bytes(directory):
    """返回目录中所有普通文件的总字节数，不跟随目录符号链接。"""
    total = 0
    for root, dirs, files in os.walk(str(directory), followlinks=False):
        dirs[:] = [
            name for name in dirs if not (Path(root) / name).is_symlink()
        ]
        for name in files:
            path = Path(root) / name
            try:
                total += path.stat().st_size
            except FileNotFoundError:
                # 允许外部清理任务与本次容量统计并发。
                continue
    return total


def _is_rosbag_file(path):
    name = path.name
    return name.endswith(".bag") or name.endswith(".bag.active")


def clear_rosbags(directory):
    """递归删除目录中的 rosbag 文件，返回 (文件数, 释放字节数)。"""
    deleted = 0
    released = 0
    for root, dirs, files in os.walk(
        str(directory), topdown=False, followlinks=False
    ):
        dirs[:] = [
            name for name in dirs if not (Path(root) / name).is_symlink()
        ]
        for name in files:
            path = Path(root) / name
            if not _is_rosbag_file(path):
                continue
            try:
                size = path.stat().st_size
                path.unlink()
            except FileNotFoundError:
                continue
            deleted += 1
            released += size
    return deleted, released


def cleanup_bag_directory(directory, limit_bytes=BAG_SIZE_LIMIT_BYTES):
    """目录超过阈值时清空 rosbag；返回清理前的目录大小。"""
    size = directory_size_bytes(directory)
    print("[rosbag] 录制目录大小：%.2f MiB" % (size / 1024.0 / 1024.0))
    if size <= limit_bytes:
        return size

    print(
        "[rosbag] 目录超过 %.2f GiB，开始清理旧 rosbag。"
        % (limit_bytes / 1024.0 / 1024.0 / 1024.0)
    )
    deleted, released = clear_rosbags(directory)
    print(
        "[rosbag] 已删除 %d 个文件，释放 %.2f MiB。"
        % (deleted, released / 1024.0 / 1024.0)
    )

    remaining = directory_size_bytes(directory)
    if remaining > limit_bytes:
        raise RuntimeError(
            "清理 rosbag 后目录仍超过限制（%.2f GiB），请检查非 bag 文件"
            % (remaining / 1024.0 / 1024.0 / 1024.0)
        )
    return size


def read_enabled_topics(config_path):
    """解析 Markdown 配置并返回开关为 1 的 topic，配置错误时拒绝录制。"""
    if not config_path.is_file():
        raise RuntimeError("topic 配置文件不存在：%s" % config_path)

    enabled = []
    seen = set()
    with config_path.open("r", encoding="utf-8") as stream:
        for line_no, line in enumerate(stream, 1):
            stripped = line.strip()
            if not stripped.startswith("- /"):
                continue
            match = _TOPIC_LINE.match(line)
            if match is None:
                raise RuntimeError(
                    "topic 配置格式错误：%s:%d" % (config_path, line_no)
                )
            topic, switch = match.groups()
            if topic in seen:
                raise RuntimeError(
                    "topic 重复：%s（%s:%d）" % (topic, config_path, line_no)
                )
            seen.add(topic)
            if switch == "1":
                enabled.append(topic)
    return enabled


def timestamped_bag_path(directory, now=None):
    """生成毫秒级时间戳文件名，避免一秒内快速重启时重名。"""
    now = now or datetime.now()
    stamp = now.strftime("%Y%m%d_%H%M%S_%f")[:-3]
    return directory / (stamp + ".bag")


def main():
    try:
        BAG_DIR.mkdir(parents=True, exist_ok=True)
        cleanup_bag_directory(BAG_DIR)

        topics = read_enabled_topics(TOPIC_CONFIG)
        if not topics:
            raise RuntimeError(
                "没有启用任何 topic；请先在 "
                "hmi/record_rostopic_list.md 中将需要录制的值改为 1"
            )

        rosbag = shutil.which("rosbag")
        if rosbag is None:
            raise RuntimeError("找不到 rosbag 命令，请检查 ROS 环境是否已 source")

        output = timestamped_bag_path(BAG_DIR)
        argv = [rosbag, "record", "-O", str(output)] + topics
        print(
            "[rosbag] 本次录制 %d 个 topic：%s"
            % (len(topics), " ".join(topics))
        )
        print("[rosbag] 输出文件：%s" % output)
        sys.stdout.flush()
        sys.stderr.flush()
        os.execv(rosbag, argv)
    except Exception as exc:
        print("[rosbag] 启动失败：%s" % exc, file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
