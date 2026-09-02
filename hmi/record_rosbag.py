#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HMI 感知/规控 rosbag 录制启动器。

每次启动依次完成：
1. 根据命令行参数选择“感知数据录制”或“规控数据录制”；
2. 创建对应的 data/bags/<类型> 独立目录；
3. 读取并校验 hmi/record_rostopic_list.md 中所选分组；
4. 确认 rosbag 命令可用；
5. 目录超过 2 GiB 时按时间清理最旧的已完成 .bag 文件；
6. 使用当前时间戳命名并 exec rosbag record。
"""

import argparse
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

RECORDINGS = {
    "perception": {
        "title": "感知数据录制",
        "directory": "perception",
    },
    "pnc": {
        "title": "规控数据录制",
        "directory": "pnc",
    },
}
_SECTION_TO_RECORDING = {
    spec["title"]: name for name, spec in RECORDINGS.items()
}

_TOPIC_LINE = re.compile(
    r"^\s*-\s+(/[A-Za-z0-9_./-]+)\s*:\s*([01])\s*(?:#.*)?$"
)
# 在两个录制分组之外，仅把“像 topic 开关”的项目视为误放配置；说明区的
# 普通 Markdown 列表仍允许存在。进入录制分组后，所有列表项都必须严格
# 匹配 _TOPIC_LINE，从而避免 `-/topic: 1`、`- topic name: 1` 被静默忽略。
_OUTSIDE_TOPIC_CANDIDATE = re.compile(r"^\s*-\s*(?:/|.+:\s*)")
_SECTION_LINE = re.compile(r"^\s*##\s+(.+?)\s*$")


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
    """只把已完成的 .bag 视为可自动清理文件。"""
    name = path.name
    return name.endswith(".bag")


def clear_rosbags(directory, bytes_to_release=None):
    """按从旧到新删除已完成 rosbag，返回 (文件数, 释放字节数)。

    .bag.active 可能仍由 rosbag 写入，绝不自动删除。bytes_to_release 为 None
    时删除全部已完成 bag；否则释放达到目标字节数即停止。
    """
    deleted = 0
    released = 0
    candidates = []
    for root, dirs, files in os.walk(str(directory), followlinks=False):
        dirs[:] = [
            name for name in dirs if not (Path(root) / name).is_symlink()
        ]
        for name in files:
            path = Path(root) / name
            if not _is_rosbag_file(path) or path.is_symlink():
                continue
            try:
                stat = path.stat()
            except FileNotFoundError:
                continue
            candidates.append((stat.st_mtime, str(path), path, stat.st_size))

    candidates.sort(key=lambda item: (item[0], item[1]))
    for _mtime, _name, path, size in candidates:
        if bytes_to_release is not None and released >= bytes_to_release:
            break
        try:
            path.unlink()
        except FileNotFoundError:
            continue
        deleted += 1
        released += size
    return deleted, released


def cleanup_bag_directory(directory, limit_bytes=BAG_SIZE_LIMIT_BYTES):
    """目录超过阈值时按需清理最旧的已完成 bag；返回清理前大小。"""
    size = directory_size_bytes(directory)
    print("[rosbag] 录制目录大小：%.2f MiB" % (size / 1024.0 / 1024.0))
    if size <= limit_bytes:
        return size

    print(
        "[rosbag] 目录超过 %.2f GiB，开始清理最旧的已完成 rosbag。"
        % (limit_bytes / 1024.0 / 1024.0 / 1024.0)
    )
    deleted, released = clear_rosbags(
        directory, bytes_to_release=size - limit_bytes
    )
    print(
        "[rosbag] 已删除 %d 个文件，释放 %.2f MiB。"
        % (deleted, released / 1024.0 / 1024.0)
    )

    remaining = directory_size_bytes(directory)
    if remaining > limit_bytes:
        raise RuntimeError(
            "清理已完成 rosbag 后目录仍超过限制（%.2f GiB），请检查"
            "非 bag 文件或仍在写入的 .bag.active"
            % (remaining / 1024.0 / 1024.0 / 1024.0)
        )
    return size


def _parse_topic_groups(config_path, selected=None):
    """解析 topic 配置；selected 指定时只校验该录制分组。"""
    if not config_path.is_file():
        raise RuntimeError("topic 配置文件不存在：%s" % config_path)

    enabled = {name: [] for name in RECORDINGS}
    seen = {name: set() for name in RECORDINGS}
    section_counts = {name: 0 for name in RECORDINGS}
    current = None
    with config_path.open("r", encoding="utf-8") as stream:
        for line_no, line in enumerate(stream, 1):
            stripped = line.strip()
            section_match = _SECTION_LINE.match(line)
            if section_match is not None:
                current = _SECTION_TO_RECORDING.get(
                    section_match.group(1).strip()
                )
                if current is not None:
                    section_counts[current] += 1
                    if section_counts[current] > 1 and (
                        selected is None or selected == current
                    ):
                        raise RuntimeError(
                            "topic 分组重复：%s（%s:%d）"
                            % (RECORDINGS[current]["title"], config_path, line_no)
                        )
                continue
            if current is None:
                if _OUTSIDE_TOPIC_CANDIDATE.match(line) is None:
                    continue
                if selected is None:
                    raise RuntimeError(
                        "topic 未归入感知数据录制或规控数据录制分组：%s:%d"
                        % (config_path, line_no)
                    )
                continue
            if selected is not None and current != selected:
                continue
            if not stripped.startswith("-"):
                continue
            match = _TOPIC_LINE.match(line)
            if match is None:
                raise RuntimeError(
                    "topic 配置格式错误：%s:%d" % (config_path, line_no)
                )
            topic, switch = match.groups()
            if topic in seen[current]:
                raise RuntimeError(
                    "%s分组 topic 重复：%s（%s:%d）"
                    % (RECORDINGS[current]["title"], topic, config_path, line_no)
                )
            seen[current].add(topic)
            if switch == "1":
                enabled[current].append(topic)

    required = [selected] if selected is not None else list(RECORDINGS)
    missing = [
        RECORDINGS[name]["title"]
        for name in required
        if section_counts[name] == 0
    ]
    if missing:
        raise RuntimeError("缺少 topic 分组：%s" % "、".join(missing))

    if selected is None and seen["perception"] != seen["pnc"]:
        only_perception = sorted(seen["perception"] - seen["pnc"])
        only_pnc = sorted(seen["pnc"] - seen["perception"])
        detail = []
        if only_perception:
            detail.append("仅感知：%s" % " ".join(only_perception[:3]))
        if only_pnc:
            detail.append("仅规控：%s" % " ".join(only_pnc[:3]))
        raise RuntimeError("两个录制分组的 topic 清单不一致（%s）" % "；".join(detail))
    return enabled


def read_topic_groups(config_path):
    """完整校验 Markdown 配置并返回两个分组中开关为 1 的 topic。"""
    return _parse_topic_groups(config_path)


def read_enabled_topics(config_path, recording):
    """仅校验并返回指定录制类型中开关为 1 的 topic。"""
    if recording not in RECORDINGS:
        raise RuntimeError("未知录制类型：%s" % recording)
    return _parse_topic_groups(config_path, selected=recording)[recording]


def timestamped_bag_path(directory, now=None):
    """生成毫秒级时间戳文件名，避免一秒内快速重启时重名。"""
    now = now or datetime.now()
    stamp = now.strftime("%Y%m%d_%H%M%S_%f")[:-3]
    return directory / (stamp + ".bag")


def _parse_args(argv):
    parser = argparse.ArgumentParser(description="HMI rosbag 数据录制")
    parser.add_argument(
        "recording",
        choices=sorted(RECORDINGS),
        help="录制类型：perception（感知）或 pnc（规控）",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    recording = RECORDINGS[args.recording]
    bag_dir = BAG_DIR / recording["directory"]
    try:
        topics = read_enabled_topics(TOPIC_CONFIG, args.recording)
        if not topics:
            raise RuntimeError(
                "%s分组没有启用任何 topic；请先在 "
                "hmi/record_rostopic_list.md 中将需要录制的值改为 1"
                % recording["title"]
            )

        # 不让另一组错误阻断本次录制，但把跨组结构问题写入组件日志，
        # 便于车端编辑后无需跑开发测试也能发现清单漂移。
        try:
            read_topic_groups(TOPIC_CONFIG)
        except RuntimeError as lint_error:
            print(
                "[rosbag:%s] 配置全量检查警告：%s；本次仍只使用所选分组"
                % (recording["title"], lint_error)
            )

        rosbag = shutil.which("rosbag")
        if rosbag is None:
            raise RuntimeError("找不到 rosbag 命令，请检查 ROS 环境是否已 source")

        # 所有非破坏性校验通过后，才允许创建目录和清理历史 bag。
        bag_dir.mkdir(parents=True, exist_ok=True)
        cleanup_bag_directory(bag_dir)

        output = timestamped_bag_path(bag_dir)
        argv = [rosbag, "record", "-O", str(output)] + topics
        print(
            "[rosbag:%s] 本次录制 %d 个 topic：%s"
            % (recording["title"], len(topics), " ".join(topics))
        )
        print("[rosbag:%s] 输出文件：%s" % (recording["title"], output))
        sys.stdout.flush()
        sys.stderr.flush()
        os.execv(rosbag, argv)
    except Exception as exc:
        print("[rosbag] 启动失败：%s" % exc, file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
