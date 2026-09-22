# -*- coding: utf-8 -*-
"""配置↔代码同步守卫(纯单元,无 ROS/无服务进程,2026-09-21 引入)。

守护"自定义指令"卡片(ultra_command)落地后三类只能靠人工纪律维持的同步点:
  1. hmi_config.py 全部 nodes 型健康 pattern 与 ros_bridge._node_patterns()
     集合相等——漏同步时 check() 计数恒 0,组件 40s DEGRADED 并中止一键启动,
     且无任何报错日志指向根因(本轮对抗审查确认的漂移点);
  2. 组3 成员与顺序;ultra_command 卡无 optional/enabled 键即参与一键启动;
     stop_pat 同时覆盖 launch 文件名与节点二进制名(防 roslaunch 被 SIGKILL/OOM
     后节点孤儿对残留检测/外部识别/重启清理不可见);
  3. pnc 卡标题节点数 == launch/control.launch 递归展开的 <node> 标签实数
     (旧"6节点"在捆绑 ultra 时期实际拉起 7 节点,差一错误曾随标题改动传播)。

用法:cd hmi && python3 tests/test_config_sync.py
"""

import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
HMI_DIR = HERE.parent
ROOT = HMI_DIR.parent

import sys
sys.path.insert(0, str(HMI_DIR))

import hmi_server   # noqa: E402
import ros_bridge   # noqa: E402

INCLUDE_RE = re.compile(r"^\$\(find ([^)]+)\)/(.+)$")


def _load():
    cfg, _ = hmi_server.load_config(HMI_DIR / "hmi_config.py")
    return cfg


def _pkg_dir(pkg):
    """按 package.xml 的 <name> 定位 catkin 包目录(src/ 下唯一)。"""
    hits = [d for d in (ROOT / "src").iterdir()
            if d.is_dir() and (d / "package.xml").is_file()
            and ("<name>%s</name>" % pkg) in (d / "package.xml").read_text()]
    if len(hits) != 1:
        raise AssertionError("包 %s 定位到 %d 个目录,期望唯一" % (pkg, len(hits)))
    return hits[0]


def _expand(path):
    """展开 $(find pkg)/... ;其余原样返回。"""
    m = INCLUDE_RE.match(str(path))
    return _pkg_dir(m.group(1)) / m.group(2) if m else Path(path)


def _count_nodes(launch_path, seen=None):
    """递归统计 launch 及其 include 展开后的 <node> 标签数(含嵌套,防环)。"""
    seen = seen if seen is not None else set()
    p = Path(launch_path)
    if p.resolve() in seen:
        return 0
    seen.add(p.resolve())
    root = ET.parse(p).getroot()   # ElementTree 默认丢弃注释,不误计注释节点
    n = len(list(root.iter("node")))
    for inc in root.iter("include"):
        n += _count_nodes(_expand(inc.get("file")), seen)
    return n


class TestConfigSync(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.cfg = _load()
        cls.comps = {c["name"]: c for c in cls.cfg["components"]}

    def test_nodes_patterns_sync(self):
        """配置 nodes 型 pattern 集合必须与 ros_bridge._node_patterns() 相等。"""
        in_cfg = {h["pattern"] for c in self.cfg["components"]
                  for h in (c.get("health") or [])
                  if h.get("type") == "nodes"}
        in_code = set(ros_bridge.RosBridge([], )._node_patterns())
        self.assertEqual(in_cfg, in_code,
                         "hmi_config nodes 型 pattern 与 _node_patterns() 失同步:"
                         "漏同步的组件会 40s DEGRADED 且中止一键启动,无报错日志")

    def test_group3_membership_and_order(self):
        g3 = [c["name"] for c in self.cfg["components"] if c["group"] == 3]
        self.assertEqual(g3, ["lidar_perception", "pnc",
                              "ultra_command", "pnc_bags"])

    def test_ultra_card_spec(self):
        c = self.comps["ultra_command"]
        # 无 optional/enabled 键 → 参与一键启动、失败中止组4(用户明确要求)
        self.assertNotIn("optional", c)
        self.assertNotIn("enabled", c)
        self.assertEqual(c["health"],
                         [{"type": "nodes", "pattern": "/ultra_command_node",
                           "min": 1}])
        pat = c["stop_pat"]
        re.compile(pat)                       # 整体必须是合法正则
        self.assertIn("ultra_command\\.launch", pat)
        self.assertIn("ultra_command_nod[e]", pat)

    def test_pnc_title_matches_real_node_count(self):
        m = re.search(r"规划控制\((\d+)节点\)", self.comps["pnc"]["title"])
        self.assertIsNotNone(m, "pnc 标题不含节点数")
        real = _count_nodes(ROOT / "launch" / "control.launch")
        self.assertEqual(int(m.group(1)), real,
                         "标题 %s 节点 != launch 实际 %d 个 <node> 标签"
                         % (m.group(1), real))


if __name__ == "__main__":
    unittest.main()
