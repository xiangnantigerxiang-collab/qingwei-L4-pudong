# -*- coding: utf-8 -*-
"""gantry_detect 捆绑启动配置校验(纯单元,无 ROS/无服务进程)。

校验 hmi_config.py 的 centerpoint("3D 感知")组件:
  1. cmd 为 bash -c 捆绑:先起 centerpoint_ros_node(保持原 cwd/环境语义),
     再 source devel/setup.bash 并 exec roslaunch gantry_detect;
  2. $ROOT 占位符经 load_config 替换后不再残留;
  3. health 同时监控 /box 与 /gantry_state(阈值 2Hz,单进程死亡互不掩盖);
  4. stop_pat 同时覆盖两个进程(gantry_detect_node 防 roslaunch 退出残留)。

用法:cd hmi && python3 tests/test_centerpoint_bundle.py
"""

import re
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
HMI_DIR = HERE.parent

import sys
sys.path.insert(0, str(HMI_DIR))

import hmi_server  # noqa: E402


def _centerpoint():
    cfg, _ = hmi_server.load_config(HMI_DIR / "hmi_config.py")
    for c in cfg["components"]:
        if c["name"] == "centerpoint":
            return c
    return None


class TestCenterpointBundle(unittest.TestCase):
    def setUp(self):
        self.c = _centerpoint()
        self.assertIsNotNone(self.c, "centerpoint 组件缺失")

    def test_cmd_bundles_gantry(self):
        cmd = self.c["cmd"]
        self.assertEqual(cmd[0], "bash")
        self.assertEqual(cmd[1], "-c")
        s = cmd[2]
        self.assertIn("./centerpoint_ros_node", s)
        # centerpoint 先于 source 启动(环境与单独运行一致)
        self.assertLess(s.index("./centerpoint_ros_node"),
                        s.index("source"))
        self.assertIn("devel/setup.bash", s)
        self.assertIn("roslaunch gantry_detect gantry_detect.launch", s)
        # $ROOT 必须已被 load_config 替换(否则 bash 在 build/ 下找不到路径)
        self.assertNotIn("$ROOT", s)
        self.assertIn(str(HMI_DIR.parent), s)

    def test_health_monitors_both(self):
        topics = {h["topic"]: h.get("min_hz")
                  for h in self.c["health"]}
        self.assertEqual(topics.get("/box"), 2)
        self.assertEqual(topics.get("/gantry_state"), 2)

    def test_stop_pat_covers_both_processes(self):
        pat = self.c["stop_pat"]
        for frag in ("centerpoint_ros_node", "gantry_detect_node",
                     "gantry_detect\\.launch"):
            self.assertIn(frag, pat)
        # 作为整体必须是合法正则(pkill -f 直接使用)
        re.compile(pat)


if __name__ == "__main__":
    unittest.main()
