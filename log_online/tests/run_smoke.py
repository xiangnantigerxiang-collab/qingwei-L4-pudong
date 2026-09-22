# -*- coding: utf-8 -*-
"""全栈冒烟:0-400s,t=100 触发 10s,t=200 再触发 5s(落在上一事件 post 窗内=窗口扩展),
期望仅 1 个事件目录、2 个触发段、末条 >= 最后清除+90s。"""
import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import log_online_config as cfg
import mock_ros

ros = mock_ros.install()
from robot.msg import can_msg as can_mod, navigation_msg as nav_mod
import recorder


def main():
    tmp = tempfile.TemporaryDirectory()
    data_dir = Path(tmp.name) / "events"
    rec = recorder.Recorder(ros, cfg, data_dir)
    rec.wire()
    t = 0.0
    while t < 400.0 - 1e-9:
        es = 1 if (100.0 <= t < 110.0 or 200.0 <= t < 205.0) else 0
        rec._on_can(can_mod.can_msg(vehicleSpeed=1.0, curGear=4, emergencyStop=es))
        rec._on_navigation(nav_mod.navigation_msg(xAxis=1.0, yAxis=2.0))
        rec.tick(t)
        t += 0.1
    events = sorted(d for d in data_dir.iterdir() if d.is_dir())
    assert len(events) == 1, "期望 1 个事件(第二次触发应扩展窗口),实际 %d" % len(events)
    meta = json.loads((events[0] / "metadata.json").read_text("utf-8"))
    assert len(meta["trigger_segments"]) == 2, meta["trigger_segments"]
    assert meta["complete"] is True
    last = float(open(events[0] / "records.jsonl").readlines()[-1].split('"ts":')[1]
                 .split(",")[0].strip())
    final_clear = max(s["end"] for s in meta["trigger_segments"])
    assert last >= final_clear + 90.0 - 1e-6, (last, final_clear)
    print("SMOKE PASS: 1 event, 2 segments, post window %.1fs >= 90s" % (last - final_clear))
    tmp.cleanup()


if __name__ == "__main__":
    main()
