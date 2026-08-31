# -*- coding: utf-8 -*-
"""
monitor 边界值测试(开发机用)——直接驱动 ros_visualizer 纯函数与降级路径,
无需起服务/伪 master。用法: cd monitor && python3 tests/edge_test.py

覆盖:heading 环绕/None、scan 空与极端 ranges、PC2 坏布局与 NaN/Inf(曾抓到
Inf 漏过滤会 OverflowError 杀死解析线程的真实 bug)、voxelize 确定性、
地图缺失/坏行免疫、pack 大时间戳、空帧。
"""

import math
import os
import struct
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
MON = os.path.dirname(HERE)
sys.path.insert(0, MON)
sys.path.insert(0, HERE)

import mock_ros          # noqa: E402
import ros_visualizer as rv  # noqa: E402

PASS = 0
FAILS = []


def chk(name, cond, detail=""):
    global PASS
    if cond:
        PASS += 1
        print("  ok", name)
    else:
        FAILS.append(name)
        print("  FAIL", name, detail)


def main():
    # ---- yaw 环绕 ----
    y = rv.yaw_from_heading(274.59 + 720)
    chk("heading+720 环绕", abs(y - math.radians(90 - 274.59)) < 1e-9)
    chk("heading=None -> yaw(90deg)",
        abs(rv.yaw_from_heading(None) - math.radians(90)) < 1e-9)

    # ---- scan 边界 ----
    m = mock_ros.build_msg("/back_left_scan", {})
    m.ranges = []
    chk("空 ranges -> 0 点",
        len(rv.scan_to_points(m, {"x": 0, "y": 0, "yaw_deg": 0})) == 0)
    m.ranges = [float("nan")] * 8
    chk("全 NaN -> 0 点", len(rv.scan_to_points(m, {})) == 0)
    m.ranges = [1e9] * 8
    chk("全超界 -> 0 点", len(rv.scan_to_points(m, {})) == 0)
    m.ranges = [-2.0] * 8
    chk("负 range 全滤", len(rv.scan_to_points(m, {})) == 0)
    m.ranges = [2.0] * 8
    m.intensities = [1e9] * 8
    chk("intensity 1e9 -> clamp 255",
        all(q[3] == 255 for q in rv.scan_to_points(m, {})))

    class BadMsg(object):
        pass
    chk("坏消息对象 -> []", rv.scan_to_points(BadMsg(), {}) == [])

    # ---- PC2 解析边界 ----
    bad = BadMsg()
    bad.fields = []
    bad.point_step = 16
    bad.data = b"\x00" * 64
    chk("fields 缺 x/y/z -> []",
        rv.parse_cloud_points(bad, 1, -1.5, 3.0) == [])
    bad2 = BadMsg()
    bad2.fields = [mock_ros._ObjField("x", 0, 7),
                   mock_ros._ObjField("y", 4, 7),
                   mock_ros._ObjField("z", 8, 7)]
    bad2.point_step = 0
    bad2.data = b"\x00" * 16
    chk("point_step=0 -> []",
        rv.parse_cloud_points(bad2, 1, -1.5, 3.0) == [])
    bad3 = BadMsg()
    bad3.fields = [mock_ros._ObjField("x", 0, 7),
                   mock_ros._ObjField("y", 4, 7),    # offset+4 > step
                   mock_ros._ObjField("z", 8, 7)]
    bad3.point_step = 4
    bad3.data = b"\x00" * 16
    chk("offset 超 step 坏布局 -> []",
        rv.parse_cloud_points(bad3, 1, -1.5, 3.0) == [])

    # NaN/Inf 各轴(y 轴 Inf 曾漏过滤 -> pack OverflowError 杀死线程)
    nan_pts = mock_ros.build_msg("/rslidar_points_mid", {"__pc2__": [
        [float("nan"), 0, 0, 1], [0, float("inf"), 0, 1],
        [0, 0, float("nan"), 1], [float("inf"), 0, 0.5, 1],
        [1, 2, 0.5, 9]]})
    out = rv.parse_cloud_points(nan_pts, 1, -1.5, 3.0)
    chk("NaN/Inf 各轴全滤(只留有效 1 点)", len(out) == 1, str(out))

    big = mock_ros.build_msg("/rslidar_points_mid", {"__pc2__": [
        [1, 2, 0.5, 1e12], [3, 4, 0.6, 1e12]]})
    out = rv.parse_cloud_points(big, 1, -1.5, 3.0)
    chk("intensity 1e12 -> clamp 255", rv._clamp_u8(out[0][3]) == 255)
    chk("z 毫米钳位", rv._clamp16(1e6) == 32767 and rv._clamp16(-1e6) == -32768)

    # ---- voxelize ----
    pts = [(i * 0.001, 0, 0, 0) for i in range(100)]
    a = rv.voxelize(pts, 0.2, 1000)
    b = rv.voxelize(pts, 0.2, 1000)
    chk("voxelize 确定性+同格收敛", a == b and len(a) == 1)
    chk("voxel<=0 透传", rv.voxelize(pts[:5], 0, 10) == pts[:5])
    chk("空输入", rv.voxelize([], 0.2, 10) == [])

    # ---- 地图缺失/坏行 ----
    viz = rv.RosVisualizer({"MAP_PATH": "$MON/map/_nope_.csv",
                            "SCAN_EXTRINSICS": {}, "CLOUD": {}})
    mp = viz.map_payload()
    chk("地图缺失 -> 空 maps+bbox[0]",
        mp["maps"] == [] and mp["n"] == 0 and mp["bbox"] == [0, 0, 0, 0])
    sn = viz.snapshot()
    chk("无地图 origin=(0,0) 快照不崩",
        sn["origin"] == [0.0, 0.0] and sn["vehicle"] is None)

    with tempfile.NamedTemporaryFile("w", suffix=".csv",
                                     delete=False) as tf:
        tf.write("1,2,3\r\n\r\nbad,line\n4,5,6\n  7,8,9  \n,,\n")
        tp = tf.name
    lines = rv.load_map_lines(tp)
    chk("坏行免疫(CRLF/空行/短行/空白)",
        len(lines["center"]) == 3, str(len(lines["center"])))
    os.unlink(tp)

    # ---- 多地图目录加载(map/ 下全部 csv 依次绘制) ----
    with tempfile.TemporaryDirectory() as td:
        with open(os.path.join(td, "a_map.csv"), "w") as f:
            f.write("0,0,90\n10,0,90\n20,0,90\n")
        with open(os.path.join(td, "b_map.csv"), "w") as f:
            f.write("100,100,90\n110,100,90\n")
        with open(os.path.join(td, "broken.csv"), "w") as f:
            f.write("not,a,map\n\n1,2\n")   # 全坏行 -> 空文件跳过
        open(os.path.join(td, "readme.txt"), "w").write("ignore me")
        viz2 = rv.RosVisualizer({"MAP_PATH": td, "SCAN_EXTRINSICS": {},
                                 "CLOUD": {}})
        mp2 = viz2.map_payload()
        chk("多地图: 2 张有效(broken 空文件与 txt 跳过)",
            mp2["n"] == 2 and [x["name"] for x in mp2["maps"]] ==
            ["a_map.csv", "b_map.csv"], str(mp2["n"]))
        chk("多地图: 合并 bbox", mp2["bbox"] == [0.0, 0.0, 110.0, 100.0],
            str(mp2["bbox"]))
        ox2 = (0.0 + 110.0) / 2.0
        oy2 = (0.0 + 100.0) / 2.0
        chk("多地图: origin=合并中心", abs(viz2.snapshot()["origin"][0] - ox2) < 0.01
            and abs(viz2.snapshot()["origin"][1] - oy2) < 0.01,
            str(viz2.snapshot()["origin"]))
        chk("多地图: 坐标已减 origin",
            mp2["maps"][0]["center"][0] == [-ox2, -oy2],
            str(mp2["maps"][0]["center"][0]))
        chk("多地图: 第二张含两线",
            len(mp2["maps"][1]["center"]) == 2 and
            len(mp2["maps"][1]["left"]) == 2)

    # ---- pack 边界 ----
    blob = rv.pack_bin([(1, 2, rv._BIN_POINT.pack(1500, -2250, 500, 200, 0)
                         * 2)], 2 ** 53 + 123)
    chk("大 ts(2^53+) pack 不崩", len(blob) == 20 + 8 + 24)
    ver, nchan, total = struct.unpack_from("<HHI", blob, 4)
    chk("大 ts 头字段正确", ver == 1 and nchan == 1 and total == 2)
    empty = rv.pack_bin([], 0)
    chk("空帧 20B", len(empty) == 20 and empty[:4] == b"QWMC")

    print("\n==== %d passed, %d failed ====" % (PASS, len(FAILS)))
    for f in FAILS:
        print("  FAILED:", f)
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
