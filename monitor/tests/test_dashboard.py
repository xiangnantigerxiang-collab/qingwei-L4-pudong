# -*- coding: utf-8 -*-
"""
dashboard 仪表板测试(8082 子服务)——对标 test_full.py 风格
==========================================================

用法: cd monitor && python3 tests/test_dashboard.py

覆盖:
  d01 .msg 解析:63 字段/分节/中文名称与 ehb_msg.msg 注释逐条对齐
  d02 can_msg 27 字段全部有中文名称(CAN_LABELS)
  d03 枚举译码(二进制键/十六进制键/冒号空格形态/区间跳过/无表 None)
  d04 值格式化(bool/数组 hex/float/None)
  d05 payload(伪 viz:字段值/年龄/缺话题)
  d06 HTTP 端点(页面 200 含标题;API JSON;404)
  d07 全栈:子进程 run_server_mock + 伪 ROS 泵 /ehb_msg -> API 见值
  d08 py3.8 兼容(dashboard.py ast feature_version)
"""

import ast
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
import types
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
MON = os.path.dirname(HERE)
sys.path.insert(0, MON)

PORT = 18083
DASH = 18084
MASTER = 21111             # 避让本机 root 的真实 rosmaster(11311,09-05 起)
BASE = "http://127.0.0.1:%d" % DASH
CONTROL = "/tmp/monitor_dash_control.json"

PASS = 0
FAILS = []


def check(name, cond, detail=""):
    global PASS
    if cond:
        PASS += 1
        print("  ok %s" % name)
    else:
        FAILS.append(name)
        print("  FAIL %s  %s" % (name, detail))


def http_get(url, timeout=6):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.status, r.read()


class StubViz(object):
    """dashboard 取数接口的最小桩。"""

    def __init__(self, stash=None):
        self.stash = stash or {}

    def latest(self, topic):
        return self.stash.get(topic)

    def master_ok(self):
        return True

    @staticmethod
    def ros_available():
        return True


def d01_msg_parse():
    print("d01 .msg 解析与中文名称对齐")
    import dashboard
    spec = dashboard.parse_msg_spec(
        os.path.join(MON, "..", "src", "canbus", "msg", "ehb_msg.msg"))
    fields = [f for s in spec["sections"] for f in s["fields"]]
    check("字段数 63", len(fields) == 63, len(fields))
    by = {f["name"]: f for f in fields}
    # 中文名称与注释对齐抽查(覆盖五种注释形态)
    want = {
        "EHB_STO_StorageSystemStatus": "EHB蓄能系统工作状态",
        "EHB_STO_PressureSensor1RawValue": "液压传感器1原始压力值",
        "EHB_STO_FAU_MotorWorkTimeout": "蓄能电机加压超时",
        "EHB_ATB_FAU_BrakePressSensorError": "制动液压信号异常",
        "EHB_ATB_FAU_MotorCommuteFail": "电机校准失败",
        "EHB_EPB_ParkingStatus": "驻车系统驻车状态",
        "VHL_ATB_BrakePressRequestFlag": "制动请求标志",
        "VHL_ATB_CheckSum": "校验和",
        "VHL_VehicleSpeed": "车辆车速",
        "VHL_HvPowerState": "高压上电状态",
        "VHL_VehicleGear": "车辆当前档位",
    }
    for name, label in want.items():
        got = by[name]["label"]
        check("名称 %s" % name, got == label, "got=%r" % got)
    check("全部 63 字段均有中文名称",
          all(f["label"] for f in fields),
          [f["name"] for f in fields if not f["label"]])
    titles = [s["title"] for s in spec["sections"] if s["title"]]
    check("分节标题含帧1/接收方向",
          any("帧1" in t for t in titles)
          and any("接收方向帧4" in t for t in titles), titles)
    check("注释保留枚举原文",
          "00:加压关" in by["EHB_STO_StorageSystemStatus"]["note"])


def d02_can_labels():
    print("d02 can_msg 中文名称覆盖")
    import dashboard
    spec = dashboard.parse_msg_spec(
        os.path.join(MON, "..", "src", "canbus", "msg", "can_msg.msg"))
    dashboard._apply_can_labels(spec)
    fields = [f for s in spec["sections"] for f in s["fields"]]
    check("can_msg 字段数 27", len(fields) == 27, len(fields))
    miss = [f["name"] for f in fields if not f["label"]]
    check("27 字段全有中文名称", not miss, miss)
    by = {f["name"]: f for f in fields}
    check("hookStatus 名称", by["hookStatus"]["label"] == "钩销状态机",
          by["hookStatus"]["label"])
    check("rawcommand note 含 hex 说明",
          "hex" in by["rawcommand"]["note"], by["rawcommand"]["note"])


def d03_enum_decode():
    print("d03 枚举译码")
    import dashboard
    D = dashboard._decode_enum
    note2 = "1.1-1.2 蓄能系统工作状态 00:加压关 01:加压开 10:错误 11:无效"
    check("二进制键 1->加压开", D(1, note2) == "加压开")
    check("二进制键 3->无效", D(3, note2) == "无效")
    check("二进制键越界 None", D(4, note2) is None)
    # 锁定"二进制先于十进制"分支顺序:若十进制前置,10/11 会被译成
    # "错误"/"无效"(int('10')==10 不成立,但 '10' 等宽二进制键存在时
    # 顺序错误会让 2bit 域外的垃圾值错译出权威中文)
    check("顺序锁:值10(超2bit域)->None", D(10, note2) is None)
    check("顺序锁:值11(超2bit域)->None", D(11, note2) is None)
    noteh = "1.3-1.5 驻车系统驻车状态 0x0:已夹紧 0x1:已释放 0x2:  夹紧中 0x3:释放中"
    check("hex 键+冒号空格 2->夹紧中", D(2, noteh) == "夹紧中")
    notex = "2.1-2.2 主动制动系统当前状态 0x0: 正常 0x1: 抑制 0x2: 故障"
    check("hex 键冒号后空格 1->抑制", D(1, notex) == "抑制")
    noteflag = ("2.1-2.2 制动请求标志 00:无请求 01:有请求 10:未定义 "
                "11:无效(AC 列 0x11 越界系原表冲突 I03)")
    check("括号截断 3->无效", D(3, noteflag) == "无效")
    check("区间写法跳过 4->None",
          D(4, noteh + " 0x4~0x6:未知") is None)
    noted = "1.1-1.2 驻车请求 0:无请求 1:请求驻车 2:请求释放 3:无效(I03 同上)"
    check("十进制键 0->无请求", D(0, noted) == "无请求")
    check("十进制键 2->请求释放", D(2, noted) == "请求释放")
    check("十进制键 3->无效", D(3, noted) == "无效")
    notehv = ("3.1-3.8 高压上电状态(原表) 0x00:高压断电 "
              "0x01:高压上电 0xFF:忽略;64768-03")
    check("译码文本截断参数编号 0xFF->忽略", D(0xFF, notehv) == "忽略")
    check("hex 0x01->高压上电", D(1, notehv) == "高压上电")
    # 真文件锁定:VHL_EPB_ParkingRequest 全值域译码
    spec = dashboard.parse_msg_spec(
        os.path.join(MON, "..", "src", "canbus", "msg", "ehb_msg.msg"))
    note = [f for s in spec["sections"] for f in s["fields"]
            if f["name"] == "VHL_EPB_ParkingRequest"][0]["note"]
    check("真文件十进制键全值域",
          [D(v, note) for v in range(4)] ==
          ["无请求", "请求驻车", "请求释放", "无效"])
    check("无枚举表 None", D(5, "7.5 电源电压过高") is None)
    check("空注释 None", D(0, "") is None)


def d04_fmt_value():
    print("d04 值格式化")
    import dashboard
    F = dashboard._fmt_value
    check("bool True->1", F(True) == 1)
    check("None->None", F(None) is None)
    check("数组->hex", F([1, 0xA2, 255]) == "01 A2 FF")
    check("bytes->hex(rospy uint8[] 真实类型)",
          F(b"\x18\x55\x01") == "18 55 01")
    check("bytearray->hex", F(bytearray([0xA2, 0xB0])) == "A2 B0")
    check("memoryview->hex", F(memoryview(b"\x0c\x02")) == "0C 02")
    check("bytes 不出 repr", "b'" not in str(F(b"\x18\x55")))
    check("float 舍入", F(1.23456) == 1.235)
    check("NaN->0", F(float("nan")) == 0.0)
    check("int 原样", F(0x2D) == 45)


def d05_payload():
    print("d05 payload 组装(瘦身契约:隐藏/折叠/精简/译码)")
    import dashboard
    spec0 = dashboard.parse_msg_spec(
        os.path.join(MON, "..", "src", "canbus", "msg", "ehb_msg.msg"))
    # sections[0] 是头部空节(payload 会跳过),按非空节取帧1/帧2
    secs0 = [s for s in spec0["sections"] if s["fields"]]
    fau1 = [f["name"] for f in secs0[0]["fields"]
            if "_FAU_" in f["name"]]
    fau2 = [f["name"] for f in secs0[1]["fields"]
            if "_FAU_" in f["name"]]
    # 帧1 全 10 位齐备(1 真 9 假);帧2/帧3 位属性缺失(三态判定用例)
    msg = types.SimpleNamespace(
        EHB_STO_StorageSystemStatus=1,
        EHB_STO_PressureSensor1RawValue=0x7B,
        VHL_VehicleSpeed=600,
        VHL_VehicleGear=0x44,
        vehicleSpeed=1.25,
        **dict((n, n == "EHB_STO_FAU_Sensor1Error") for n in fau1))
    viz = StubViz({"/ehb_msg": (msg, time.monotonic() - 0.4)})
    app = dashboard.DashboardApp(viz)
    d = app.payload()
    check("ok/ros/master", d["ok"] and d["ros_available"] and d["master_ok"])
    ehb = d["topics"]["/ehb_msg"]
    check("ehb age≈0.4", 0.3 <= ehb["age"] <= 1.0, ehb["age"])
    can = d["topics"]["/can_msg"]
    check("can 无数据 age=None", can["age"] is None)

    # ---- ehb:分节/行数/标题精简 ----
    secs = ehb["sections"]
    titles = [s["title"] for s in secs]
    check("分节标题精简(去 0x/位号/括注)",
          titles == ["帧1 蓄能系统", "帧2 主动制动", "帧3 电子驻车",
                     "帧4 制动请求", "帧5 驻车请求", "补充项"], titles)
    counts = [len(s["fields"]) for s in secs]
    check("分节行数 11/4/5/2/1/4(FAU 折叠+RC/CS 隐藏)",
          counts == [11, 4, 5, 2, 1, 4], counts)
    rows = [f for s in secs for f in s["fields"]]
    names = [f["name"] for f in rows]
    check("ehb 总行数 27", len(rows) == 27, len(rows))
    check("RC/CS 四字段不在 payload",
          not ({"VHL_ATB_RollingCounter", "VHL_ATB_CheckSum",
                "VHL_EPB_RollingCounter", "VHL_EPB_CheckSum"}
               & set(names)))
    check("FAU bool 不再逐行", not any("_FAU_" in n for n in names))

    # ---- 故障折叠 ----
    s1 = secs[0]["fields"][0]
    check("帧1 首行为故障汇总", s1["name"].startswith("__faults")
          and s1["label"] == "故障位", s1)
    check("汇总计数 1 并点名", s1["value"] == 1
          and s1["text"] == "1号液压传感器信号异常", s1)
    check("帧2 位不可读→无数据(不得假全部正常)",
          secs[1]["fields"][0]["value"] is None
          and secs[1]["fields"][0]["text"] is None,
          secs[1]["fields"][0])
    check("帧3 位不可读→无数据", secs[2]["fields"][0]["value"] is None)
    app_f = dashboard.DashboardApp(StubViz({"/ehb_msg": (
        types.SimpleNamespace(**dict((n, False) for n in fau2)),
        time.monotonic())}))
    s2 = app_f.payload()["topics"]["/ehb_msg"]["sections"]
    check("帧2 全位可读且全 False→全部正常",
          s2[1]["fields"][0]["value"] == 0
          and s2[1]["fields"][0]["text"] == "全部正常")
    check("帧4 无汇总行(本节无 FAU)",
          not secs[3]["fields"][0]["name"].startswith("__faults"))

    # ---- 译码不受注释精简影响(顺序:先译码后裁剪) ----
    by = {f["name"]: f for f in rows}
    row = by["EHB_STO_StorageSystemStatus"]
    check("值 1+译码 加压开", row["value"] == 1 and row["text"] == "加压开")
    check("缺属性字段 None", by["EHB_STO_PressureSensor2RawValue"]["value"] is None)

    # ---- ehb 注释只留换算 ----
    check("压力 ×0.1 MPa",
          by["EHB_STO_PressureSensor1RawValue"]["note"] == "×0.1 MPa",
          by["EHB_STO_PressureSensor1RawValue"]["note"])
    check("请求压力 ×0.04 MPa",
          by["VHL_ATB_BrakePressRequest"]["note"] == "×0.04 MPa",
          by["VHL_ATB_BrakePressRequest"]["note"])
    check("车速 ×0.1 km/h",
          by["VHL_VehicleSpeed"]["note"] == "×0.1 km/h",
          by["VHL_VehicleSpeed"]["note"])
    check("踏板 0-100%",
          by["VHL_BrakePedalStatus"]["note"] == "0-100%",
          by["VHL_BrakePedalStatus"]["note"])
    check("无换算注释清空", by["EHB_STO_FailureNum"]["note"] == "",
          by["EHB_STO_FailureNum"]["note"])

    # ---- 档位 uint16 字符译码 ----
    check("档位 0x44->D", by["VHL_VehicleGear"]["text"] == "D",
          by["VHL_VehicleGear"]["text"])
    app_g = dashboard.DashboardApp(StubViz({"/ehb_msg": (
        types.SimpleNamespace(VHL_VehicleGear=0x5244), time.monotonic())}))
    rg = {f["name"]: f
          for s in app_g.payload()["topics"]["/ehb_msg"]["sections"]
          for f in s["fields"]}
    check("档位歧义(两字母并存)不译", rg["VHL_VehicleGear"]["text"] is None,
          rg["VHL_VehicleGear"]["text"])

    # ---- can:隐藏 5 字段/单位保留/枚举译码 ----
    canrows = [f for s in can["sections"] for f in s["fields"]]
    check("can 22 行值全 None(隐藏 rawcommand/rawfeedback/epsERR1/2/faultCode)",
          len(canrows) == 22 and all(f["value"] is None for f in canrows),
          len(canrows))
    check("can 隐藏字段不在 payload",
          not ({"rawcommand", "rawfeedback", "epsERR1", "epsERR2", "faultCode"}
               & {f["name"] for f in canrows}))
    check("can 无故障汇总行",
          not any(f["name"].startswith("__faults") for f in canrows))
    cby = {f["name"]: f for f in canrows}
    check("can 单位保留(m/s / % / A)",
          cby["vehicleSpeed"]["note"] == "m/s"
          and cby["batteryPower"]["note"] == "%"
          and cby["epsCurrent"]["note"] == "A")
    check("can 枚举括注不当单位(含=丢弃)",
          cby["curGear"]["note"] == "" and cby["hookStatus"]["note"] == "")

    canmsg = types.SimpleNamespace(curGear=4, emergencyStop=1, hookStatus=3)
    app2 = dashboard.DashboardApp(
        StubViz({"/can_msg": (canmsg, time.monotonic() - 0.2)}))
    rows2 = {f["name"]: f
             for s in app2.payload()["topics"]["/can_msg"]["sections"]
             for f in s["fields"]}
    check("curGear 4->D", rows2["curGear"]["text"] == "D")
    check("emergencyStop 1->急停", rows2["emergencyStop"]["text"] == "急停")
    check("hookStatus 3->底", rows2["hookStatus"]["text"] == "底")
    app3 = dashboard.DashboardApp(
        StubViz({"/can_msg": (types.SimpleNamespace(curGear=9),
                              time.monotonic())}))
    rows3 = {f["name"]: f
             for s in app3.payload()["topics"]["/can_msg"]["sections"]
             for f in s["fields"]}
    check("can 枚举外值(9)不译", rows3["curGear"]["text"] is None)

    # ---- 无数据:汇总不得假'全部正常' ----
    app4 = dashboard.DashboardApp(StubViz())
    e4 = app4.payload()["topics"]["/ehb_msg"]["sections"]
    f4 = e4[0]["fields"][0]
    check("无数据:汇总 value/text 均 None", f4["value"] is None
          and f4["text"] is None, f4)


def d06_http():
    print("d06 HTTP 端点(进程内)")
    import dashboard
    from http.server import ThreadingHTTPServer
    app = dashboard.DashboardApp(StubViz())
    httpd = ThreadingHTTPServer(("127.0.0.1", 0),
                                dashboard.make_dashboard_handler(app))
    port = httpd.server_address[1]
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    try:
        base = "http://127.0.0.1:%d" % port
        st, body = http_get(base + "/")
        check("页面 200", st == 200)
        check("页面含 仪表板", "仪表板".encode("utf-8") in body)
        st, body = http_get(base + "/api/dashboard")
        d = json.loads(body.decode("utf-8"))
        check("API ok", d["ok"] is True)
        check("API 话题键", set(d["topics"]) == {"/can_msg", "/ehb_msg"})
        try:
            http_get(base + "/nope")
            check("404", False)
        except urllib.error.HTTPError as e:
            check("404", e.code == 404)
    finally:
        httpd.shutdown()
        httpd.server_close()


def d07_fullstack():
    print("d07 全栈(mock ROS 子进程)")
    with open(CONTROL, "w") as f:
        json.dump({"master_alive": True,
                   "rates": {"/ehb_msg": 10, "/can_msg": 10},
                   "fields": {}}, f)
    env = dict(os.environ)
    env["MOCK_CONTROL"] = CONTROL
    env["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % MASTER
    env["DASHBOARD_PORT"] = str(DASH)
    srv = None
    master = None
    try:
        sys.path.insert(0, HERE)
        import mock_ros
        master = mock_ros.FakeMaster(port=MASTER)
        master.start()
        srv = subprocess.Popen(
            [sys.executable, os.path.join(HERE, "run_server_mock.py"),
             "--port", str(PORT), "--config",
             os.path.join(HERE, "ros_test_config.py")],
            cwd=HERE, env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        d = None
        for _ in range(40):
            time.sleep(0.5)
            try:
                st, body = http_get(BASE + "/api/dashboard", timeout=3)
                d = json.loads(body.decode("utf-8"))
                ehb = d["topics"]["/ehb_msg"]
                if ehb["age"] is not None:
                    break
            except Exception:
                pass
        check("服务起来且收到 /ehb_msg", d is not None
              and d["topics"]["/ehb_msg"]["age"] is not None)
        if d:
            check("ros_available", d["ros_available"] is True)
            check("master_ok", d["master_ok"] is True)
            esecs = d["topics"]["/ehb_msg"]["sections"]
            rows = {f["name"]: f
                    for s in esecs for f in s["fields"]}
            check("STO 状态值 1+译码",
                  rows["EHB_STO_StorageSystemStatus"]["value"] == 1
                  and rows["EHB_STO_StorageSystemStatus"]["text"] == "加压开")
            check("bool 折叠:FAU 行不在,汇总点名",
                  "EHB_STO_FAU_Sensor1Error" not in rows
                  and esecs[0]["fields"][0]["value"] == 1
                  and "1号液压传感器信号异常" in esecs[0]["fields"][0]["text"])
            check("车速 600+换算注释",
                  rows["VHL_VehicleSpeed"]["value"] == 600
                  and rows["VHL_VehicleSpeed"]["note"] == "×0.1 km/h")
            check("档位 0x44->D", rows["VHL_VehicleGear"]["text"] == "D")
            crows = {f["name"]: f
                     for s in d["topics"]["/can_msg"]["sections"]
                     for f in s["fields"]}
            check("can 译码:挡位 D/急停 正常",
                  crows["curGear"]["text"] == "D"
                  and crows["emergencyStop"]["text"] == "正常")
            check("can_msg 也活",
                  d["topics"]["/can_msg"]["age"] is not None)
            st, body = http_get(BASE + "/")
            check("dashboard 页面 200", st == 200
                  and "dashboard".encode("utf-8") in body)
    finally:
        if srv:
            srv.terminate()
            srv.wait(timeout=5)
        if master:
            master.stop()


def d08_py38():
    print("d08 py3.8 兼容")
    src = os.path.join(MON, "dashboard.py")
    with open(src, "r", encoding="utf-8") as f:
        tree = ast.parse(f.read(), feature_version=(3, 8))
    check("ast 3.8 可解析", isinstance(tree, ast.Module))


def d09_robustness():
    print("d09 .msg 健壮性(GBK 兜底/缺文件降级)")
    import dashboard
    tmp = tempfile.mkdtemp()
    with open(os.path.join(tmp, "ehb_msg.msg"), "wb") as f:
        f.write("uint8 X # 1.1-1.2 测试字段 00:关 01:开\n".encode("gbk"))
    with open(os.path.join(tmp, "can_msg.msg"), "w", encoding="utf-8") as f:
        f.write("uint8 a\n")
    app = dashboard.DashboardApp(StubViz(), msg_dir=tmp)
    check("GBK msg 可解析(兜底)", not app._load_errors)
    rows = [f for s in app._specs["/ehb_msg"]["sections"]
            for f in s["fields"]]
    check("GBK 中文名照常", rows and rows[0]["label"] == "测试字段",
          rows and rows[0]["label"])

    app2 = dashboard.DashboardApp(StubViz(),
                                  msg_dir=os.path.join(tmp, "nope"))
    check("缺目录降级不炸", len(app2._load_errors) == 2)
    d = app2.payload()
    check("降级后 payload 仍 ok", d["ok"] is True
          and d["topics"]["/ehb_msg"]["sections"] is None)


def d12_fmt_robust():
    print("d12 单字段格式化失败降级(float[] 等不炸端点)")
    import dashboard
    tmp = tempfile.mkdtemp()
    with open(os.path.join(tmp, "ehb_msg.msg"), "w", encoding="utf-8") as f:
        f.write("uint8 X # 1.1-1.2 测试字段 00:关 01:开\n"
                "float32[] arr # 2.1-2.8 数组字段\n")
    with open(os.path.join(tmp, "can_msg.msg"), "w", encoding="utf-8") as f:
        f.write("uint8 a # x\n")
    app = dashboard.DashboardApp(
        StubViz({"/ehb_msg": (types.SimpleNamespace(X=1, arr=[1.5, 2.5]),
                              time.monotonic())}), msg_dir=tmp)
    d = app.payload()
    check("合成 spec 可载", not app._load_errors, app._load_errors)
    rows = {f["name"]: f
            for s in d["topics"]["/ehb_msg"]["sections"]
            for f in s["fields"]}
    check("float[] 坏字段降级 str,端点不炸",
          d["ok"] is True and isinstance(rows["arr"]["value"], str),
          rows["arr"]["value"])
    check("邻字段不受累", rows["X"]["value"] == 1 and rows["X"]["text"] == "开")


def _spawn_monitor(mon_port, dash_env, cfg_path=None, mport=None):
    """起 run_server_mock 子进程(stderr 用 PIPE 供告警断言),返回 (proc, master)。"""
    sys.path.insert(0, HERE)
    import mock_ros
    master = mock_ros.FakeMaster(port=mport)
    master.start()
    with open(CONTROL, "w") as f:
        json.dump({"master_alive": True, "rates": {}, "fields": {}}, f)
    env = dict(os.environ)
    env["MOCK_CONTROL"] = CONTROL
    env["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % mport
    env["DASHBOARD_PORT"] = dash_env
    proc = subprocess.Popen(
        [sys.executable, os.path.join(HERE, "run_server_mock.py"),
         "--port", str(mon_port), "--config",
         cfg_path or os.path.join(HERE, "ros_test_config.py")],
        cwd=HERE, env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    return proc, master


def _wait_main_up(mon_port, rounds=24):
    for _ in range(rounds):
        time.sleep(0.5)
        try:
            st, _ = http_get("http://127.0.0.1:%d/api/snapshot" % mon_port,
                             timeout=3)
            if st == 200:
                return True
        except Exception:
            pass
    return False


def _stop(proc, master):
    proc.terminate()
    proc.wait(timeout=5)
    master.stop()
    err = ""
    if proc.stderr:
        try:
            err = proc.stderr.read().decode("utf-8", "replace")
        except Exception:
            pass
    return err


def d10_port_robustness():
    print("d10 DASHBOARD_PORT 语义(空串回退配置/非法告警/主服务存活)")
    # (a) 环境变量空串 -> 回退配置值,不遮蔽、不算非法
    cfgdash = 18095
    with open(os.path.join(HERE, "ros_test_config.py"),
              encoding="utf-8") as f:
        src = f.read()
    tmpdir = tempfile.mkdtemp()
    tmpcfg = os.path.join(tmpdir, "cfg_dash.py")
    with open(tmpcfg, "w", encoding="utf-8") as f:
        f.write(src + "\nCONFIG['DASHBOARD_PORT'] = %d\n" % cfgdash)
    proc, master = _spawn_monitor(PORT + 1, "", cfg_path=tmpcfg,
                                  mport=MASTER + 10)
    dash_up = False
    try:
        if _wait_main_up(PORT + 1):
            for _ in range(10):
                time.sleep(0.5)
                try:
                    st, _ = http_get("http://127.0.0.1:%d/api/dashboard"
                                     % cfgdash, timeout=3)
                    dash_up = (st == 200)
                    if dash_up:
                        break
                except Exception:
                    pass
    finally:
        err = _stop(proc, master)
    check("空串回退配置:dashboard 按配置口起", dash_up, err[:120])
    check("空串不算非法(无告警)", "DASHBOARD_PORT 非法" not in err, err[:120])

    # (b) 非法值:stderr 告警 + 关闭 + 主服务存活
    for i, bad in enumerate(("abc", "70000")):
        proc, master = _spawn_monitor(PORT + 2, bad, mport=MASTER + 11 + i)
        try:
            main_up = _wait_main_up(PORT + 2)
        finally:
            err = _stop(proc, master)
        check("DASHBOARD_PORT=%r 主服务仍起" % bad, main_up)
        check("DASHBOARD_PORT=%r 有 stderr 告警" % bad,
              "DASHBOARD_PORT 非法" in err, err[:120])


def d11_occupied_guard():
    print("d11 dashboard 端口被占:启动守卫生效(主服务活+告警)")
    import socket
    s = socket.socket()
    # SO_REUSEADDR 只为越过 d07 遗留连接的 TIME_WAIT;活跃 LISTEN 仍会
    # 让子进程 ThreadingHTTPServer 绑定失败(守卫正是为此而设)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", DASH))
    s.listen(1)
    try:
        proc, master = _spawn_monitor(PORT + 3, str(DASH), mport=MASTER + 20)
        try:
            main_up = _wait_main_up(PORT + 3)
        finally:
            err = _stop(proc, master)
        check("端口被占:主服务仍起(守卫承重)", main_up)
        check("端口被占:stderr 告警", "dashboard 启动失败" in err, err[:160])
    finally:
        s.close()


def main():
    for fn in (d01_msg_parse, d02_can_labels, d03_enum_decode, d04_fmt_value,
               d05_payload, d06_http, d07_fullstack, d08_py38,
               d09_robustness, d12_fmt_robust,
               d10_port_robustness, d11_occupied_guard):
        try:
            fn()
        except Exception as exc:
            import traceback
            traceback.print_exc()
            FAILS.append(fn.__name__)
            print("  FAIL %s 异常: %s" % (fn.__name__, exc))
    print("\n%d 通过, %d 失败" % (PASS, len(FAILS)))
    if FAILS:
        for n in FAILS:
            print("  FAIL:", n)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
