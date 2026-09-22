# log_online 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 仿 monitor 形态新建 `log_online/`：10Hz 采集七类车辆数据入环形缓冲，急停/故障触发时落盘"前 ≥90s + 事件 + 后 ≥90s"事件目录，附 HTTP 状态页与磁盘轮转。

**Architecture:** 单进程双线程（主线程 rospy 回调+Timer tick；daemon 线程 ThreadingHTTPServer 状态页）。纯逻辑（快照构建/触发判定/状态机）与 IO（EventWriter/Rotator）分层，全部可注入 mock 在无 ROS 本机测试。

**Tech Stack:** Python 3.8+（车载 noetic），仅标准库（json/collections/http.server/threading/shutil/unittest），无第三方依赖。

**Spec:** `log_online/DESIGN.md`（本计划从 spec 出发，执行者两份都要读）

## Global Constraints

- 本工程**非 git**：无 commit 步骤；每任务以"该测试文件全部用例通过"收尾，最后统一快照+记 workflow.md。
- Python **3.8 兼容**（车载 Py3.8）：不用 `match`、不用 `X | Y` 类型注解、不用 3.9+ 语法；类型注解用 `typing.Optional` 等或干脆不加。
- 仅标准库；测试用 `unittest`，运行方式 `python3 tests/<file>.py`（文件内 `unittest.main()`）。
- 代码风格对齐 monitor：4 空格缩进、中文注释解释"为什么"、话题绝对名（`/` 前缀）。
- 消息字段以 spec §4 映射表为准（已对照 `.msg` 核实）：`curGear`（N=2/R=3/D=4）、`controlPanelState`、`vehicleSpeed`、`emergencyStop`、`faultCode`(uint8[])、navigation 的 `lat/lon/xAxis/yAxis/heading/rtkState/longitudinal_accelerate`、object 的 `id/type/x/y/dx/dy/heading/height/vx/vy/confidence`、plan 的 `desireSpeed/safety`、control 的 `throttlePercent/brakePercent/wheelAngle/desireAcc`。
- 数据目录默认 `log_online/data/events/`；状态页默认 8083，env `LOG_ONLINE_PORT` 覆盖；**严禁公网映射**（写入 README）。
- 不修改 monitor/、start_l4.sh、HMI（保护清单）。

---

### Task 1: 测试基础设施（config + mock_ros）

**Files:**
- Create: `log_online/log_online_config.py`
- Create: `log_online/tests/__init__.py`（空文件）
- Create: `log_online/tests/mock_ros.py`
- Test: `log_online/tests/test_infra.py`

**Interfaces:**
- Produces: `log_online_config` 全部常量（spec §11）；`mock_ros.install()` 向 `sys.modules` 注入 `rospy`、`robot.msg`、`sensor_msgs.msg` 假模块，供后续任务在 import recorder 前安装；假消息类 `CanMsg/NavigationMsg/ObjectMsg/PerceptionMsg/PlanPathMsg/ControlMsg/CompressedImage` 构造参数即字段、缺省 0/空。

- [ ] **Step 1: 写失败测试** `tests/test_infra.py`

```python
# -*- coding: utf-8 -*-
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import log_online_config as cfg  # noqa: E402
import mock_ros  # noqa: E402


class TestConfig(unittest.TestCase):
    def test_defaults(self):
        self.assertEqual(cfg.VEHICLE_ID, "A03")
        self.assertEqual(cfg.TOPIC_CAN, "/can_msg")
        self.assertEqual(cfg.SAMPLE_HZ, 10)
        self.assertEqual(cfg.RING_SECONDS, 120.0)
        self.assertEqual(cfg.PRE_WINDOW_S, 90.0)
        self.assertEqual(cfg.POST_WINDOW_S, 90.0)
        self.assertEqual(cfg.DEBOUNCE_TICKS, 5)
        self.assertEqual(cfg.DISK_CAP_BYTES, 1073741824)
        self.assertEqual(cfg.ROTATE_KEEP_RATIO, 0.9)
        self.assertEqual(cfg.MAX_OBJECTS, 64)
        self.assertEqual(cfg.STALE_AFTER_S, 2.0)
        self.assertEqual(cfg.HTTP_PORT, 8083)
        self.assertEqual(cfg.TOPIC_CAMERA, "/cam0/compressed")


class TestMockRos(unittest.TestCase):
    def test_install_and_subscriber(self):
        mock_ros.install()
        import rospy  # noqa: F401
        from robot.msg import can_msg as can_msg_mod  # noqa: F401

        sub = rospy.Subscriber("/can_msg", can_msg_mod.can_msg, lambda m: None, queue_size=1)
        self.assertEqual(sub.topic, "/can_msg")
        msg = can_msg_mod.can_msg(vehicleSpeed=1.5, curGear=4, emergencyStop=1)
        self.assertEqual(msg.vehicleSpeed, 1.5)
        self.assertEqual(msg.emergencyStop, 1)

    def test_get_param_default(self):
        mock_ros.install()
        import rospy
        self.assertEqual(rospy.get_param("/planning/sensorstate", 0), 0)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 运行确认失败**

Run: `cd log_online && python3 tests/test_infra.py`
Expected: FAIL（ModuleNotFoundError: log_online_config / mock_ros）

- [ ] **Step 3: 实现 `log_online_config.py`**

```python
# -*- coding: utf-8 -*-
"""log_online 配置。所有键带默认值，车载可直接改本文件。"""

VEHICLE_ID = "A03"

TOPIC_CAN = "/can_msg"
TOPIC_NAVIGATION = "/navigation_msg"
TOPIC_PERCEPTION = "/perception"
TOPIC_PLAN = "/plan_path_msg"
TOPIC_CONTROL = "/control_msg"
# cam_geac ros1_jpg 按 cam{n}/compressed 相对名发布;实车 rostopic list 核对,空串=视频状态禁用
TOPIC_CAMERA = "/cam0/compressed"

PARAM_SENSORSTATE = "/planning/sensorstate"
PARAM_NETCHECK = "/robot/planning/netcheck"
PARAM_ULTRA_SAFE = "/ultra/status/safe"
PARAM_LIGHT = "/canbus/light"
PARAM_HORN = "/canbus/horn"

SAMPLE_HZ = 10                 # 快照频率
RING_SECONDS = 120.0           # 环形缓冲时长(需求90s+30s余量)
PRE_WINDOW_S = 90.0            # 事件前窗口
POST_WINDOW_S = 90.0           # 事件后窗口
DEBOUNCE_TICKS = 5             # 触发去抖(0.5s@10Hz)
FLUSH_INTERVAL_S = 1.0         # 落盘 flush 周期
DISK_CAP_BYTES = 1073741824    # 事件数据磁盘上限(1GiB)
ROTATE_KEEP_RATIO = 0.9        # 轮转目标水位
MAX_OBJECTS = 64               # 单快照障碍物上限
STALE_AFTER_S = 2.0            # 链路 stale 阈值
ACCEL_EMA_S = 0.5              # 自算加速度 EMA 窗口
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8083               # 状态页(monitor 已占 8081/8082)
```

注意：`PARAM_ULTRA_SAFE` 那行的 `.replace` 是笔误示范勿抄——直接写 `PARAM_ULTRA_SAFE = "/ultra/status/safe"`。

- [ ] **Step 4: 实现 `tests/mock_ros.py`**

```python
# -*- coding: utf-8 -*-
"""无 ROS 本机的 rospy/消息桩。recorder.py 顶部 import rospy/robot.msg/sensor_msgs.msg,
测试在 import recorder 之前先 install()。仿 monitor tests 的桩思路。"""
import sys
import types


def _msg_class(fields):
    class _Msg(object):
        def __init__(self, **kwargs):
            for name, default in fields.items():
                setattr(self, name, kwargs.get(name, default))
    return _Msg


class _TransportHints(object):
    def __init__(self, tcp_nodelay=False):
        self.tcp_nodelay = tcp_nodelay

    def tcpNoDelay(self):
        self.tcp_nodelay = True
        return self


class _Subscriber(object):
    def __init__(self, topic, data_class, callback, queue_size=None, tcp_nodelay=False):
        self.topic = topic
        self.data_class = data_class
        self.callback = callback
        self.queue_size = queue_size
        self.tcp_nodelay = tcp_nodelay


class _Timer(object):
    def __init__(self, period_secs, callback):
        self.period_secs = period_secs
        self.callback = callback


class _FakeRospy(object):
    def __init__(self):
        self.params = {}
        self.init_node_names = []
        self.subscribers = []

    def init_node(self, name, anonymous=False):
        self.init_node_names.append(name)

    def Subscriber(self, topic, data_class, callback, queue_size=None, tcp_nodelay=False):
        sub = _Subscriber(topic, data_class, callback, queue_size, tcp_nodelay)
        self.subscribers.append(sub)
        return sub

    def TransportHints(self):
        return _TransportHints()

    def Timer(self, period_secs, callback):
        return _Timer(period_secs, callback)

    def get_param(self, name, default=None):
        return self.params.get(name, default)

    def is_shutdown(self):
        return False

    def spin(self):
        pass


def install():
    """把假 rospy/robot.msg/sensor_msgs.msg 注入 sys.modules(幂等)。"""
    rospy = getattr(install, "_rospy", None)
    if rospy is None:
        rospy = _FakeRospy()
        install._rospy = rospy
    robot_msg = types.ModuleType("robot.msg")
    robot_pkg = types.ModuleType("robot")
    robot_pkg.msg = robot_msg
    robot_msg.can_msg = _msg_class({
        "controlPanelState": 0, "eabPanelState": 0, "curGear": 0,
        "vehicleSpeed": 0.0, "emergencyStop": 0, "faultCode": b""})
    robot_msg.navigation_msg = _msg_class({
        "lat": 0.0, "lon": 0.0, "altitude": 0.0, "heading": 0.0,
        "xAxis": 0.0, "yAxis": 0.0, "zAxis": 0.0, "gpsSpeed": 0.0,
        "longitudinal_accelerate": 0.0, "rtkState": ""})
    robot_msg.object = _msg_class({
        "id": 0, "type": 0, "x": 0.0, "y": 0.0, "dx": 0.0, "dy": 0.0,
        "heading": 0.0, "height": 0.0, "vx": 0.0, "vy": 0.0, "confidence": 0.0})
    robot_msg.perception = _msg_class({"objs": []})
    robot_msg.path_plan_msg = _msg_class({"desireSpeed": 0.0, "safety": False})
    robot_msg.control_msg = _msg_class({
        "throttlePercent": 0, "brakePercent": 0, "wheelAngle": 0.0,
        "desireSpeed": 0.0, "desireAcc": 0.0})
    sensor_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msg.CompressedImage = _msg_class({"format": "jpeg", "data": b""})
    sys.modules["rospy"] = rospy
    sys.modules["robot"] = robot_pkg
    sys.modules["robot.msg"] = robot_msg
    sys.modules["sensor_msgs"] = types.ModuleType("sensor_msgs")
    sys.modules["sensor_msgs.msg"] = sensor_msg
    return rospy
```

同时创建空文件 `tests/__init__.py`（`touch`）。

- [ ] **Step 5: 运行确认通过**

Run: `cd log_online && python3 tests/test_infra.py`
Expected: OK (3 tests)

---

### Task 2: 快照构建（fault_code_hex / AccelEstimator / build_snapshot）

**Files:**
- Create: `log_online/recorder.py`（本任务先建文件，只含本任务三个单元 + 顶部 import）
- Test: `log_online/tests/test_snapshot.py`

**Interfaces:**
- Consumes: `log_online_config`（STALE_AFTER_S/MAX_OBJECTS/TOPIC_CAMERA）。
- Produces（后续任务依赖，签名固定）:
  - `fault_code_hex(code_bytes) -> str`（bytes 或 int 列表 → 小写 hex 串，空→`""`）
  - `class AccelEstimator(window_s)`，`update(speed, now) -> float`（speed=None 返回当前 EMA 不更新）
  - `build_snapshot(latest, last_seen, accel, now, config) -> dict`：
    - `latest`: `{"can","navigation","perception","plan","control","camera"}` 为消息或 None，加 `"sensorstate","netcheck","ultra_safe","light","horn"`（数值）与 `"camera_fps"`（float|None）
    - `last_seen`: 上述六个消息键 → 时间戳
    - 返回 spec §5 的七类快照 dict，缺失/stale 子字段为 None 且子对象带 `"stale": true`

- [ ] **Step 1: 写失败测试** `tests/test_snapshot.py`

```python
# -*- coding: utf-8 -*-
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import log_online_config as cfg  # noqa: E402
import mock_ros  # noqa: E402

mock_ros.install()
from robot.msg import can_msg as can_mod, navigation_msg as nav_mod, object as obj_mod, \
    perception as perc_mod, path_plan_msg as plan_mod, control_msg as ctrl_mod  # noqa: E402
import recorder  # noqa: E402

NOW = 1000.0


def fresh_latest(**over):
    latest = {
        "can": can_mod.can_msg(vehicleSpeed=1.2, curGear=4, controlPanelState=1,
                               emergencyStop=0, faultCode=b"\x01\x02"),
        "navigation": nav_mod.navigation_msg(lat=31.1, lon=121.8, xAxis=3.0, yAxis=4.0,
                                             heading=90.0, rtkState="fixed",
                                             longitudinal_accelerate=0.0),
        "perception": perc_mod.perception(objs=[
            obj_mod.object(id=1, type=0, x=3.0, y=4.0, dx=4.0, dy=2.0, confidence=0.9)]),
        "plan": plan_mod.path_plan_msg(desireSpeed=1.5, safety=False),
        "control": ctrl_mod.control_msg(throttlePercent=9, brakePercent=0, wheelAngle=12.5),
        "camera": object(),
        "sensorstate": 0, "netcheck": 0, "ultra_safe": 0, "light": 3, "horn": 0,
        "camera_fps": 15.0,
    }
    latest.update(over)
    return latest


def fresh_seen(**over):
    seen = {"can": NOW - 0.05, "navigation": NOW - 0.02, "perception": NOW - 0.01,
            "plan": NOW - 0.1, "control": NOW - 0.05, "camera": NOW - 0.1}
    seen.update(over)
    return seen


class TestFaultCodeHex(unittest.TestCase):
    def test_bytes_and_list(self):
        self.assertEqual(recorder.fault_code_hex(b"\x01\xaa"), "01aa")
        self.assertEqual(recorder.fault_code_hex([1, 170]), "01aa")
        self.assertEqual(recorder.fault_code_hex(b""), "")
        self.assertEqual(recorder.fault_code_hex(None), "")


class TestAccelEstimator(unittest.TestCase):
    def test_constant_accel_converges(self):
        est = recorder.AccelEstimator(0.5)
        t, v, out = 0.0, 0.0, 0.0
        for i in range(200):
            t = i * 0.1
            v = 0.5 * t
            out = est.update(v, t)
        self.assertGreater(out, 0.45)
        self.assertLess(out, 0.55)

    def test_none_speed_keeps_value(self):
        est = recorder.AccelEstimator(0.5)
        est.update(0.0, 0.0)
        est.update(1.0, 1.0)
        val = est.update(None, 2.0)
        val2 = est.update(None, 3.0)
        self.assertEqual(val, val2)


class TestBuildSnapshot(unittest.TestCase):
    def test_schema_complete(self):
        snap = recorder.build_snapshot(fresh_latest(), fresh_seen(), 0.1, NOW, cfg)
        for key in ("ts", "vehicle_id", "control", "position", "motion", "perception",
                    "response", "lights", "video", "fault"):
            self.assertIn(key, snap)
        self.assertEqual(snap["vehicle_id"], "A03")
        self.assertEqual(snap["control"]["controlPanelState"], 1)
        self.assertEqual(snap["position"]["lat"], 31.1)
        self.assertEqual(snap["motion"]["gear"], 4)
        self.assertEqual(snap["motion"]["speed"], 1.2)
        self.assertEqual(snap["motion"]["accel"], 0.1)
        self.assertEqual(snap["motion"]["accel_raw"], 0.0)
        self.assertEqual(snap["perception"]["obj_count"], 1)
        self.assertEqual(snap["perception"]["nearest_dist"], 0.0)  # 障碍(3,4)与自车(3,4)重合
        self.assertEqual(snap["response"]["throttle"], 9)
        self.assertEqual(snap["lights"]["light"], 3)
        self.assertTrue(snap["video"]["online"])
        self.assertEqual(snap["fault"]["faultCode_hex"], "0102")

    def test_missing_msg_gives_null_and_stale(self):
        snap = recorder.build_snapshot(fresh_latest(can=None), fresh_seen(), 0.0, NOW, cfg)
        self.assertIsNone(snap["motion"]["speed"])
        self.assertTrue(snap["motion"]["stale"])
        self.assertTrue(snap["control"]["stale"])

    def test_stale_by_age(self):
        snap = recorder.build_snapshot(fresh_latest(), fresh_seen(can=NOW - 3.0), 0.0, NOW, cfg)
        self.assertTrue(snap["motion"]["stale"])

    def test_object_truncation(self):
        objs = [obj_mod.object(id=i, x=10.0 + i, y=0.0) for i in range(100)]
        snap = recorder.build_snapshot(
            fresh_latest(perception=perc_mod.perception(objs=objs)), fresh_seen(), 0.0, NOW, cfg)
        self.assertEqual(snap["perception"]["obj_count"], 100)
        self.assertEqual(len(snap["perception"]["objs"]), cfg.MAX_OBJECTS)

    def test_nearest_dist_uses_ego_position(self):
        objs = [obj_mod.object(id=1, x=6.0, y=8.0)]
        snap = recorder.build_snapshot(
            fresh_latest(perception=perc_mod.perception(objs=objs)), fresh_seen(), 0.0, NOW, cfg)
        self.assertEqual(snap["perception"]["nearest_dist"], 5.0)  # hypot(6-3, 8-4)

    def test_camera_disabled_when_topic_empty(self):
        class NoCamCfg(object):
            STALE_AFTER_S = cfg.STALE_AFTER_S
            MAX_OBJECTS = cfg.MAX_OBJECTS
            TOPIC_CAMERA = ""
        snap = recorder.build_snapshot(fresh_latest(), fresh_seen(), 0.0, NOW, NoCamCfg)
        self.assertIsNone(snap["video"]["online"])

    def test_camera_stale_offline(self):
        snap = recorder.build_snapshot(fresh_latest(), fresh_seen(camera=NOW - 3.0), 0.0, NOW, cfg)
        self.assertFalse(snap["video"]["online"])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 运行确认失败**

Run: `cd log_online && python3 tests/test_snapshot.py`
Expected: FAIL（ImportError: recorder / 无属性 fault_code_hex）

- [ ] **Step 3: 实现 `recorder.py`（本任务部分）**

```python
# -*- coding: utf-8 -*-
"""log_online 记录器:订阅缓存 -> 10Hz 快照 -> 环形缓冲 -> 事件状态机 -> 落盘/轮转。
纯逻辑单元(快照/判定/状态机)不依赖 ROS,便于本机测试。"""
import collections
import json
import os
import shutil
import threading
import time
from pathlib import Path

import rospy
# 注意:不要 import robot.msg 的 object 类——会遮蔽内建 object,使后续 class X(object)
# 继承到消息类上;objs 元素仅按属性访问,无需导入
from robot.msg import can_msg, control_msg, navigation_msg, path_plan_msg, perception
from sensor_msgs.msg import CompressedImage

import log_online_config as default_config

SOFTWARE_VERSION = "log_online 1.0.0"


def fault_code_hex(code_bytes):
    """can_msg.faultCode(uint8[],rospy 到达为 bytes)转小写 hex 串。"""
    if not code_bytes:
        return ""
    return "".join("%02x" % int(b) for b in code_bytes)


class AccelEstimator(object):
    """车速差分加速度 EMA。navigation 的 longitudinal_accelerate 无生产者,自算为准。"""

    def __init__(self, window_s):
        self._window_s = max(1e-6, float(window_s))
        self._last_speed = None
        self._last_ts = None
        self._ema = 0.0

    def update(self, speed, now):
        if speed is None:
            return self._ema
        if self._last_speed is None or now <= self._last_ts:
            self._last_speed, self._last_ts = speed, now
            return self._ema
        dt = now - self._last_ts
        raw = (speed - self._last_speed) / dt
        alpha = min(1.0, dt / self._window_s)
        self._ema += alpha * (raw - self._ema)
        self._last_speed, self._last_ts = speed, now
        return self._ema

    def ema_value(self):
        """当前 EMA 值(get_status 用,不推进状态)。"""
        return self._ema


def _block(fields_getters, ok):
    """按数据新鲜度构建子对象:不新鲜则全 None + stale=True。"""
    if not ok:
        out = {name: None for name in fields_getters}
        out["stale"] = True
        return out
    out = {name: getter() for name, getter in fields_getters.items()}
    out["stale"] = False
    return out


def build_snapshot(latest, last_seen, accel, now, config):
    def fresh(key):
        ts = last_seen.get(key)
        return ts is not None and (now - ts) <= config.STALE_AFTER_S

    can = latest.get("can")
    can_ok = can is not None and fresh("can")
    nav = latest.get("navigation")
    nav_ok = nav is not None and fresh("navigation")

    control = _block({
        "controlPanelState": lambda: can.controlPanelState,
        "eabPanelState": lambda: can.eabPanelState,
    }, can_ok)

    position = _block({
        "lat": lambda: nav.lat,
        "lon": lambda: nav.lon,
        "x": lambda: nav.xAxis,
        "y": lambda: nav.yAxis,
        "heading": lambda: nav.heading,
        "rtkState": lambda: nav.rtkState,
    }, nav_ok)

    motion = {
        "gear": can.curGear if can_ok else None,
        "speed": can.vehicleSpeed if can_ok else None,
        "accel": accel,
        "accel_raw": nav.longitudinal_accelerate if nav_ok else None,
        "stale": not can_ok,
    }

    perc = latest.get("perception")
    if perc is None or not fresh("perception"):
        perception_block = {"obj_count": None, "objs": None, "nearest_dist": None,
                            "stale": True}
    else:
        objs_raw = list(getattr(perc, "objs", []) or [])
        ego_x = nav.xAxis if nav_ok else None
        ego_y = nav.yAxis if nav_ok else None
        objs = []
        nearest = None
        for o in objs_raw[:config.MAX_OBJECTS]:
            objs.append({
                "id": o.id, "type": o.type, "x": o.x, "y": o.y,
                "dx": o.dx, "dy": o.dy, "heading": o.heading, "height": o.height,
                "vx": o.vx, "vy": o.vy, "confidence": o.confidence,
            })
            if ego_x is not None:
                d = (o.x - ego_x) ** 2 + (o.y - ego_y) ** 2
                nearest = d if nearest is None else min(nearest, d)
        if nearest is not None:
            nearest = round(nearest ** 0.5, 3)
        perception_block = {"obj_count": len(objs_raw), "objs": objs,
                            "nearest_dist": nearest, "stale": False}

    plan = latest.get("plan")
    plan_ok = plan is not None and fresh("plan")
    ctrl = latest.get("control")
    ctrl_ok = ctrl is not None and fresh("control")
    response = {
        "safety": plan.safety if plan_ok else None,
        "desireSpeed": plan.desireSpeed if plan_ok else None,
        "throttle": ctrl.throttlePercent if ctrl_ok else None,
        "brake": ctrl.brakePercent if ctrl_ok else None,
        "wheelAngle": ctrl.wheelAngle if ctrl_ok else None,
        "desireAcc": ctrl.desireAcc if ctrl_ok else None,
        "sensorstate": latest.get("sensorstate", 0),
        "ultra_safe": latest.get("ultra_safe", 0),
        "stale": not (plan_ok and ctrl_ok),
    }

    lights = {"light": latest.get("light", 0), "horn": latest.get("horn", 0), "stale": False}

    camera_ts = last_seen.get("camera")
    if not getattr(config, "TOPIC_CAMERA", ""):
        video = {"online": None, "last_frame_age_ms": None, "fps": None, "stale": None}
    elif camera_ts is None or (now - camera_ts) > config.STALE_AFTER_S:
        video = {"online": False,
                 "last_frame_age_ms": None if camera_ts is None else round((now - camera_ts) * 1000.0, 1),
                 "fps": None, "stale": True}
    else:
        video = {"online": True, "last_frame_age_ms": round((now - camera_ts) * 1000.0, 1),
                 "fps": latest.get("camera_fps"), "stale": False}

    fault = {
        "emergencyStop": can.emergencyStop if can_ok else None,
        "netcheck": latest.get("netcheck", 0),
        "faultCode_hex": fault_code_hex(can.faultCode) if can_ok else None,
        "stale": not can_ok,
    }

    return {
        "ts": now,
        "vehicle_id": config.VEHICLE_ID,
        "control": control,
        "position": position,
        "motion": motion,
        "perception": perception_block,
        "response": response,
        "lights": lights,
        "video": video,
        "fault": fault,
    }
```

- [ ] **Step 4: 运行确认通过**

Run: `cd log_online && python3 tests/test_snapshot.py && python3 tests/test_infra.py`
Expected: 全部 OK

---

### Task 3: 环形缓冲 / 触发判定 / 去抖

**Files:**
- Modify: `log_online/recorder.py`（追加三个单元）
- Test: `log_online/tests/test_trigger.py`

**Interfaces:**
- Produces:
  - `class RingBuffer(seconds)`：`append(snap)` / `trim(now)` / `tail(since_ts) -> list` / `__len__` / `span(now) -> float`
  - `evaluate_trigger(snapshot) -> list[str]`（空=无；顺序 emergencystop, sensorstate, netcheck, faultcode；None 按 0 不触发）
  - `class Debouncer(ticks)`：`feed(active) -> bool`（连续第 ticks 次激活的那一拍返回 True，仅此一拍）

- [ ] **Step 1: 写失败测试** `tests/test_trigger.py`

```python
# -*- coding: utf-8 -*-
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mock_ros  # noqa: E402

mock_ros.install()
import recorder  # noqa: E402


def snap(**over):
    base = {
        "ts": 0.0,
        "response": {"sensorstate": 0, "safety": 0},
        "fault": {"emergencyStop": 0, "netcheck": 0, "faultCode_hex": ""},
    }
    base.update(over)
    return base


class TestRingBuffer(unittest.TestCase):
    def test_trim_keeps_window(self):
        ring = recorder.RingBuffer(120.0)
        for i in range(1500):
            ring.append({"ts": float(i) * 0.1})
            ring.trim(float(i) * 0.1)
        self.assertEqual(ring.span(149.9), 149.9 - ring.tail(0.0)[0]["ts"])
        self.assertLessEqual(ring.span(149.9), 120.0 + 0.1)
        self.assertGreaterEqual(ring.span(149.9), 119.0)
        tail = ring.tail(60.0)
        self.assertAlmostEqual(tail[0]["ts"], 60.0, places=5)   # 0.1 步进有浮点尾差
        self.assertAlmostEqual(tail[-1]["ts"], 149.9, places=5)


class TestEvaluateTrigger(unittest.TestCase):
    def test_none_triggers_nothing(self):
        self.assertEqual(recorder.evaluate_trigger(
            snap(fault={"emergencyStop": None, "netcheck": 0, "faultCode_hex": None},
                 response={"sensorstate": None})), [])

    def test_each_condition(self):
        self.assertEqual(recorder.evaluate_trigger(
            snap(fault={"emergencyStop": 1, "netcheck": 0, "faultCode_hex": ""})),
            ["emergencystop"])
        self.assertEqual(recorder.evaluate_trigger(
            snap(response={"sensorstate": 3})), ["sensorstate"])
        self.assertEqual(recorder.evaluate_trigger(
            snap(fault={"emergencyStop": 0, "netcheck": 1, "faultCode_hex": ""})),
            ["netcheck"])
        self.assertEqual(recorder.evaluate_trigger(
            snap(fault={"emergencyStop": 0, "netcheck": 0, "faultCode_hex": "01"})),
            ["faultcode"])

    def test_multiple_order(self):
        self.assertEqual(recorder.evaluate_trigger(
            snap(fault={"emergencyStop": 1, "netcheck": 1, "faultCode_hex": "aa"},
                 response={"sensorstate": 2})),
            ["emergencystop", "sensorstate", "netcheck", "faultcode"])


class TestDebouncer(unittest.TestCase):
    def test_fires_exactly_once_at_nth(self):
        deb = recorder.Debouncer(5)
        results = [deb.feed(True) for _ in range(7)]
        self.assertEqual(results, [False, False, False, False, True, False, False])

    def test_reset_on_inactive(self):
        deb = recorder.Debouncer(5)
        for _ in range(4):
            deb.feed(True)
        self.assertFalse(deb.feed(False))
        self.assertFalse(deb.feed(True))
        self.assertFalse(deb.feed(True))
        self.assertFalse(deb.feed(True))
        self.assertFalse(deb.feed(True))
        self.assertTrue(deb.feed(True))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 运行确认失败**

Run: `cd log_online && python3 tests/test_trigger.py`
Expected: FAIL（无属性 RingBuffer / evaluate_trigger / Debouncer）

- [ ] **Step 3: 实现（追加到 recorder.py）**

```python
class RingBuffer(object):
    """时间窗环形缓冲:append 后按 now-seconds 裁剪,事件开启时用 tail 取前窗数据。"""

    def __init__(self, seconds):
        self._seconds = float(seconds)
        self._items = collections.deque()

    def append(self, snapshot):
        self._items.append(snapshot)

    def trim(self, now):
        cutoff = now - self._seconds
        while self._items and self._items[0]["ts"] < cutoff:
            self._items.popleft()

    def tail(self, since_ts):
        return [s for s in self._items if s["ts"] >= since_ts]

    def __len__(self):
        return len(self._items)

    def span(self, now):
        if not self._items:
            return 0.0
        return now - self._items[0]["ts"]


def evaluate_trigger(snapshot):
    """触发判定;None(数据缺失/stale)按 0 处理不触发,避免开机无数据误开事件。"""
    types = []
    fault = snapshot.get("fault") or {}
    response = snapshot.get("response") or {}
    if fault.get("emergencyStop"):
        types.append("emergencystop")
    if response.get("sensorstate"):
        types.append("sensorstate")
    if fault.get("netcheck"):
        types.append("netcheck")
    if fault.get("faultCode_hex"):
        types.append("faultcode")
    return types


class Debouncer(object):
    """连续 ticks 拍激活才放行一次(去抖沿);失活即清零。"""

    def __init__(self, ticks):
        self._ticks = int(ticks)
        self._count = 0

    def feed(self, active):
        if not active:
            self._count = 0
            return False
        self._count += 1
        return self._count == self._ticks
```

- [ ] **Step 4: 运行确认通过**

Run: `cd log_online && python3 tests/test_trigger.py && python3 tests/test_snapshot.py && python3 tests/test_infra.py`
Expected: 全部 OK

---

### Task 4: 事件状态机（纯逻辑）

**Files:**
- Modify: `log_online/recorder.py`（追加）
- Test: `log_online/tests/test_machine.py`

**Interfaces:**
- Produces: `class EventStateMachine(pre_window_s, post_window_s)`
  - 属性 `state`（"IDLE"/"RECORDING"/"POST"）、`segments`（`[{"start","end","types"}]`，open 中 end 为 None）
  - `feed(now, debounced_start, active_types) -> list[action]`，action ∈ `("open", first_type)` / `("extend", [types])` / `("close",)`
  - **追加当前拍快照由调用方负责**：feed 返回后只要 `state != "IDLE"` 就 append；`close` 动作那一拍调用方要先 append 再执行 close（保证 post 窗末条 ts ≥ 清除+90s）。

- [ ] **Step 1: 写失败测试** `tests/test_machine.py`

```python
# -*- coding: utf-8 -*-
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mock_ros  # noqa: E402

mock_ros.install()
import recorder  # noqa: E402

T_NONE, T_ES = [], ["emergencystop"]


class TestStateMachine(unittest.TestCase):
    def test_full_lifecycle(self):
        m = recorder.EventStateMachine(90.0, 90.0)
        # t=100 去抖通过开事件
        self.assertEqual(m.feed(100.0, True, T_ES), [("open", "emergencystop")])
        self.assertEqual(m.state, "RECORDING")
        self.assertEqual(m.segments, [{"start": 100.0, "end": None, "types": T_ES}])
        # 持续激活:无动作(调用方照 state 追加)
        self.assertEqual(m.feed(100.1, False, T_NONE), [])
        self.assertEqual(m.state, "POST")  # 触发清除 -> POST
        self.assertEqual(m.segments[-1]["end"], 100.1)
        # post 未满:无动作
        self.assertEqual(m.feed(190.0, False, T_NONE), [])
        # post 满:close
        self.assertEqual(m.feed(190.1, False, T_NONE), [("close",)])
        self.assertEqual(m.state, "IDLE")

    def test_post_retrigger_extends(self):
        m = recorder.EventStateMachine(90.0, 90.0)
        m.feed(100.0, True, T_ES)
        m.feed(100.1, False, T_NONE)
        # post 期间(第 50s)再次触发(无需去抖)
        self.assertEqual(m.feed(150.0, False, ["sensorstate"]), [("extend", ["sensorstate"])])
        self.assertEqual(m.state, "RECORDING")
        self.assertEqual(len(m.segments), 2)
        self.assertEqual(m.segments[1]["types"], ["sensorstate"])
        # 再清除后重新计满 90s 才关
        m.feed(150.1, False, T_NONE)
        self.assertEqual(m.feed(240.0, False, T_NONE), [])
        self.assertEqual(m.feed(240.2, False, T_NONE), [("close",)])

    def test_debounce_only_from_idle(self):
        m = recorder.EventStateMachine(90.0, 90.0)
        # 去抖未通过
        self.assertEqual(m.feed(100.0, False, T_ES), [])
        self.assertEqual(m.state, "IDLE")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 运行确认失败**

Run: `cd log_online && python3 tests/test_machine.py`
Expected: FAIL（无属性 EventStateMachine）

- [ ] **Step 3: 实现（追加到 recorder.py）**

```python
class EventStateMachine(object):
    """IDLE -> RECORDING(去抖通过) -> POST(触发清除) -> IDLE(满 post 窗)。
    不做 IO;feed 返回动作由 Recorder 执行。追加当前拍快照由调用方按 state 负责。"""

    def __init__(self, pre_window_s, post_window_s):
        self.pre_window_s = float(pre_window_s)
        self.post_window_s = float(post_window_s)
        self.state = "IDLE"
        self.segments = []
        self._post_since = None

    def feed(self, now, debounced_start, active_types):
        actions = []
        active = bool(active_types)
        if self.state == "IDLE":
            if debounced_start and active:
                self.state = "RECORDING"
                self.segments.append(
                    {"start": now, "end": None, "types": list(active_types)})
                actions.append(("open", active_types[0]))
        elif self.state == "RECORDING":
            if not active:
                self.segments[-1]["end"] = now
                self.state = "POST"
                self._post_since = now
        elif self.state == "POST":
            if active:
                self.state = "RECORDING"
                self._post_since = None
                self.segments.append(
                    {"start": now, "end": None, "types": list(active_types)})
                actions.append(("extend", list(active_types)))
            elif now - self._post_since >= self.post_window_s:
                self.state = "IDLE"
                actions.append(("close",))
        return actions
```

- [ ] **Step 4: 运行确认通过**

Run: `cd log_online && python3 tests/test_machine.py && python3 tests/test_trigger.py`
Expected: 全部 OK

---

### Task 5: EventWriter 与 Rotator（IO 层）

**Files:**
- Modify: `log_online/recorder.py`（追加）
- Test: `log_online/tests/test_writer.py`

**Interfaces:**
- Consumes: `SOFTWARE_VERSION`、`fault_code_hex` 无关；config 常量经构造传入。
- Produces:
  - `class EventWriter(data_dir, vehicle_id, config)`：
    - `open(event_type, trigger_ts, pre_records, health) -> str`（返回目录名；同名加 `_1` 后缀；写初始 metadata `complete:false`）
    - `append(snapshot)`；`maybe_flush(now)`（按 `FLUSH_INTERVAL_S`）；`refresh_metadata(segments)`
    - `close(segments, now) -> dict`（fsync、终版 metadata `complete:true`、`record_end=now`，返回 `{"dir","records","bytes"}`）
    - metadata 字段：`schema_version=1, vehicle_id, event_dir, trigger_segments, record_start, record_end(None until close), pre_window_s, post_window_s, sample_hz, debounce_ticks, complete, software, health_at_trigger, stats{records,bytes}`
  - `class Rotator(data_dir)`：`scan() -> int`（总字节数，缓存各目录大小）；`evict_if_over(cap_bytes, keep_ratio) -> list[str]`（重扫后按目录名升序删最旧至 ≤cap×keep_ratio，`shutil.rmtree(ignore_errors=True)`，返回被删目录名）

- [ ] **Step 1: 写失败测试** `tests/test_writer.py`

```python
# -*- coding: utf-8 -*-
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import log_online_config as cfg  # noqa: E402
import mock_ros  # noqa: E402

mock_ros.install()
import recorder  # noqa: E402


def rec(ts):
    return {"ts": ts, "vehicle_id": "A03"}


class TestEventWriter(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.data_dir = Path(self.tmp.name) / "events"

    def tearDown(self):
        self.tmp.cleanup()

    def test_open_append_close(self):
        w = recorder.EventWriter(self.data_dir, "A03", cfg)
        name = w.open("emergencystop", 100.0, [rec(10.0), rec(10.1)],
                      {"can_msg": 0.02})
        self.assertEqual(name, time_name(100.0) + "_emergencystop")
        w.append(rec(100.0))
        w.append(rec(100.1))
        w.maybe_flush(100.2)
        info = w.close([{"start": 100.0, "end": 100.1, "types": ["emergencystop"]}], 200.0)
        self.assertEqual(info["records"], 4)
        meta = json.loads((self.data_dir / name / "metadata.json").read_text("utf-8"))
        self.assertTrue(meta["complete"])
        self.assertEqual(meta["record_start"], 10.0)
        self.assertEqual(meta["record_end"], 200.0)
        self.assertEqual(meta["vehicle_id"], "A03")
        self.assertEqual(meta["health_at_trigger"], {"can_msg": 0.02})
        lines = (self.data_dir / name / "records.jsonl").read_text("utf-8").strip().split("\n")
        self.assertEqual(len(lines), 4)
        self.assertEqual(json.loads(lines[0])["ts"], 10.0)

    def test_name_collision_suffix(self):
        w1 = recorder.EventWriter(self.data_dir, "A03", cfg)
        n1 = w1.open("emergencystop", 100.0, [], {})
        w1.close([], 100.5)
        w2 = recorder.EventWriter(self.data_dir, "A03", cfg)
        n2 = w2.open("emergencystop", 100.0, [], {})
        w2.close([], 100.5)
        self.assertNotEqual(n1, n2)
        self.assertTrue(n2.endswith("_1"))

    def test_metadata_while_open_incomplete(self):
        w = recorder.EventWriter(self.data_dir, "A03", cfg)
        w.open("faultcode", 300.0, [rec(210.0)], {})
        meta = json.loads((self.data_dir / w._dir_name / "metadata.json").read_text("utf-8"))
        self.assertFalse(meta["complete"])
        self.assertIsNone(meta["record_end"])
        # 不 close 直接丢引用 = 模拟进程被 kill,metadata 保持 incomplete


class TestRotator(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.data_dir = Path(self.tmp.name) / "events"
        self.data_dir.mkdir(parents=True)

    def tearDown(self):
        self.tmp.cleanup()

    def make_event(self, name, size_bytes):
        d = self.data_dir / name
        d.mkdir()
        (d / "records.jsonl").write_bytes(b"x" * size_bytes)

    def test_evict_oldest_until_under_target(self):
        self.make_event("a_1", 600)
        self.make_event("b_2", 600)
        self.make_event("c_3", 600)
        rot = recorder.Rotator(self.data_dir)
        self.assertEqual(rot.scan(), 1800)
        removed = rot.evict_if_over(1000, 0.9)  # 目标 900
        self.assertEqual(removed, ["a_1", "b_2"])
        self.assertFalse((self.data_dir / "a_1").exists())
        self.assertFalse((self.data_dir / "b_2").exists())
        self.assertTrue((self.data_dir / "c_3").exists())
        self.assertEqual(rot.scan(), 600)

    def test_under_cap_no_evict(self):
        self.make_event("a_1", 100)
        rot = recorder.Rotator(self.data_dir)
        self.assertEqual(rot.evict_if_over(1000, 0.9), [])


def time_name(ts):
    import time as _t
    return _t.strftime("%Y%m%d_%H%M%S", _t.localtime(ts))


if __name__ == "__main__":
    unittest.main()
```

时间戳→目录名断言两侧统一用 `%Y%m%d_%H%M%S` 本地时间（测试的 `time_name` 与 writer 内部一致）。

- [ ] **Step 2: 运行确认失败**

Run: `cd log_online && python3 tests/test_writer.py`
Expected: FAIL（无属性 EventWriter / Rotator）

- [ ] **Step 3: 实现（追加到 recorder.py）**

```python
class EventWriter(object):
    """单事件落盘:records.jsonl 逐行快照 + metadata.json(开启即写,关闭补终版)。"""

    def __init__(self, data_dir, vehicle_id, config):
        self._data_dir = Path(data_dir)
        self._vehicle_id = vehicle_id
        self._cfg = config
        self._dir_name = None
        self._fh = None
        self._segments = []
        self._trigger_ts = None
        self._record_start = None
        self._health = {}
        self._records = 0
        self._bytes = 0
        self._last_flush = None

    def open(self, event_type, trigger_ts, pre_records, health):
        base = time.strftime("%Y%m%d_%H%M%S", time.localtime(trigger_ts)) + "_" + event_type
        name = base
        n = 1
        self._data_dir.mkdir(parents=True, exist_ok=True)
        while (self._data_dir / name).exists():
            name = "%s_%d" % (base, n)
            n += 1
        (self._data_dir / name).mkdir()
        self._dir_name = name
        self._trigger_ts = trigger_ts
        self._health = dict(health or {})
        self._fh = open(str(self._data_dir / name / "records.jsonl"), "w",
                        encoding="utf-8")
        for r in pre_records:
            self._write(r)
        self._record_start = pre_records[0]["ts"] if pre_records else trigger_ts
        self._write_metadata()
        return name

    def _write(self, snapshot):
        line = json.dumps(snapshot, ensure_ascii=False, separators=(",", ":")) + "\n"
        self._fh.write(line)
        self._records += 1
        self._bytes += len(line.encode("utf-8"))

    def append(self, snapshot):
        self._write(snapshot)

    def maybe_flush(self, now):
        if self._fh is None:
            return
        if self._last_flush is None or now - self._last_flush >= self._cfg.FLUSH_INTERVAL_S:
            self._fh.flush()
            self._last_flush = now

    def refresh_metadata(self, segments):
        self._segments = [dict(s) for s in segments]
        self._write_metadata()

    def close(self, segments, now):
        self._segments = [dict(s) for s in segments]
        if self._fh is not None:
            self._fh.flush()
            os.fsync(self._fh.fileno())
            self._fh.close()
            self._fh = None
        record_end = now
        self._write_metadata(complete=True, record_end=record_end)
        return {"dir": self._dir_name, "records": self._records, "bytes": self._bytes}

    def _write_metadata(self, complete=False, record_end=None):
        meta = {
            "schema_version": 1,
            "vehicle_id": self._vehicle_id,
            "event_dir": self._dir_name,
            "trigger_segments": self._segments,
            "record_start": self._record_start,
            "record_end": record_end,
            "pre_window_s": self._cfg.PRE_WINDOW_S,
            "post_window_s": self._cfg.POST_WINDOW_S,
            "sample_hz": self._cfg.SAMPLE_HZ,
            "debounce_ticks": self._cfg.DEBOUNCE_TICKS,
            "complete": complete,
            "software": SOFTWARE_VERSION,
            "health_at_trigger": self._health,
            "stats": {"records": self._records, "bytes": self._bytes},
        }
        path = self._data_dir / self._dir_name / "metadata.json"
        path.write_text(
            json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")


class Rotator(object):
    """事件目录磁盘轮转:超上限删最旧至 keep_ratio 水位。"""

    def __init__(self, data_dir):
        self._data_dir = Path(data_dir)
        self._sizes = {}

    def scan(self):
        self._sizes = {}
        total = 0
        if self._data_dir.is_dir():
            for event_dir in sorted(self._data_dir.iterdir()):
                if not event_dir.is_dir():
                    continue
                size = sum(f.stat().st_size for f in event_dir.rglob("*") if f.is_file())
                self._sizes[event_dir.name] = size
                total += size
        return total

    def evict_if_over(self, cap_bytes, keep_ratio):
        self.scan()
        target = cap_bytes * keep_ratio
        total = sum(self._sizes.values())
        removed = []
        for name in sorted(self._sizes):
            if total <= target:
                break
            size = self._sizes[name]
            if shutil.rmtree(str(self._data_dir / name), ignore_errors=True):
                pass
            removed.append(name)
            total -= size
            print("[log_online] 轮转删除旧事件 %s (%d bytes)" % (name, size))
        return removed
```

- [ ] **Step 4: 运行确认通过**

Run: `cd log_online && python3 tests/test_writer.py && python3 tests/test_machine.py`
Expected: 全部 OK

---

### Task 6: Recorder 组装（订阅/参数/tick 编排）

**Files:**
- Modify: `log_online/recorder.py`（追加 Recorder 类 + main 占位）
- Test: `log_online/tests/test_recorder.py`

**Interfaces:**
- Consumes: 前五个任务的全部单元；`mock_ros.install()` 返回的假 rospy（`get_param/Subscriber/init_node`）。
- Produces:
  - `class Recorder(ros, config=None, data_dir=None)`：
    - `wire()`：`init_node("log_online", anonymous=True)` + 六个订阅（`queue_size=1, tcp_nodelay=True`，话题取 config）
    - 回调由 `_on(key)` 工厂产生（线程安全更新 `_latest/_last_seen/_hz_win/_camera_ts`）
    - `tick(now)`：参数热读→快照→环形→`evaluate_trigger`→`Debouncer.feed`→状态机→执行动作（open 时 `since = now - PRE_WINDOW_S - 1/SAMPLE_HZ - 1e-6` 保证首条 ≤ trigger−90s；close 拍先 append 再 close，随后 `Rotator.evict_if_over`）
    - `get_status() -> dict`：`snapshot/health/trigger/state/ring/disk/vehicle_id/ts`
    - `recent_events(limit=20) -> list[dict]`：扫 `data_dir/*/metadata.json` 按目录名倒序摘要
    - `run()`：`wire()` + `rospy.Timer(1/SAMPLE_HZ, ...)` + `spin()`（真实环境用；测试不调）
  - `main()`（本任务先写最小版：data_dir 默认 `Path(__file__).parent/"data"/"events"`，Recorder + 状态页留 Task 8 接线；本任务 main 只建 Recorder 并 run）

- [ ] **Step 1: 写失败测试** `tests/test_recorder.py`

```python
# -*- coding: utf-8 -*-
import json
import sys
import tempfile
import unittest
from pathlib import Path

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

    def rec_status(self):
        return self.h.rec.get_status()


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 运行确认失败**

Run: `cd log_online && python3 tests/test_recorder.py`
Expected: FAIL（无属性 Recorder）

- [ ] **Step 3: 实现（追加到 recorder.py）**

```python
class Recorder(object):
    """编排:订阅缓存(ZOH) -> tick 快照 -> 环形/状态机/落盘/轮转。"""

    def __init__(self, ros, config=None, data_dir=None):
        self._ros = ros
        self.cfg = config or default_config
        self.data_dir = Path(data_dir) if data_dir else \
            Path(__file__).resolve().parent / "data" / "events"
        self._lock = threading.Lock()
        self._latest = {"can": None, "navigation": None, "perception": None,
                        "plan": None, "control": None, "camera": None}
        self._last_seen = {}
        self._camera_ts = collections.deque(maxlen=128)
        self._hz_win = {k: collections.deque(maxlen=20)
                        for k in ("can", "navigation", "perception", "plan", "control", "camera")}
        self._accel = AccelEstimator(self.cfg.ACCEL_EMA_S)
        self._ring = RingBuffer(self.cfg.RING_SECONDS)
        self._deb = Debouncer(self.cfg.DEBOUNCE_TICKS)
        self._machine = EventStateMachine(self.cfg.PRE_WINDOW_S, self.cfg.POST_WINDOW_S)
        self._writer = None
        self._rotator = Rotator(self.data_dir)
        self._rotator.scan()

    # ---- 订阅与回调 ----------------------------------------------------
    def wire(self):
        self._ros.init_node("log_online", anonymous=True)
        self._ros.Subscriber(self.cfg.TOPIC_CAN, can_msg, self._on_can,
                             queue_size=1, tcp_nodelay=True)
        self._ros.Subscriber(self.cfg.TOPIC_NAVIGATION, navigation_msg,
                             self._on_navigation, queue_size=1, tcp_nodelay=True)
        self._ros.Subscriber(self.cfg.TOPIC_PERCEPTION, perception,
                             self._on_perception, queue_size=1, tcp_nodelay=True)
        self._ros.Subscriber(self.cfg.TOPIC_PLAN, path_plan_msg,
                             self._on_plan, queue_size=1, tcp_nodelay=True)
        self._ros.Subscriber(self.cfg.TOPIC_CONTROL, control_msg,
                             self._on_control, queue_size=1, tcp_nodelay=True)
        if getattr(self.cfg, "TOPIC_CAMERA", ""):
            self._ros.Subscriber(self.cfg.TOPIC_CAMERA, CompressedImage,
                                 self._on_camera, queue_size=1, tcp_nodelay=True)

    def _store(self, key, msg):
        now = time.time()
        with self._lock:
            self._latest[key] = msg
            self._last_seen[key] = now
            self._hz_win[key].append(now)
            if key == "camera":
                self._camera_ts.append(now)

    def _on_can(self, msg):
        self._store("can", msg)

    def _on_navigation(self, msg):
        self._store("navigation", msg)

    def _on_perception(self, msg):
        self._store("perception", msg)

    def _on_plan(self, msg):
        self._store("plan", msg)

    def _on_control(self, msg):
        self._store("control", msg)

    def _on_camera(self, msg):
        self._store("camera", msg)

    # ---- 周期 ----------------------------------------------------------
    def tick(self, now):
        with self._lock:
            latest = dict(self._latest)
            last_seen = dict(self._last_seen)
            camera_ts = list(self._camera_ts)
        for key, param_name in (("sensorstate", self.cfg.PARAM_SENSORSTATE),
                                ("netcheck", self.cfg.PARAM_NETCHECK),
                                ("ultra_safe", self.cfg.PARAM_ULTRA_SAFE),
                                ("light", self.cfg.PARAM_LIGHT),
                                ("horn", self.cfg.PARAM_HORN)):
            latest[key] = self._ros.get_param(param_name, 0)
        recent_frames = [t for t in camera_ts if now - t <= 5.0]
        latest["camera_fps"] = round(len(recent_frames) / 5.0, 2) if recent_frames else None
        speed = latest["can"].vehicleSpeed if latest.get("can") is not None else None
        accel = self._accel.update(speed, now)
        snapshot = build_snapshot(latest, last_seen, accel, now, self.cfg)
        self._ring.append(snapshot)
        self._ring.trim(now)

        active_types = evaluate_trigger(snapshot)
        debounced = self._deb.feed(bool(active_types))
        actions = self._machine.feed(now, debounced, active_types)
        for act in actions:
            if act[0] == "open":
                # 多留一个 tick 余量,保证 records 首条 ts <= trigger - PRE_WINDOW_S
                since = now - self.cfg.PRE_WINDOW_S - (1.0 / self.cfg.SAMPLE_HZ) - 1e-6
                pre_records = self._ring.tail(since)
                self._writer = EventWriter(self.data_dir, self.cfg.VEHICLE_ID, self.cfg)
                self._writer.open(act[1], now, pre_records,
                                  self._health(last_seen, now))
            elif act[0] == "extend":
                if self._writer is not None:
                    self._writer.refresh_metadata(self._machine.segments)

        if ("close",) in actions:
            if self._writer is not None:
                self._writer.append(snapshot)  # 关闭拍数据先补上,post 窗末条 >= 清除+90s
                self._writer.close(self._machine.segments, now)
                self._writer = None
                self._rotator.evict_if_over(self.cfg.DISK_CAP_BYTES,
                                            self.cfg.ROTATE_KEEP_RATIO)
        elif self._machine.state != "IDLE" and self._writer is not None:
            self._writer.append(snapshot)
            self._writer.maybe_flush(now)

    def _health(self, last_seen, now):
        return {key: round(now - ts, 3) for key, ts in sorted(last_seen.items())}

    # ---- 状态页数据 ------------------------------------------------------
    def get_status(self):
        now = time.time()
        with self._lock:
            latest = dict(self._latest)
            last_seen = dict(self._last_seen)
            camera_ts = list(self._camera_ts)
            hz_win = {k: list(v) for k, v in self._hz_win.items()}
        for key, param_name in (("sensorstate", self.cfg.PARAM_SENSORSTATE),
                                ("netcheck", self.cfg.PARAM_NETCHECK),
                                ("ultra_safe", self.cfg.PARAM_ULTRA_SAFE),
                                ("light", self.cfg.PARAM_LIGHT),
                                ("horn", self.cfg.PARAM_HORN)):
            latest[key] = self._ros.get_param(param_name, 0)
        recent_frames = [t for t in camera_ts if now - t <= 5.0]
        latest["camera_fps"] = round(len(recent_frames) / 5.0, 2) if recent_frames else None
        snapshot = build_snapshot(latest, last_seen,
                                  self._accel.ema_value(), now, self.cfg)
        health = {}
        for key, ts in last_seen.items():
            hz = None
            stamps = hz_win.get(key) or []
            if len(stamps) >= 2 and stamps[-1] != stamps[0]:
                hz = round((len(stamps) - 1) / (stamps[-1] - stamps[0]), 2)
            health[key] = {"last_seen_age_s": round(now - ts, 3), "hz_est": hz,
                           "stale": (now - ts) > self.cfg.STALE_AFTER_S}
        return {
            "ts": now,
            "vehicle_id": self.cfg.VEHICLE_ID,
            "snapshot": snapshot,
            "health": health,
            "trigger": {"conditions": evaluate_trigger(snapshot),
                        "state": self._machine.state,
                        "segments": self._machine.segments},
            "ring": {"count": len(self._ring), "span_s": round(self._ring.span(now), 2)},
            "disk": {"used_bytes": self._rotator.scan(),
                     "cap_bytes": self.cfg.DISK_CAP_BYTES},
            "event_active": self._writer is not None,
        }

    def recent_events(self, limit=20):
        out = []
        if self.data_dir.is_dir():
            for event_dir in sorted(self.data_dir.iterdir(), reverse=True):
                if not event_dir.is_dir():
                    continue
                meta_path = event_dir / "metadata.json"
                if not meta_path.is_file():
                    continue
                try:
                    meta = json.loads(meta_path.read_text("utf-8"))
                except (ValueError, OSError):
                    continue
                out.append({
                    "dir": event_dir.name,
                    "vehicle_id": meta.get("vehicle_id"),
                    "trigger_segments": meta.get("trigger_segments"),
                    "record_start": meta.get("record_start"),
                    "record_end": meta.get("record_end"),
                    "complete": meta.get("complete"),
                    "stats": meta.get("stats"),
                })
                if len(out) >= limit:
                    break
        return out

    def run(self):
        self.wire()
        self._ros.Timer(1.0 / self.cfg.SAMPLE_HZ,
                        lambda _event=None: self.tick(time.time()))
        self._ros.spin()


def main():
    rec = Recorder(rospy)
    rec.run()


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 运行确认通过**

Run: `cd log_online && python3 tests/test_recorder.py && python3 tests/test_writer.py && python3 tests/test_machine.py && python3 tests/test_trigger.py && python3 tests/test_snapshot.py && python3 tests/test_infra.py`
Expected: 全部 OK

---

### Task 7: 端到端事件生命周期（pre/post ≥90s、扩展、状态数据）

**Files:**
- Test: `log_online/tests/test_lifecycle.py`

**Interfaces:**
- Consumes: Task 6 的 `Recorder`/`RecorderHarness` 模式（本文件自带 harness，不 import 测试模块）。

- [ ] **Step 1: 写测试**（本任务是验收性测试，直接写对，跑通即过；失败说明 Task 2-6 有缺陷）

```python
# -*- coding: utf-8 -*-
"""端到端:200s 模拟时间线,t=100 急停 10s;验证事件目录/前后窗/complete/状态数据。"""
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import log_online_config as cfg  # noqa: E402
import mock_ros  # noqa: E402

ros = mock_ros.install()
from robot.msg import can_msg as can_mod, navigation_msg as nav_mod  # noqa: E402
import recorder  # noqa: E402


class TestLifecycle(unittest.TestCase):
    def test_event_window_and_metadata(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        data_dir = Path(tmp.name) / "events"
        rec = recorder.Recorder(ros, cfg, data_dir)
        rec.wire()

        t = 0.0
        while t < 210.0 - 1e-9:  # 210s:清除(~110s)+90s 后窗落在时间线内,事件能关闭
            es = 1 if 100.0 <= t < 110.0 else 0
            rec._on_can(can_mod.can_msg(vehicleSpeed=1.0, curGear=4,
                                        controlPanelState=1, emergencyStop=es))
            rec._on_navigation(nav_mod.navigation_msg(xAxis=1.0, yAxis=2.0))
            rec.tick(t)
            t += 0.1

        events = [d for d in data_dir.iterdir() if d.is_dir()]
        self.assertEqual(len(events), 1)
        ev = events[0]
        meta = json.loads((ev / "metadata.json").read_text("utf-8"))
        self.assertTrue(meta["complete"])
        self.assertEqual(len(meta["trigger_segments"]), 1)
        self.assertAlmostEqual(meta["trigger_segments"][0]["start"], 100.5, delta=0.2)
        records = [json.loads(line) for line in
                   (ev / "records.jsonl").read_text("utf-8").strip().split("\n")]
        first, last = records[0]["ts"], records[-1]["ts"]
        trigger_start = meta["trigger_segments"][0]["start"]
        trigger_clear = meta["trigger_segments"][0]["end"]
        # 需求:事发前至少 90s,清除后至少 90s
        self.assertLessEqual(first, trigger_start - 90.0 + 1e-6)
        self.assertGreaterEqual(last, trigger_clear + 90.0 - 1e-6)
        # 七类字段在记录里齐
        for key in ("control", "position", "motion", "perception", "response",
                    "lights", "video", "fault"):
            self.assertIn(key, records[0])
        # 关闭后状态机回 IDLE
        self.assertEqual(rec.get_status()["state"], "IDLE")

    def test_status_shape(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        rec = recorder.Recorder(ros, cfg, Path(tmp.name) / "events")
        rec.wire()
        rec._on_can(can_mod.can_msg(vehicleSpeed=0.5, curGear=4))
        rec.tick(500.0)
        st = rec.get_status()
        for key in ("ts", "vehicle_id", "snapshot", "health", "trigger", "ring", "disk"):
            self.assertIn(key, st)
        self.assertIn("can", st["health"])
        self.assertEqual(st["vehicle_id"], "A03")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 运行**

Run: `cd log_online && python3 tests/test_lifecycle.py`
Expected: OK。若 FAIL：按断言定位（首条时间→Task 6 `since` 计算；末条→close 拍先 append 的顺序；segments→Task 4）。

- [ ] **Step 3: 全量回归**

Run: `cd log_online && for f in tests/test_*.py; do python3 "$f" || exit 1; done`
Expected: 全部 OK

---

### Task 8: 状态页（status_server + index.html）

**Files:**
- Create: `log_online/status_server.py`
- Create: `log_online/static/index.html`
- Test: `log_online/tests/test_status.py`

**Interfaces:**
- Consumes: `Recorder.get_status() -> dict`、`Recorder.recent_events(limit) -> list[dict]`（注入为回调）。
- Produces: `class StatusServer(status_fn, events_fn, static_dir, host, port)`：`start() -> bool`（daemon 线程 `serve_forever`，绑定失败 stderr 告警返回 False）、`stop()`。路由：`GET /` → index.html；`GET /api/status` → `status_fn()` JSON；`GET /api/events?limit=N` → `{"events": events_fn(N)}`。

- [ ] **Step 1: 写失败测试** `tests/test_status.py`

```python
# -*- coding: utf-8 -*-
import json
import sys
import tempfile
import threading
import unittest
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from status_server import StatusServer  # noqa: E402


class TestStatusServer(unittest.TestCase):
    def test_endpoints(self):
        static_dir = Path(__file__).resolve().parents[1] / "static"
        fake_status = {"ts": 1.0, "vehicle_id": "A03", "snapshot": {}, "health": {},
                       "trigger": {"conditions": [], "state": "IDLE"},
                       "ring": {"count": 0}, "disk": {"used_bytes": 0}}
        fake_events = [{"dir": "20260916_000000_emergencystop", "complete": True}]

        srv = StatusServer(lambda: fake_status, lambda limit: fake_events[:limit],
                           static_dir, "127.0.0.1", 0)
        self.assertTrue(srv.start())
        self.addCleanup(srv.stop)
        port = srv.port

        with urllib.request.urlopen("http://127.0.0.1:%d/api/status" % port, timeout=3) as r:
            self.assertEqual(json.loads(r.read().decode("utf-8"))["vehicle_id"], "A03")
        with urllib.request.urlopen(
                "http://127.0.0.1:%d/api/events?limit=5" % port, timeout=3) as r:
            data = json.loads(r.read().decode("utf-8"))
            self.assertEqual(len(data["events"]), 1)
        with urllib.request.urlopen("http://127.0.0.1:%d/" % port, timeout=3) as r:
            body = r.read().decode("utf-8")
            self.assertIn("log_online", body)
            for anchor in ("id=\"live\"", "id=\"health\"", "id=\"events\""):
                self.assertIn(anchor, body)

    def test_port_conflict_returns_false(self):
        import socket
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        s.listen(1)
        port = s.getsockname()[1]
        srv = StatusServer(lambda: {}, lambda limit: [], Path("."), "127.0.0.1", port)
        self.assertFalse(srv.start())
        srv.stop()
        s.close()


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 运行确认失败**

Run: `cd log_online && python3 tests/test_status.py`
Expected: FAIL（ModuleNotFoundError: status_server）

- [ ] **Step 3: 实现 `status_server.py`**

```python
# -*- coding: utf-8 -*-
"""log_online 状态页:只读展示,不做控制。内网使用,严禁公网映射。"""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


class StatusServer(object):
    def __init__(self, status_fn, events_fn, static_dir, host, port):
        self._status_fn = status_fn
        self._events_fn = events_fn
        self._static_dir = Path(static_dir)
        self._host = host
        self._port = int(port)
        self._server = None
        self._thread = None

    @property
    def port(self):
        return self._server.server_address[1] if self._server else self._port

    def start(self):
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path == "/":
                    outer._serve_file(self, "index.html", "text/html; charset=utf-8")
                elif parsed.path == "/api/status":
                    outer._serve_json(self, outer._status_fn())
                elif parsed.path == "/api/events":
                    query = parse_qs(parsed.query)
                    limit = 20
                    if "limit" in query:
                        try:
                            limit = max(1, min(200, int(query["limit"][0])))
                        except ValueError:
                            pass
                    outer._serve_json(self, {"events": outer._events_fn(limit)})
                else:
                    self.send_error(404)

            def log_message(self, fmt, *args):
                pass  # 轮询噪声不刷日志

        try:
            self._server = ThreadingHTTPServer((self._host, self._port), Handler)
        except OSError as exc:
            print("[log_online] 状态页绑定失败(%s:%s): %s -- 记录不受影响"
                  % (self._host, self._port, exc))
            return False
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        daemon=True, name="log_online_http")
        self._thread.start()
        return True

    def stop(self):
        if self._server is not None:
            self._server.shutdown()
            self._server = None

    def _serve_json(self, handler, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        handler.send_response(200)
        handler.send_header("Content-Type", "application/json; charset=utf-8")
        handler.send_header("Content-Length", str(len(body)))
        handler.end_headers()
        handler.wfile.write(body)

    def _serve_file(self, handler, name, content_type):
        path = self._static_dir / name
        if not path.is_file():
            handler.send_error(404)
            return
        body = path.read_bytes()
        handler.send_response(200)
        handler.send_header("Content-Type", content_type)
        handler.send_header("Content-Length", str(len(body)))
        handler.end_headers()
        handler.wfile.write(body)
```

（Python 3.8 无 `ThreadingHTTPServer` 于 `http.server`？有——3.7 起提供。）

- [ ] **Step 4: 实现 `static/index.html`**

```html
<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<title>log_online 在线监控</title>
<style>
body{font-family:sans-serif;margin:16px;background:#f4f5f7;color:#222}
h2{border-bottom:2px solid #3572b0;padding-bottom:4px}
table{border-collapse:collapse;background:#fff;min-width:560px}
td,th{border:1px solid #ccd;padding:4px 10px;font-size:14px}
td.k{background:#eef2f7;white-space:nowrap}
.ok{color:#1a7f37}.bad{color:#c62828}.warn{color:#b26a00}
#events td{font-family:monospace}
.note{color:#777;font-size:12px}
</style>
</head>
<body>
<h2>log_online 在线监控 <span id="veh" class="note"></span></h2>

<h3>链路健康</h3>
<div id="health">加载中…</div>

<h3>实时数据</h3>
<div id="live">加载中…</div>

<h3>最近事件</h3>
<table id="events"><thead><tr>
<th>事件目录</th><th>触发段</th><th>记录区间</th><th>complete</th><th>大小</th>
</tr></thead><tbody></tbody></table>
<p class="note">仅内网使用,严禁公网映射。灯光/挡位文案按 path_plan 语义显示,灯光左右方向待实车核对。</p>

<script>
var GEAR={2:"N",3:"R",4:"D"};
var LIGHT={0:"关",3:"右转向",4:"左转向",5:"倒车",7:"报警",8:"安全停车"};

function esc(s){return String(s).replace(/&/g,"&amp;").replace(/</g,"&lt;");}

function fmtHealth(h){
  var out="<table>";
  for(var k in h){
    var v=h[k];
    var cls=v.stale?"bad":"ok";
    var hz=v.hz_est===null||v.hz_est===undefined?"-":v.hz_est+"Hz";
    out+="<tr><td class=\"k\">"+esc(k)+"</td><td class=\""+cls+"\">"+
         (v.stale?"离线":"在线")+"</td><td>"+v.last_seen_age_s+"s 前</td><td>"+hz+"</td></tr>";
  }
  return out+"</table>";
}

function row(k,v){return "<tr><td class=\"k\">"+k+"</td><td>"+esc(v)+"</td></tr>";}

function fmtLive(s){
  var m=s.motion||{},p=s.position||{},c=s.control||{},r=s.response||{},
      l=s.lights||{},v=s.video||{},f=s.fault||{},pe=s.perception||{};
  var out="<table>";
  out+=row("控制模式",c.controlPanelState===null?"-":("panel="+c.controlPanelState));
  out+=row("位置",p.x===null?"-":("("+p.x.toFixed(1)+", "+p.y.toFixed(1)+") heading "+p.heading.toFixed(0)+"° "+p.rtkState));
  out+=row("方向/速度/加速度",m.gear===null?"-":(GEAR[m.gear]||m.gear)+" 挡, "+
        (m.speed===null?"-":m.speed.toFixed(2))+" m/s, "+
        (m.accel===null?"-":m.accel.toFixed(2))+" m/s²");
  out+=row("感知",pe.obj_count===null?"-":(pe.obj_count+" 个障碍, 最近 "+
        (pe.nearest_dist===null?"-":pe.nearest_dist)+" m"));
  out+=row("响应","safety="+r.safety+" 期望速度 "+r.desireSpeed+" 油门 "+r.throttle+
        " 刹车 "+r.brake+" 转角 "+r.wheelAngle);
  out+=row("灯光/信号","light="+l.light+"("+(LIGHT[l.light]||"?")+") horn="+l.horn);
  out+=row("视频",v.online===null?"未配置":(v.online?"在线 "+v.fps+"fps":
        "离线 "+(v.last_frame_age_ms===null?"":"("+v.last_frame_age_ms+"ms 前)")));
  out+=row("故障","emergencyStop="+f.emergencyStop+" sensorstate="+r.sensorstate+
        " netcheck="+f.netcheck+" faultCode="+(f.faultCode_hex||""));
  return out+"</table>";
}

function fmtTime(ts){return ts===null?"-":new Date(ts*1000).toLocaleString();}

function refresh(){
  fetch("/api/status").then(function(r){return r.json();}).then(function(st){
    document.getElementById("veh").textContent="车辆 "+st.vehicle_id+
      " ｜ 状态机 "+st.trigger.state+" ｜ 缓冲 "+st.ring.count+" 条 / "+st.ring.span_s+"s"+
      " ｜ 磁盘 "+(st.disk.used_bytes/1048576).toFixed(1)+"/"+(st.disk.cap_bytes/1048576).toFixed(0)+"MB";
    document.getElementById("health").innerHTML=fmtHealth(st.health);
    document.getElementById("live").innerHTML=fmtLive(st.snapshot);
  }).catch(function(){});
  fetch("/api/events?limit=20").then(function(r){return r.json();}).then(function(d){
    var tb=document.querySelector("#events tbody");
    tb.innerHTML="";
    (d.events||[]).forEach(function(e){
      var tr=document.createElement("tr");
      var segs=(e.trigger_segments||[]).map(function(s){
        return s.types.join("+")+" ["+fmtTime(s.start)+"~"+fmtTime(s.end)+"]";
      }).join("; ");
      var size=e.stats&&e.stats.bytes?(e.stats.bytes/1048576).toFixed(2)+"MB":"-";
      tr.innerHTML="<td>"+esc(e.dir)+"</td><td>"+esc(segs)+"</td><td>"+
        fmtTime(e.record_start)+" ~ "+fmtTime(e.record_end)+"</td><td class=\""+
        (e.complete?"ok":"warn")+"\">"+e.complete+"</td><td>"+size+"</td>";
      tb.appendChild(tr);
    });
  }).catch(function(){});
}
refresh();
setInterval(refresh,2000);
</script>
</body>
</html>
```

- [ ] **Step 5: 运行确认通过**

Run: `cd log_online && python3 tests/test_status.py`
Expected: OK (2 tests)

---

### Task 9: 入口 / 启动脚本 / 冒烟 / README

**Files:**
- Modify: `log_online/recorder.py`（`main()` 接入状态页）
- Create: `log_online/log_online.sh`
- Create: `log_online/tests/run_smoke.py`
- Create: `log_online/README.md`

**Interfaces:**
- Consumes: Task 8 `StatusServer`；env `LOG_ONLINE_PORT`。
- Produces: 可执行入口 `python3 recorder.py` / `bash log_online.sh`；`tests/run_smoke.py`（模拟 400s 含两次触发段与一次窗口扩展，校验扩展语义）。

- [ ] **Step 1: 写冒烟脚本** `tests/run_smoke.py`

```python
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
```

- [ ] **Step 2: 运行（此时应已通过——它只依赖 Task 2-6 已实现单元）**

Run: `cd log_online && python3 tests/run_smoke.py`
Expected: `SMOKE PASS: 1 event, 2 segments, post window >= 90s`（若失败回 Task 4/6 修）

- [ ] **Step 3: `recorder.py` 的 `main()` 接入状态页（替换 Task 6 的最小版）**

```python
def main():
    port = int(os.environ.get("LOG_ONLINE_PORT", str(default_config.HTTP_PORT)))
    rec = Recorder(rospy)
    server = status_server.StatusServer(
        rec.get_status, rec.recent_events,
        Path(__file__).resolve().parent / "static",
        default_config.HTTP_HOST, port)
    server.start()
    rec.run()
```

并在 `recorder.py` 顶部 import 区加：

```python
import status_server
```

- [ ] **Step 4: 创建 `log_online.sh`**

```bash
#!/bin/bash
# log_online 启动包装(对标 monitor/monitor.sh)
# 用法: bash log_online/log_online.sh   (默认 0.0.0.0:8083,环境变量 LOG_ONLINE_PORT 可改)
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$DIR")"

if [ -f "$ROOT/devel/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$ROOT/devel/setup.bash"
fi

cd "$DIR" || exit 1
exec python3 recorder.py
```

`chmod +x log_online.sh`。

- [ ] **Step 5: 写 `README.md`**

内容必须包含（实施时展开成文）：模块职责一段（引用 DESIGN.md）；运行方式（log_online.sh / LOG_ONLINE_PORT）；**数据字段七类映射表**（从 DESIGN §4 抄）；事件目录结构示例（metadata.json + records.jsonl 各一行的样例）；轮转策略（1GiB/0.9）；**纪律：状态页仅内网，严禁公网映射**；车载部署（同步整目录、相机话题 `rostopic list | grep compressed` 核对 `TOPIC_CAMERA`、手动接入 start_l4.sh/HMI 的建议命令行、验收四条：触发一次急停→事件目录生成且前后窗 ≥90s、metadata 七类可查、状态页与 rostopic echo 一致、轮转生效）；本机测试（`for f in tests/test_*.py; do python3 $f; done` 与 `python3 tests/run_smoke.py`）。

- [ ] **Step 6: 全量回归 + 冒烟**

Run: `cd log_online && for f in tests/test_*.py; do python3 "$f" || exit 1; done && python3 tests/run_smoke.py`
Expected: 全部 OK + SMOKE PASS

---

### Task 10: 收尾（回归 / 快照 / workflow 记录）

**Files:**
- Modify: `qingwei-L4-No2-pudong/workflow.md`（现状 + 日志条目）
- Modify: `log_online/DESIGN.md`（状态行改为"已实施"）

- [ ] **Step 1: 全量回归（Python 3.8 语法自查）**

Run: `cd log_online && python3 -m py_compile recorder.py status_server.py log_online_config.py tests/*.py && for f in tests/test_*.py; do python3 "$f" || exit 1; done && python3 tests/run_smoke.py`
Expected: 编译零错 + 全部 OK + SMOKE PASS。

- [ ] **Step 2: 模块快照**

Run: `cd /home/mothotob/work/projects/claude-code/qingwei-L4-No2-pudong && tar -czf ../log_online_delivered_$(date +%Y%m%d_%H%M%S).tar.gz log_online/`
（新模块首版留档，后续改动前再按惯例打 before 快照。）

- [ ] **Step 3: workflow.md 记录**

「现状」加一条（09-16 log_online 新增：职责/触发条件/窗口/轮转/端口/快照名）；「日志」加 09-16 结论级条目（用户需求原文要点、四项用户决策——触发仅急停故障类/视频只记状态/磁盘轮转/后台+状态页、验证摘要=各测试计数、车载部署清单含相机话题核对与保护清单未动说明）。

- [ ] **Step 4: DESIGN.md 状态行更新**

`状态：待用户审阅` → `状态：已实施（2026-09-16）`。

---

## 计划自审记录

- **Spec 覆盖**：spec §2 范围四项决策→T6/T8/T5/T9；§4 映射表→T2；§5 schema→T2；§6 缓冲→T3；§7 状态机→T3/T4；§8 落盘→T5；§9 轮转→T5；§10 状态页→T8；§11 配置→T1；§12 降级→T2/T5/T8 测试用例；§13 测试 12 条→T2(1,8,9,10)/T3(2,3)/T4(5,6)/T5(7)/T6/T7(4)/T8(11)/T9 冒烟(12)；§14 部署→T9 README + T10；§15 边界→README。无缺口。
- **占位符**：无 TBD/TODO；三处易错点（hex 空值、nearest 参照系、evict 断言）已直接写成正确代码。
- **类型一致性**：`Recorder._on_can/_on_navigation` 等回调名在 T6 harness 与 T7/T9 中直接调用，签名一致；`EventWriter._dir_name` 在 T5 测试内访问（同模块白盒）；`AccelEstimator.ema_value()` 在 T6 Step 3 补充说明；`status_server.StatusServer.port` 属性 T8 测试用到（port=0 时取实际端口）。
