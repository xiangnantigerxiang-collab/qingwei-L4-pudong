# -*- coding: utf-8 -*-
"""log_online 记录器:订阅缓存 -> 10Hz 快照 -> 环形缓冲 -> 事件状态机 -> 落盘/轮转。
纯逻辑单元(快照/判定/状态机)不依赖 ROS,便于本机测试。"""
import collections
import json
import os
import shutil
import sys
import threading
import time
from pathlib import Path

import rospy
# 注意:不要 import robot.msg 的 object 类——会遮蔽内建 object,使后续 class X(object)
# 继承到消息类上;objs 元素仅按属性访问,无需导入
from robot.msg import can_msg, control_msg, navigation_msg, path_plan_msg, perception
from sensor_msgs.msg import CompressedImage

import log_online_config as default_config
import status_server

SOFTWARE_VERSION = "log_online 1.0.0"


def fault_code_hex(code_bytes):
    """can_msg.faultCode(uint8[],rospy 到达为 bytes)转小写 hex 串。"""
    if not code_bytes:
        return ""
    return "".join("%02x" % int(b) for b in code_bytes)


def _sanitize(value):
    """递归把 float NaN/inf 换成 None:标准 JSON 严格解析器(浏览器 JSON.parse/jq)
    不认这三个字面量。status_server 另持一份同款(避免其反向依赖本模块)。"""
    if isinstance(value, float) and (value != value or value in (float("inf"), float("-inf"))):
        return None
    if isinstance(value, dict):
        return {k: _sanitize(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_sanitize(v) for v in value]
    return value


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
        # 部分配置(如测试桩)可缺 VEHICLE_ID,回落到默认配置
        "vehicle_id": getattr(config, "VEHICLE_ID", default_config.VEHICLE_ID),
        "control": control,
        "position": position,
        "motion": motion,
        "perception": perception_block,
        "response": response,
        "lights": lights,
        "video": video,
        "fault": fault,
    }


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
                # 新事件开新段表:close 后 IDLE 期间残留的上一事件段不得混入新事件。
                # 必须在下一次 open 时清空而非 close 时——close 的终版 metadata 要用本表
                self.segments = []
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
        # 落盘前消毒 NaN/inf -> null,records.jsonl 保持严格合法 JSON
        line = json.dumps(_sanitize(snapshot), ensure_ascii=False,
                          separators=(",", ":")) + "\n"
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
                try:
                    # 统计途中文件/目录可能被本线程 evict 并发删除,消失即不计入
                    size = sum(f.stat().st_size for f in event_dir.rglob("*") if f.is_file())
                except OSError:
                    continue
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
            shutil.rmtree(str(self._data_dir / name), ignore_errors=True)
            # 复查删除结果:目录仍在(权限/挂载等异常)则告警且不虚报水位
            if (self._data_dir / name).exists():
                sys.stderr.write("[log_online] 轮转删除失败: %s\n" % name)
                continue
            removed.append(name)
            total -= size
            print("[log_online] 轮转删除旧事件 %s (%d bytes)" % (name, size))
        return removed


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
        # 启动即轮转一次(DESIGN §9):进程重启后磁盘残留超限也能回落水位
        self._rotator.scan()
        self._rotator.evict_if_over(self.cfg.DISK_CAP_BYTES, self.cfg.ROTATE_KEEP_RATIO)

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
                writer = EventWriter(self.data_dir, self.cfg.VEHICLE_ID, self.cfg)
                try:
                    writer.open(act[1], now, pre_records,
                                self._health(last_seen, now))
                    # 初始 metadata 即含首段:事件中途被 kill 也不丢段信息
                    writer.refresh_metadata(self._machine.segments)
                except OSError as exc:
                    # 磁盘写失败:丢弃本事件不丢拍,状态机照常推进;轮转兜底缓解磁盘压力
                    self._drop_writer(exc)
                    self._rotator.evict_if_over(self.cfg.DISK_CAP_BYTES,
                                                self.cfg.ROTATE_KEEP_RATIO)
                else:
                    self._writer = writer
            elif act[0] == "extend":
                if self._writer is not None:
                    try:
                        self._writer.refresh_metadata(self._machine.segments)
                    except OSError as exc:
                        # post 窗内扩展拍写失败:与 open/close 同款善后,丢事件不丢机制;
                        # 本拍无 close 动作、追加分支 writer 已空 -> evict 恰此一次
                        self._drop_writer(exc)
                        self._rotator.evict_if_over(self.cfg.DISK_CAP_BYTES,
                                                    self.cfg.ROTATE_KEEP_RATIO)

        if ("close",) in actions:
            if self._writer is not None:
                try:
                    self._writer.append(snapshot)  # 关闭拍数据先补上,post 窗末条 >= 清除+90s
                    self._writer.close(self._machine.segments, now)
                except OSError as exc:
                    self._drop_writer(exc)
                else:
                    self._writer = None
                # 成败都复查一次磁盘水位(同拍仅此一次)
                self._rotator.evict_if_over(self.cfg.DISK_CAP_BYTES,
                                            self.cfg.ROTATE_KEEP_RATIO)
        elif self._machine.state != "IDLE" and self._writer is not None:
            try:
                self._writer.append(snapshot)
                self._writer.maybe_flush(now)
            except OSError as exc:
                # 记录中写失败:丢事件不丢机制,状态机照常走 POST->IDLE
                self._drop_writer(exc)
                self._rotator.evict_if_over(self.cfg.DISK_CAP_BYTES,
                                            self.cfg.ROTATE_KEEP_RATIO)

    def _drop_writer(self, exc):
        """磁盘写失败善后(DESIGN §12):stderr 告警并丢弃本事件,置 None 后
        各调用点按 writer 判空自然跳过;被丢弃对象随引用释放关 _fh。"""
        sys.stderr.write("[log_online] 事件落盘失败,丢弃本事件: %s\n" % exc)
        self._writer = None

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
                        "segments": [dict(s) for s in self._machine.segments]},
            "state": self._machine.state,
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
        self._ros.Timer(self._ros.Duration(1.0 / self.cfg.SAMPLE_HZ),
                        lambda _event=None: self.tick(time.time()))
        self._ros.spin()


def _resolve_port(env_get, default):
    """从环境变量解析状态页端口(env_get 取 os.environ.get 同签名)。
    未设用默认;非法数字 stderr 告警后回落默认,不拖死记录主流程。"""
    raw = env_get("LOG_ONLINE_PORT")
    if raw is None:
        return default
    try:
        return int(raw)
    except ValueError:
        sys.stderr.write("[log_online] 非法端口值 %r,回落默认 %d\n" % (raw, default))
        return default


def main():
    port = _resolve_port(os.environ.get, default_config.HTTP_PORT)
    rec = Recorder(rospy)
    server = status_server.StatusServer(
        rec.get_status, rec.recent_events,
        Path(__file__).resolve().parent / "static",
        default_config.HTTP_HOST, port)
    server.start()
    rec.run()


if __name__ == "__main__":
    main()
