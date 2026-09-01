# -*- coding: utf-8 -*-
"""
ROS 桥:健康检查(话题频率/节点数/master)+ 车辆状态面板数据聚合。

- rospy 可选导入:本机无 ROS 时由 hmi_server 改用 process_manager.HealthProvider
- 健康判定全部用缓存值(check() 是纯内存操作,不产生网络调用,可被每秒轮询)
  master 探活 / 节点统计用原生 xmlrpc 直连 ROS_MASTER_URI(带超时,不依赖 rosgraph)
- 健康计数一律 AnyMsg(不依赖 devel 生成的消息包);仅车辆面板字段用类型化订阅,
  消息包导入失败只影响对应面板字段(记录在 msg_import_errors),不影响健康判定
- 点云类话题(配置 sampled=True)间歇采样:每周期订阅→收首条→退订,避免常年在
  Python 层搬运每秒几十 MB 的点云
- master 重启后 rospy 不会自动重连:检测 master pid 变化时重建全部订阅
"""

import os
import threading
import time
import xmlrpc.client
from collections import deque

try:
    import rospy
    ROS_AVAILABLE = True
except ImportError:
    rospy = None
    ROS_AVAILABLE = False

from process_manager import HealthProvider

# 车辆面板的类型化订阅表:(话题, 模块, 类名, 处理函数名)
TYPED_SUBS = [
    ("/can_msg",                 "canbus.msg",   "can_msg",        "_on_can_msg"),
    ("/navigation_msg",          "robot.msg",    "navigation_msg", "_on_navigation"),
    ("/path_plan_status",        "robot.msg",    "path_plan_status", "_on_path_status"),
    ("/control_msg",             "robot.msg",    "control_msg",    "_on_control"),
    ("/task_plan_msg",           "robot.msg",    "task_plan_msg",  "_on_task_plan"),
    ("/cloud/task/task_status",  "robot.msg",    "TaskStatus",     "_on_task_status"),
    ("/v2nHeartBeat",            "robot.msg",    "v2nHeartBeat",   "_on_heartbeat"),
    ("/hook_position",           "auto_couple.msg", "center_position", "_on_hook"),
]

# 面板展示字段对应的话题(用于数据新鲜度/灰化显示)
PANEL_TOPICS = [t for t, _, _, _ in TYPED_SUBS]

GEAR_MAP = {1: "P", 2: "N", 3: "R", 4: "D"}
# v2nHeartBeatValue.msg 注释:4-已挂钩/3-未挂钩/2-挂钩异常/1-脱钩告警
HOOK_TEXT_V2N = {0: "无", 1: "脱钩告警", 2: "挂钩异常", 3: "未挂钩", 4: "已挂钩"}
TASK_EXEC_TEXT = {0: "无任务", 1: "执行中", 2: "完成"}
DRIVING_TEXT = {0: "停车", 1: "减速", 2: "加速", 3: "巡航"}   # v2nHeartBeat.drivingState


def _to_int(x):
    """rosparam 规范化:写入方均为 int,防御字符串形态("0"/"")造成误判或位运算异常。"""
    if x is None:
        return None
    try:
        return int(x)
    except (TypeError, ValueError):
        return None


def _g(obj, attr, default=None):
    return getattr(obj, attr, default) if obj is not None else default


def _file_age(path):
    try:
        return time.time() - os.path.getmtime(path)
    except OSError:
        return None


class _TimeoutTransport(xmlrpc.client.Transport):
    """带超时的 xmlrpc 传输(Transport 构造函数不收 timeout,需在连接上设)。"""

    def __init__(self, timeout=2.5):
        xmlrpc.client.Transport.__init__(self)
        self._timeout = timeout

    def make_connection(self, host):
        conn = xmlrpc.client.Transport.make_connection(self, host)
        conn.timeout = self._timeout
        return conn


class RosBridge(HealthProvider):

    def __init__(self, health_specs, sample_period=30):
        HealthProvider.__init__(self)
        self._lk = threading.RLock()

        # 健康规格归类
        self._cont = {}      # topic -> min_hz(连续计数)
        self._sampled = {}   # topic -> dict(min_hz, period)
        for sp in health_specs or []:
            if sp.get("type", "topic") != "topic":
                continue
            topic = sp["topic"]
            if sp.get("sampled"):
                self._sampled[topic] = {"min_hz": sp.get("min_hz", 1),
                                        "period": sp.get("sample_period", sample_period)}
            else:
                self._cont[topic] = sp.get("min_hz", 1)

        # 计数与新鲜度
        all_topics = list(self._cont) + list(self._sampled) + PANEL_TOPICS
        self._count = {t: 0 for t in all_topics}
        self._last = {t: None for t in all_topics}
        self._samples = {t: deque(maxlen=4) for t in self._cont}
        self._rates = {t: None for t in self._cont}

        # master / 节点统计缓存
        self.master_ok = None
        self._master_pid = None
        self._master_fail_cnt = 0
        self._node_prefix_counts = {}   # pattern -> count

        # 订阅句柄
        self._subs = []                 # 连续订阅
        self._probe_subs = {}           # topic -> Subscriber(采样)
        self._probe_next = {}           # topic -> monotonic(下次探测时间)
        self._probe_close = set()       # 待退订

        self._node_ready = False
        self._msg_import_errors = {}
        self._typed_ok = {}             # topic -> bool

        # 面板数据缓存(回调只写这里,vehicle_state() 读)
        self._veh = {}
        self._veh_default()

        self._typed_cache = {}          # (module, class) -> class 对象

    # ================= 对外接口 =================

    def ros_available(self):
        return True

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def vehicle_state(self):
        with self._lk:
            veh = dict(self._veh)
            veh["ros_available"] = True
            veh["master_ok"] = self.master_ok
            # 类型化解析可用性(AnyMsg 兜底时字段解析不可用,前端据此降级显示)
            veh["typed_ok"] = dict(self._typed_ok)
            now = time.monotonic()
            veh["ages"] = {t: (round(now - l, 1) if l is not None else None)
                           for t, l in self._last.items() if t in PANEL_TOPICS}
            return veh

    def msg_import_errors(self):
        with self._lk:
            return dict(self._msg_import_errors)

    def check(self, specs):
        """纯内存判定,返回 (ok, detail)。"""
        ok, parts = True, []
        for sp in specs:
            st = sp.get("type", "topic")
            if st == "file":
                age = self._file_age(sp["path"])
                if age is None or age > sp["max_age"]:
                    ok = False
                    parts.append("%s 心跳超时" % os.path.basename(sp["path"]))
            elif st == "master":
                if not self.master_ok:
                    ok = False
                    parts.append("master 失联")
                else:
                    parts.append("master 正常")
            elif st == "nodes":
                cnt = self._node_prefix_counts.get(sp["pattern"], 0)
                if cnt < sp["min"]:
                    ok = False
                    parts.append("%s* 节点 %d/%d" % (sp["pattern"], cnt, sp["min"]))
                else:
                    parts.append("%s* 节点 %d" % (sp["pattern"], cnt))
            else:
                topic = sp["topic"]
                if topic in self._sampled:
                    info = self._sampled[topic]
                    last = self._last.get(topic)
                    limit = info["period"] * 2
                    if last is None or time.monotonic() - last > limit:
                        ok = False
                        parts.append("%s 采样超时" % topic)
                    else:
                        parts.append("%s 采样%.0fs前" % (topic, time.monotonic() - last))
                else:
                    hz = self._rates.get(topic)
                    need = sp.get("min_hz", 1) * 0.8   # 容差,避免临界抖动
                    if hz is None or hz < need:
                        ok = False
                        parts.append("%s %.1fHz" % (topic, hz if hz is not None else 0.0))
                    else:
                        parts.append("%s %.1fHz" % (topic, hz))
        return ok, " / ".join(parts) if parts else "仅进程监控"

    # ================= 后台主循环 =================

    def _run(self):
        # 阶段一:等 init_node 成功(依赖 master 已启动)
        while not self._node_ready:
            try:
                rospy.init_node("hmi_monitor", disable_signals=True,
                                anonymous=True, log_level=rospy.ERROR)
            except Exception:
                # init 可能半途注册成功后抛错:is_initialized 为真则视为就绪,
                # 否则稍后重试(重试时 rospy 会因已初始化再次抛错,同样走到这里)
                try:
                    if not rospy.is_initialized():
                        time.sleep(5.0)
                        continue
                except Exception:
                    time.sleep(5.0)
                    continue
            self._node_ready = True
        self._rebuild_subs()

        tick5 = 0.0
        while True:
            time.sleep(1.0)
            now = time.monotonic()
            with self._lk:
                # 1s:频率滑窗采样 + 采样型探测调度
                for t in self._cont:
                    self._samples[t].append((now, self._count[t]))
                    s = self._samples[t]
                    if len(s) >= 2 and (s[-1][0] - s[0][0]) > 0.5:
                        dt = s[-1][0] - s[0][0]
                        self._rates[t] = (s[-1][1] - s[0][1]) / dt
                    else:
                        self._rates[t] = None
                self._probe_schedule(now)
                # 1s:rosparam 轮询
                self._poll_params()
            if now - tick5 >= 5.0:
                tick5 = now
                self._master_check()

    def _probe_schedule(self, now):
        # 持锁调用。采样型话题:到期就订一条,收到首条后由主循环退订
        for topic, info in self._sampled.items():
            sub = self._probe_subs.get(topic)
            if sub is None and now >= self._probe_next.get(topic, 0):
                try:
                    self._probe_subs[topic] = rospy.Subscriber(
                        topic, rospy.AnyMsg, self._probe_cb, callback_args=topic,
                        queue_size=1)
                    self._probe_next[topic] = now + info["period"] * 3
                except Exception:
                    self._probe_next[topic] = now + 5.0
        for topic in list(self._probe_close):
            sub = self._probe_subs.pop(topic, None)
            if sub is not None:
                try:
                    sub.unregister()
                except Exception:
                    pass
            info = self._sampled[topic]
            self._probe_next[topic] = time.monotonic() + info["period"]
        self._probe_close.clear()

    def _probe_cb(self, _msg, topic):
        with self._lk:
            self._count[topic] = self._count.get(topic, 0) + 1
            self._last[topic] = time.monotonic()
            self._probe_close.add(topic)

    # ================= master 探活与节点统计 =================

    def _master_xmlrpc(self, method, *args):
        uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
        proxy = xmlrpc.client.ServerProxy(uri, transport=_TimeoutTransport(2.5))
        return getattr(proxy, method)(*args)

    def _master_check(self):
        try:
            code, _msg, pid = self._master_xmlrpc("getPid", "/hmi_monitor")
            if code == 1:
                pid_changed = (self._master_pid is not None
                               and pid != self._master_pid)
                self._master_pid = pid
                self._master_fail_cnt = 0
                with self._lk:
                    self.master_ok = True
                if pid_changed:
                    self._rebuild_subs()   # master 重启,重连全部订阅
            else:
                raise RuntimeError("master 返回异常 %s" % code)
        except Exception:
            self._master_fail_cnt += 1
            if self._master_fail_cnt >= 2:   # 连续 2 次失败才判定失联
                with self._lk:
                    self.master_ok = False
            return
        # master 正常时顺带刷新节点前缀统计(nodes 型健康检查用)
        try:
            code, _msg, state = self._master_xmlrpc("getSystemState", "/hmi_monitor")
            if code != 1:
                return
            nodes = set()
            for _topic, nds in state[0]:
                nodes.update(nds)
            for _topic, nds in state[1]:
                nodes.update(nds)
            counts = {}
            for pat in self._node_patterns():
                counts[pat] = sum(1 for n in nodes if pat in n)
            with self._lk:
                self._node_prefix_counts = counts
        except Exception:
            pass

    def _node_patterns(self):
        return ["/cloud"]   # 与 hmi_config 中 nodes 型规格保持一致;如增改配置需同步

    # ================= 订阅管理 =================

    def _load_class(self, module, cls):
        key = (module, cls)
        if key in self._typed_cache:
            return self._typed_cache[key]
        try:
            mod = __import__(module, fromlist=[cls])
            klass = getattr(mod, cls)
            self._typed_cache[key] = klass
            return klass
        except Exception as exc:
            self._msg_import_errors["%s/%s" % (module, cls)] = str(exc)
            self._typed_cache[key] = None
            return None

    def _rebuild_subs(self):
        # 全部退订后重建(master 重启 / 首次)
        for sub in self._subs:
            try:
                sub.unregister()
            except Exception:
                pass
        self._subs = []
        for topic, sub in list(self._probe_subs.items()):
            try:
                sub.unregister()
            except Exception:
                pass
            self._probe_subs[topic] = None
            self._probe_next[topic] = 0.0

        for topic, module, cls, handler in TYPED_SUBS:
            klass = self._load_class(module, cls)
            try:
                if klass is not None:
                    self._subs.append(rospy.Subscriber(
                        topic, klass, getattr(self, handler),
                        queue_size=1))
                    self._typed_ok[topic] = True
                    continue
                self._typed_ok[topic] = False
                self._subs.append(rospy.Subscriber(
                    topic, rospy.AnyMsg, self._generic_cb,
                    callback_args=topic, queue_size=1))
            except Exception:
                pass
        # 仅出现在健康规格里、面板表没有的连续话题,AnyMsg 计数
        typed_topics = set(t for t, _, _, _ in TYPED_SUBS)
        for topic in self._cont:
            if topic not in typed_topics:
                try:
                    self._subs.append(rospy.Subscriber(
                        topic, rospy.AnyMsg, self._generic_cb,
                        callback_args=topic, queue_size=1))
                except Exception:
                    pass

    def _generic_cb(self, _msg, topic):
        self._bump(topic)

    def _bump(self, topic):
        with self._lk:
            self._count[topic] = self._count.get(topic, 0) + 1
            self._last[topic] = time.monotonic()

    # ================= rosparam 轮询 =================

    def _poll_params(self):
        # 持锁调用;master 不可达时全部置 None(前端显示"未知")
        keys = {
            "sensorstate": "/planning/sensorstate",
            "alive": "/planning/alive",
            "netcheck": "/robot/planning/netcheck",
            "alarm": "/alarmcmd",
        }
        vals = {}
        for k, roskey in keys.items():
            try:
                vals[k] = rospy.get_param(roskey, None)
            except Exception:
                vals[k] = None
        self._veh["params"] = vals

    # ================= 面板字段处理器 =================

    def _veh_default(self):
        self._veh = {
            "speed_kmh": None, "speed_ms": None, "gear": None, "gear_raw": None,
            "mode": None, "emergency_stop": None,
            "driving_state": None, "driving_state_text": None,
            "task": {"status": None, "status_text": None, "task_id": None,
                     "type": None, "stop_xy_m": None, "procedure": None,
                     "fail_text": None},
            "battery_pct": None,
            "localization": {"x_m": None, "y_m": None, "heading_deg": None,
                             "rtk_raw": None, "rtk_text": None},
            "obstacle": {"raw": None, "text": None, "level": None},
            "distance_to_stop_m": None,
            "lateral_dev_m": None, "lateral_dev_warn": None,
            "hook": {"text": None, "state": None, "v2n": None,
                     "center_distance": None, "beta": None},
            "can": {"hz": None, "fault_codes": [], "fault": None},
            "sensors": {"sensorstate": None, "fault": None, "lidar": None,
                        "camera": None, "gnss": None, "alive": None,
                        "vehicle_ok": None},
            "net": {"internet_ok": None, "alarm": None},
            "status_str": None,
            "params": {},
        }

    def _on_can_msg(self, msg):
        self._bump("/can_msg")
        with self._lk:
            v = self._veh
            spd = _g(msg, "vehicleSpeed")
            v["speed_ms"] = spd     # 原始 m/s(取整会掩盖 0.36~0.45 km/h 区间)
            v["speed_kmh"] = round(spd * 3.6, 1) if spd is not None else None
            gear = _g(msg, "curGear")
            v["gear_raw"] = gear
            v["gear"] = GEAR_MAP.get(gear, str(gear) if gear is not None else None)
            mode = _g(msg, "controlPanelState")
            v["mode"] = {1: "自动", 0: "手动"}.get(mode,
                                                str(mode) if mode is not None else None)
            v["emergency_stop"] = bool(_g(msg, "emergencyStop", 0))
            fc = list(_g(msg, "faultCode", []) or [])
            v["can"]["fault_codes"] = fc
            v["can"]["fault"] = any(fc)
            # 挂接状态兜底:v2nHeartBeat 未到时用 can_msg.hookState(1=挂载 0=未挂载)
            if v["hook"]["v2n"] is None:
                hs = _g(msg, "hookState")
                if hs is not None:
                    v["hook"]["text"] = "已挂载" if hs == 1 else "未挂载"

    def _on_navigation(self, msg):
        self._bump("/navigation_msg")
        with self._lk:
            loc = self._veh["localization"]
            loc["x_m"] = _g(msg, "xAxis")
            loc["y_m"] = _g(msg, "yAxis")
            loc["heading_deg"] = _g(msg, "heading")
            st = _g(msg, "localization_status")
            loc["rtk_raw"] = st
            loc["rtk_text"] = {0: "RTK固定", 2: "异常"}.get(
                st, str(st) if st is not None else None)

    def _on_path_status(self, msg):
        self._bump("/path_plan_status")
        with self._lk:
            tes = _g(msg, "taskExecuStatus")
            self._veh["task"]["status"] = tes
            self._veh["task"]["status_text"] = TASK_EXEC_TEXT.get(
                tes, str(tes) if tes is not None else None)
            d2o = _g(msg, "distance2Object")
            self._veh["obstacle"]["raw"] = d2o
            if d2o is None:
                self._veh["obstacle"]["text"] = None
                self._veh["obstacle"]["level"] = None
            elif d2o >= 200:
                self._veh["obstacle"]["text"] = "有风险(非前方)"
                self._veh["obstacle"]["level"] = "warn"
            elif d2o >= 100:
                self._veh["obstacle"]["text"] = "无风险"
                self._veh["obstacle"]["level"] = "ok"
            else:
                self._veh["obstacle"]["text"] = "%.1f m" % d2o
                self._veh["obstacle"]["level"] = ("danger" if d2o < 6.5
                                                  else "warn" if d2o < 15 else "ok")
            d2s = _g(msg, "distance2Stop")
            self._veh["distance_to_stop_m"] = d2s if d2s is None else round(d2s, 1)

    def _on_control(self, msg):
        self._bump("/control_msg")
        with self._lk:
            bia = _g(msg, "biaDistance")
            self._veh["lateral_dev_m"] = bia if bia is None else round(bia, 2)
            if bia is not None:
                # biaDistance 带符号(叉积定向, control_comply.cpp:596-603), 负值=另一侧偏差
                self._veh["lateral_dev_warn"] = abs(bia) > 5.5

    def _on_task_plan(self, msg):
        self._bump("/task_plan_msg")
        with self._lk:
            t = self._veh["task"]
            t["task_id"] = _g(msg, "task_id")
            t["type"] = _g(msg, "taskType")
            sx, sy = _g(msg, "stopX"), _g(msg, "stopY")
            t["stop_xy_m"] = [round(sx, 1), round(sy, 1)] if sx is not None and sy is not None else None

    def _on_task_status(self, msg):
        self._bump("/cloud/task/task_status")
        with self._lk:
            t = self._veh["task"]
            t["procedure"] = _g(msg, "procedure")
            inner = _g(msg, "task_info")
            if _g(inner, "task_id") is not None:
                t["task_id"] = _g(inner, "task_id")
            fcode, freason = _g(msg, "fail_code"), _g(msg, "fail_reason")
            if fcode:
                t["fail_text"] = "%s(%s)" % (freason or "失败", fcode)
            else:
                t["fail_text"] = None

    def _on_heartbeat(self, msg):
        self._bump("/v2nHeartBeat")
        with self._lk:
            v = self._veh
            vals = _g(msg, "values")
            soc = _g(vals, "soc")
            v["battery_pct"] = soc
            # status 真实消息定义在 values 内(v2nHeartBeat 顶层仅 ts/deviceId/type/values)
            v["status_str"] = _g(msg, "status") or _g(vals, "status")
            hook = _g(vals, "hookState")
            v["hook"]["v2n"] = hook
            v["hook"]["state"] = hook
            v["hook"]["text"] = HOOK_TEXT_V2N.get(
                hook, str(hook) if hook is not None else None)
            ds = _g(vals, "drivingState")
            v["driving_state"] = ds
            v["driving_state_text"] = DRIVING_TEXT.get(
                ds, str(ds) if ds is not None else None)
            s = v["sensors"]
            s["vehicle_ok"] = not bool(_g(vals, "vehicleState", 0))
            for name in ("lidar", "camera", "gnss"):
                st = _g(vals, name + "State")
                if st is not None:
                    s[name] = not bool(st)
            if v["speed_kmh"] is None:
                sp = _g(vals, "speed")
                if sp is not None:
                    v["speed_kmh"] = round(sp, 1)   # v2n 的 speed 已是 km/h

    def _on_hook(self, msg):
        self._bump("/hook_position")
        with self._lk:
            h = self._veh["hook"]
            cd = _g(msg, "center_distance")
            h["center_distance"] = cd if cd is None else round(cd, 2)
            beta = _g(msg, "beta")
            h["beta"] = beta if beta is None else round(beta, 1)

    def vehicle_state_final(self):
        """聚合 params 到 sensors/net 后的完整状态(hmi_server 调用)。"""
        veh = self.vehicle_state()
        p = veh.get("params") or {}
        ss = _to_int(p.get("sensorstate"))
        with self._lk:
            s = self._veh["sensors"]
            # v2nHeartBeat 停更(>5s)时清 v2n 来源键,让 sensorstate 位图兜底接管:
            # 两者来自不同节点(task_plan vs path_plan),一方停更时不能粘住旧值
            lt = self._last.get("/v2nHeartBeat")
            if lt is None or time.monotonic() - lt > 5:
                for key in ("lidar", "camera", "gnss", "vehicle_ok"):
                    if s.get(key) is not None:
                        s[key] = None
            if ss is not None:
                s["sensorstate"] = ss
                s["fault"] = bool(ss & 1)
                s["lidar"] = (not bool(ss & 2)) if s.get("lidar") is None else s["lidar"]
                s["camera"] = (not bool(ss & 4)) if s.get("camera") is None else s["camera"]
                s["gnss"] = (not bool(ss & 8)) if s.get("gnss") is None else s["gnss"]
            s["alive"] = _to_int(p.get("alive"))
            veh["sensors"] = dict(self._veh["sensors"])
            nc = _to_int(p.get("netcheck"))
            veh["net"] = {"internet_ok": None if nc is None else nc == 0,
                          "alarm": _to_int(p.get("alarm"))}
            veh["can"]["hz"] = self._rates.get("/can_msg")
        return veh
