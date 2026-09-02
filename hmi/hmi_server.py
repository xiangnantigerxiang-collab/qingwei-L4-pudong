# -*- coding: utf-8 -*-
"""
HMI 服务入口:配置加载、系统资源监控、HTTP API、启动编排。

用法:
  车载:  bash hmi/hmi.sh                (默认 0.0.0.0:8080,加载 hmi_config.py)
  本机:  python3 hmi_server.py --config test_config.py --port 18080

API:
  GET  /                     前端页面(no-store)
  GET  /api/state            全量快照(server/sequence/components/vehicle/
                             calibration/system)
  GET  /api/logs/<name>      日志尾部 ?tail=N(默认200,上限2000);?download=1 全文下载
  POST /api/start            一键启动(分组并行+健康门控,后台执行)
  POST /api/stop             全部停止(逆序);body 必须含 {"confirm":"STOP"}
  POST /api/calibrate        脱挂钩一键标定:前置检查(自动/N档/车速≈0/CAN 新鲜)
                             不满足返回提醒文案不触发;满足则 rosparam 触发
                             canbus 标定并后台轮询,结果经 /api/state 的
                             calibration 键暴露(前端据此弹窗)
  POST /api/components/<name>/{start,stop,restart}
"""

import argparse
import importlib.util
import json
import os
import re
import shutil
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs, unquote

import process_manager
import record_rosbag
from process_manager import ProcessManager

# rospy 可用性探测(独立于 RosBridge 实例化;本机无 ROS 时为 False)
try:
    import rospy  # noqa: F401
    ROS_OK = True
except ImportError:
    ROS_OK = False

VERSION = "1.1.0"
HMI_DIR = Path(__file__).resolve().parent
ROOT = HMI_DIR.parent
LOG_TAIL_MAX = 2000
LOG_READ_BYTES = 256 * 1024

# ---------------------------------------------------------------- 配置加载

def load_config(path):
    """加载配置模块并做校验与占位符替换($ROOT/$VARDIR)。"""
    spec = importlib.util.spec_from_file_location("hmi_cfg", str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    cfg = mod.CONFIG

    vardir = HMI_DIR / "var"
    subs = {"$ROOT": str(ROOT), "$VARDIR": str(vardir)}

    def repl(s):
        for k, v in subs.items():
            s = s.replace(k, v)
        return s

    def repl_item(x):
        if isinstance(x, str):
            return repl(x)
        if isinstance(x, list):
            return [repl_item(i) for i in x]
        if isinstance(x, dict):
            return {k: repl_item(v) for k, v in x.items()}
        return x

    names = set()
    for c in cfg["components"]:
        for key in ("name", "title", "group", "cmd", "cwd"):
            if key not in c:
                raise ValueError("组件缺少字段 %s: %r" % (key, c.get("name")))
        if c["name"] in names:
            raise ValueError("组件名重复: %s" % c["name"])
        names.add(c["name"])
        if c["group"] not in cfg["groups"]:
            raise ValueError("组件 %s 的 group %r 不在 groups 定义中" % (c["name"], c["group"]))
    cfg["components"] = [repl_item(c) for c in cfg["components"]]
    return cfg, names


# ---------------------------------------------------------------- 系统资源

class SystemMonitor(object):

    def __init__(self):
        self._lk = threading.Lock()
        self._last_stat = self._read_stat()
        self._last_t = time.monotonic()

    @staticmethod
    def _read_stat():
        with open("/proc/stat", "r") as f:
            parts = f.readline().split()[1:]
        return [int(x) for x in parts]

    def snapshot(self):
        with self._lk:
            st, t = self._read_stat(), time.monotonic()
            d_total = sum(st) - sum(self._last_stat)
            d_idle = st[3] - self._last_stat[3] + (st[4] - self._last_stat[4] if len(st) > 4 else 0)
            cpu = None if d_total <= 0 else round(100.0 * (1 - d_idle / d_total), 1)
            self._last_stat, self._last_t = st, t
        out = {"cpu_pct": cpu, "mem_pct": None, "mem_total_gb": None,
               "cpu_temp_c": None, "disk_pct": None, "disk_free_gb": None,
               "disk_warn": False, "load1": None, "sys_uptime_s": None}
        try:
            with open("/proc/meminfo", "r") as f:
                mi = {}
                for line in f:
                    k, v = line.split(":", 1)
                    mi[k] = int(v.strip().split()[0])
            total, avail = mi.get("MemTotal", 0), mi.get("MemAvailable", 0)
            if total:
                out["mem_pct"] = round(100.0 * (1 - avail / total), 1)
                out["mem_total_gb"] = round(total / 1048576.0, 1)
        except (OSError, ValueError):
            pass
        try:
            temps = []
            for z in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
                try:
                    temps.append(int(z.read_text().strip()) / 1000.0)
                except (OSError, ValueError):
                    continue
            if temps:
                out["cpu_temp_c"] = round(max(temps), 1)
        except OSError:
            pass
        try:
            du = shutil.disk_usage(str(HMI_DIR))
            out["disk_pct"] = round(100.0 * du.used / du.total, 1)
            out["disk_free_gb"] = round(du.free / (1000 ** 3), 1)
            out["disk_warn"] = du.free < 1024 ** 3
        except OSError:
            pass
        try:
            with open("/proc/loadavg", "r") as f:
                out["load1"] = float(f.read().split()[0])
        except (OSError, ValueError):
            pass
        try:
            with open("/proc/uptime", "r") as f:
                out["sys_uptime_s"] = int(float(f.read().split()[0]))
        except (OSError, ValueError):
            pass
        return out


# ---------------------------------------------------------------- 脱挂钩一键标定

# canbus 节点的标定触发参数(canbus_core StepCalibration,5Hz 轮询;
# 任何退出路径——完成/拒绝/中止——节点自己把它清回 0)
CALIB_PARAM = "/canbus/calibration/hook"
CALIB_REMINDER = "一键标定前请停车挂N档，切换到自动驾驶模式"
# > 四阶段最坏 4×20s 兜底;正常 15~30s(测试用环境变量调短)
CALIB_TIMEOUT_S = float(os.environ.get("HMI_CALIB_TIMEOUT_S", "100"))
CALIB_LOG_MAX_BYTES = 2 * 1024 * 1024   # 结果行解析的单次读取上限
_CALIB_DONE_RE = re.compile(
    r"calibration done: hook \[(\d+)\.\.(\d+)\], pallet \[(\d+)\.\.(\d+)\]")
_CALIB_FAIL_RE = re.compile(r"calibration (refused|aborted|failed)[:：]\s*(.+)")


class CalibrationManager(object):
    """脱挂钩一键标定编排(canbus 侧状态机的前端入口)。

    流程:前置检查(自动驾驶 + N 档 + 车速≈0 + CAN 数据新鲜,不满足返回
    提醒文案,不触发)→ rosparam 置 1 → 后台轮询参数清零 → 从 canbus 组件
    日志的"增量"里解析结果行(成功行带四条限值;refused/aborted/failed
    行带原因)→ 日志不可用时退回 config.cfg 触发前后值比对。

    结果挂在 /api/state 的 "calibration" 键上,前端轮询到 running →
    done/failed 跳变时弹窗;后端不主动推送。
    """

    def __init__(self, root):
        self._lk = threading.Lock()
        self._root = Path(root)
        self._phase = "idle"        # idle / running / done / failed
        self._message = None
        self._hook = None           # [min, max](成功后)
        self._pallet = None         # [min, max]
        self._t0 = None
        self._cfg_stamp = (None, None)   # (mtime, 解析结果) 缓存
        self._log_resolver = None        # ()->Path,canbus 当前日志路径(结果
                                         # 解析兜底用,HmiApp 注入)

    # ---- config.cfg 当前限值(mtime 缓存;键解析规则与 canbus 节点一致) ----

    def current_limits(self):
        path = self._root / "src" / "canbus" / "config.cfg"
        try:
            mtime = path.stat().st_mtime
        except OSError:
            return None
        if self._cfg_stamp[0] == mtime:
            return self._cfg_stamp[1]
        keys = ("hook_position_min", "hook_position_max",
                "pallet_position_min", "pallet_position_max")
        vals = {}
        try:
            for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
                p = line.split("#", 1)[0].strip()
                if "=" not in p:
                    continue
                k, v = p.split("=", 1)
                k = k.strip()
                if k in keys:
                    try:
                        vals[k] = int(v.strip())
                    except ValueError:
                        pass
        except OSError:
            return None
        out = None
        if len(vals) == 4:
            out = {"hook": [vals["hook_position_min"], vals["hook_position_max"]],
                   "pallet": [vals["pallet_position_min"], vals["pallet_position_max"]]}
        self._cfg_stamp = (mtime, out)
        return out

    # ---- 前置检查:满足返回 None,否则 (提醒文案, 当前状态描述) ----

    def precheck(self, veh):
        if veh is None or not veh.get("ros_available"):
            return CALIB_REMINDER, "无 ROS 环境,车辆状态不可用"
        ages = veh.get("ages") or {}
        age_can = ages.get("/can_msg")
        if age_can is None or age_can > 5:
            return CALIB_REMINDER, "CAN 数据超时或无数据(canbus 节点未运行?)"
        problems = []
        if veh.get("mode") != "自动":
            problems.append("驾驶模式为%s" % (veh.get("mode") or "未知"))
        if veh.get("gear") != "N":
            problems.append("档位为%s" % (veh.get("gear") or "未知"))
        spd_raw = veh.get("speed_ms")
        spd = veh.get("speed_kmh")
        if spd_raw is not None:
            if spd_raw >= 0.1:      # 与 canbus 侧门槛一致(m/s)
                problems.append("车速%s km/h" % (spd if spd is not None
                                                 else spd_raw))
        elif spd is None or spd > 0.4:
            problems.append("车速%s km/h" % ("未知" if spd is None else spd))
        if problems:
            return CALIB_REMINDER, "当前" + "、".join(problems)
        return None

    # ---- 触发 ----

    def start(self, veh, log_path):
        # 预留在同一个锁段内完成(对抗审查/red-team 实证:检查与提交之间隔着
        # set_param 的 XML-RPC,并发点击曾产生多个轮询线程,done 可被迟到的
        # 超时判定覆盖成 failed)
        with self._lk:
            if self._phase == "running":
                return 409, {"ok": False, "error": "标定进行中,请等待完成"}
            self._phase = "running"
            self._message = None
            self._hook = None
            self._pallet = None
            self._t0 = time.time()
        hit = self.precheck(veh)
        if hit is not None:
            self._reset_idle()
            return 200, {"ok": False, "reminder": hit[0], "detail": hit[1]}
        # 已有一次标定被置位(上次超时未清/外部触发)不得再叠加
        try:
            if rospy.get_param(CALIB_PARAM, 0) == 1:
                self._reset_idle()
                return 409, {"ok": False,
                             "error": "标定参数已被置位,可能有一次标定正在进行"}
        except Exception:
            pass
        before = self.current_limits()
        try:
            rospy.set_param(CALIB_PARAM, 1)
        except Exception as exc:
            self._reset_idle()
            return 409, {"ok": False, "error": "触发参数写入失败:%s" % exc}
        offset = 0
        if log_path is not None:
            try:
                offset = log_path.stat().st_size
            except OSError:
                offset = 0
        threading.Thread(target=self._run, args=(log_path, offset, before),
                         daemon=True).start()
        return 200, {"ok": True, "state": "running"}

    def _reset_idle(self):
        with self._lk:
            self._phase = "idle"
            self._t0 = None

    # ---- 后台:轮询参数清零 + 解析结果 ----

    def _run(self, log_path, offset, before):
        deadline = time.time() + CALIB_TIMEOUT_S
        cleared = False
        while time.time() < deadline:
            time.sleep(0.4)
            try:
                if rospy.get_param(CALIB_PARAM, 0) != 1:
                    cleared = True
                    break
            except Exception:
                pass   # master 瞬断不判死,继续等到超时

        phase, message, hook, pallet = "done", None, None, None
        if not cleared:
            phase = "failed"
            message = ("标定超时(%d 秒参数未清零),请查看 canbus 组件日志"
                       % int(CALIB_TIMEOUT_S))
            # 清掉触发参数:超时多为节点卡死,不清的话节点恢复后会自行开始
            # 一次无人观察的标定(节点自身在退出路径也会清,这里双保险)
            try:
                rospy.set_param(CALIB_PARAM, 0)
            except Exception:
                pass
        else:
            # 节点经 stdio 全缓冲写日志(emitLog 在 emitParam 之前入缓冲,
            # 参数清零先于行落盘):清零后稍等并重试读取,等缓冲冲出来
            text = None
            for _ in range(3):
                time.sleep(0.7)
                text = self._read_log_from(log_path, offset)
                if text and ("calibration" in text):
                    break
            res = self._parse_result(text)
            if res is None and log_path is not None:
                # 结果行可能落在重启后的新日志文件里(capture 的旧路径变冷):
                # 从头扫当前路径
                cur = self._current_log()
                if cur is not None and str(cur) != str(log_path):
                    text2 = self._read_log_from(cur, 0)
                    res = self._parse_result(text2)
            if res is not None:
                phase, message, hook, pallet = res
            else:
                # 日志无线索:退回 config.cfg 前后比对。只把"值变化"作为
                # 疑似完成上报,不冒充成功(red-team:窗口内任何手工编辑四键
                # 都曾被报成标定成功)
                after = self.current_limits()
                if after is not None and before is not None and after != before:
                    phase = "failed"
                    message = ("标定疑似完成(config.cfg 限值已变化)但未见"
                               "完成日志,请核对 canbus 组件日志")
                elif after is None or before is None:
                    phase = "failed"
                    message = "标定未完成(未见完成日志,config.cfg 不可读)"
                else:
                    phase = "failed"
                    message = "标定未完成(参数已清零但未见完成日志,限值未变化)"
        with self._lk:
            self._phase = phase
            self._message = message
            self._hook = hook
            self._pallet = pallet
            self._t0 = None

    def _parse_result(self, text):
        """从日志增量解析结果行;返回 (phase, message, hook, pallet) 或 None。

        - 只在最后一次 "calibration: started" 之后找(之前的行为上一轮
          残留,stdio 缓冲冲出时会混进本轮增量);
        - **done 行必须见到本轮 start 标记才采信**(标记与结果同缓冲同刷,
          真实完成必然两者都在;只有 done 没有标记 = 上一轮残留回放,
          t19 实证);refused/aborted/failed 行保持宽松——假失败可重试,
          假成功危险;
        - refused/aborted/failed 优先于 done(假失败可重试,假成功危险);
        - done 行带 "NOT written"(写盘失败,值重启即丢)按失败上报;
        - 值域校验:四值须为 0..255 且 min<max。
        """
        if not text:
            return None
        lines = text.splitlines()
        has_marker = False
        for i in range(len(lines) - 1, -1, -1):
            if "calibration: started" in lines[i]:
                lines = lines[i:]
                has_marker = True
                break
        done_line, done_m, fail_m = None, None, None
        for line in lines:
            m = _CALIB_DONE_RE.search(line)
            if m:
                done_m, done_line = m, line
            m = _CALIB_FAIL_RE.search(line)
            if m:
                fail_m = m
        if fail_m is not None:
            return ("failed",
                    "%s:%s" % (fail_m.group(1), fail_m.group(2).strip()),
                    None, None)
        if done_m is not None and has_marker:
            hook = [int(done_m.group(1)), int(done_m.group(2))]
            pallet = [int(done_m.group(3)), int(done_m.group(4))]
            if "NOT written" in done_line:
                return ("failed", "标定完成但 config.cfg 写入失败(值重启即丢)"
                        + (done_line.split("NOT written", 1)[1].strip(" ()；;")
                           and ":%s" % done_line.split("NOT written", 1)[1]
                           .strip(" ()；;")[:120] or ""), None, None)
            ok = all(0 <= v <= 255 for v in hook + pallet) \
                and hook[0] < hook[1] and pallet[0] < pallet[1]
            if not ok:
                return ("failed", "标定结果异常(hook %s, pallet %s)"
                        % (hook, pallet), None, None)
            return ("done", None, hook, pallet)
        return None

    def _current_log(self):
        """canbus 组件此刻的日志路径(重启后会是新文件;由 HmiApp 注入)."""
        if self._log_resolver is None:
            return None
        try:
            return self._log_resolver()
        except Exception:
            return None

    @staticmethod
    def _read_log_from(log_path, offset):
        """读取日志 offset 之后的内容(上限 2 MB)。

        文件比 offset 小 = 截断/轮转:增量已丢,返回空走兜底——绝不从头
        重扫(red-team 实证:截断后从头扫会把触发前的旧行当成本轮结果)。
        """
        if log_path is None:
            return ""
        try:
            size = log_path.stat().st_size
            if size < offset:
                return ""
            with open(str(log_path), "rb") as f:
                f.seek(offset)
                return f.read(CALIB_LOG_MAX_BYTES).decode("utf-8", "replace")
        except OSError:
            return ""

    # ---- 状态(挂 /api/state) ----

    def status(self):
        with self._lk:
            out = {
                "active": self._phase == "running",
                "phase": self._phase,
                "message": self._message,
                "hook_range": self._hook,
                "pallet_range": self._pallet,
                "current": self.current_limits(),
            }
            if self._phase == "running" and self._t0 is not None:
                out["elapsed_s"] = round(time.time() - self._t0, 1)
            return out


# ---------------------------------------------------------------- 录制配置监控

class RecordingConfigMonitor(object):
    """按文件时间戳缓存 rosbag topic 全量校验结果，供 HMI 告警显示。"""

    def __init__(self, enabled, config_path=None):
        self.enabled = bool(enabled)
        self.config_path = Path(config_path or record_rosbag.TOPIC_CONFIG)
        self._lk = threading.Lock()
        self._stamp = object()
        self._status = {"enabled": self.enabled, "ok": True, "error": None}

    def snapshot(self):
        if not self.enabled:
            return dict(self._status)
        with self._lk:
            try:
                stat = self.config_path.stat()
                stamp = (stat.st_mtime_ns, stat.st_size)
            except OSError as exc:
                stamp = (None, str(exc))
            if stamp == self._stamp:
                return dict(self._status)

            try:
                record_rosbag.read_topic_groups(self.config_path)
                status = {"enabled": True, "ok": True, "error": None}
            except Exception as exc:
                status = {"enabled": True, "ok": False, "error": str(exc)}
            self._stamp = stamp
            self._status = status
            return dict(status)


# ---------------------------------------------------------------- HTTP 服务

class HmiApp(object):
    """聚合各模块,供 handler 访问。"""

    def __init__(self, cfg, names):
        self.names = names
        self.started_at = time.time()
        self.sm = SystemMonitor()
        self.recording_config = RecordingConfigMonitor(
            {"perception_bags", "pnc_bags"}.issubset(names)
        )

        # ROS 可用则用 RosBridge 做健康判定,否则退化为仅进程存活
        self.bridge = None
        provider = process_manager.HealthProvider()
        if ROS_OK:
            import ros_bridge
            health_specs = []
            for c in cfg["components"]:
                health_specs.extend(c.get("health") or [])
            self.bridge = ros_bridge.RosBridge(
                health_specs, sample_period=cfg["defaults"].get("sample_period", 30))
            provider = self.bridge
        self.provider = provider
        self.pm = ProcessManager(cfg, ROOT, provider)
        self.pm.detect_foreign()
        threading.Thread(target=self.pm.monitor_loop, daemon=True).start()
        if self.bridge is not None:
            self.bridge.start()
        # 脱挂钩一键标定编排(触发/轮询/结果解析,见 CalibrationManager)
        self.calib = CalibrationManager(ROOT)
        self.calib._log_resolver = self._canbus_log_path

    def state(self):
        if self.bridge is not None:
            vehicle = self.bridge.vehicle_state_final()
            master_ok = self.bridge.master_ok
            errs = self.bridge.msg_import_errors()
        else:
            vehicle = {"ros_available": False,
                       "reason": "无 ROS 环境(rospy 导入失败),仅进程管理可用"}
            master_ok = None
            errs = {}
        st = self.pm.state()
        return {
            "server": {"version": VERSION, "started_at": self.started_at,
                       "ros_available": ROS_OK, "master_ok": master_ok,
                       "msg_import_errors": errs},
            "sequence": st["sequence"],
            "components": st["components"],
            "recording_config": self.recording_config.snapshot(),
            "vehicle": vehicle,
            "calibration": self.calib.status(),
            "system": self.sm.snapshot(),
        }

    # ---- 一键标定(供 handler 调) ----

    def _canbus_log_path(self):
        runner = self.pm.runners.get("canbus")
        if runner is None:
            return None
        with runner._lk:
            return runner.log_path

    def calib_start(self):
        if not ROS_OK:
            return 409, {"ok": False, "reminder": CALIB_REMINDER,
                         "detail": "无 ROS 环境,无法触发标定"}
        vehicle = None
        if self.bridge is not None:
            vehicle = self.bridge.vehicle_state_final()
        # canbus 组件日志路径(结果行解析的首选来源;组件未启动则退回
        # config.cfg 比对)
        return self.calib.start(vehicle, self._canbus_log_path())

    # ---- 组件操作(供 handler 调;stop/restart 走后台线程) ----

    def comp_start(self, name):
        if self.pm.seq_active():
            return 409, {"ok": False, "error": "一键编排进行中,请稍候"}
        ok, st = self.pm.start_component(name)
        if not ok:
            if st == "FOREIGN":
                return 409, {
                    "ok": False,
                    "error": "检测到外部进程，请先点击停止完成清理",
                }
            return 409, {"ok": False, "error": "当前状态 %s 不允许启动" % st}
        return 200, {"ok": True, "state": st}

    def comp_stop(self, name):
        if self.pm.seq_active():
            return 409, {"ok": False, "error": "一键编排进行中,请稍候"}
        r = self.pm.runners.get(name)
        if r is None:
            return 404, {"ok": False, "error": "未知组件"}
        with r._lk:
            if r.state == "STOPPED":
                if r.foreign:
                    threading.Thread(
                        target=self.pm.stop_component,
                        args=(name,),
                        daemon=True,
                    ).start()
                    return 200, {"ok": True, "note": "清理外部进程中"}
                return 200, {"ok": True, "note": "already stopped"}
            if r.state == "STOPPING":
                return 409, {"ok": False, "error": "正在停止中"}
        threading.Thread(target=self.pm.stop_component, args=(name,),
                         daemon=True).start()
        return 200, {"ok": True, "state": "STOPPING"}

    def comp_restart(self, name):
        if self.pm.seq_active():
            return 409, {"ok": False, "error": "一键编排进行中,请稍候"}
        r = self.pm.runners.get(name)
        if r is None:
            return 404, {"ok": False, "error": "未知组件"}
        with r._lk:
            if r.foreign:
                return 409, {
                    "ok": False,
                    "error": "检测到外部进程，请先点击停止完成清理",
                }
            if r.state in ("STARTING", "STOPPING"):
                return 409, {"ok": False, "error": "当前状态 %s 不允许重启" % r.state}
        threading.Thread(target=self.pm.restart_component, args=(name,),
                         daemon=True).start()
        return 200, {"ok": True, "state": "RESTARTING"}

    # ---- 日志 ----

    def logs(self, name, tail, download):
        if name not in self.names:
            return 404, {"ok": False, "error": "未知组件"}
        r = self.pm.runners[name]
        with r._lk:
            path = r.log_path
        if path is None or not path.exists():
            return 200, {"name": name, "lines": [], "size": 0, "note": "暂无日志"}
        size = path.stat().st_size
        if download:
            return 200, ("file", path, size)
        with open(str(path), "rb") as f:
            if size > LOG_READ_BYTES:
                f.seek(size - LOG_READ_BYTES)
                f.readline()   # 丢弃半行
            data = f.read()
        text = data.decode("utf-8", "replace")
        lines = text.splitlines()[-tail:]
        return 200, {"name": name, "lines": lines, "size": size}


def make_handler(app):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "qingweiHMI/" + VERSION

        # ---- 基础 ----

        def log_message(self, fmt, *args):
            pass   # 静默访问日志,避免刷屏

        def _send_json(self, code, obj):
            body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_page(self):
            page = HMI_DIR / "static" / "index.html"
            try:
                body = page.read_bytes()
            except FileNotFoundError:
                self._send_json(404, {"ok": False, "error": "缺少 static/index.html"})
                return
            try:
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _body_json(self):
            n = int(self.headers.get("Content-Length") or 0)
            if n <= 0:
                return {}
            try:
                return json.loads(self.rfile.read(n).decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                return {}

        def _guard_wfile(fn):
            def wrapped(self, *a, **kw):
                try:
                    return fn(self, *a, **kw)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            return wrapped

        # ---- GET ----

        @_guard_wfile
        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/" or u.path == "/index.html":
                self._send_page()
                return
            if u.path == "/api/state":
                self._send_json(200, app.state())
                return
            if u.path.startswith("/api/logs/"):
                name = unquote(u.path[len("/api/logs/"):])
                qs = parse_qs(u.query)
                try:
                    tail = min(int(qs.get("tail", ["200"])[0]), LOG_TAIL_MAX)
                except ValueError:
                    tail = 200
                tail = max(1, tail)
                download = qs.get("download", ["0"])[0] == "1"
                code, res = app.logs(name, tail, download)
                if code == 200 and isinstance(res, tuple):
                    _, path, size = res
                    self.send_response(200)
                    self.send_header("Content-Type",
                                     "text/plain; charset=utf-8")
                    self.send_header("Content-Disposition",
                                     'attachment; filename="%s.log"' % name)
                    self.send_header("Content-Length", str(size))
                    self.end_headers()
                    with open(str(path), "rb") as f:
                        while True:
                            chunk = f.read(65536)
                            if not chunk:
                                break
                            self.wfile.write(chunk)
                    return
                self._send_json(code, res)
                return
            if u.path == "/favicon.ico":
                self.send_response(204)
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            self._send_json(404, {"ok": False, "error": "not found: %s" % u.path})

        # ---- POST ----

        @_guard_wfile
        def do_POST(self):
            u = urlparse(self.path)
            parts = [p for p in u.path.split("/") if p]
            # 排空请求体:HTTP/1.1 keep-alive 下未读的 Content-Length 字节
            # 会被当作下一条请求行,复用连接的下一个请求立刻 501/400
            # (red-team 以浏览器真实报文复现;此前仅 /api/stop 读体)
            body = self._body_json()
            # /api/start | /api/stop | /api/components/<name>/<op>
            if u.path == "/api/start":
                if not app.pm.start_all_async():
                    return self._send_json(409, {"ok": False, "error": "编排已在进行中"})
                return self._send_json(200, {"ok": True, "state": "STARTING"})
            if u.path == "/api/calibrate":
                code, res = app.calib_start()
                return self._send_json(code, res)
            if u.path == "/api/stop":
                if body.get("confirm") != "STOP":
                    return self._send_json(
                        400, {"ok": False, "error": '需要 body {"confirm":"STOP"} 确认'})
                if not app.pm.stop_all_async():
                    return self._send_json(409, {"ok": False, "error": "编排已在进行中"})
                return self._send_json(200, {"ok": True, "state": "STOPPING"})
            if len(parts) == 4 and parts[:2] == ["api", "components"]:
                name, op = unquote(parts[2]), parts[3]
                if name not in app.names:
                    return self._send_json(404, {"ok": False, "error": "未知组件"})
                if op == "start":
                    code, res = app.comp_start(name)
                elif op == "stop":
                    code, res = app.comp_stop(name)
                elif op == "restart":
                    code, res = app.comp_restart(name)
                else:
                    code, res = 404, {"ok": False, "error": "未知操作 %s" % op}
                return self._send_json(code, res)
            self._send_json(404, {"ok": False, "error": "not found: %s" % u.path})

    return Handler


# ---------------------------------------------------------------- 入口

def main():
    ap = argparse.ArgumentParser(description="qingwei L4 Web HMI")
    ap.add_argument("--config", default=str(HMI_DIR / "hmi_config.py"))
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=int(os.environ.get("HMI_PORT", "8080")))
    args = ap.parse_args()

    cfg_path = Path(args.config)
    if not cfg_path.is_absolute():
        cfg_path = HMI_DIR / cfg_path
    try:
        cfg, names = load_config(cfg_path)
    except Exception as exc:
        print("[HMI] 配置加载失败(%s): %s" % (cfg_path, exc), file=sys.stderr)
        sys.exit(1)

    app = HmiApp(cfg, names)
    try:
        httpd = ThreadingHTTPServer((args.host, args.port), make_handler(app))
    except OSError as exc:
        print("[HMI] 端口绑定失败 %s:%d(%s)" % (args.host, args.port, exc),
              file=sys.stderr)
        sys.exit(1)
    httpd.daemon_threads = True

    def _shutdown(signum, _frame):
        print("[HMI] 收到信号 %d,退出(不停止组件;组件保持运行)" % signum)
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    print("[HMI] 已启动: http://%s:%d  (ROS=%s, 组件数=%d, 配置=%s)" % (
        args.host, args.port, "可用" if ROS_OK else "不可用", len(names), cfg_path.name))
    try:
        httpd.serve_forever()
    finally:
        httpd.server_close()
        print("[HMI] 已退出")


if __name__ == "__main__":
    main()
