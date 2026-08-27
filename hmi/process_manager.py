# -*- coding: utf-8 -*-
"""
进程管理:组件状态机、subprocess 拉起/停止、日志轮转、分组编排。

设计要点(见 hmi 配置头注释与 workflow 文档):
- 每个组件独立进程组(start_new_session),组杀不影响 HMI 自身
- 日志 fd 以追加模式打开并持有到组件停止,O_APPEND 语义下 copytruncate 轮转安全
- 本模块不 import rospy:健康判定通过注入的 health_provider(见 ros_bridge.py)
- 全部时长判断用 time.monotonic(),wall time 只用于日志与显示
"""

import os
import shlex
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path


# ---------------------------------------------------------------- 健康提供者

class HealthProvider(object):
    """健康判定基类。无 ROS 环境时的默认行为:话题类检查一律放行,文件类照常判。"""

    def ros_available(self):
        return False

    def check(self, specs):
        # specs 为空列表 → 进程活着即健康,由调用方处理;这里只处理非空
        ok, parts = True, []
        for sp in specs:
            if sp.get("type", "topic") == "file":
                age = _file_age(sp["path"])
                if age is None or age > sp["max_age"]:
                    ok = False
                    parts.append("%s 心跳超时" % os.path.basename(sp["path"]))
                else:
                    parts.append("%s 心跳 %.1fs" % (os.path.basename(sp["path"]), age))
        if not parts:
            parts.append("仅进程监控")
        return ok, " / ".join(parts)


def _file_age(path):
    try:
        return time.time() - os.path.getmtime(path)
    except OSError:
        return None


# ---------------------------------------------------------------- 组件运行器

# 状态集:STOPPED/STARTING/RUNNING/DEGRADED/STOPPING/CRASHED
_ACTIVE_STATES = ("STARTING", "RUNNING", "DEGRADED")


class ComponentRunner(object):

    def __init__(self, spec, logdir, vardir, health_provider):
        self.spec = spec
        self.name = spec["name"]
        self.logdir = Path(logdir)
        self.vardir = Path(vardir)
        self.hp = health_provider
        self._lk = threading.RLock()

        self.state = "STOPPED"
        self.proc = None
        self.logf = None
        self.log_path = None
        self.started_mono = None
        self.exit_code = None
        self.restarts = 0
        self.stop_failed = False
        self.last_error = None
        self.foreign = False
        self._ever_started = False
        self._health_bad_since = None
        self._health_ok = None
        self._health_detail = ""

    # ---------------- 对外接口(带锁的快照/操作) ----------------

    def snapshot(self):
        with self._lk:
            up = None
            if self.started_mono is not None and self.state in _ACTIVE_STATES + ("STOPPING",):
                up = round(time.monotonic() - self.started_mono, 1)
            log_size = 0
            if self.log_path is not None:
                try:
                    log_size = self.log_path.stat().st_size
                except OSError:
                    pass
            health = None
            if self.spec["health"] and self.state in ("RUNNING", "DEGRADED"):
                health = "ok" if self._health_ok else "bad"
            return {
                "name": self.name,
                "title": self.spec["title"],
                "group": self.spec["group"],
                "state": self.state,
                "uptime_s": up,
                "pid": self.proc.pid if self.proc is not None else None,
                "exit_code": self.exit_code,
                "restarts": self.restarts,
                "log_size": log_size,
                "log_file": str(self.log_path) if self.log_path else None,
                "health": health,
                "health_detail": self._health_detail,
                "optional": self.spec.get("optional", False),
                "enabled": self.spec.get("enabled", True),
                "foreign": self.foreign,
                "stop_failed": self.stop_failed,
                "last_error": self.last_error,
            }

    def do_start(self):
        """拉起进程(快速返回,Popen 本身不等待子进程)。仅 STOPPED/CRASHED 受理。"""
        with self._lk:
            if self.state not in ("STOPPED", "CRASHED"):
                return False
            self._open_log()
            argv = self._build_argv()
            try:
                env = dict(os.environ)
                env["HMI_VARDIR"] = str(self.vardir)
                env["HMI_COMP"] = self.name
                self.proc = subprocess.Popen(
                    argv, cwd=self.spec["cwd"],
                    stdin=subprocess.DEVNULL,
                    stdout=self.logf, stderr=subprocess.STDOUT,
                    start_new_session=True, env=env)
            except Exception as exc:  # 命令不存在/权限等,Popen 直接抛
                self._close_log()
                self.state = "CRASHED"
                self.exit_code = None
                self.last_error = "启动失败: %s" % exc
                return True
            self.logf.write(("==== start %s %s pid=%d ====\n   cmd: %s\n" % (
                self.name, time.strftime("%Y-%m-%d %H:%M:%S"), self.proc.pid,
                " ".join(argv))).encode("utf-8", "replace"))
            self.logf.flush()
            if self._ever_started:
                self.restarts += 1
            self._ever_started = True
            self.state = "STARTING"
            self.started_mono = time.monotonic()
            self.exit_code = None
            self.stop_failed = False
            self.last_error = None
            self._health_bad_since = None
            self._health_ok = None
            self.foreign = False
            return True

    def stop(self, timeout=25.0):
        """同步执行完整停止链。可从工作线程/编排线程调用,不要在 HTTP 线程调用。"""
        with self._lk:
            if self.state == "STOPPED":
                return True
            if self.state == "STOPPING":
                return False
            self.state = "STOPPING"
            proc = self.proc
        # 1) 自定义清理命令(杀 root 子进程等)
        stop_cmd = self.spec.get("stop_cmd")
        if stop_cmd:
            try:
                subprocess.run(["bash", "-c", stop_cmd], timeout=8,
                               stdin=subprocess.DEVNULL,
                               stdout=self.logf if self.logf else subprocess.DEVNULL,
                               stderr=subprocess.STDOUT)
            except Exception:
                pass
        # 2) SIGTERM 进程组 → 3) SIGKILL 进程组
        if proc is not None and proc.poll() is None:
            self._signal_group(proc, signal.SIGTERM, 5.0)
            if proc.poll() is None:
                self._signal_group(proc, signal.SIGKILL, 5.0)
        # 4) 等 monitor 线程回收(正常路径:poll 收尸 → _on_exit → STOPPED)
        if proc is not None:
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                with self._lk:
                    if self.state != "STOPPING":
                        break
                time.sleep(0.1)
        # 5) 残留检测与收尾
        with self._lk:
            if self.proc is not None and self.proc.poll() is not None:
                self._on_exit(self.proc.returncode)
            if self.state == "STOPPING":   # 极端情况:仍未回收,强制置位
                self._close_log()
                self.proc = None
                self.state = "STOPPED"
            pat = self.spec.get("stop_pat")
            if pat and _pgrep(pat):
                self.stop_failed = True
                self.last_error = "停止后仍有残留进程(pgrep -f %s)" % pat
            else:
                self.stop_failed = False
            return self.state == "STOPPED"

    def poll(self):
        """monitor 线程每秒调用:收尸、启动超时、健康降级/恢复。"""
        with self._lk:
            if self.proc is not None:
                rc = self.proc.poll()
                if rc is not None:
                    self._on_exit(rc)
                    return
            if self.state == "STARTING":
                self._eval_health(timeout_phase=True)
            elif self.state in ("RUNNING", "DEGRADED"):
                self._eval_health(timeout_phase=False)

    def rotate_if_needed(self, limit_scale=1.0):
        """日志超过上限时 copytruncate 轮转,保留 keep_logs 份副本。"""
        with self._lk:
            if self.logf is None or self.log_path is None:
                return
            spec = self.spec
            limit = spec.get("max_log_bytes")
            if limit is None:
                limit = spec["max_log_mb"] * 1024 * 1024
            limit = int(limit * limit_scale)
            try:
                if self.log_path.stat().st_size < limit:
                    return
                keep = spec.get("keep_logs", 3)
                for i in range(keep - 1, 0, -1):
                    src = Path(str(self.log_path) + (".%d" % i if i > 1 else ".1"))
                    if src.exists():
                        src.replace(Path(str(self.log_path) + ".%d" % (i + 1)))
                shutil.copy2(str(self.log_path), str(self.log_path) + ".1")
                self.logf.truncate(0)   # O_APPEND:后续写入自动回到新 EOF
                self.logf.flush()
            except OSError:
                pass

    # ---------------- 内部实现 ----------------

    def _eval_health(self, timeout_phase):
        specs = self.spec["health"]
        now = time.monotonic()
        if not specs:
            # 无健康规格:存活超过宽限期即 RUNNING,之后不再降级
            if self.state == "STARTING":
                if now - self.started_mono >= self.spec.get("ready_s", 1.5):
                    self.state = "RUNNING"
                    self._health_detail = "仅进程监控"
            return
        ok, detail = self.hp.check(specs)
        self._health_ok, self._health_detail = ok, detail
        if self.state == "STARTING":
            if ok:
                self.state = "RUNNING"
                self._health_bad_since = None
            elif now - self.started_mono > self.spec["start_timeout"]:
                self.state = "DEGRADED"
                self.last_error = "启动超时未达健康(%ds)" % self.spec["start_timeout"]
        else:
            if ok:
                self._health_bad_since = None
                if self.state == "DEGRADED":
                    self.state = "RUNNING"
                    if self.last_error and self.last_error.startswith("健康丢失"):
                        self.last_error = None
            else:
                if self._health_bad_since is None:
                    self._health_bad_since = now
                elif now - self._health_bad_since >= self.spec["health_lost_s"]:
                    if self.state != "DEGRADED":
                        self.state = "DEGRADED"
                        self.last_error = "健康丢失(%s)" % detail

    def _on_exit(self, rc):
        # 持锁调用
        dur = 0.0
        if self.started_mono is not None:
            dur = time.monotonic() - self.started_mono
        self._close_log()
        self.proc = None
        self.exit_code = rc
        if self.state == "STOPPING":
            self.state = "STOPPED"
            return
        self.state = "CRASHED"
        if rc is not None and rc < 0:
            reason = "被信号 %d 终止" % (-rc)
        else:
            reason = "返回码 %s" % rc
        self.last_error = "异常退出:%s(存活 %.1fs)" % (reason, dur)

    def _open_log(self):
        d = self.logdir / self.name
        d.mkdir(parents=True, exist_ok=True)
        today = time.strftime("%Y%m%d")
        seq = 1
        for p in d.glob("%s-*.log" % today):
            tail = p.stem.split("-")[-1]
            if tail.isdigit():
                seq = max(seq, int(tail) + 1)
        self.log_path = d / ("%s-%d.log" % (today, seq))
        self.logf = open(str(self.log_path), "ab")

    def _close_log(self):
        if self.logf is not None:
            try:
                self.logf.close()
            except OSError:
                pass
            self.logf = None

    def _build_argv(self):
        cmd = self.spec["cmd"]
        setup = self.spec.get("setup") or []
        if not setup:
            return list(cmd)
        chain = " && ".join(
            ["source '%s'" % p for p in setup] +
            ["exec " + " ".join(shlex.quote(c) for c in cmd)])
        return ["bash", "-c", chain]

    def _signal_group(self, proc, sig, wait_s):
        try:
            os.killpg(proc.pid, sig)
        except (ProcessLookupError, PermissionError):
            pass
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                return
            time.sleep(0.2)


def _pgrep(pattern):
    """进程特征命中检测(pgrep -f)。找不到 pgrep 命令时视为无残留。"""
    try:
        r = subprocess.run(["pgrep", "-f", pattern], stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, timeout=5)
        return r.returncode == 0
    except Exception:
        return False


# ---------------------------------------------------------------- 进程管理器

class ProcessManager(object):

    def __init__(self, cfg, root, health_provider):
        self.root = Path(root)
        self.hmi_dir = self.root / "hmi"
        self.logdir = self.hmi_dir / "logs"
        self.vardir = self.hmi_dir / "var"
        self.logdir.mkdir(parents=True, exist_ok=True)
        self.vardir.mkdir(parents=True, exist_ok=True)

        self.groups = cfg["groups"]
        defaults = cfg["defaults"]
        self.runners = {}
        for raw in cfg["components"]:
            spec = dict(defaults)
            spec.update(raw)
            self.runners[spec["name"]] = ComponentRunner(
                spec, self.logdir, self.vardir, health_provider)

        self._seq_lk = threading.Lock()
        self._seq = {"active": False, "action": None, "current_group": None,
                     "failed": [], "started_at": None, "note": None}
        self._limit_scale = 1.0

    # ---------------- 查询 ----------------

    def state(self):
        with self._seq_lk:
            seq = dict(self._seq)
        comps = [self.runners[n].snapshot() for n in sorted(
            self.runners, key=lambda n: (self.runners[n].spec["group"],
                                         self.runners[n].name))]
        for c in comps:
            c["group_title"] = self.groups.get(c["group"], str(c["group"]))
        return {"sequence": seq, "components": comps}

    def seq_active(self):
        with self._seq_lk:
            return self._seq["active"]

    def names(self):
        return set(self.runners.keys())

    # ---------------- 单组件操作 ----------------

    def start_component(self, name):
        """快速操作,Popen 即返。返回 (ok, state_or_msg)。"""
        r = self.runners.get(name)
        if r is None:
            return False, "未知组件"
        ok = r.do_start()
        with r._lk:
            return ok, r.state

    def stop_component(self, name):
        """阻塞操作(最长 ~25s),调用方需在工作线程中执行。"""
        r = self.runners.get(name)
        if r is None:
            return False, "未知组件"
        ok = r.stop()
        with r._lk:
            return ok, r.state

    def restart_component(self, name):
        """阻塞操作:完整停止后再启动。"""
        r = self.runners.get(name)
        if r is None:
            return False, "未知组件"
        with r._lk:
            if r.state in ("STARTING", "STOPPING"):
                return False, r.state
        r.stop()
        r.do_start()
        with r._lk:
            return True, r.state

    # ---------------- 一键编排(各自在后台线程执行) ----------------

    def start_all_async(self):
        with self._seq_lk:
            if self._seq["active"]:
                return False
            self._seq.update(active=True, action="start", current_group=None,
                             failed=[], started_at=time.time(), note=None)
        threading.Thread(target=self._orchestrate_start, daemon=True).start()
        return True

    def stop_all_async(self):
        with self._seq_lk:
            if self._seq["active"]:
                return False
            self._seq.update(active=True, action="stop", current_group=None,
                             failed=[], started_at=time.time(), note=None)
        threading.Thread(target=self._orchestrate_stop, daemon=True).start()
        return True

    def _orchestrate_start(self):
        try:
            for g in sorted(self.groups):
                with self._seq_lk:
                    self._seq["current_group"] = g
                comps = [r for r in self.runners.values()
                         if r.spec["group"] == g and r.spec.get("enabled", True)]
                pending = [r for r in comps
                           if r.state in ("STOPPED", "CRASHED")]
                for r in pending:
                    r.do_start()
                if not pending:
                    continue
                deadline = time.monotonic() + max(
                    r.spec["start_timeout"] for r in pending) + 3.0
                while time.monotonic() < deadline:
                    if all(r.state != "STARTING" for r in pending):
                        break
                    time.sleep(0.5)
                # 健康门控:非 optional 组件未达 RUNNING/DEGRADED 之外的健康态则中止
                failed = [r.name for r in pending
                          if not r.spec.get("optional", False)
                          and r.state in ("CRASHED", "DEGRADED")]
                if failed:
                    with self._seq_lk:
                        self._seq.update(
                            active=False, failed=failed,
                            note="启动中止:组%d %s 未就绪,后续组未启动"
                                 % (g, ",".join(failed)))
                    return
            with self._seq_lk:
                self._seq.update(active=False, current_group=None,
                                 note="启动完成")
        except Exception as exc:  # 编排自身异常也要解除占用
            with self._seq_lk:
                self._seq.update(active=False, note="编排异常: %s" % exc)

    def _orchestrate_stop(self):
        try:
            for g in sorted(self.groups, reverse=True):
                with self._seq_lk:
                    self._seq["current_group"] = g
                comps = [r for r in self.runners.values()
                         if r.spec["group"] == g and r.state != "STOPPED"]
                if not comps:
                    continue
                threads = [threading.Thread(target=r.stop, daemon=True)
                           for r in comps]
                for t in threads:
                    t.start()
                for t in threads:
                    t.join(timeout=35.0)
            with self._seq_lk:
                self._seq.update(active=False, current_group=None, note="已全部停止")
        except Exception as exc:
            with self._seq_lk:
                self._seq.update(active=False, note="编排异常: %s" % exc)

    # ---------------- 后台监控 ----------------

    def monitor_loop(self):
        """常驻线程:1s 收尸/健康判定;30s 日志轮转+磁盘水位。"""
        rot_at = time.monotonic() + 30.0
        while True:
            time.sleep(1.0)
            for r in self.runners.values():
                try:
                    r.poll()
                except Exception:
                    pass
            now = time.monotonic()
            if now >= rot_at:
                rot_at = now + 30.0
                self._housekeeping()

    def _housekeeping(self):
        # 磁盘两级水位:1GB 警告只缩上限;500MB 删轮转副本再缩上限
        try:
            free = shutil.disk_usage(str(self.hmi_dir)).free
        except OSError:
            return
        if free < 1024 * 1024 * 1024:
            self._limit_scale = 0.5
        if free < 500 * 1024 * 1024:
            self._limit_scale = 0.25
            for r in self.runners.values():
                keep = r.spec.get("keep_logs", 3)
                for i in range(1, keep + 2):
                    for p in self.logdir.glob("*/*.%d" % i):
                        try:
                            p.unlink()
                        except OSError:
                            pass
        for r in self.runners.values():
            try:
                r.rotate_if_needed(self._limit_scale)
            except Exception:
                pass

    def detect_foreign(self):
        """HMI 启动时识别疑似外部启动的组件(不收养,仅标记提示)。"""
        for r in self.runners.values():
            pat = r.spec.get("stop_pat")
            if pat and r.state == "STOPPED" and _pgrep(pat):
                r.foreign = True
