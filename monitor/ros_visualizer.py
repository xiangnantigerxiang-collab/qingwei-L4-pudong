# -*- coding: utf-8 -*-
"""
ros_visualizer - monitor 的 rospy 桥(对标 hmi/ros_bridge.py 的骨架)
====================================================================

职责:
- 类型化订阅全部可视化话题,回调只持锁存最新消息引用(近零开销)
- snapshot() 组装 JSON 快照(换算语义对标 simviewer C++ 实现)
- load_map() 解析 view.csv 为三条折线(中心+双紫边线),一次缓存
- convert_scan() LaserScan 极坐标->笛卡尔->乘安装外参->紧凑二进制
- _CloudWorker 专职线程:活动门控订阅 + 节拍解析 4 路 PointCloud2
  (跨步预抽稀->体素->上限),多浏览器共用 packed bytes

线程模型:回调线程只写 _latest/_ages;snapshot/scan_bin 在 HTTP 线程读;
cloud 解析在专职线程。全部经 self._lk(RLock)。

无 ROS 环境(开发机):import rospy 失败 -> 一切降级,
snapshot 仍可返回结构(全 None 字段),便于前端/服务自测。
"""

import math
import os
import struct
import threading
import time
import xmlrpc.client

try:
    import rospy
    ROS_AVAILABLE = True
except ImportError:
    rospy = None
    ROS_AVAILABLE = False

MON_DIR = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------
# 常量与换算(语义真源:src/simview/src/{draw,thread}.cpp)
# ---------------------------------------------------------------------------

# 车身轮廓(米),对标 DrawVehicle:前 2.3 / 后 -0.8 / 半宽 0.73
VEHICLE_SHAPE = (2.3, -0.8, 0.73)

# 二进制点云协议(小端):
#   头 20B: magic(4) ver(u16) 通道数(u16) 总点数(u32) 时刻ms(u64)
#   通道子头 8B: id(u8) flags(u8) 保留(u16) 点数(u32)
#   点 12B: x_mm(i32) y_mm(i32) z_mm(i16) intensity(u8) pad(u8)
BIN_MAGIC = b"QWMC"
BIN_VERSION = 1
_BIN_HDR = struct.Struct("<4sHHIQ")
_BIN_CHAN = struct.Struct("<BBHI")
_BIN_POINT = struct.Struct("<iihBB")

# PointCloud2 datatype 码 -> (struct 前缀字符, 字节数)
PC2_TYPES = {
    1: ("b", 1), 2: ("B", 1),      # INT8/UINT8
    3: ("h", 2), 4: ("H", 2),      # INT16/UINT16
    5: ("i", 4), 6: ("I", 4),      # INT32/UINT32
    7: ("f", 4), 8: ("d", 8),      # FLOAT32/FLOAT64
}

# 类型化订阅表:(话题, 模块, 类名, 处理函数名)
TYPED_SUBS = [
    ("/navigation_msg",   "robot.msg", "navigation_msg",   "_on_navigation"),
    ("/perception",       "robot.msg", "perception",       "_on_perception"),
    ("/plan_path_msg",    "robot.msg", "path_plan_msg",    "_on_plan_path"),
    ("/refer_path_msg",   "robot.msg", "path_plan_msg",    "_on_refer_path"),
    ("/path_plan_status", "robot.msg", "path_plan_status", "_on_path_status"),
    ("/palletpos",        "robot.msg", "palletpos",        "_on_palletpos"),
    # 右侧状态栏扩展:任务/控制/CAN 反馈(2026-08-28)
    ("/task_plan_msg",      "robot.msg",  "task_plan_msg", "_on_task"),
    ("/cloud/task/task_status", "robot.msg", "TaskStatus", "_on_task_status"),
    ("/control_msg",        "robot.msg",  "control_msg",   "_on_control"),
    ("/can_msg",            "canbus.msg", "can_msg",       "_on_can_msg"),
]


def yaw_from_heading(heading_deg):
    """heading(度,北基准顺时针) -> 显示系偏航弧度。

    对标 simviewer: yaw = 90 - heading,环绕到 (-360,360] 后转弧度。
    前端直接 THREE.rotation.y = yaw(不取负,推导见 README)。
    NaN/Inf 输入归 0(否则会以 nan 形式进入 JSON,浏览器解析失败)。
    """
    try:
        y = 90.0 - float(heading_deg or 0.0)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(y):
        return 0.0
    while y > 360.0:
        y -= 360.0
    while y <= -360.0:
        y += 360.0
    return math.radians(y)


def _r2(v):
    """round 到厘米,压 JSON 体积;NaN/Inf 归 0(见 yaw_from_heading 注)。"""
    try:
        f = float(v)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(f):
        return 0.0
    return round(f, 2)


# ---------------------------------------------------------------------------
# 地图解析(对标 simviewer LoadMap/DrawMap,免疫 C 的 feof 重复末行怪癖)
# ---------------------------------------------------------------------------

def load_map_lines(path):
    """view.csv(x,y,heading 三列) -> {center,left,right,bbox} 三条折线。

    - 点距 > 0.1m 才保留(对标 LoadMap 的抽稀)
    - heading 列经 90-z 转弧度后作为该点法向
    - 边线 = 中心点 ± 1m * (sin(zg), -cos(zg))(对标 DrawMap 紫线)
    """
    center = []
    # errors="replace":GBK 表头等非 UTF-8 字节降级为 U+FFFD 后按坏行
    # 跳过(默认严格模式会让 UnicodeDecodeError 整文件丢弃);
    # isfinite 守卫:float('inf'/'nan') 能解析成功,不滤则 inf 进环绕
    # 循环死循环(实测挂死 API 线程)、nan 毒化文件 bbox 使合并 origin
    # 静默偏移 —— 与 yaw_from_heading 的守卫同源
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) < 3:
                continue
            try:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            except ValueError:
                continue
            if not (math.isfinite(x) and math.isfinite(y)
                    and math.isfinite(z)):
                continue
            zg = 90.0 - z
            while zg > 360.0:
                zg -= 360.0
            while zg <= -360.0:
                zg += 360.0
            zg = math.radians(zg)
            if center and math.hypot(x - center[-1][0], y - center[-1][1]) <= 0.1:
                continue
            center.append((x, y, zg))

    left, right = [], []
    for x, y, zg in center:
        s, c = math.sin(zg), math.cos(zg)
        left.append((x + 1.0 * s, y - 1.0 * c))
        right.append((x - 1.0 * s, y + 1.0 * c))

    xs = [p[0] for p in center]
    ys = [p[1] for p in center]
    bbox = [min(xs), min(ys), max(xs), max(ys)] if center else [0, 0, 0, 0]
    return {"center": center, "left": left, "right": right, "bbox": bbox}


# ---------------------------------------------------------------------------
# LaserScan -> 二进制点(含安装外参)
# ---------------------------------------------------------------------------

def scan_to_points(msg, ext):
    """极坐标->笛卡尔->乘外参(平移+旋转)->[(x_mm,y_mm,0,intensity)]。

    跳过 NaN/Inf/0/负值/超 [range_min,range_max) 的 beam;
    intensities 缺项填 0。外参 ext = {"x","y","yaw_deg"},车体系。
    """
    try:
        amin = float(msg.angle_min)
        ainc = float(msg.angle_increment)
        rmin = float(msg.range_min)
        rmax = float(msg.range_max)
        ranges = msg.ranges or []
        ints = msg.intensities or []
    except (AttributeError, TypeError, ValueError):
        return []

    ex = float(ext.get("x", 0.0))
    ey = float(ext.get("y", 0.0))
    eyaw = math.radians(float(ext.get("yaw_deg", 0.0)))
    c, s = math.cos(eyaw), math.sin(eyaw)

    pts = []
    for k in range(len(ranges)):
        r = ranges[k]
        if not (rmin <= r < rmax):       # 覆盖 nan/inf/0/超界
            continue
        a = amin + k * ainc
        x = r * math.cos(a)
        y = r * math.sin(a)
        xr = x * c - y * s + ex
        yr = x * s + y * c + ey
        iv = ints[k] if k < len(ints) else 0
        try:
            iv = int(iv)
        except (TypeError, ValueError, OverflowError):   # nan/inf 安全
            iv = 0
        if iv < 0:
            iv = 0
        elif iv > 255:
            iv = 255
        pts.append((int(round(xr * 1000.0)), int(round(yr * 1000.0)), 0, iv))
    return pts


# ---------------------------------------------------------------------------
# PointCloud2 解析(纯函数,便于单测)
# ---------------------------------------------------------------------------

def parse_cloud_points(msg, stride, z_min, z_max):
    """PointCloud2 -> [(x,y,z,intensity)](米,车体系)。

    - stride: 每 N 点取 1(解包前的字节级跨步,省 CPU 的第一道闸)
    - 过滤 NaN/Inf 与 [z_min,z_max] 之外的高度
    - 支持 datatype 2/4/6/7/8(x/y/z 实践恒 FLOAT32,intensity 可能变)
    - packed float32 快路径走 iter_unpack;否则逐点 unpack_from
    - is_bigendian 时格式前缀 '>'
    """
    try:
        fields = {f.name: (f.offset, f.datatype) for f in msg.fields}
        point_step = int(msg.point_step)
        data = msg.data
        big = bool(getattr(msg, "is_bigendian", 0))
    except (AttributeError, TypeError):
        return []
    if "x" not in fields or "y" not in fields or "z" not in fields:
        return []
    if point_step <= 0 or not data:
        return []

    end = ">" if big else "<"
    fmts = {}
    for name in ("x", "y", "z", "intensity"):
        if name not in fields:
            fmts[name] = None
            continue
        off, dt = fields[name]
        if dt not in PC2_TYPES:
            return []
        ch, sz = PC2_TYPES[dt]
        if off + sz > point_step:
            return []              # 坏布局,整体放弃
        fmts[name] = (off, end + ch, sz)

    # 快路径:x/y/z/intensity 恰为 0/4/8/12、全 FLOAT32、step=16、小端
    try:
        row_step = int(msg.row_step)
        height = int(msg.height)
    except AttributeError:
        row_step = point_step
        height = 1
    if not big and point_step == 16 and height == 1 and all(
            fmts[n] is not None and fmts[n][0] == o and fmts[n][1] == "<f"
            for n, o in (("x", 0), ("y", 4), ("z", 8), ("intensity", 12))):
        usable = len(data) - (len(data) % 16)
        it = struct.Struct("<ffff").iter_unpack(data[:usable])
        out = []
        for idx, (x, y, z, i) in enumerate(it):
            if idx % stride:
                continue
            # isfinite 同时滤 NaN 与 Inf(Inf 若漏过,int(round(inf)) 会
            # OverflowError 杀死解析线程)
            if not (math.isfinite(x) and math.isfinite(y)
                    and math.isfinite(z)):
                continue
            if not (z_min <= z <= z_max):
                continue
            out.append((x, y, z, i))
        return out

    # 慢路径:逐点 unpack_from(支持任意 offset/datatype/organized)
    total = min(len(data) // point_step,
                (row_step * height) // point_step if height > 1
                else len(data) // point_step)
    out = []
    for idx in range(0, total, stride):
        base = idx * point_step
        vals = []
        ok = True
        for name in ("x", "y", "z", "intensity"):
            f = fmts[name]
            if f is None:
                vals.append(0.0)
                continue
            try:
                v = struct.unpack_from(f[1], data, base + f[0])[0]
            except struct.error:
                ok = False
                break
            vals.append(v)
        if not ok:
            break
        x, y, z, i = vals
        if not (math.isfinite(x) and math.isfinite(y)
                and math.isfinite(z)):          # 滤 NaN 与 Inf
            continue
        if not (z_min <= z <= z_max):
            continue
        out.append((x, y, z, i))
    return out


def voxelize(points, voxel, max_points):
    """体素网格抽稀(每格留最后一点,dict 插入序保确定性别),超上限截断。

    points: [(x,y,z,i)] 米。返回同构列表。
    """
    if voxel <= 0:
        return points[:max_points]
    inv = 1.0 / voxel
    grid = {}
    for p in points:
        key = (int(p[0] * inv), int(p[1] * inv), int(p[2] * inv))
        grid[key] = p
        if len(grid) >= max_points * 4:   # 防极端输入撑爆 dict
            break
    vals = list(grid.values())
    return vals[:max_points] if len(vals) > max_points else vals


def pack_bin(channels, ts_ms):
    """[(chan_id, count, blob)] -> 自描述二进制帧(blob 为已 pack 的点串)。"""
    total = sum(c for _, c, _ in channels)
    parts = [_BIN_HDR.pack(BIN_MAGIC, BIN_VERSION, len(channels), total,
                           ts_ms)]
    for chan_id, count, blob in channels:
        parts.append(_BIN_CHAN.pack(chan_id, 0, 0, count))
        parts.append(blob)
    return b"".join(parts)


# ---------------------------------------------------------------------------
# 主类
# ---------------------------------------------------------------------------

class RosVisualizer(object):

    def __init__(self, config):
        cfg = config or {}
        self._cfg = cfg
        self._cloud_cfg = dict(cfg.get("CLOUD") or {})

        self._lk = threading.RLock()
        self._latest = {}          # topic -> (msg, monotonic)
        self._typed_cache = {}     # (module, cls) -> class or None
        self._msg_import_errors = []
        self._subs = []
        self._master_ok = False
        self._master_epoch = 0     # 每次 master pid 变化 +1(cloud 线程对齐用)
        self._started = False
        self._stop = threading.Event()

        # 地图(懒加载缓存)
        self._map_payload = None

        # scan 缓存(按 msg 身份;数据小,身份变了才重算)
        self._scan_cfg = dict(cfg.get("SCAN_EXTRINSICS") or {})
        for t, ext in (cfg.get("SCAN_EXTRAS") or {}).items():
            self._scan_cfg[t] = ext
        self._scan_subs = []
        self._scan_cache = {}      # topic -> (msg_id, bytes)

        # cloud worker 状态
        self._cloud_last_pull = 0.0
        self._cloud_subs = []      # [(topic, sub)]
        self._cloud_subscribed = False
        self._cloud_epoch_seen = -1
        self._cloud_cache = b""    # 最新 packed 帧
        self._cloud_worker = None

    @staticmethod
    def _safe_print(text):
        """请求路径打印守卫:stdout 断开(终端关闭)或编码异常时,日志
        丢失可以,API 挂掉不行(实测 BrokenPipeError 会沿 map_payload
        ->snapshot 传播且因缓存未设而每请求重炸)。"""
        try:
            print(text)
        except Exception:
            pass

    # ---------------- 生命周期 ----------------

    def start(self):
        if self._started:
            return
        self._started = True
        if not ROS_AVAILABLE:
            return
        t = threading.Thread(target=self._run, name="mon-ros", daemon=True)
        t.start()
        w = threading.Thread(target=self._cloud_run, name="mon-cloud",
                             daemon=True)
        w.start()
        self._cloud_worker = w

    def stop(self):
        self._stop.set()

    def _run(self):
        """后台:init_node -> 订阅 -> 5s 周期 master 探活(重启则重建订阅)。"""
        name = "monitor_visualizer"
        while not self._stop.is_set():
            try:
                rospy.init_node(name, disable_signals=True, anonymous=True,
                                log_level=rospy.ERROR)
                break
            except Exception:
                time.sleep(5.0)
        if self._stop.is_set():
            return

        self._rebuild_subs()
        self._rebuild_scan_subs()
        # 立即探活一次:否则启动后前 5s 页面恒显"master 失联"横幅
        if self._master_check():
            with self._lk:
                self._master_epoch += 1
            self._rebuild_subs()
            self._rebuild_scan_subs()
        while not self._stop.is_set():
            time.sleep(5.0)
            if self._master_check():
                with self._lk:
                    self._master_epoch += 1
                self._rebuild_subs()
                self._rebuild_scan_subs()

    # ---------------- 订阅管理(对标 hmi ros_bridge) ----------------

    def _load_class(self, module, cls):
        key = (module, cls)
        if key in self._typed_cache:
            return self._typed_cache[key]
        klass = None
        try:
            m = __import__(module, fromlist=[cls])
            klass = getattr(m, cls)
        except (ImportError, AttributeError):
            self._msg_import_errors.append("%s.%s" % (module, cls))
        self._typed_cache[key] = klass
        return klass

    def _rebuild_subs(self):
        with self._lk:
            for s in self._subs:
                try:
                    s.unregister()
                except Exception:
                    pass
            self._subs = []
            if not ROS_AVAILABLE:
                return
            for topic, module, cls, handler in TYPED_SUBS:
                fn = getattr(self, handler)
                klass = self._load_class(module, cls)
                if klass is not None:
                    self._subs.append(rospy.Subscriber(
                        topic, klass, fn, queue_size=1, tcp_nodelay=True))

    def _rebuild_scan_subs(self):
        with self._lk:
            for s in self._scan_subs:
                try:
                    s.unregister()
                except Exception:
                    pass
            self._scan_subs = []
            if not ROS_AVAILABLE:
                return
            for topic in self._scan_cfg:
                klass = self._load_class("sensor_msgs.msg", "LaserScan")
                if klass is not None:
                    self._scan_subs.append(rospy.Subscriber(
                        topic, klass,
                        lambda m, t=topic: self._stash(t, m),
                        queue_size=1, tcp_nodelay=True))

    def _master_check(self):
        """探活 ROS master;返回 pid 是否发生了变化(需重建订阅)。"""
        import xmlrpc.client
        uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
        try:
            t = _TimeoutTransport()
            proxy = xmlrpc.client.ServerProxy(uri, transport=t)
            code, _, pid = proxy.getPid("/monitor")
            ok = (code == 1)
        except Exception:
            ok = False
            pid = None
        changed = False
        with self._lk:
            if ok and self._master_pid is not None and pid != self._master_pid:
                changed = True
            self._master_ok = ok
            self._master_pid = pid
        return changed

    _master_pid = None

    # ---------------- 回调(只存引用) ----------------

    def _stash(self, topic, msg):
        with self._lk:
            self._latest[topic] = (msg, time.monotonic())

    def _on_navigation(self, msg):
        self._stash("/navigation_msg", msg)

    def _on_perception(self, msg):
        self._stash("/perception", msg)

    def _on_plan_path(self, msg):
        self._stash("/plan_path_msg", msg)

    def _on_refer_path(self, msg):
        self._stash("/refer_path_msg", msg)

    def _on_path_status(self, msg):
        self._stash("/path_plan_status", msg)

    def _on_palletpos(self, msg):
        self._stash("/palletpos", msg)

    def _on_task(self, msg):
        self._stash("/task_plan_msg", msg)

    def _on_task_status(self, msg):
        self._stash("/cloud/task/task_status", msg)

    def _on_control(self, msg):
        self._stash("/control_msg", msg)

    def _on_can_msg(self, msg):
        self._stash("/can_msg", msg)

    # ---------------- 快照组装 ----------------

    def _age(self, topic, now):
        with self._lk:
            ent = self._latest.get(topic)
        if not ent:
            return None
        return round(now - ent[1], 1)

    def _origin(self):
        mp = self.map_payload()
        b = mp["bbox"]
        return ((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0)

    def snapshot(self):
        now = time.monotonic()
        ox, oy = self._origin()
        with self._lk:
            latest = dict(self._latest)      # 一次性快照,后续无锁读

        veh = None
        m = latest.get("/navigation_msg")
        if m:
            m = m[0]
            veh = {
                "x": _r2(getattr(m, "xAxis", 0.0) - ox),
                "y": _r2(getattr(m, "yAxis", 0.0) - oy),
                "yaw": round(yaw_from_heading(getattr(m, "heading", 0.0)), 4),
                "speed": _r2(getattr(m, "gpsSpeed", 0.0)),
            }

        obstacles = []
        m = latest.get("/perception")
        if m:
            for o in (getattr(m[0], "objs", None) or []):
                try:
                    obstacles.append({
                        # 对标 DrawLidarObjects:length=dy / width=dx(代码
                        # 实际行为,与 object.msg 注释相反)
                        "x": _r2(getattr(o, "x", 0.0) - ox),
                        "y": _r2(getattr(o, "y", 0.0) - oy),
                        "l": _r2(getattr(o, "dy", 0.0)),
                        "w": _r2(getattr(o, "dx", 0.0)),
                        "h": _r2(getattr(o, "height", 0.0)),
                        "yaw": round(yaw_from_heading(
                            getattr(o, "heading", 0.0)), 4),
                        "id": int(getattr(o, "id", 0)),
                        "vx": _r2(getattr(o, "vx", 0.0)),
                        "vy": _r2(getattr(o, "vy", 0.0)),
                    })
                except (AttributeError, TypeError, ValueError):
                    continue

        def path_of(topic):
            ent = latest.get(topic)
            if not ent:
                return None
            m = ent[0]
            xs = list(getattr(m, "x", None) or [])
            ys = list(getattr(m, "y", None) or [])
            if len(xs) != len(ys):      # 对标 thread.cpp:长度不等丢弃整帧
                return None
            return [[_r2(x - ox), _r2(y - oy)] for x, y in zip(xs, ys)]

        stop = None
        m = latest.get("/path_plan_status")
        if m:
            m = m[0]
            sa = float(getattr(m, "stopAngle", 0.0) or 0.0)
            stop = {
                "x": _r2(getattr(m, "stopX", 0.0) - ox),
                "y": _r2(getattr(m, "stopY", 0.0) - oy),
                # 对标 rviz 实际行为:C++ 忽略 stopAngle 恒发 identity;
                # 有值时用显示系换算(增强,config 不可关--0 值自然退化)
                "yaw": round(yaw_from_heading(sa), 4) if sa else 0.0,
            }

        pallet = None
        m = latest.get("/palletpos")
        if m:
            m = m[0]
            pallet = {"x": _r2(getattr(m, "xg", 0.0) - ox),
                      "y": _r2(getattr(m, "yg", 0.0) - oy)}

        # ---- 任务(pnc task 模块) ----
        # exec 来自 path_plan_status.taskExecuStatus(0 无任务/1 执行中
        # /2 完成);0 是合法值,不可用 or 兜底(会把 0 吞成缺省)
        st = None
        mps = latest.get("/path_plan_status")
        if mps:
            v = getattr(mps[0], "taskExecuStatus", None)
            st = {"exec": None if v is None else int(v)}
        task = None
        m = latest.get("/task_plan_msg")
        if m:
            m = m[0]
            task = {
                "id": int(getattr(m, "task_id", 0) or 0),
                "type": int(getattr(m, "taskType", 0) or 0),
                "work_mode": int(getattr(m, "workMode", 0) or 0),
                # 执行状态来自 path_plan_status(10Hz,同属任务链)
                "exec": (st["exec"] if st else None),
            }
        if st and task is None:
            # 仅状态链存活时:任务字段用 None(前端显示 "--"),
            # 不可合成 0 —— 真实任务 id 均非 0,"#0" 会被读作真实任务
            task = {"id": None, "type": None, "work_mode": None,
                    "exec": st["exec"]}
        # 云端任务状态(/cloud/task/task_status:1=执行中)
        m = latest.get("/cloud/task/task_status")
        if m:
            m = m[0]
            ti = getattr(m, "task_info", None)
            if task is None:
                task = {"id": None, "type": None, "work_mode": None,
                        "exec": None}
            # procedure 0 是潜在合法值(协议未注释取值域),不可用 or
            # 兜底吞 0(与 taskExecuStatus 同款 None 检查式)
            pv = getattr(m, "procedure", None)
            task["cloud_proc"] = None if pv is None else int(pv)
            task["fail_code"] = int(getattr(m, "fail_code", 0) or 0)
            fr = str(getattr(m, "fail_reason", "") or "")
            task["fail_reason"] = fr[:40]
            if ti is not None and not task["id"]:
                task["id"] = int(getattr(ti, "task_id", 0) or 0)

        # ---- 规划(plan 模块:期望速度/safety) ----
        plan = None
        m = latest.get("/plan_path_msg") or latest.get("/refer_path_msg")
        if m:
            m = m[0]
            plan = {
                "desire_speed": _r2(getattr(m, "desireSpeed", 0.0)),
                "planspeed": _r2(getattr(m, "planspeed", 0.0)),
                "safety": bool(getattr(m, "safety", True)),
            }

        # ---- 控制(control 模块:转角/制动/油门) ----
        control = None
        m = latest.get("/control_msg")
        if m:
            m = m[0]
            control = {
                "steer": _r2(getattr(m, "wheelAngle", 0.0)),
                "brake": int(getattr(m, "brakePercent", 0) or 0),
                "throttle": int(getattr(m, "throttlePercent", 0) or 0),
                "bia": _r2(getattr(m, "biaDistance", 0.0)),
            }

        # ---- CAN 反馈(can_msg 有效字段) ----
        # 不入栏:throttlePercent/epsCMD/wheelAngleCMD/epsCentring(反馈侧
        # 无写入者恒 0,见 canbus_core.h 字段注释)与 rawcommand/rawfeedback
        # (原始帧字节,调试用)。epsERR1/2 名为 eps 实为 0x285 的挂钩/托盘
        # 位置码(码值越小位置越高,181/122 为最高点),按语义命名输出。
        can = None
        m = latest.get("/can_msg")
        if m:
            m = m[0]
            can = {
                "gear": int(getattr(m, "curGear", 0) or 0),
                "mode": int(getattr(m, "controlPanelState", 0) or 0),
                "estop": int(getattr(m, "emergencyStop", 0) or 0),
                "battery": int(getattr(m, "batteryPower", 0) or 0),
                "hook": int(getattr(m, "hookState", 0) or 0),
                "steer_fb": _r2(getattr(m, "wheelAngle", 0.0)),
                "fault": [int(x) for x in
                          (getattr(m, "faultCode", None) or [])][:8],
                "speed": _r2(getattr(m, "vehicleSpeed", 0.0)),
                "brake_fb": int(getattr(m, "brakePercent", 0) or 0),
                "link_pallet": int(getattr(m, "linkPallet", 0) or 0),
                "eab": int(getattr(m, "eabPanelState", 0) or 0),
                "hook_btn": int(getattr(m, "hookButton", 0) or 0),
                "link_btn": int(getattr(m, "linkButton", 0) or 0),
                "eps_mode": int(getattr(m, "epsMode", 0) or 0),
                "eps_current": _r2(getattr(m, "epsCurrent", 0.0)),
                "pin_pos": int(getattr(m, "epsERR1", 0) or 0),
                "seat_pos": int(getattr(m, "epsERR2", 0) or 0),
            }

        ages = {}
        for topic in list(latest.keys()):
            ages[topic] = self._age(topic, now)
        for topic in self._scan_cfg:
            ages.setdefault(topic, self._age(topic, now))
        for topic in self._cloud_cfg.get("topics", []):
            ages.setdefault(topic, self._age(topic, now))

        with self._lk:
            master_ok = self._master_ok
            import_errors = list(self._msg_import_errors)

        return {
            "ros_available": ROS_AVAILABLE,
            "master_ok": master_ok if ROS_AVAILABLE else None,
            "typed_ok": (not import_errors),
            "msg_import_errors": import_errors,
            "origin": [_r2(ox), _r2(oy)],
            "ages": ages,
            "vehicle": veh,
            "obstacles": obstacles,
            "paths": {"plan": path_of("/plan_path_msg"),
                      "refer": path_of("/refer_path_msg")},
            "stop": stop,
            "pallet": pallet,
            "task": task,
            "plan": plan,
            "control": control,
            "can": can,
        }

    # ---------------- 地图 ----------------

    def map_payload(self):
        """加载全部地图文件(monitor/map/*.csv 逐个绘制)。

        数据源优先级:rosparam /robot/mapfile 显式单文件(只画该张)
        > MAP_PATH 目录(按文件名排序加载全部 .csv)。
        单个文件坏行由 load_map_lines 逐行免疫;整文件失败/空文件跳过。
        """
        if self._map_payload is not None:
            return self._map_payload
        sources = []
        explicit = None
        if ROS_AVAILABLE:
            try:
                p = rospy.get_param("/robot/mapfile", None)
                if p and os.path.isfile(p):
                    explicit = p
            except Exception:
                pass
        if explicit is not None:
            sources = [explicit]
            self._safe_print("[MONITOR] rosparam /robot/mapfile=%s "
                             "覆盖目录模式(仅画该张地图)" % explicit)
        else:
            mdir = self._cfg.get("MAP_PATH", "$MON/map")
            mdir = mdir.replace("$MON", MON_DIR)
            try:
                if os.path.isdir(mdir):
                    # lower():Windows 导出的 .CSV 大写扩展名同样加载
                    sources = sorted(
                        os.path.join(mdir, f) for f in os.listdir(mdir)
                        if f.lower().endswith(".csv"))
                elif os.path.isfile(mdir):  # 兼容:配置指向单个文件
                    sources = [mdir]
            except OSError as exc:
                # 权限/目录竞态:按空目录处理,绝不能让 /api/snapshot
                # (经 _origin)持续失败(实测会每请求重炸且不缓存)
                self._safe_print("[MONITOR] 地图目录不可读 %s: %s"
                                 % (mdir, exc))
                sources = []

        loaded = []
        for src in sources:
            try:
                data = load_map_lines(src)
            except (OSError, ValueError) as exc:
                self._safe_print("[MONITOR] 地图文件跳过 %s: %s"
                                 % (src, exc))
                continue
            if not data["center"]:
                self._safe_print("[MONITOR] 地图文件无有效点,跳过 %s" % src)
                continue
            name = os.path.basename(src)
            self._safe_print("[MONITOR] 地图 %s: %d 点"
                             % (name, len(data["center"])))
            loaded.append((name, data))
        if not loaded and explicit is not None:
            # rosparam 显式文件无有效点:回退目录扫描(车端残留参数指向
            # 空/坏文件时,不能让含有效地图的目录整体失效)
            self._safe_print("[MONITOR] 显式地图 %s 无有效点,回退目录扫描"
                             % explicit)
            try:
                mdir2 = self._cfg.get("MAP_PATH", "$MON/map")
                mdir2 = mdir2.replace("$MON", MON_DIR)
                if os.path.isdir(mdir2):
                    for f2 in sorted(os.listdir(mdir2)):
                        if not f2.lower().endswith(".csv"):
                            continue
                        p2 = os.path.join(mdir2, f2)
                        try:
                            d2 = load_map_lines(p2)
                        except (OSError, ValueError):
                            continue
                        if d2["center"]:
                            loaded.append((f2, d2))
                            self._safe_print("[MONITOR] 地图 %s: %d 点"
                                             % (f2, len(d2["center"])))
            except OSError:
                pass
        if not loaded:
            # 无任何有效地图:干净的空语义(origin 回落 (0,0))
            self._map_payload = {"maps": [], "n": 0, "bbox": [0, 0, 0, 0]}
            return self._map_payload

        # 合并 bbox -> origin(全部地图的整体中心)
        xs0 = min(d["bbox"][0] for _, d in loaded)
        ys0 = min(d["bbox"][1] for _, d in loaded)
        xs1 = max(d["bbox"][2] for _, d in loaded)
        ys1 = max(d["bbox"][3] for _, d in loaded)
        ox = (xs0 + xs1) / 2.0
        oy = (ys0 + ys1) / 2.0

        def line(pts):
            # center 是 (x,y,zg) 三元组,边线是 (x,y) 二元组,统一兼容
            return [[_r2(p[0] - ox), _r2(p[1] - oy)] for p in pts]

        maps = []
        for name, d in loaded:
            maps.append({
                "name": name,
                "center": line(d["center"]),
                "left": line(d["left"]),
                "right": line(d["right"]),
            })
        self._map_payload = {
            "maps": maps,
            "n": len(maps),
            # bbox 保留原始全局坐标(合并全部地图;_origin 依赖它定 origin)
            "bbox": [_r2(xs0), _r2(ys0), _r2(xs1), _r2(ys1)],
        }
        return self._map_payload

    # ---------------- scan.bin ----------------

    def scan_bin(self):
        """两路(或含 extras)LaserScan -> 一帧二进制。按 msg 身份缓存。"""
        with self._lk:
            topics = sorted(self._scan_cfg.keys())
            snapshot_latest = {t: self._latest.get(t) for t in topics}
            exts = {t: dict(self._scan_cfg[t]) for t in topics}
        channels = []
        for idx, topic in enumerate(topics):
            ent = snapshot_latest[topic]
            if not ent:
                continue
            msg = ent[0]
            cache = self._scan_cache.get(topic)
            if cache and cache[0] is msg:
                pts_blob = cache[1]
                count = len(pts_blob) // _BIN_POINT.size
            else:
                pts = scan_to_points(msg, exts[topic])
                pts_blob = b"".join(
                    _BIN_POINT.pack(p[0], p[1], p[2], p[3], 0) for p in pts)
                count = len(pts)
                with self._lk:
                    self._scan_cache[topic] = (msg, pts_blob)
            channels.append((idx + 1, count, pts_blob))

        ts = int(time.time() * 1000) & 0xFFFFFFFFFFFFFFFF
        return pack_bin(channels, ts)

    # ---------------- cloud.bin + 专职线程 ----------------

    def mark_cloud_pull(self):
        self._cloud_last_pull = time.monotonic()

    def cloud_bin(self):
        with self._lk:
            return self._cloud_cache

    def _cloud_run(self):
        """活动门控 + 节拍解析 + 打包缓存(所有 cloud 订阅只在本线程动)。"""
        cfg = self._cloud_cfg
        topics = list(cfg.get("topics", []))
        interval = float(cfg.get("parse_interval", 1.0))
        stride = max(1, int(cfg.get("stride", 2)))
        voxel = float(cfg.get("voxel", 0.2))
        maxpts = int(cfg.get("max_points_per_lidar", 8000))
        timeout = float(cfg.get("activity_timeout", 30))
        zmin = float(cfg.get("z_min", -1.5))
        zmax = float(cfg.get("z_max", 3.0))
        last_parse = {}
        klass = self._load_class("sensor_msgs.msg", "PointCloud2") \
            if ROS_AVAILABLE else None

        while not self._stop.is_set():
            now = time.monotonic()
            active = (self._cloud_last_pull > 0 and
                      now - self._cloud_last_pull < timeout)
            with self._lk:
                master_ok = self._master_ok
                epoch = self._master_epoch

            want = active and master_ok and klass is not None
            # epoch 变了(master 重启)-> 先退订,让重建走同一条路
            if self._cloud_subscribed and (not want or
                                           epoch != self._cloud_epoch_seen):
                for _, s in self._cloud_subs:
                    try:
                        s.unregister()
                    except Exception:
                        pass
                self._cloud_subs = []
                self._cloud_subscribed = False
                # 退订即失效缓存:否则新浏览器首帧拿到陈旧点云,
                # 雷达死掉时旧帧被永久当"当前帧"渲染
                with self._lk:
                    self._cloud_cache = b""
            if want and not self._cloud_subscribed:
                made = 0
                for t in topics:
                    try:
                        self._cloud_subs.append((t, rospy.Subscriber(
                            t, klass, lambda m, tt=t: self._stash(tt, m),
                            queue_size=1, buff_size=2 ** 22,
                            tcp_nodelay=True)))
                        made += 1
                    except Exception:
                        continue
                if made > 0:              # 全部失败则不置位,下轮重试
                    self._cloud_subscribed = True
                    self._cloud_epoch_seen = epoch

            channels = []
            for idx, topic in enumerate(topics):
                if now - last_parse.get(topic, 0.0) < interval:
                    continue
                with self._lk:
                    ent = self._latest.get(topic)
                if ent:
                    # 单路坏数据绝不杀死解析线程;节拍无论成败都推进
                    # (否则坏数据会以 0.2s 周期反复重试)
                    try:
                        msg = ent[0]
                        pts = parse_cloud_points(msg, stride, zmin, zmax)
                        pts = voxelize(pts, voxel, maxpts)
                        chan = [(int(round(p[0] * 1000.0)),
                                 int(round(p[1] * 1000.0)),
                                 _clamp16(p[2] * 1000.0), _clamp_u8(p[3]))
                                for p in pts]
                        blob = b"".join(
                            _BIN_POINT.pack(p[0], p[1], p[2], p[3], 0)
                            for p in chan)
                        channels.append((idx + 1, len(chan), blob))
                    except Exception:
                        pass
                    last_parse[topic] = now
            if channels:
                ts = int(time.time() * 1000) & 0xFFFFFFFFFFFFFFFF
                with self._lk:
                    self._cloud_cache = pack_bin(channels, ts)
            time.sleep(0.2)


def _clamp16(v):
    """钳到 int16 范围(毫米)。"""
    v = int(round(v))
    if v > 32767:
        return 32767
    if v < -32768:
        return -32768
    return v


def _clamp_u8(v):
    try:
        v = int(v)
    except (TypeError, ValueError):
        return 0
    if v < 0:
        return 0
    if v > 255:
        return 255
    return v


class _TimeoutTransport(xmlrpc.client.Transport):
    """带超时的 xmlrpc Transport(构造函数不收 timeout,与 hmi 同款)。"""

    def __init__(self, timeout=1.5):
        xmlrpc.client.Transport.__init__(self)
        self._timeout = timeout

    def make_connection(self, host):
        conn = xmlrpc.client.Transport.make_connection(self, host)
        conn.timeout = self._timeout
        return conn
