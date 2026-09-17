# -*- coding: utf-8 -*-
"""
伪 ROS 环境(仅测试用,车载不部署)——对标 hmi/tests/mock_ros.py 精简版
=====================================================================

在 monitor 进程内安装假的 rospy / 消息包,把 ros_visualizer.py 的 ROS 侧
代码"骗"进可运行状态:

- fake rospy:init_node 探活 ROS_MASTER_URI 指向的伪 master;
  Subscriber 登记进注册表(支持 queue_size/buff_size/tcp_nodelay 参数)
- 话题泵:后台线程按控制文件 rates 触发回调;fields 支持:
    普通字段      {"xAxis": 12.3, ...}
    LaserScan     {"__scan__": {"ranges": [...], ...}}
    PointCloud2   {"__pc2__": [[x,y,z,i], ...], "__pc2_opt__":
                    {"bigendian":true, "dtype":{...}, "offsets":{...},
                     "point_step":32, "organized":2}}
- 伪消息包:robot.msg / sensor_msgs.msg 占位类(类型化订阅路径)
- 伪 master:XML-RPC 子集(getPid/getSystemState),pid 可变可停启

控制文件(JSON,MOCK_CONTROL 环境变量,0.5s 重载):
{
  "master_alive": true,
  "rates":  {"/navigation_msg": 50, ...},
  "fields": {"/navigation_msg": {"xAxis": 12.3}, ...}
}
"""

import json
import math
import os
import struct
import threading
import time
import types
import xmlrpc.client
import xmlrpc.server

_CONTROL_PATH = os.environ.get("MOCK_CONTROL",
                               "/tmp/monitor_mock_control.json")

DEFAULT_FIELDS = {
    "/navigation_msg": {"xAxis": 55.94, "yAxis": -12.58, "heading": 274.59,
                        "gpsSpeed": 2.5},
    "/perception": {"objs": [
        {"id": 1, "type": 0, "x": 58.0, "y": -10.0, "dx": 1.0, "dy": 4.0,
         "heading": 100.0, "height": 1.8, "vx": 0.0, "vy": 0.0,
         "confidence": 0.87},
    ]},
    "/plan_path_msg": {"x": [55.0, 56.0, 57.0], "y": [-12.0, -12.1, -12.2],
                       "desireSpeed": 1.8, "planspeed": 1.6,
                       "safety": False},
    "/refer_path_msg": {"x": [55.0, 56.0], "y": [-12.0, -12.1]},
    "/path_plan_status": {"stopX": 60.0, "stopY": -11.0, "stopAngle": 90.0,
                          "taskExecuStatus": 1},
    "/palletpos": {"xg": 59.0, "yg": -13.0},
    "/task_plan_msg": {"task_id": 88, "taskType": 1, "workMode": 1},
    "/cloud/task/task_status": {"procedure": 1, "fail_code": 0,
                                "fail_reason": "",
                                "task_info.task_id": 88},
    "/control_msg": {"wheelAngle": -11.0, "brakePercent": 0,
                     "throttlePercent": 18, "biaDistance": 0.21},
    # 闸机口(gantry_detect 包);未设 rates 时不泵,现有用例零影响
    "/gantry_state": {"active": True, "gantry_open": False},
    "/can_msg": {"curGear": 4, "controlPanelState": 1, "emergencyStop": 0,
                 "batteryPower": 77, "hookState": 1, "faultCode": [0],
                 "wheelAngle": -220.0, "vehicleSpeed": 1.2,
                 "brakePercent": 30, "linkPallet": 1,
                 # eab/link_btn 取非零值:与 getattr 默认 0 错开,
                 # 字段名拼错(静默回落默认值)时快照可见地变化
                 "eabPanelState": 1,
                 "hookButton": 1, "linkButton": 2, "epsMode": 3,
                 # hookPos/palletPos 为现役位置码字段(epsERR1/2 已悬空
                 # 不写,canbus 侧恒 0,故 mock 不提供=回落 0)
                 "epsCurrent": -4.75, "hookPos": 185, "palletPos": 130},
    # dashboard(8082)用;未设 rates 时不泵,现有用例零影响
    "/ehb_msg": {"EHB_STO_StorageSystemStatus": 1, "EHB_STO_FailureNum": 0,
                 "EHB_ATB_ActualBrakePressure": 100,
                 "EHB_ATB_AutoBrakeSystemStatus": 1,
                 "EHB_EPB_SystemStatus": 0, "EHB_EPB_ParkingStatus": 5,
                 "EHB_EPB_ParkingPressure": 150,
                 "EHB_STO_FAU_Sensor1Error": True,
                 "VHL_ATB_BrakePressRequest": 150,
                 "VHL_ATB_BrakePressRequestFlag": 1,
                 "VHL_VehicleSpeed": 600, "VHL_HvPowerState": 1,
                 "VHL_BrakePedalStatus": 64, "VHL_VehicleGear": 0x44,
                 # 其余 34 个 FAU 位补 False:对齐 catkin 生成类"全字段必有"
                 # 的真实形态(dashboard 故障折叠按属性存在性判可读,
                 # mock 只给部分位会被判"无数据")
                 "EHB_STO_FAU_PowerSupplyVoltageHigh": False,
                 "EHB_STO_FAU_PowerSupplyVoltageLow": False,
                 "EHB_STO_FAU_SensorSupplyError": False,
                 "EHB_STO_FAU_Sensor2Error": False,
                 "EHB_STO_FAU_Sensor3Error": False,
                 "EHB_STO_FAU_SensorJudgmentFailure": False,
                 "EHB_STO_FAU_MotorOpenLoad": False,
                 "EHB_STO_FAU_MotorCannotStop": False,
                 "EHB_STO_FAU_MotorWorkTimeout": False,
                 "EHB_ATB_FAU_PowerSupplyVoltageHigh": False,
                 "EHB_ATB_FAU_PowerSupplyVoltageLow": False,
                 "EHB_ATB_FAU_CanBusOff": False,
                 "EHB_ATB_FAU_SensorSupplyError": False,
                 "EHB_ATB_FAU_MotorDriverError": False,
                 "EHB_ATB_FAU_BrakePressSensorError": False,
                 "EHB_ATB_FAU_LosOfEHBSTOCom": False,
                 "EHB_ATB_FAU_MotorCommuteFail": False,
                 "EHB_ATB_FAU_MotorOpenLoad": False,
                 "EHB_ATB_FAU_MotorShort": False,
                 "EHB_ATB_FAU_MotorStall": False,
                 "EHB_ATB_FAU_ValveBodyError": False,
                 "EHB_ATB_FAU_LosOfBrakeReqSignal": False,
                 "EHB_ATB_FAU_BrakeReqRcError": False,
                 "EHB_ATB_FAU_BrakeReqCsError": False,
                 "EHB_EPB_FAU_PowerSupplyVoltageHigh": False,
                 "EHB_EPB_FAU_PowerSupplyVoltageLow": False,
                 "EHB_EPB_FAU_NoValueError": False,
                 "EHB_EPB_FAU_NcValueError": False,
                 "EHB_EPB_FAU_SensorSupplyError": False,
                 "EHB_EPB_FAU_ParkingPresSensorError": False,
                 "EHB_EPB_FAU_ParkingPressureLow": False,
                 "EHB_EPB_FAU_LosOfParkingReqSignal": False,
                 "EHB_EPB_FAU_ParkingReqRcError": False,
                 "EHB_EPB_FAU_ParkingReqCsError": False},
    "/back_left_scan": {"__scan__": {
        "amin": -math.pi, "ainc": math.pi / 4, "rmin": 0.1, "rmax": 100.0,
        "ranges": [2.0, 2.0, float("inf"), 0.0, -1.0, 2.0, 2.0, 2.0],
        "intensities": [10, 20]}},
    "/back_right_scan": {"__scan__": {
        "amin": -math.pi, "ainc": math.pi / 4, "rmin": 0.1, "rmax": 100.0,
        "ranges": [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
        "intensities": []}},
    "/rslidar_points_mid": {"__pc2__": [
        [1.0, 2.0, 0.5, 100], [1.1, 2.0, 0.5, 90], [3.0, 4.0, 0.6, 80]]},
}


def read_control():
    try:
        with open(_CONTROL_PATH, "r") as f:
            return json.load(f)
    except Exception:
        return {}


# ---------------------------------------------------------------- 消息对象

class _Msg(object):
    def __init__(self, fields=None):
        for k, v in (fields or {}).items():
            setattr(self, k, v)


class _ObjField(object):
    """PointField 占位(name/offset/datatype/count)。"""
    def __init__(self, name, offset, datatype, count=1):
        self.name = name
        self.offset = offset
        self.datatype = datatype
        self.count = count


def _build_scan(spec):
    m = _Msg()
    m.angle_min = float(spec.get("amin", -math.pi))
    m.angle_max = float(spec.get("amax", math.pi))
    m.angle_increment = float(spec.get("ainc", math.pi / 180))
    m.range_min = float(spec.get("rmin", 0.1))
    m.range_max = float(spec.get("rmax", 100.0))
    m.ranges = [float(r) for r in spec.get("ranges", [])]
    ints = spec.get("intensities")
    m.intensities = [float(i) for i in ints] if ints is not None else []
    m.header = _Msg({"frame_id": "laser"})
    return m


def _build_pc2(spec, opt):
    """points [[x,y,z,i]...] -> PointCloud2(bytes data)。

    opt 可覆盖:bigendian / point_step / organized(行数) /
    offsets {name: off} / dtype {name: code}(7=float32,4=uint16,6=uint32)
    """
    opt = opt or {}
    big = bool(opt.get("bigendian", False))
    step = int(opt.get("point_step", 16))
    offs = dict(opt.get("offsets", {}) or {})
    dts = dict(opt.get("dtype", {}) or {})
    names = ("x", "y", "z", "intensity")
    dflt_off = {"x": 0, "y": 4, "z": 8, "intensity": 12}
    dflt_dt = {"x": 7, "y": 7, "z": 7, "intensity": 7}
    fields = []
    for n in names:
        off = int(offs.get(n, dflt_off[n]))
        dt = int(dts.get(n, dflt_dt[n]))
        fields.append(_ObjField(n, off, dt))
    end = ">" if big else "<"
    fmtc = {7: ("f", 4), 4: ("H", 2), 6: ("I", 4), 8: ("d", 8)}

    rows = int(opt.get("organized", 1)) or 1
    pts = spec
    per_row = (len(pts) + rows - 1) // rows
    data = bytearray()
    for r in range(rows):
        for k in range(per_row):
            idx = r * per_row + k
            # 填充行用 NaN:产品端会过滤,不产生 (0,0,0) 幻影点
            # (真实 organized 云没有 padding 概念,这是 mock 的人为行)
            nan4 = (float("nan"), float("nan"), float("nan"), 0.0)
            p = pts[idx] if idx < len(pts) else nan4
            row = bytearray(step)
            for i2, n in enumerate(names):
                f = fields[i2]
                ch, sz = fmtc[f.datatype]
                val = float(p[i2])
                if ch in ("b", "h", "i"):       # 有符号整数格式
                    val = int(round(val))
                elif ch in ("B", "H", "I"):     # 无符号整数格式
                    val = int(round(val)) & {
                        "B": 0xFF, "H": 0xFFFF, "I": 0xFFFFFFFF}[ch]
                try:
                    struct.pack_into(end + ch, row, f.offset, val)
                except struct.error:
                    pass
            data += row
    m = _Msg()
    m.fields = fields
    m.point_step = step
    m.row_step = step * per_row
    m.height = rows
    m.width = per_row
    m.is_bigendian = big
    m.data = bytes(data)
    m.header = _Msg({"frame_id": "rslidar"})
    return m


def build_msg(topic, override):
    merged = dict(DEFAULT_FIELDS.get(topic, {}))
    merged.update(override or {})
    spec = merged.get("__pc2__")
    if spec is not None:
        return _build_pc2(spec, merged.get("__pc2_opt__"))
    spec = merged.get("__scan__")
    if spec is not None:
        return _build_scan(spec)
    if topic == "/perception":
        objs = [_Msg(o) for o in merged.get("objs", [])]
        m = _Msg()
        m.objs = objs
        m.header = _Msg({})
        return m
    m = _Msg(merged)
    if topic == "/cloud/task/task_status":
        m.task_info = _Msg({"task_id": merged.get("task_info.task_id", 0)})
    return m


# ---------------------------------------------------------------- 假 rospy

_REGISTRY = {}
_REG_LK = threading.Lock()


class _FakeRoSpy(object):
    ERROR = 40
    AnyMsg = _Msg

    def init_node(self, name, disable_signals=False, anonymous=False,
                  log_level=None):
        # 探活伪 master(真 xmlrpc 调用;master 不在则快速失败触发重试)
        uri = os.environ.get("ROS_MASTER_URI", "http://127.0.0.1:11311")
        try:
            proxy = xmlrpc.client.ServerProxy(uri)
            proxy.getPid("/mock")
        except Exception:
            raise RuntimeError("mock master not reachable at %s" % uri)

    def is_initialized(self):
        return True

    def Subscriber(self, topic, msg_type, cb, queue_size=None,
                   buff_size=None, tcp_nodelay=None):
        with _REG_LK:
            _REGISTRY.setdefault(topic, []).append(cb)
        return _FakeSub(topic, cb)


def _conn_with_timeout(host):
    import http.client
    conn = http.client.HTTPConnection(host, timeout=2)
    return conn


class _FakeSub(object):
    def __init__(self, topic, cb):
        self.topic = topic
        self.cb = cb
        self._unreg = False

    def unregister(self):
        self._unreg = True
        with _REG_LK:
            lst = _REGISTRY.get(self.topic, [])
            if self.cb in lst:
                lst.remove(self.cb)


def _pump_loop():
    acc = {}
    while True:
        ctl = read_control()
        rates = ctl.get("rates", {})
        fields = ctl.get("fields", {})
        for topic, cbs in list(_REGISTRY.items()):
            hz = float(rates.get(topic, 0) or 0)
            if hz <= 0 or not cbs:
                continue
            acc[topic] = acc.get(topic, 0.0) + hz * 0.1
            n = int(acc[topic])
            if n <= 0:
                continue
            acc[topic] -= n
            msg = build_msg(topic, fields.get(topic))
            for cb in list(cbs):
                try:
                    cb(msg)
                except Exception:
                    pass
        time.sleep(0.1)


def install():
    rospy_mod = types.ModuleType("rospy")
    fake = _FakeRoSpy()
    for attr in ("init_node", "is_initialized", "Subscriber", "AnyMsg"):
        setattr(rospy_mod, attr, getattr(fake, attr))
    rospy_mod.ERROR = fake.ERROR
    rospy_mod.get_param = lambda *a, **kw: None
    rospy_mod.signal_shutdown = lambda *a: None
    sys_mods = __import__("sys").modules
    sys_mods["rospy"] = rospy_mod

    for pkg, cls_names in (
            ("robot.msg", ["navigation_msg", "perception", "path_plan_msg",
                           "path_plan_status", "palletpos",
                           "task_plan_msg", "TaskStatus", "control_msg"]),
            ("canbus.msg", ["can_msg", "ehb_msg"]),
            ("gantry_detect.msg", ["gantry_state"]),
            ("sensor_msgs.msg", ["LaserScan", "PointCloud2"])):
        parts = pkg.split(".")
        parent = types.ModuleType(parts[0])
        parent.__path__ = []
        mod = types.ModuleType(pkg)
        for cn in cls_names:
            setattr(mod, cn, type(cn, (_Msg,), {}))
        setattr(parent, parts[1], mod)
        sys_mods[parts[0]] = parent
        sys_mods[pkg] = mod

    t = threading.Thread(target=_pump_loop, daemon=True)
    t.start()


# ---------------------------------------------------------------- 伪 master

class FakeMaster(object):
    """标准 master XML-RPC 子集;pid/死活由控制文件驱动(测重启重连)。

    控制文件键:
      "master_pid": int     -> getPid 返回该 pid(pid 变化触发被测端重建订阅)
      "master_alive": false -> getPid 抛异常(模拟 master 失联)
    """

    def __init__(self, port=11311):
        self.port = port
        self.pid = 4242
        self.srv = xmlrpc.server.SimpleXMLRPCServer(
            ("127.0.0.1", port), logRequests=False, allow_none=True)
        self.srv.register_function(self._getPid, "getPid")
        self.srv.register_function(self._getSystemState, "getSystemState")
        self._th = threading.Thread(target=self.srv.serve_forever,
                                    daemon=True)

    def _getPid(self, caller):
        ctl = read_control()
        if ctl.get("master_alive") is False:
            raise Exception("master down (mock)")
        if "master_pid" in ctl:
            self.pid = int(ctl["master_pid"])
        return [1, "", self.pid]

    def _getSystemState(self, caller):
        return [1, "", [[], [], []]]

    def start(self):
        self._th.start()

    def stop(self):
        self.srv.shutdown()
