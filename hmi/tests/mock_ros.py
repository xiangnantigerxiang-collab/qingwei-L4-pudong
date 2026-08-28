# -*- coding: utf-8 -*-
"""
伪 ROS 环境(仅测试用,车载不部署)
====================================

在 HMI 进程内安装一套假的 rospy / 消息包,把 ros_bridge.py 的 ROS 侧代码
"骗"进可运行状态:

- fake rospy:init_node 会真实探活 ROS_MASTER_URI 指向的伪 master;
  Subscriber 登记进注册表;get_param 读控制文件
- 话题泵:后台线程按控制文件里各话题的 hz 定时触发订阅回调,
  消息字段来自控制文件(支持 "values.soc" 点路径覆盖嵌套对象)
- 伪消息包:canbus.msg / robot.msg / auto_couple.msg 占位类,
  让 _load_class 走"类型化订阅"路径
- 伪 master(在测试驱动进程内运行):标准 ROS master XML-RPC 子集
  (getPid / getSystemState),pid 可变(测重启重连),可停启(测失联)

控制文件( JSON,测试驱动进程写入,本进程 0.5s 重载):
{
  "master_alive": true,
  "rates":   {"/can_msg": 50, ...},
  "fields":  {"/can_msg": {"vehicleSpeed": 2.5, ...}, ...},
  "params":  {"/planning/sensorstate": 0, ...}
}
"""

import json
import os
import threading
import time
import types
import xmlrpc.client
import xmlrpc.server

_CONTROL_PATH = os.environ.get("MOCK_CONTROL", "/tmp/hmi_mock_control.json")

# 各话题的消息字段默认值(测试驱动可用控制文件覆盖任意字段)
DEFAULT_FIELDS = {
    "/can_msg": {"vehicleSpeed": 2.5, "curGear": 4, "controlPanelState": 1,
                 "emergencyStop": 0, "faultCode": [], "hookState": 1},
    "/navigation_msg": {"xAxis": 12.34, "yAxis": -5.67, "heading": 91.2,
                        "localization_status": 0},
    "/path_plan_status": {"taskExecuStatus": 1, "distance2Object": 120.0,
                          "distance2Stop": 25.0},
    "/control_msg": {"biaDistance": 0.21, "biaAngle": 0.02},
    "/task_plan_msg": {"task_id": 88, "taskType": 1, "stopX": 3.5, "stopY": -2.0},
    "/cloud/task/task_status": {"procedure": 1, "fail_code": 0, "fail_reason": "",
                                 "task_info.task_id": 88},
    # status 按真实 v2nHeartBeat.msg 放在 values 内(顶层仅 ts/deviceId/type/values)
    "/v2nHeartBeat": {"values.status": "RUNNING", "values.speed": 9.0,
                      "values.soc": 77, "values.hookState": 4,
                      "values.vehicleState": 0, "values.lidarState": 0,
                      "values.cameraState": 0, "values.gnssState": 0,
                      "values.drivingState": 3},
    "/hook_position": {"center_point_x": 0.5, "center_point_y": -0.2,
                       "center_distance": 1.8, "beta": 3.2},
    "/localization": {}, "/rslidar_points_mid": {}, "/rslidar_points_front": {},
    "/rslidar_points_left": {}, "/rslidar_points_right": {},
    "/can_recv": {}, "/box": {}, "/back_pointcloud": {},
    "/perception_back_bbox": {}, "/cam0/compressed": {},
    "/robot/simviewer/vehicle": {},
}

DEFAULT_PARAMS = {"/planning/sensorstate": 0, "/planning/alive": 1,
                  "/robot/planning/netcheck": 0, "/alarmcmd": 0}


def read_control():
    try:
        with open(_CONTROL_PATH, "r") as f:
            return json.load(f)
    except Exception:
        return {}


class _TimeoutTransport(xmlrpc.client.Transport):
    """带超时的 xmlrpc 传输(Transport 构造函数不收 timeout)。"""

    def __init__(self, timeout=1.5):
        xmlrpc.client.Transport.__init__(self)
        self._timeout = timeout

    def make_connection(self, host):
        conn = xmlrpc.client.Transport.make_connection(self, host)
        conn.timeout = self._timeout
        return conn


# ---------------------------------------------------------------- 消息对象

class _Msg(object):
    def __init__(self, fields=None):
        for k, v in (fields or {}).items():
            try:
                self._set_nested(k, v)
            except AttributeError:
                pass

    def _set_nested(self, dotted, val):
        obj = self
        for p in dotted.split(".")[:-1]:
            obj = getattr(obj, p)
        setattr(obj, dotted.split(".")[-1], val)


class AnyMsg(_Msg):
    pass


def build_msg(topic, override_fields):
    """构造带嵌套骨架的消息对象,再用字段表覆盖。"""
    m = _Msg()
    if topic == "/v2nHeartBeat":
        m.values = _Msg({"status": "", "speed": 0, "soc": 0, "hookState": None,
                         "vehicleState": 0, "lidarState": None,
                         "cameraState": None, "gnssState": None,
                         "drivingState": 0})
    elif topic == "/cloud/task/task_status":
        m.task_info = _Msg({"task_id": None})
    merged = dict(DEFAULT_FIELDS.get(topic, {}))
    merged.update(override_fields or {})
    _Msg.__init__(m, merged)
    return m


# ---------------------------------------------------------------- 订阅注册表与话题泵

_REGISTRY = {}
_REGISTRY_LOCK = threading.Lock()


class Subscriber(object):
    def __init__(self, topic, data_class, callback=None,
                 callback_args=None, queue_size=None):
        self.topic = topic
        self.callback = callback
        self.callback_args = callback_args
        with _REGISTRY_LOCK:
            _REGISTRY.setdefault(topic, []).append(self)

    def unregister(self):
        with _REGISTRY_LOCK:
            lst = _REGISTRY.get(self.topic, [])
            if self in lst:
                lst.remove(self)


def subscriber_count(topic=None):
    with _REGISTRY_LOCK:
        if topic is None:
            return sum(len(v) for v in _REGISTRY.values())
        return len(_REGISTRY.get(topic, []))


class _Pump(threading.Thread):
    """按控制文件的 rates 定时触发订阅回调。"""

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)
        self._acc = {}
        self._rates = {}
        self._fields = {}
        self._last_reload = 0.0

    def run(self):
        tick = 0.1
        while True:
            time.sleep(tick)
            now = time.time()
            if now - self._last_reload > 0.5:
                c = read_control()
                self._rates = c.get("rates", {})
                self._fields = c.get("fields", {})
                self._last_reload = now
            with _REGISTRY_LOCK:
                topics = list(_REGISTRY.items())
            for topic, subs in topics:
                hz = float(self._rates.get(topic, 0) or 0)
                if hz <= 0 or not subs:
                    continue
                self._acc[topic] = self._acc.get(topic, 0.0) + hz * tick
                fires = 0
                while self._acc.get(topic, 0) >= 1.0 and fires < 30:
                    self._acc[topic] -= 1.0
                    fires += 1
                    msg = build_msg(topic, self._fields.get(topic, {}))
                    for s in list(subs):
                        try:
                            if s.callback_args is not None:
                                s.callback(msg, s.callback_args)
                            else:
                                s.callback(msg)
                        except Exception:
                            pass


# ---------------------------------------------------------------- fake rospy

class _FakeRoSpy(object):
    ERROR = 40

    def __init__(self):
        self._initialized = False

    def init_node(self, name, disable_signals=False, anonymous=False,
                  log_level=None):
        uri = os.environ.get("ROS_MASTER_URI", "http://127.0.0.1:11311")
        try:
            tr = _TimeoutTransport(1.5)
            code, _m, _pid = xmlrpc.client.ServerProxy(uri, transport=tr).getPid(
                "/mock_init")
            if code != 1:
                raise RuntimeError("mock master code=%s" % code)
        except Exception as exc:
            raise RuntimeError("mock: master 不可达 (%s)" % exc)
        self._initialized = True

    def is_initialized(self):
        return self._initialized

    def Subscriber(self, topic, data_class, callback=None,
                   callback_args=None, queue_size=None):
        return Subscriber(topic, data_class, callback, callback_args, queue_size)

    def get_param(self, name, default=None):
        c = read_control()
        if not c.get("master_alive", True):
            raise RuntimeError("mock: master 不可达")
        return c.get("params", {}).get(name, default)

    def signal_shutdown(self, reason=""):
        self._initialized = False


def install():
    """在当前进程安装伪 rospy 与伪消息包。必须在 import hmi_server 之前调用。"""
    spy_mod = types.ModuleType("rospy")
    impl = _FakeRoSpy()
    spy_mod.init_node = impl.init_node
    spy_mod.is_initialized = impl.is_initialized
    spy_mod.Subscriber = impl.Subscriber
    spy_mod.get_param = impl.get_param
    spy_mod.signal_shutdown = impl.signal_shutdown
    spy_mod.AnyMsg = AnyMsg
    spy_mod.ERROR = _FakeRoSpy.ERROR

    msg_sets = {
        "canbus.msg": {"can_msg": type("can_msg", (), {})},
        "robot.msg": {"navigation_msg": type("navigation_msg", (), {}),
                      "path_plan_status": type("path_plan_status", (), {}),
                      "control_msg": type("control_msg", (), {}),
                      "task_plan_msg": type("task_plan_msg", (), {}),
                      "TaskStatus": type("TaskStatus", (), {}),
                      "v2nHeartBeat": type("v2nHeartBeat", (), {})},
        "auto_couple.msg": {"center_position": type("center_position", (), {})},
    }
    import sys
    for pkg_name, classes in msg_sets.items():
        mod = types.ModuleType(pkg_name)
        for cls_name, cls in classes.items():
            setattr(mod, cls_name, cls)
        sys.modules[pkg_name] = mod
        parent_name = pkg_name.split(".")[0]
        if parent_name not in sys.modules:
            parent = types.ModuleType(parent_name)
            parent.__path__ = []
            sys.modules[parent_name] = parent
        setattr(sys.modules[parent_name], "msg", mod)

    sys.modules["rospy"] = spy_mod
    _Pump().start()


# ---------------------------------------------------------------- 伪 ROS master
# (在测试驱动进程内使用,不在 HMI 进程内)

CLOUD_NODES = ["/cloud_task_publisher", "/cloud_taskstatus_subscriber",
               "/cloud_routing_server", "/cloud_command_publisher",
               "/cloud_command_status_subscriber", "/cloud_heatbeat_subscriber"]


class FakeMaster(object):
    """标准 ROS master XML-RPC 子集。pid 可改、服务可停启,用于测重连与失联。"""

    def __init__(self, port=None):
        self.pid = 4200
        self.cloud_count = len(CLOUD_NODES)
        self._srv = None
        self._th = None
        self.port = port
        self._bind()

    def _bind(self):
        self._srv = xmlrpc.server.SimpleXMLRPCServer(
            ("127.0.0.1", self.port or 0), logRequests=False)
        self._srv.register_function(self._getPid, "getPid")
        self._srv.register_function(self._getSystemState, "getSystemState")
        self.port = self._srv.server_address[1]
        self._th = threading.Thread(target=self._srv.serve_forever, daemon=True)

    def start(self):
        self._th.start()

    def _getPid(self, caller_id):
        return [1, "ok", self.pid]

    def _getSystemState(self, caller_id):
        nodes = CLOUD_NODES[:self.cloud_count]
        pubs = [["/cloud/task/task_info", nodes],
                ["/cloud/task/task_status", nodes],
                ["/can_msg", ["/mock_canbus"]]]
        return [1, "ok", [pubs, [], []]]

    def restart_new_pid(self):
        """停掉再以同端口新实例启动,pid+1,模拟 master 重启。"""
        port = self.port
        self._srv.shutdown()
        self._srv.server_close()
        time.sleep(0.6)
        self.pid += 1
        self.port = port
        self._bind()
        self.start()

    def stop(self):
        self._srv.shutdown()
        self._srv.server_close()
