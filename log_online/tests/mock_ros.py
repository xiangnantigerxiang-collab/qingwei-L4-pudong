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


def _msg_module(name, fields):
    """robot.msg.<name> 的容器模块:测试统一按 `from robot.msg import can_msg
    as can_mod` + `can_mod.can_msg(...)` 构造消息(plan 各任务测试段同此用法),
    故 <name> 属性须挂在容器上而非直接暴露消息类。"""
    mod = types.ModuleType("robot.msg." + name)
    setattr(mod, name, _msg_class(fields))
    return mod


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


class _Duration(object):
    def __init__(self, secs=0):
        self._secs = float(secs)

    def to_sec(self):
        return self._secs


class _Timer(object):
    def __init__(self, period_secs, callback):
        # 与 rospy.Timer 一致:周期须提供 to_sec(),不能直接传 float。
        self.period_secs = period_secs.to_sec()
        self.callback = callback


class _FakeRospy(object):
    Duration = _Duration

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
    robot_msg.can_msg = _msg_module("can_msg", {
        "controlPanelState": 0, "eabPanelState": 0, "curGear": 0,
        "vehicleSpeed": 0.0, "emergencyStop": 0, "faultCode": b""})
    robot_msg.navigation_msg = _msg_module("navigation_msg", {
        "lat": 0.0, "lon": 0.0, "altitude": 0.0, "heading": 0.0,
        "xAxis": 0.0, "yAxis": 0.0, "zAxis": 0.0, "gpsSpeed": 0.0,
        "longitudinal_accelerate": 0.0, "rtkState": ""})
    robot_msg.object = _msg_module("object", {
        "id": 0, "type": 0, "x": 0.0, "y": 0.0, "dx": 0.0, "dy": 0.0,
        "heading": 0.0, "height": 0.0, "vx": 0.0, "vy": 0.0, "confidence": 0.0})
    robot_msg.perception = _msg_module("perception", {"objs": []})
    robot_msg.path_plan_msg = _msg_module("path_plan_msg", {"desireSpeed": 0.0, "safety": False})
    robot_msg.control_msg = _msg_module("control_msg", {
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
