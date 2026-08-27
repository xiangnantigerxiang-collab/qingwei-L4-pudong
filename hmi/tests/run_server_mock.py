# -*- coding: utf-8 -*-
"""
以伪 ROS 环境启动 HMI 服务(测试用)。
先安装 mock_ros(伪 rospy/消息包/话题泵),再进入 hmi_server 主流程。
环境变量:ROS_MASTER_URI 指向伪 master;MOCK_CONTROL 指向控制文件。
"""

import os
import sys

HMI_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HMI_DIR)
sys.path.insert(0, TESTS_DIR)

import mock_ros
mock_ros.install()

import hmi_server   # noqa: E402  (必须在 install() 之后导入)

if __name__ == "__main__":
    hmi_server.main()
