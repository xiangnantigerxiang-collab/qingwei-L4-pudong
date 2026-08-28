# -*- coding: utf-8 -*-
"""子进程入口:先安装伪 ROS 再起 monitor 服务(对标 hmi 同名文件)。"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MON = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, MON)

import mock_ros          # noqa: E402
mock_ros.install()

import monitor_server    # noqa: E402  (import 前必须已 install)

if __name__ == "__main__":
    sys.exit(monitor_server.main())
