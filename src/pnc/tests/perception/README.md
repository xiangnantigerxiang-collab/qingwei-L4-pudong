# 感知测试入口与历史HDMap接入夹具

更新：2026-09-19。本目录是09-15仅接入HDMap阶段的测试，当前转换链已加入空观测历史、
跟踪速度与专用规划输出，因此旧夹具不再代表完整现行发布语义。

## 使用方法

现行发布/跟踪验证，在工程根执行：

```bash
python3 src/pnc/src/robot_perception_convert/tests/verify_publish.py --output /tmp/perception-publish-check
python3 src/pnc/src/robot_perception_convert/tests/verify_tracking.py --output /tmp/perception-tracking-check
```

仅在复现旧HDMap接入阶段、并使用对应源码基线时执行本目录入口：

```bash
python3 src/pnc/tests/perception/verify.py --output /tmp/pnc-hdmap-historical-check
```

默认链接 `src/hdmap/sdk/lib/libhdmap_server.so`，`--hdmap-sdk`可指定已安装SDK。
SDK必须先按 [HDMap README](../../../hdmap/README.md) 在本机架构构建；消息桩从真实.msg生成。

## 注意事项与容易疏忽的点

- 旧71项检查包含“空MarkerArray不发布”期望，已被后续有效空观测入窗需求取代；
  不要把旧计数称为当前整链通过，也不要为跑通旧测试回退当前业务。
- 使用真实SDK与消息桩验证API，不包括ROS1传输、Orin构建或车辆响应。
- x86_64 SDK不能复制给aarch64运行；变更地图后需要重启消费者加载新缓存。
- 输出目录只放临时日志和result.json，不改用户地图或标定。

详细现行规则见 [转换模块](../../src/robot_perception_convert/README.md)。
09-15原始验证过程见 [归档](../../../../docs/history/2026-09-19-before-docs/src/pnc/tests/perception/README.md)。
