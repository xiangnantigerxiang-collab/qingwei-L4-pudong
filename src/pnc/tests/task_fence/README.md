# 电子围栏迁移修复验证

纯主机ROS桩，测试使用临时CSV/YAML；不运行ROS节点或改业务地图。

```bash
python3 src/pnc/tests/compile_all.py --output /tmp/pnc-fence-build
python3 src/pnc/tests/task_fence/verify.py --build /tmp/pnc-fence-build --output /tmp/pnc-fence-check
python3 -B hmi/tests/test_fence_alarm.py
node hmi/tests/frontend_test.js
```

59项联调断言与原六组围栏专项通过，core单独C++11编译；规划全量桩因主机Boost使用C++14，生产标准仍为C++11。
覆盖失败关闭、截停后取消作业、原角点保护、连续线段、缓存失效、独立许可与反馈的任务版本、planning最终输出、安全条件隔离以及正常D/R输出对照。
原专项将旧固定第365点断言改为前角有余量/不超过首次越界点；不再把92点路线的主机耗时阈值作为车载性能保证。

本轮另跑 control D制动13462、状态9764、曲率17609项；非D220场景8925条完整消息对照；
地图限速/闸机/末停/感知/起步/控制交接及CSV ASan/UBSan专项通过。
完整日志、哈希、生产19文件部署包：工程同级 `pnc_fence_fix_delivery_20260922/`。
修改前备份：工程同级 `pnc_fence_fix_before_20260922_092231.tar.gz`。
无实机ROS/实车验收；保护模型与部署边界见 [模块说明](../../src/robot_task_plan/README.md)。
