# 感知发布前车道分类校验

在工程根执行：

```bash
python3 src/pnc/tests/perception/verify.py --output /tmp/pnc_hdmap_publish_verify
```

默认链接 `src/hdmap/sdk/lib/libhdmap_server.so`，可用 `--hdmap-sdk` 指定其他已安装 SDK。
SDK 必须先按 HDMap README 在本机架构编译安装。

测试从真实 `robot/*.msg` 生成消息桩，CMake 编译实际 `perception_msg_convert` 目标。
ROS 的传输、包目录和 catkin 消息生成用桩模拟；分类计算使用真实动态库。
SDK 的命名空间导出目标和 catkin 的传统 include/library 变量分别编译、检查动态链接。

`integration.cpp` 直接包含生产转换节点，捕获调用 `publish` 时传入的消息副本。
71 项检查覆盖五种 type、跨车道最大覆盖、逆向行驶、自车离道、不同框朝向、
排除区域与 CUBE 过滤、原有空输入行为、错误帧停止发布与恢复、启动地图目录覆盖，
以及加载后移开 CSV 仍能查询。脚本同时检查包依赖、重建顺序和脚本语法。

日志及 `result.json` 写入指定输出目录。本机桩验证不包含 ROS1 通信和车载运行。
