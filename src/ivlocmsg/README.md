# ivlocmsg 定位消息包

核对：2026-09-19。本包提供ROS1定位/惯导相关消息，仍被工程使用，不是独立运行节点。
较早清理已将其列入保护清单，不能因没有main或可执行文件删除。

## 使用方法

在ROS1工程根执行 `catkin_make --pkg ivlocmsg -j4`，然后 `source devel/setup.bash`。
使用 `rosmsg show ivlocmsg/ivmsglocpos` 查看当前生成的定位契约。
C++消费者包含生成头并在catkin/package.xml声明依赖；消息源位于 `msg/`。
不要手工复制生成头替代catkin消息生成，也不要寻找不存在的本包launch节点。

## 注意事项与容易疏忽的点

- 字段名、类型、顺序都是ROS MD5的一部分；改动前搜索发布者、订阅者与同名副本。
- xAxis/yAxis/zAxis与heading的坐标/角度含义由生产者契约决定，不凭字段名猜单位。
- PNC本机测试的消息桩从真实.msg生成，不能用简化手写桩掩盖字段差异。
- 跨架构部署重新生成/构建消费者，不能把开发机产物直接当车端已兼容。
