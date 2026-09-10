# CLAUDE.md — 项目级指令（每次会话自动加载）

## 强制规则：pnc 代码工作前必读风格规范

**任何涉及 `src/pnc/` 的代码阅读、修改、新增、清理、重构任务，开始前必须先完整阅读
`src/pnc/README.md`（pnc 编码风格与习惯规范），并全程遵循其中约定。**

该 README 由 2026-08-30 全量风格采样提炼，核心三条（详见 README）：

1. **新增代码风格必须与所在文件的上下文一致**——robot_control/ 与 robot_task_plan/
   已 Google 化（2 空格，后者 08-30 随 core/node 解耦重写），其余节点目录是历史格式
   （4 空格 Allman），不要混代
2. **不重命名任何既有标识符**（包括拼写错误）；清理/格式化绝不改业务逻辑，
   删除类改动需逐项 grep 实证零读者
3. **话题一律绝对名**（/ 前缀）；include 顺序有隐式依赖，禁止重排

## 强制规则：canbus 代码工作前必读 Vibe Coding 指南

任何涉及 src/canbus/ 的代码阅读、修改、新增、清理或协议接入任务，开始前必须完整阅读：

1. src/canbus/VIBE_CODING_GUIDE.md（大模型执行规则、编码风格、验证清单）；
2. src/canbus/README.md（现行架构、消息契约、下线功能和已知风险）；
3. 本次 CAN ID 对应的 src/canbus/docs/ 协议源。

保持当前 node/Comply 分工、Timer 频率和最小改动习惯；不得把未初始化、缺少 DLC/校验和验证、重复 include 等历史缺陷当作风格复制。

## 项目速览

- 机场 L4 无人牵引车整车软件（浦东机场），本目录为车载 `/home/nvidia/qingwei-L4-No2` 的重建副本（非 git）
- `workflow.md` 是工作延续文档（开工前通读，含全部历史结论与保护清单）；
  `PNC_ANALYSIS.md` 是 pnc 深度分析报告（行号为 08-29 基准）
- `src/pnc/`（catkin 包名 robot）为规划控制模块，其余见 workflow.md「工程概况」
- 本机无 ROS：C++ 改动用桩编译验证（方法见 workflow.md 08-30 日志）；
  auto_couple 等 GBK 文件 grep 需 `-a`
