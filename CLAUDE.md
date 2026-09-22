# CLAUDE.md — 项目级指令（每次会话自动加载）

## 强制规则：编码前必读规范与风格记忆

1. **全局技术偏好与风格记忆**：
   开始任何编码、排查或改动任务前，必须阅读 `docs/CODING_MEMORY.md`（编码与技术风格记忆），遵循用户明确指定的技术偏好、几何与时延口径、修改边界及验证要求。
   **每次开始代码修改任务前，必须重新读取其中「恒定准则：时间复杂度与车载 CPU」；车载 CPU 负荷接近饱和，优先减少时间复杂度和高频重复计算，允许以有界额外内存换取时间，同时保持业务语义并及时清除失效缓存。**

2. **PNC 代码工作前必读风格规范**：
   **任何涉及 `src/pnc/` 的代码阅读、修改、新增、清理、重构任务，开始前必须先完整阅读 `src/pnc/README.md`（pnc 编码风格与习惯规范），并全程遵循其中约定。**
   核心三条（详见 README）：
   - **PNC 自有 C/C++ 统一使用用户 2026-09-14 指定的风格**——4 空格缩进，左大括号同行，例如 `if(condition) {`，`else` 使用 `} else {`。根配置为 `src/pnc/.clang-format`；第三方源码保持原样，include 顺序不调整。
   - **不重命名任何既有标识符**（包括拼写错误）；清理/格式化绝不改业务逻辑，删除类改动需逐项 grep 实证零读者。
   - **话题一律绝对名**（`/` 前缀）；include 顺序有隐式依赖，禁止重排。

3. **CANBUS 代码工作前必读 Vibe Coding 指南**：
   任何涉及 `src/canbus/` 的代码阅读、修改、新增、清理或协议接入任务，开始前必须完整阅读：
   - `src/canbus/VIBE_CODING_GUIDE.md`（大模型执行规则、编码风格、验证清单）；
   - `src/canbus/README.md`（现行架构、消息契约、下线功能和已知风险）；
   - 本次 CAN ID 对应的 `src/canbus/docs/` 协议源。
   保持当前 node/Comply 分工、Timer 频率和最小改动习惯；不得把未初始化、缺少 DLC/校验和验证、重复 include 等历史缺陷当作风格复制。

## 项目速览

- 机场 L4 无人牵引车整车软件（浦东机场），本目录为车载 `/home/nvidia/qingwei-L4-No2` 的重建副本（非 git）
- `workflow.md` 是工作延续文档（开工前通读，含全部历史结论与保护清单）；
  `PNC_ANALYSIS.md` 是 pnc 深度分析报告（行号为 08-29 基准）
- `src/pnc/`（catkin 包名 robot）为规划控制模块，其余见 workflow.md「工程概况」
- 本机无 ROS：C++ 改动用桩编译验证（方法见 workflow.md 08-30 日志）；
  auto_couple 等 GBK 文件 grep 需 `-a`

## Antigravity (AGY) 规则与协同约束

1. **保护清单绝对不可触碰（清理/修改严禁触碰）**：
   - `src/fms_agent/env/`、`.git`、`data/tmp_path.txt`（运行时依赖与版本史）
   - `src/CUDA-CenterPoint/build/` 与 `model/`（start_l4.sh 原地启动+相对路径）
   - `src/ivlocmsg/`（PNC 核心定位消息，严禁删除）
   - `src/driver/cam_geac/demo/` 全部可执行文件（rb_camera.sh 按名调用）
   - `src/pnc/path/`、`param/`、`3rd-party/arm/*/lib*`（业务数据与预编译库正本）
   - 根目录启动运维脚本（`start_l4.sh`, `rebuild_all.sh`, `fms.sh`, `record_*.sh` 等）及工程外快照与备份 md
2. **开发与验证铁律**：
   - **本机无 ROS1**：C++ 修改必须在 `src/pnc/tests/` 等桩编译框架下严格编译并通过回归断言，严禁未经桩测试交付。
   - **严禁擅自重构或扩大修改面**：保持小步修改，严格杜绝改变非目标文件的排版、符号重命名或“顺手优化”。
   - **编码保护**：`auto_couple` 等部分模块文件为 GBK 编码，`grep` 检索时必须加 `-a`，禁止擅自批量转码破坏车载构建。
   - **文档延续纪律**：任何代码逻辑变更完成后，必须在 `workflow.md` 的「现状」与「日志」追加结论级条目（包含变更文件、验证命令、断言结果与备份快照路径）。
3. **多 Agent 规则同步与对齐纪律**：
   - 本工程同时支持 Antigravity (AGY)、Claude Code 和 Codex Code 共同协同开发。
   - 任何规则文件的调整（无论是修改 `GEMINI.md`、`CLAUDE.md`、`CODEX.md` 还是 `AGENTS.md`），必须保持多端规则完全对齐。
   - 每次规则变更由 `scripts/sync_agent_rules.py`（及 `.agents/hooks.json`）自动触发，同步更新工程内 `GEMINI.md`、`CLAUDE.md`、`CODEX.md`、`AGENTS.md`，并同步覆盖同级工作空间（`claude-code` 和 `codex-code`）。
