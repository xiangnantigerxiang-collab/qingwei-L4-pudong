# 2026-09-21：清理9月10日以前的无用备份

用户明确要求：“整理9月10日以前的文件哪些是没有用的备份，都删掉”。
范围为当前工作区 `/home/mothotob/work/projects/claude-code`，日期边界为2026-09-10之前，不含9月10日当天。
本次按该明确授权清理旧备份，不改变日后对其他快照和业务文件的保护规则。

已删除11个文件：8个tar.gz备份、2个旧感知模块ZIP、1份旧workflow副本。
删除文件总大小3,216,679字节，约3.07 MiB；未另行复制这些旧备份。

## 已删除清单

以下路径均相对工作区根目录，SHA-256为删除前校验值。

| 文件 | 字节数 | 删除前SHA-256 |
|---|---:|---|
| baseline_vehicle_deployed_20260905.tar.gz | 1425663 | 0e0dda583bb95b4ba56e92f50cedd1cf134ea1812097e0f83dbb431613883684 |
| canbus_before_ehb_20260905.tar.gz | 507837 | cf70dea004680da93b40b51bc0b8210cfc7dd5395f01bd560992571ee05812a4 |
| canbus_before_staleness_config_20260903.tar.gz | 354819 | 40930b93ccda30fdd8d20ac2f9bfe1d33e0c254baeae3deaf99353056e0166b2 |
| canbus_user_rewrite_before_fix_20260903.tar.gz | 354809 | 0b1cb2d66de118fd8bb4b20ff415f7740b0c7a0c3a16a3cf37c5c303e15cb0e3 |
| canbus_user_tidy_20260903.tar.gz | 355444 | b9590d84af98d35b7fb6038dc7c5a4e22711a861a25cbdfef802ab8d4ddfed30 |
| docs_before_slim_20260905.tar.gz | 66255 | f957f1f337eed0bb5de74d7c42481a0704f1979fa82a77169748485380fa9fdc |
| pnc_before_hookstatus_hb_20260904.tar.gz | 28221 | 9d5bdfad62551aa58a56ceba7d5ca4940c30bbd69878b44afd1cd4bb4f6d2638 |
| pnc_before_inline_fix_20260904.tar.gz | 1296 | 85c7a469d0c657d48e1af2c2769998597ac05bed9548f3bf194b6d12d201a85c |
| robot_perception_convert0909.zip | 6688 | 961bb66825f5e5aad1f8e9135fff2eb81051e28a2ae49ef95cbe8cf6f2ad7524 |
| robot_perception_convert_需修改csv轨迹.zip | 7137 | 8edc978d18871b4d04dbdaaf18a191ad956677d9ec4caaadc27ae3407f182b0a |
| workflow_full_backup_20260905.md | 108510 | d2357061c8248538c49c3331ad61e1f6e55a958d343a1df8e00f7367ad21e970 |

## 判定与保留范围

- 检查了压缩包目录：均为旧代码/文档副本，两个ZIP为旧robot_perception_convert源码。
- 对11个名称进行工作区精确引用检索：仅在旧workflow副本和9月19日归档workflow中出现；未发现构建、运行或测试代码依赖。
- 当前PNC已在9月10日重设基底，9月11日后继续更新；保留9月10日及以后的备份、部署包和本轮control验证证据。
- 保留 `baseline_sweep_reports_20260905.md`：它是分析报告，当前canbus README仍引用其中的组合真值表，不能归为无用备份。
- 9月19日归档正文及其哈希清单保持原样，其中指向上述已删除文件的历史引用不再代表文件仍存在。
- 源码、地图、参数、依赖库、运行环境、日志、设计资料及浏览器调试产物不属于本次删除对象。

## 核对记录

- 删除采用明确的11个文件名，逐项检查普通文件、日期、大小和SHA-256，无递归删除。
- 删除后复核候选文件均不存在，9月10日基线及保留分析报告哈希不变。
- 当前工程及其他工作区文件保持，另更新workflow现状/日志并新增本清单；control最新交付文件哈希保持。
- 本轮没有代码修改，不需要重新编译或重启；未以运行测试替代文件清单核对。
- 详细删除前文件元数据、引用检索结果及删除结果位于 `/tmp/backup_cleanup_pre0910_20260921/`。
