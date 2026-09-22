# 天轶遥操 PR 交付索引

当前 Driver 实施范围、架构后续事项和验证结果统一维护在[天轶遥操执行链适配与审查计划](tianyi-teleop-execution.md)。接口与操作说明见 [TELEOP.md](../../x-humanoid/tianyi2.0/TELEOP.md) 和[部署说明](../../x-humanoid/tianyi2.0/deploy/TELEOP_RUNBOOK.md)。

沿用已有 [Draft PR #321](https://github.com/4paradigm/phanthymotus-driver/pull/321)，不重复创建。本次在现有 PR 分支合入主线 19729470 并同步离线审查结果，保留原开发树及机器人现状，不操作承担接待任务的天轶。

本仓仅提交 Driver 执行入口、必要主入口/动作取消调整、构建、测试和文档。PICO、普通 ActuCore 内 teleop 卡片、Canvas 面板与 Core 接入属于主仓独立 PR，不用早期快照的排除范围描述当前主仓交付。

本次新基线定向验证 176 项通过；扩展验证 250 通过 / 7 失败，7 项均在未修改主线隔离副本复现，细节见实施计划。旧快照中缺少 trace 字段的 6 项测试失败已修复，并移除测试收集顺序依赖；未修改无关上游失败以制造全绿结果。

现场已有用户确认的双臂跟随及结束收臂证据，但本次仅离线整理和提交。手部、长时运行和全部故障注入不宣称完成；bot review 不替代设备验收。现场录制、凭据和私有调试流水不随代码发布。
