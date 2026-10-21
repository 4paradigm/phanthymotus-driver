# 通用网页示教模块

本模块来自 [G1 teach PR #412](https://github.com/4paradigm/phanthymotus-driver/pull/412)，固定来源提交为 `a335e68c9fb331f0bc961fda2a8bf8d8bd854d2d`。本分支仅引入公共模块及动作包回归测试，未合入 G1 Driver 或修改 G1 轨迹格式。

公共服务负责草稿、试运行结果、效果确认、Skill 定义和两文件动作包。机器人适配器负责设备绑定、录制、预检、运动、停止和真实完成回执。页面通过 `motus.workbench/1` 返回 HTML，由 Core 工作台承载；不增加业务 Web 服务。

TRON2 适配器显式声明 `presentation.passive_recording=true`。只有该分支允许直接调用 `record(args)`，隐藏“进入遥操”，并显示任务标签和训练导出。其他适配器继续使用原有 `record()`、遥操会话检查及试运行保存流程。

- 保存 Skill 必须有成功试运行和 `effect_confirmed=true`。
- 训练 episode 和 LeRobot 数据保存不经过 Skill 保存门禁。
- `motus.teaching.package/1` 仍只允许 `manifest.json` 与 `trajectory.jsonl`。ZIP 上限 16 MiB，轨迹上限 64 MiB。
- 适配器必须校验本机模型与关节顺序，并重新绑定导入资产。导入不产生试运行证明。
- Core 使用既有 `POST /file/upload` 上传动作包。导出返回设备内路径；不自动下载、上传或发布。
- 适配器未声明 `execution_binding()` 时，不启用逐技能自动执行。

当前 TRON2 的能力、阻塞和测试入口见 [TRON2 录制说明](../../limx/tron2/TEACHING.md)。本轮未重新验收 G1 真机；`tests/test_teaching_package.py` 保留来源 PR 的公共包和保存门禁回归。
