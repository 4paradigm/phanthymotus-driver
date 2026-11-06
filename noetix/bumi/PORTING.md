# 既有 Card 与 Skill 迁移记录

迁移日期：2026-11-06。目标分支：`feat/bumi-driver`。

## 迁入内容

| 原独立项目 | 现在的位置 | 迁移说明 |
|---|---|---|
| `bumi-foot-pose` | `foot_pose.py`、`tests/test_foot_pose.py`、`FOOT_POSE.md` | 保留执行保护，接入当前 Bundle、YAML、Dockerfile 与卡片目录 |
| `bumi-person-wave` | `skills/bumi-person-wave/` | 保留检测去重、冷却、挥手、TTS 流程、指令、打包工具与测试 |
| `station-photo-inspection` | `skills/station-photo-inspection/` | 保留原生定义、指令、核验及应急安装工具与测试 |
| `bumi-foot-rps` | `skills/bumi-foot-rps/` | 保留规则模块、测试及未完成状态 |

没有用旧导出的 `main.py`、`device.py` 或 `common/` 覆盖当前仓库；仅将脚步插件的注册点迁入现有入口。
基础镜像采用本仓库 `Dockerfile` 的既有定义，不沿用早期机器人容器快照镜像。

## 原工作保留范围

早期独立项目原件保留在原目录。以下内容不属于当前源码交付，未复制到 Git：

- 机器人抓拍照片、运行时 JSON、进程 PID 与错误日志。
- 旧版本备份、安装回执、容器快照安装／恢复脚本和生成的整套驱动副本。
- Python 虚拟环境、缓存、压缩包、下载的第三方源码和模型、实验仿真产物。
- `doorway-counter` 仅有现场照片，没有可迁入的 Card 或 Skill 源码。

脚步动作、石头剪刀布的真机完成状态没有因迁移而改变。
后续提交以真实能力、运行证据和老师验收为准。

## 迁移后的本地验证

2026-11-06 在 Windows 开发机执行：

| 检查 | 命令（在对应目录运行） | 结果 |
|---|---|---|
| 当前 Bumi 驱动回归 | 仓库根目录：`python -X utf8 -m pytest noetix/bumi/tests -q` | 158 passed，包含脚步 Card 的 16 项 |
| 迎宾 Skill | `skills/bumi-person-wave`：`python -X utf8 -m unittest -v test_workflow.py` | 18 项通过 |
| 工位拍照 Skill | `skills/station-photo-inspection`：`python -X utf8 -m unittest -v test_skill.py` | 7 项通过 |
| 游戏规则草稿 | `skills/bumi-foot-rps`：`python -X utf8 -m unittest -v test_game.py` | 6 项通过 |

合计 189 项通过。额外检查：

- 13 个相关 Python 文件语法有效；插件配置、卡片目录、Dockerfile COPY 和空资产模板一致。
- 用本机平台源码的实际 MCP Schema 转换函数检查脚步工具：四种姿态、preview、status、stop 均可见，无结构错误或警告。
- 重新生成迎宾 Skill 附件清单，并核对 Git 暂存内容的字节数、SHA-256 和指令一致性；LF 属性保证跨平台检出后附件字节稳定。
- `git diff --cached --check` 通过。

本次未构建 ARM64 镜像，未运行真实 ROS/DDS 集成或真机动作，也未安装／发布 Skill。
后续通过 PR 测试镜像完成设备接入验证；脚步卡仍为无动作资产的预览原型。

## 后续脚步基础功能

迁移后已按用户确认的“先完成能独立测试的基础功能”范围补齐 `read_feet`、`check_pose`、`reload_assets` 和离线计算入口。
已修正旧原型对 SDK 接收回调的错误假设，改用真实 getter；未知来源时效仍阻止运动。
动作资产允许逐个载入，后续真实执行仍需已验证的中立动作与新鲜遥测。
最新驱动回归为 173 项通过，操作与边界见 [FOOT_POSE.md](FOOT_POSE.md)，复用核查及验证细节见 [FOOT_POSE_REUSE.md](FOOT_POSE_REUSE.md)。
