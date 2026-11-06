# 看到人招手 Skill

目标：Bumi 固定站在正门内侧朝外，持续看到人约 1 秒后执行已有 `semantic_action.wave`，并通过机器人扬声器说一次“您好，欢迎光临！”。原厂动作自动结束，不需要再开发动作卡。没有进出计数和人员身份识别。

## 使用

优先通过技能广场的“我的技能”创建草稿，按 `skill.json` 填写字段并上传声明的两个随附文件，再安装到目标机器人。
修改文件后先运行 `python package_skill.py --build-only` 更新文件清单；编辑已安装技能时先停止迎宾，再按页面的卸载、重新安装流程更新。
`package_skill.py` 的直接导入仅作为开发／网络异常时的应急入口，不会随驱动镜像构建而执行。

安装后刷新网页，在“技能 → 已安装”中找到“看到人招手”，保持启用。

你现在画布里的两条连接可以保留：

1. `remote_message → decision_core` 数据线。
2. `decision_core → semantic_action` 执行绑定。

先启动 Perception Stack，确保 vop 出现在工具列表中，并保持智能控制开启。Skill 会配置自己独立的 vop 实例 `skill-bumi-person-wave`，使用 Bumi 相机当前声明的 Topic；不改其他检测实例。

在消息卡片中发送：

```text
机器人已经正常站稳、双脚着地，地面平坦防滑，手臂活动范围内无人和障碍物。开始看到人招手。
```

仅在这些条件实际成立时发送。人体检测不负责判断机器人是否已经站稳或双脚着地。

停止与查询：

```text
停止迎宾。
```

```text
查看迎宾状态。
```

## 行为与验收

| 场景 | 预期 |
|---|---|
| 一个人持续进入画面约 1 秒 | 机器人发起一次挥手 |
| 人一直停在画面里 | 不连续挥手 |
| 人真实离开约 3 秒，再次出现 | 距上次尝试超过 30 秒才再次挥手 |
| 30 秒冷却期间重新出现 | 不补发、也不排队延迟挥手 |
| 多人一起出现 | 作为一次“有人出现”，不逐人挥手 |
| 相机或运动状态断流 | 不发新挥手，旧画面不视为当前观测 |
| 正在走动、跳舞、保护或有已确认电机故障 | 不发新挥手 |
| 停止智能控制、停用技能或解除挥手绑定 | 执行流程自动退出 |

“停止迎宾”停止后续触发。已发出的挥手由原厂动作自动结束，不能把进程退出当成已确认机器人所有关节停止。

## 实现边界

这是场景 Skill，复用 vop、motion_state、semantic_action.wave 和 tts/speaker；所带 `person_wave.py` 负责持续确认、重新触发和冷却，不增加新的运动算法或动作卡。语音和挥手共用一次触发，持续停留不重复说话。

prepare 会准备 TTS，并将空闲扬声器连接到真实 TTS 输出；若已有有效混音链路则复用，保留当前音量。不覆盖其他正在使用的音频路由。迎宾进程停下后不再提交新的语音；已提交的短句可能继续收尾。

检测与机器人状态经当前服务已有 `/ws/bus` 接口读取。前两帧中可能存在旧缓存，因此流程会丢弃每次连接后的第一份数据；还会检查检测时间戳、重复帧、运动状态新鲜度及现有执行绑定。

Skill 只在收到“开始迎宾”后启动后台流程。安装、技能启用、check、prepare、observe 都不调用挥手。流程不保存身份、抓拍图片或进出日志；本地 runtime 目录只存进程状态、上次动作时间和必要错误信息。

## 文件与本地检查

- `skill.json`：本地导入的原生 Skill 定义。
- `instruction.md`：完整指令原文。
- `person_wave.py`：随 Skill 安装的场景执行流程。
- `package_skill.py`：生成清单并通过当前原生本地导入接口安装，不需要资源中心账号。
- `test_workflow.py`：确认、去重、断流、冷却、运动条件和只读观察测试。
- `VALIDATION.md`：实际验证状态。

```powershell
cd D:\phanthymotus-driver\noetix\bumi\skills\bumi-person-wave
python -m unittest -v test_workflow.py
python package_skill.py --build-only
```

仅需应急直接导入时，首次安装运行 `python package_skill.py`，升级已安装版本运行 `python package_skill.py --replace`，按提示输入 Dashboard Token。升级前需确认迎宾已停止且上一动作结束；脚本先校验并备份原定义、随附文件与进程状态，再通过原生卸载/导入接口更换文件，失败则恢复旧定义与文件。备份在本地 backups 目录。文件内容按字节长度与 SHA-256 由服务校验；不会保存 Token。

## 手动运行与排错

若当前模型没有 Bash 工具，可在已经登录的机器人 SSH 终端执行下列命令。这里是 `docker exec`，不要再次在机器人里运行 `ssh -L`。

```bash
docker exec phanthy-motus-agent-core-1 /work/.venv/bin/python /work/resource/skill_files/bumi-person-wave/person_wave.py check
```

首次准备检测模型，可能需要下载引擎：

```bash
docker exec phanthy-motus-agent-core-1 /work/.venv/bin/python /work/resource/skill_files/bumi-person-wave/person_wave.py prepare
```

只读观察 5 秒，不执行动作：

```bash
docker exec phanthy-motus-agent-core-1 /work/.venv/bin/python /work/resource/skill_files/bumi-person-wave/person_wave.py observe --seconds 5
```

只测试一句迎宾语，不挥手（先完成 prepare）：

```bash
docker exec phanthy-motus-agent-core-1 /work/.venv/bin/python /work/resource/skill_files/bumi-person-wave/person_wave.py voice-test
```

满足上述实际站立与空间条件，并确认已有执行绑定和智能控制开启后，才启动：

```bash
docker exec phanthy-motus-agent-core-1 /work/.venv/bin/python /work/resource/skill_files/bumi-person-wave/person_wave.py start
```

查询或停止：

```bash
docker exec phanthy-motus-agent-core-1 /work/.venv/bin/python /work/resource/skill_files/bumi-person-wave/person_wave.py status
docker exec phanthy-motus-agent-core-1 /work/.venv/bin/python /work/resource/skill_files/bumi-person-wave/person_wave.py stop
```

先用 status 确认进程停止；需要查看异常时读取本技能目录下 `runtime/runner.log`。原厂动作不能在 20 秒内确认结束、API 返回不确定或网络断开时，流程停止且不自动重发。
