# Q5 健身教练：深蹲、哑铃弯举与侧平举

固定机位，画面中锁定最明显的人。用户口头选择深蹲、哑铃弯举或侧平举；本地卡片负责计数、组目标和休息计时，`decision_core` 负责计划表达、口令和鼓励。手臂动作须让上半身和双臂完整入画，深蹲须全身入画。

## 画布接线

1. Q5 `camera_rgb`（`image/jpeg`）→ perception `pose`。
2. `pose` 的**完整骨架** `.../poses/skeleton`（`sensor/pose2d`）→ Q5 `squat_counter`；精简 `/poses` 缺少所需关节。同一卡片通过 `exercise` 选择动作，保留原卡片名和 `/fitness/squat/` 话题路径，画布接线无需增加卡片。
3. `squat_counter` 画布第 1 个输出 `events` → `decision_core`；第 2 个 `status`、第 3 个 `debug` 留给 Dashboard；**第 4 个 `.../fitness/squat/voice` → perception `tts` 的文本输入**。代码中的索引依次是 0、1、2、3。把 `squat_counter` 工具绑定给 `decision_core`。
4. Q5 `mic` → perception `asr` 的正常识别输出 → `decision_core`；同一张 TTS 卡片的 PCM → Q5 `speaker`，`decision_core` 的 TTS 执行器也必须绑定这张卡片。TTS 接入 `voice` 后，其 PCM 输出变为 `<voice 话题>/tts`；改线后重新连接 TTS → speaker 并重启画布。默认 `asr_kws` 模式的唤醒词是“小范小范”，例如“小范小范，我要开始深蹲锻炼了”。不要把 `/asr_background` 接作控制输入。

Q5 主驱动在 ROS Domain 211；媒体桥把 `camera_rgb` 送到 Agent Core 的 Domain 42，`pose` 和动作处理进程在 Domain 42。`squat_counter` 的 MCP 操作由 Q5 主驱动代理。子进程未启动时 `start`/`status` 会返回错误。

## 训练流程

`plugins.squat_counter.require_explicit_workout_start: true` 使卡片在画布启动后保持 `waiting_start`。说出“小范小范，我要开始深蹲／哑铃弯举／侧平举”后，`decision_core` 调用 `begin(exercise, sets, reps_per_set, rest_s)`；`exercise` 可选 `squat`、`dumbbell_curl`、`lateral_raise`。无特殊要求时计划为 **2 组 × 8 次、组间休息 90 秒**。支持 1–5 组、每组 1–20 次、组间休息 15–180 秒。以卡片校验后的 `plan` 和 `exercise` 为准；同一动作的重复 `begin` 不清零或修改计划。正在训练时改动作须先 `finish`，随后调用新动作的 `begin`。明确“重新计数”才调用 `reset(exercise=当前动作)`。省略 `exercise` 时沿用当前动作，初始默认深蹲；技能仍须显式传参以核对口令。

卡片确认所选动作的准备姿态后进入计数。启用 `local_milestone_voice` 后，卡片直接向 TTS 发“可以开始”、第 1 次、过半、本组达标、休息结束和训练达标的短提示，不等待模型查询状态或推理；组内不再向模型发送 `rep_completed`、就绪或 `rest_finished`。提示带 `expires_at`，新版 perception TTS 会跳过排队后过期的提示。Dashboard 的 `status.exercise`、`set_count` 显示当前动作与每次计数，`last_voice` 表示已发送提示，不等于已播出。非最后一组达到目标发 `set_completed` 并进入休息；用户说“小范小范，开始下一组”后，`decision_core` 调用 `next_set`，卡片才重新计数。最后一组达标时发 `workout_finished`，模型随后播报训练总结。用户说结束、画面持续 6 秒无人且姿态流正常，或跟踪目标换人时，也会结束并冻结总结。姿态流断开或过期不等于用户离场。

深蹲判定在 `common/squat_logic.py`：先站稳约 1.2 秒建立基线，再保持站姿约 0.4 秒进入本组计数，随后识别下蹲、蹲底、起身和站稳。站姿位置变化时可自动更新基线并保留次数。建议镜头固定、全身入画且略偏侧面。手臂动作判定在 `common/arm_exercise_logic.py`；弯举建议正面或微侧面，锁定一侧较清晰手臂，以伸直→弯曲→伸直为一次，双臂同时弯举只算一次。侧平举建议正面，以双臂下垂→向两侧抬到肩高→放下为一次。换动作时先调整站位，保持肩、肘、腕全程可见。二维姿态不能确认哑铃是否存在或判断重量。Q5 镜头无法覆盖动作时，可把外部相机接入 `pose`。各动作阈值须结合真机画面校准；手臂阈值可在 `plugins.squat_counter.arm_exercises.<exercise>` 下按 `arm_exercise_logic.DEFAULT_CONFIG` 覆盖。

`status` 可查看训练阶段、计划、当前组次数、累计次数、`input_stale` 和帧数；`debug` 给出中文角度、阈值判断及未计数原因。排查时先确认 `frames_received` 持续增长且 `input_stale=false`。`reset` 不能修复中断的 pose 或 ASR 流。`status.phase=paused` 或 `target_lost` 事件里的 `state=paused` 表示已锁定的人体暂时丢失或关节不可读，**不代表卡片没有启动**；新版事件另有 `card_state` 和 `workout_state`。结合工具 `status` 的 `state`、`coach_mode`、`workout_state`、`target_state` 和 `debug` 的“判定原因”查看原因。

播报仍慢时，先看 `status.last_voice.sent_at`：若卡片已及时发送而没听到，检查 TTS 输入是否接在 `voice` 输出、speaker 是否接同一 TTS 实例的 PCM，以及 TTS 日志是否出现 `discarded expired topic cue`。短提示过期会跳过，避免训练结束后补念旧数字；若常被丢弃，测量 TTS 首帧延迟并确认 Q5 与 perception 的系统时钟同步。Dashboard 状态仍慢时，对比工具 `status.set_count` 与画布显示，并检查 `frames_received` 和 `last_frame_age_s`。

若机器人完全无声，先运行 `speaker.info`：`state=idle` 表示还没开始播放，检查画布中的 TTS PCM → speaker 连线，并对比 `topic_in[0].topic` 与 TTS `info.topic_out[0].topic`；输入话题为空也可能是音频路由在绑定前启动失败，需看启动记录。新版 speaker 会以 `state=error` 和 `startup_error` 报告启动失败，以 `self_check_error` 报告自检音失败。旧版只能在 Q5 driver 日志中找 `playback unavailable` 或 `startup self-check failed`。改线后停止项目，等 `project_stop_done` 再启动，以免旧停止任务关掉新实例。`state=running` 且 `frames_received`、`frames_written` 增长但听不到时，可用 `speaker.set_volume` 临时微调，当前默认值为 `5`。

## 技能和验证

技能指令模板仍在 `fitness_squat_skill.md`。在 Resource Center → My Skills 创建或更新技能，可沿用 slug `q5-squat-coach`，分类 `utility`，所需工具为 `squat_counter, tts`；简介应说明支持深蹲、哑铃弯举和侧平举。将新版模板正文粘贴为 instruction，安装到运行中的 Agent Core 并在技能界面启用。**仅启用还不够**：模型须调用 `activate_skill("q5-squat-coach")` 才能读到完整指令；Agent Core 重启后需要重新激活。已安装技能不会随仓库文件更新，用技能界面或 `/api/skills/update` 更新，并检查 `/api/skills/q5-squat-coach` 的 `instruction` 包含三个 `exercise` 值。此版本需同步更新 Q5 driver、perception TTS、Agent Core 中运行的技能；原画布接线继续使用。

若 ASR 已输出而训练未开始，先调用计数卡片 `status`：`state=idle` 表示画布未把骨架输入启动到卡片，核对 `pose` 的完整骨架输出口和项目启动报错；`state=running, workout_state=waiting_start` 表示等待 `begin`，核对 Decision Core 的执行器连接、技能运行时激活状态（`/api/skills/active`）和工具调用记录；`state=running, workout_state=active, phase=paused` 表示训练已启动但目标不可读，查看 `frames_received`、`candidate_count`、`target_state` 和中文 `判定原因`。新版教练模式的 `status.coach_mode` 应为 `true`，`target_lost` 事件也应带 `set_index`、`set_state`；若缺失，检查 Q5 驱动镜像是否包含训练组数版本、运行中的配置是否开启 `coach_mode`。训练计划应在成功调用 `begin` 且返回 `plan` 后通过 TTS 播报。

运行 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_arm_exercise_logic.py tests/test_bumi_squat_counter.py robotera/q5_bundle/test_q5_squat_counter.py -q` 做本地回归。真机语音、相机视角和播报延迟仍需在 Q5 上验证。
