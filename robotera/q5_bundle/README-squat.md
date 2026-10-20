# Q5 健身教练：深蹲训练

固定机位、全身入画，画面中锁定最明显的人。训练由语音开启，本地卡片负责计数、组目标和休息计时，`decision_core` 负责计划表达、口令和鼓励。

## 画布接线

1. Q5 `camera_rgb`（`image/jpeg`）→ perception `pose`。
2. `pose` 的**完整骨架** `.../poses/skeleton`（`sensor/pose2d`）→ Q5 `squat_counter`；精简 `/poses` 缺少所需关节。
3. `squat_counter` 输出口 0 `.../fitness/squat/events` → `decision_core`；`status`、`debug` 输出留给 Dashboard。把 `squat_counter` 工具绑定给 `decision_core`。
4. Q5 `mic` → perception `asr` 的正常识别输出 → `decision_core`；TTS PCM → Q5 `speaker`，并绑定 TTS 工具。默认 `asr_kws` 模式的唤醒词是“小范小范”，例如“小范小范，我要开始深蹲锻炼了”。不要把 `/asr_background` 接作控制输入。

Q5 主驱动在 ROS Domain 211；媒体桥把 `camera_rgb` 送到 Agent Core 的 Domain 42，`pose` 和深蹲处理进程在 Domain 42。`squat_counter` 的 MCP 操作由 Q5 主驱动代理。子进程未启动时 `start`/`status` 会返回错误。

## 训练流程

`plugins.squat_counter.require_explicit_workout_start: true` 使卡片在画布启动后保持 `waiting_start`。说出开始口令后，`decision_core` 调用 `begin(sets, reps_per_set, rest_s)`；无特殊要求时计划为 **2 组 × 8 次、组间休息 90 秒**。用户可在开始前提出不同组数、次数或休息时长；支持 1–5 组、每组 1–20 次、组间休息 15–180 秒。参数以卡片校验后的返回值为准；已开始的会话不会因重复 `begin` 清零或修改计划。

卡片确认站稳后发 `set_ready`；完成完整下蹲到站起循环时立即更新本组次数。为避免慢模型积压旧数字，组内只在第 1 次和过半时发语音进度事件；Dashboard 的 `status` 仍显示实时计数，达标事件立即发出。非最后一组达到目标发 `set_completed` 并进入休息；休息计时结束发 `rest_finished`。用户说“小范小范，开始下一组”后，`decision_core` 调用 `next_set`，卡片才重新计数。最后一组达标时直接发 `workout_finished`。用户说结束、画面持续 6 秒无人且姿态流正常，或跟踪目标换人时，也会结束并冻结总结。姿态流断开或过期不等于用户离场。

动作判定在 `common/squat_logic.py`：Q5 先站稳约 1.2 秒建立基线，再保持站姿约 0.4 秒进入本组计数，随后识别下蹲、蹲底、起身和站稳。站姿位置变化时可自动更新基线并保留次数。建议镜头固定、全身入画且略偏侧面；Q5 镜头无法拍全身时，可把外部相机接入 `pose`。阈值须结合真机画面校准。

`status` 可查看训练阶段、计划、当前组次数、累计次数、`input_stale` 和帧数；`debug` 给出中文角度、阈值判断及未计数原因。排查时先确认 `frames_received` 持续增长且 `input_stale=false`。`reset` 不能修复中断的 pose 或 ASR 流。`status.phase=paused` 或 `target_lost` 事件里的 `state=paused` 表示已锁定的人体暂时丢失或关节不可读，**不代表卡片没有启动**；新版事件另有 `card_state` 和 `workout_state`。结合工具 `status` 的 `state`、`coach_mode`、`workout_state`、`target_state` 和 `debug` 的“判定原因”查看原因。

## 技能和验证

技能指令模板在 `fitness_squat_skill.md`。在 Resource Center → My Skills 创建技能，建议 slug 为 `q5-squat-coach`、分类 `utility`、所需工具为 `squat_counter, tts`；简介写明“用户说开始深蹲训练时，制定组数、次数和休息时间并调用计数卡片带练”。将模板正文粘贴为 instruction，安装到运行中的 Agent Core 并在技能界面启用。**仅启用还不够**：模型须调用 `activate_skill("q5-squat-coach")` 才能读到完整指令；Agent Core 重启后需要重新激活。已安装的同名技能不会随仓库或 Resource Center 更新，检查 `/api/skills/q5-squat-coach` 的 `instruction` 是否包含“2 组、每组 8 次、组间休息 90 秒”，旧版本用技能界面或 `/api/skills/update` 更新。

若 ASR 已输出而训练未开始，先调用计数卡片 `status`：`state=idle` 表示画布未把骨架输入启动到卡片，核对 `pose` 的完整骨架输出口和项目启动报错；`state=running, workout_state=waiting_start` 表示等待 `begin`，核对 Decision Core 的执行器连接、技能运行时激活状态（`/api/skills/active`）和工具调用记录；`state=running, workout_state=active, phase=paused` 表示训练已启动但目标不可读，查看 `frames_received`、`candidate_count`、`target_state` 和中文 `判定原因`。新版教练模式的 `status.coach_mode` 应为 `true`，`target_lost` 事件也应带 `set_index`、`set_state`；若缺失，检查 Q5 驱动镜像是否包含训练组数版本、运行中的配置是否开启 `coach_mode`。训练计划应在成功调用 `begin` 且返回 `plan` 后通过 TTS 播报。

运行 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_bumi_squat_counter.py robotera/q5_bundle/test_q5_squat_counter.py -q` 做本地回归。真机语音、相机视角和播报延迟仍需在 Q5 上验证。
