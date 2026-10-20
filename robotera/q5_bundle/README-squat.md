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

`status` 可查看训练阶段、计划、当前组次数、累计次数、`input_stale` 和帧数；`debug` 给出中文角度、阈值判断及未计数原因。排查时先确认 `frames_received` 持续增长且 `input_stale=false`。`reset` 不能修复中断的 pose 或 ASR 流。

## 技能和验证

技能指令模板在 `fitness_squat_skill.md`。**须在 Resource Center 安装并激活更新后的模板**；修改仓库文件不会自动更新已安装的技能。运行 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_bumi_squat_counter.py robotera/q5_bundle/test_q5_squat_counter.py -q` 做本地回归。真机语音、相机视角和播报延迟仍需在 Q5 上验证。
