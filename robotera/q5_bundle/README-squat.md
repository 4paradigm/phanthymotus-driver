# Q5 深蹲教练

固定机位、单人、手动选择深蹲。画布接线：

1. Q5 `camera_rgb`（`image/jpeg`）→ perception `pose`。
2. `pose` 的**完整骨架**输出 `.../poses/skeleton`（`sensor/pose2d`，通常为输出口 1）→ Q5 `squat_counter`；精简 `/poses` 缺少所需关节。
3. `squat_counter` 输出口 0 `.../fitness/squat/events` → `decision_core`。输出口 1 `status` 和输出口 2 `debug` 只接 Dashboard；同时把 `squat_counter` 工具绑定给 `decision_core`。
4. Q5 `mic` → perception `asr` 的**正常识别输出** → `decision_core`；TTS 的 PCM 输出 → Q5 `speaker`，并绑定 TTS 工具。想直接说“我要开始深蹲了”“结束训练”，将 ASR `trigger_mode` 设为 `vad`；默认 `asr_kws` 要先说唤醒词。不要把 `/asr_background` 接作控制输入。

Q5 主驱动使用 ROS Domain 211，`camera_rgb` 已由媒体桥送进 Agent Core 的 Domain 42。`pose` 与深蹲处理进程都运行在 Domain 42；`squat_counter` 的 MCP 操作由 Q5 主驱动代理。子进程启动失败时 `start`/`status` 返回错误。

`config.yaml` 中 Q5 的 `plugins.squat_counter.require_explicit_workout_start: true`：卡片随画布启动后保持 `waiting_start`，听到明确开始指令后由 `decision_core` 调用 `begin` 才计数。`begin` 幂等；`reset` 清零并重新开始一组；`finish` 冻结结果。用户结束、画面连续 6 秒无人且姿态流正常，或跟踪目标换人时，训练结束。自动结束发出 `workout_finished` 事件；姿态流断开或过期不会被当作人离开。`status.workout_state` 为 `waiting_start`、`active` 或 `finished`，`status.summary` 保存本组 `count_total`、`duration_s`、动作平均/最短/最长用时及 `end_reason`。结束后需再次说开始，不能靠重新入画自动计数。

检测状态机和阈值在 `common/squat_logic.py`，与 Bumi 共用：先站稳约 2 秒建立基线，再确认下蹲、蹲底、起身和站稳。可见状态下站姿位置变化时会自动更新基线并保留次数。建议全身入画、镜头固定且略偏侧面；若 Q5 镜头看不完整人体，可使用接入 `pose` 的外部相机。阈值须结合真机画面校准。

`status` 可查实时累计数、`input_stale` 和接收/解析帧数；`debug` 给出中文角度、阈值判断及未计数原因。排查时先确认 `frames_received` 持续增长且 `input_stale=false`。`reset` 无法修复中断的 pose 或 ASR 流。

技能指令模板见 `fitness_squat_skill.md`。**须将它安装到 Resource Center 并激活**；修改此文件不会自动更新已安装的技能。运行 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_bumi_squat_counter.py robotera/q5_bundle/test_q5_squat_counter.py -q` 做本地回归。真机视角、跨 ROS 域通信和语音延迟仍需在 Q5 上验证。
