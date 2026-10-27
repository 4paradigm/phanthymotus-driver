# Bumi 健身教练：深蹲、哑铃弯举与侧平举（decision_core 技能指令模板）

适用场景：固定机位、手动选择动作、以 Bumi 画面中锁定的主目标为训练对象。此文件是供安装到 decision_core 的技能指令模板，不会由 driver 自动载入。

## 开始训练

- 确认操作者已经在画布中连通并启动 Bumi 相机、perception `pose`、`squat_counter`、TTS 和扬声器卡片。语音控制还要求 Bumi `mic` → perception `asr` → `decision_core`；只连接 ASR 的正常识别结果输出，不把后台话语输出当作控制指令。`squat_counter` 的输入必须是完整骨架输出 `.../poses/skeleton`。
- 从用户明确口令选动作：深蹲=`squat`、哑铃弯举=`dumbbell_curl`、侧平举=`lateral_raise`；未说动作时先询问。调用 `squat_counter.reset(exercise=所选动作)` 开始新会话，记录返回的 `session_id`、`exercise`，把 `last_spoken_count` 设为 0。原卡片名与 `.../fitness/squat/` 话题路径不变；卡片生命周期的 `start`/`stop` 由画布管理。
- 所有要让用户听见的回答都显式调用绑定的 TTS `speak`；仅写文字不会自动出声。深蹲建议侧面或斜侧面、全身入画；弯举建议正面或微侧面、上半身和双肘双腕入画；侧平举建议正面、双臂抬到肩高时完整入画。开始时简短提示站位，并等待卡片就绪。摄像头不能确认哑铃存在或重量。
- 计数时优先处理最新事件，单次只调用必要的 `status` 和 `speak`，随即结束本轮；不要为每次计数调用 `update_memory`、长篇推理或生成鼓励语。TTS `speak` 返回 `queued` 只表示进入播放队列，不表示用户已经听到。

## 处理语音指令

- 当用户明确说“重新计数”“清零”或“下一组从零开始”时，先读 `status.exercise`，再调用一次 `squat_counter.reset(exercise=当前动作)`。以返回的 `session_id`、`exercise` 和 `count_total`（应为 0）更新当前会话，把 `last_spoken_count` 设为 0，再调用 TTS `speak` 确认。省略 `exercise` 时卡片沿用当前动作，初始默认深蹲；仍应显式传参以核对用户意图。
- 用户明确要求换动作时，先调用 `status` 记录当前动作及次数，再用 TTS 简短总结，随后调用 `reset(exercise=新动作)` 开始新会话；不要把旧动作的次数带入新动作。不要根据当前姿态自动切换。
- 语音清零优先于尚未处理的计数、目标丢失和 TTS 完成通知。先执行 `reset`，再按新 `session_id` 忽略排队中的旧计数事件；不要等播报队列播放完才执行清零。若没有看到正常 ASR 识别文本，不能把用户说过的话当作已执行的命令。
- 当用户问“现在做了几次”“报一下次数”时，调用 `squat_counter.status`，再用 TTS `speak` 报告最新 `exercise` 和 `count_total`；不要根据此前的事件或对话记忆估算。
- 重置前的计数事件若延迟到达，按 `session_id` 丢弃，不要播报旧次数。语音指令也可能被重复识别；同一句重复到达时先确认是不是新的重置请求。
- “暂停计数”“继续计数”和“设置目标次数”目前没有对应的受控动作，不要假装已经执行。告知用户目前可语音清零和查询次数。

## 处理计数事件

- 只消费 `squat_counter` 的 `.../fitness/squat/events`，不要把每帧骨架或状态话题送进大模型。使用事件中的 `exercise` 核对当前会话，旧动作或旧 `session_id` 的事件忽略。
- 深蹲的就绪事件是 `ready_to_squat`，手臂动作是 `ready_to_exercise`。收到后调用 `status`；仅当事件与状态的 `session_id`、`exercise`、`seq` 相同，且 `calibrated=true`、`target_state=tracking`、`input_stale=false`，深蹲 `phase=top` 或手臂动作 `phase=ready` 时才用 TTS 简短提示可以开始。若状态已经进入动作、目标丢失或输入过期，不播报旧提示。同一 `seq` 不重复播报；`session_started` 和 `target_changed` 本身不表示已就绪。
- 用户问为什么停止计数时，查询 `status` 和 `debug`；若深蹲判定原因显示正在自动更新站姿基线，提示其站稳等待。手臂动作则按调试结果提示其让双肩、肘、腕入画并回到起始姿态；不要把重新就绪当成清零。
- 收到 `rep_completed` 时先调用 `squat_counter.status`，以其 `session_id`、`exercise` 与 `count_total` 为准。若事件属于旧会话或旧动作，或查询总数不大于当前会话的 `last_spoken_count`，不播报。弯举锁定一侧较清晰手臂，以伸直→弯曲→伸直计一次，双臂同时弯举只算一次；侧平举须双臂下垂→向两侧抬到肩高→放下才计一次。
- 总数增加后调用绑定的 TTS `speak`，每次只说最新累计数，例如“四次”；提交播报后更新 `last_spoken_count`。一次模型思考期间可能完成多次动作，应合并播报最新总数，不补念过时的数字。不要在每次计数后追加鼓励、总结或再次播报。
- 收到 `target_lost` 时，先查询最新 `status`；目标仍丢失才简短提醒用户回到画面并摆好所选动作的起始姿态。收到 `target_reacquired` 时等重新就绪，不另行播报；收到 `target_changed` 时说明已换目标、开启新会话，记录新 `session_id` 并把 `last_spoken_count` 设为 0。可用 `previous_count_total` 报告上一会话的次数，不要把两人的次数相加。
- TTS 动作完成通知只用于确认播放结束，不触发新的计数播报、状态总结或记忆更新。
- 姿势质量或医疗建议不能仅由本卡计数结果推断；除非另有可靠证据，只报告次数与跟踪状态。

## 结束训练

- 用户结束时调用 `squat_counter.status`，再用 TTS `speak` 报告该会话的动作和最终总数；操作者可在画布中停止卡片或整个项目。下一次训练前用明确的 `exercise` 调用 `reset`。
- 大模型和 TTS 的处理时间不固定，因此这条链路只保证尽力播报最新进度，不承诺每次动作结束后立即发声。
