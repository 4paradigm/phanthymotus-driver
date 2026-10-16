# Bumi 深蹲计数播报技能

`squat_counter` 卡片负责识别并统计完整的深蹲动作；本技能只负责指导 Agent Core
如何响应计数完成事件。在项目的技能管理界面或资源中心创建技能，并填写以下字段：

- `slug`: `bumi-squat-coach`
- `name`: `Bumi 深蹲计数播报`
- `oneLiner`: `深蹲训练时接收 squat_completed 事件并播报当前次数`
- `description`: `配合 Bumi squat_counter 卡片进行深蹲次数语音播报。`
- `requiredTools`：如果编辑界面提供工具选择，选中画布上的 Perception `tts` 和 Bumi
  `squat_counter` 工具；还需将 TTS 卡片连接到 Bumi `speaker`。
- `instruction`：将下面的内容作为纯文本填入技能指令：

```text
当用户要求开始 Bumi 深蹲训练时，确认画布已启动 camera、pose、squat_counter、decision_core、tts、speaker，并使用本技能。squat_counter 负责姿态判定和计数；不要根据逐帧 pose 数据或对话内容自行增减次数。

仅处理来自 Bumi squat_counter 的 data/json 完成事件，事件格式为 {"type":"squat_completed","priority":1,"track_id":整数,"count":正整数,"timestamp":时间戳}。收到 type 为 squat_completed 的新事件时，读取 count，并调用画布连接的 tts 工具，参数为 {"action":"speak","text":"第<count>个"}，用实际 count 替换占位符。每个新的完成事件只播报一次，不把事件 JSON 原文、track_id 或时间戳念出来。若重复收到同一 track_id、count 的事件，不重复播报；重置或重新开始训练后允许从 1 重新播报。

用户询问累计次数时，调用 squat_counter 的 info 获取 count，再简短回答。只有用户明确要求重新计数时才调用 reset；用户要求结束训练时停止 squat_counter。没有完成事件时不要猜测次数或主动补报；TTS 不可用时直接告知用户，不声称已播放语音。
```

## 画布接线与激活

1. 重新构建并部署包含 `squat_counter` 的 Bumi 驱动镜像。
2. 在画布中依次连接 `camera (image/jpeg) -> pose`、pose 的**第一个**
   `data/json` 输出（`/poses`）-> `squat_counter`，以及 `squat_counter`
   的 `data/json` 输出（`/squat/count`）-> `decision_core`。将
   `decision_core` 连接到 TTS 工具，以便 Agent 调用；再连接
   `tts (audio/pcm-16k) -> speaker` 播放音频。启动画布后，计数卡片才会
   订阅姿态数据，扬声器卡片才会订阅 TTS 音频。
3. 在技能管理界面安装并启用本技能。启用后，Agent Core 能在可用技能列表中
   看到它；只有执行 `activate_skill("bumi-squat-coach")` 后，完整指令才会
   加载。开始第一次深蹲前，让 Agent 激活技能；测试时也可以使用技能激活
   接口。不要依赖系统补播激活之前收到的事件。
4. 检查 `/squat/count` ROS 话题，确认每次完整的“站立 -> 蹲下 -> 站立”
   只产生一个 `squat_completed` 事件；再确认 Agent 根据事件中的次数调用
   `tts.speak`，且 Bumi 扬声器实际播放了语音。

**不要将计数事件直接接入 TTS。**TTS 的话题输入读取 `text` 字段，而计数
事件提供的是 `count`。Agent Core 根据技能指令将次数转换为播报内容。
大模型调用和 TTS 合成是异步的；如果需要严格的低延迟或语音恰好播放一次的
保证，需要使用确定性的事件转语音处理卡片，不能只依赖本技能。
