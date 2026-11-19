# Pose Check 跟练 Skill 规划

## 定位

`pose_check` 是通用人体姿态感知卡，属于 Perception。它只接收相机画面、估计人体关键点、判断一个指定姿态，并返回结构化结果。它不负责语音、计时、计数或机器人动作。

当前支持：`hands_up`、`arms_open`、`one_hand_up`、`squat`。

## 卡片分层

```text
camera_head
    ↓ image/jpeg 或 sensor_msgs/Image
pose_check
    ↓ {pose, status, score, feedback}
exercise_tracker
    ↓ {step, count, progress, event}
workout_skill / decision_core
    ├── tts
    ├── arm_gesture / head_gesture
    └── plan_store / notification
```

### 1. `camera_head`

输出天轶 RGB 图像。真机原始输入是 `/ob_camera_head/color/image_raw`，`pose_check` 服务负责转换 `rgb8` 等常见编码。

### 2. `pose_check`

公开接口只保留：

```json
{"action":"check","pose":"hands_up"}
```

返回：

```json
{
  "detected": true,
  "matched": false,
  "status": "almost",
  "score": 0.5,
  "pose": "hands_up",
  "feedback": "右臂再向上伸直，保持接近垂直"
}
```

`status` 只描述当前帧：`completed`、`almost` 或 `retry`。相机失效、关键点不足时必须返回 `retry` 和具体原因。

跟练开始后，卡片只处理 `begin` 指定的目标动作，不再在每帧同时返回四个动作。独立调用 `check` 后会持续检查该目标动作，5 帧中至少 3 帧匹配才算稳定，并在稳定检测到、离开动作时分别发布一次 `pose_detected`、`pose_lost` 事件。深蹲先要求站直并完成一次稳定校准，校准完成后才开始 `timeout_seconds` 和运动计时。每帧识别只在服务内部进行；发给决策核的 `data/json` 话题只在 `task_started`、校准、过半、`rep_completed`、`step_completed`、超时或取消等事件发生时发送一条 `pose_check.event.v1` JSON。当前进度可随时用 `status(session_id)` 查询，不再逐帧刷到决策核。TTS 只消费消息中的 `narration`，不把 `info` 查询误当播报事件。

### 3. `exercise_tracker`

这是后续新增的流程卡，负责：

- 按计划维护动作顺序；
- 连续有效帧保持时间；
- 深蹲等动作的下蹲/站起阶段和重复次数；
- 防止同一动作重复计数；
- 处理相机过期、动作跳过、暂停、取消和恢复；
- 输出 `step_started`、`correction`、`rep_completed`、`step_completed`、`plan_completed`。

它不重新做姿态识别，而是消费 `pose_check` 的结构化结果。

### 4. `workout_skill`

上层 Skill 负责体验：开始前说明计划，动作完成后调用 TTS 鼓励，动作不正确时播放纠正反馈，计划全部完成后播报总结并提醒休息/补水。TTS 和手臂、头部动作都由 Skill 调用对应 actuator 卡片。

## 推荐的最小计划

```yaml
name: office_stretch
steps:
  - pose: hands_up
    hold_seconds: 0.5
    repetitions: 1
  - pose: arms_open
    hold_seconds: 0.5
    repetitions: 1
  - pose: one_hand_up
    side: both
    hold_seconds: 0.5
    repetitions: 1
  - pose: squat
    repetitions: 5
```

首版建议先完成三个上肢动作，再把深蹲作为第二阶段；深蹲要求髋、膝、脚踝完整入画，并需要更严格的机位验收。

## 反馈策略

- `retry`：提示重新进入画面或重新做动作。
- `almost`：播报 `feedback`，同一反馈限频，避免连续刷屏。
- `completed`：只在状态从未完成变为完成时鼓励一次。
- `task_started` / `calibration_completed`：任务开始提示一次“请站直”，站直稳定后提示“开始计时”。
- `rep_completed`：播报“第 N 次完成”。
- `progress_milestone`：达到目标一半时播报当前次数和剩余次数。
- `plan_completed`：播报完成总结，并提醒休息、喝水或下一计划。

## 验收标准

1. 相机持续有新鲜帧时，`pose_check` 延迟和帧率可测量。
2. 明显正确动作不能漏判，明显错误动作不能误判为完成。
3. 关键点不可靠时返回 `retry`，不猜测姿态。
4. 同一重复动作只计数一次。
5. TTS 不写入 `pose_check`，由上层 Skill 统一调度。
6. 需要跨机器人时，只替换相机 topic、消息转换和部署依赖，保留同一套姿态规则。

## 后续顺序

1. 固化 v5 的天轶 raw Image 输入和单帧 `check` Schema。
2. 增加天轶现场的延迟、帧率和动作混淆测试。
3. 将 `pose_session` 收敛为独立 `exercise_tracker`，不继续扩张 `pose_check`。
4. 在画布接通 `camera_head → pose_check → exercise_tracker → tts`。
5. 最后再接 `arm_gesture`、计划存储和完成提醒。
