# Q5 Pose Playback Window

`pose_game` 是离线原型：它通过现有 `q5_media_bridge` 订阅
`/nvidia_desktop/camera/rgb/poses`，在独立泵线程中持续记录 JSON Pose，供每轮动作判分模块导出完整序列。它不执行动作识别或机器人控制。

## 执行模型与时间窗口

- Bridge ROS 回调、Driver Pose 泵线程和 TTS/Agent 调用互不阻塞。
- `start_round` 必须在提交 TTS 前调用。
- 可信播放完成由现有 Speaker Telemetry 定义；本模块不降低完成条件。
- 完成事件在 Driver 被观察到后继续采集整整 3 秒，窗口两端均包含。
- Bridge 与 Driver 是同主机、同启动周期的父子进程，因此二者的 `time.monotonic_ns()` 可比较；窗口归属只使用 Bridge monotonic 时间。
- ROS 源 `timestamp` 和远端 ALSA 时间仅作为元数据保存，绝不参与窗口比较。如果以后 Bridge 移到另一台主机，必须先增加时钟映射。
- 冻结前通过同一有界 Pose 队列发送 barrier，确保 barrier 之前已经排队的数据先被 Driver 消费；冻结上界使用 barrier 自身的 Bridge monotonic 时间，而不是稍后的 Driver 唤醒时间。barrier 超时会显式失败，并保留 active round。

## 数据与容量

每帧保留原始 JSON、源 timestamp、Bridge monotonic 时间、Driver 接收 monotonic 时间，以及所有带有效 `id` 的 person 字段。关键点不是必填项；未知字段原样复制，便于以后扩展。

默认 live 环形缓存 600 帧、冻结轮次 20 个、每轮快照 600 帧，单条 Pose JSON 最大 1 MiB。状态中的计数分别表示 Bridge 传输丢帧、Recorder 缓存淘汰、冻结轮次淘汰、快照截断、超限消息和无效输入。本轮 Bridge 丢帧、淘汰本轮帧或快照截断都会令导出的 `complete=false`，并写入 `bridge_transport_drops` 与 `incomplete_reasons`；内存不会随消息数量或单条消息大小无限增长。

## 插件动作

- `start_round`: `{"round_id": "round-1"}`
- `finish_round`: `round_id`、六种终态之一，以及可选 `playback_result` 和 `completion_observed_monotonic_ns`
- `export_round`: `{"round_id": "round-1"}`
- `info`: 当前轮次、容量和分类计数

自动编排使用 `PoseRoundController`。以下示例展示调用顺序；`submit_tts` 与 `wait_for_playback` 需由上层注入：

```python
recorder = PoseRecorder()
controller = PoseRoundController(
    recorder,
    submit_tts=submit_tts,
    wait_for_playback=wait_for_playback,
)
# run_round 内部先 start_round，再提交 TTS。
snapshot = controller.run_round("round-1", "请举起右手", 15.0)
# 独立 Pose 泵在线程中持续采集，包括播放等待和完成后 3 秒。
assert snapshot["outcome"] == "COMPLETED"
sequence = recorder.export_round("round-1")["frames"]
```

## 验证范围

本地测试只使用 Fake Clock、Fake Pose 队列和 Mock TTS/播放事件，验证了阻塞 TTS 期间采集、3 秒边界、10 轮隔离、容量限制、异常输入、失败/超时/取消和冻结 barrier。普通 TTS 的 `speech_id/sequence/eof` 元数据链路、真实 Q5 ALSA 完成事件、真实 Pose 频率与跨进程延迟仍未在设备上验证，不能把 Mock 结果视为真机通过。

真机验收时应只读观察：确认两个进程处于同一主机/启动周期，Pose topic 持续发布；使用带完整跟踪元数据的 TTS；核对播放期间与完成后 3 秒的帧时间边界；制造可控队列压力验证丢帧计数；最后检查停止插件后订阅与泵线程都退出。部署、音频路由和机器人动作需另行批准。
