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

## Driver MCP 功能卡片

卡片（tool）名称是 `pose_game`，description 是
`Bounded Pose capture windows for offline action scoring`。它由 `config.yaml`
中的同名启用项自动加载，属于 Q5 Driver MCP 功能卡片，不是 AgentCore Skill。

当前 `COMPLETED` 的 MCP 示例只用于本地 Mock 流程验证。普通 TTS 的受跟踪元数据尚未与该卡片形成 Driver 内部可信编排，因此部署后不能让 Dashboard 用户手工填写一份播放结果并将其当作真实可信完成。真实设备链路打通前，生产调用应由受信任的 Driver 编排者提供 Speaker Tracker 结果，或者使用实际的失败、超时、取消等非完成终态。

列出卡片：

```json
{"jsonrpc":"2.0","id":1,"method":"tools/list","params":{}}
```

返回的 `result.tools` 中应包含一个 `name=pose_game`、`type=sensor` 的工具，action 为
`start_round`、`finish_round`、`export_round` 和 `info`。

在提交 TTS 之前开始一轮：

```json
{"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"pose_game","arguments":{"action":"start_round","round_id":"round-1"}}}
```

预期文本内容解码后包含：

```json
{"round_id":"round-1","state":"RECORDING","started_monotonic_ns":123456789}
```

播放期间可查询采集状态：

```json
{"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"pose_game","arguments":{"action":"info"}}}
```

返回包含 `active_round`、`buffered_frames`、容量限制，以及 Bridge 丢帧、Recorder
淘汰和无效输入等分类计数。

在离线 Mock 测试中观察到模拟的可信 Speaker 完成后，调用方继续等待完整 3 秒，再冻结本轮。以下时间仅为 Mock 格式示例，不代表已验证的真机生产调用：

```json
{"jsonrpc":"2.0","id":4,"method":"tools/call","params":{"name":"pose_game","arguments":{"action":"finish_round","round_id":"round-1","outcome":"COMPLETED","completion_observed_monotonic_ns":1000000000,"playback_result":{"result":"completed","completion_level":"alsa_device_consumption","timestamp_quality":"DEVICE_PROGRESS_OBSERVED","playback_finished_at_ns":999000000000,"error_code":null,"error_message":null,"underrun_count":0}}}}
```

正常返回包含 `outcome=COMPLETED`、窗口起止时间、`complete`、
`bridge_transport_drops`、`incomplete_reasons` 和完整 `frames`。失败、超时、取消或不可信完成必须使用对应终态，不能声明 `COMPLETED`。

导出已经冻结的轮次：

```json
{"jsonrpc":"2.0","id":5,"method":"tools/call","params":{"name":"pose_game","arguments":{"action":"export_round","round_id":"round-1"}}}
```

返回与冻结时相同的轮次快照。`frames` 中每帧包含原始 JSON、源 timestamp、Bridge/Driver monotonic 时间和 persons 数据。

部署包含本文件及配置的新 Q5 Driver 镜像并重启现有 Driver 后，Dashboard 应显示名为
`pose_game` 的卡片。现有 bundle MCP 注册线程会向 Agent Core 注册整个 Q5 Driver，
不需要再创建独立 MCP 服务、额外注册卡片或安装 AgentCore Skill。

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


### Bot Review 安全修复（PR #439）

- Driver 画布生命周期支持 `start` / `stop`，与轮次 `start_round` 分离。
- 默认 live 缓存 300 帧、冻结轮次 10 个、每轮最多 300 帧、Pose JSON 最大 8 KiB；保留原始 JSON 的配置理论上限 32 MiB。
- 限制嵌套 JSON 节点数与深度，解析对象和导出副本仍额外占内存；超过限制会拒绝采集而不影响后续数据。
- `pose_game` 已加入 `driver.yaml` 的 Driver marketplace 卡片清单。
