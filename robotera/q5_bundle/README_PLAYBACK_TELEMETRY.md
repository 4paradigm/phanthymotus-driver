# Q5 Speaker Playback Telemetry

## 实际链路

```text
AgentCore / TTS (ROS domain 42)
  -> audio_msgs/AudioChunk
  -> q5_media_bridge.py (Domain 42 / FastDDS subprocess)
  -> multiprocessing speaker queue
  -> SpeakerPlugin._pump() (Q5 Driver, Domain 211 process)
  -> framed SSH stdin
  -> embedded q5_speaker_playback Python process on Q5
  -> audioop 16 kHz mono -> 44.1/48 kHz stereo
  -> snd_pcm_writei + snd_pcm_delay
  -> hw:2,0
```

`AudioChunk.msg` 只有 `header`、`format` 和 `data`，没有原生 `speech_id` 或结束标志。普通 `audio/pcm-16k` 仍照常播放，但不能得到可靠的逐句状态。

## 可追踪 AudioChunk 协议

逐块在 `format` 后附加三个参数：

```text
audio/pcm-16k;speech_id=speech_001;sequence=0;eof=0
audio/pcm-16k;speech_id=speech_001;sequence=1;eof=1
```

- `speech_id`：UTF-8 标识经 URL percent-encoding，解码后 1～128 字符。
- `sequence`：从 0 开始严格递增；重复块被幂等丢弃，缺号使该任务失败。
- `eof`：`0` 或 `1`；最后一个数据块设为 `1`，也允许空 EOF 块。
- 每个块都必须携带完整三字段。元数据不完整时不退化成另一条语音。

生产者示例：

```python
from urllib.parse import quote

msg.format = (
    "audio/pcm-16k;"
    f"speech_id={quote(speech_id, safe='')};"
    f"sequence={sequence};eof={int(is_final)}"
)
msg.data = pcm16_mono_bytes
```

平台 TTS 生产者不在本仓库中；它必须采用此格式后，端到端逐句遥测才会生效。

## Speaker 接口

原有 `start`、`stop`、`set_volume`、`get_volume` 和 `info` 保持兼容。新增：

- `get_playback_status(speech_id)`：返回状态、输入样本、输出帧、已提交帧、已消费帧和进度。
- `wait_for_completion(speech_id, timeout_seconds)`：等待 `completed`、`failed` 或 `interrupted`；调用超时不篡改任务状态。
- `get_recent_events(speech_id?, limit?)`：读取进程内有界事件历史。
- `get_capabilities()`：返回观测能力、时钟域、事件 topic 和声学能力限制。

Domain 42 事件订阅 topic：

```text
/<namespace>/speaker/playback_events
```

消息类型为 `std_msgs/String`，内容为 JSON。典型完成事件：

```json
{
  "speech_id": "speech_001",
  "event": "device_playback_completed",
  "state": "COMPLETED",
  "source_clock_id": "q5_audio_host_monotonic",
  "source_timestamp_ns": 123456789,
  "local_received_monotonic_ns": 987654321,
  "consumed_frames": 88200,
  "total_output_frames": 88200,
  "timestamp_quality": "DEVICE_PROGRESS_OBSERVED"
}
```

## 进度和完成语义

远端只累计 `snd_pcm_writei` 实际返回的正帧数；部分写继续提交，`EAGAIN` 不推进计数。`snd_pcm_delay` 给出 ALSA 尚未消费的输出帧，系统据此计算全局消费位置，再映射到每条语音的输出帧区间。通道数不乘入时长；时长为 `output_frames / device_sample_rate_hz`。

`EPIPE` 会调用 `snd_pcm_prepare` 恢复后端，但当前语音及最后可信消费边界之后仍可能位于设备缓冲区的语音都会立即标为 `FAILED/ALSA_UNDERRUN`。若没有可信消费边界，则保守失败全部可能受影响的活动语音；同一 `speech_id` 的后续块会被丢弃，无关的新语音仍可继续。设备重开、SSH 断线、非法序号、空 PCM 和停止操作也不会产生 completed。

只有同时收到 EOF、已知总输出帧、可信设备消费帧达到总帧数且没有终止错误时才会完成。远端持续流不对每句话调用 `snd_pcm_drain`；`drain` 只在整个远端进程关闭时使用，并且只有成功返回才形成可信证据。逐句完成通常来自 `snd_pcm_delay` 表示的 ALSA 消费边界。

远端记录有固定容量。创建新语音前只清理已终止记录，绝不淘汰活动记录；容量被活动任务占满时以 `PLAYBACK_CAPACITY` 拒绝新任务。连续无进展写入、持续 `snd_pcm_delay` 失败、消费进度超时、排空失败和播放器异常退出都会产生明确失败事件，避免任务永久挂起。

## 时间与可信度

- Driver 事件：`driver_host_monotonic`。
- Q5 远端播放器：`q5_audio_host_monotonic`。
- 两个 monotonic 时钟不直接相减，也不伪造跨主机映射。
- PCM 提交：`PCM_SUBMITTED`。
- ALSA delay 进度/完成：`DEVICE_PROGRESS_OBSERVED`，完成层级 `alsa_device_consumption`。
- 没有麦克风回采或硬件声学时间戳，因此不支持 `ACOUSTIC_OBSERVED`。

## 无硬件测试

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  robotera/q5_bundle/test_playback_telemetry.py \
  robotera/q5_bundle/test_fake_playback_backend.py \
  robotera/q5_bundle/test_speaker_playback_telemetry.py \
  robotera/q5_bundle/test_q5_bus_bridge.py \
  robotera/q5_bundle/test_audio_timing.py -q

python3 robotera/q5_bundle/playback_telemetry_soak.py \
  --speeches 10000 --chunks-per-speech 10 --seed 20261020
```

这些命令不打开 ALSA、不执行 SSH、不产生声音。

## 构建与配置

配置默认开启：

```yaml
plugins:
  speaker:
    telemetry_enabled: true
    telemetry_recent_events: 2048
    telemetry_max_speeches: 10000
```

修改涉及 Driver 镜像内的 Python 文件，需要重新构建镜像：

```bash
./build.sh --mirror none robotera/q5_bundle
```

本任务不自动替换镜像、不重启容器。远端播放器由 Driver 动态生成，不需要另行安装文件，但必须用新 Driver 启动新的 `q5_speaker_playback` 进程。

## 有人值守的真机验收

1. 在低音量、无机器人运动任务时部署新镜像，并确认 `telemetry_enabled: true`。
2. 连接带 `speech_id/sequence/eof` 的短 PCM 源，订阅 playback event topic。
3. 确认 `PCM_SUBMITTED` 早于 `COMPLETED`，进度单调且输出帧率为实际 44.1/48 kHz。
4. 连续播放 A/B/C，确认事件 ID 和完成顺序独立且音频不乱序。
5. 人工断开 SSH 或停止播放器，确认活动任务 FAILED，绝不 completed。
6. 在可恢复环境中注入 underrun，确认 `ALSA_UNDERRUN` 和最后可信进度。
7. 用外部录音/示波设备另测功放和扬声器尾延迟；该值不能由本遥测推断。

## 能力边界（A/B/C）

- A. 能否知道 PCM 已全部提交给声卡？对采用追踪元数据的语音可以：依据实际 `snd_pcm_writei` 返回帧数；旧格式只能给出流级写入信息。
- B. 能否知道某条语音已被播放设备消费完毕？对采用追踪元数据且 `snd_pcm_delay` 正常工作的语音可以在 ALSA 语义下判断；仍需真机验证目标驱动的 delay 行为。
- C. 能否知道最后一个声音真正从物理扬声器结束？不能。需要麦克风声学回采、功放/codec 硬件时间戳或厂商提供的末端反馈。
