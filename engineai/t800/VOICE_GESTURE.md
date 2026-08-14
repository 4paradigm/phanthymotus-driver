# T800 Voice Gesture Extension

`voice_gesture.py` 是可选扩展：它订阅 Perception ASR 的最终 JSON，不采集音频、
不运行 ASR 模型，也不调用外部 LLM。原有 `main.py`、`device.py`、`MicPlugin` 和
`config.yaml` 不作修改。

## 数据流

```text
mic/audio -> Perception asr -> mic/audio/asr -> voice_gesture -> gesture.play
                                                `- 固定短语 -> gesture.play
```

默认 ASR 输入 Topic 为 `/{namespace}/mic/audio/asr`。实际 namespace 必须与 T800
Driver 和 Perception 中使用的 namespace 一致；部署时可通过 `asr_topic` 显式配置。

Perception 当前输出形如：

```json
{"text":"你好","kws_triggered":true}
```

只有包含文本、不是显式非最终结果、并且（默认）`kws_triggered: true` 的事件才会
被处理。`cooldown_sec` 防止一句话或重复回声触发多次动作。

## 规则动作

从 `voice_gesture.example.yaml` 把 `voice_gesture:` 段复制到独立的
`config.voice-gesture.yaml` 的 `plugins:` 下。动作配置定义的是固定的 `action_id`
到现有 Driver `gesture` 的映射；ASR 文字不会被当作任意函数或关节参数执行。

默认提供：

- `你好` / `您好` / `打招呼` -> `gesture.play(wave_hands)`
- `握手` / `握个手` -> `gesture.play(shake_hand)`

## 独立镜像与入口

先构建原 Driver 镜像，再构建扩展镜像；不改原 Dockerfile：

```bash
cd engineai/t800
docker build -t engineai-t800-driver .
docker build -f Dockerfile.voice-gesture \
  --build-arg BASE_IMAGE=engineai-t800-driver \
  -t engineai-t800-voice-gesture .
```

运行扩展镜像时，用 `CONFIG_PATH` 挂载独立配置：

```bash
docker run --rm --network host --privileged \
  -e CONFIG_PATH=/work/config.voice-gesture.yaml \
  -v /path/to/config.voice-gesture.yaml:/work/config.voice-gesture.yaml:ro \
  -v /dev:/dev \
  -v /opt/engineai/native_sdk:/opt/engineai/native_sdk \
  -v /run/user/1000/pulse:/run/user/1000/pulse \
  -v /home/ubuntu/.config/pulse:/root/.config/pulse:ro \
  -e NETWORK_INTERFACE=eth1 \
  -e PULSE_SERVER=unix:/run/user/1000/pulse/native \
  engineai-t800-voice-gesture
```

扩展入口是 `voice_gesture_main.py`；它在运行时把插件追加到原 Bundle。因此旧的
T800 Driver 镜像/入口可继续用于回退。
