# T800 Voice Gesture Extension

`voice_gesture.py` 是可选扩展：它订阅 Perception ASR 的最终 JSON，不采集音频、
不运行 ASR 模型，也不调用外部 LLM。它直接使用众擎官方的
`JointMotionPlanRequest` / `JointMotionPlanState`；不依赖 Driver 的 `GesturePlugin`。
原有 `main.py`、`device.py`、`MicPlugin` 和 `config.yaml` 不作修改。

## 数据流

```text
mic/audio -> Perception asr -> mic/audio/asr -> voice_gesture
                                                `- 官方 JointMotionPlanRequest
```

默认 ASR 输入 Topic 为 `/{namespace}/mic/audio/asr`。实际 namespace 必须与 T800
Driver 和 Perception 中使用的 namespace 一致；部署时可通过 `asr_topic` 显式配置。

Perception 当前输出形如：

```json
{"text":"你好","kws_triggered":true}
```

只有包含文本、不是显式非最终结果、并且（默认）`kws_triggered: true` 的事件才会
被处理。`cooldown_sec` 防止一句话或重复回声触发多次动作。

`led_feedback.enabled` 默认为启用：当最终 ASR 结果带有 `kws_triggered: true` 时，
Driver 直接向官方 `LedControl` Topic 发布 `blink_green`；默认一秒后改为
`constant_white`。这不依赖 Dashboard，也不调用 `GesturePlugin`。该信号是在整句识别
完成后确认唤醒成功，不是说出唤醒词瞬间的指示。

`auto_enable_motors` 默认启用。插件只会在唤醒词和动作关键词均匹配时尝试调用
`/hardware/enable_motor`，不会在容器启动时自动上电，也不会在插件停止时自动失能。
T800 实机通常仍需先用遥控器完成物理使能。部分 runtime 不提供该 ROS 服务，因此默认
`motor_enable_required: false`：服务不存在、超时或拒绝时发布 `motor_enable_degraded`，
随后继续向官方关节规划器提交动作，由规划器状态决定是否接受。需要服务失败时立即终止
动作的部署可设为 `true`；也可设置 `auto_enable_motors: false` 完全跳过服务调用。
部署前必须确保急停已释放、机器人姿态稳定且周围无人。

插件的事件既发布到 `/{namespace}/voice_gesture/events`，也以 `[voice_gesture]` 前缀写入
容器标准输出，便于直接通过 `docker logs` 区分使能降级、规划发布和规划器超时。

## 规则动作

仓库内的 `config.voice-gesture.yaml` 是可直接挂载到 T800 的完整实机配置，固定使用
`ubuntu` namespace、`/ubuntu/mic/audio/asr` 输入和 `/ubuntu/voice_gesture/events` 输出。
`voice_gesture.example.yaml` 继续保留为仅含 `voice_gesture:` 的配置片段，供其他 namespace
或派生配置复用。每个动作的关节轨迹直接写在 `motion_plan` 内；ASR 文字不会被当作任意
函数、关节参数或路径执行。

默认提供：

- `你好` / `您好` / `打招呼` -> 内嵌挥手规划
- `握手` / `握个手` -> 内嵌握手规划

若已收到官方规划器状态，插件先等待 `IDLE`；若固件尚未发布初始状态，则先发送第一条
官方规划并使用本地递增 request id。每一步仍等待 `EXECUTING`，再等待同一 request 回到
`IDLE`，才发送下一步。

## 独立镜像与入口

先构建原 Driver 镜像，再构建扩展镜像；扩展镜像会同时内置完整实机配置：

```bash
cd engineai/t800
docker build -t engineai-t800-driver .
docker build -f Dockerfile.voice-gesture \
  --build-arg BASE_IMAGE=engineai-t800-driver \
  -t engineai-t800-voice-gesture .
```

运行扩展镜像时，可用 `CONFIG_PATH` 直接读取镜像内配置。开发时若要覆盖配置，再挂载仓库
里的同名文件：

```bash
docker run --rm --network host --privileged \
  -e CONFIG_PATH=/work/config.voice-gesture.yaml \
  -v /dev:/dev \
  -v /opt/engineai/native_sdk:/opt/engineai/native_sdk \
  -v /run/user/1000/pulse:/run/user/1000/pulse \
  -v /home/ubuntu/.config/pulse:/root/.config/pulse:ro \
  -e NETWORK_INTERFACE=eth1 \
  -e PULSE_SERVER=unix:/run/user/1000/pulse/native \
  engineai-t800-voice-gesture
```

覆盖镜像内配置时增加：

```bash
-v /path/to/config.voice-gesture.yaml:/work/config.voice-gesture.yaml:ro
```

扩展入口是 `voice_gesture_main.py`；它在运行时把插件追加到原 Bundle。因此旧的
T800 Driver 镜像/入口可继续用于回退。
