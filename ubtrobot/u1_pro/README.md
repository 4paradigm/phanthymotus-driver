# 优必选 U1 Pro 驱动

本驱动为 Agent Core 提供 U1 Pro 的音频、相机、动作和系统行为开关卡片。机器人 SDK 控制服务使用 ROS domain `20`，设备音频/相机 topic 使用 domain `2`，Agent Core 使用 domain `42`。

## 卡片

- `mic`：调用 U1 SDK 的音频 `open_stream`/`stream_state`，优先从 SDK 音频共享内存读取；启动时给 SDK ring 1 秒首帧优先窗口，未收到 SDK 帧才切换到设备 domain `2` 的 `/sys/device/audio_in/raw` topic 备用源。驱动按消息采样率、声道和格式转换为统一的 `audio/pcm-16k`，启动时等待首个 PCM 帧；失败会关闭流并报告错误。
- `speaker`：订阅连接到卡片的 `audio/pcm-16k` 输入，优先通过设备 PulseAudio TCP sink 实时播放；`PULSE_SERVER` 环境变量可覆盖默认地址 `tcp:127.0.0.1:34567`。运行镜像中的 `pulseaudio-utils` 提供 `paplay`，设备侧必须开放对应 PulseAudio sink。队列短时满载时施加背压以保留 PCM；播放进程退出或 writer 持续无进展时停止接收并报告错误，避免无限阻塞 ROS 回调。未安装或无法启动 `paplay` 时才尝试已验证可用的 `EnableAudioOut`/`/sys/device/audio_out/raw` 兼容路径；两条路径都不可用时拒绝启动。音量和静音服务不属于本驱动能力。

部署服务使用 `restart: always`，用于主机或 Docker daemon 重启后自动恢复驱动；运维人员手动停止容器后，Docker 会在 daemon 重启时再次启动它。若需临时停用，应通过部署编排管理该服务，而不是依赖 `docker stop` 保持停机。
- `tts`：通过 U1 SDK `play_text` 进行文本播报，支持打断和 ACP 异步完成回报。真机音量服务不纳入本驱动能力。
- `expression`、`head`：通过 SDK `play_action` 播放声明的动作名称。动作提交后以 ACP 关联播放结果；`interrupt`/`stop` 会调用 SDK 中断服务。两张卡共享同一个动作执行队列。
- `system_controls`：统一控制 `wakeup`、`wakeup_followup` 和 `visual_behavior` 三个固件行为开关。
- `camera_left`、`camera_right`：打开 SDK video stream，并分别订阅 domain `2` 的 `/sensor/camera/left_eye/color/raw`、`/sensor/camera/right_eye/color/raw`，转换后输出 JPEG。SDK 共享内存视频流没有选择左右眼的参数；只有元数据明确标记了对应眼睛时，才会将环形缓冲帧作为该眼的备用输入，避免把一幅画面伪装成左右两路。

## 启动与行为控制

启动时先检查/完成 SDK 授权，然后依次关闭并读取确认 `wakeup`、`wakeup_followup` 和 `visual_behavior`。任一关闭请求失败或读回状态不符，驱动启动失败，不会注册卡片。需要使用时，可通过 `system_controls` 单独或批量重新开启。

这些开关不能停止固件内部 Agent 进程、ROS 服务、安全控制环或已经提交的动作。关闭 `visual_behavior` 用于停止固件自主视觉行为，不会屏蔽驱动显式提交的 expression/head 动作；显式动作可通过对应卡片的 `interrupt` 中断。

授权使用只读挂载的 `U1_PRO_AUTH_FILE` JSON 和同目录 license 文件，或受保护的环境变量。认证文件不得提交到仓库或写入镜像。

## 运行环境

部署需要 host network、domain `20`/`2`/`42` 所需的 ROS 访问，以及对 `/tmp/robo/ipc` 和 `/dev/shm` 的运行时挂载。`/tmp/robo/ipc` 是临时运行态，不应作为持久配置目录；服务端每次打开流都会重新创建共享内存文件。容器需要 ROS Humble 的 CycloneDDS RMW。Docker 构建阶段的 colcon/CMake/C++ 工具仅用于编译本地 ROS 接口；最终镜像还包含运行期 Pillow、NumPy、PyYAML 和 CycloneDDS 依赖。

## 本地检查

```bash
PYTHONPATH=ubtrobot/u1_pro python3 -m unittest discover -s ubtrobot/u1_pro -p 'test_*.py' -v
python3 scripts/check_service_yml.py ubtrobot/u1_pro
```

## sound_direction：独立只读听声辨位

保留现有 mic，新增 `direction_card.py` 方位卡片，不新增 ASR、head_orientation 或自动转头。
其他分支的头部实验代码未纳入本分支。

### 网页接线

- `mic` 的 `/<namespace>/mic/audio`（`audio/pcm-16k`）→ 平台现有 ASR 的音频输入。
- `sound_direction` 的 `/<namespace>/sound_direction`（`sensor/sound-direction`）→ 同一 ASR 的方位输入。

公开方位 topic 使用 `std_msgs/msg/String` JSON，与 Bumi 的公开协议一致。
不要将方位连到音频端口，不混合两类消息；已有项目应重新添加卡片或重新接线以更新端口。

### 输入依据和坐标

2026-10-27 只读复核：SDK Adapter 的 ROS domain `20` 中
`/robo/audio/subscribe/doa_event` 实际类型为 `std_msgs/msg/String`。
同机 `walker-audio.audio-1` 容器内的
`/opt/walker/audio_msgs/share/audio_msgs/idls/audio_msgs/msg/DoaEvent.msg` 说明：
`azimuth` 单位度、范围 `[-180,180)`、左负右正、0°通常为正前方；`confidence` 为 `[0,1]`。
卡片订阅 Adapter 的 String JSON，而非该 typed 消息；复用已有直接 JSON 和 SDK `data`
包裹解析实现。尚未取得实时事件样本，Adapter JSON 形状仍须真机验收。

按文档转换 `angle_deg=-azimuth`，即前方 0°、左正右负、后方 +180°；
`angle_convention` 为 `robot_front_0_left_positive_ccw`。
这是厂商前方定义的符号适配：文档没有确定设备安装方向、是否随头部转动，
与实际机器人正前方的对齐仍须验收。不提供可任意猜测的安装偏移配置。
不跨事件平滑，避免混入上一位说话人的方向。
仅在 SDK 事件实际提供有效置信度时输出 `confidence`，缺失时省略。

### 公开状态和时间限制

SDK `azimuth=-45` 且包含 `confidence=0.8` 时，公开 JSON 的主要字段如下
（`timestamp_ms` 示例仅说明格式，实际为该事件的本地接收时间）：

```json
{
  "state": "fresh",
  "angle_deg": 45.0,
  "angle_convention": "robot_front_0_left_positive_ccw",
  "confidence": 0.8,
  "audio_window_start_us": 0,
  "audio_window_end_us": 0,
  "timestamp_ms": 1793000000000,
  "timestamp_basis": "receipt_time_only",
  "association_state": "unknown",
  "association_reason": "SDK DOA has no verified shared audio capture window"
}
```

无事件或事件明确无效为 `no_event`；最新消息满 2 秒后为 `stale`。
解析失败立即清除旧方向，发布 `no_event` 并附带 `error`。
非 fresh 状态的 `angle_deg` 均为 null，不用 0°代表无数据；不保留旧置信度。
每 500 ms 重发当前状态供迟到查看器/丢包恢复，心跳不更新事件时间或有效期。
`info` 返回生命周期状态、嵌套 `sound_direction` 观察值及 SDK 实际消息计数。

**当前没有已核实、与 mic 共享采集时钟的 DOA 音频窗口。** typed 定义中的 Header
不能证明 Adapter JSON 转发了它，也不能证明它与 mic 时间可关联。
不把接收时间或未经核实的 JSON 时间字段伪造为采集窗口，因此 `audio_window_*_us` 固定为 0。
平台现有 ASR 拒绝这样的窗口，在实际输出方向字段的唤醒词路径中保持
`speaker_direction_state=unknown`，避免把旧方向套到新的语音结果；平台无需修改。
接线和独立方位显示可验收，但**可靠的语音结果与方向关联尚未实现**，需厂商确认
Adapter 采集时间、窗口及其与 mic 时钟的对应后另行适配。
现有 ASR 的关联粒度为带时间的唤醒词，并非逐句说话人跟踪。

### 生命周期和事件生产条件

遵循当前 U1 传感器卡片生命周期：驱动启动时调用 start，网页也支持 `start/stop/info`。
重复 start 不重复订阅；stop 发布 `no_event`、清除数据并销毁订阅/定时器/发布者。
重启不继承旧样本，已排队的旧订阅回调不能写入新会话。

卡片只订阅 DOA，不调用唤醒、跟随或运动服务，不改变行为开关。
已有检查材料提示厂商 DOA 生产可能依赖原生唤醒条件；平台 ASR 唤醒词不等于厂商唤醒词。
当前驱动已有初始化会关闭厂商行为开关，本次保持该行为。
若一直无事件，只能报告 `no_event`，不能为出数据自动打开开关。本次未部署或执行动作。

复用已有 std_msgs/rclpy 依赖及 domain 20/42 节点，无需新自定义消息或定位算法。
Dockerfile 已包含新模块。

### 网页真机验收（操作者执行）

1. 部署与 PR 最终提交一致的 Try it 镜像，确认原卡片及新增 sound_direction 可见。
2. 分别接 mic→ASR 音频输入、sound_direction→ASR 方位输入，确认原语音识别正常。
3. 观察 info 和实时流：首次无事件为 no_event/null；若计数始终为零，记录厂商事件
   生产条件与固件版本，不通过本卡自动改开关。
4. 前、左、右、后方分别发声，核对前方 0、左正右负；记录实际 JSON、安装偏移及
   是否相对头部。多人交替说话时确认新方向未混入旧方向。
5. 最新事件 2 秒后应为 stale/null；stop 后 no_event，重新 start 后无旧方向。
6. 在 ASR 唤醒词路径确认方向保持 unknown，不出现基于接收时间的假关联。
   本次不能以“识别文本带精确方向”作为已完成验收；采集时间缺口需单独解决。

离线测试覆盖解析、角度、置信度缺失、无事件/过期、生命周期、重启隔离和部分启动清理。
物理方位精度由操作者验收。
