# 优必选 U1 Pro 驱动

本驱动为 Agent Core 提供 U1 Pro 的音频、相机、动作和系统行为开关卡片。机器人 SDK 控制服务使用 ROS domain `20`，设备音频/相机 topic 使用 domain `2`，Agent Core 使用 domain `42`。

## 卡片

- `mic`：调用 U1 SDK 的音频 `open_stream`/`stream_state`，优先从 SDK 音频共享内存读取；启动时给 SDK ring 1 秒首帧优先窗口，未收到 SDK 帧才切换到设备 domain `2` 的 `/sys/device/audio_in/raw` topic 备用源。驱动按消息采样率、声道和格式转换为统一的 `audio/pcm-16k`。两条输入路径共用流式语音增强：高通削弱低频机械噪声，启动后自动学习环境噪声底，非人声衰减 `24 dB`，检测到人声时快速提升至目标响度并保留 `280 ms` 字尾，最后进行平滑限幅。人声最大补偿默认 `18 dB`，可通过部署环境变量 `U1_PRO_MIC_GAIN_DB` 在 `0～24 dB` 范围内覆盖。`mic.info` 会返回当前输入电平、噪声底、人声状态和实际增益，便于真机调试。启动时等待首个 PCM 帧；失败会关闭流并报告错误。
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

## sound_direction：独立厂商定位数据源

公开卡片仍为 `sound_direction`，topic 为 `/<namespace>/sound_direction`，
格式 `sensor/sound-direction`，传输为 `std_msgs/msg/String` JSON。
前方 0°、左正右负、后方 +180°；保留原 mic，不新增 ASR、head_orientation 或自动转头。

### 为什么 SDK 事件桥接不足以工作

现场核实了 domain 20 的 `/robo/audio/subscribe/doa_event` 为 String，
但默认关闭原生行为时，该 topic 没有持续方向事件。厂商运行日志表明 DoaNode
在原生唤醒后才启动一次 500ms 定位；其实际分数还可能为 4.31，
并不符合 DoaEvent.msg 注释中的 `[0,1]` 概率范围。
因此原来只订阅 SDK topic 的实现不能提供普通说话时的持续方向输出。

现在默认 `sound_direction.source: native`：复用机器已安装的厂商 DOA 模型和
`voice_doa::DoaPlugin`，在独立 domain **43** 的只读容器运行。
没有复制 Bumi 定位算法、重新训练模型或将厂商模型/库加入本仓库。
`source: sdk` 保留为原始事件模式，仍依赖厂商事件生产条件，不能保证普通说话时有数据。

### 数据路径和行为隔离

1. 现有 domain 2 的 `/sys/device/audio_in/raw` 原样单向转发到 domain 43
   的 `/phanthy/u1_doa/raw`，不改变现有 mic 的 PCM 通路。
2. `/sys/device/audio_in/cur_device_info` 的真实设备状态也原样转发。
   此消息必须使用 RELIABLE / TRANSIENT_LOCAL；只有音频而缺少设备状态时，厂商节点会丢帧。
3. 私有容器复用真实音频容器的配置卷，**只读**取得麦克风通道布局；仅用镜像而不挂该卷会缺少布局。
4. 厂商 AudioSubscriptionNode、AecNode 与 DoaNode 通过原生流式黑板连接。
   AEC 沿用厂商设备布局，把输入整理为定位所需的六个通道。
5. 私有流程用间隔脉冲触发推理，避免每帧重置厂商 500ms 窗口。
   这只是 domain 43 内的局部布尔值，不发布原生 wakeup_event、不改变系统开关。
6. typed `/phanthy/u1_doa/event` 返回驱动，驱动取反厂商左负右正角度后发布公开 JSON。

定位容器不加载任何运动、KWS、唤醒事件发布或播报节点；没有声卡设备、宿主 PID、
授权目录、SSH 目录或 Docker socket 挂载，不使用 privileged。配置卷只读，根文件系统只读。
不修改或重启厂商 `walker-audio.audio-1`。驱动 DDS 发现限于 loopback。

2026-10-27 私有通路实测收到 11 条新厂商定位事件；操作者站在右侧，
末次原始角度 +80°，转换为平台 -80°，同段日志出现 +75°。
这验证真实事件和角度符号，不代表全部方位精度已验收。

### 安装定位容器

需要机器人已运行厂商音频容器，默认名称 `walker-audio.audio-1`。
管理脚本使用该**正在运行的容器的精确 image ID**和配置卷，不下载其他厂商版本。
SDK 模型或授权材料不会被写入驱动镜像或 PR。

先拉取 PR 最终驱动镜像，将下面的 `FINAL_DRIVER_IMAGE` 替换为对应最终版本。
从镜像取出管理脚本（临时容器只用于读取文件，不会启动）：

```bash
image=FINAL_DRIVER_IMAGE
cid=$(docker create --entrypoint /bin/true "$image")
docker cp "$cid:/deploy/native_doa_manage.py" /tmp/u1-native-doa-manage.py
docker rm "$cid"
sudo python3 /tmp/u1-native-doa-manage.py start --driver-image "$image"
```

然后使用该镜像正常部署 U1 驱动。定位容器名为 `embodied-u1pro-native-doa`，
独立重启策略为 `unless-stopped`。脚本拒绝使用非支持的优必选音频仓库镜像。
厂商音频镜像或配置卷升级后，重新执行 start。
其配置按驱动 image ID 保存，更新时不覆盖运行中的配置目录。

```bash
sudo python3 /tmp/u1-native-doa-manage.py status
sudo python3 /tmp/u1-native-doa-manage.py stop
```

stop 只删除带有本脚本所有权标签的私有定位容器，拒绝删除同名的其他容器。
普通驱动 Try it 的 `--down` 不会删除这个辅助容器；退出测试时需另行执行上述 stop。

### 状态、时间和生命周期

公开 JSON 的主要字段如下（示意值，实际分数由模型提供）：

```json
{
  "state": "fresh",
  "angle_deg": -80.0,
  "angle_convention": "robot_front_0_left_positive_ccw",
  "confidence": 2.86,
  "confidence_semantics": "vendor_raw_score_not_probability",
  "source_backend": "native",
  "audio_window_start_us": 0,
  "audio_window_end_us": 0,
  "timestamp_basis": "receipt_time_only",
  "association_state": "unknown"
}
```

- `confidence` 仅在真实事件含有有限分数时输出，保留原值，不归一化、不限幅、不当作概率。实测厂商模型既输出大于 1 的值，也输出负值（例如 -1.0812）；负分数不等同于缺少事件，NaN/无穷值仍按无效事件处理。
- 初次无数据为 `no_event`；最新有效事件超过 2 秒为 `stale`。
  无效、无事件和过期时 `angle_deg=null`，不使用 0°冒充缺失数据。
- 每 500ms 重发状态；重发不刷新事件接收时间。不同事件之间不做角度平滑。
- `info.source_status` 提供原始音频转发数、设备状态更新数、native 事件数、
  工作容器订阅者数和 `waiting_for_device/worker_not_discovered/waiting_for_audio/receiving/error`。
  已确认的订阅端点消失会立即清除旧方向；静默失联仍由 2 秒过期界限兜底。
- 当前设备必须被厂商 internal_mics.yml 识别为阵列，由私有流程的 IsInternalMicNode 把关。
  真机元数据的 is_built_in 实测为 False，不能仅凭该标志拒绝实际阵列。
  设备切换或音频中断后，清除旧事件并等待输入重新稳定，
  不把工作容器缓存的旧方向当作新观测。
- 卡片 start 创建自身的订阅/发布者和 domain 43 上下文；stop 停止音频转发、发布 no_event
  并释放自身资源；不销毁共用的 domain 2/42 节点。定位容器在无输入时等待。
  重启卡片不会继承旧事件。定位容器进程的管理使用上面的独立脚本。

定位是阵列对声音的估计，不提供说话人身份；环境声也可能产生方向，分数低时需谨慎解释。
完整物理方位精度仍由操作者验收。

尚未验证 DOA 采集窗口与 mic 共用时钟。`timestamp_ms` 仅为事件接收时间，
`audio_window_*_us` 固定为 0；**现有 ASR 的方向关联仍应为 unknown**。
不得将周期触发或接收时间伪造成精确采集窗口。本次先验收独立卡片输出。

### 网页验收

1. 确认私有定位容器和最终驱动镜像均已启动；添加 sound_direction，查看实时流及 info。
2. 在前、左、右、后分别持续说话数秒：前约 0°、左正右负。核对设备安装偏移和是否相对头部。
3. info 应有持续增长的音频转发数和真实方向事件数。若为 worker_not_discovered，先检查辅助容器；
   waiting_for_device 检查设备状态 topic，而不是打开行为开关。
4. 停止/重启卡片、暂停音频输入时，检查旧方向被清除或过期，恢复后只采用新事件。
5. 确认原 mic 仍可用于 ASR，三个系统行为开关保持部署前状态，不产生任何自动转头。

以后接 ASR 时：mic → ASR 音频输入；sound_direction → ASR 方位输入。
旧画布可能保存了只有音频输入的 ASR 卡片，需要平台端口迁移修补或重新添加 ASR。
这与本次独立方位输出是两项不同的验收。

### 声音活动门控（第一版）

native 后端现在先判断声音活动，再接受厂商方位事件；保持原始音频转发和厂商模型不变，不增加角度平滑，也不修改 mic/ASR。依据真机 internal_mics.yml，8 通道中仅 0–5 是物理麦克风，6–7 是回放通道。门控对前六路去直流，取各路 RMS 的中位数，转换为 dBFS；这不是现场声压级 dB SPL。

启动、设备切换、音频断流恢复或定位服务重新发现后，先用约 1 秒音频学习背景（建议此时保持安静）。阈值为 `max(minimum_dbfs, background_dbfs + margin_db)`。连续约 150ms 超过阈值后进入活动状态，再等待至少 0.5 秒才接受新定位结果；最近约 150ms 没有超过门限就抑制并清空方向，但保留最多 0.8 秒的活动上下文以容纳字词停顿。超过 0.8 秒安静后重新要求起声和等待。公开心跳最多再等 500ms，输出 `no_event`、`angle_deg: null`。低于阈值期间背景缓慢更新，活动期间冻结背景，避免持续说话/音乐把自身吸收为噪声。回放通道、直流、启动学习期、无效 PCM 都不能单独触发方向。

`config.yaml` 的 `sound_direction.activity` 可调整 `minimum_dbfs`（默认 -65，范围 -90 至 -10）和 `margin_db`（默认 2.5，范围 1 至 30）。默认值只是初始调试值，尚未经真机门控验收：漏掉较轻声音时可降低门限，背景误触发时可提高门限；过高会漏检短句或远处声音。暂不提供网页调参或持久化设置卡。

卡片 `info` → `source_status.activity` 提供 `active`、`reason`、`level_dbfs`、`background_dbfs`、`threshold_dbfs` 及当前配置。`calibrating` 为学习期，`below_threshold` 为被声音门限抑制，`settling` 为等待定位窗口更新，`sound_detected` 为允许接收事件，`unsupported_or_invalid_audio` 为输入布局/格式不受支持。仅支持已核验的 Bothlent UAC Dongle / Mic Arry 8、16kHz S16LE 8 通道布局，未知输入保守抑制。

这是接收时钟上的保守过滤，不是可靠采集时间关联：等待 0.5 秒降低前次发声结果混入的风险，但无法证明厂商排队消息绝不迟到。ASR 方向关联仍 unknown。持续且足够响的风扇/音乐仍可能过门限；本版不识别人声，也不提高厂商模型本身的角度精度。启动时持续响声可能被学习成背景；短促声音可能不输出方向，这是初版偏保守的取舍。

验收：启动后安静 2 秒，观察无角度；固定位置持续说话约 3 秒，应出现 fresh；停止说话后应清空角度；换到另一侧重新说话，等待后应出现新的方向。分别验证普通底噪、较轻说话、持续说话/音乐及重启。记录 `activity` 的水平与门限，再调整参数，不以角度看起来稳定替代精度验收。


现场 A/B 基线：操作者确认 `release.261029.474df1d` 真人说话和画布效果可用，手机音乐未触发。基线配置为 `minimum_dbfs: -65.0`、`margin_db: 3.0`，settle 0.5 秒、pause hold 0.8 秒。下一组仅将 margin 改为 2.5dB，其他不变；当前常见背景水平下有效门限降低 0.5dB。调参比较时应保持相同位置/距离，分别测试轻声、普通说话、安静误触发；音乐未触发不能单凭此判定硬件只支持人声，门控本身不做语音分类。

### mic 启动时 SDK open_stream 超时

部分固件会持续发布 domain 2 的 `/sys/device/audio_in/raw`，但 domain 20 的 `/robo/audio/call/open_stream` 不返回。驱动现在仅在该打开调用抛出 TimeoutError 时，启用已有原始音频转换路径；MicPlugin 仍须收到新 PCM 帧才报告 running，旧会话事件会清除，没有音频仍返回错误并停止转发。SDK 的明确拒绝/其他异常不被吞掉。

`mic info` 会保留 `sdk_warning` 并显示 `audio_source: topic`。这是音频输入降级，不是伪造 SDK 打开成功；超时的远端请求最终是否执行仍不确定，停止该输入只释放本驱动的转发，不冒认共享 SDK 流的所有权。无需重启厂商服务、改行为开关或重新接线。原始通道音质与 SDK 路径可能不同，真实语音识别效果仍需验收。


## 独立开发与可用版本回退

本分支 feat/u1pro-sound-direction-only 仅包含听声辨位、既有麦克风修复及必要接口，不包含同事未合并的 idle_actions 卡片或其原生控制模块/注册/依赖。当前真机认可部署是 release.261029.49fb570 + perception release.261029.f959e96-jetson-jp6.1；该历史整合镜像含 idle_actions，作为原样回退保存，不代表本分支包含该卡片。本分支未部署，不替换同事真机开发。

认可门限：minimum_dbfs=-65、margin_db=2.5，settle=.5s、pause hold=.8s。当前输入输出和单人估计 ASR 关联保留；真实采集窗口及头→胸补偿仍在可行性核查，未开启或宣称可用。
