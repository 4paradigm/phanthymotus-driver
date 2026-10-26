# 优必选 U1 Pro 驱动

本驱动为 Agent Core 提供 U1 Pro 的音频、相机、动作和系统行为开关卡片。机器人 SDK 控制服务使用 ROS domain `20`，设备音频/相机 topic 使用 domain `2`，Agent Core 使用 domain `42`。

## 卡片

- `mic`：调用 U1 SDK 的音频 `open_stream`/`stream_state`，优先从 SDK 音频共享内存读取；启动时给 SDK ring 1 秒首帧优先窗口，未收到 SDK 帧才切换到设备 domain `2` 的 `/sys/device/audio_in/raw` topic 备用源。驱动按消息采样率、声道和格式转换为统一的 `audio/pcm-16k`，启动时等待首个 PCM 帧；失败会关闭流并报告错误。
- `speaker`：订阅连接到卡片的 `audio/pcm-16k` 输入，优先通过设备 PulseAudio TCP sink 实时播放；`PULSE_SERVER` 环境变量可覆盖默认地址 `tcp:127.0.0.1:34567`。运行镜像中的 `pulseaudio-utils` 提供 `paplay`，设备侧必须开放对应 PulseAudio sink。队列短时满载时施加背压以保留 PCM；播放进程退出或 writer 持续无进展时停止接收并报告错误，避免无限阻塞 ROS 回调。未安装或无法启动 `paplay` 时才尝试已验证可用的 `EnableAudioOut`/`/sys/device/audio_out/raw` 兼容路径；两条路径都不可用时拒绝启动。音量和静音服务不属于本驱动能力。

部署服务使用 `restart: always`，用于主机或 Docker daemon 重启后自动恢复驱动；运维人员手动停止容器后，Docker 会在 daemon 重启时再次启动它。若需临时停用，应通过部署编排管理该服务，而不是依赖 `docker stop` 保持停机。
- `tts`：通过 U1 SDK `play_text` 进行文本播报，支持打断和 ACP 异步完成回报。真机音量服务不纳入本驱动能力。
- `expression`、`head`：通过 SDK `play_action` 播放声明的动作名称。动作提交后以 ACP 关联播放结果；`interrupt`/`stop` 会调用 SDK 中断服务。两张卡共享同一个动作执行队列。中断请求后保留当前任务，返回 `interrupt_unconfirmed`，等待匹配 UUID 的厂商终态后才释放占用；服务拒绝或异常不会提前报告 cancelled。
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

## head_orientation：水平朝向（首版待真机标定）

与 `head` 播放 A010–A014 预设不同，此卡只使用 domain `2` 的
`/mc/uworld_sound_source`（`uworld_motion_msgs/srv/SoundSource`），
`with_waist=false`，不接入额外控制后端、不控制五个颈部舵机。
`start` 只建立客户端、控制器状态订阅和超时定时器，不发送运动。

| 动作 | 参数与结果 |
|---|---|
| `start` | 无参数；准备服务与只读反馈 |
| `turn` | 必填 `degree`（有限数，暂限 ±20）、`max_vel`、`max_acc`（有限数，`0 < value <= 0.4`）；可选 `timeout`（1–60 秒，默认 15） |
| `center` | 中立朝向与绝对/相对语义尚未确认，明确返回错误，不发送 `degree=0` |
| `stop` | 禁止后续 turn；提交前可取消本地检查；提交后返回 `stop_unconfirmed`，不声称物理停止，不调用 SDK 音频中断 |
| `info` | 当前任务、厂商回执、反馈、反馈年龄、错误、停止能力和角度语义 |

调用示例（由操作者部署后在网页卡片中执行）：

```json
{"action":"start"}
{"action":"turn","degree":5,"max_vel":0.1,"max_acc":0.1,"timeout":15}
{"action":"info"}
{"action":"stop"}
```

上例是**待验收的厂商输入**，不是已验证的“小角度低速”保证：`degree` 的正负、
参考点、绝对/相对语义以及 MotionSetup 速度/加速度单位仍未确认。
±20 与 0.4 仅为首版输入上限，不能由组件配置的 ±28 度推导为实际安全范围。
`toy_zone/servo_zone` 与禁用的 `prismatic_setup` 显式置零，其厂商处理仍待验收。
因此要求显式填写速度与加速度，不隐藏猜测值；厂商拒绝时会显示 status/message。

发送前必须有 1 秒内的 `neck_base_joint` 控制器反馈，并只读确认三类自主行为均关闭。
不会自动切换行为。与 `head`、`expression`、`tts` 原有任务双向互斥（包括尚未确认结束的音频中断）；占用期间
`system_controls` 不能开启自主行为（仍可关闭）。这只协调本驱动，不会锁住其他厂商进程。

状态包括 `checking`、`request_pending`、`accepted_unverified`、`settled_unverified`、
`error`、`outcome_unknown`、`timeout_unconfirmed`、`stop_unconfirmed`。
服务 success/status=0 只表示接受，**不是到位**。订阅控制器 `actual/desired` 中的
`neck_base_joint`，原样输出弧度；直线杆不会被当成角度。
只有 observed desired 相比提交时变化 >0.005 rad，且服务接受后连续新鲜反馈的
actual/desired 误差 <0.02 rad、desired 每次变化 <0.001 rad 持续 1 秒，才释放共享占用，
并标记 `settled_unverified`，`arrival_confirmed` 始终为 false。
这证明观察到的控制器目标收敛，不能证明其对应请求角度、左右方向或中立朝向。
首版不声明 ACP 物理完成契约，不会自动编排下一运动。

超时、传输异常、无法观察目标变化或物理停止未确认时保留互斥占用；`start`、
音频 stop 和迟到回执都不会强行释放。继续收到满足上述条件的反馈后可恢复。
同角度无目标变化可能一直保持未确认，这是有意保守的限制。请先人工确认机器人状态、
排除外部控制后再由操作者决定恢复驱动，不能将重启驱动视为停止机器人。

## sound_direction：只读声源方向

已于 2026-10-26 只读核实：domain `20` 的 `/robo/audio/subscribe/doa_event`
实际为 **`std_msgs/msg/String`**；Adapter 二进制表明其转发 `azimuth/confidence`。
同运行包 `/opt/walker/audio_msgs/share/audio_msgs/idls/audio_msgs/msg/DoaEvent.msg`
注明 `azimuth` 单位度、`[-180,180)`、左负右正、0°通常为正前方，confidence 为 `[0,1]`。
没有收到短时采样事件，未验证实时 JSON 样本、是否相对头部或设备安装坐标。
解析支持 JSON 对象和 SDK `data` 包裹；不直接订阅仓库的 typed DoaEvent，不增加假消息定义。

```json
{"action":"start","stale_after":2,"alpha":0.35}
{"action":"info"}
{"action":"stop"}
```

`stale_after` 范围 0.1–60 秒，默认 2；`alpha` 范围 0.01–1，默认 0.35。
采用环形角度指数平滑，避免 +179/-179 被平均成零；过期或无效事件后重新初始化平滑。
输出 `/{ros_namespace}/u1_pro/sound_direction`，格式 `data/json`，并在 `info` 返回同一结构：

- `direction_degrees`：有效时的平滑角度；无数据、停用、解析错误或过期时为 null。
- `valid`、`stale`、`age_seconds`、`received_at`（接收端 Unix 秒）；不把接收时间冒充设备时间。
- `raw_degrees`、`last_direction_degrees`：保留最近样本供诊断，不表示当前仍有效。
- `confidence`：SDK 提供的原始置信度，缺失为 null，不合成置信度；不对置信度另设猜测门限。
- `unit`、`coordinate_convention`、`error`：明确角度约定、参考系限制和解析错误。

新事件立即发布；本地定时器在过期时发布一次无效状态，stop 也发布无效状态并退订。
不启用厂商唤醒、跟随或自主行为，不向 head_orientation 发送任何命令。
若未收到事件，应检查既有厂商事件生产条件，卡片不会为获取事件而修改行为开关。

## 网页验收（由操作者执行）

1. 自行部署 PR 的 Try it 镜像，确认原 8 张卡片和新增两张卡片均存在。
2. 先验收 sound_direction：start 后静默时为无数据/null；在不同方向发声，核对
   原始角度、左右符号、坐标参考系、真实置信度；静默超过 2 秒确认失效，stop 后不再采集。
   不要将 DOA 数值直接连到 SoundSource.degree。
3. 在确认安全作业条件后，head_orientation start/info 应只显示新鲜反馈，不产生动作。
   三类自主行为需由操作者确认关闭，先校验越界参数拒绝，再单次提交上例并观察实际方向、
   幅度、服务回执、控制器反馈与保持状态，记录绝对/相对语义及速度单位。
4. 转向占用期间尝试 head/expression/tts 和开启自主行为，确认被拒绝；
   测试 stop/超时显示未确认而非完成，观察稳定反馈恢复路径。center 应明确拒绝，不能误报回中。
5. 回中和物理停止需进一步确认厂商接口后再实现；首版只分别验收两张卡片，不做自动联动。

针对性离线测试：`python3 -m unittest discover -s ubtrobot/u1_pro -p 'test_*.py'`；
部署配置检查：`python3 scripts/check_service_yml.py ubtrobot/u1_pro`。
镜像增加本地 `uworld_motion_msgs` 最小接口包，以及标准 `ros-humble-control-msgs` 运行/构建依赖。
