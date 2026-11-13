# Noetix Bumi driver

The bundle exposes Bumi sensor, locomotion, audio, camera, motion-state, and independent sound-direction cards. The direction card is implemented in `direction_card.py`; the other cards remain in `device.py`.

## `mic` and `sound_direction`

`mic` publishes mono PCM `AudioChunk` messages on `/<namespace>/mic/audio`.
`sound_direction` is the only direction card. Its canvas output is
`std_msgs/msg/String` on `/<namespace>/sound_direction`, declared as
`sensor/sound-direction`. Like `sensor/pose2d`, it has a specific canvas type
with the standard String transport; it is not a generic `data/json` port.
**The `/<namespace>/sound_direction` topic name is the contract ASR uses to
find its direction input — renaming it disconnects wake direction.** The
monitor shows the observation as plain data. The angle uses the robot frame:
front 0°, **left positive** (counter-clockwise, REP-103 — the same sign as
the locomotion `vyaw` command), right negative, rear ±180°. A source at
front-left reads about +45 and passes straight to a `vyaw` turn with no
negation. The observation reports `state: "fresh"` and the exact `angle_deg`
for ASR and display, with
`angle_convention: "robot_front_0_left_positive_ccw"`, `audio_window_start_us`,
`audio_window_end_us`, and `timestamp_ms`.
After activity stops it publishes `state: "no_event"`; if updates disappear,
the driver publishes `state: "stale"` after two seconds. Latest-mode viewers
retain the previous reading with that explicit state. No-event/stale messages
are not valid ASR observations. Driver repeats terminal states every 500 ms,
including no-event before first activity, so late readers or a lost
best-effort packet do not leave an old reading marked fresh.

The worker's `audio_msgs/msg/SoundDirection` stream is internal, on
`/<namespace>/_internal/sound_direction`. Only the public observation topic
is declared by the card, so monitoring shows one `sound_direction` window.

The two capture workers use the SDK frame timestamp as their common capture
clock only when it has a Unix-microsecond shape. `mic` stamps each chunk with
the first frame's time; the direction window ends after the final frame's last
sample. Invalid timestamps leave the audio stamp unset and direction window
at zero, so ASR cannot mistake publication time for capture time. The SDK
clock's meaning still requires verification on Bumi. Simultaneous readers were
verified on Bumi with commit cc2c1e2; this capture topology is unchanged.

Lifecycle follows the same contract as the other sensor cards: the driver
spawns the direction worker at boot and tears it down on driver shutdown,
a config save, or a failed worker start. The canvas `start`/`stop` actions
are status replies — they never spawn or kill the worker — so stopping
intelligent control leaves both direction streams running. No special canvas
start call is required to view the sensor.

A worker that dies mid-session is **not** revived by `start`/`stop`/`info`;
restart the driver to bring the stream back. A config save restarts the
worker only while it is running; on a dead worker it just writes the
settings.

Set the one shared parameter from the direction card's left-list gear.
Saving it restarts a running direction worker and waits for it to become ready.
If stopping the worker times out, the previous settings file is restored and
no replacement worker is spawned. If settings recovery also fails, the error
reports the file and running parameters separately.

| Parameter | Default | Effect |
| --- | --- | --- |
| `sensitivity` | `normal` | How loud a sound must be to count as someone speaking |

Environment presets:

- **普通环境** (`normal`, default) — 日常室内使用，优先识别正常音量说话。
- **嘈杂环境** (`noisy`) — 风扇或环境噪声较大时使用，需要更靠近或更大声说话。

The publish throttle is fixed at 100 ms (about 10 updates per second) and
is no longer operator-adjustable. Gate thresholds per preset live in the
engineer appendix below.

### 旧 settings.json 迁移

The legacy 5-key file at `/opt/phanthy-motus/data/bumi/sound_direction_settings.json`
(`onset_level` / `onset_ratio` / `burst_level` / `burst_ratio` /
`update_interval_ms`) is still read after upgrade: it maps **in memory** to
an environment preset (`onset_ratio` <2.25→normal, otherwise noisy) and is never rewritten — the file keeps its old shape until
the operator saves the config once. `update_interval_ms` is dropped; the
throttle stays at 100 ms regardless of any hand-tuned 50/300 value.

旧三档值 `high` / `normal` 映射为 `normal`，`low` 映射为 `noisy`，
读取设置文件和接收配置请求时都会兼容转换。读取不回写，保存会写入新值。
升级后请在齿轮面板重新选择并保存一次，将画布保留的旧值更新为新值。

Rolling back to an older image: before the first config save the old image
reads the legacy file as before; after a save the old key-set check falls
back to defaults silently. Legacy key names sent to `config` are rejected
(`invalid_parameters`). Keys saved in a canvas config are dropped by the
platform, not migrated.

### 工程师附录：门限映射表

```
SENSITIVITY_GATE_PARAMS = {
    "normal": {"onset_level": 10.0, "onset_ratio": 2.0, "burst_level": 15.0, "burst_ratio": 4.0},
    "noisy":  {"onset_level": 20.0, "onset_ratio": 3.0, "burst_level": 30.0, "burst_ratio": 6.0},
}
UPDATE_INTERVAL_MS = 100
```

Preset meaning is defined by this table, not by the panel wording. The
noisy preset doubles both absolute level floors and raises both background
ratios by 50% compared with normal. Sensitivity
only adjusts how easily sounds trigger; it does **not** remove fan-noise
false angles (known, unfixed).

The card first loads a complete, valid per-robot calibration from
`/opt/phanthy-motus/data/bumi/sound_direction_calibration.json`, then tries the
packaged `resource/sound_direction_calibration.json`. Each file must contain
`front` and `right`, each with three finite values of magnitude at most 24.
Each vector must also have a norm of at least 0.5 samples, matching the
estimator's minimum meaningful delay signature; a near-zero override falls
back to the packaged calibration. The existing condition-number limit still
rejects nearly collinear vectors.
The two files are never mixed. The packaged file was copied unchanged from
the calibrated Bumi host `ubuntu` on 2026-10-14, at the per-robot path above.
Its SHA-256 is `89813968f99881310d5f49ca748d48303dfadfd27e550893d27f1023e5fc660a`.
The direction card is registered by default; without
a valid file, `start` and `info` report it unavailable instead of starting a
silent worker. On 2026-10-14, the operator confirmed that this calibration
can be shared between Bumi robots. This portability confirmation is supplied
by the operator; the repository's tests validate file geometry and loading,
not physical direction accuracy.

For existing canvas projects, remove the old `sound_direction_control` card,
remove/re-add `sound_direction` and ASR, and reconnect the dedicated direction output
to the ASR direction input (and mic to the audio input). This replaces saved
typed ROS or `/events` topics; update Driver and Perception together for the standard
sensor transport. The canvas type remains `sensor/sound-direction`.
Update Core once for this migration to remove its old custom ROS decoder.
Calibration actions have been removed; update the packaged file through a reviewed change, or use
the per-robot override as an emergency measure. The previous
`/<namespace>/mic/sound_direction` stream is no longer produced.

Run local tests from the repository root with
`python -m pytest noetix/bumi/tests -q`.
The driver image builds its bundled `audio_msgs` definitions, including
`SoundDirection`, over a pinned ROS base image. It does not require a new
`ros-base` release from the main repository. The canvas/ASR path does not require
`SoundDirection` in Agent Core or Perception. Legacy typed consumers still
need the matching message definition.

The published-image [size measurement](review-evidence/image-size-2026-10-14.md)
records a 2,088,960-byte increase in summed history-layer size over the earlier
release without the local interface overlay; the ROS overlay and import-check layers account for
1,146,880 bytes. Both measured images use the same ordered base layers, matching
this robot's cached `ros-base:latest` and the pinned digest. The report separates
Docker size accounting from actual disk consumption and includes raw evidence.

The internal `/<namespace>/_internal/sound_direction` stream uses the bundled
`audio_msgs/msg/SoundDirection`. The Driver itself
builds that definition; the standard sensor observation is a separate topic
so there are never two ROS types on one topic.

For the migration release, record the immutable image digests for Bumi,
Perception, and the compatible Core.
Confirm a direction event reaches the ASR input and the canvas viewer on the
target robot, including no-event/stale states. Keep the release check open
until those published-image and robot checks pass.

## App 图传与 Phanthy Camera Card

Bumi 的官方 App 图传服务 `noetix-video-capture.service` 会占用 RealSense。
因此，官方 App 图传与本 Driver 的 `camera` / `depth` Card 不能同时使用。

要让 Phanthy Dashboard 直接调用相机，设定
`plugins.camera.disable_vendor_capture_service: true`（提供的 Bumi 配置已
开启）。Driver 在初始化相机前，会利用部署配置中的宿主机 PID namespace 和
特权能力，在机器人宿主机执行：

```bash
systemctl disable --now noetix-video-capture.service
```

该设置会跨机器人重启生效：Phanthy 的 `camera` / `depth` Card 优先使用
RealSense，但官方 App 图传不可用。操作失败只会记录日志，不会阻止其他 Bumi
Card 启动。

要恢复官方 App 图传，请在机器人宿主机显式执行：

```bash
sudo systemctl enable noetix-video-capture.service
sudo systemctl start noetix-video-capture.service
```

## `camera` / `depth` 声明自己的镜头参数（`motus.camera/1`）

两张卡片的 `info()` 各带一份 `camera_info`。消费者按**自己绑的那个话题**去查，
不是按位置、也不是按上游卡片名。`depth` 在发布前通过
`rs.align(rs.stream.color)` 重投影到 RGB 像素网格，因此它与 `camera`
声明同一个 `id` 和同一组 RGB 内参。这是 VOP 的 RGB 检测框能直接查
深度像素的前提；仅仅让两路都是 640×480 并不够。

为什么需要它：深度图里没有任何几何信息 —— 640×480 个数字，每个是一段距离，没有
一处说明镜头有多宽。而据此做的决策是米制的（"我 0.4 m 宽的肩膀过不过得去"），所以
某处必须把像素列换算成横向偏移，那一步用的就是水平半视场角。这个数填错**不会报错**：
它让机器人拒绝过门，而深度图同时报告前方通畅 —— r1_sz 上这个症状被当成跟踪问题查过
两次。

**参数优先取运行时内参。** 相机子进程在 `pipeline.start()` 之后从 RealSense 拿到
真实的 `fx/ppx/畸变系数`，打印一行 `[camera_subprocess] INTRINSICS {...}`，父进程
那个本来就有的 stdout 转发线程认这个前缀并存下来（这一行也照常留在 `docker logs`
里）。声明里于是带 `K`，`source` 是 `derived-from-K`。

**手册数只是兜底，而且彩色那路不能直接用手册的数。** D435i 的 RGB 传感器是 16:9，
驱动开的是 640×480（4:3）—— 这一档保垂直裁水平，所以真实水平视场比手册上的 69.4°
窄三成左右。`camera_specs.py` 因此按宽高比换算过一次，并把 `source` 写成 `manual`
而不是 `vendor-spec`：那是一个在 Intel 写下的数之外多走了一步算术的值。对齐后的
深度输出也使用这个 RGB 兜底值，而不是原始深度镜头的 87°。

相机没起来时走兜底 —— 声明必须在不串流时也答得出来，这是消费者能在 `start` 时就
降级（而不是一帧一帧地发现问题）的前提。

镜头换了必须重来一遍：`phanthymotus/actucore/tools/measure_fov.py`。

## `vision_capture` card

Persistent RGB photo/video capture, with the card/action names and file layout
from Q5 PR #220. This replaces `state_record`; existing workflows must select
the new card. State scopes, JSON snapshots, labels and interval logging are no
longer supported. Historical files under `/opt/phanthy-motus/data/bumi/state-records`
are left untouched.

- `start`: check whether the camera process is ready (does not take a photo).
- `capture_photo`: save one current RGB JPEG.
- `record_video`: asynchronously save an RGB MP4/H.264 video; `duration_s` is
  an integer from 1 to 30, default 5 seconds.
- `info`: return storage directories, camera/frame freshness and active recording.
- `stop`: cancel recording and remove the incomplete video. This is not
  “finish early and save”. It does not stop the existing camera sensor card.

Bumi reuses its existing 640x480 JPEG topic `/<namespace>/camera/color`.
A recent cached frame is used for a photo; if none is available or it is older
than 3 seconds, capture waits for a fresh frame (up to 5 seconds per wait).
Recording requests a strictly later frame sequence for each encoded frame.
No second RealSense pipeline, depth capture, audio or state JSON is added.

Default persistent output:

```text
/opt/phanthy-motus/data/vision_capture/
  photos/IMG_20260903_143025_123456.jpg
  videos/video_20260903_143025_123456.mp4
```

Names use local date, time and microseconds, with no label or counter.
`deploy/service.yml` already mounts `/opt/phanthy-motus/data` from the robot
host at the same path. Download files from that host and open them in a normal
image viewer/video player. The MCP card returns paths, not a browser preview
or a download server.

Photos return `ok`, `media_type`, `file_path`, `captured_at`, `frame_age_s`.
Photo failure returns `CAPTURE_FAILED` without creating a JSON sidecar.
Video requests return `state=queued` and an `action_id`; completion/failure/
cancellation is sent to `AGENT_CORE_URL/api/acp/complete` (default Agent Core
URL: `https://localhost:15678`), matching Q5's ACP contract. Only one video
recording may run at a time. A callback delivery failure is logged; it does not
delete a successfully saved video.

Configuration is under `plugins.vision_capture`: `output_dir`, `fps`
(default/max 15), and `max_duration_s` (default/max 30). Rebuild the driver
image to install the new FFmpeg dependency. All runtime implementation remains
in `device.py`; no extra runtime Python file is required.

## New cards

### `motion_state`

Passive, read-only whole-body motion telemetry from `HighController`. The card has no action parameters or execute button. Its single JSON output combines the former summary and joint views:

- current activity, workmode, protection flag, body orientation, angular velocity, linear acceleration and whole-body joint activity statistics;
- active motor faults whose codes are documented by Noetix;
- position, velocity, torque, temperature and raw error value for all 21 joints.
- ROS2 output: `/<namespace>/motion/state`, JSON.

The default polling and topic publication rate is 2 Hz (`poll_interval_s: 0.5`).
Motion activity uses a configurable joint-speed threshold, defaulting to
`0.15 rad/s`. Only motor error codes explicitly documented by Noetix are
classified as faults. Undocumented non-zero raw values are not shown in the
documented fault list and remain available only in each joint's raw `error`
field for device-side verification.

Every published state identifies `Noetix HighController/CycloneDDS` as its source and includes a freshness flag. It deliberately excludes battery data, which belongs to the existing `battery` card. The SDK does not expose world-frame position or translational velocity, so the card reports only documented IMU and joint measurements and does not invent odometry.

The same card also publishes the IMU's angular velocity as **`motus.odom/1`** on
`/<namespace>/state/odom` (format `state/odom`), at **10 Hz** — its own loop, not
the 2 Hz joint poll. 2 Hz would land exactly on navi's 500 ms observation window,
so half the samples would read as stale and a consumer would alternate between
using odometry and not. `info()` carries the `odom_interface` declaration; the
shape lives in `odom_spec.py`, outside `device.py`, so it can be asserted without
rclpy (`tests/test_bumi_odom.py`).

**`vx`, `vy` and `vz` are `null` in every sample, and are not in `provides`.**
There is no body-frame speed anywhere in `HighController`. Reporting `0.0` would
be a different claim — that the robot measured itself standing still — and it is
the claim that makes a consumer's stuck detector ("commanded 0.3 m/s, measured
nothing, therefore we have hit something") fire on every step. So:

- anything that needs a measured translational speed **cannot work on Bumi**,
  and that is the robot's limitation, not a fault to go looking for;
- a navigation policy falls back to predicting its own motion from the commands
  it sent, with the process noise widened accordingly;
- `wz` — the axis such a policy leans on hardest, because self-rotation is the
  main reason a target moves across the image — **is** measured.

`pose` is `null` and `pose_drift` is `none` for the same reason: there is no
position to report, which is a different statement from a position that drifts.

### `loco_servo` — 底盘的流式速度控制

订阅一路 `motus.control/1` 的 **twist**（6 维 `[vx, vy, vz, wx, wy, wz]`，机体系，
m/s 与 rad/s）驱动底盘。给 actucore 的 `navi` 这类策略用 —— 它们一秒发十个速度，
并且不等每一个的回答，而 `loco` 是调用形态：一次 `tools/call` 一个动作，对人和 LLM
是对的，对策略是错的。

`vz` / `wx` / `wy` 在 descriptor 里钉成 `lower == upper == 0`。一个以为自己在指挥
垂直运动的策略会被**响亮拒掉**，而不是三分之一的输出凭空消失。

**和 R1 的同名卡片有两处结构性不同**，都在 `loco_servo.py` 的文件注释里：

1. Bumi 的 `publish_cmd` 收的是归一化的 `[-1, 1]`，不是 m/s，所以这张卡片自己做
   换算 —— 见下面的标定；
2. Bumi 的指令**不驻留**（R1 的 `Move(..., True)` 会一直走到 `StopMove`），所以
   卡片带一个 50 Hz 重发线程，而"保持"意味着**持续发零**，不是停止发送。

**仲裁**：`loco`、`stand_up_lie_prone`、`semantic_action`、`action_recording` 里
任何一个动作都会先把这张卡片暂停 —— 后三个还会把 `workmode` 从正在跑的流底下换掉。
反过来，`loco` 有动作在飞时这张卡片拒绝启动。人和 LLM 的显式指令优先于正在跑的
策略，这个方向不能反。

姿态门槛是 `workmode == 2`（walking），**按每条指令检查**而不是在 `start` 时：
启动是接线事件，机器人那一刻常常是趴着的，拒绝启动会连累整张画布。

`dry_run` / `rotate_only` / `require_standing` 三个开关在卡片上就能改（`configSchema`），
**并且立即生效**，不用重新部署。

#### 标定 `loco_servo`

`config.yaml` 里 `loco_servo` 下那七个数描述的是**这台底盘的速度空间**。
2026-10-09 已完成真机标定，当前为 `calibration_source: measured`；首次接入完整
navi 流水线时仍保留 `dry_run: true`，用于核对视觉控制方向。标定方法如下：

| 量什么 | 怎么量 | 填到哪 |
|---|---|---|
| 满舵前进速度 | 关掉 `dry_run`，用 `loco` 的 `move` 发 `vx=1.0, duration=3`，卷尺量走了多远，除以实际走动秒数 | `full_scale_vx_mps` |
| 满舵横移速度 | 同上，`vy=1.0` | `full_scale_vy_mps` |
| 满舵转向角速度 | `vyaw=1.0, duration=4`，数转过几圈，`圈数 × 2π ÷ 秒数` | `full_scale_wz_rads` |
| 前进死区 | 从 `vx=0.05` 起每次加 0.05，第一个真的让机器人挪动的值 | `min_vx_mps`（乘上面量到的满舵值换算成 m/s） |
| 横移死区 / 偏航死区 | 同上，`vy` / `vyaw` | `min_vy_mps` / `min_wz_rads` |
| 行进中的偏航死区 | 一边 `vx` 走一边加小 `vyaw`，第一个看得出转向的值。R1 上这个数比站立时小 **20 倍** | `min_wz_moving_rads` |
| 机身宽度 | 卷尺量手臂自然下垂时的最大宽度，取一半 | `loco_servo.py` 的 `FOOTPRINT_HALF_WIDTH`，并把 `footprint.source` 改成 `measured` |

本机实测值：

| 参数 | 实测值 | 原始结果 |
|---|---:|---|
| `full_scale_vx_mps` | `0.55` | `vx=1.0` 两次均约 165 cm / 3 s |
| `full_scale_vy_mps` | `0.20` | 可靠三次 58、68、52.5 cm / 3 s，均值约 0.20 m/s |
| `full_scale_wz_rads` | `0.83` | 正向 0.876、反向 0.831 rad/s，取较慢方向 |
| `min_vx_mps` | `0.25` | 归一化 0.40 无位移、0.45 有位移；0.45 × 0.55 |
| `min_vy_mps` | `0.12` | 归一化 0.55 静止、0.60 有效；0.60 × 0.20 |
| `min_wz_rads` | `0.33` | 静置后归一化 0.35 无效、±0.40 有效；0.40 × 0.83 |
| `min_wz_moving_rads` | `0.25` | 行进时 0.25 接近基线、±0.30 明确转向；0.30 × 0.83 |

**这些数错了不会报错**：满舵值填得偏小会使换算后的归一化命令偏大，机器人会
走得比策略要求的快；死区填零则策略的小幅修正会被接收、计数、然后什么都不发生。

`footprint` 的出错方向是单边的（以为自己更宽只是多减速，以为更窄就是把肩膀送进
门框），所以默认值刻意偏宽；死区不是，所以它宁可声明成估计值也不写零。

## `walking_mode` 行走模式切换（待现场验收）

本分支提供独立 actuator 卡片 `walking_mode`。参数区只有必填的
`walking_model` 下拉框，选项为 `lower_body`（仅腿部行走）与
`whole_body`（全身行走），不再显示 `action` 下拉框。无 `action` 字段的
MCP 请求由 bundle 按工具名分发，例如：

```json
{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"walking_mode","arguments":{"walking_model":"lower_body"}}}
```

**已实现 App TCP 后端，默认关闭，真实双向切换仍待现场验收。**
`config.yaml` 的 `plugins.walking_mode.enabled` 为 `false`，需要同时启用 `loco`。
构造、启动、查询和停止均不建立 App 连接或发送 SDK 命令；执行时才连接。
当前 SDK 没有已核验的步态选择入口，因此本卡复用官方 App 的 TCP 命令，
SDK 只用于读取现有控制器的反馈，不实例化另一个 `HighController`。

执行时先预留 `loco` 的底盘，已有移动、转向或身体动作则拒绝切换；
处于使能、准备、保护等非行走模式时同样拒绝，不会自动使能或起身。
App 与 SDK 必须先报告一致的 2/3 模式；已处于目标模式则直接返回，不重发命令。
实际切换前暂停 `loco_servo`，检查全部 21 个关节速度均低于 `0.15 rad/s`，
并再次检查模式未变化。仅发送一次 `walk` 或 `run`，随后连续两次确认 App 与
SDK 的反馈均为目标代码才返回 `state=completed`、`mode_verified=true`。
这表示模式切换获确认，不代表真实行走行为已验收。
超时、保护或断连不会自动重试、回退或恢复流式移动；发送中断时会保留
`command_may_have_been_sent`，避免误报指令肯定未到达。

默认在 UDP 9090 被动接收 `discovery_source_ip` 指定的运动板广播，
取其 `perception_ip` 连接 TCP 8888；不发送扫描报文，也不把实验机器 IP 写进代码。
UDP 不可用时可在部署配置中指定 `app_host`。部署前须验证驱动容器能访问该地址；
本次 Windows 端查询已成功，随后机器人断连，容器内连通性尚未确认。
现有 `loco` / `loco_servo` 的移动门槛仍为状态 2；状态 3 下通过 SDK 发速度的行为
尚未验收，本次没有放宽这些运动限制。

2026-11-11 核对了本仓库附带的 SDK 3.1.0 头文件、Python 绑定和示例，
并对照了厂家的公开文档与当前 SDK 源码：

- [厂家 HighController 文档](https://web.noetixrobotics.com/docs/#3-highcontroller常用接口及参数表)
  仅声明 `WALK` 为走路模式，`RUN` 为当前不可用的预留命令，`index` 为示教索引。
- [厂家 HighController 头文件](https://github.com/Noetix-Robotics/noetix_sdk_bumi/blob/main/include/highcontroller.h)
  公开 `publish_cmd` 和状态订阅，没有分别设置两种行走模式的接口。
- [厂家 ControlCmd 定义](https://github.com/Noetix-Robotics/noetix_sdk_bumi/blob/main/include/common.h)
  与本仓库的枚举一样，未列出 lower-body / whole-body 选择项。

这些资料不能确定两种模式对应的 DDS 调用，也不能证明目标固件没有该能力。
本卡没有向 SDK 发送 `WALK` / `RUN`，也没有用 `WALK` 的 `index=0/1`
选择步态。2026-11-13 的分支基线为官方 driver 主分支
`18211f9902549ce931867a3622cbb96d4c79183b`。

2026-11-11 用户提供了 NOETIX Run 连接实机后的控制页照片，明确显示独立的
“行走”和“全身行走”入口。因此，App 入口存在已获确认，当前缺口是它们与 SDK
命令的准确对应关系，不能据公开文档未列出就判断机器人不支持。

进一步只读检查了[厂家下载页](https://app-api.noetixrobotics.com/)提供的
`noetix-installer-3.1.2.apk`（SHA-256：
`dc3f035c6bf7a2a99d73dd3fcb41be461673f890f24d256352e9b9440c3eee17`）。
其中有 `walk_mode_*`、`bumi_run_*` 图标资源，以及 `walkMode`、`runMode`、
`RobotCommandSender` 等内部名称。继续解析包内 Flutter 3.6.1 的 AOT 快照后，
已核对两种命令对象的初始化和序列化逻辑：

| 选择需求 | App Wi-Fi JSON | App 蓝牙 JSON |
| --- | --- | --- |
| `lower_body`，App 的“行走” | `{"cmd":"walk"}` | `{"cmd":18,"data":[1]}` |
| `whole_body`，Bumi 的“全身行走”使用内部 run 分支 | `{"cmd":"run"}` | `{"cmd":18,"data":[2]}` |

定位依据为 `WifiCommand.walkMode`（`0x604138`）、`WifiCommand.runMode`
（`0x626fc8`）、`BleCommand.walkMode`（`0x6040d0`）、`BleCommand.runMode`
（`0x626f60`），以及两者的 `toJson` 和 `ControlRobotController.updateModeList`
（`0x626a50`）；显示名称由 `ModeType.title`（`0x61d52c`）区分机型。
这些是上述特定安装包的代码地址，不是稳定协议版本号。

关键限制是：同一 App 的 `BumiRobotStatusType` 将 `walk` 的状态值定义为 2，
将 `run` 的状态值定义为 3，并在后者附带“DDS无法触发”的说明。
因此，App 的 `cmd="run"` 不能直接视为 DDS SDK 的 `ControlCmd.RUN`（枚举值 4）。
这些静态结果证明 App 的编码方法，不能把它们直接塞入 DDS。

继续检查同一 APK 的 `classes3.dex`，`WifiService.connect` 使用设备发现消息中的
`perception_ip:8888`；`MessageContent.parse` 与 `ProtocolUtils.buildPacket`
将 UTF-8 JSON 封为：8 字节 magic `be ba fe ca ef be ad de`、4 字节正文长度、
4 字节消息类型（JSON 为 0）、正文、4 字节重复正文长度；整数均为小端。
`get_status` 是查询命令，响应为 `{"cmd":"status","data":{"status":2}}`。
上述方法在该 DEX 中的 code offset 分别为 `0x345974`、`0x348228`、`0x346924`；
这些偏移仅用于定位该安装包。

现场只读监听收到运动板 `192.168.55.102` 的 UDP 广播，其中包含网络地址。
在 Windows 端使用上述封包发送一次 `get_status`，已收到合法的
`status=2` 响应，连接随后关闭；没有发送 `walk` 或 `run`。这验证了查询协议，
尚未验证命令会切换当前实机或多客户端之间的控制权处理。

2026-11-11 用户提供了在 App 中反复手动切换时的实机 `motion_state` 日志，
并确认期间一直在主动往返点击。20:50:22—20:50:33 共 23 条记录，
状态码 2 出现 13 次，状态码 3 出现 10 次，共有 10 次双向变化，
最后回到 2；所有记录均为 `fresh=true`、`protection=false`。
例如 20:50:22 为 2、20:50:23 进入 3、20:50:24 返回 2。
结合 App 的状态定义，该实机的“行走”与“全身行走”可分别通过反馈码 2、3
区分。驱动保留 2 的既有名称 `walking`，为 3 补上 `whole_body_walking`，
避免显示 `unknown`。这次观察没有验证 SDK 切换命令，也不能把反馈码 3
当作 `publish_cmd` 的 action；SDK 命令枚举 3 实际为 `CHEER`。

厂家 SDK 当前提交 `20750fda734da0ef4c2fe28226b0d506a1f32273` 的 x86 静态库中，
`publish_cmd` 直接把枚举写入 DDS action，只在保存与播放示教时写入 index；
因此给 `WALK` 设置不同的 index 无法用该接口选步态。本分支没有向实机发送
切换命令；上述切换由用户在 App 中手动完成。

下一步待连接恢复后，先只读验证容器内访问及状态一致性，再由现场操作者确认
机器人静止站立、暂停其他控制来源后，验收 `2 → 3 → 2` 的命令与反馈。
同时核对固件/App 版本、并发客户端控制权及断连行为，再决定是否默认启用。
目前不能将本卡声明为已通过实机切换测试或可发布的切换能力。

离线验证（仓库根目录）：

```powershell
python -X utf8 -m pytest tests/test_bumi_walking_mode.py tests/test_bumi_walking_mode_client.py tests/test_bumi_motion_state.py tests/test_bumi_bundle.py tests/test_bumi_loco_servo.py -q
```

上述测试检查下拉框 schema、无 action 的 bundle 分发、启用依赖、底盘预留、
静止检查、双路反馈、断连/超时/保护处理、TCP 分帧，以及真实 `motion_state`
读取逻辑对 2/3 模式、保护模式和未知代码的区分；不证明真实模式切换。

## Direct action cards

The former `switch_mode` tool is split into three user-facing cards. Internal
`enable`, `ready` and `walk` transitions are completed automatically and are no
longer exposed as user choices:

- `stand_up_lie_prone`: `stand_up` from a face-up lying pose, or `lie_prone`
  from a stable standing pose into the prone storage posture;
- `semantic_action`: wave, handshake, cheer, three dances and wipe-tears;
- `action_recording`: start recording, finish and save a recording, or play a
  saved recording by `recording_id`.

Every result reports the automatically executed preparation steps, the observed
workmode, whether the requested action start was confirmed, plain-language
safety requirements and the fact that the SDK cannot verify the robot's real
physical pose. An observed target action mode returns `running`, not
`completed`, because mode feedback does not prove the physical motion has
finished. If any preparation or action enters protection mode, the card stops
the sequence and tells the user to restart, place Bumi face-up on a flat,
non-slip surface with a clear 3 m × 3 m area, and then use `stand_up`.

`semantic_action.reset` exits or interrupts an active semantic action and
returns the robot to workmode 2 (`walking`). It is accepted only from semantic
action workmodes, or treated as a no-op when already walking. It never promotes
disabled, enabled or ready modes into walking because the SDK cannot verify the
physical pose.

`stand_up` is accepted only from disabled or enabled mode and its description
requires the operator to place the robot face-up before calling it. `lie_prone` requires
the operator to confirm stable standing through the card instructions and is
accepted only from walking mode. These
guards prevent a standing robot from receiving the get-up trajectory.

`play_recording` remains `running` after play-teach mode is observed. The SDK
does not document a physical playback-completion event, so the driver does not
send `WALK` on a guessed timeout. After visible completion, or to interrupt
playback, use `action_recording.stop_playback`; it is accepted only from
play-teach mode and confirms the return to walking.

`finish_and_save_recording` maps to the supported `SAVETEACH` command. The
vendor-deprecated `ENDTEACH` command and unavailable `RUN` command remain
unexposed.

## `speaker`

Plays the PCM audio of its connected input topic (`audio_msgs/AudioChunk`, mono
16 kHz S16LE) on the robot speaker. Actions: `start` / `stop` / `info` /
`get_volume` / `set_volume`.

`start` **is** the playback action — the canvas starts a card by sending `start`
with the resolved `input_topic`, so there is no separate `play`. A card whose
`start` only answers `{"state": "ready"}` shows as running while the driver
holds no subscription at all, which is silence that looks like success.

The vendor voice agent (`MediaController.wakeup()` / `sleep()` / `restart()`) is
deliberately **not** exposed. Waking it would hand the robot's microphone to the
vendor's own model and let it talk over this stack through the same speaker.
Two behaviours verified on hardware and worth not re-deriving:

- **`work_status=SLEEPED` does not block playback.** A full TTS utterance played
  while the agent sat at `SLEEPED/CMD_SLEEPED`. Only `ERROR_SLEEPED` means the
  audio agent is really down; `start` then calls the documented `restart()`
  recovery by itself and reports it.
- **`get_volume()` is not trustworthy.** A freshly created MediaController reads
  `0` for 12 s and longer while audio plays normally, and each client appears to
  read back its own last `set_volume`. Treat a `0` as "unknown", never as the
  reason for silence — `info` reports it for reference only.

Likewise `frames_submitted` in `info` counts frames handed to the SDK, not
frames heard: `publish_external_audio_playback_stream` is fire-and-forget, so a
rising count proves the subscription works and nothing more.

Useful observations while the driver is running:

```bash
ros2 topic echo /<robot_namespace>/motion/state
ros2 topic hz /<robot_namespace>/motion/state
docker logs -f embodied-noetix-bumi
```

## Read-only RGB-D visual odometry card (`rgbd_visual_odometry`)

Single container: the estimator runs inside this bundle, not in a sidecar.
`main.py` starts a `VisualOdometryPlugin` when `plugins.visual_odometry.enabled`
is true (**enabled by default**); the plugin
supervises one `rtabmap_odom/rgbd_odometry` child and an optional IMU attitude filter, and
publishes a read-only `motus.odom/1` stream on
`/<namespace>/bumi_visual_odometry/state/odom`. No second image, no second container, no USB
or privileged change.

Camera/depth output formats are unchanged. A single pyrealsense2 capture process
owns the device (`device.py::_camera_subprocess`). When the card is enabled,
that process also publishes an optional, read-only raw branch
(`visual_inputs.py`) on `/<namespace>/odometry_input/`, sampled at
`plugins.visual_odometry.max_rate_hz` (default 10 Hz):

| Topic | Type | Meaning |
| --- | --- | --- |
| `rgb` | `sensor_msgs/Image`, bgr8 | source colour frame |
| `depth` | `sensor_msgs/Image`, 32FC1 metres | depth in the colour grid, scaled by the SDK `depth_scale` |
| `info` | `sensor_msgs/CameraInfo` | live K/D/R/P; zero coefficients are published as `plumb_bob` |
| `camera_meta` | `std_msgs/String` | JSON provenance: epoch, per-stream source stamp/frame/clock domain, depth scale, receive time |

The raw branch never opens a second pipeline, never touches the USB device and
never publishes a chassis command. Construction and every publish are wrapped,
so a fault here only prints a log line; JPEG, depth, intrinsics, photo and video
capture continue as before. `main.py` sets `BUMI_VISUAL_ODOMETRY=1` only when the
card is enabled — a disabled card costs no extra DDS traffic and starts no
estimator.

The compact status includes `turn_angle_deg`: body yaw relative to the first IMU
sample of the current segment, wrapped to -180..180 degrees.

`plugins.visual_odometry.imu_assisted` defaults to true with
`imu_source: controller`: the visual card polls the HighController body IMU
directly and republishes changed samples as `sensor_msgs/Imu`. RTAB-Map uses the
orientation as a rotation prior; translation is still estimated from RGB-D, with
no inertial position integration or robot_localization/EKF. `imu_source: state`
falls back to the StatePlugin topic. Set `imu_assisted: false` to run visual-only.
The output is not wired
into `loco_target`/`loco_servo`. The full `motus.odom/1` sample is also declared
as a second `state/odom` canvas output for future consumers, but no loco card is
wired to it. The shipped config applies the standard level-forward mount so `position` uses robot body axes (x forward, y left, z up;
reported as `pose_reference: body_axes`, a convention rather than a measured
extrinsic). The card defaults to `initializing` and
publishes no pose until the estimator is tracking; a fault ends that segment and recovery
starts a fresh one automatically. Stopping the card reaps its estimator and private topics and disables the
shared raw RGB-D branch through `/NS/odometry_input/raw_enabled`; the camera,
media cards and motion cards keep running.

The output contract, quality states, assumed body-axis calibration, validation
status and the ROS probe live in [VISUAL_ODOMETRY.md](VISUAL_ODOMETRY.md).
