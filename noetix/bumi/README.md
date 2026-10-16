# Noetix Bumi driver

The bundle exposes the original Bumi sensor, locomotion, audio and camera cards plus one higher-level motion-state card backed by documented Noetix SDK APIs. All card implementations are kept in `device.py`.

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
不是按位置、也不是按上游卡片名。

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
而不是 `vendor-spec`：那是一个在 Intel 写下的数之外多走了一步算术的值。深度那路不需要
这个修正，它原生就是 4:3，87° 对应的就是我们发出去的那张图。

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

`config.yaml` 里 `loco_servo` 下那七个数描述的是**这台底盘的速度空间**，
**一个都没有在 Bumi 上量过**，所以 `calibration_source: estimate`、`dry_run: true`
是出厂状态。navi 会把这一栏显示在它的 `degraded` 里。量一次大约二十分钟：

| 量什么 | 怎么量 | 填到哪 |
|---|---|---|
| 满舵前进速度 | 关掉 `dry_run`，用 `loco` 的 `move` 发 `vx=1.0, duration=3`，卷尺量走了多远，除以实际走动秒数 | `full_scale_vx_mps` |
| 满舵横移速度 | 同上，`vy=1.0` | `full_scale_vy_mps` |
| 满舵转向角速度 | `vyaw=1.0, duration=4`，数转过几圈，`圈数 × 2π ÷ 秒数` | `full_scale_wz_rads` |
| 前进死区 | 从 `vx=0.05` 起每次加 0.05，第一个真的让机器人挪动的值 | `min_vx_mps`（乘上面量到的满舵值换算成 m/s） |
| 横移死区 / 偏航死区 | 同上，`vy` / `vyaw` | `min_vy_mps` / `min_wz_rads` |
| 行进中的偏航死区 | 一边 `vx` 走一边加小 `vyaw`，第一个看得出转向的值。R1 上这个数比站立时小 **20 倍** | `min_wz_moving_rads` |
| 机身宽度 | 卷尺量手臂自然下垂时的最大宽度，取一半 | `loco_servo.py` 的 `FOOTPRINT_HALF_WIDTH`，并把 `footprint.source` 改成 `measured` |

量完把 `calibration_source` 改成 `measured`。**这些数错了不会报错**：满舵值偏小
机器人就走得比策略要的慢，死区填零则策略的小幅修正会被接收、计数、然后什么都不发生。

`footprint` 的出错方向是单边的（以为自己更宽只是多减速，以为更窄就是把肩膀送进
门框），所以默认值刻意偏宽；死区不是，所以它宁可声明成估计值也不写零。

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
