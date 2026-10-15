# Noetix Bumi driver

The bundle exposes Bumi sensor, locomotion, audio, camera, motion-state, and independent sound-direction cards. The direction card is implemented in `direction_card.py`; the other cards remain in `device.py`.

## `mic` and `sound_direction`

`mic` publishes mono PCM `AudioChunk` messages on `/<namespace>/mic/audio`.
`sound_direction` is the only direction card. It publishes
`audio_msgs/msg/SoundDirection` on `/<namespace>/sound_direction` with the
`sensor/sound-direction` canvas format. The angle uses the robot frame:
front 0°, **left positive** (counter-clockwise, REP-103 — the same sign as
the locomotion `vyaw` command), right negative, rear ±180°. A source at
front-left reads about +45 and passes straight to a `vyaw` turn with no
negation. `NO_EVENT` clears the live angle after activity stops.

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
intelligent control leaves `/<namespace>/sound_direction` streaming.

A worker that dies mid-session is **not** revived by `start`/`stop`/`info`;
restart the driver to bring the stream back. A config save restarts the
worker only while it is running; on a dead worker it just writes the
settings.

Set the one shared parameter from the direction card's left-list gear.
Saving it restarts a running direction worker and waits for it to become ready.

| Parameter | Default | Effect |
| --- | --- | --- |
| `sensitivity` | `normal` | How loud a sound must be to count as someone speaking |

Sensitivity presets (choose by room, not by gear knowledge):

- **高** — 安静房间 / 轻声说话也要识别
- **中** — 普通室内（默认）
- **低** — 嘈杂环境 / 只需大声说话

The publish throttle is fixed at 100 ms (about 10 updates per second) and
is no longer operator-adjustable. Gate thresholds per preset live in the
engineer appendix below.

### 旧 settings.json 迁移

The legacy 5-key file at `/opt/phanthy-motus/data/bumi/sound_direction_settings.json`
(`onset_level` / `onset_ratio` / `burst_level` / `burst_ratio` /
`update_interval_ms`) is still read after upgrade: it maps **in memory** to
the nearest sensitivity preset (`onset_ratio` ≤1.75→high, <2.25→normal,
otherwise low) and is never rewritten — the file keeps its old shape until
the operator saves the config once. `update_interval_ms` is dropped; the
throttle stays at 100 ms regardless of any hand-tuned 50/300 value.

Rolling back to an older image: before the first config save the old image
reads the legacy file as before; after a save the old key-set check falls
back to defaults silently. Legacy key names sent to `config` are rejected
(`invalid_parameters`). Keys saved in a canvas config are dropped by the
platform, not migrated.

### 工程师附录：门限映射表

```
SENSITIVITY_GATE_PARAMS = {
    "high":   {"onset_level": 8.0,  "onset_ratio": 2.0, "burst_level": 12.0, "burst_ratio": 3.2},
    "normal": {"onset_level": 10.0, "onset_ratio": 2.2, "burst_level": 15.0, "burst_ratio": 4.0},
    "low":    {"onset_level": 15.0, "onset_ratio": 2.5, "burst_level": 25.0, "burst_ratio": 5.0},
}
UPDATE_INTERVAL_MS = 100
```

Preset meaning is defined by this table, not by the panel wording. The
onset ratios are 2.0 / 2.2 / 2.5 for high / normal / low. Sensitivity
only adjusts how easily sounds trigger; it does **not** remove fan-noise
false angles (known, unfixed).

The card first loads a complete, valid per-robot calibration from
`/opt/phanthy-motus/data/bumi/sound_direction_calibration.json`, then tries the
packaged `resource/sound_direction_calibration.json`. Each file must contain
`front` and `right`, each with three finite values of magnitude at most 24.
The two files are never mixed. The packaged file was copied unchanged from
the calibrated Bumi host `ubuntu` on 2026-10-14, at the per-robot path above.
Its SHA-256 is `89813968f99881310d5f49ca748d48303dfadfd27e550893d27f1023e5fc660a`.
The direction card is registered by default; without
a valid file, `start` and `info` report it unavailable instead of starting a
silent worker. On 2026-10-14, the operator confirmed that this calibration
can be shared between Bumi robots. This portability confirmation is supplied
by the operator; the repository's tests validate file geometry and loading,
not physical direction accuracy.

For existing canvas projects, remove the old `sound_direction_control` card and
reconnect `sound_direction` to the ASR direction input. Its calibration actions
have been removed; update the packaged file through a reviewed change, or use
the per-robot override as an emergency measure. The previous
`/<namespace>/mic/sound_direction` stream is no longer produced.

Run local tests from the repository root with
`python -m pytest noetix/bumi/tests -q`.
The driver image builds its bundled `audio_msgs` definitions, including
`SoundDirection`, over a pinned ROS base image. It does not require a new
`ros-base` release from the main repository. ASR and Agent Core still need the
matching message definition to consume the direction topic.

The published-image [size measurement](review-evidence/image-size-2026-10-14.md)
records a 2,088,960-byte increase in summed history-layer size over the earlier
release without the local interface overlay; the ROS overlay and import-check layers account for
1,146,880 bytes. Both measured images use the same ordered base layers, matching
this robot's cached `ros-base:latest` and the pinned digest. The report separates
Docker size accounting from actual disk consumption and includes raw evidence.

Before deploying the three images together, compare
`noetix/bumi/audio_msgs/msg/SoundDirection.msg` with the main repository's
`deploy/ros-base/audio_msgs/msg/SoundDirection.msg` byte for byte. They match
in these source branches (SHA-256
`78c52d09d430df6e12e1200529d99f3b0f219abe0ffb340f6a847453a5e1016d`).
For the release, record the immutable image digests for Bumi, Perception and
Agent Core, then verify `ros2 interface show audio_msgs/msg/SoundDirection`
inside each published image. Confirm one direction event reaches the ASR input
and the canvas on the target robot. Source equality alone does not establish
which image versions were deployed together; keep this release check open
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
