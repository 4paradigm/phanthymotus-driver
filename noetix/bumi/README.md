# Noetix Bumi driver

The bundle exposes Bumi sensor, locomotion, audio and camera cards, a motion-state card, and a pose-driven squat counter. Hardware card implementations are in `device.py`; the squat counter is in `squat_counter.py` and `squat_logic.py`.

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

### `squat_counter`

Fixed-camera squat counting from the **full** `pose` skeleton stream. Connect
`/<namespace>/camera/color` to the perception `pose` card, then connect its
`/<namespace>/camera/color/poses/skeleton` output (`sensor/pose2d`) to
`squat_counter`. On the canvas this is pose output port **1**; port 0 is the
lean `/poses` output and lacks the joints needed for counting.
Keep the user fully visible, with shoulder, hip, knee and ankle in frame through
the whole movement; start with a roughly 45-degree oblique view and calibrate
against footage from Bumi's final position.

Start the counter with the skeleton `input_topic`. `status` returns its latest
session and authoritative `count_total`; `debug` returns the latest measurements
and checks; `reset` starts a fresh count; `stop` ends the current subscription.
The card publishes:

| Topic | Use |
| --- | --- |
| `/<namespace>/fitness/squat/events` | Sparse `session_started`, `rep_completed`, `target_lost`, `target_reacquired`, and `target_changed` events for `decision_core`. |
| `/<namespace>/fitness/squat/status` | Current state and count for the dashboard; **do not** connect its frequent updates to `decision_core`. |
| `/<namespace>/fitness/squat/debug` | Latest target choice, smoothed knee/hip angles, baseline and deltas, threshold checks, and rejection reason; dashboard only. |

The counter locks onto the person with the clearest usable full-body skeleton
and ignores other people. It keeps that target through brief occlusions. A
confirmed target change abandons an unfinished repetition and starts a new
session, so counts from different people are not silently combined. A
repetition is recorded only after a qualified descent and a stable return to
standing. The geometric and timing thresholds are **provisional**; validate
them with complete squats, partial squats, bends, occlusions, and crossings
recorded from the intended camera position.

The default calibration needs 2 s of stable standing. A rep then needs knee
flexion of at least 40°, hip flexion of at least 25°, and hip descent of at
least 0.10 standing leg lengths, followed by a stable return to standing.
These are relative 2D image measurements, not joint angles measured by a
depth sensor. Adjust the keys in `squat_logic.DEFAULT_CONFIG` under
`plugins.squat_counter` in `config.yaml` after checking Bumi footage. The
wrapper publishes dashboard status and debug data every 0.5 s and coalesces
quick rep events within 3 s (`status_interval_s`, `debug_interval_s`,
`min_rep_event_interval_s`). The debug stream contains no raw skeleton; when
pose input stops, inspect `input_stale`, `last_frame_age_s`,
`frames_received`, `frames_parsed`, and `last_error`. For missed reps, compare
`measurements.knee_drop_deg`, `measurements.hip_descent_ratio`,
`checks.bottom`, `timing.bottom_dwell_elapsed_s`, and `reason`; inspect
`frame_gap_exceeded` for pose stalls.
In dashboard **Monitor** mode, open `/fitness/squat/debug` in Latest or Log
view. The card's `debug` action returns the same snapshot on demand.

Connect `squat_counter` output port **0** (events) to `decision_core`, and bind
its `status` action for readback. Leave output ports **1** (status) and **2**
(debug) off the model's input. Each event includes a top-level `priority: 1`,
`session_id`, `seq`, `count_total`, and `event_time`. Treat the card's total as
the source of truth: a model turn and TTS playback can outlast several squats.
Before speaking, read `status` and announce only the latest total for the
current session; discard stale event numbers. This path gives best-effort
progress narration, not a guarantee that every number is spoken immediately.
On `target_changed`, `previous_count_total` preserves the prior session's
final total. `status.input_stale` flags a pose stream that has stopped sending
frames; it does not infer that a missing frame contains a person.
To install the instruction in `fitness_squat_skill.md`, create a skill under
Resource Center → My Skills with slug `bumi-squat-coach`, category `utility`,
and required tools `squat_counter, tts`; paste the file's instruction body,
install the draft, and mark it active. Then let `decision_core` call
`activate_skill("bumi-squat-coach")` when the user chooses squat. Bind TTS
`speak` and connect its PCM output to Bumi `speaker`. The driver has no local
skill loader. The model can call `status` and `reset`; the canvas owns `start`
and `stop`.

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
