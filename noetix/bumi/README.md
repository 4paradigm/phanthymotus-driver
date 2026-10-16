# Noetix Bumi driver

The bundle exposes the original Bumi sensor, locomotion, audio and camera
cards, plus higher-level motion-state and local visitor cards. Base hardware
cards live in `device.py`; whitelist and automatic guard logic are separate
driver-local modules.

## `auto_guard` card: unattended speech after project start

The `auto_guard` processor turns the existing local whitelist into automatic
greetings. After deploying this driver image, install the four-card canvas
chain from the running container (no updated host checkout is required):

```bash
docker exec -e AGENT_CORE_TOKEN embodied-noetix-bumi python3 /deploy/install_guard_canvas.py
docker exec -e AGENT_CORE_TOKEN embodied-noetix-bumi python3 /deploy/install_guard_canvas.py --apply
```

The first command is a read-only preview. If API authentication is enabled,
set `AGENT_CORE_TOKEN` in the host shell before running them. The script uses
Agent Core's HTTPS loopback address and verifies its persistent certificate. It
preserves other cards and refuses to edit while a project is running. Its
dedicated Perception TTS card ID matches `plugins.auto_guard.tts_instance_id`.
The chain is **Bumi `camera` → Bumi `auto_guard` → Perception `tts` → Bumi
`speaker`**. Start the project once, then set the project to auto-start if it
must resume after a robot reboot. The guard
does not require an operator to click `recognize` for each visitor. It waits
until the exact TTS instance and speaker route are running, then checks fresh
camera frames every 0.5 seconds. Three consecutive single-person matches
produce a fixed welcome; three `unknown` results produce a neutral request to
contact staff. Multiple faces, missing enrollment, stale frames and errors
produce no speech. A visitor must leave for three frames and the 30-second
cooldown must elapse before another announcement.

Review every existing canvas card before starting the project: Agent Core
starts the whole saved canvas, not just this four-card route. If TTS or ROS
publishing fails, the guard discards the old decision and requires three new
fresh observations after recovery. Speech delivery is best-effort; an
ambiguous transport failure can still cause a repeated announcement.

Prepare photos as described below and verify the recognition threshold on the
actual Bumi before enabling project auto-start. `auto_guard` does not control
a door, save visitor frames, call `semantic_action`, or move the robot.
Only the fixed text is sent to TTS; no names/photos are sent. The `info`
action reports `waiting_for_pipeline` or `running`, `pipeline_ready`, and the
last announcement type. ROS topic delivery is not proof that audio was heard:
verify one known and one unknown visitor on the physical robot.

## Agent Core `bumi-guard` Skill

This is a landing-style Agent Core Skill, not another driver card. Its JSON
definition is `deploy/bumi_guard_skill.json`; the `instruction` tells the LLM
how to discuss the guard and query its read-only status. The driver-side
`auto_guard` still performs continuous recognition and fixed speech without
an LLM turn for each visitor. Installing or activating the Skill alone does
not start the camera or the canvas project.

Deploy an image containing this version of the Bumi driver first. Stop the
Agent Core project in the authenticated Dashboard before editing its canvas.
Install the four-card route above, then ensure the canvas has exactly one
`decision_core` card. The following commands run on the **robot host**;
each first previews and then applies one scoped change:

```bash
docker exec -e AGENT_CORE_TOKEN embodied-noetix-bumi python3 /deploy/install_guard_status_binding.py
docker exec -e AGENT_CORE_TOKEN embodied-noetix-bumi python3 /deploy/install_guard_status_binding.py --apply
docker exec embodied-noetix-bumi python3 /deploy/install_bumi_guard_skill.py
docker exec embodied-noetix-bumi python3 /deploy/install_bumi_guard_skill.py --apply
```

The status binder adds only `guard_status` and an execution connection from
the existing `decision_core`. It refuses a running project or a missing or
ambiguous controller. The Skill installer updates only Agent Core's
`skills.installed` record in the host-mounted SQLite database; it preserves
other skills and requires `--replace` if the same slug has different content.
It starts with `active: false`. Refresh the Dashboard, open **Skills**, enable
**Bumi 门卫**, and start the project. In a conversation, ask the agent to use
the Bumi guard skill and report status; Agent Core's `activate_skill` loads
the instruction for that conversation. `requiredTools` is descriptive
metadata, not a wiring command. If the status card is not connected, the
Skill must say it cannot verify live state.

Only an authenticated operator should start or stop the project in the
Dashboard; the Skill does not call hidden processor lifecycle actions or
authorize visitors. Keep enrollment photos on the robot host. Validate one
known and one unknown visitor, TTS playback, cooldown, and the read-only
status result on the real Bumi before enabling project auto-start. Publishing
the Skill to Resource Center is a separate review step after validation.

## `face_whitelist` card

Local proof-of-concept face recognition for one person in the current RGB
frame. The on-demand actuator reuses the existing Bumi camera worker and never
opens a second RealSense pipeline. It does not save visitor frames, speak, move
the robot, or make an access-control decision. It is an actuator because each
`recognize` call returns a result directly; it does not subscribe to an input
topic or publish an output topic as a processor would.

Enrollment remains in the host-mounted private directory:

```text
/opt/phanthy-motus/data/face_whitelist/
  employee_001/
    01.jpg
    02.jpg
    person.json       # optional: {"display_name": "Zhang San"}
```

Use 3–5 clear images per person, each containing exactly one frontal face.
After changing the directory, call `reload`; call `recognize` to inspect one
recent camera frame. Results are `matched`, `unknown`, `no_face`, or
`multiple_faces`. OpenCV LBPH reports a distance (lower is closer), not a
calibrated probability. Validate `distance_threshold` on the actual device and
do not treat this proof of concept as an unattended physical access-control
system.

The Bumi image replaces `opencv-python-headless` with the pinned
`opencv-contrib-python-headless==4.11.0.86` wheel because the recognition code
requires `cv2.face.LBPHFaceRecognizer_create`. For Linux ARM64, the compressed
wheel grows from [29.5 MB](https://pypi.org/project/opencv-python-headless/4.11.0.86/)
to [35.1 MB](https://pypi.org/project/opencv-contrib-python-headless/4.11.0.86/),
an increase of about 5.6 MB. This is a wheel download comparison, not the
installed Docker image delta; measure the latter after the ARM64 build. The
extra size is accepted for this proof of concept because the chosen LBPH API
is only in OpenCV contrib. The dependency is scoped to this driver, installed
without a pip cache, and listed once in `requirements.txt`. `driver.yaml`
publishes the card to Resource Center and has no meaningful image-size effect.

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
