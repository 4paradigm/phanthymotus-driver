# Noetix Bumi driver

The bundle exposes the original Bumi sensor, locomotion, audio and camera cards plus one higher-level motion-state card backed by documented Noetix SDK APIs. All card implementations are kept in `device.py`.

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

Useful observations while the driver is running:

```bash
ros2 topic echo /<robot_namespace>/motion/state
ros2 topic hz /<robot_namespace>/motion/state
docker logs -f embodied-noetix-bumi
```
