# Noetix Bumi driver

The bundle exposes the original Bumi sensor, locomotion, audio and camera cards plus two higher-level cards backed by documented Noetix SDK APIs. All card implementations are kept in `device.py`.

## New cards

### `motion_state`

Read-only whole-body motion telemetry from `HighController`.

- `snapshot` with `detail: summary`: current activity, workmode context, protection flag, body orientation, angular velocity, linear acceleration, whole-body joint activity statistics and active motor faults.
- `snapshot` with `detail: joints`: only the position, velocity, torque, temperature and error for each of all 21 joints, plus source/freshness metadata; it does not repeat the summary.
- `history`: recent motion-start/stop, mode, protection, controller-read and motor-fault events. Select `detail: none`; `limit` is optional and defaults to 20.
- `clear_history`: clear only the in-memory motion event history. It does not stop motion, clear robot faults, or change robot state. `history` is empty afterwards until a new change is recorded.
- ROS2 output: `/<namespace>/motion/state`, JSON.

The default polling and topic publication rate is 2 Hz (`poll_interval_s: 0.5`).
Motion activity uses a configurable joint-speed threshold, defaulting to
`0.15 rad/s`. Only motor error codes explicitly documented by Noetix are
classified as faults. Undocumented non-zero raw values are not shown in the
summary and remain available only in the `joints` view's per-joint `error`
field for device-side verification.

Every snapshot identifies `Noetix HighController/CycloneDDS` as its source and includes freshness and sample-age fields. It deliberately excludes battery data, which belongs to the existing `battery` card. The SDK does not expose world-frame position or translational velocity, so the card reports only documented IMU and joint measurements and does not invent odometry.

### `media_system`

MediaController system management.

- `status`: work status, change reason and last system error.
- `get_config` / `set_config`: timeout, cue, response text, audio routing, video routing and internal 3A selection.
- `get_wake_words`: read the current wake-word grammar and extracted human-readable phrases. The SDK does not expose wake-word mutation.
- `wakeup`, `sleep`, `restart`: control the media Agent itself. Command-triggered `wakeup` and `sleep` do not play the configured response text; their `accepted` result means the command was sent, not that a voice reply occurred.
- `pause` / `resume`: audio capture, audio playback, video capture, or all paths.

All mutating MediaController calls are serialized with the documented minimum 500 ms interval. `set_config` reads the configuration back before returning `completed`.

On the canvas, enter `set_config.config` as a JSON object, for example
`{"audio_cue":true,"timeout_ms":30000}`. The driver accepts both this JSON text form and a native object supplied by Agent Core. Fields omitted from the object are left unchanged.
`wakeup_response` and `sleep_response` are voice-interaction reply texts, not
wake words, and are not played by command-triggered `wakeup` or `sleep`.

Useful observations while the driver is running:

```bash
ros2 topic echo /<robot_namespace>/motion/state
ros2 topic hz /<robot_namespace>/motion/state
docker logs -f embodied-noetix-bumi
```
