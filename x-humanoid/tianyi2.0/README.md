# Tianyi 2.0 Pro Driver

Phanthy Motus driver bundle for the Tianyi 2.0 Pro humanoid robot. The driver
bridges robot-side ROS2 topics on domain 0 to Agent Core topics on domain 42 and
exposes the capabilities as MCP tools.

## Locomotion state card

`loco_state` starts with the driver and publishes JSON on
`/{namespace}/loco/state` and the same poll as `motus.odom/1` on
`/{namespace}/state/odom` (format `state/odom`). Its input schema is an empty
object. The namespace comes from `ros_namespace` or the hostname.
`nav_state` remains available with its existing map-localized pose output.

The new card uses its own read-only HTTP client in `loco_state.py`, leaving
`nav_client.py` and existing cards unchanged. It reads the chassis address from
`SLAMTEC_URL` or `slamtec.base_url` and polls these endpoints concurrently:

| Reading | Endpoint | Meaning |
| --- | --- | --- |
| Pose | `/api/core/slam/v1/localization/odopose` | Odometry frame position and roll/pitch/yaw, distinct from map localization |
| Velocity | `/api/core/motion/v1/speed` | Measured body-frame `vx`, `vy`, `omega`; unmeasured vertical velocity is null |
| IMU | `/api/core/slam/v1/imu` | Chassis IMU; only fields enabled in `availibilityBitMap` are forwarded |
| Total mileage | `/api/core/statistics/v1/odometry` | Vendor-reported total travelled distance in metres; not displacement from a task origin |

`position` and `velocity` are XYZ arrays; `rpy` is roll/pitch/yaw and
`yaw_speed` comes from `omega`. `v` is the signed measured forward speed `vx`,
`w` is measured `omega`, and `length` is the chassis total mileage. These
aliases and units are also available under `vendor` on the standard port.
IMU quaternion order is WXYZ. IMU values retain
the vendor units; the supplied API does not specify units for those fields.
IMU `euler_angle` retains vendor XYZ order without assuming a roll/pitch/yaw convention.
Its `timestamp` is chassis uptime and is exposed as `imu.uptime_ms`, separately
from the host Unix `timestamp_ms` and per-source `sample_timestamps_ms`.
The REST requests are separate observations, not a synchronized hardware sample.

Unavailable R1 fields `mode` and `body_height` are `null`. `motion_status` is
derived from measured speed and does not claim to be a controller operating mode.
Failed requests produce current errors and null readings instead of reusing a
previous successful pose or speed. `available` requires valid odometry pose and
speed; IMU validity is reported separately in `imu.available`.
Mileage validity is independent too: a missing mileage endpoint sets `length`
to null and reports `errors.length`, while valid pose and twist remain usable.

`plugins.loco_state.hz` defaults to a target of 10Hz (maximum 10); HTTP latency
can lower the actual rate. `request_timeout` defaults to 0.5 seconds per request.
There is at most one four-request group in flight. MCP `stop` returns the
canvas lifecycle marker `{"state": "idle"}` while leaving this always-active
observer running, matching R1; driver shutdown stops the worker.

The standard output uses `common.odom` from the official driver repository:
`twist = [vx, vy, null, null, null, omega]`, `frame = "body"`, SI units,
and a host receive timestamp for the speed response. Partial speed responses
retain each valid axis, including measured zero. Invalid or absent axes are
null. There is one speed observation per publish, with no faster stream being
discarded. The IMU boot-relative timestamp is never used as `stamp_ms`.

`info()` includes a validated `odom_interface` with `provides = ["vx", "vy", "wz"]`,
the target rate, and `pose_drift = "unbounded"`: the source is uncorrected wheel
odometry, not map localization. Standard `pose` now contains measured position
and an orientation quaternion derived from the same endpoint's roll/pitch/yaw:

```json
{
  "stamp_ms": 1792052530000,
  "frame_id": "odom",
  "child_frame_id": "base_link",
  "position": {"x": 1.2, "y": -2.0, "z": 0.1},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.70710678, "w": 0.70710678}
}
```

This is the `pose` member of the existing JSON `motus.odom/1` message, not a
ROS `nav_msgs/Odometry` message. Its frame is explicit: standard `frame = body`
still describes the twist, whereas `pose.frame_id = odom` describes position.
Quaternion components are named to avoid XYZW/WXYZ ambiguity. `pose.stamp_ms`
is the host receive time of the pose reply; the message's top-level stamp is
the independently received speed reply. Missing or malformed position or RPY
produces `pose = null`, never a fabricated identity pose. No pose covariance,
ground contact, or unmeasured velocity axes are invented.

`displacement` is calculated from the first valid odometry pose during this
driver instance. It contains `origin`, XYZ `translation` in the odom frame,
`distance` (Euclidean translation from that origin, metres), and `yaw` (heading
change wrapped to [-pi, pi], radians). Standard consumers find it under
`vendor.displacement`. Returning to the origin makes `distance` approach zero;
the vendor's total `length` still counts the path travelled. Neither value is
computed by integrating velocity. On a pose failure displacement is null and
the original reference is retained for recovery. Canvas start/stop does not
reset that reference; restarting the driver creates a new one. If the chassis
itself resets its odometry frame, restart the driver to establish a new origin.

Odometry position, RPY, IMU, mileage, displacement, errors, and separate request
timestamps remain under `vendor` too. Short-distance displacement and pose
scale/drift need hardware validation against a measured distance before they
can be used as a precision target. This follows the upstream
[Robot Odometry specification](https://github.com/4paradigm/phanthymotus-driver/blob/main/README_dev.md#robot-odometry-motusodom1).

## Locomotion servo card

`loco_servo` is a separate actuator for an incoming `control/velocity` stream.
It defaults to `/actucore/navi/cmd` on agent-core's domain 42 and accepts JSON
`std_msgs/String` messages using the shared `motus.control/1` twist contract:

```json
{
  "schema": "motus.control/1", "mode": "twist", "dof": 6,
  "values": [0.1, 0, 0, 0, 0, 0.2],
  "source": "navi", "seq": 1, "priority": 0,
  "stamp_ms": 1792370000000,
  "obs_stamp_ms": 1792370000000,
  "ttl_ms": 200
}
```

The timestamps above are examples: producers must supply current Unix
milliseconds and increasing sequence numbers. The six values are body-frame
`[vx, vy, vz, wx, wy, wz]` in m/s and rad/s. This implementation supports `vx`
and `wz` only; lateral, vertical and roll/pitch commands must be zero. This
restriction does not claim lateral motion has been validated on the chassis.
Configured operating caps default to 0.3 m/s and 0.6 rad/s, not vendor maximum
ratings. Non-finite values, unsupported axes and out-of-range commands are
rejected. `ControlSink` checks the schema, TTL, observation age, sequence and
source priority. Rejected/stale messages never feed the watchdog.

The domain-42 input uses the existing socket subscription bridge. Output now
uses the official Slamware C++ SDK, **not ROS `/cmd_vel`**: the Tianyi hardware
check found zero subscribers on that topic. `nav.move_to` and `chassis_raw`
use REST actions; their existence does not establish a `/cmd_vel` receiver.
Discrete REST direction actions cannot preserve arbitrary simultaneous vx/wz.

A private `tianyi-loco-sdk` worker connects to `sdk_host:sdk_port` (defaults
`192.168.11.1:1445`, distinct from HTTP port 1448). It calls
`SlamwareCorePlatform::velocityControl()` with the official localization-quality
monitor enabled, then `VelocityControlMoveAction::setVelocity(vx, 0, wz)`.
It does not use `NotMonitored` or change global speed/brake/emergency-stop
parameters. `native/build_sdk.sh` pins the official ROS2 SDK 5.1.1 GCC11 package
and SHA256 for ARM64 and AMD64; it compiles only this private C++ worker, not
the vendor ROS server. Third-party license notices are installed in
`/usr/share/doc/tianyi-loco-sdk/`.

Bundle startup is inert. MCP/canvas `start` subscribes to the default input,
or an explicitly provided `input_topic` / first `input_topics` entry. **New
cards default to dry-run.** Dry-run does not launch/connect the SDK worker or
create a chassis action. Turning dry-run off while running first requires a
successful read-only SDK connection. Connection alone creates no motion action.
An incompatible SDK/firmware, a disconnected SDK endpoint, or low localization
quality is reported as an error; no direction-action fallback is attempted.
The endpoint and actual firmware velocity/heartbeat behavior still require
hardware validation with the new image.

`info` returns the control descriptor, counts, last verdict/output, settings,
and `backend` (type, host/port, connection status, last acknowledged action ID,
error and heartbeat handover wait). `output_topic` is null: there is no ROS
velocity publisher to inspect. `published` is retained for clients, but now
counts acknowledged SDK velocity/stop operations, not measured physical motion.
The backend state/action ID describe the last acknowledgement, not a substitute
for odometry feedback; verify physical motion with `loco_state`.

The Python and native layers enforce the input TTL/observation-age deadline,
capped at 300ms. The native worker independently stops when that lease expires
(with a 20ms polling interval while SDK calls are responsive), even if ROS stops
spinning. It never holds a queue of old velocity commands or reconnects/replays
a failed request. A blocked SDK reply times out (default 250ms), kills the worker
and pauses the card. While the worker owns a motion action it enables the
firmware's one-second heartbeat and refreshes it only within a live command
lease. This is a **seconds-resolution backup**, not a claim of a 300ms stop
after a process crash or network failure. Firmware heartbeat behavior must be
checked on hardware. On an ambiguous failure, handover waits 1.5 seconds for
that firmware lease before another card is permitted to act. A faulted worker
requires explicit stop/start; it does not automatically regain control.

Acceleration caps remain 0.3 m/s² and 0.6 rad/s². Explicit zero, pause, stop,
lease expiration and watchdog stop bypass the ramp. Cancelling an SDK action
addresses only this worker's held action handle. Before beginning or updating
motion, the worker checks for another active action or replacement of its own
handle; it refuses to overwrite that action. Raw velocity streaming does not
add obstacle avoidance.

Configure loco_servo exposes three live boolean settings via `action: config`:

| Setting | Default | Effect |
| --- | --- | --- |
| `dry_run` | true | Validate, shape and count commands without SDK motion; enabling it during real execution first stops/cancels the old SDK action |
| `rotate_only` | false | Suppress translation and preserve yaw rate; unsupported axes remain rejected |
| `require_standing` | false | Require fresh true feedback from a configured domain-0 Bool source; missing/stale/false feedback stops/refuses motion |

Tianyi has no verified native standing FSM in the supplied API. Do not enable
`require_standing` without a verified `standing_topic`. Feedback expires after
`standing_timeout` (default one second). These source parameters and SDK endpoint
parameters live in `plugins.loco_servo` in the deployment YAML.

`pause` stops but retains the subscription; `resume` accepts newly generated
commands. Start/resume/config changes drop pre-transition commands. `stop`
returns idle and releases nodes/worker only once the stop is confirmed (or the
failed worker's firmware lease wait has elapsed); otherwise it reports an error
and remains paused. Starting after stop creates a new worker with no replay.

At the bundle dispatch boundary, explicit `nav`, `home` and `chassis_raw`
motion/cancel actions pause this stream before executing. An unconfirmed SDK
stop blocks the handover. Successful handover cancels the owned velocity action
and disables its heartbeat before the new owner runs; the yielded stream emits
no more motion, zero or heartbeat writes. Starting/resuming after call-shaped
motion still requires explicit `nav.cancel`, `home.cancel` or
`chassis_raw.brake`. Existing card implementations, `nav_client.py` and
`loco_state` are unchanged. Controllers bypassing the bundle need separate
coordination; the SDK firmware heartbeat is global, with only one active token.

### First deployment checks

1. Keep **空跑 = Yes**, start the canvas and issue a navi goal. `received`,
   `accepted` and `dry_run_commands` should increase; `published` stays zero.
   Navi's blocked detector can fire because dry-run intentionally causes no motion.
2. In a controlled hardware test, turning dry-run off must report
   `backend.type = slamware_sdk`, `connected = true`, and no backend error.
   Check odometry against small vx-only, wz-only and combined commands.
3. Test zero, pause, stop, input loss, worker loss, low localization quality,
   and transfer to/from nav/home before using unattended navigation. Read the
   actual stop result, not just the UI button state.

Official references: [platform and heartbeat API](https://developer.slamtec.com/docs/slamware/cpp-sdk-en/4.6.2_rtm/slamware_core_platform/),
[velocity action](https://developer.slamtec.com/docs/slamware/cpp-sdk-en/2.8.2_rtm/VelocityControlMoveAction/),
[SDK downloads](https://www.slamtec.com/en/support).

## Head camera snapshot card

The `vision_capture` card subscribes to `/ob_camera_head/color/image_raw` and
provides `capture_image`, `record_video`, `start_recording`, `stop_recording`,
`list`, and `delete` actions. Each capture or recording is saved under
`/opt/phanthy-motus/data/images` and returns a channel-visible path. The
driver derives the corresponding `/work/resource/...` path from the persistent
mount (or uses `PHANTHY_CHANNEL_OUTPUT_DIR` when a deployment uses another
mount). Use that returned path with `channel_reply(files=[{"path": "..."}])`
to send media through Feishu or Telegram. Use `image_name` (without `.jpg`)
for photos and `video_name` (without `.mp4`) for videos. To delete media, pass
Use `delete` with the complete filename in `name`, including `.jpg` or `.mp4`.

## Raw arm joint card

`arm` directly commands seven joints per selected arm in this canonical order:

```text
[shoulder_pitch, shoulder_roll, shoulder_yaw,
 elbow_pitch, wrist_yaw, wrist_pitch, wrist_roll]
```

The card always exposes separate `left_positions` and `right_positions` arrays
in degrees and publishes both arrays in one 14-joint command, so the arms can
move to different poses simultaneously. There is no `side` selector. Raw arm
input is never mirrored automatically. To create a mirrored right-arm pose manually, negate
shoulder roll, shoulder yaw, wrist yaw, and wrist roll (indices 1, 2, 4, and 6)
from the left pose. Right-arm motor IDs are 21-27 and left-arm IDs are 11-17.
`move_pos` accepts speed [0.2, 1.5] rad/s. `move_ctrl` accepts seven-element
`kp` [10, 200] and `kd` [5, 50] arrays shared by both arms. Raw-control defaults
are `kp=50` and `kd=20` for every joint. The bounded tuning range includes the
tested lower-gain and original-default combinations while excluding zero
position stiffness and the previous unverified extreme upper bounds. These are
still not vendor-certified gains.

`move_pos` uses the rated-current limits from the Tianyi joint specification
table instead of one fixed current for every arm motor. In joint order, each
arm uses `[35, 23, 8, 8, 8, 5, 5]` A for shoulder pitch, shoulder roll,
shoulder yaw, elbow pitch, wrist yaw, wrist pitch, and wrist roll. The same
limits apply to left motor IDs 11-17 and right motor IDs 21-27. Raw and semantic
head position commands use 5 A for motor IDs 1-3. `move_ctrl` has no current
field and is unaffected by this mapping. Do not increase these values beyond
the specification; clear the workspace and keep the emergency stop available
when validating the higher-current shoulder joints.

`move_ctrl` sends a position step with target speed and feed-forward torque set
to zero. The vendor-side controller applies `kp` and `kd`; this driver does not
calculate the PD output or impose a trajectory speed limit. Use small target
changes, begin with low gains, keep the emergency stop available, and use
`move_pos` when bounded motion speed is required.

In practical terms, `move_ctrl` is intended for controller commissioning,
pose holding and already-validated compliant interaction where the operator
needs to tune joint stiffness and damping. Higher `kp` generally increases
position response and holding stiffness but can increase impact and overshoot;
lower `kp` is more compliant but can leave a larger load-dependent position
error. Higher `kd` generally adds damping, while insufficient `kd` can allow
overshoot or oscillation. The exact control law runs in the vendor controller,
so these effects are guidance rather than a driver-side torque calculation.
The same seven-element gain arrays are applied to corresponding joints on both
arms. Do not use `move_ctrl` as a substitute for `move_pos` for routine poses,
large position steps or semantic gestures.

Both modes reject malformed arrays and poses outside the checked-in URDF
limits. They check fresh `/arm/status`, selected motor faults, emergency stop,
and power state before publishing, then wait for newer feedback. Do not command
the raw `arm` and `arm_gesture` cards concurrently because both can publish to
`/arm/cmd_pos`.

For compatibility with dashboard array fields, `left_positions`,
`right_positions`, `kp`, and `kd` accept either native JSON arrays or strings
containing a JSON array. Both forms are decoded into seven numeric values before
the same range and URDF checks run. Direct callers using the former `positions`
field remain supported as a fallback, but its values are sent unchanged to both
arms and are no longer mirrored.

## Head gesture card

`head_gesture` turns safe, bounded head-position commands into cancellable
semantic sequences.

| Item | Value |
|---|---|
| Tool name | `head_gesture` |
| Tool type | `actuator` |
| Robot-side output | `/head/cmd_pos` |
| Message type | `bodyctrl_msgs/msg/CmdSetMotorPosition` |
| Actions | `nod`, `shake`, `scan`, `tilt`, `reset`, `stop` |

Yaw, pitch and roll are clamped to the limits already documented by the raw
`head` card. Starting a new gesture cancels the remaining frames of the previous
gesture; `stop` cancels future frames without issuing an additional pose. A nod
moves from neutral to positive pitch (down) and back to neutral; it never passes
through negative pitch (up).

Before publishing, the card checks fresh `/head/status` data, all three head
motors, motor error codes, emergency-stop state and power state. It then waits
up to two seconds for newer status and verifies that the head moved or was
already at the target. A verified call returns `feedback_verified: true`;
failures return `state: error`, a stable diagnostic `code`, details in the MCP
result, and protocol-level `isError: true`.

| Action | Dashboard defaults | Allowed range |
|---|---|---|
| `nod` | cycles 2, amplitude 12°, speed 30°/s | cycles 1–5, amplitude 5–20°, speed 5–60°/s |
| `shake` | cycles 2, amplitude 25°, speed 30°/s | cycles 1–5, amplitude 5–45°, speed 5–60°/s |
| `scan` | cycles 2, amplitude 25°, speed 30°/s, hold 1.0s/side | cycles 1–5, amplitude 5–45°, speed 5–60°/s, hold 0.2–3.0s/side |
| `tilt` | left, amplitude 12°, speed 30°/s, hold 0.8s | side left/right, amplitude 5–20°, speed 5–60°/s, hold 0.2–3.0s |
| `reset` | speed 30°/s | speed 5–60°/s |
| `stop` | no parameters | no parameters |

## Arm gesture card

`arm_gesture` provides semantic arm motions on top of the existing joint-level
`arm` card.

| Item | Value |
|---|---|
| Tool name | `arm_gesture` |
| Tool type | `actuator` |
| Robot-side output | `/arm/cmd_pos` |
| Message type | `bodyctrl_msgs/msg/CmdSetMotorPosition` |
| Actions | `salute`, `welcome`, `raise`, `shake_hands`, `high_five`, `reset`, `stop` |

The right-arm poses are mirrored from the left-arm definitions and remain
inside the checked-in URDF limits. The preset poses still require low-speed,
clear-area calibration on the target robot before production use. Do not run
the raw `arm` card and `arm_gesture` concurrently because both publish to the
same controller topic.

The semantic poses use a preparation frame before the final gesture. For
`welcome`, `salute`, and `high_five`, shoulder roll and shoulder yaw first
establish the elbow-flexion plane; elbow pitch can then raise the forearm
instead of leaving it horizontal in front of the torso. `salute` uses two
blended stages: the preparation frame bends the elbow and moves wrist yaw to
25 degrees; the second frame raises the upper arm laterally, flexes the elbow
to approximately 110 degrees, and completes wrist yaw at 50 degrees. In this
URDF chain, shoulder yaw rotates the downstream
elbow-pitch axis: positive left-arm yaw (mirrored negative on the right) makes
negative elbow pitch lift and fold the forearm inward, while the opposite yaw
direction drives the forearm downward. The preparation stage has no dwell and
hands off at 90% of its calculated transition time to avoid stop-start motion.
The action is intentionally limited to one arm at a time to avoid interference
near the head. The preparation frame bends the elbow before completing the
shoulder rotation, avoiding a fully extended sweep near the head. `welcome`
raises the hand beside and above the torso rather than in front of the chest.
It keeps shoulder yaw and all wrist axes fixed at their selected values, with
all three wrist angles remaining neutral throughout. It sweeps elbow pitch
between -110 and -90 degrees, which the checked-in URDF places mainly along the
lateral axis; changing shoulder yaw here would instead move the hand mainly
forward and backward.
`raise` lifts the upper arm close to overhead while keeping only a moderate
elbow bend, making its silhouette distinct from `welcome`.
`shake_hands` extends one arm with a smaller shoulder-pitch angle than the old
forward-reach pose and uses a small elbow sweep for the handshake.
`high_five` places the wrist approximately 0.37 m in front of the torso plane
and at approximately the shoulder-joint height in the checked-in URDF.

Wrist pitch stays neutral in every semantic frame. `salute` moves wrist yaw
progressively from 25 degrees in the preparation frame to 50 degrees in the
final frame. `welcome` keeps all wrist joints at zero for the entire sequence.
For the left-arm `high_five`, wrist roll moves from 10 degrees in the
preparation frame to 50 degrees in the final frame; the right arm uses the
mirrored negative angles.

The URDF defines joint axes and limits but contains no hand palm frame and no
arm visual/collision geometry. Palm-facing wrist values are therefore
conservative starting points, not geometrically proven orientations. Calibrate
`welcome`, `salute`, and `high_five` one arm at a time at low speed
with a clear workspace. Bilateral `salute` remains blocked because the hands
move near the head. Bilateral `high_five` is allowed because its mirrored paths
remain separated in front of the shoulders. Actions such as clapping, hugging,
and crossing arms are intentionally not provided until collision geometry or a
separate collision checker is available.

Before publishing an action, the card checks fresh `/arm/status` data, all
selected motor IDs, motor error codes, the physical/remote emergency stop and
power state. After publishing, it waits up to two seconds for a newer
`/arm/status` sample and verifies that a selected joint moved (or was already
at the requested target). The MCP result therefore distinguishes a scheduled
action from controller feedback with `feedback_verified: true`. Failures return
`state: error`, a stable `code`, the human-readable `error`, and relevant
diagnostic details.
MCP tool failures are also marked with the protocol-level `isError: true`, so
the dashboard/agent can distinguish them from successful results without
parsing the text payload first.

The SDK does not expose a dedicated "self-check completed" field in the message
types used here. A no-motion result can therefore identify an incomplete
self-check/Not Ready state as a likely cause, but cannot claim it conclusively.
Complete the documented startup self-check and confirm the whole-body status
light indicates Ready before running arm actions.

| Action | Dashboard defaults | Allowed range |
|---|---|---|
| `salute` | right arm, speed 0.5rad/s | side left/right, speed 0.2–1.5rad/s |
| `welcome` | right arm, cycles 2, speed 0.5rad/s | side left/right/both, cycles 1–5, speed 0.2–1.5rad/s |
| `raise` | right arm, speed 0.5rad/s | side left/right/both, speed 0.2–1.5rad/s |
| `shake_hands` | right arm, cycles 2, speed 0.5rad/s | side left/right/both, cycles 1–5, speed 0.2–1.5rad/s |
| `high_five` | right arm, speed 0.5rad/s | side left/right/both, speed 0.2–1.5rad/s |
| `reset` | right arm, speed 0.5rad/s | side left/right/both, speed 0.2–1.5rad/s |
| `stop` | no parameters | no parameters |
