# Tianyi 2.0 Pro Driver

Phanthy Motus driver bundle for the Tianyi 2.0 Pro humanoid robot. The driver
bridges robot-side ROS2 topics on domain 0 to Agent Core topics on domain 42 and
exposes the capabilities as MCP tools.

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

## External mic on PC1 (`ext_mic` network source)

A USB mic (e.g. a DJI Wireless Mic receiver) plugged into **PC1** is captured
there by `audio_sender.py` and streamed to the driver on PC2 as raw PCM-16k
over TCP (`tcp://192.168.41.1:9800/pcm16k`). PC1 is configured under
`plugins.ext_mic.network_sources` in `config.yaml`.

At startup, and again whenever the card's SSH settings are saved, the driver
prepares PC1 over SSH (`ext_devices.probe_remote_host`):

| Step | What it does | When it cannot |
|---|---|---|
| Devices | `arecord -l` as the SSH user; prefers a USB card | logs which of: SSH failed / no card in the kernel / no capture device |
| Permission | if the kernel has cards but the user is not in `audio`, runs `sudo usermod -aG audio <user>` with the SSH password | logs the exact command to run by hand |
| Sender | copies `audio_sender.py` when **its content** differs (md5), and restarts it | logs `deploy failed` |
| Streaming | ready only once bytes arrive on the port — silence counts, an open port does not | names the option `[not streaming]` and logs the tail of `/tmp/audio_sender.log` |

`audio_sender.py` needs `python3` and `numpy` on PC1 (Ubuntu 22.04 ships both).
It uses pyalsaaudio when a working one is installed and otherwise captures
through `arecord`, so PC1 needs no pip install and no internet. If its capture
fails, the process exits rather than keep an empty port open, and the next
health check restarts it.

## RGB-D input for generic visual odometry

The existing `camera_head`, `camera_depth` and `imu` cards share one private
calibrated source. The standalone `camera_rgbd` card is retired. Original
`camera_head` display output stays unchanged and appends registered RGB on
`/<ns>/camera/rgbd/color`. The sole `camera_depth` output stays on
`/<ns>/camera/head/depth`: display, RGB-D odometry and Point Cloud all subscribe
to the same 640×480 little-endian uint16 millimetre depth, zlib-compressed in
`CompressedImage`; zero pixels are invalid. The old independent display depth
crop/encoder and `/camera/rgbd/depth` publisher are removed. Geometry follows
the calibrated computation path, so the displayed field of view may change.

Each image card waits up to ten seconds for registration/time parameters,
calibration and first pair before source startup returns. Color is sampled
using its distortion and intrinsics onto the registered depth optical rays;
both cards declare the common pinhole K on their actual output topics and
retain source headers. Connect registered RGB and the original depth output
to Perception's `rgbd_odometry`. The same depth output feeds Point Cloud with
`metric_cloud=true`; its `sensor/pointcloud2` and the odometry's `state/odom`
feed SLAM in the common camera frame. Existing display pointcloud packets are
not metric SLAM inputs. This uses the existing runtime identical-frame route,
not a new verified independent-clock/provenance protocol.

The existing `imu` card keeps its display output and appends the named optional
`camera_imu` port (`sensor/imu`) on `/<ns>/camera/rgbd/imu`. It publishes
`motus.imu/1` JSON through
`std_msgs/String`, using the existing `/ob_camera_head/{gyro,accel}/sample`
streams. Each raw vector keeps its actual source frame and capture timestamp;
separate timestamped TF rotations express both source frames in the RGB-D
camera optical frame. It reuses the RGB-D global header clock gate and pairs
motion captures within 20 ms. Missing streams, TF, calibration or fresh captures
withhold only IMU; `info.imu_status` explains availability independently of
RGB-D readiness. Disabling `plugins.imu.enabled` removes the card and its
computation subscriptions. No camera or vendor service is started for motion, and no body/head
mount or native orientation is assumed. Consumer attitude assistance remains
off until its `imu_assisted` option is enabled. A source restart or capture
interruption over 500 ms creates a new motion epoch.

Tianyi hosts running Orbbec ROS 2.5.4 need the separate camera-service
[capture-clock patch](patches/README.md) for independent gyro/accel timestamps
to honor `time_domain=global`. A Motus driver image update alone does not fix
that host service. Samples still pass the unchanged 500 ms freshness and
20 ms pairing checks. Superseded accel history is pruned, and
missing captures cannot pin subsequent valid gyro/accel pairs.

With IMU enabled, the camera input node runs on its own single-threaded
executor in the existing vendor ROS context. It remains alive across source
stop/start so stopped subscriptions discard old input; context shutdown stops
the executor. Callback failure makes the source unavailable and requires a
Driver restart. IMU-disabled sources retain the shared executor. Sampling uses
depth capture time and the original `plugins.camera_depth.hz` limit (default
8 Hz); arrival jitter does not change that sampling clock. This is a rate
limit, not an exact delivery guarantee. Compressed payloads use ROS byte arrays to
avoid Python's per-byte message setter validation without changing encoding.

The internal sidecar publishes `motus.camera_mount/1` JSON on
`camera/rgbd/body_mount`. Both RGB-D `camera_info.vendor.body_mount`
entries declare this topic automatically, so the generic consumer needs no
manual body matrix or extra input wire. Each packet queries TF from the actual
rectified optical frame into `base_link` at the original depth capture
nanoseconds and is published before its matching images. `CAMERA_BODY_FRAME`
can select another body frame name; it does not supply a transform. The
`matrix` field is a 4×4 body-from-camera transform with translations in metres.
Head motion is supported by querying every capture rather than freezing a
startup transform. A source restart creates a new body-mount epoch.

Each card's start is idempotent; stopping it suppresses only its own outputs.
The last user stops the shared worker, while persistent ROS subscriptions
discard stopped input. Capture and per-card queue epochs prevent buffered
frames from crossing a restart. IMU restart requires fresh gyro and accel.
Depth geometry/clock failure stops shared depth rather than falling back to
display-only pixels or arrival timestamps. Optional IMU/body-TF failure is
reported separately and does not stop valid image pairs.

Canvas migration: replace old `camera_rgbd` connections with `camera_head`'s
registered RGB, the single original `camera_depth` port and optional `imu`'s
`camera_imu` port. Existing depth display links keep their topic and format.
This redistribution has offline regression/consumer checks only; earlier
hardware IMU results do not validate this new card lifecycle or performance.

Missing TF, invalid frames/timestamps or unavailable `tf2_ros` produce an
explicit unavailable packet without a held matrix; images and IMU continue.
`info.body_mount_status` reports this independently of camera readiness. TF
availability is not physical calibration: `accuracy_validated` stays false,
and the local visualization URDF does not establish the installed optical
chain. Real TF availability, timing and mount accuracy remain hardware checks.
Camera odometry and pointcloud SLAM keep their camera reference; a compatible
consumer may attach a separately named body pose from matching capture evidence.

The native cloud budget is configured to 20 MB for the observed 1280×720
source. Clock and physical alignment acceptance remain real-device checks;
source declarations do not turn the unverified PR519 SLAM metadata into a
verified clock relation. Sustained tracking and map save/reopen are pending
the user's next test window, not established by offline source tests.

### Generic RGB-D short-term compensation

Perception's generic `rgbd_odometry` card supports the instance settings
`imu_assisted=true`, `short_term_compensation=true`, `depth_geometry=true`.
Connect the same RGB-D pair and optional camera IMU port. The driver reports
full factory/TF sensor-origin translations in `motus.imu/1`; inspect
`imu_status.fusion` for calibration availability. Rotation-only or unsupported
motion sources still support the existing video/attitude modes but cannot
provide translational compensation. No extra SDK owner or robot-specific
fusion algorithm is added.

Keep the camera still for at least0.5s of visual/IMU initialization. The
consumer estimates camera pose using ESKF and conditionally uses observable
depth geometry. Inertial/depth predictions are explicitly non-mapping; SLAM
pauses during coasting/recovery. The hard visual correction timeout is1s
and modeled position sigma limit0.10m; these are software/model thresholds,
not measured robot accuracy. Source clock/mount identity must remain fixed.
See the [2026-11-12 IMU validation](patches/validation-20261112.md) for
the 600-second static IMU continuity result and remaining visual continuity
and motion-accuracy limits.

### Shared-depth RGB pixel consumers

Consumers that combine RGB pixel coordinates with the shared depth must use
`/<ns>/camera/rgbd/color` (the registered RGB port), not the unchanged display
RGB. This also applies to VOP/Measure connections. If Measure overlays an RGB
image, set its existing `camera_topic` to that registered RGB topic; its default
`/camera/color` or `/camera/rgb` guess refers to the native display image.
No camera-to-body matrix or guessed calibration is introduced by this wiring.
