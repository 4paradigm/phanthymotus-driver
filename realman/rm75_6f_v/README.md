# RealMan RM75-6F-V Driver

This Linux ARM64 image exposes RM75 controller data and bounded joint motion to
Phanthy Motus through MCP. It calls the official RealMan API2 Python SDK
directly; it does not build or run the official ROS2 `rm_driver`.

```text
Card / Agent Core -> MCP Driver -> RealMan API2 SDK -> arm controller
```

The deployment service connects to `192.168.1.18:8080` with motion enabled by
default; set `RM75_ARM_IP` on the host to select a different controller. Before
starting it, verify the network, physical E-stop, work area and joint order.
Every movement still requires explicit per-call confirmation:

```bash
confirm_motion=true
```

Available tools are:

- `connection`: SDK connection state.
- `joint_states`: seven joint angles in radians, plus raw SDK degrees; also
  publishes a 10 Hz `sensor/skeleton` stream for the Canvas URDF renderer.
- `robot_info`, `software_info`, `arm_all_state`, `controller_state`: read-only
  API2 queries.
- `model`: simplified RM75-6F-V URDF for live skeleton display. Its seven
  movable joint names exactly match the `joint_states` skeleton stream.
- `joint_control`: bounded joint-space motion and controlled stop.
- `servo`: continuous 8-dimension control from a `motus.control/1` stream, and
  the `state/joint` proprioception a VLA binds as its observation input. See
  "Continuous control" below.
- `gripper`: two-finger gripper position, one call-shaped motion at a time.
  Mutually exclusive with `servo`, which streams the same gripper as dimension
  8 of its action space.
- `abs_move`: `move` reads the current TCP as A and moves to
  the requested base-frame B using
  joint-space planning by default, with an explicit TCP-linear mode available.
  Relative `move_offset`, absolute `movel`, and waypoint `movep` are not exposed.
- `ext_camera`: multi-instance upper-computer USB camera card. A RealSense
  instance can publish RGB, depth, or left infrared without going through the
  RealMan controller.

## Continuous control (`servo`)

`joint_control` is the call-shaped path: one absolute target per `tools/call`,
an ACP completion, a `confirm_motion` per request. That is right for an LLM
deciding to move the arm somewhere, and wrong for an execution model, which
produces tens of commands per second and is not asking a question each time.

`servo` is the stream-shaped path. It subscribes to a `control/joint` topic,
runs every message through the shared `common/control.ControlSink`, and passes
what survives to the RealMan passthrough API (`rm_movej_canfd`). Freshness,
arbitration between sources, step clamping, hard limits, the watchdog and its
escalation all live in the sink and are tested without an arm.

Both paths reserve the same physical arm through one shared motion gate, so
while `servo` is streaming, `joint_control`, `gripper`, `abs_move` and
`pick_place` are all refused with `another arm operation is active`. Every
movement still requires `RM_MOTION_ENABLED=1`; a card that streams does not
earn an exemption because its commands are frequent — it is checked once at
`start`, which is where a human is present.

### The action space is 8 dimensions

| index | meaning | unit | limits |
| --- | --- | --- | --- |
| 0-6 | joint 1-7 | rad | the same vendor limits `joint_control` enforces |
| 7 | gripper | normalized closure | 0.0-1.0 |

**The gripper is dimension 8, not a side channel.** An execution model emits
"move the hand here *and* close the gripper" in one timestep; the call-shaped
`gripper` card cannot take that, and a model whose gripper dimension went
nowhere would negotiate successfully, report every command as applied, and
never pick anything up.

Setting `servo.gripper: false` makes this a genuinely **7-dimension** card, not
an 8-dimension one that ignores the last value — so a model with a gripper
fails negotiation instead of being half-executed. Which one it is gets decided
by asking the robot at `start` (`rm_get_gripper_state` must report
`status: 1` **and** `enable_state: 1`); a config claiming a gripper the arm does
not have is refused rather than silently downgraded.

**Gripper polarity: 0 is open, 1 is closed — and this is not yet verified on
hardware.** The SDK takes an *aperture* of 1-1000 in the opposite direction, so
closure 0 maps to 1000 and closure 1 maps to 1. Whether a given model's gripper
dimension means closure or aperture is **not** something negotiation can check:
`motus.vla/1` passes the gripper through unchanged, deliberately, because
normalizing it needs the travel of this particular gripper, which is the
robot's knowledge and not the model's. Getting it backwards turns "open" into
"clench" around an object — the dangerous direction. **Before trusting a policy
with this arm, command closure 0 and 1 by hand once and watch the gripper.**

The gripper is a serial device behind the controller, not a 30 Hz joint, so its
commands are rate-capped (`servo.gripper_max_hz`, default 10) and deadbanded
(`servo.gripper_deadband`, default 0.02). The arm itself keeps running at
`expected_hz`. Both values are reported in `info()` — a cap nobody can see reads
as "every command reached the gripper" when it did not.

On a watchdog or an abort the arm slow-stops and the gripper is **held**, not
released: releasing drops whatever is being carried onto whatever is underneath,
and this driver cannot know which is worse. Set
`servo.release_gripper_on_watchdog: true` to open it instead.

### Proprioception comes out of this card

`servo` publishes `state/joint` on `/{namespace}/servo/state`, which is what a
VLA card binds as its observation input. Wire it back to the policy on the
canvas: `vla → servo →(state)→ vla`.

```json
{"schema": "motus.control/1", "kind": "joint_state", "dof": 8,
 "joint_names": ["joint_1", ..., "gripper"],
 "values": [7 joint angles in rad, gripper closure],
 "eef": [x, y, z, qx, qy, qz, qw, gripper closure],
 "stamp_ms": 1760000000000}
```

`values` is in the same order and the same units as the commands this card
accepts, because both come out of one `build_descriptor()` — the two cannot be
changed independently. `eef` is the current end-effector pose, in metres and a
unit quaternion (**xyzw**), and exists so an incremental model (OpenVLA emits an
end-effector delta) has the base pose its delta is relative to — only the robot
knows where "here" is.

Two things to be aware of:

- **`stamp_ms` is when the sample was taken**, not when it was published. A
  policy uses it to compute observation age, and reporting the publish time
  would claim the channel is fresher than it is.
- **`eef` is in the controller's currently active tool frame.** This driver does
  not own that setting — see the `cartesian.tool_length_mm` comment in
  `config.yaml`, which is only 222.5 mm when the controller has been set to the
  gripper TCP. The assumption is reported in `info()` under
  `state_out.eef_frame` rather than compensated for here: silently adding an
  offset for a setting we cannot confirm is worse than not adding one.

If any channel is unreadable, **nothing is published** rather than padded with
zeros: zero radians is "arm straight out" and zero closure is "gripper open",
both plausible-looking readings a policy has no way to tell apart from the truth.
The reason lands in `info()` under `state_out.error`.

### Six-axis force aborts the stream

RM75-6F-V has an integrated six-axis force/torque sensor, so step 7 of the sink
chain is real on this arm. Over any per-axis threshold, the sink **aborts and
latches**: the next command after hitting something is precisely the one that
must not run. The only way out is the `servo` card's `reset` action — `resume`
does not clear a latch.

**It reads `zero_force_data` (external force, with the tool's weight
compensated), never `force_data`.** The raw vector includes the gripper's own
weight, so a threshold on it trips on gravity in some poses and never trips in
others, and both look like "the threshold needs tuning".

Two preconditions, and the driver refuses to present a protection it cannot
trust:

- **The sensor must be zeroed with the arm hanging free, and the payload
  configured.** The driver deliberately does **not** call
  `rm_clear_force_data` at start: zeroing while the arm is already touching
  something bakes that contact in as zero, after which a real collision reads as
  nothing.
- If `zero_force_data` comes back as exactly zero on all six axes, that cannot
  be told apart from an unwired or unpopulated sensor, so the protection is
  **not enabled** and `info()` says why under `force_torque.note`.

**The default thresholds (50 N per force axis, 5 Nm per torque axis) are a
guess and have not been checked against this hardware.** The vendor rates the
sensor at 200 N / 8 Nm. They are deliberately loose: too tight makes the arm
abort constantly, which is how a protection gets switched off for good. To set
them properly, with the arm zeroed and hanging free, run the motion you intend
at reduced speed, read `info()["force_torque"]["last_reading"]` throughout to
see the quiescent envelope, and set each axis above that envelope and below the
force you are willing to let the arm apply. `servo.force_torque: false` turns
it off entirely, and a 6-list `[Fx, Fy, Fz, Mx, My, Mz]` sets each axis.

## Upper-computer RealSense camera

The camera is connected to the Linux upper computer that runs this Driver,
not to the RM75 controller. Camera capture and arm control therefore have
independent data paths:

```text
ext_camera -> pyrealsense2 -> upper-computer RealSense USB camera
joint_control -> RealMan API2 -> RM75 controller TCP port 8080
```

`ext_camera` is a multi-instance sensor. Configure each card with an
`instance_id`, a RealSense selected by serial number, and one `channel`:

| channel | topic | format |
| --- | --- | --- |
| `rgb` | `/{namespace}/ext_camera/{instance_id}/rgb` | `image/jpeg` |
| `depth` | `/{namespace}/ext_camera/{instance_id}/depth` | `image/depth-zlib` |
| `infrared` | `/{namespace}/ext_camera/{instance_id}/infrared` | `image/jpeg` |

Hyphens in `instance_id` become underscores in the ROS topic. Discovery and
capture use `pyrealsense2`; the selected camera is bound by its stable serial
number rather than a dynamic `/dev/videoN` path in Canvas. The target Linux
ARM64 wheel uses its V4L2 backend internally, so deployment must expose the
current video nodes as described below. Existing single-camera
Canvas projects that saved an old `/dev/videoN` selection are migrated
automatically when they next start.

Depth is 640x480 little-endian uint16 millimetres compressed with zlib. Zero
is invalid/unrepresentable depth. Infrared is the left Y8 stream rendered as a
grayscale JPEG; it is reflected near-infrared intensity, not temperature.
RGB, depth and infrared instances on one camera share one SDK pipeline. The
pipeline captures each physical stream once and fans it out to every card that
selected that channel, so three cards do not compete for the D435.

The image pins `pyrealsense2==2.56.5.9235`, matching the repository's verified
Linux ARM64 / Python 3.10 runtime. The image does not install the `v4l-utils`
command-line package. Do not run `realsense-viewer` or another capture process
against the same D435 while the card is active.

Deployment binds `/dev:/dev:ro` and grants character-device read/write access
for V4L2 major 81 and USB major 189, including newly allocated minor numbers.
No `/dev/videoN` or USB bus address is configured. Docker can start without a
camera; later connections and node renumbering are visible through the directory
bind. The service does not use privileged mode and drops MKNOD. The read-only
mount protects directory entries; device I/O is still read/write under cgroup
rules. A separate writable, container-private 128 MiB tmpfs is mounted at
`/dev/shm` for Python multiprocessing semaphores and DDS shared memory. Without
this override, a read-only `/dev` can cause camera start to fail with
`[Errno 30] Read-only file system` before the capture process starts.
This deliberately exposes host device names and permits access to all
video/USB devices, not just one camera; the SDK selection remains serial-bound.

Both Web Console and `deploy/run-pr-image.sh` use this same service fragment.
Redeploy the new image once to replace older exact-node mappings; no per-node
environment variables or host scanning script are needed afterwards.

Each tools-list request refreshes camera discovery, so a camera connected after
Driver startup can appear in its configuration schema. Existing active cards
keep their serial number: after a disconnect or frame timeout the worker closes
its pipeline, waits two seconds, then recreates the SDK context and retries the
same camera. It never substitutes another connected serial. Errors remain
visible and `fresh` stays false until new frames arrive. Stopping the last card
cancels retries. Frame counters restart when a new capture session is opened.
A UI which caches tool schemas may need its device/tool list refreshed.

The observed target hardware is an Intel RealSense D435 (USB ID `8086:0b07`).
On USB 3, the shared pipeline uses RGB 1280x720 and depth/infrared 640x480 at
15 fps. A USB 2 connection falls back to 640x480 at 6 fps for all streams.

For a supervised hardware check, create separate RGB, depth and infrared card
instances, start them, then confirm that `frames_published` increases and the
three topics render independently. USB disconnect becomes an explicit error until the same camera reconnects.

The deployment enables its motion capability, and every `set` call must still
include `confirm_motion=true`. `joint1_deg` through `joint7_deg` are absolute
targets in degrees. An omitted joint defaults to its measured position at the
start of the request, while the supplied targets are sent together as one API2
`movej` trajectory so the controller plans all joints concurrently. The
Driver rejects non-finite values, targets outside the
official RM75 limits, speed above 10 percent, disabled
joints, and any reported arm or joint error. It sends non-blocking API2
`rm_movej`, monitors the measured joints until they reach the target, and
registers `stopmotion` as the Agent Core `on_interrupt_motion` hook so it
bypasses an active ACP barrier. The card does not expose a
fixed `timeout_seconds`: after a 2-second startup grace period, the driver asks
for a controlled slow stop and reports `motion_stalled` through ACP if the
maximum joint error has not improved by at least 0.05 degrees for 10 seconds.
It also derives an internal deadline from commanded distance and speed, capped
at 300 seconds, as a final safeguard.

The first supervised hardware test should change exactly one joint by no more
than 1 degree at 1 percent speed. A reachable physical E-stop and a clear work
area are required. Software interlocks do not replace the robot safety system.

Physical motion and `abs_move` are enabled in the default deployment;
`abs_move` can be disabled with `RM75_CARTESIAN_ENABLED=0`. Verify the active
TCP, arrival device, and workspace envelope before use. Every
`abs_move.move` request must also set both
`cartesian_enabled=true` and `confirm_motion=true`. `move` reads the
measured TCP as A and accepts B pose fields (`x/y/z` in millimetres and `rx/ry/rz` in
degrees). Any omitted B axis keeps the measured A value, so callers
can provide only `x/y/z` to preserve the current orientation. At least one B
field is required. It validates the composed B pose against the configured
workspace envelope. `motion_mode=joint` (the default) submits `rm_movej_p` and
is intended for general repositioning or large orientation changes;
`motion_mode=linear` submits `rm_movel` and is intended for a verified straight
approach or retreat. Planning failure, an unreachable pose, collision stop,
timeout, or an operator stop ends the action and releases the motion lock.
After a controller-event timeout or cancellation, `info` may report
`controller_trajectory_state: draining`. The action is already terminal, but a
new controller trajectory is rejected until the previous untagged SDK event is
consumed; this prevents a late event from completing the next action.
The controller's collision level, electronic fences, virtual walls, and
physical safety system remain responsible for collision protection; the driver
has no environment model and does not plan a detour around obstacles.

The HTTP service listens on port `15718` and provides `/health` and `/mcp`.
The normal Agent Core runtime still initializes its ROS/DDS transport, but robot
communication itself goes directly through API2 TCP port `8080`.

Only the Python SDK wrapper is stored in Git. The operator's licensed Linux
ARM64 `libapi_c.so` must be installed on the robot host at
`/opt/realman/rm_api2/libs/linux_arm/libapi_c.so`; `service.yml` mounts that
directory read-only at the path expected by API2. The expected SHA-256 is
`5b9d236a5cf901cdf05418d9ef5815a77a8c717af0ff037e7aad9247beb76fb9`.
Verify the operator-provided file before starting the enabled Driver:

```bash
echo "5b9d236a5cf901cdf05418d9ef5815a77a8c717af0ff037e7aad9247beb76fb9  /opt/realman/rm_api2/libs/linux_arm/libapi_c.so" \
  | sha256sum -c -
```

The image can still be smoke-tested manually with `RM_DRIVER_ENABLED=0` without
the library, but the deployment service defaults to a live, motion-capable
connection. An enabled connection fails with an explicit mount error when the
library is absent. Set `RM_API2_LIB_DIR` to override the host directory.
Following the standard ACP contract in `README_dev.md`, `joint_control.set` immediately returns a
unique `action_id`; its background monitor later reports exactly one
`completed`, `error`, or `cancelled` terminal result to `/api/acp/complete`.
The callback reads `AGENT_CORE_URL` inside the worker-thread function and
verifies TLS against the Agent Core CA (see the deployment prerequisite
below); it never falls back to unverified HTTPS. Agent Core
owns pending-action barrier release and completion-event delivery.

### Agent Core CA prerequisite

Completion callbacks POST over TLS with certificate verification enabled, so
every RM75 deployment must provide the Agent Core CA on the host at

```text
/opt/phanthy-motus/data/certs/cert.pem
```

`service.yml` mounts that directory read-only and points `AGENT_CORE_CA_CERT`
at the file; override either with `RM75_CA_DIR` / `RM75_AGENT_CORE_CA_CERT`
when the host layout differs. When the CA file is missing, each asynchronous
joint/gripper/Cartesian completion callback fails with
`AGENT_CORE_CA_CERT is required` while motion itself continues — verify the
path before enabling the deployment.

The immediate card result contains only `state` and `action_id`. Completion
callbacks keep the standard status and a short reason; full final joint evidence
is available from `joint_control.info` in `last_completion`. Its `callback` is
`accepted` only when Core acknowledges the same ID, or `failed` with an error.
Acceptance confirms HTTP receipt, not pending-action matching or a visible UI
trigger. No extra `/api/event` notifications are sent. This Driver cannot repair
a stalled Core decision loop; a missing UI trigger alone is not proof of motion
or callback failure.

The component installs `python3-pip` to install the pinned camera wheels,
`python3-yaml` because the shared runtime loads `config.yaml`, and
`ros-humble-rmw-fastrtps-cpp` because the shared runtime creates the Agent Core
ROS 2 participant. The camera layer installs the ARM64 `pyrealsense2` wheel,
NumPy 1.23.5 and headless OpenCV 4.11.0.86 through pip. The wheel supplies its
own RealSense implementation but dynamically loads `libusb-1.0.so.0`. The build
downloads Ubuntu's signed `libusb-1.0-0` runtime package and extracts only that
shared object into `/opt/realman/libusb`; it does not execute or suppress package
maintainer scripts and does not claim the package is installed in dpkg.
The SDK uses V4L2 directly without `v4l-utils`; no compiler, desktop OpenCV backend or
ROS build tooling is installed. Every download/extraction step must succeed and
the apt layer must finish with an empty `dpkg --audit`; package-install failures
and partially configured package states are never accepted.
See `vendor/SOURCE.md` for provenance notes.

The RM75 API2 TCP path itself does not require privileged mode or host devices.
The bundled upper-computer camera receives only the USB bus access required by
RealSense enumeration and capture, as described above.
Skeleton publication skips disconnected/busy SDK clients, retries failed samples
at most every two seconds, and logs once per outage until a successful sample.
An already-running SDK TCP query still holds the SDK lock until it returns.
