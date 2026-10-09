# Unitree As2W driver

This bundle targets the Unitree **As2W** wheel-legged robot and its SDK service documented in the Unitree
SDK guide. It vendors Unitree's official `unitree_sdk2_python` master at
`65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5` and uses its dedicated
`unitree_sdk2py.as2.sport.SportClient`: `Move`, `StopMove`, `StandUp`, `StandDown`,
`BalanceStand`, `RecoveryStand`, `Damp`, `Euler`, `BodyHeight`, `BodyPosition`,
`SwitchGait`, `SpeedLevel`, `SwitchJoystick`, `SetAutoRecovery`, `GetState`,
`FrontFlip`, `BackFlip`, `HandStand`, and `BipedStand`.

The driver accepts an explicit robot interface from a command-line argument or
`NETWORK_INTERFACE`, for example `NETWORK_INTERFACE=eno1`. Command-line values
take priority over the environment, followed by `robot_interface` in
`config.yaml`. An empty value or `auto` enables safe discovery: the driver
selects a unique, UP, wired, non-virtual interface on Unitree's
`192.168.123.0/24` robot network. It never falls back to an arbitrary Wi-Fi or
office-network interface. If no unique robot interface can be identified, or
CycloneDDS cannot bind to the selected interface, MCP starts in degraded mode
without DDS publishers. The bundle exposes MCP on port
`15709`, publishes JSON state streams under the resolved ROS namespace, and
uses a dedicated process for RPC calls so ROS callbacks cannot starve SDK
responses.

Deployment follows agent-core's DDS isolation contract: ROS2 uses FastDDS
Domain 42 and the mounted `/opt/phanthy-motus/dds-local.xml` loopback profile;
the Unitree SDK uses CycloneDDS Domain 0 and binds to the robot interface passed
to `main.py`. These are deliberately separate DDS implementations and domains.
The image installs the small CMake toolchain because the SDK's pinned
`cyclonedds==0.10.5` Python binding must link against a matching CycloneDDS
build; the vendored CRC `.so` files are the official SDK's architecture-specific
runtime dependencies and are required on both amd64 and aarch64.

`duration=-1` starts a 10 Hz velocity command loop; `stop_move`, shutdown, and
any plugin stop path terminate that loop and issue `StopMove`. Velocity and
attitude inputs are clamped before reaching the robot. Special motions should only
be invoked with a clear area and appropriate operator approval.

The checked-in `resource/as2w.urdf` kinematic model is based on Unitree's
official `unitree_ros/robots/as2w_description`; it retains inertial and joint
limits but omits the vendor STL visual/collision meshes. The driver only needs
the kinematic chain for the `joints` skeleton card, avoiding large binary
assets in the repository. As2W has 16 movable joints (12 leg joints plus 4
continuous wheel-foot joints) and the fixed JT128 sensor mount. The live
`joints`/`joint_state` cards intentionally publish the 12 populated leg-motor
slots from LowState; the remaining vendor array slots are reserved zeros.

`controlled_spatial` is a thin adapter for Unitree's documented `slam_operate`
service: mapping, relocalization, and point-goal navigation. The latest AS2
SDK does not package a model-specific SLAM client, so the driver implements the
documented common RPC contract directly in an isolated CycloneDDS process. It
requires the vendor `unitree_slam` service to be installed and already running
on the robot or extension host; the driver does not start that service.

`slam_mapping` is the matching read-only visualization card. It listens to the
vendor mapping and relocation point-cloud topics plus `rt/slam_info`, keeps a
bounded voxel map, and publishes the shared `sensor/mapping` packet on
`/<namespace>/spatial/mapping`. The control and visualization cards are kept
separate, following the G1/Go2 card contract, so stopping the dashboard view
does not interrupt mapping or navigation.

`special_motion` exposes the AS2 SportClient's `FrontFlip`, `BackFlip`,
`HandStand`, and `BipedStand` actions. It is intentionally separate from the
continuous `loco` control card. Dangerous motions validate the current FSM
state before dispatch, while the existing asynchronous completion and
cancellation behavior is retained. Timed and continuous locomotion actions
return action IDs and report completion, cancellation, or RPC failure through
ACP; `stop_move` also returns immediately and completes through ACP so a slow
firmware RPC cannot hold the MCP request open. If AS2 reports a standing state
but rejects the first velocity command, the driver performs the required hidden
balance transition and retries. AS2 `AI_*` standing, walking, and down states
are recognized explicitly.

`loco.stop_move` confirms physical stopping from fresh SportModeState linear
velocity and `yaw_speed`, not from the selected FSM mode. Confirmation requires
at least three distinct post-command samples spanning 0.3 seconds, with linear
speed at most 0.03 m/s and yaw speed at most 0.05 rad/s. Missing, stale, invalid,
repeated or restarted telemetry cannot establish a stop. A timed move reports
an error if its final StopMove fails; an accepted final stop alone does not claim
physical confirmation. These checks preserve the robot's deployed stop fix.

High-rate LowState, BMS, and sport-state callbacks retain only their newest
sample and publish from a 60 Hz worker, preventing stale JSON work from
blocking DDS callbacks. Lidar runs in a separate OS process and uses a one-frame
queue. It probes for an already-installed CuPy CUDA runtime without downloading
one: CPU fallback is capped at 2,000 latest points, while CUDA may use the
configured 12,000-point budget. `led` controls the AS2 RGB LED through a
separate audio-service worker, so its keepalive cannot delay locomotion or
speaker PCM.

The multimedia cards use transports verified on As2W hardware. `speaker`
streams PCM-16k through the A2 `voice` service, while `camera` publishes JPEG
frames returned by the Go2-compatible `videohub` service. `mic` listens for the
official A2 multicast stream at `239.168.123.161:5555`; when the robot's voice
assistant / wake-up conversation mode is disabled, the card remains in a
diagnostic `waiting` state and automatically recovers when packets appear.
Speaker ROS subscription, jitter buffering, and the A2 audio client now live in
one spawned process. PCM therefore stays in a thread-local queue instead of
being copied through a multiprocessing queue, and a busy main ROS executor
cannot delay playback. Cold playback uses the G1/R1-proven 300 ms prefill and
300 ms `PlayStream` blocks with a 240 ms maximum queued lead. A genuine
underflow uses a separate 500 ms recovery prefill, so lowering startup latency
does not make a delayed TTS segment resume one block at a time. Internal EOF
markers produced by split TTS text preserve the current playback timeline. Its
`info` response reports first-input-to-play latency, recovery wait, input gaps,
queue drops, partial flushes, underflows, `PlayStream` failures, and RPC latency.

Camera capture and ROS publication likewise run together in a spawned process.
JPEG frames stay in a one-frame, latest-only thread queue and are assigned to
`CompressedImage.data` through a buffer-compatible byte array, avoiding both a
roughly 300 KB multiprocessing copy and slow per-byte ROS conversion. Camera
`info` reports separate capture/publish rates, videohub RPC time, message-build
and publish-call time, average frame size, and dropped stale frames. AS2W camera
`stop` closes the spawned capture/publication process and reports idle only
after it exits. A later `start` creates a new backend; repeated starts while
capture is already active reuse it without resetting frame counters. Lifecycle
requests are serialized so restart cannot race shutdown. If shutdown fails,
the card keeps the process handle and reports an error until it can stop it;
it does not create a replacement alongside the old process. This is AS2W's
explicit-stop behavior, rather than the optional always-on sensor lifecycle
described in the general driver guide.
The component uses the three-field `audio_msgs/AudioChunk` interface already
built by the [shared ROS base](https://github.com/4paradigm/phanthymotus/blob/b42effb65b395d0860e36f85ba40f6e10129e55a/deploy/ros-base/Dockerfile).
Its `/ros_ws/install` overlay supplies `header`, `format` and byte-array `data`.
The AS2W image validates those fields and native type support at build time and
sources that same overlay at runtime. It does not build or copy a second audio
interface. Custom `ROS_BASE_IMAGE` overrides must provide this shared contract.

No-hardware checks are available with `python3 test_driver.py`; they cover
action lifecycle, schemas, model resources, RPC correlation, and full-size
low-state arrays.

## Visual navigation with ActuCore

`loco_servo` accepts the shared `motus.control/1` body twist stream from
ActuCore `navi`. This enables visual-object approach using the existing VOP
and `visual_depth` cards without installing `unitree_slam`. The navigation
policy remains in the main project's ActuCore component; this driver supplies
the AS2W control adapter. `controlled_spatial` retains its vendor-SLAM role.

Connect the same front camera to VOP and Visual Depth, both processors to
`navi`, `loco_state`'s **state/odom** output to `navi`, and `navi`'s
**control/velocity** output to `loco_servo`. Use the complete depth image
(`image/depth-zlib`), not just the three-band summary. The optional original
camera input to `navi` supplies the debugging overlay. These are local visual
targets, not map coordinates or global path planning.

The default is **dry_run=true**: starting a wired card validates the stream
without sending motion. Bundle startup alone never subscribes to a control
topic. After inspecting `info`, explicitly pause, configure `dry_run=false`,
then resume for supervised low-speed commissioning. Each real start/resume
acquires the chassis, requests `StopMove`, checks the controller posture and
accepts only frames generated after the new start epoch. It never stands the
robot up or changes its gait implicitly. Use `loco.stop_move` before handing
over from a previous direct locomotion command.

Default commissioning ceilings are 0.30 m/s forward, 0.20 m/s lateral and
0.50 rad/s yaw. These are software limits, not measured AS2W capabilities.
The `plugins.loco_servo` YAML configuration accepts `vx_limit`, `vy_limit`,
`wz_limit`, `expected_hz`, `watchdog_ms`, `max_obs_age_ms`,
`linear_acceleration`, `angular_acceleration` and a measured/vendor-specified
`footprint`. Runtime canvas configuration exposes `dry_run` and `rotate_only`;
entering dry run first stops real motion. Limits are immutable for a running
driver so the descriptor cannot change underneath an existing navigator.

The ROS callback decodes JSON and replaces one raw pending frame; `QUEUED`
means admission to that mailbox, not validation or SDK acceptance. One worker
calls the shared `common.control.ControlSink` immediately before SDK writes.
The sink owns contract/freshness checks, source arbitration, velocity bounds,
step limiting, TTL/watchdog and fault latching. AS2W supplies its posture gate,
SDK conversion and chassis ownership boundary. The standard
`limits.max_delta_per_step` declares acceleration divided by `expected_hz`;
the worker sends nonzero commands no faster than that rate. A validated full
zero stops immediately instead of ramping down. Short-timeout SDK processes isolate
velocity, state queries and the fallback stop path from legacy multi-second
RPCs. A timed-out velocity process is terminated and cannot queue later
commands; an RPC already delivered to firmware cannot be retracted by Python.
The driver refuses legacy writes while the servo owns the chassis, including
late finalizers from old locomotion workers. Explicit canvas locomotion and
special-motion requests first wait for the servo's stop acknowledgement.
Vendor `controlled_spatial` navigation also reserves the chassis; only an
accepted vendor pause/shutdown releases that reservation. A timed-out vendor
request remains reserved because its execution status is unknown. Explicitly
pause the vendor navigator before switching to the visual velocity stream.

The reservation blocks legacy `loco`/posture writes as well as `loco_servo`,
including late stop finalizers. Conversely, a legacy write in flight or a
movement without an accepted stop prevents the vendor navigator from reserving
the chassis. Read-only state and the independent LED lane remain available.

The observed vendor `task_result` has no reliable goal identity. It is not
proof that the current target was reached: a delayed result can belong to an
older goal. The adapter therefore requests an explicit vendor pause on a
terminal result or navigation timeout, and releases the chassis only on that
RPC's matching success reply. It reports ACP `error` with `arrival_reported`,
`arrival_verified=false`, `stop_acknowledged` and `chassis_reserved`; it does
not report unverified arrival as `completed`. A stale event can conservatively
stop the current goal. This is an intentional limitation of this vendor
adapter, not a validation of vendor goal completion. The ActuCore visual navi
path is separate and does not use these SLAM messages.

`info.last_navigation_result` retains this result. `requires_vendor_pause`
identifies a retained reservation without an active action. Card `stop` still
attempts vendor pause in that state, even after a failed navigation RPC or an
earlier teardown; a failed pause returns an error and preserves the reservation
across restart. `pause_navigation` and `shutdown` remain the explicit recovery
paths. Local RPC request IDs prevent an old navigation reply from being mistaken
for a newer pause acknowledgement. An SDK acknowledgement is not a measured
physical stop.

Explicitly pausing an active goal saves its target. `resume_navigation` requires
that saved target and an available completion subscription, allocates a new
action ID and starts a new 180-second waiter with the same terminal-stop policy.
An untracked resume is rejected before any motion RPC; `stop` and `shutdown`
clear the saved target. Both navigation actions advertise a 210-second ACP
budget to allow the bounded terminal pause RPC to finish after the deadline.

Expired/invalid streams do not refresh the watchdog. Stream loss pauses the
card and requires an explicit resume; it never automatically replays an old
goal's velocity. SDK rejection or a failed stop latches a fault and prevents
further nonzero commands. `reset_fault` retries stopping, clears a successful
software fault and leaves the card paused; a terminated RPC process requires
a driver restart. After changing a navigation goal following a stop/arrival,
resume `loco_servo` as well as the navigator when the receiver is paused.

`info` separates received/queued/validated/simulated frames, SDK attempts, SDK
acceptances and errors. `last_outcome` and `safety_sink` expose the worker's
shared verdict and fault state. `stop_acknowledged` only means the SDK accepted the
request. `physical_stop_verified` remains unknown unless fresh body-frame
vx/vy/wz measurements confirm rest over several samples. A configured
watchdog threshold is not a guaranteed physical stopping time: scheduling,
RPC latency, braking and firmware behaviour after host/link failure must be
measured on the actual robot. The receiver itself does not detect obstacles.

`loco_state` preserves its original raw JSON output and additionally publishes
`motus.odom/1` at `/<namespace>/state/odom`. By default all motion axes are
null. After checking frame, units and signs, configure
`plugins.state.odom.frame`, `verified_axes` and `verified_on`; only the
explicitly verified axes become measurements. No global pose is inferred
from the vendor's position fields. Timestamp provenance and sample freshness
are exposed in `info`; republishing does not renew the measurement timestamp.

The camera declares `motus.camera/1` with stable ID
`unitree/as2w/camera_front`. Lens geometry remains unknown until supplied
under `plugins.camera.camera_info`; no R1 calibration is borrowed. Continue
using the existing Visual Depth model, accounting for its distance error
when selecting clearance, speed and stopping distance. Missing geometry,
partial odometry, and optical/depth error remain explicit commissioning
limitations, not implicit guarantees of obstacle clearance.

Run the navigation regressions separately from SDK-stubbing tests for other
robot models:

```bash
python3 -m unittest unitree/as2w/test_driver.py
python3 -m unittest unitree/as2w/test_loco_servo.py
python3 -m unittest unitree/as2w/test_navigation_integration.py
python3 -m unittest unitree/as2w/test_lidar_interface.py
python3 -m pytest tests/test_control_sink.py tests/test_control_sink_stream.py
```

The extra Dockerfile COPY entries package only the Python navigation adapter
and metadata helpers. They introduce no additional system/pip dependency and
preserve the existing DDS isolation and service deployment configuration.

The lidar subprocess receives the resolved robot adapter explicitly. It refuses
an empty/`auto` interface and exits if DDS binding fails, before creating sensor
subscriptions. The parent waits at most five seconds for a ready/error handshake;
ready requires DDS binding, ROS setup and at least one initialized subscription.
A startup failure/timeout or later child exit is exposed as `state=error`, not
`running`. Failed children and pipes are reaped/closed; `start` can retry and
`stop` reports whether cleanup succeeded. The no-hardware regressions follow
auto-detected `eno1` through child initialization and exercise real OS spawn,
readiness, failure, repeated stop/start and process cleanup.

The change to `common/control/sink.py` implements the shared-sink integration
requested in [PR #369](https://github.com/4paradigm/phanthymotus-driver/pull/369).
Keeping timestamp, arbitration, limit and stopping decisions in the AS2W
adapter would create a second safety implementation that could diverge from
the other drivers. These decisions therefore live in `ControlSink`; AS2W keeps
only its mailbox, SDK writer, posture gate and ownership/lifecycle boundary.

The shared sink's opt-in `strict_stream=True` adds bounded finite timestamps,
required observation age, session/sequence checks and execution deadlines for
twist streams. Its public reset can establish a zero baseline after a successful
stop; its vendor gate rechecks freshness after a slow posture query. Existing
cards do not opt in automatically: default source arbitration, step limiting
and recovery after watchdog stand-down retain their previous policy. Two fixes
do apply to default callers: exceptions from actuator/stop callbacks latch a
fault rather than leaving the sink usable, and a first application at monotonic
time zero now starts the watchdog correctly. A fresh command still resumes a
normal default watchdog stand-down; a latched callback fault requires reset.

This sink is shared by driver bundles, not only AS2W. At the pinned platform
revision below, ActuCore uses its own producer/negotiation code and does not
import `common.control` or `ControlSink`: the cross-repository dependency is
the descriptor and command contract. Driver consumers must be included in
rebuild/regression validation, and platform producers must be checked against
the changed receiver. An AS2W image build alone does not establish that
compatibility. No new pip/APT dependency, model, generated workspace or base
image is introduced by the sink changes. They update source already packaged
under `common`; this is not a claim of identical image byte size.

The completed offline coverage includes the existing default sink tests,
strict-stream and rotation tests, AS2W adapter tests, and R1, G1, Tianyi and RM75
servo regressions. The default tests cover source priority and recovery after
watchdog stand-down; a separate compatibility case preserves equal-priority
source takeover without strict mode. G1 numerical end-effector tests requiring
unavailable `pinocchio` remain skipped. These checks do not constitute builds
and runtime verification of every image in both repositories, nor hardware
validation on all robot models. Release validation must retain that distinction.

## Control stream transport and cross-repository validation

`control/velocity` is a logical canvas port format. The `motus.control/1`
command is a structured JSON object carried in `std_msgs/msg/String.data`;
it does not require a custom control ROS message. This is explicitly specified
in the [platform protocol](https://github.com/4paradigm/phanthymotus-driver/blob/8a7632fc93bac53b4e20b661cec3f7c953b23855/README_dev.md#L1528-L1556).
The actual [ActuCore navi publisher](https://github.com/4paradigm/phanthymotus/blob/b42effb65b395d0860e36f85ba40f6e10129e55a/actucore/plugins/navi/plugin.py#L992-L998)
creates a `String` publisher and [serializes the command into its data field](https://github.com/4paradigm/phanthymotus/blob/b42effb65b395d0860e36f85ba40f6e10129e55a/actucore/plugins/navi/plugin.py#L1146-L1150).
Agent Core independently [maps every `control/*` format to `String`](https://github.com/4paradigm/phanthymotus/blob/b42effb65b395d0860e36f85ba40f6e10129e55a/agent-core/src/ros2_bridge.py#L226-L242).
AS2W uses the matching `String` subscription and JSON decoder. Both endpoints
use ROS's integer-depth RELIABLE/VOLATILE defaults; producer depth 10 and
subscriber depth 1 do not change message-type or QoS compatibility.

`info.transport` exposes that contract. `connected` means the local subscription
was created, not that DDS discovery or message delivery has been verified.
During dry-run integration, check that the controller's `simulated` counter
increases and inspect `last_outcome` / `last_rejection`; a connected card with no
accepted samples is not evidence of a working stream. Actual DDS discovery and
delivery must still be checked on the final image.

For reproducible cross-repository validation, check out `phanthymotus` at
`b42effb65b395d0860e36f85ba40f6e10129e55a` and run from this driver repository:

```bash
PHANTHYMOTUS_CHECKOUT=/path/to/phanthymotus \
  python3 -m pytest tests/test_as2w_control_stream_compat.py -q
```

These four tests execute the real navi startup negotiation, policy, publisher
and JSON serialization, then the real AS2W callback and shared sink through
fake ROS endpoints in dry run. They cover nonzero command delivery, expired
commands, stale observation timestamps and incompatible descriptors. The test
requires that exact producer revision and explicitly skips when no platform
checkout is supplied; it does not copy producer code into this repository.

At that revision, cross-repository tests plus shared sink/rotation and
R1/G1/Tianyi/RM75 consumer regressions passed 231 cases; the G1 EEF module was
skipped because `pinocchio` is unavailable. The platform's ActuCore suite
passed 451 cases with 21 OpenCV render tests skipped; Agent Core's actual
ROS format resolver passed its eight tests. Agent Core project-start tests
could not be collected without `fastapi` and are not counted as passed.
These results check the protocol and consumer behavior without claiming real
DDS discovery, generated type support, final-image builds or robot validation.

## Packaging rationale and validation scope

The full diff against main also includes the latest AS2W card baseline from
PR #285. `multimedia.py`, `lidar_backend.py`, `sensor_worker.py` and
`slam_mapping.py` are imported runtime modules needed to preserve those cards.
The navigation addition packages `loco_servo.py`, `as2w_control.py`,
`odom_specs.py` and `camera_specs.py`; no model weights, tests or build workspace
are copied with them. Removing the duplicate audio overlay avoids redundant
generated libraries. Application source still contributes bytes to the image;
this is not a claim that the complete image has zero size increase.

The service fragment no longer forces `NETWORK_INTERFACE=eth0`, because the
robot interface name varies between hosts (the commissioning machine uses
`eno1`). Explicit overrides remain supported; automatic selection requires a
unique active wired interface on the Unitree subnet and fails closed otherwise.
The automatic resolver specifically requires exactly one active, nonvirtual,
nonwireless IPv4 interface on `192.168.123.0/24`. On a nonstandard robot subnet,
or a host with multiple matching adapters, set `NETWORK_INTERFACE` explicitly
to the robot-facing adapter (for example `eno1`) in the deployment environment.
An override bypasses subnet selection; verify its actual robot-network wiring
before deployment. There is no fallback to the default-route/office interface.
`driver.yaml` advertises the implemented cards, including `loco_servo`, so the
catalog matches `tools/list`. These two metadata changes add no runtime package.

Offline tests cover restartable vendor-navigation resources and completion
notifications, cancellation, servo stream validation, RPC failure/ownership,
measured-stop confirmation, camera/odom contracts, and deployment packaging.
The lifecycle regression uses fake SDK transports; it is not a vendor-SLAM
hardware result. The AS2W commissioning machine has no `unitree_slam` service,
so vendor mapping/navigation completion cannot be validated there.

The pre-navigation robot baseline has been observed publishing camera frames,
and a stationary stop-only check confirmed fresh linear/yaw samples. These
observations do not validate the new image's audio playback quality, microphone
capture, SLAM, obstacle clearance or physical navigation. The final reviewed
image still needs dry-run integration followed by supervised low-speed motion
and stop tests. Preserve concurrent robot fixes before replacing its driver.
