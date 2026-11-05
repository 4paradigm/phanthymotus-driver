# Tianyi 2.0 Pro Driver

Phanthy Motus driver bundle for the Tianyi 2.0 Pro humanoid robot. The driver
bridges robot-side ROS2 topics on domain 0 to Agent Core topics on domain 42 and
exposes the capabilities as MCP tools.

## Hardware depth for visual navigation

`camera_depth` forwards the newest Orbbec frame from
`/ob_camera_head/depth/image_raw` on domain 0 to
`/{namespace}/camera/head/depth/visual_depth` on domain 42. The `/visual_depth`
suffix lets older Navi versions identify depth before ROS topic discovery;
it does not mean a monocular model is involved. Following upstream driver
PR #419, its output is `image/depth-zlib` on `sensor_msgs/CompressedImage`,
with `format = "16UC1; compressedDepth zlib"`. The payload is a plain zlib
stream containing exactly 640x480 little-endian uint16 millimetres; zero
remains invalid depth. Compression preserves every original depth value and the
source header. This replaces the previous raw `image/depth-z16` output and
requires a consumer/bridge that supports `image/depth-zlib`.

The default rate is 15Hz (previously 8Hz), configurable with
`plugins.camera_depth.hz` up to 30Hz. The source camera and available CPU/network
still determine the actual rate. Both source and output retain only the newest
frame; a frame waiting locally for 500ms is discarded. A single ROS timer
has been replaced by one latest-only worker: the ROS callback only swaps the
pending frame, while crop, zlib compression, and the synchronous bridge write
run outside both executors. The worker never replays old frames or catches up
in a burst. Compression uses libdeflate level 1 (still a standard zlib stream)
instead of CPython's stock encoder on ARM. The compressed bytes are assigned
to the ROS `uint8[]` using `array('B')`, which uses Humble's bulk buffer setter
instead of validating every byte in Python. This preserves the exact compressed
payload, source header and decoded depth pixels. `info` reports the
active compression backend, target/average Hz,
coalesced/stale/error counts, worker health, payload size, and
transform/compress/message-pack/bridge/total processing and interval timings so
compression and ROS message packing can be measured separately on the robot.
The existing 500ms local stale-frame checks still apply.

On the canvas, connect head RGB to `vop`, `vop` to `navi`, `camera_depth` to
`navi`'s depth input, `loco_state` to its odometry input, and `navi` to
`loco_servo`. Disconnect the `visual_depth` depth input when selecting hardware
depth. After redeploying, refresh the card's port metadata and reconnect the
depth wire to the new `/depth/visual_depth` topic with `image/depth-zlib`.
No `lidar_depth` or `visual_depth` processing card is needed in this path.

`camera_head` reads `/ob_camera_head/color/camera_info` and declares its live RGB
intrinsics and distortion alongside the actual published image dimensions.
With `plugins.camera.require_camera_info: true`, startup waits up to 10 seconds
for both a JPEG and matching calibration before Core snapshots this geometry.
Unavailable or incompatible calibration is never replaced by guessed values.

The factory `head_330_ty.launch.py` profile uses `depth_registration=true`,
`align_mode=HW`, `align_target_stream=COLOR`, with matching 1280x720 profiles.
For that profile the bundle explicitly sets
`plugins.camera_depth.alignment: vendor_hw_d2c`. Depth retains the full source
view and is resized with nearest-neighbour sampling to 640x480; it is no longer
cropped independently of RGB. Its declaration uses the registration target's
live RGB calibration and identity, with the two rows of K scaled independently
for the output width and height. The zlib format, millimetre samples, source
header, 15Hz target, 500ms checks and bulk `array('B')` packing stay unchanged.
Some older Orbbec wrappers still publish the native depth K even when D2C is
enabled, so this configured mode does not use that K as the aligned projection.

`camera_depth.info` reports the configured registration source and whether a
matching calibration is ready. Hardware D2C is a vendor configuration claim,
not a pixel-level test: `pixel_alignment_verified` remains false. A missing
calibration, wrong frame ID or mismatched source size does not produce a false
RGB geometry declaration. For a different camera setup without confirmed HW
D2C, select `alignment: native`; that retains the original native-depth crop
and reports registration as unknown. The host camera startup script is not
changed by this geometry setting.

After redeploying, restart the connected camera/VOP/depth/Navi cards so Core
takes a fresh metadata snapshot. Verify a stationary doorway's RGB and depth
edges, centrally and off-centre, with the servo paused, then compare reported
distance against tape measurements before testing passage. Software tests
verify projection bookkeeping and wire bytes, not physical alignment or the
chassis clearance. The declared conservative 0.70m envelope plus Navi's default
0.15m margin on each side requests a 1.00m free corridor.

## Locomotion state card

`loco_state` starts with the driver and publishes JSON on
`/{namespace}/loco/state` and the same poll as `motus.odom/1` on
`/{namespace}/state/odom` (format `state/odom`). Its input schema is an empty
object. The namespace comes from `ros_namespace` or the hostname.
`nav_state` remains available with its existing map-localized pose output.

The new card uses its own read-only, persistent HTTP transport in
`loco_state.py`, leaving `nav_client.py` and existing cards unchanged. It reads
the chassis address from `SLAMTEC_URL` or `slamtec.base_url`. Each polling lane
owns its keep-alive connection; a failed or server-closed connection is
discarded and recreated on the next request.

| Lane | Default cadence | Reading | Endpoint | Meaning |
| --- | --- | --- | --- | --- |
| Fast | 20Hz target | Pose | `/api/core/slam/v1/localization/odopose` | Odometry-frame position and roll/pitch/yaw, distinct from map localization |
| Fast | 20Hz target | Velocity | `/api/core/motion/v1/speed` | Measured body-frame `vx`, `vy`, `omega`; unmeasured vertical velocity is null |
| Slow | 5Hz | IMU | `/api/core/slam/v1/imu` | Chassis IMU; only fields enabled in `availibilityBitMap` are forwarded |
| Slow | 1Hz | Total mileage | `/api/core/statistics/v1/odometry` | Vendor-reported total travelled distance in metres; not displacement from a task origin |

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

`plugins.loco_state.hz` defaults to a 20Hz fast-lane target (maximum 50Hz),
`imu_hz` to 5Hz, `length_hz` to 1Hz, and `request_timeout` to 0.2 seconds per
request. HTTP latency can lower a lane's actual rate, but slow IMU or mileage
requests cannot block pose/speed publication. MCP `stop` returns the canvas
lifecycle marker `{"state": "idle"}` while leaving this always-active observer
running, matching R1; driver shutdown stops both workers and their connections.

The standard output uses `common.odom` from the official driver repository:
`twist = [vx, vy, null, null, null, omega]`, `frame = "body"`, SI units,
and a host receive timestamp for the speed response. Partial speed responses
retain each valid axis, including measured zero. Invalid or absent axes are
null. There is one speed observation per publish, with no faster stream being
discarded. The IMU boot-relative timestamp is never used as `stamp_ms`.

`info()` includes a validated `odom_interface` with `provides = ["vx", "vy", "wz"]`,
the fast target rate, and `pose_drift = "unbounded"`: the source is uncorrected
wheel odometry, not map localization. Its optional timing declaration is
`{"stamp_source": "received"}` because the REST replies do not carry a usable
measurement clock. It deliberately omits `max_measurement_delay_ms`: the HTTP
timeout is a failure bound, not a measured sensor delay. Standard `pose` now
contains measured position and an orientation quaternion derived from the same
endpoint's roll/pitch/yaw:

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
Configured operating caps default to 0.3 m/s and 1.0 rad/s, not vendor maximum
ratings. Non-finite values, unsupported axes and out-of-range commands are
rejected. `ControlSink` checks the schema, TTL, observation age, sequence and
source priority. Rejected/stale messages never feed the watchdog.

The control descriptor declares the operator-requested conservative footprint
as `half_width: 0.35` m (`0.70` m overall, source `estimate`). The physical
arms-rest width was measured as `0.55` m; the larger declaration must not be
reported as a measurement. Navi owns its separate dynamic clearance margin;
with the default 0.15 m on each side the displayed avoidance corridor is 1.00 m
wide. A 0.70 m footprint declaration is not a 0.70 m passage-width setting.

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

This SDK-backed path intentionally increases only the Tianyi image by the
installed native worker, its linked vendor dependencies and required notices.
The SDK archive, headers, sources and build tree are removed after compilation;
the size tradeoff is required because Tianyi exposes no usable ROS velocity
subscriber, while discrete REST actions cannot provide continuous combined
linear/angular servo control.

Bundle startup is inert. MCP/canvas `start` subscribes to the default input,
or an explicitly provided `input_topic` / first `input_topics` entry. `start`
also accepts the settings advertised by `config`, including `dry_run: false`;
they are checked and applied before the SDK connection opens. Omitted settings
retain deployment or previously configured values. Form defaults reflect those
current settings, and a successful start preserves them for a later stop/start.
An SDK connection failure is returned as an error, without silently succeeding
in dry-run. Starting an already paused/yielded session does not resume it.

**New cards default to dry-run.** Dry-run does not launch/connect the SDK worker or
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
counts successful backend velocity/stop calls (including a stop that was already
acknowledged idle), not network RPCs or measured physical motion.
The backend state/action ID describe the last acknowledgement, not a substitute
for odometry feedback; verify physical motion with `loco_state`.

`wz_limit` (rad/s, default = `max_wz`) clamps admitted non-zero turn commands
before the shared step clamp; explicit all-zero commands are never shaped.
This runtime setting takes effect
without stopping. `last_command_timing.turn` shows `requested_wz`, `wz_limit`
and `shaped_wz`. Angular velocity follows navi's requested turn subject to these
normal limits. Delay compensation and executed-turn integration have been
removed; legacy `yaw_latency_gain` values in saved configuration are ignored
and cannot reactivate them. The coefficient is absent from public card options.

The control descriptor may also carry per-axis response metadata such as
`response.wz = {gain, delay_s, settling_s, source}`. Those values are robot
properties for a planner to consume, not operator tuning knobs. Tianyi leaves
the block absent until the response has been measured or obtained from a
traceable vendor specification; it does not publish guessed timing values.

Input TTL and observation age (600ms) are admission checks: an already-stale
command is dropped and cannot resume motion or renew the watchdog. Once
admitted, a command no longer stops motion at its producer's per-frame TTL
(commonly 200ms). Its execution deadline ends 500ms after the original
ROS/bridge callback receipt; observation age does not shorten it. (With
`obs_stamp_ms + max_obs_age_ms` as an execution deadline, a stream of old
detections expired every frame and the per-frame clamp restarted from zero,
capping output at one step.) Input age/TTL failures
only drop the stale frame: they do not immediately stop a previously valid
command or renew either deadline. The motion deadline sets velocity to zero
without cancelling the held SDK action or its firmware heartbeat. A fresh
accepted frame can then update the same action directly. If no new valid frame
arrives for 1000ms, the action is cancelled and its heartbeat stops.
IPC, local queue and SDK reply time consume the original receive budgets,
never renew them. A detection older than 600ms is never admitted, however
many new packets reuse it. `control_interface.rate.watchdog_ms` reports 500 and
`max_obs_age_ms` reports 600 (admission). Deploy Python and the native
worker together; staged stopping requires the worker's advertised capability.
Python uses the original monotonic receipt time for its silence watchdog; the
native layer receives the remaining bounded execution deadline and independently
sets zero at the motion deadline and cancels at the one-second silence deadline
(with a 20ms polling interval while SDK calls are responsive), even if ROS stops
spinning. It never holds a queue of old velocity commands or reconnects/replays
a failed request. A blocked SDK reply times out (default 250ms), kills the worker
and pauses the card. While the worker owns a motion action it enables the
firmware's one-second heartbeat, including during a soft zero until the bounded
cancel deadline. This is a **seconds-resolution backup**, not a guarantee of an exact one-second stop
after a process crash or network failure. Firmware heartbeat behavior must be
checked on hardware. On an ambiguous failure, handover waits 1.5 seconds for
that firmware lease before another card is permitted to act. A faulted worker
requires explicit stop/start; it does not automatically regain control.

An already-confirmed idle backend does not send a second STOP RPC: this requires
a live worker, no fault, no armed motion, state `ready` and action ID zero. Motion,
unknown state, a dead worker and fault/firmware handover still follow the normal
stop and error paths. Fresh motion uses the same velocity action across short
soft stops; explicit stop, cancellation timeout or failure ends that action.
Forward and angular speeds are submitted together.

An input already past its TTL is dropped without changing its timestamps or
replaying it. A new staged-follow request that has already expired reports
`STALE` and does not renew or cancel the existing session; its original timers
continue to run. Legacy `SET`/`UPDATE` retain their confirmed-stop `EXPIRED`
semantics. A failed stop, transport/ownership error, or a cancellation lease
over 1000ms still follows the fatal pause/firmware-handover path.
`backend.expired_commands`, `last_expired_error`, and `last_write` distinguish
expiry from faults and expose the lease remaining before IPC and the latest
write round-trip time. These timings diagnose latency; they do not establish
the physical stopping time. Expiry and excessive-lease errors are now separate,
with remaining milliseconds reported for the native ingress check.

`backend.native` contains the latest ACK snapshot, not live chassis feedback:
`sequence`, `status` and `stats`. Native `stats.last_set` separates busy checks,
heartbeat start, action creation, ownership verification and velocity-write
times, plus the lease remaining before/after the write. Action start and
confirmed-stop counters, heartbeat refresh counts/times and a fixed stop reason
help distinguish SDK delay from action restarts. Timings include the full SDK
call and do not assume each SDK method is a network round trip. Missing or
malformed optional diagnostics do not change ACK/ERROR handling and never
extend the lease; older workers remain compatible.

The 20Hz command cap compares the starts of SDK applications, not the time
since the previous SDK acknowledgement. A regular 10Hz stream must not lose
every other command just because a vendor call took 70ms of its 100ms period.
When transport bunches arrivals closer than 50ms, the newest motion stays in
the bounded inbox until the next permitted apply slot instead of being thrown
away. Explicit zero always bypasses that pacing. Input TTL freshness,
observation-age limits and the 500ms receive watchdog remain enforced.

The ROS/MCP facade now runs the existing velocity controller in a separate
Python process (`loco_servo_process.py`), rather than in the camera bundle's
Python interpreter. The facade sends original input and receive timestamps over
a nonblocking Unix datagram channel, retaining a priority zero and the latest
motion under backpressure. Lifecycle RPC uses a separate channel; waiting for an
`info` response does not block command reception or SDK execution in the child.
The child owns the shared ControlSink, declared velocity-step limits, watchdog and SDK
backend. Its dedicated control thread consumes the latest command and performs
synchronous SDK calls. The event wakes on arrivals;
an independent maximum-20ms wait bounds watchdog, standing and deferred-apply
checks, without periodically replaying old commands. A separate priority zero slot clears
older pending motion and cannot be overwritten by a later nonzero command;
at most two messages are retained. Intermediate motion frames are coalesced,
not queued in arrival order. Original timestamps/TTLs remain unchanged, so a
frame that has expired while waiting is rejected rather than retimestamped or
replayed. Pause, stop, configuration changes and restart clear pending input;
old-generation receivers and workers cannot deliver motion to a new session.
The process starts only on card start. A session nonce, lifecycle epoch and
subscription generation reject input queued before pause/resume/config/restart.
Standing feedback must be received again after each lifecycle barrier, so a
queued old Bool cannot authorize new motion. Even an idempotent start/config
confirms STOP before clearing pending input. Stop confirms the chassis hold,
joins the controller thread and reaps both child and native helper. Parent EOF
also closes input and stops the child. Unexpected transport/child failure closes
admission and terminates the owned process group; unconfirmed termination or an
unexpired firmware backup wait blocks handover/restart. No automatic restart or
command replay occurs.

When the previous motion lease has expired, a capable native worker accepts
the next fully validated fresh motion in one `UPDATE` request. It confirms any
expired old action's stop, rechecks the new command's original deadline, then
starts/updates velocity before returning one ACK. The driver synchronizes the
shared sink's baseline to zero without an extra STOP/ACK round trip. This
reduces recovery delay; it does not keep expired motion alive or remove the
physical stop required by an expired lease. A failed stop remains fatal, and a
new deadline consumed by stopping never starts a new action. Ordinary `SET`,
zero, pause, configuration changes and explicit handover keep their normal paths.

The capability is negotiated only by a valid initial idle handshake, exposed as
`backend.capabilities.atomic_expiry_update` and `dispatch.expiry_update=native_atomic`.
Older workers remain on `stop_then_set`: after a synchronous confirmed stop,
the controller checks the inbox once more for a priority zero or newer motion
from the same source and priority. Replacement frames still pass full validation
and arbitration with unchanged timestamps. Other sources, priorities and
duplicate/regressing sequence numbers stay queued for normal processing.

`input_queue` reports bridge-delivered `received`, control-selected `processed`,
`coalesced`, lifecycle-`cleared`, `pending`, the latest receive gap and inbox
wait in milliseconds. Existing `counts.received/accepted/dropped` describe the
selected frames examined by the control sink, not intermediate coalesced
frames. `soft_holds` counts local zero-speed holds that retain the action;
`holds` counts hard-stop paths, not distinct physical stops. Native
`soft_stops` and `confirmed_stops` distinguish zero-speed holds from confirmed
action cancellation. During a short gap, `action_id` should stay the same;
after the one-second cancellation deadline, the next motion creates a new one.
`counts.refreshed_after_stop` is retained as a legacy diagnostic field. Freshness
and arbitration now run before any compatibility stop, rather than stopping to
refresh a not-yet-admitted candidate.
`dispatch.mode=process_worker`, `worker_pid`, `worker_alive` and `worker_failed`
identify the isolated controller and expose its health. `controller_mode` is
`dedicated_worker`; `backend.process_id` identifies the native helper.
`transport` reports facade pending slots and coalescing; `input_queue` reports
the child's inbox. Neither counter acknowledges an SDK write.
`last_command_timing`
reports the selected command sequence/TTL, command and observation ages,
remaining lease before/after apply, and apply round-trip duration. `receive_age_ms`
measures generation-to-driver-callback age; `local_wait_ms` measures callback-to-
submission delay, including any synchronous stop before submission. These
measurements are tied to that selected frame, even if a newer one has arrived.
`dispatch.execution_deadline_basis` and the same field in `last_command_timing`
are `receive_watchdog`; `receive_watchdog_ms` is 500. `ttl_ms`
remains the producer's input freshness limit and is not the execution lease.
Continuous
publication alone cannot guarantee continuous motion: expired observations,
receive-watchdog gaps, stale input, explicit zeros, invalid commands and SDK faults still stop or refuse
motion. These timings distinguish those cases from local queue delay.

Nominal acceleration settings default to 0.3 m/s² and 2.0 rad/s². At the
declared 10Hz cadence they become 0.03 m/s and 0.2 rad/s per accepted command in
`limits.max_delta_per_step`; pinned axes have positive tiny step values.
ControlSink is the only step limiter, and its `CLAMPED` verdict reports the
result. There is no second ramp in the SDK callback. Like R1, this is a fixed
per-command step contract, not a strict elapsed-time acceleration guarantee at
a different cadence. The first command is also step-limited from a zero
baseline. `limits.zero_immediate=true` makes a requested zero on either active
axis take effect immediately, including stopping forward motion while still
turning. Pause, stop, lease expiration and watchdog stop also bypass shaping.
Cancelling an SDK action
addresses only this worker's held action handle. Before beginning or updating
motion, the worker checks for another active action or replacement of its own
handle; it refuses to overwrite that action. Raw velocity streaming does not
add obstacle avoidance.

If a native `STALE` reply explicitly confirms that the SDK has already zeroed
velocity, the driver also rebases the shared sink to zero. A stale packet alone
does not change that baseline or renew any accepted deadline. This prevents the
next valid frame from jumping from a remembered old speed after a real stop.

Like R1's per-frame velocity execution, continuous fresh input updates the held
SDK velocity action. A nonzero requested target that passes through exactly
zero during a step-limited reversal sends a zero-velocity update,
without cancelling and recreating that action. The staged `FOLLOW` operation
carries the intermediate zero while retaining the original bounded deadlines.
Legacy workers with the `ZERO` capability can also retain an existing action
across a ramp crossing; older workers safely fall back to stopping. Neither
zero path creates an action while idle. An explicit all-zero input during a
live lease updates the held SDK action to zero without cancelling it, so a
following fresh command can update the same action. A zero received while idle
is a no-op. Lifecycle stop/pause/handover still cancels immediately; the 500ms
receive watchdog first commands zero and the one-second cancellation deadline
then cancels a silent stream. The 600ms observation admission limit still
rejects stale commands.

Configure loco_servo exposes live settings via `action: config`:

| Setting | Default | Effect |
| --- | --- | --- |
| `dry_run` | true | Validate, shape and count commands without SDK motion; enabling it during real execution first stops/cancels the old SDK action |
| `rotate_only` | false | Suppress translation and preserve yaw rate; unsupported axes remain rejected |
| `require_standing` | false | Require fresh true feedback from a configured domain-0 Bool source; missing/stale/false feedback stops/refuses motion |
| `speed_preset` | normal | Forward-speed limit: `slow` = 0.15, `normal` = 0.30, `fast` = 0.50 m/s; yaw is unchanged |
| `min_vx` | 0 | Measured minimum stable forward speed (m/s), advertised only as `control_interface.limits.min_magnitude[0]`; 0 declares no floor |
| `linear_acceleration` | 0.3 | Nominal m/s² at expected_hz, converted to a shared per-command velocity step |
| `angular_acceleration` | 2.0 | Nominal rad/s² at expected_hz, converted to a shared per-command angular-velocity step |

Changing a speed preset or acceleration setting stops the old motion before
applying the new setting. The next fresh command starts under the new setting.
The preset changes the advertised and enforced `vx` limit; it never silently
multiplies Navi's command and never changes `wz`. Restart Navi after changing
the preset so it snapshots the new downstream limit. `info` reports
`speed_preset`, `vx_limit`, both acceleration settings and the existing
`acceleration_limits` block; a control-process restart preserves settings
within the running Driver. Persist custom settings in the card configuration
or deployment YAML for container recreation.

The 0.50 m/s fast preset is an operational ceiling, not a claim about the
vendor's maximum speed. First verify it with dry-run, then in an open area with
an operator ready to stop the chassis. Use `normal` again if tracking or braking
distance is not acceptable.

`min_vx` must be measured on the chassis before setting it, and cannot exceed
`max_vx`. The servo never raises small input velocities to this value. Existing
Navi can read this planner metadata after it is restarted; Navi code and arrival
logic are unchanged. With the default 0, this change cannot promise to resolve a
goal stalled just outside its distance threshold. Arrival completion/tolerance
requires the planner or a measured minimum-speed declaration, not an invented
floor applied blindly to all motion (including obstacle slowdowns).

Tianyi has no verified native standing FSM in the supplied API. Do not enable
`require_standing` without a verified `standing_topic`. Feedback expires after
`standing_timeout` (default one second). These source parameters and SDK endpoint
parameters live in `plugins.loco_servo` in the deployment YAML.

`pause` stops but retains the subscription; `resume` accepts newly generated
commands. Start/resume/config changes drop pre-transition commands. `stop`
returns idle and releases nodes/worker only once the stop is confirmed (or the
failed worker's firmware lease wait has elapsed); otherwise it reports an error
and remains paused. Starting after stop creates a new worker with no replay.

At the bundle dispatch boundary, a running `loco_servo` session reserves the
chassis for its incoming velocity stream (normally ActuCore `navi`). Waiting
for commands, dry-run and temporary watchdog holds retain this reservation.
`info.chassis_reserved` reports this local reservation; it does not confirm SDK
ownership. Ordinary `nav.move_to/move_by/rotate/rotate_to`, `home.go_home` and
`chassis_raw.move/rotate` requests return an occupancy error without pausing the
stream or writing zero velocity. Pause or stop `loco_servo` explicitly before
switching to those cards. Driver `nav` is a separate REST navigation card from
ActuCore `navi`, which publishes velocity for `loco_servo`.

`nav.cancel`, `home.cancel` and `chassis_raw.brake` bypass the priority rejection
and retain the existing stop handshake. An unconfirmed SDK stop still blocks
handover. Successful handover cancels the owned velocity action and disables its
heartbeat before the other card runs; the yielded stream emits no more motion,
zero or heartbeat writes. Starting/resuming after call-shaped motion still
requires explicit `nav.cancel`, `home.cancel` or `chassis_raw.brake`. Existing
card implementations, `nav_client.py` and `loco_state` are unchanged. Controllers
bypassing the bundle need separate coordination; the SDK firmware heartbeat is
global, with only one active token.

If the first velocity command finds a foreign SDK chassis action, the native
worker reports a recoverable `BUSY` hold. It does not start a heartbeat, create
an action, write velocity, cancel or preempt the foreign owner, and the card
remains running. Each later **fresh** navi frame may retry; once the chassis is
idle, that frame creates the velocity action without requiring a card restart.
No old command is replayed after the owner releases the chassis. A foreign
action held for several seconds may still make navi's independent blocked-motion
detector end its task; explicit `nav.cancel`, `home.cancel` or
`chassis_raw.brake` remains the immediate handover path. Ownership loss after
loco_servo has created its own action remains a fatal fault requiring restart.

SDK ownership failures distinguish an empty held handle, an empty current action,
and different action IDs. Error-path diagnostics include available handle IDs,
statuses and reasons. These diagnostics do not relax ownership checks or add
SDK reads to successful velocity updates. An empty current action alone does
not establish that another controller took over; inspect the recorded details.

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
