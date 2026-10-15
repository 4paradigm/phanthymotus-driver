# Arm control + Joints contract — review remediation, 2026-10-15

This document supersedes the previous single-card/saved-position contract.
The canvas has Arm control and Joints. Camera capture resources and `model`
remain available for explicit use; the live camera needs no canvas node.

## Public interface

```json
{"action":"move","translation_mm":[1,0,0],"speed_percent":1}
{"action":"set_joints","joint_mode":"absolute","joints_deg":[null,30,null,null,null,null],"speed_percent":1}
{"action":"set_joints","joint_mode":"relative","joints_deg":[1,0,0,0,0,0],"speed_percent":1}
{"action":"reset","speed_percent":1}
{"action":"set_gripper_opening","width_mm":20}
{"action":"stop_motion"}
{"action":"get_status"}
```

These are schema examples, not authorization for physical movements; 30 degrees
is not a recommended on-site target. Requests use `arm_control`. Absolute Pose
and Move rotation appear only after existing TCP calibration requirements pass.
Arrays may be JSON-array strings; length, finite numeric values, nullable absolute
targets, default speed/ranges and no-op handling retain their existing behavior.
Official Core retains some inactive hidden fields in requests; the driver ignores
only declared inactive fields and still validates active values strictly.

Only one gripper action is public: Set gripper, using millimetres, normally 0–70.
Open/Close and Set position are absent from schema, descriptions and completion
metadata. Direct legacy requests fail before taking control. Reset always uses
the official six-zero joint target, never a saved file or legacy configured
preset. Saved files remain untouched. Reset is a guarded joint trajectory with
unchanged gripper, not firmware reset, zero calibration or error clearing.
All actual control actions retain automatic stationary-teaching exit/hold before
target resolution; read-only operations and Stop do not take over teaching.

## Independent monitoring

`joints` is a sensor with an empty input schema, title Joints, and one output:
`/<namespace>/piper/state/joints`, `sensor/skeleton`, 10 Hz, measured radians.
It supports framework start/stop/info and shares the same receiver as control.
Starting only Joints connects a read receiver and sends no control command.
Stopping Arm control leaves the receiver, Joints and live camera available.
The whole driver owns final shutdown. Namespaces remain isolated per device.

`arm_control.topic_out` contains only the existing live image topic when enabled.
No public tool in the new bundle declares `state/parameters` or a JSON stream;
the new monitor does not publish one. Internal snapshots/Info and Get status keep
faults, mode, enable state, gripper width, limits, freshness and task results.
Legacy monitor construction retains parameter publication by default.
`model` still returns ordinary `{"urdf":"..."}` with the unchanged PiPER chain.
Current Core has no prismatic animation; fingers are static and width is queried
from feedback. No Core or artificial finger-motion changes are included.

## Completion delivery

A running request has one `action_id`, also used as its task ID. Actual completed,
error or cancelled outcomes are recorded before releasing the operation gate.
ACP delivery happens outside that gate and never repeats robot instructions.
The worker allows .15 second after publishing its MCP return for Core to register
its waiter. A request completed inside .05 second instead returns its actual
terminal result immediately, without a pending `action_id` or a spurious callback.

Callback HTTP must return JSON `ok: true` and the same `action_id`. There are at
most four attempts, each with a 2-second timeout and .25/.5/1-second backoff.
No extra POST follows a successful acknowledgment. TLS/loopback behavior remains
compatible with the existing local deployment; AGENT_CORE_CA_CERT, if configured,
is used to verify the Core. No credentials are logged.

Get status includes `snapshot.action_completions`, up to 64 records with actual
status/result plus delivery state/attempts/error. Pending deliveries are not
evicted. A callback from an older action cannot overwrite the latest action.
Records are in memory and delivery logs go to stdout; callbacks are not durable
across process replacement, and no robot action resumes after restart.

**Historical model entry boundary:** official Core release.261015.0897a8d's
`detail-panel.js` mounts skeleton renderers with the literal MCP ID `detail`.
Joints' View data stream button consequently requests `/api/mcp/detail/call`
and shows a fallback humanoid. The top Monitor resolves the real device ID and
correctly displays PiPER. The browser test reproduces and records both outcomes;
this is a known Core defect, not a passed detail-entry acceptance or a driver
model failure. Use top Monitor until the Core owner fixes device forwarding.

**Historical completion boundary:** release.261015.0897a8d acknowledges receipt even if
`mark_action_complete` did not find a pending waiter. It does not buffer an early
completion. The unchanged-Core integration test reproduces this: acknowledge
before registration, register afterwards, and sync times out. A fixed grace is
not a protocol guarantee for arbitrary response/network delay. Fully resolving
that race requires a Core acknowledgment/buffering change and remains an explicit
platform issue outside this driver-only request. An ambiguous failed HTTP retry
may also be delivered more than once to Core, which has no deduplication promise;
the robot trajectory still executes once.

Core PR #330 resolves both boundaries, including bounded early receipts,
deduplication and manual-canvas registration. The operator's current Core is
release.261015.5440953; it is unchanged by the driver-only local fix below.

## Boundary handling and rejection diagnostics

- Cartesian Move/Pose require an interior joint posture. An exact effective
  joint endpoint is rejected before takeover/preparation; posture is rechecked
  after preparation and under the send lock, before mode and target submission.
  This conservative guard is not IK, collision checking or proof of reachability.
  Use Set joints to choose a working posture; zero-angle Reset is not one.
- Command and measured-joint limits stay strict. Only an already-submitted Reset
  boundary target may observe a small endpoint excursion (within the existing
  0.5 degree arrival window) for at most 0.5 second. No additional command is
  sent during observation. Feedback must return inside every strict limit and
  then satisfy the existing 0.5 second stable-arrival test before success.
  Persistent/gross excursions, faults, stale data, lost mode/enable/permission
  and cancellation still fail. No measurement is clamped or accepted outside
  limits; the grace is not a new measurement tolerance or retry.
- A rejection or measured-limit failure never triggers another hold target,
  even if a subsequent read becomes healthy. A new task is refused while the
  controller still reports a previous trajectory in motion.
- Get status/journal distinguish pre-existing reports, setup failures and target
  rejection after submission. They retain the SDK target, first error feedback,
  measured joints/limits, actual status, submission flags and hold result.
  Stop without a live driver task reports `stop_unconfirmed` when fresh healthy
  stationary feedback cannot be confirmed, rather than claiming physical stop.
- Existing controller faults are never cleared automatically. Reset returns
  joint angles; it does not clear a latched target-limit report.

This repair adds no dependencies and changes no Core/shared runtime. Local
regression passed 449 tests plus 331 subtests; the ARM64 isolated image passed
220 unittest tests without hardware mappings. Physical acceptance remains
separate from those results. A local overlay tag is not a PR Bot Version.

## Acceptance

Local tests cover schema, one gripper width, official Reset, legacy-file
preservation, independent monitoring, stale freezing, cancellation and ACP
success/error/cancellation, failed acknowledgments, bounded retry and interleaved
results. A source snapshot of the installed official Core is used unchanged for
ACP register/sync/complete integration and real-browser rendering tests with
fake SDK feedback. No test connects to a physical robot.

ARM64 test builds are development overlays over the verified Bot dependency
image, not a clean release build or a Bot Version. On-site acceptance remains:
small relative/partial absolute joints, gripper widths, Stop during movement,
then the official zero-angle posture from reviewed starting poses. Record actual
feedback, terminal outcome, callback reception and physical observations.
The remediation and boundary repair are submitted through driver PR #382. A
registry deployment must use the actual Version reported by its Bot build and
preserve the operator's CAN identity, camera binding, DDS and Core configuration.
