# Single-card control implementation, 2026-10-14

User requirements: one Arm control card using RealMan RM75's shared connection,
mutually exclusive task ownership, asynchronous completion and interruptible
submission structure. PiPER official SDK remains the hardware planner; no custom
IK, SDK replacement, teaching card, recording or replay is introduced.

## Public requests

All requests use `arm_control` and the `action` parameter. Examples below are
interface examples; they do not authorize or imply a hardware trial.

```json
{"action":"move","translation_mm":[1,0,0],"rotation_deg":[0,0,0],"speed_percent":1}
{"action":"pose","pose_mm_deg":[200,0,350,0,0,0],"speed_percent":1}
{"action":"set_joints","joint_mode":"relative","joints_deg":[1,0,0,0,0,0],"speed_percent":1}
{"action":"set_gripper_opening","width_mm":20}
{"action":"reset","speed_percent":1}
{"action":"stop_motion"}
{"action":"get_status"}
```

Core currently submits array fields as text, so JSON-array strings also work.
Wrong lengths, nonfinite values, boolean/string items, malformed JSON, unknown
fields and out-of-range active values are rejected. Clearing optional values
uses schema defaults; required pose/joint arrays have no default. Inactive
declared fields retained by old Core are discarded. Legacy public actions fail
with an interface-update message and are not automatically translated.

Relative translations use the initial TCP's axes. Relative rotations compose
`R_initial * R_delta`, with `R_delta=Rz(yaw)Ry(pitch)Rx(roll)`. Pose is a base-frame
absolute TCP target. Without calibration, zero-rotation Move is permitted in
axes parallel to the SDK J6/end frame, retaining the exact measured orientation.
No tool length or absolute tip coordinate is invented; arrival is checked at
the SDK reference. Nonzero rotation and absolute Pose still require calibration.
For calibrated actions, the configured full rigid TCP offset is applied before
target composition and inverted before the SDK command. Completion is checked
at the actual TCP, including tool displacement due to rotational error.

Joint absolute targets use six effective SDK/deployment limits. Relative targets
are calculated once from measured joints; the final absolute values must satisfy
the same limits. Numeric preset/absolute joint movements have no tiny-step
restriction beyond those limits, but SDK speed is limited to 1–10%.

## Operator configuration and hardware acceptance

Defaults intentionally leave the TCP and return posture unconfigured:

```yaml
intent_control:
  tcp_offset_mm_deg: null
  tcp_verified: false
  preset_joints_deg: null
```

Measure the installed gripper-tip offset relative to the SDK feedback reference,
verify axes/rotation and set `tcp_verified` only after this physical check. A
zero offset is acceptable only when the reference is actually verified as the
tip. Save the user's intended fixed return posture as six measured degrees;
do not assume zero angles correspond to a desired straight posture.

Control checks task cancellation and deployment permission before submission
and throughout preparation/gripper loops. A healthy, stationary teaching arm
uses the retained seamless-exit primitive only on recognized supported firmware.
Active recording/playback and unknown firmware are rejected. Any exit failure
blocks the requested target. Mode preparation holds measured joints; excessive
drift cancels the original target. No fault reset or motor disable is used.

After simulation, verify arm 1 on Jetson ARM64 using Docker: TCP identity and
axes, low-speed small translations/rotations, absolute Pose, six joints in both
modes, gripper, auto-preparation, teaching exit and Stop during each stage.
Configure and verify the return posture separately. Record actual feedback,
arrival/cancellation and stopping latency; prior base-frame direction acceptance
does not establish acceptance of this interface.

## Tests and packaging

`test_piper_unified.py` exercises the production bundle and recording SDK.
Historical intent/exit tests use an explicit legacy fixture so internal behavior
remains covered after public replacement. Model/monitor tests preserve the
official chain and radians, stale freezing and independent namespaces.
Both normal and ARM64 development Dockerfiles copy the new modules explicitly.

Local acceptance on 2026-10-14: 336 driver tests plus 283 subtests passed;
14 related Core tests plus 6 subtests passed. Actual Edge browser tests passed
for action-specific card inputs, strict JSON-array submission, tip Move,
relative joints, passive status, background camera/skeleton/parameters, both
model entrances and model-loading failures/races. These are fake-hardware
results, not ARM64 image build or physical acceptance.

The companion Core browser regression loads the actual detail-panel, dashboard
and skeleton modules using a local fake MCP server. It validates the six PiPER
joints, animation, both view entrances, topic-owner lookup, model failure,
invalid URDF, missing device ID, delayed responses and unmount cleanup.
The change touches only model addressing/loading and its tests in Core.

No server deployment, hardware commands, Git commits, pushes, PR updates or Bot
requests are included in this implementation turn. Preserve existing files,
backups, arm 2 and the original untracked smoke-test.py. Request submission only
when the user explicitly says to submit.


## Usability update, 2026-10-15

Absolute Pose/absolute joints accept partial JSON arrays (`null` = retain fresh
measured component), for example `joints_deg=[null,30,null,null,null,null]`.
No default posture is substituted. Relative joints/Move reject nulls. All-null
absolute arrays are invalid; all-zero relative requests are successful no-ops
without hardware preparation. The measured target is composed once and remains
subject to effective bounds, preparation drift and arrival checks.

Public tools are generated from valid configured capabilities. Uncalibrated
Move exposes only translation and speed. Pose/rotation and preset return remain
hidden until their configuration is valid. Hidden old requests fail explicitly.
`capabilities` is available through passive status and background parameters.
Pose retains the linear default; candidate point-to-point is acceptance-bound
and absent from current public definitions. See PATH_VALIDATION.md.


## Automatic teaching exit before control, 2026-10-15

Every nonzero Arm control request (Move relative, Move to pose, Set joints,
return preset and gripper actions) first checks the actual mode. In stationary
teaching it performs the existing guarded SDK exit-and-measured-joint-hold,
confirms CAN mode and stable feedback, and only then resolves the requested
movement/gripper target from post-exit feedback. Failure ends the same task;
no user target follows a failed takeover. No separate exit button is required.
This does not clear faults, reset, home, release torque, interrupt active
recording/playback, or bypass firmware/freshness/permission checks.

Core's lifecycle Start precedes Info and Stop as well as control calls, so it
remains passive. Status, monitoring, selection, zero relative requests and Stop
do not take over teaching. Actual control owns preparation, exit and execution
under one cancellable reservation. A repeated action already in CAN skips exit.


## Set position and Reset, 2026-10-15

`set_position` (**Set position**) first performs the same automatic teaching
exit/hold as control actions, then captures the stationary six measured joint
angles for 0.5 seconds (maximum drift 0.2 degree). It atomically replaces
`data_dir/return-preset.json`, bound to this USB-CAN serial/interface. No return
trajectory or gripper command is sent. Failed exit, stale/moving feedback,
cancellation, missing adapter identity or write failure retains the old preset.
The operator must re-save after replacing the physical arm on the same adapter.

`reset` is displayed as **Reset** when a valid saved/configured target exists.
It exits teaching/holds first, then returns all six joints via SDK MOVE_J at
1% by default (range 1–10%). The gripper remains unchanged. This is a planned
movement, not ResetPiper, fault clearing, zero calibration or motor disable.
The return path is a joint-space trajectory, not a straight TCP segment.
Effective limits, fresh feedback, cancellation, task serialization and stable
arrival checks still apply. The saved target survives driver/container restarts.
Invalid/mismatched stored data hides Reset and never falls back to an unrelated
configuration preset. Get status/background parameters show the saved position.
Refresh device/tool metadata after first save to make Reset visible in Core.
