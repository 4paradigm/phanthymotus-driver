# Named waypoints (first version)

`waypoints` is an actuator card on the **same** PiPER runtime and CAN connection
as Arm control. It stores discrete six-axis joint targets, requires no TCP
calibration, and does not record or play back dragged trajectories.

| Action | Parameters | Result |
| --- | --- | --- |
| `decision_control` | — | Ready the card to await decision_core instructions; no connection or motor commands |
| `capture` | `name`, `overwrite=false` | Save a stable measured posture, without motor commands |
| `list` | — | Read all stored records, including connection identity |
| `read` | `name` | Read one record |
| `move_to` | `name`, `speed_percent=1` (1–10) | Execute an absolute six-axis joint target and verify arrival |
| `delete` | `name` | Remove one record |
| `start` / `stop` / `info` | — | Card lifecycle, cancellation, readiness and last completion |
| `stop_motion` | — | Cancel the current shared arm operation without releasing motor torque |

Names support Chinese, are trimmed and Unicode NFC normalized, and contain
1–80 printable characters. They are JSON keys, never file paths. A duplicate
capture requires explicit `overwrite=true`. The initial action is a passive
selection. Start and Info never move or enable the arm.

Example MCP arguments for the `waypoints` tool:

```json
{"action":"decision_control"}
{"action":"capture","name":"取料上方"}
{"action":"list"}
{"action":"read","name":"取料上方"}
{"action":"move_to","name":"取料上方","speed_percent":1}
{"action":"delete","name":"取料上方"}
```

The menu label for `decision_control` is **智能托管（decision_core）**, matching
the local simulation card. It is a passive readiness shortcut, not an exclusive
control mode: manual actions remain available. It can ready a stopped card while
offline, does not probe CAN or verify hardware/canvas connectivity, and does not
enable motors, move, clear cancellation, modify waypoints or grant deployment
motion permission. Busy shared arm/camera operations reject this action.
The response explicitly marks hardware/canvas binding as unverified. Connect the
canvas execution binding to decision_core and enable platform intelligent control
separately; subsequent movement still enforces all existing hardware checks.

Capture requires fresh, healthy joint feedback and at least 0.5 seconds of
stable sampling (at most 0.2° change from the first sample); mode and enabled
state must remain unchanged. Feedback must advance over the capture window and
the controller must report idle before and after it. Stationary teaching mode
is allowed; capture never exits teaching, switches mode, enables or moves.
Missing/stale gripper feedback is stored as `null`; otherwise its opening,
effort, enabled state, error code and feedback time are inspection metadata.
**Move to only replays the six joints and leaves the gripper unchanged.**

Capture and Move to require a readable USB-CAN adapter serial plus the CAN
interface name. Move to refuses records from another connection, including
unknown identity. This binds the adapter/interface, not a cryptographic robot
identity: moving the same adapter to another physical arm requires recapture.
List, Read, Delete and Info work offline and do not connect CAN.

Motion uses the existing Arm control implementation: deployment motion
permission, guarded teaching takeover/preparation, current effective limits,
speed validation, one target submission, fresh arrival feedback, cancellation,
and ACP completion. It uses joint interpolation; the path is not a guaranteed
straight Cartesian line and has no obstacle avoidance. Capture, Delete and Move
to share the operation gate with Arm control and photos; busy operations are
rejected rather than queued. Either card's Stop motion can cancel a waypoint
move; stopping the Waypoints card cancels only its own operation. Core's repeated
Start while it is busy does not clear cancellation. An async `running` response
is acceptance; use the ACP result or Info's `last_action` for the outcome.

Storage is `<data_dir>/waypoints.json`, version 1, with up to 256 records.
Each record contains `joints_deg`, `device_serial` (USB-CAN adapter serial),
`can_interface`, `saved_at`, `feedback_time`, `gripper`, and
`gripper_replayed=false`. Writes use a process lock, temporary file, file fsync,
and atomic replacement. Waiting for a writer is cancellable and bounded to two
seconds; disk writes do not hold the runtime lifecycle lock. Just before replacement,
the driver rechecks cancellation, connection identity, healthy fresh feedback,
mode, enabled state and posture, so a delayed write cannot commit a cancelled or
changed capture. Invalid or unsupported libraries are rejected and
preserved; write failure before replacement preserves the previous library.
Persist `data_dir` across container replacement using the existing data volume.
`return-preset.json` and Reset configuration are never read or modified by this
card. Deleting a point does not change Reset.

## Verification scope

`tests/test_piper_waypoints.py` checks persistence, connection binding, stable
capture, invalid/corrupt data, failed writes, concurrent writers, passive
lifecycle, permission/limits, target execution/timeout, shared locking, async
completion and cancellation with a recording SDK. Existing unified control,
completion and deployment tests also cover the reused movement primitives.
Dockerfile tests verify both source manifests include the new modules.
These are software checks; supervised physical path and arrival acceptance
remain pending. No real robot movement or live deployment was performed.
