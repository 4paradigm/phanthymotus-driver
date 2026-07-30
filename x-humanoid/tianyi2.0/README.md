# Tianyi 2.0 Pro Driver

Phanthy Motus driver bundle for the Tianyi 2.0 Pro humanoid robot. The driver
bridges robot-side ROS2 topics on domain 0 to Agent Core topics on domain 42 and
exposes the capabilities as MCP tools.

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
| Actions | `wave`, `salute`, `welcome`, `raise`, `reach_forward`, `present`, `high_five`, `cheer`, `reset`, `stop` |

The right-arm poses are mirrored from the left-arm definitions and remain
inside the checked-in URDF limits. The preset poses still require low-speed,
clear-area calibration on the target robot before production use. Do not run
the raw `arm` card and `arm_gesture` concurrently because both publish to the
same controller topic.

The semantic poses use a preparation frame before the final gesture. For
`wave`, `welcome`, `salute`, and `high_five`, shoulder roll and shoulder yaw
first establish an upward elbow-flexion plane; elbow pitch can then raise the
forearm instead of leaving it horizontal in front of the torso. `wave` keeps
the elbow flexed and combines a small shoulder-yaw sweep with wrist roll; wrist
yaw remains neutral because it primarily twists the forearm around its own
axis. `salute` uses
three blended stages: shoulder lift until the upper arm is nearly horizontal,
elbow flexion to approximately 120 degrees so the forearm is nearly vertical,
then wrist yaw/pitch/roll alignment into the final salute. Wrist yaw orients
the palm around the forearm axis; it does not determine whether the forearm
itself is vertical. Intermediate stages have no dwell and hand
off at 90% of their calculated transition time to avoid stop-start motion.
The action is intentionally limited to one arm at a time to avoid interference
near the head. `welcome` fixes wrist roll at a
conservative angle to keep the palm upright and facing forward, then moves only
wrist pitch through a bounded range for the welcoming motion.
`raise` lifts the upper arm close to overhead while keeping only a moderate
elbow bend, making its silhouette distinct from `welcome`.
`reach_forward` extends one arm toward a target; `present` uses a bent-elbow,
open display pose; `high_five` holds one raised palm forward; and `cheer`
provides a second overhead silhouette with a slightly flexed elbow.

The URDF defines joint axes and limits but contains no hand palm frame and no
arm visual/collision geometry. Palm-facing wrist values are therefore
conservative starting points, not geometrically proven orientations. Calibrate
`welcome`, `salute`, `present`, and `high_five` one arm at a time at low speed
with a clear workspace. Bilateral `salute` and `high_five` are blocked; actions
such as clapping, hugging, and crossing arms are intentionally not provided
until collision geometry or a separate collision checker is available.

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
| `wave` | right arm, cycles 2, speed 0.5rad/s | side left/right/both, cycles 1–5, speed 0.2–1.5rad/s |
| `salute` | right arm, speed 0.5rad/s | side left/right, speed 0.2–1.5rad/s |
| `welcome` | right arm, cycles 2, speed 0.5rad/s | side left/right/both, cycles 1–5, speed 0.2–1.5rad/s |
| `raise` | right arm, speed 0.5rad/s | side left/right/both, speed 0.2–1.5rad/s |
| `reach_forward` | right arm, speed 0.5rad/s | side left/right/both, speed 0.2–1.5rad/s |
| `present` | right arm, speed 0.5rad/s | side left/right/both, speed 0.2–1.5rad/s |
| `high_five` | right arm, speed 0.5rad/s | side left/right, speed 0.2–1.5rad/s |
| `cheer` | right arm, speed 0.5rad/s | side left/right/both, speed 0.2–1.5rad/s |
| `reset` | right arm, speed 0.5rad/s | side left/right/both, speed 0.2–1.5rad/s |
| `stop` | no parameters | no parameters |
