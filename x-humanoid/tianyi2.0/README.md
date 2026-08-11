# Tianyi 2.0 Pro Driver

Phanthy Motus driver bundle for the Tianyi 2.0 Pro humanoid robot. The driver
bridges robot-side ROS2 topics on domain 0 to Agent Core topics on domain 42 and
exposes the capabilities as MCP tools.

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
