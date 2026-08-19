# Noetix Bumi driver

The bundle exposes the original Bumi sensor, locomotion, audio and camera cards,
plus the higher-level motion-state and direct-action cards. All card
implementations are kept in `device.py` and use HighController or MediaController;
the bundle does not initialize LowController.

## New cards

### `motion_state`

Passive, read-only whole-body motion telemetry from `HighController`. The card has no action parameters or execute button. Its single JSON output combines the former summary and joint views:

- current activity, workmode, protection flag, body orientation, angular velocity, linear acceleration and whole-body joint activity statistics;
- active motor faults whose codes are documented by Noetix;
- position, velocity, torque, temperature and raw error value for all 21 joints.
- ROS2 output: `/<namespace>/motion/state`, JSON.

The default polling and topic publication rate is 2 Hz (`poll_interval_s: 0.5`).
Motion activity uses a configurable joint-speed threshold, defaulting to
`0.15 rad/s`. Only motor error codes explicitly documented by Noetix are
classified as faults. Undocumented non-zero raw values are not shown in the
documented fault list and remain available only in each joint's raw `error`
field for device-side verification.

Every published state identifies `Noetix HighController/CycloneDDS` as its source and includes a freshness flag. It deliberately excludes battery data, which belongs to the existing `battery` card. The SDK does not expose world-frame position or translational velocity, so the card reports only documented IMU and joint measurements and does not invent odometry.

## Direct action cards

### `loco`

Uses only `HighController` to send bounded normalized forward, lateral and
turning commands. A move is accepted only in workmode 2 (`walking`), requires
the caller to select exactly one `velocity_channel` (`vx`, `vy` or `vyaw`) and
provide one non-zero `velocity` (`vx`/`vyaw`: magnitude 0.5-1.0; `vy`: magnitude
0.6-1.0). The two unselected channels are always forced to zero, so one MCP
call cannot combine translation and turning. Duration defaults to 2 seconds and
is limited to 1-10 seconds. Before every bounded move, the card sends one `WALK`
edge to activate this SDK client's walking command path and then follows the
vendor example by sending `DEFAULT` velocity frames at 100 Hz. Adaptive joint
feedback must confirm that locomotion started; otherwise the card sends zero
velocity and returns an error instead of reporting a false `running` state. It
also sends zero velocity when the duration expires, `stop_move` is called, or
the observed workmode leaves walking. Move replacement, thread assignment and
ACP binding are serialized as one session, so concurrent MCP requests cannot
create two velocity publishers over the shared stop event. Every command
preroll checks workmode before each frame and aborts before the action edge if
protection mode is observed. The card does not automatically promote another
mode into walking because the SDK cannot verify that the robot is physically
standing.

Long-running physical operations use Agent Core's Action Completion Protocol
(ACP). `loco.move`, both posture actions, every `semantic_action`, and every
`action_recording` action return a unique `action_id`; their background
workers report `completed`, `error`, or `cancelled` to `/api/acp/complete`.
Every terminal result includes the same `action_id`, action name, terminal
`state`, a boolean `success`, and a plain-language completion message; failures
also include a concrete `error` reason.
Terminal callbacks use bounded retries and remain registered in the driver
until Agent Core acknowledges delivery, preventing a fast completion or a
transient localhost/registration race from silently losing the result.
This keeps Agent Core's actuator barrier active until the bounded move or action
really terminates. Stopping the plugin cancels pending timers and playback
monitoring before reporting the affected ACP actions as cancelled.
Preset semantic actions use workmode feedback plus joint displacement and a
rolling joint-velocity window to distinguish physical completion from command
acceptance. A firmware return to walking is also treated as successful
completion; protection, unexpected workmode changes, missing physical motion,
feedback errors and monitor timeouts are reported as ACP errors. `wipe_tears`
keeps its guarded five-second return-to-walking behavior, while `reset` is
complete only after walking mode is confirmed.
For action recording, starting and saving report completion after the vendor
target workmode is confirmed; this completes the requested command, not the
open-ended user-guided recording session. Playback remains pending until joint
feedback indicates the motion ended and walking-mode recovery is confirmed.
Posture workmode and stand-up pose checks run before the asynchronous request is
accepted, so invalid requests such as calling `stand_up` while already standing
return an immediate plain-language error. A valid request returns only its ACP
acknowledgement, `action_id`, requested action, and safety context. Before a
disabled robot is enabled for stand-up, the driver explicitly sends zero
velocity and then sustains a neutral DDS preroll. The `START` edge is still sent
only while workmode is confirmed disabled. If the first edge is not
acknowledged after 10 seconds, stand-up makes one bounded retry. The full
preroll continuously guards workmode, so a delayed first transition cancels the
retry instead of toggling the robot back to disabled. After enabled mode is
observed, rolling joint velocity feedback must remain stable before
`FALLTOSTAND` is sent; this avoids treating policy selection as proof that
motor-enable initialization has finished. Successful and failed posture
startup results and ACP callbacks are logged with their `action_id` for field
diagnosis. The accepted stand-up response asks the user to allow about 20
seconds for this asynchronous preparation and not to submit a conflicting retry
until the current action reports an error.

The former `switch_mode` tool is split into three user-facing cards. Internal
`enable`, `ready` and `walk` transitions are completed automatically and are no
longer exposed as user choices:

- `stand_up_lie_prone`: `stand_up` from a face-up lying pose, or `lie_prone`
  from a stable standing pose into the prone storage posture;
- `semantic_action`: wave, handshake, cheer, three dances and wipe-tears;
- `action_recording`: start recording, finish and save a recording, or play a
  saved recording by `recording_id`.

Every result reports the automatically executed preparation steps, the observed
workmode, whether the requested action start was confirmed, plain-language
safety requirements and the fact that the SDK cannot verify the robot's real
physical pose. An observed target action mode returns `running`, not
`completed`, because mode feedback does not prove the physical motion has
finished. If any preparation or action enters protection mode, the card stops
the sequence and tells the user to restart, place Bumi face-up on a flat,
non-slip surface with a clear 3 m × 3 m area, and then use `stand_up`.

`semantic_action.reset` exits or interrupts an active semantic action and
returns the robot to workmode 2 (`walking`). It is accepted only from semantic
action workmodes, or treated as a no-op when already walking. It never promotes
disabled, enabled or ready modes into walking because the SDK cannot verify the
physical pose.

`stand_up` is accepted only from disabled or enabled mode and its description
requires the operator to place the robot face-up before calling it. `lie_prone` requires
the operator to confirm stable standing through the card instructions and is
accepted only from walking mode. These
guards prevent a standing robot from receiving the get-up trajectory.
`lie_prone` treats workmode 28 as the action start; a direct transition to
disabled is not reported as `running`. Whole-body joint displacement must also
confirm that the physical action started.

`wipe_tears` similarly requires arm-joint displacement after tear mode is
observed. Its five-second automatic return timer starts only after physical arm
motion is confirmed. Automatic return uses at most two guarded `WALK` edges and
stops retrying as soon as workmode changes, so a missed first edge can recover
without sending another command after walking mode is reached. If either action
mode is observed without the required joint motion, the result reports an error
and includes only the documented BMS SOC and alarm fields. A non-zero BMS alarm
is reported as a possible low-charge or battery-condition cause; no
undocumented SOC threshold is invented by the driver.

`play_recording` returns `running` after play-teach mode is observed, then
monitors workmode, all 21 joint velocities, and displacement from the playback
starting pose. A five-sample rolling velocity median filters isolated encoder
spikes; sustained movement resets the stationary score, while three seconds of
stationary feedback completes playback. The same guarded two-attempt `WALK`
exit then returns the robot to workmode 2. No duration or manual stop parameter
is exposed. The monitor also has no-motion and maximum-runtime safeguards
because the SDK does not expose a dedicated physical playback-completion event.

`finish_and_save_recording` maps to the supported `SAVETEACH` command. The
vendor-deprecated `ENDTEACH` command and unavailable `RUN` command remain
unexposed.

Useful observations while the driver is running:

```bash
ros2 topic echo /<robot_namespace>/motion/state
ros2 topic hz /<robot_namespace>/motion/state
docker logs -f embodied-noetix-bumi
```
