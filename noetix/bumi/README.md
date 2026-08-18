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
at least one non-zero velocity (`vx`/`vyaw`: magnitude 0.5-1.0; `vy`: magnitude
0.6-1.0), defaults to 2 seconds and is limited to 1-10 seconds. The SDK interface
does not document `vx` and `vy` as mutually exclusive, so using both requests
diagonal translation. Before every bounded move, the card sends one `WALK`
edge to activate this SDK client's walking command path and then follows the
vendor example by sending `DEFAULT` velocity frames at 100 Hz. Adaptive joint
feedback must confirm that locomotion started; otherwise the card sends zero
velocity and returns an error instead of reporting a false `running` state. It
also sends zero velocity when the duration expires, `stop_move` is called, or
the observed workmode leaves walking. It does not automatically promote another
mode into walking because the SDK cannot verify that the robot is physically
standing.

Long-running physical operations use Agent Core's Action Completion Protocol
(ACP). `loco.move`, both posture actions, `semantic_action.wipe_tears`, and
`action_recording.play_recording` return a unique `action_id`; their background
workers report `completed`, `error`, or `cancelled` to `/api/acp/complete`.
This keeps Agent Core's actuator barrier active until the bounded move or action
really terminates. Stopping the plugin cancels pending timers and playback
monitoring before reporting the affected ACP actions as cancelled.
Posture calls initially return `execution_phase=queued`; no `command_sent`
claim is made until the background worker has completed its safety checks. A
disabled-to-enabled `START` edge is still sent exactly once because it is a
toggle, but its workmode observation window is six seconds to tolerate delayed
firmware state feedback. Successful and failed ACP callbacks are logged with
their `action_id` for field diagnosis.

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
motion is confirmed. If either action mode is observed without the required
joint motion, the result reports an error and includes only the documented BMS
SOC and alarm fields. A non-zero BMS alarm is reported as a possible low-charge or
battery-condition cause; no undocumented SOC threshold is invented by the
driver.

`play_recording` returns `running` after play-teach mode is observed, then
monitors workmode and all 21 joint velocities. Once joint motion has started and
subsequently remained stationary for the configured confirmation window, the
driver sends `WALK` to return automatically to workmode 2. No duration or manual
stop parameter is exposed. The monitor also has no-motion and maximum-runtime
safeguards because the SDK does not expose a dedicated physical playback-
completion event.

`finish_and_save_recording` maps to the supported `SAVETEACH` command. The
vendor-deprecated `ENDTEACH` command and unavailable `RUN` command remain
unexposed.

Useful observations while the driver is running:

```bash
ros2 topic echo /<robot_namespace>/motion/state
ros2 topic hz /<robot_namespace>/motion/state
docker logs -f embodied-noetix-bumi
```
