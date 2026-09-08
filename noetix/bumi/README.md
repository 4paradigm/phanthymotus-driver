# Noetix Bumi driver

The bundle exposes the original Bumi sensor, locomotion, audio and camera cards plus one higher-level motion-state card backed by documented Noetix SDK APIs. All card implementations are kept in `device.py`.

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

`play_recording` remains `running` after play-teach mode is observed. The SDK
does not document a physical playback-completion event, so the driver does not
send `WALK` on a guessed timeout. After visible completion, or to interrupt
playback, use `action_recording.stop_playback`; it is accepted only from
play-teach mode and confirms the return to walking.

`finish_and_save_recording` maps to the supported `SAVETEACH` command. The
vendor-deprecated `ENDTEACH` command and unavailable `RUN` command remain
unexposed.

## `speaker`

Plays the PCM audio of its connected input topic (`audio_msgs/AudioChunk`, mono
16 kHz S16LE) on the robot speaker, or switches to the vendor voice Agent.
The tool schema exposes these actions through the existing card action controls:

| Action | Behaviour |
| --- | --- |
| `start` / `play` | Require `input_topic`, disable vendor Agent routes, sleep an awake Agent, then enable external PCM playback and subscribe. Sound begins when upstream AudioChunk data arrives; this is not a test-tone or file-playback action. |
| `wakeup` | Stop external PCM, close its playback route, enable internal microphone → vendor Agent and vendor Agent → speaker, resume capture/playback, then wake the Agent if needed. External custom audio → Agent stays disabled. |
| `sleep` | Close all three vendor Agent routes and sleep an awake Agent. Leave any external PCM subscription and playback route untouched; never pause capture/playback here. |
| `stop` | Stop external PCM only. In `vendor_agent` mode this is a no-op, including when the Agent itself is temporarily SLEEPED. Canvas stop and driver lifecycle stop do not explicitly disable that mode; use `sleep` to exit it. |
| `reset` | Map to `MediaController.restart()`, not a robot/factory reset. Stop PCM, wait for CMD_RESET followed by healthy READY/SLEEPED, then reapply route isolation. Leave idle; do not automatically wake or resume PCM. |
| `info` | Report mode, in-progress operation, external PCM statistics, media status/error, desired routes, cached SDK readback and the last control result. |
| `get_volume` / `set_volume` | Read / set volume (0–200). Volume changes do not select a mode. |

The canvas already sends `start` with its resolved input topic; explicit `play`
now appears in the schema and uses exactly the same implementation. Merely
constructing/starting the driver bundle does not select a Speaker mode.

`audio_mode` tracks this Speaker instance's selected policy (`idle`,
`external_playback`, `vendor_agent`, or `unknown`), separately from the SDK's
`work_status`. It is not a hardware-wide ownership guarantee. `wakeup` hands the
robot microphone and speaker to the vendor's own model. Starting Speaker again
switches back to external PCM and disables these three Agent routes:

```python
set_internal_capture_audio_data_to_agent_enable(False)
set_external_custom_audio_data_to_agent_enable(False)
set_internal_agent_audio_data_to_playback_enable(False)
```

Mic, its capture subprocess and `main.py` are unchanged. PCM still travels
directly from the ROS AudioChunk subscription to
`publish_external_audio_playback_stream`, with mono samples duplicated to stereo
as before. There is no AudioRouteCoordinator or audio-forwarding layer. The
media module is shared hardware, however: explicit reset or automatic fault
recovery can interrupt microphone capture even though no Mic code changes.

### Timing and failures

Speaker serializes mode changes and volume writes. Every configuration setter
attempt, including a failed attempt and a volume write, leaves at least **0.8 s**
before the next setter. The PCM callback does not acquire the control/config
locks, so `sleep` configuration waits do not block an existing external stream.
Allow several seconds for a normal mode switch and longer for recovery; do not
interpret the first submitted command as a completed switch.

- Route setters get one retry per step. Wake/sleep/restart commands are not
  blindly repeated within an action. Wake/sleep status waits are bounded at
  8 s; reset waits are bounded at 20 s and require observing CMD_RESET before
  accepting a subsequent healthy READY/SLEEPED sample.
- `start`/`play`/`wakeup` retain automatic restart recovery for ERROR_SLEEPED.
  Ordinary `sleep` never starts a reset, avoiding disruption of external PCM.
- Activation failure prevents a new subscription (or removes it if creation
  fails), stops external delivery and attempts all Agent isolation steps.
  It does not restore the previous stream or Agent mode. Failed isolation
  reports `audio_mode=unknown`, not a claim that the Agent has stopped.
- Results return `state=error`, the failing stage, individual steps/errors,
  subscription state, SDK status/error and suggested next action. Cleanup
  errors are retained alongside the initial failure. Inspect connectivity and
  the errors, then retry the desired action, or use explicit `reset` for a
  media-module fault. Explicit reset still attempts recovery if its pre-reset
  cleanup fails, and rechecks isolation afterward.
- Desired routes are intentions. SDK route setters submit commands and their
  getters may return cached values, not per-command acknowledgements. `info`
  exposes both separately. Other SDK clients/processes are not serialized by
  this instance's locks and can change the shared hardware state.

Previous hardware observations recorded for this driver (the new switching
behaviour still needs on-device acceptance testing):

- **`work_status=SLEEPED` does not block playback.** A full TTS utterance played
  while the agent sat at `SLEEPED/CMD_SLEEPED`. Only `ERROR_SLEEPED` means the
  audio agent is really down; `start` then calls the documented `restart()`
  recovery by itself and reports it.
- **`get_volume()` is not trustworthy.** A freshly created MediaController reads
  `0` for 12 s and longer while audio plays normally, and each client appears to
  read back its own last `set_volume`. Treat a `0` as "unknown", never as the
  reason for silence — `info` reports it for reference only.

Likewise `frames_submitted` in `info` counts frames handed to the SDK, not
frames heard: `publish_external_audio_playback_stream` is fire-and-forget, so a
rising count proves the subscription works and nothing more.
These counters cover only this instance's external PCM stream, never vendor
Agent speech; the last stream's counters remain visible after a mode switch.

### Local tests and device acceptance

Run the hardware-free ROS/SDK-double tests without starting the robot:

```bash
python3 -m unittest discover -s tests -v
```

On the robot, check `play` with a connected PCM source, `wakeup` → `stop`
(Agent remains available), `wakeup` → `play` (only external PCM plays), and
`play` → `sleep` (external PCM continues). Check `reset` separately because
it restarts the shared media module. Inspect `info` and listen to actual output;
local tests and SDK submissions alone cannot confirm on-device routing.

Useful observations while the driver is running:

```bash
ros2 topic echo /<robot_namespace>/motion/state
ros2 topic hz /<robot_namespace>/motion/state
docker logs -f embodied-noetix-bumi
```
