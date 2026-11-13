# Audio environment Card

`audio_environment` is a read-only, single-instance processor in the Bumi bundle.
It uses existing microphone PCM; it opens no recording device, changes no gain,
saves no audio and issues no motion commands. No new dependencies are needed.

## Operator-set dBFS threshold

In the Card's **配置** panel choose **固定 dBFS 界限**, set a negative threshold
(for example `-20`), and choose **大于界限触发** or **小于界限触发**. `-20 dBFS`
is louder than `-40 dBFS`. This is digital signal level, not acoustic dB SPL;
do not enter a sound-level-meter value such as `60 dB`.

- `mode=threshold`: compare each 0.25-second RMS window directly with
  `threshold_dbfs` (-160 to 0), without a quiet-background calibration.
- `comparison=above`: strictly greater than the threshold; equality does not enter.
- `comparison=below`: strictly less than the threshold; equality does not enter.
- `hold_s` (default 2 seconds): valid audio duration needed to enter. Continued
  matching audio emits no duplicate event.
- `hysteresis_db` (default 3): above-mode recovery is at or below threshold minus
  this margin; below-mode recovery is at or above threshold plus the margin.
  Below-mode recovery must not exceed 0 dBFS, or configuration is rejected.
- `recover_s` (default 2 seconds): valid audio duration needed to clear.

Threshold mode publishes `threshold_entered` / `threshold_cleared`, including
`comparison`, `threshold_dbfs`, `rms_dbfs` and a sequence number. State is
`normal` / `triggered` / `unknown`; `baseline_dbfs` and `relative_db` stay null.
Connect the **events** port to Decision Core and update the consuming Skill to
handle these names. `loud` / `recovered` remain exclusive to relative mode.

Valid all-zero audio qualifies for below-mode detection, with `rms_dbfs=null`
rather than negative infinity. Missing/malformed audio never counts as quiet:
continuity resets discard pending/active detections, and stale state is unknown.
Short peaks or dips must not be confused with sustained threshold crossings.
Gaps, stop and changed settings reset detection without emitting a clearing
event. Events are discrete notifications, not a permanently latched state;
consumers must check `state`/`read`, freshness and settings after interruptions.

```json
{"name":"audio_environment","arguments":{"action":"config","mode":"threshold","threshold_dbfs":-20,"comparison":"above","hold_s":2,"recover_s":2,"hysteresis_db":3}}
```

Use `comparison: "below"` to trigger below the selected level. `config` with no
settings queries the current values. Invalid changes are rejected atomically.
Changes take effect on new audio, clear statistics and pending detection, and
retain event sequence numbers; they do not restart the ROS subscription or the
microphone. Reapplying identical valid settings preserves current detection.
`calibrate` is rejected in threshold mode because no background is needed.
Direct MCP changes are in-memory; use deployment YAML for driver-start defaults.
PM saves panel settings and reapplies them through its existing config mechanism;
after recreating a driver verify `read.settings` before treating it as configured.

Default `mode=relative` preserves the original behavior described below. Neither
mode saves raw audio or calibrates physical sound pressure.

## Adjustable relative-mode baseline

The background baseline is an absolute digital RMS level in **dBFS**. `rise_db`
and `fall_db` are **dB differences from that baseline**, not baseline values.
In PM configuration choose `baseline_mode=auto` (default) to collect the median
of quiet calibration-window levels, or `manual` to enter `manual_baseline_dbfs`.
The manual field defaults to -40 dBFS but is unused while auto is selected.
Manual baseline range is -90 to 0 dBFS, matching the relative comparator's
-90 dBFS floor; baseline plus `rise_db` must not exceed 0 dBFS.

For a manual baseline of -40 dBFS, `rise_db=12` and `fall_db=6`:

- Level **at or above -28 dBFS** for `hold_s` (default 2 seconds of valid audio)
  enters `loud` once.
- While loud, level **at or below -34 dBFS** for `recover_s` (default 2 seconds)
  emits `recovered` once.
- The -34 to -28 dBFS band prevents rapid repeated transitions.

```json
{"name":"audio_environment","arguments":{"action":"config","mode":"relative","baseline_mode":"manual","manual_baseline_dbfs":-40,"rise_db":12,"fall_db":6,"hold_s":2,"recover_s":2}}
```

Manual mode skips quiet sampling. Changing the baseline takes effect on new
audio, resets pending detection and retains event sequence numbers. Stop, gaps
and invalid packets clear observations; manual settings remain available for
the next valid input. `calibrate` rejects manual mode instead of overwriting
the chosen baseline; switch back to auto to collect a new quiet baseline.
With no complete window, `relative_db` remains null even if the manual baseline
is configured. In threshold mode both baseline settings are unused.

## Canvas

Connect `mic` (or an existing mono PCM16k source) to `audio_environment`.
Connect **events** to Decision Core; use **state** for the dashboard. Sending
the 4 Hz state stream to the model creates unnecessary model traffic.

In relative/auto mode, start with the room quiet for five seconds of valid audio.
`calibrate` discards the current auto baseline and collects a new one.
`read`/`info` report state without consuming
events; `stop` releases this Card's ROS node, subscription and timer and clears
its measurements. It never stops the microphone or other subscribers.

```json
{"name":"audio_environment","arguments":{"action":"start","input_topic":"/bumi/mic/audio"}}
```

The namespace/topic above is an example: use the actual upstream Card port.
Supported input is `audio_msgs/msg/AudioChunk`, format `audio/pcm-16k`, mono
16,000 Hz signed 16-bit little-endian PCM, 1–8,000 samples per packet. The
existing Bumi mic's equivalent legacy message format `pcm_16k_16bit_mono` is
also accepted; other encodings are rejected. Rate and
channel count are contractual: AudioChunk has no separate rate/channel fields,
so mislabeled audio cannot be detected reliably.

## Outputs

- `/<namespace>/audio_environment/state`: `data/json`, 4 Hz while running.
- `/<namespace>/audio_environment/events`: `data/json`, only `loud`/`recovered`
  transitions, with process-local `sequence` and `priority=1`.

Each 4,000-sample (0.25-second) window reports normalized RMS and peak [0,1],
`clip_ratio` (fraction with absolute PCM value >=32760), and `rms_dbfs`.
Digital silence has `rms_dbfs=null`; JSON never contains infinity or NaN.
Baseline comparison floors levels at -90 dBFS to make silence comparisons finite.
`relative_db` compares the current window with the fixed calibrated baseline.
These are digital signal levels, **not calibrated dB SPL or exposure limits**.

Relative-mode states: `unknown`, `calibrating`, `normal`, `loud`. `normal` only means below
the configured relative event criterion; it does not mean an acoustically safe
or legally compliant noise level. No automatic speech or alarm is performed.

Defaults in config.yaml: `calibration_s=5`, `hold_s=2`, `recover_s=2`,
`rise_db=12`, `fall_db=6`. Baseline is the median of calibration windows.
Sustained rise of at least 12 dB triggers one `loud`; sustained return to within
6 dB triggers one `recovered`. Durations are accumulated **valid audio time**,
rounded up to full 0.25-second windows. Short peaks do not trigger a sustained
noise event. Configure at deployment; restart the bundle after editing YAML.

## Failure and measurement limits

Invalid packets reset calibration. Receive gaps over 0.75 seconds reset continuity
and require calibration again. No valid packet for over 2 seconds reports
`unknown`, null measurements and `fresh=false`; missing data is never recovery.
Sequence numbers survive recalibration and Card restart within the same process.
Input header stamps, when positive, must increase; repeated or reversed stamps
invalidate observations. Restart the Card after an upstream source clock reset.
Zero/missing stamps use receive timing only. Freshness is receive-clock based;
strict capture-time latency and replay protection are not claimed.

The current Bumi mic applies 50x gain and clipping before publication; this
Card measures that processed signal, not raw microphone pressure. Clipping
statistics therefore include upstream gain saturation.
AGC, microphone placement, robot speaker playback, motor noise and other audio
processing affect the result. The Card cannot distinguish speech, music and
mechanical noise; no sound classification or echo cancellation is included.
Calibrate again after input gain/device/environment changes. Generic algorithm
placement in Perception remains subject to maintainer review; this change adds
one Bumi-packaged processor without changing shared drivers.

## Validation and acceptance

```bash
python -X utf8 -m pytest noetix/bumi/tests/test_audio_environment.py tests/test_bumi_bundle.py -q
python scripts/check_service_yml.py noetix/bumi
```

On Bumi: connect actual mic, calibrate in quiet, create sustained louder audio,
verify exactly one entry event, then one recovery event; disconnect the source
and verify unknown. Verify stop/restart, input clock reset, clipping and that
other mic consumers continue working. Unit/ROS tests do not replace this test.
