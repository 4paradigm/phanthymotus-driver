# Audio environment Card

`audio_environment` is a read-only, single-instance processor in the Bumi bundle.
It uses existing microphone PCM; it opens no recording device, changes no gain,
saves no audio and issues no motion commands. No new dependencies are needed.

## Canvas

Connect `mic` (or an existing mono PCM16k source) to `audio_environment`.
Connect **events** to Decision Core; use **state** for the dashboard. Sending
the 4 Hz state stream to the model creates unnecessary model traffic.

Start with the room quiet for five seconds. `calibrate` discards the current
baseline and collects a new one. `read`/`info` report state without consuming
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

States: `unknown`, `calibrating`, `normal`, `loud`. `normal` only means below
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
