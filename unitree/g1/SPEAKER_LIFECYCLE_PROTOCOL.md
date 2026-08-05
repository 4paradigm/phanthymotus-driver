# G1 speaker lifecycle protocol v1

This protocol adds a correlatable lifecycle to PCM streamed through the G1
`SpeakerPlugin`. It does not add a completion callback to Unitree's native
`TtsMaker` API.

## Audio input

The producer publishes the existing `audio_msgs/AudioChunk` wire format:

- `format`: `audio/pcm-16k`
- `header.frame_id`: `utt:<producer-generated-id>` on every audio frame
- EOF: the existing 8-byte `AUDIO_EOF_MAGIC`, with the same `header.frame_id`

The producer must not reuse an utterance ID during one speaker session. A UUID
is recommended. Adding `frame_id` is backward compatible with old speakers,
which only inspect `data`. The PCM topic remains best-effort for compatibility;
a correlated stream whose EOF is lost terminates with `missing_eof`, not a false
`completed` receipt.

An empty or non-`utt:` frame ID is treated as a legacy stream. It still plays,
but its driver-generated `legacy:<n>` ID cannot be correlated with the original
`speak` call.

## Receipt output

For input topic `<audio_topic>`, the speaker publishes JSON in
`std_msgs/String` on `<audio_topic>/speaker_receipts`:

```json
{
  "type": "speech_playback_receipt",
  "version": 1,
  "session_id": "speaker:<uuid>",
  "utterance_id": "utt:<producer-generated-id>",
  "state": "completed",
  "completion_basis": "g1_play_state_observed",
  "audio_bytes": 32000,
  "ts": 1780000000.0,
  "reason": "",
  "receipt_topic": "/perception/tts/speaker_receipts"
}
```

States are `started`, `completed`, `cancelled`, and `error`. The last three are
terminal. Terminal identity is `(session_id, utterance_id)`, and consumers must
deduplicate it. A terminal receipt may arrive without `started` when delivery of
the earlier event failed; consumers must accept that transition.

With `completion_mode: hardware_state`, the driver also subscribes to the G1
controller's `rt/audio_msg` DDS topic. The body-speaker service publishes
`{"play_state":1}` when its player starts and `{"play_state":0}` when it
becomes idle. Before the first and every subsequent successful `PlayStream`
submission, the driver captures an event-sequence checkpoint. It also records
the event sequence when each successful submission is acknowledged, so a late
idle event from the preceding block cannot complete the stream. It reports
`completed` only after observing a `1` after the first pre-submit checkpoint and
a `0` after the final successful-submission checkpoint. The resulting basis is
`g1_play_state_observed`.

Before accepting the first audio block, strict mode waits for CycloneDDS to
report that the `rt/audio_msg` reader has matched a firmware publisher. This
distinguishes a constructed local reader from a usable state source and avoids
turning DDS discovery delay into a false first-play timeout. State callbacks run
directly on the DDS reader, and an idle transition must remain the newest state
for a short settle window before it becomes terminal.

This is a firmware player-idle acknowledgement, not an acoustic measurement:
it cannot prove that a failed amplifier produced audible sound. The state topic
is global rather than stream-ID keyed, so the driver keeps speaker playback
serialized and fails closed on a missing transition or timeout. `PlayStop`
during interrupt produces `play_state=0`, but generation invalidation prevents
that event from becoming a false `completed` receipt.

Because the topic is global, the bundled device also hides/rejects native
`tts.speak` whenever strict Speaker completion is enabled; volume actions remain
available. Audio writers outside this Driver bundle cannot be coordinated, so a
deployment relying on these receipts must give the Speaker plugin exclusive
ownership of the G1 body-speaker service.

`completion_mode: estimated` remains available for older firmware. In that
mode `completed` retains the previous `driver_drained_estimated` meaning: all
PCM was submitted and its calculated duration plus tail grace elapsed. To
restore native synthesis in such a configuration, also set
`plugins.tts.speak_enabled: true` explicitly.

Hardware-state mode does not offer resumable pause. Unitree exposes no playback
cursor, so stopping during the final hardware tail would discard audio that
cannot be resumed safely; callers should use interrupt/cancel instead.

## Reviewer hardware smoke

Run this only against a dedicated G1 test deployment. It replaces the active
Speaker input topic, plays the normal startup beep, sends 1.5 seconds of
near-silent non-zero PCM, validates a hardware-state `completed` receipt, and
stops Speaker during cleanup:

```bash
docker exec -it embodied-unitree-g1 bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /ros_ws/install/setup.bash &&
  python3 /work/tools/g1_speaker_receipt_smoke.py \
    --confirm-exclusive-hardware
'
```

Success prints JSON containing `result: passed`, `play_state.ready: true`, at
least one matched publisher, and a terminal receipt whose basis is
`g1_play_state_observed`. Missing DDS discovery, missing transitions, receipt
timeout, non-hardware completion, and incomplete PCM delivery all fail with a
non-zero exit code. The command is intentionally opt-in because it interrupts
any existing Speaker session.

## Delivery and recovery

Receipts use RELIABLE, TRANSIENT_LOCAL, KEEP_LAST(50) QoS. A consumer that needs
late-join replay must request compatible transient-local QoS. The driver keeps a
bounded retry outbox for local publish exceptions and leaves the receipt
publisher alive after `stop`.

Consumers should still implement this recovery sequence:

1. subscribe before submitting the utterance;
2. wait for a matching terminal `(session_id, utterance_id)`;
3. deduplicate retries;
4. on timeout, call speaker `info` and reconcile against
   `recent_terminal_receipts` and `pending_receipts`;
5. treat a remaining timeout as unknown/error rather than assuming completion.

## Rollout compatibility

| Producer | Speaker | Result |
| --- | --- | --- |
| old | old | Existing playback; no correlatable completion |
| new | old | Playback remains compatible; receipt unavailable |
| old | new | Playback plus legacy receipt; no call correlation |
| new | new | Correlated lifecycle receipts |

The safe rollout order is speaker driver first, then a producer that assigns IDs
and repeats them on EOF, then a Core receipt waiter with timeout-to-`info`
reconciliation. Until the latter two changes ship, this driver PR is protocol
foundation rather than an end-to-end fix for Core interaction ordering.
