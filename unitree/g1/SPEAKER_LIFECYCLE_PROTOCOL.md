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

## Direct MCP status and wait

The `speaker` MCP tool exposes the same terminal state directly, so callers do
not need to subscribe to ROS or repeatedly inspect the latest global status:

```json
{
  "action": "wait_playback",
  "session_id": "speaker:<value returned by speaker start>",
  "utterance_id": "utt:<value carried by every PCM and EOF frame>",
  "timeout_sec": 20
}
```

For example, a direct JSON-RPC call is:

```bash
curl -sS http://127.0.0.1:15701/mcp \
  -H 'Content-Type: application/json' \
  -d '{
    "jsonrpc":"2.0",
    "id":"speaker-wait",
    "method":"tools/call",
    "params":{
      "name":"speaker",
      "arguments":{
        "action":"wait_playback",
        "session_id":"speaker:REPLACE_ME",
        "utterance_id":"utt:REPLACE_ME",
        "timeout_sec":20
      }
    }
  }'
```

The terminal receipt is returned directly. In strict G1 mode,
`state: completed`, `terminal: true`, and
`completion_basis: g1_play_state_observed` mean the firmware player was
observed returning to idle for that exact utterance. `cancelled` and `error`
are also terminal. `timeout` means the utterance was observed but has not yet
reached a terminal state; `unknown` means its `started` frame was not observed
before the timeout. Both are non-terminal and retryable with the same identity.

The default wait is 20 seconds and one call is capped at 25 seconds, below
Core's 30-second MCP request timeout. Use the same identity in a later call for
long audio; a terminal result retained in bounded history returns immediately.
`timeout_sec: 0` performs an immediate correlated lookup. A stale or incorrect
session returns a non-retryable session error instead of matching another
utterance with the same ID.

This action shape is suitable as a common Speaker contract for other robots,
but this PR implements it only for the G1 plugin. Each other driver still needs
to connect its own hardware/player acknowledgement to the terminal receipt.

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
near-silent non-zero PCM, waits through the MCP `wait_playback` action, then
cross-checks the matching ROS hardware-state receipt and stops Speaker during
cleanup:

```bash
docker exec -it embodied-unitree-g1 bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /ros_ws/install/setup.bash &&
  python3 /work/tools/g1_speaker_receipt_smoke.py \
    --confirm-exclusive-hardware
'
```

Success prints JSON containing `result: passed`, `play_state.ready: true`, at
least one matched publisher, an `mcp_wait_result`, and a matching
`terminal_receipt`; both terminal results must use
`g1_play_state_observed`. Missing DDS discovery, missing transitions, receipt
timeout, non-hardware completion, inconsistent MCP/ROS results, and incomplete
PCM delivery all fail with a non-zero exit code. The command is intentionally
opt-in because it interrupts any existing Speaker session.

## Delivery and recovery

Receipts use RELIABLE, TRANSIENT_LOCAL, KEEP_LAST(50) QoS. A consumer that needs
late-join replay must request compatible transient-local QoS. The driver keeps a
bounded retry outbox for local publish exceptions and leaves the receipt
publisher alive after `stop`.

Consumers should implement this sequence:

1. use the `session_id` returned by speaker `start`;
2. assign one `utt:` ID and carry it on every PCM frame and EOF;
3. call `wait_playback` with that exact pair;
4. retry the same pair after a non-terminal timeout/unknown result;
5. use speaker `info` or the ROS receipt topic for diagnostics and recovery;
6. never assume completion from elapsed audio duration alone.

## Rollout compatibility

| Producer | Speaker | Result |
| --- | --- | --- |
| old | old | Existing playback; no correlatable completion |
| new | old | Playback remains compatible; receipt unavailable |
| old | new | Playback plus legacy receipt; no call correlation |
| new | new | Correlated lifecycle receipts |

The safe rollout order is speaker driver first, then a producer that assigns IDs
and repeats them on EOF. At that point Core or any MCP client can invoke the
Driver's `wait_playback` action directly; no separate ROS subscriber is needed
in Core. Interaction logic must still call the wait action before advancing to
the next dependent step.
