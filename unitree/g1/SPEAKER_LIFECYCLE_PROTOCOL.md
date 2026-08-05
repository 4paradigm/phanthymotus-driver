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
  "completion_basis": "driver_drained_estimated",
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

`completed` means the driver submitted all PCM, waited its calculated duration,
and applied a short tail grace. Unitree exposes no hardware drain/audible-end
acknowledgement, so this is deliberately reported as
`driver_drained_estimated`, not exact acoustic completion.

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
