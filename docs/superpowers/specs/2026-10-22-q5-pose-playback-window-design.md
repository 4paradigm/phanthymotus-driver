# Q5 Pose Playback Window Design

## Goal

Build an offline-verifiable Q5 prototype that continuously records JSON Pose
messages while a TTS instruction is submitted, waits for trusted speaker
completion, records a further three seconds, and freezes a bounded per-round
sequence for a later scoring component.

This change is confined to `phanthymotus-driver-speaker-pr`. It does not modify
Motus TTS, deploy an image, contact a robot, control motion, or weaken Speaker
Playback Telemetry completion rules.

## Existing Capabilities and Gaps

- Q5 Speaker Playback Telemetry already distinguishes queued/submitted/playing,
  trusted ALSA device completion, failure, interruption, and timeout.
- Motus perception can publish lean JSON Pose frames on
  `/nvidia_desktop/camera/rgb/poses`. A frame contains a source wall-clock
  `timestamp` and a `persons` array whose entries include a tracked person `id`
  and pose/action fields. Keypoints are optional on this topic; the prototype
  must preserve any extra person fields without requiring them.
- A separate skeleton topic exists in Motus, but this prototype does not
  subscribe to or assume it is available.
- The Q5 driver process uses the vendor ROS domain, while `q5_media_bridge`
  already owns the Agent Core Domain 42/FastDDS subprocess. Pose traffic must
  reuse that bridge and its lifecycle.
- Agent non-sensor tool calls are normally sequential. Waiting in an Agent,
  MCP request, TTS submitter, or `sleep(3)` is therefore not a safe sampling
  mechanism. Pose receipt needs its own bridge callback and Driver pump thread.
- Motus already contains pose/action recognition. It does not provide this
  round-window recorder or round scoring. This prototype exports sequences and
  does not duplicate the classifier or invent a scoring algorithm.
- Current Motus TTS producers do not yet prove the tracked
  `speech_id/sequence/eof` contract end to end. Offline tests therefore use
  mock trusted playback results. Real TTS metadata integration remains a device
  acceptance item.

## Architecture

### Domain 42 bridge path

`q5_media_bridge` gains one bounded Pose lane. Its ROS subprocess subscribes to
the configured String topic, initially only
`/nvidia_desktop/camera/rgb/poses`. The callback captures
`time.monotonic_ns()` before any inter-process transfer and offers this payload:

```json
{
  "raw_json": "{...}",
  "bridge_received_monotonic_ns": 123456789
}
```

The multiprocessing queue is bounded. On overflow the callback removes the
oldest visible entry, retries once, and increments a shared Bridge transport
drop counter. It never blocks the ROS callback. The parent exposes configure,
pop, and statistics methods through the existing `BridgeWorker`; `main.py`
binds those methods onto the existing shared client.

### PoseRecorder

`pose_recorder.py` contains a ROS-independent, thread-safe recorder. A dedicated
plugin pump thread continuously drains the bridge Pose lane and calls
`PoseRecorder.ingest`. Ingestion continues while any MCP dispatch, TTS submit,
playback wait, or post-playback delay blocks another thread.

Each accepted frame preserves:

- exact raw JSON text;
- source `timestamp` as supplied by perception, or `null` when invalid;
- Bridge receive monotonic nanoseconds;
- Driver receive monotonic nanoseconds;
- all valid persons, including their `id` and an unmodified copy of every
  additional pose field.

The recorder does not require keypoints and does not reinterpret posture,
activity, gesture, or future pose fields.

Invalid JSON and invalid root/person schemas are rejected without terminating
the pump. A missing or non-finite source timestamp is retained as `null` and
counted. Persons without a usable `id` are omitted and counted; the containing
frame may still be recorded. Counter categories remain separate:

- Bridge transport drops;
- recorder ring-buffer evictions;
- invalid JSON;
- invalid schema/person records;
- invalid source timestamps;
- cross-process monotonic anomalies.

### Clock model and window membership

Bridge and Driver are parent/child processes on the same Linux host and boot,
so Python `time.monotonic_ns()` in both processes uses the same system monotonic
clock and is directly comparable. Round membership uses
`bridge_received_monotonic_ns`, which is captured closest to DDS receipt.
`driver_received_monotonic_ns` is retained to diagnose queue delay. If the
processes are ever moved to different hosts, this assumption becomes false and
the adapter must map clocks or switch membership to Driver receipt time.

The perception source timestamp is a wall-clock value and is never used for
round membership. The remote playback timestamp is also not compared directly
with Pose timestamps. The post-playback window starts from the Driver's local
monotonic observation of a trusted completion result.

Window bounds are inclusive: `start_ns <= bridge_received_ns <= end_ns`.

### Bounded storage and frozen rounds

The live frame buffer is a fixed-capacity `deque`. Evicting its oldest frame
increments `recorder_cache_evictions`. Only one round may be active at a time.
`start_round(round_id)` records its boundary before invoking the TTS submitter.

Finishing filters the live ring by the round window and stores an immutable
snapshot in a separate ordered, bounded completed-round collection. Starting a
new round does not overwrite the preceding snapshot. The number of snapshots
and frames per snapshot are both configured limits; any eviction or truncation
is explicit in status and in the exported snapshot's `complete` flag. Thus
memory is bounded without silently claiming a complete sequence.

Round IDs are unique while retained. Duplicate active IDs and overlapping
rounds are rejected.

### Round controller

`PoseRoundController` is independent of ROS and receives injected TTS submit,
playback wait, clock, and wait functions. Its order is fixed:

1. start the recorder round;
2. submit TTS;
3. wait while the independent Pose pump continues running;
4. accept completion only when the playback result says `completed`, has
   `completion_level=alsa_device_consumption`, and reports trusted device
   progress;
5. record the Driver monotonic completion-observation time;
6. wait three seconds while Pose ingestion continues;
7. freeze the inclusive window and return it.

Playback `failed`, `interrupted`, wait `timeout`, manual termination, submit
errors, and malformed/untrusted completion results produce distinct terminal
round outcomes. They never become `COMPLETED`. Only successful trusted playback
gets the three-second post window; error outcomes freeze at their local
observation time.

### Plugin surface

A small Q5 plugin owns the recorder and pump lifecycle and exposes:

- `start_round(round_id)`;
- `finish_round(round_id, outcome, ...)` for explicit/manual workflows;
- `export_round(round_id)`;
- `info()` for active round, capacities, counters, and Bridge drops.

The offline `PoseRoundController` supplies the tested automatic orchestration.
It is not presented as a working real-TTS integration until tracked AudioChunk
metadata and real Q5 completion have been verified.

## Testing

Tests use fake clocks, fake Bridge queues, mock Pose JSON, blocking mock TTS,
and mock playback results. They cover:

- round start occurs before TTS submission;
- Pose frames continue entering while TTS submit/playback wait is blocked;
- speaking-time and inclusive boundary frames belong to the current round;
- the three-second post-completion interval remains live;
- later rounds do not overwrite or reclassify earlier frozen data;
- live and frozen storage stay within configured limits;
- Bridge drops, recorder evictions, and invalid-data counters stay distinct;
- invalid JSON, missing fields, invalid timestamps, and missing person IDs;
- trusted completion, playback failure, timeout, interruption, manual stop, and
  untrusted/missing completion evidence;
- source, Bridge, and Driver timestamps are retained and queue delay is visible;
- future extra pose/keypoint fields survive round-trip export unchanged.

The repository's existing Q5 test suite, Python compilation, generated remote
ALSA script compilation, and `git diff --check` remain required regressions.

## Safety and Delivery

- No robot connection, motion command, deployment, or remote push.
- No XOS routing bypass and no Speaker completion semantic changes.
- No changes to the Motus repository.
- The pre-existing untracked file `nvidia@10.100.128.225` is not read,
  modified, staged, or committed.
- Results explicitly distinguish offline Mock/Fake verification from pending
  Q5 device validation.
