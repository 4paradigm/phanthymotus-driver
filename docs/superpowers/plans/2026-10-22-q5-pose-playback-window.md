# Q5 Pose Playback Window Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an offline-verifiable, bounded Q5 Pose recorder and round-window controller that keeps sampling throughout blocked TTS playback and the three seconds after trusted playback completion.

**Architecture:** Reuse the existing Domain 42 `q5_media_bridge` subprocess for a bounded Pose String lane, and drain it from an independent Driver plugin pump thread into a ROS-free `PoseRecorder`. A separate injected `PoseRoundController` owns start-before-TTS ordering and trusted-completion timing without changing Speaker telemetry or claiming the current Motus TTS metadata path works.

**Tech Stack:** Python 3, standard-library dataclasses/deque/threading/multiprocessing, ROS2 String adapter inside the existing bridge, pytest with Fake Clock/Queue/TTS.

**Spec:** `docs/superpowers/specs/2026-10-22-q5-pose-playback-window-design.md`

## Global Constraints

- Modify only `/Users/4paradigm/Projects/phanthymotus-driver-speaker-pr`; do not modify Motus.
- Do not access Q5 hardware, deploy images, bypass XOS routes, push, or weaken playback-completion evidence.
- Do not read, modify, stage, or commit `nvidia@10.100.128.225`.
- Subscribe only to `/nvidia_desktop/camera/rgb/poses`; preserve optional future fields without requiring skeleton or gesture topics.
- Window membership uses Bridge `time.monotonic_ns()` only under the same-host/same-boot parent-child clock assumption; source and ALSA timestamps are metadata, not window clocks.
- Real TTS `speech_id/sequence/eof` remains unverified; automatic round tests use Fake completion and reports must say so.
- Production storage and inter-process queues are bounded, with separate Bridge-drop, recorder-eviction, frozen-round-eviction, and invalid-input counters.

## Review Focus

- A temporarily empty multiprocessing queue must not reorder or silently lose Pose accounting; bridge overflow remains nonblocking and increments the transport-drop counter.
- A source wall-clock timestamp or remote playback timestamp must never determine round membership; tests use deliberately incompatible values.
- A completed round whose early live frames were evicted must export `complete=false`, never a plausible-looking complete sequence.
- Concurrent stop during an empty or active pump must join cleanly and remove the bridge subscription without leaving a thread behind.
- Unknown fields and optional keypoints in a person record must survive export byte-for-structure without becoming required schema.

---

### Task 1: Thread-safe bounded PoseRecorder

**Files:**
- Create: `robotera/q5_bundle/pose_recorder.py`
- Create: `robotera/q5_bundle/test_pose_recorder.py`

**Interfaces:**
- Produces: `PoseFrame`, `RoundSnapshot`, and `PoseRecorder`.
- `PoseRecorder.__init__(max_frames=600, max_rounds=20, max_frames_per_round=600, clock_ns=time.monotonic_ns)`.
- `PoseRecorder.ingest(raw_json: str, bridge_received_monotonic_ns: int, driver_received_monotonic_ns: Optional[int] = None) -> bool`.
- `PoseRecorder.start_round(round_id: str, started_monotonic_ns: Optional[int] = None) -> dict`.
- `PoseRecorder.finish_round(round_id: str, outcome: str, ended_monotonic_ns: Optional[int] = None, playback_result: Optional[dict] = None, completion_observed_monotonic_ns: Optional[int] = None) -> dict`.
- `PoseRecorder.export_round(round_id: str) -> dict` and `PoseRecorder.status(bridge_transport_drops: int = 0) -> dict`.
- Terminal outcomes are exactly `COMPLETED`, `PLAYBACK_FAILED`, `TIMEOUT`, `MANUAL_TERMINATED`, `SUBMIT_FAILED`, and `UNTRUSTED_COMPLETION`.

- [ ] **Step 1: Write failing parsing and timestamp tests**

  Add tests asserting valid frames retain exact raw JSON, source timestamp, Bridge time, Driver time, person IDs, keypoints, and unknown fields. Add separate tests for invalid JSON/root schema, non-finite or missing source timestamps, missing person IDs, and Driver-before-Bridge monotonic anomalies with distinct counters.

- [ ] **Step 2: Run the parsing tests and verify RED**

  Run: `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q test_pose_recorder.py -k 'ingest or invalid or timestamp'`

  Expected: collection/import failure because `pose_recorder.py` does not exist.

- [ ] **Step 3: Implement immutable frame parsing and bounded live storage**

  Implement copied/immutable exported records, `deque(maxlen=...)`, exact raw preservation, finite numeric source timestamp validation, usable scalar/string person IDs, and independent counters. Increment `recorder_cache_evictions` before appending into a full ring.

- [ ] **Step 4: Run the parsing tests and verify GREEN**

  Run the Step 2 command; expected all selected tests pass.

- [ ] **Step 5: Write failing round-window and capacity tests**

  Test inclusive start/end boundaries using Bridge monotonic time while source timestamps are deliberately unrelated. Test overlapping/duplicate round rejection, next-round isolation, prior frozen-round retention, live eviction causing `complete=false`, per-snapshot truncation, bounded completed-round eviction, and separate counters.

- [ ] **Step 6: Run round tests and verify RED**

  Run: `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q test_pose_recorder.py -k 'round or capacity or boundary'`

  Expected: failures for missing round methods/behavior.

- [ ] **Step 7: Implement active-round and bounded frozen-snapshot behavior**

  Use one active round and an `OrderedDict` of immutable snapshots. Mark snapshots incomplete when their start precedes the oldest retained Bridge timestamp or when snapshot truncation occurs. Validate terminal outcomes and require trusted playback fields plus at least 3,000,000,000 ns after local completion observation for `COMPLETED`.

- [ ] **Step 8: Run all recorder tests and verify GREEN**

  Run: `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q test_pose_recorder.py`

  Expected: all tests pass.

### Task 2: Fake-testable round orchestration and continuous sampling

**Files:**
- Modify: `robotera/q5_bundle/pose_recorder.py`
- Create: `robotera/q5_bundle/test_pose_round_controller.py`

**Interfaces:**
- Produces: `PoseRoundController(recorder, submit_tts, wait_for_playback, clock_ns=time.monotonic_ns, wait_post_window=time.sleep)`.
- Produces: `PoseRoundController.run_round(round_id: str, instruction: str, timeout_seconds: float) -> dict`.
- Consumes playback results shaped like existing `PlaybackTracker.wait_for_completion` output.

- [ ] **Step 1: Write the blocking concurrency test first**

  Run `run_round` in a worker thread. Make `submit_tts` assert the recorder already has the active round, block playback on an Event, ingest speaking frames from the test thread, release a trusted Fake completion, block the post-window wait on another Event, ingest more frames through exactly completion+3 seconds, then assert the frozen export contains both periods in order.

- [ ] **Step 2: Run the concurrency test and verify RED**

  Run: `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q test_pose_round_controller.py::test_pose_continues_during_blocked_tts_and_three_second_post_window`

  Expected: import/attribute failure for missing controller.

- [ ] **Step 3: Implement start-before-submit and trusted completion flow**

  Accept completion only for `result=completed`, `completion_level=alsa_device_consumption`, `timestamp_quality=DEVICE_PROGRESS_OBSERVED`, and a non-null playback finish timestamp. Capture the completion observation with the injected Driver clock, wait exactly 3.0 seconds through the injected waiter, and freeze using local monotonic bounds.

- [ ] **Step 4: Run the concurrency test and verify GREEN**

  Run the Step 2 command; expected pass.

- [ ] **Step 5: Add failure matrix and ten-round tests**

  Add parameterized tests mapping playback failed, interrupted/cancelled, timeout, submit exception, and malformed/untrusted completion to the exact terminal outcomes from Task 1. Add a 10-round sequential test with unique frames and assert every frozen round contains only its own speaking/post-window frames and earlier rounds remain exportable within configured capacity.

- [ ] **Step 6: Run controller and recorder tests**

  Run: `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q test_pose_recorder.py test_pose_round_controller.py`

  Expected: all tests pass and no real sleeps longer than test-controlled Events.

### Task 3: Reuse q5_media_bridge for a bounded Pose lane

**Files:**
- Modify: `robotera/q5_bundle/q5_media_bridge.py`
- Modify: `robotera/q5_bundle/main.py`
- Create: `robotera/q5_bundle/test_pose_bridge.py`
- Modify: `robotera/q5_bundle/test_q5_bus_bridge.py`

**Interfaces:**
- Produces: `BridgeWorker.configure_pose(topic: str)`, `BridgeWorker.pop_pose()`, and `BridgeWorker.pose_stats()`.
- The parent binds these methods to the existing `Q5SdkClient` instance.
- The subprocess command `pose_config` owns at most one `std_msgs/String` subscription and destroys/replaces it through the existing command loop.

- [ ] **Step 1: Write failing bounded-queue and callback tests**

  Test a callback payload contains exact `msg.data` and injected Bridge monotonic time. Fill a capacity-limited fake queue, assert callback returns without blocking, oldest/drop behavior stays bounded, and only the shared Bridge transport-drop counter changes.

- [ ] **Step 2: Run bridge tests and verify RED**

  Run: `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q test_pose_bridge.py`

  Expected: missing Bridge Pose APIs.

- [ ] **Step 3: Implement the Pose queue using the existing BridgeWorker lifecycle**

  Add a bounded multiprocessing queue and locked shared drop counter to `BridgeWorker`; pass both to `_run_bridge_subprocess`. Add a nonblocking `_offer_pose` helper, a String callback capturing `time.monotonic_ns()`, and `pose_config` handling in the existing command loop. Empty topic destroys the existing subscription. Do not create a second process or executor.

- [ ] **Step 4: Bind parent methods in main.py and add lifecycle tests**

  Explicitly bind configure/pop/stats beside existing Speaker bindings. Test start/reconfigure/stop command behavior with fakes, including idempotent teardown and no callback thread creation.

- [ ] **Step 5: Run bridge regressions**

  Run: `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q test_pose_bridge.py test_q5_bus_bridge.py test_speaker_playback_telemetry.py`

  Expected: all tests pass; existing Speaker queues and subscriptions remain unchanged.

### Task 4: Pose recorder plugin and independent pump lifecycle

**Files:**
- Create: `robotera/q5_bundle/pose_game.py`
- Create: `robotera/q5_bundle/test_pose_game.py`
- Modify: `robotera/q5_bundle/config.yaml`
- Modify: `robotera/q5_bundle/Dockerfile`

**Interfaces:**
- Produces standard Q5 `make_plugin(plugin_config, namespace, executor, client)`.
- Plugin methods: `start()`, `stop()`, `dispatch("start_round" | "finish_round" | "export_round" | "info", args)`.
- Consumes the Bridge configure/pop/stats methods from Task 3 and `PoseRecorder` from Task 1.

- [ ] **Step 1: Write failing pump independence tests**

  Use a fake client whose Pose getter is fed from another thread. Assert `start()` configures only `/nvidia_desktop/camera/rgb/poses`, the daemon pump ingests while the dispatch thread is blocked, and `stop()` clears the subscription, signals the pump, joins it, and is idempotent.

- [ ] **Step 2: Run plugin tests and verify RED**

  Run: `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q test_pose_game.py`

  Expected: missing plugin module.

- [ ] **Step 3: Implement the minimal plugin**

  Keep ROS out of the plugin. Poll `pop_pose()` with a stop Event and short bounded wait, stamp Driver receipt using injected/default monotonic clock, and forward raw Bridge items to the recorder. `finish_round` preserves all six Task 1 outcomes, including the required completed/failed/timeout/manual distinctions, and rejects a fake `COMPLETED` lacking the exact trusted evidence/post-window bound.

- [ ] **Step 4: Add schema/config/container wiring**

  Enable `pose_game` in `config.yaml` with `topic=/nvidia_desktop/camera/rgb/poses`, `max_frames=600`, `max_rounds=20`, and `max_frames_per_round=600`; copy `pose_recorder.py` and `pose_game.py` in the Dockerfile. Do not alter XOS, Speaker, camera, navigation, or motion settings.

- [ ] **Step 5: Run plugin and integration tests**

  Run: `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q test_pose_game.py test_pose_bridge.py test_pose_recorder.py test_pose_round_controller.py`

  Expected: all tests pass with all spawned test threads joined.

### Task 5: Architecture/use documentation and complete verification

**Files:**
- Create: `robotera/q5_bundle/README_POSE_PLAYBACK_WINDOW.md`
- Modify only if required by tests: files from Tasks 1-4

**Interfaces:**
- Documents the topic, plugin actions, exported frame/snapshot format, same-host monotonic assumption, counters, Mock-only completion status, and Q5 acceptance checklist.

- [ ] **Step 1: Write the concise operational document**

  Include a 10-line example showing `start_round` before TTS, trusted wait, three-second continued capture, and export. State that the current TTS metadata chain and real device completion are not verified.

- [ ] **Step 2: Run focused and full Q5 tests**

  Run from `robotera/q5_bundle`:

  `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q test_pose_recorder.py test_pose_round_controller.py test_pose_bridge.py test_pose_game.py`

  `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q --ignore=test_basic_sensors.py`

- [ ] **Step 3: Run static/script checks**

  Compile every Q5 Python source with in-memory `compile()`, extract and compile `_q5_alsa_speaker_command` without importing ROS2, then run `git diff --check`.

- [ ] **Step 4: Review scope and local state**

  Confirm only planned Driver files changed, Motus has no changes from this task, `nvidia@10.100.128.225` remains untracked and untouched, no hardware/network/deployment command ran, and no remote push occurred.

- [ ] **Step 5: Request final read-only code review and address findings**

  Review clock boundaries, trusted completion validation, bounded memory, queue-overflow accounting, thread teardown, and 10-round isolation. Re-run all verification after any fix.

- [ ] **Step 6: Create one local implementation commit only after green verification**

  Stage exact planned files individually, inspect the staged diff, and commit locally without push.
