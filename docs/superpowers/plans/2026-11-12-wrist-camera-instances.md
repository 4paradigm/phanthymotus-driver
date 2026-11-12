# Wrist Camera Instances Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace the four fixed wrist cards in PR #525 with one repeatable `wrist_camera` sensor, independently bound to a device and left/right wrist, with RGB and depth outputs.

**Architecture:** Keep SDK and ROS publication in isolated device workers. A plugin owns instance configurations and one capture session per physical device; workers fan out encoded frames to instance topics. Discovery runs outside the body-controller process and periodically refreshes device capabilities and reconnects requested instances.

**Tech Stack:** Python 3.10, multiprocessing, RealSense SDK, V4L2/OpenCV, ROS 2, pytest.

**Spec:** docs/superpowers/specs/2026-11-12-wrist-camera-instances.md (approved in chat; implementation and push explicitly requested).

## Global Constraints

- Tool name `wrist_camera`, `multiInstance: true`, instance-scoped side/device/fps/resolution.
- RGB then depth, stable instance topics, JPEG and uint16 millimetre zlib depth.
- No rotation, mirroring, flipping, alignment, or image resizing.
- Preserve 640x480 depth decoder compatibility; negotiate only supported native profiles.
- UVC cameras publish RGB with an explicitly unavailable depth port.
- Never guess left/right or silently change a saved device binding.
- Preserve shared physical acquisition, isolated worker crashes, independent instance stop, bounded shutdown, channel freshness, and preview without intelligent control.
- Push verified commits to the existing PR head `origin/feat/tianyi-wrist-cameras`; do not merge or deploy.

## Review Focus

- Two cards bound to one device: one worker; stopping one must preserve the other.
- A disconnected selected device must not be replaced by the next enumerated camera.
- A config replay before start must not restart a healthy worker or undo an intentional stop.
- Native SDK/ROS stalls must not hang body-controller lifecycle calls.
- Dynamic devices must not be duplicated across SDK and V4L2; metadata and head-camera nodes must not be opened accidentally.

### Task 1: Instance and device discovery contracts

**Files:** `tests/test_tianyi_wrist_cameras.py`, `tests/test_tianyi_wrist_worker.py`, `tests/test_tianyi_wrist_discovery.py`, `x-humanoid/tianyi2.0/wrist_cameras.py`, `x-humanoid/tianyi2.0/wrist_camera_devices.py`.

**Interfaces:** `camera_topic(namespace, instance_id, channel)`, `WristCameraPlugin(config, namespace)`, `discover_devices() -> {devices, errors}`, session-managed instance routes.

- [x] Write and run failing tests for repeatable schema, stable topics, device selection, unavailable depth, profile negotiation, UVC deduplication, and stale/missing-device handling.
- [x] Implement bounded isolated discovery and minimal RealSense/UVC device/profile selection.
- [x] Implement per-device sessions, instance routes, persistence, preview configuration, and reconnect monitoring.
- [x] Run targeted contract/discovery tests; expect all pass.

### Task 2: Capture worker and lifecycle

**Files:** `tests/test_tianyi_wrist_worker.py`, `x-humanoid/tianyi2.0/wrist_cameras.py`, `x-humanoid/tianyi2.0/main.py`, `x-humanoid/tianyi2.0/config.yaml`, `x-humanoid/tianyi2.0/driver.yaml`, `x-humanoid/tianyi2.0/Dockerfile`.

**Interfaces:** Workers receive an initial instance routing map and subsequent acknowledged route replacements. Discovery descriptors select the backend and native stream profiles. Plugin `config/start/stop/info` use the standard ext_mic instance protocol.

- [x] Write and run failing worker tests: native RGB-D/real depth scale, UVC RGB-only, fanout, stop during acquisition/publication, failed worker restart, per-channel timeout, and incompatible shared profiles.
- [x] Implement the capture loop and acknowledged route updates; retain loopback domain 42 and host-clock timestamps.
- [x] Register one plugin and remove hard-coded serial defaults; add V4L2 tooling and the discovery module to the image.
- [x] Test the real bundle and HTTP tool handler; expect one repeatable card and independent instance stop/restart.

### Task 3: Documentation, review, and push

**Files:** `x-humanoid/tianyi2.0/README.md`, relevant tests and plan checkboxes.

- [x] Document automatic discovery limits, configuration persistence/preview, shared settings, unavailable depth, stable side binding, native resolution compatibility, and old-card replacement.
- [x] Run all Tianyi and camera-info tests, full repository pytest, compile checks, and `git diff --check`; compare any existing collection failures with the base revision.
- [x] Request one independent whole-change review and address actionable findings with regression tests.
- [ ] Commit only relevant files, push fast-forward to the existing PR head, verify the remote SHA, and report tests and unverified hardware limits.

## Validation and implementation decisions

- `tests/test_tianyi_wrist_{cameras,discovery,worker}.py` plus MCP error contracts: 55 passed.
- Standalone `tests/test_camera_info.py`: 14 passed, 1 skipped (ROS unavailable).
- Tianyi top-level suite: 147 passed. Full repository: 75 existing collection errors, no new error paths.
- Python compilation and `git diff --check` pass. No deployment or hardware compatibility claim.
- Independent read-only review found two issues; both have RED-to-GREEN regressions:
  native capture hangs now recover using whole-pipeline freshness, including failed-discovery cycles;
  UVC node renumbering no longer changes physical identity or preferred interface.
- Instance state is persisted under the existing mounted data directory because Core only restores
  shared configs after reconnect. Intentional stop remains disabled after restart; driver shutdown
  retains the requested preview state. The tradeoff is one driver-owned state file.
- Route replacements use an acknowledged pipe instead of the old two-channel bitmask. ACK is
  serialized with publication, and timeout terminates the unresponsive worker. Other enabled
  users of that physical camera may briefly lose frames during crash recovery.
- A missing channel reports its own error while a fresh sibling remains live. Entirely stalled
  capture is restarted. Hardware-specific stream faults still require actual device validation.
- Existing canvases must replace the old four card names; no speculative compatibility aliases.

The following broad-suite failures also reproduce on base `75ee8f2e` and were left unchanged:

- `tests/test_camera_info.py::test_the_declaration_rides_along_with_info`
- `x-humanoid/tianyi2.0/tests/test_servo.py::test_both_interrupt_hooks_stop_it`
- `x-humanoid/tianyi2.0/tests/test_servo.py::test_start_requires_explicit_confirmation`
- `x-humanoid/tianyi2.0/tests/test_servo.py::test_commands_are_taken_on_core_and_published_on_tianyi`
- `x-humanoid/tianyi2.0/tests/test_servo.py::test_an_out_of_range_shoulder_never_reaches_the_robot`
- `x-humanoid/tianyi2.0/tests/test_servo.py::test_a_finger_beyond_full_closure_is_rejected`
- `x-humanoid/tianyi2.0/tests/test_servo.py::test_a_valid_command_reaches_every_publisher`

Whole-repository pytest cannot collect without optional vendor/ROS dependencies; the changed tree
has 75 collection errors, all also present on the base. The temporary base archive has one extra
launcher-manifest collection error because it lacks Git metadata.
