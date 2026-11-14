# Tianyi timestamped odometry implementation plan

> Execute inline with superpowers:executing-plans; steps below are the verification ledger.

**Goal:** Feed loco_state's existing state/odom port from timestamped gyro-assisted chassis increments, accepted by the SLAM pose decoder.

**Architecture:** A read-only diagnostic worker owns one TCP subscription. A pure planar integrator consumes every source-stamped delta. Bounded request/response clock observations map chassis milliseconds to host realtime; REST telemetry remains on the existing JSON port. Loss, invalid increments, clock discontinuity and reconnect invalidate the pose and create a new segment rather than joining trajectories.

**Tech stack:** Python standard library, existing ROS String publishers, pytest.

**Spec:** User's request in this chat, and diagnostics/loco-state-strict-sync-20261114 outside the repository. Timestamp means the chassis diagnostic source event; no hardware-trigger accuracy claim. No automatic motion, production service restart, or invented camera mounting transform.

## Constraints / review focus

- Preserve the first two ports and telemetry JSON contract; state/odom must never fall back to REST-arrival pose while advertising SLAM capability.
- Source increment timestamp and clock mapping must remain separate from read time. Delayed batches retain source stamps.
- Validate complete batches before applying them; reject nonfinite values, duplicates/reordering, excessive gaps and device time rollback.
- Restart, clock mapping change, or loss uses a new segment and no old integration state.
- Existing SLAM rejects changing mapping within one segment: use a fixed per-segment offset/uncertainty and monitor its validity.
- Source is planar chassis odometry, not full 6-DoF pose; covariance is unknown, not invented. Mount geometry remains explicit at the SLAM boundary.

## Tasks

- [x] 1. Extend stationary on-robot clock/source-time evidence; compare camera clock configuration/source; preserve evidence locally.
- [x] 2. RED: tests for clock interval admission, delayed source stamps, planar turn/translation, malformed batches, gaps, duplicate samples, lost/reconnect segment changes. Implement `loco_odometry.py` pure mapping/integration and read-only transport; GREEN.
- [x] 3. RED: loco_state publishes diagnostic pose without REST fallback, exposes SLAM declaration, starts/stops worker without leaking subscriptions; implement integration and Docker copy; GREEN.
- [x] 4. Replay real diagnostic data into actual SLAM decoder/alignment, verify local ROS publishing with a separate read-only probe, run focused and repository checks; fresh final review.

## Evidence / rulings

- Initial driver checkout is clean, linked worktree at f517916f. Consumer checkout has unrelated pointcloud edits: do not modify them.
- Original diagnostic stream observation: 198 samples in 2.113 seconds, same timestamp set as processed IMU; delayed read preserves source stamps.
- Hardware acquisition latency, moving-frame convention and full map acceptance remain distinct from software wire/clock validation; do not report them passed from stationary evidence.


## Verification results

- Source-only live worker: 35 seconds, 695 tracking samples, zero loss, one segment; shutdown completed.
- Actual Perception decoder, frame identity validator and strict pose-stream admission accepted all 695 samples; 694 adjacent interpolations succeeded. Loss clears buffered poses. This validates the pose input boundary, not camera geometry or an end-to-end map.
- Read-only ROS roundtrip with camera probe in another process: 226 published/received, 225 tracking and one timeout loss in 18 seconds; 450 camera frames. Camera time_domain is confirmed `global`; received cloud age was 33.6–55.4 ms (pipeline latency, not clock error).
- Same-process full-cloud stress: 104 tracking, ten timeout losses and three clock-budget losses in 20 seconds. Continuous full-system strict synchronization is NOT accepted; bounded failure behavior is verified. No production service was replaced or restarted, and no motion command was sent.
- Clock long run: 60 observations over about 30 seconds, offset variation 3.085 ms and intersected interval width 3.042 ms; host wall-minus-monotonic change 0.012 ms. Diagnostic long run: 3,000 gyro-assisted deltas over 32.14 seconds, with matching IMU source time.
- Focused regression: 381 passed, 33 subtests passed (loco_state, incremental odometry, common odometry, loco servo/process, camera depth/metadata/lifecycle). Final narrow suite: 100 passed, 33 subtests passed.
- Bare repository pytest: blocked during collection with 75 errors from unavailable device/ROS dependencies and duplicate test module names; no whole-repository green claim.
- Fresh independent review found low-rate diagnostic starvation. A failing 1 Hz test reproduced it; drain/integration now runs at least 20 Hz independently of publication. Regression is green.
- Ruling: discard only wide-RTT clock probes without refreshing any mapping state; keep the fixed offset only within the existing two-second expiry. Calibration requires five accepted probes within two seconds. Rollback/disagreement remains fatal. Added expiry and calibration-count regression tests.
- Ruling: publish source-time pose capability and per-frame mapping, but do not fabricate a cross-sensor relation or mount declaration. Existing native cloud timing remains unverified. A moving-robot frame-convention test, calibrated camera transform and loaded production acceptance remain outstanding hardware work.
- Local evidence: `diagnostics/loco-state-strict-sync-20261114/adapter-verification-summary.json` and `adapter-consumer-verification.json` in the workspace parent. Raw captures remain outside git.
