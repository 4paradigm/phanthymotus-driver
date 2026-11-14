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


## Follow-up: loaded continuous verification, 2026-11-14

- Camera `time_domain=global` verified before and after; no camera parameter, production service, or motion command changed. Read-only diagnostic probe received the full native cloud stream in the same Python process. Production driver was release.261114.f517916; Perception/core were release.261114.e472c49, so earlier-release timeouts cannot be attributed solely to these fixes.
- Confirmed false clock-drift classification: a 3.766 ms request produced the interval [-0.477, +5.289] ms relative to the frozen mapping. It overlaps the admissible budget and cannot prove drift. Inconclusive intervals now reject without refreshing evidence; intervals proving drift still retire the segment. The 5 ms verification budget, 10 ms advertised uncertainty and two-second expiry are unchanged.
- Confirmed publication starvation: alternating 1/3 ms reads resulted in only 19 publications for 39 reads in the deterministic regression. Publication deadlines now advance from the scheduled deadline instead of resetting to receipt time.
- Confirmed clock retry starvation: several inconclusive observations, spaced about 550 ms apart, exhausted freshness. Rejected observations now retry on the next drain; sustained rejection still expires normally. All three behavioral regressions were observed failing before their fixes.
- Final uninterrupted 600-second probe: 17,980 clouds, 11,192 tracking poses, 15 loss events, 16 tracking segments. Diagnostic reads: median 1.678 ms, p99 10.860 ms, max 19.629 ms. Clock requests: median 3.317 ms, p99 13.439 ms, max 17.395 ms. Zero transport errors/timeouts; 471 inconclusive clock observations rejected without extending freshness.
- Remaining losses: 14 duplicate source timestamps and one clock expiry. The expiry occurred despite fast retries and mostly 2–3 ms HTTP replies; the intervals persistently failed to verify the frozen mapping within the unchanged budget. This is not a connection timeout and is not fixed by extending the connection timeout.
- Tracking output intervals within a segment: median 53 ms, p95 65 ms, p99 98 ms, max 151 ms. Scheduler-watch maximum interval was 162 ms. These outliers and segment resets prevent continuous strict SLAM acceptance.
- Earlier saved IMU evidence also contains two different IMU payloads at source timestamp 4141686, matching duplicate odometry timestamps. Do not treat this as harmless TCP retransmission or deduplicate increments without a source sequence/acquisition-time contract. No fabricated timestamps, permissive deduplication, or relaxed timing limits were introduced.
- Final focused regression: 385 passed, 33 subtests passed. Actual Perception decoder and frame-identity validator accepted all 11,192 tracking poses; emitted stamps increased strictly within each segment. The probe exited with its worker stopped. Baseline probes interrupted for instrumentation were not counted as a completed soak; their helper processes were explicitly cleaned up.
- Acceptance result: **continuous strict SLAM failed**. The software fixes are bounded improvements, not a claim that chassis timestamps are unique hardware sampling times or that a fixed mapping remains verifiable indefinitely. A producer acquisition timestamp/sequence contract and a justified long-term clock mapping are required before claiming uninterrupted strict operation.
- Raw data, probe, test logs and machine-readable report: workspace `diagnostics/loco-load-stability-20261114/summary.json`, `consumer-verification.json`, `load-stability-evidence.tgz` (outside git).

## User-approved revision: bounded software synchronization, 50 ms

The user selected a maximum 50 ms software synchronization budget, prioritizing continuous mapping. This replaces the previous strict 10 ms policy. Hardware acquisition accuracy and calibrated camera geometry remain unproven.

Implementation: freeze the per-segment offset with 50 ms advertised uncertainty and a 25 ms verification budget; retain the 8 ms probe RTT limit, two-second freshness expiry, 250 ms stale-data limit and 100 ms source-gap limit. Preserve every exact_once subscription event in delivery order. Equal source timestamps form one group, including across reads. Hold the newest group until a later source timestamp closes it, then compose all increments and emit one final pose without fabricated timestamps. Compute group-average twist over distinct source times. First-group displacement establishes the origin; reconnect/loss still retires the segment. Metadata identifies bounded software synchronization and the number of source events per group.

Validation: nonzero translation/turn tests for equal stamps across reads, monotonic output, no emission of open groups, reset and malformed-batch protection, 50 ms budget boundaries; actual SLAM decoder replay; uninterrupted 600-second native-cloud load probe. Follow the inline execution workflow and obtain one fresh whole-change review before completion.

SDK inspection: message_seq_num_t/range types exist, but Message<TPayload> exposes only timestamp and payload; MovementEstimation has only XY displacement and angular displacement; DiagnosisSubscribeProvider::readMessages returns vector<Message<...>>. STMS MessageReadStream exposes topic/type/timestamp/payload, not a sample sequence. No usable per-event sequence or separate hardware timestamp was found in these public interfaces. Grouping assumes the requested exact_once subscription delivers ordered increment records; it does not claim transport replay detection through an exposed sequence number.


50 ms revision verification:

- Fresh review identified a retained-group freshness bypass (259 ms old data could be emitted after a delayed read). A failing regression reproduced it; pending groups are now revalidated atomically before composition. Final focused suite: 390 passed, 33 subtests passed; narrow suite: 109 passed, 33 subtests passed.
- Final unchanged-runtime 600-second probe: 17,979 native clouds, 11,373 tracking outputs, one segment, zero losses, zero transport errors/timeouts, zero clock expirations. Camera global mode reconfirmed. All probe processes stopped; no production service was restarted.
- 56,076 source events received; 56,075 belong to closed groups, one final event remains in the intentionally unclosed group at shutdown. All source records accounted for. Four extra equal-timestamp events were retained and composed, not deduplicated.
- 1,382 clock requests; 288 high-latency observations rejected without refreshing validity. Clock RTT p99 13.871 ms/max 26.320 ms; diagnostic read p99 11.130 ms/max 24.731 ms. Tracking publication interval median 53 ms, p99 105 ms, max 200 ms. The 50 ms policy is a clock-mapping uncertainty budget, not a latency or publication-period bound.
- Actual Perception decoder, frame identity validator and pose stream accepted all 11,373 outputs; all 11,372 adjacent midpoint interpolations passed its 200 ms gap setting. All output timestamps increase strictly within the single segment. The identity transform used in this pose-only replay is not a camera extrinsic claim.
- Scope of acceptance: stationary source/clock continuity under full native-cloud reception passed for ten minutes. Hardware acquisition latency, body-frame convention under actual motion, calibrated camera transform, and an end-to-end moving map remain unverified. No relaxation was made to stale-data, rollback, gap or disconnect handling.
- Evidence: workspace `diagnostics/loco-load-stability-20261114/soft50/summary.json`, `consumer-verification.json`, raw capture, SDK interface archive and `tests-soft50-acceptance.log` in the parent evidence folder. SDK inspection did not find a usable subscription sample sequence/hardware timestamp.
