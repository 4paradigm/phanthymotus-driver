# Tianyi IMU validation — 2026-11-12

Status: the vendor capture-clock mismatch is fixed on the tested host. A
600-second static F2F/ESKF run with depth assistance enabled maintained active
IMU without terminal loss. Intermittent coasting/recovering remains; motion
accuracy, geometric recovery and uninterrupted SLAM are **not yet accepted**.

## Evidence

The original independent gyro/accel callback used device timestamps despite
`time_domain=global`. Installed ARM64 disassembly confirmed the direct device
clock call. The isolated source-built library uses the same timestamp selector
as RGB-D; the original vendor installation remains untouched.

Independent 15-second probes, measuring receive time minus capture time:

| Profile / input | Samples | Median age | Future / nonmonotonic samples |
| --- | ---: | ---: | ---: |
| Original IMU at 200 Hz | about 3000 per sensor | about -1164 ms | future captures rejected |
| Patched gyro at 200 Hz | 2994 | 5.65 ms | 0 / 0 |
| Patched accel at 200 Hz | 2995 | 5.39 ms | 0 / 0 |
| Patched gyro at 100 Hz | 1500 | 1.93 ms | 0 / 0 |
| Patched accel at 100 Hz | 1501 | 1.18 ms | 0 / 0 |

No receipt-time offset, fabricated TF or relaxed freshness threshold was used.
Factory camera-internal TF supplies separate sensor rotations/translations.
`base_link` TF remains absent; these tests estimate camera motion, not body
motion. Missing body TF is not the IMU capture-clock cause.

A missing accel capture also pinned the source gyro queue. The source now
skips an unmatchable gyro after accel advances and prunes superseded accel
history. Offline tests reproduce the original queue blockage.

The test camera profile is native 640×480 RGB/depth at 30 Hz and 100 Hz IMU;
[build/deployment/rollback notes](README.md) describe the separate host service
change. This reduces raw image/pointcloud resolution. Intrinsics are refreshed
from CameraInfo rather than copied from the older 1280×720 profile.

## Earlier end-to-end limits

Tests used isolated PR519 source and PR371 consumer instances with the existing
native RTAB-Map runtime. Business Driver/Perception containers were not
replaced. No robot or head motion was commanded.

An attitude-only F2M run achieved 40.19 seconds of static tracking, 330 accepted
estimates and zero native lost reports. Longer runs and later revisions still
hit IMU freshness protection. ESKF with and without geometry reached tracking
outputs but later retired on delayed visual corrections; geometry also rejected
registration. Therefore the short successful window is not evidence of stable
operation, motion accuracy, blindness compensation or SLAM acceptance.

The generic consumer changes preserve IMU captures in a bounded FIFO while
visual processing owns estimator state, retain processing-time freshness and
overflow retirement, and refresh visual-correction timing during recovery.
Their offline tests pass; they do not establish a hardware acceptance result.

These dependent test processes were stopped and their data saved before the
user replaced the Driver container. The camera-service overlay remains active.

## Static retest after Driver replacement

The business Driver was `release.261112.5f0b926`, without this PR's RGB-D
plugin. PR519 source ran as a separate process in that container; PR371 ran
in an isolated consumer container with the existing RTAB-Map runtime mounted.
The business Driver/Perception images and original Driver entry point were
not replaced. No robot or head motion was commanded.

Two delay sources were isolated:

- The older diagnostic saved a growing, indented history in the estimator's
  Python process. A controlled 8.33 MB history write took 567–915 ms, pushing
  IMU consumption age to 629 ms and retiring an otherwise working attitude
  run after 66 seconds. Bounded telemetry ran for 180 seconds with active IMU.
  The final test saves bounded summaries during measurement and full history
  only after shutdown. One intermediate diagnostic GC-hook deadlock was also
  discarded; it is not production or acceptance evidence.
- Humble's generated compressed-image setter validates Python `bytes`
  element by element. On-device construction/assignment of a 204,800-byte
  payload took 33.56 ms median with `bytes`, versus 0.026 ms with
  `array('B', ...)`, including array construction. The test verified equal
  payload/metadata, serialized messages and deserialized payloads. Driver
  JPEG/zlib and consumer depth PNG now use typed byte buffers. IMU-enabled
  input/consumer nodes use owned single-threaded executors in their existing
  contexts, reducing callback contention. Source sampling uses capture time
  rather than callback arrival time. No new dependencies were installed.

The final F2F optical-flow run enabled `imu_assisted`,
`short_term_compensation` and `depth_geometry` for **600.027 seconds**:

| Measurement | Result |
| --- | ---: |
| Consumed IMU captures | 59,920 |
| IMU consumption age: median / p95 / maximum | 29.90 / 64.34 / 265.96 ms |
| Maximum consumed IMU capture interval | 30.03 ms |
| Native reports / native lost reports | 5,821 / 0 |
| Tracking poses | 5,075 |
| Output packets: initializing / tracking / coasting / recovering | 71 / 5,075 / 402 / 2,088 |
| Native diagnostic capture age: median / p95 / maximum | 130.79 / 186.53 / 285.78 ms |
| Depth capture interval: median / p95 / maximum | 100.12 / 133.49 / 233.61 ms |

Medians/p95 use the final bounded 4,096-sample window; counts and maxima
cover the run. Output states publish at different cadences, so their packet
counts are not percentages of time. The final state was tracking with active
IMU; no IMU retirement or terminal loss occurred. This is a static continuity
result, not an arbitrary-duration guarantee. The unchanged 250 ms visual
liveness check still caused short coasting/recovery intervals, so mapping
could not admit every native pose. All visual frames remained usable and
depth registration was never attempted; enabling geometry did not test its
recovery path. `base_link` TF remains unavailable.

The tested source SHA-256 was
`cb5ce9f4e69fd53c9409bdc6150b5772b65d93a6643bf7ff69d3b87c414c68cc`;
consumer runtime SHA-256 was
`954412a7761d106dd3d6a8d6523f570e4fcef4153be96b373674a6bf4111edfa`.
The later fail-closed source restart guard and atomic callback-failure state
update were verified offline; those failure-path changes were not loaded in
the 600-second run. Normal processing code is unchanged from that run.
Evidence is retained as `tianyi-imu-stability-evidence-20261112.tgz`, containing
run summaries, traces, native logs, code hashes and consumer exit status 0.

The isolated source/raw probe stopped gracefully within 3.5 seconds. Their
processes were absent afterward, and the exited test consumer was removed.
Business containers and the camera service stayed running. G1/Bumi hardware
acceptance remains pending; their Driver PRs were not changed in this retest.

## Offline checks

- Tianyi, shared mount and control scope: 241 passed.
- Generic RGB-D scope: 169 passed.
- Full Perception suite: 2967 passed, 46 skipped in the test environment.
  `AGENT_CORE_URL=https://127.0.0.1:9` isolates optional completion callbacks
  from live HTTPS services. No audio implementation was changed.
- Vendor library built successfully on the matching Jetson/Humble host.
