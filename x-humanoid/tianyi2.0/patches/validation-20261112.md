# Tianyi IMU validation — 2026-11-12

Status: the vendor capture-clock mismatch is fixed on the tested host. IMU
packets and camera-internal TF calibration are available, but sustained
end-to-end IMU-assisted tracking is **not yet accepted**.

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

## End-to-end limits

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

The dependent test processes were stopped and their data saved before the user
replaces the Driver container. Camera-service overlay remains active. Continue
static continuity checks against the replacement container before enabling the
full ESKF/depth route for mapping.

## Offline checks

- Tianyi, shared mount and control scope: 235 passed.
- Generic RGB-D scope: 165 passed.
- Full Perception suite: 2963 passed, 46 skipped in the test environment.
  `AGENT_CORE_URL=https://127.0.0.1:9` isolates optional completion callbacks
  from live HTTPS services. No audio implementation was changed.
- Vendor library built successfully on the matching Jetson/Humble host.
