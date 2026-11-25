# R1 RGB-D / point-cloud SLAM evidence

Read-only observation on 2026-11-14, 21:50–22:10 Asia/Shanghai. This is an
input-readiness investigation, not stationary tracking, motion accuracy or
sustained-fusion acceptance. No robot motion, sensor start/stop, application
restart, image replacement or deployment was issued.

## Versions and running tasks

- Platform PR371 and its fork branch `feat/pointcloud-slam-offline-eval` were
  both checked at `73c6b70df1090477c46aa7fb21d1b9b7f66e65f8`. The supplied
  development worktree was clean. Its other worktrees were left untouched.
- This Driver change starts from upstream main
  `b3e8e78851b29456ac01a8abe5df5ffa2d7dbc2e`, in a separate worktree/branch.
  The pre-existing Driver checkout was clean at `02e8b71c` and was not edited.
- Live Core: `release.261111.2b00f1d`; Perception:
  `release.261113.fa8193f-jetson-jp5.11`; ActuCore:
  `release.261113.a332240-jetson-jp5.11`; R1 Driver:
  `release.261105.29ed0a0`. These tags do not certify unmodified source.
- Live `/work/device.py` SHA-256:
  `3b84e1579d03adc6c3b57d3e8c1422de47bd6644d38548f7dc34f8eed08eae5c`;
  `/work/camera_specs.py`:
  `974d380bc29fea699fc36f3cd05f0efda6773fa2bf75eef488d2600567546341`.
- Four existing GStreamer receivers were already running. Read-only MCP info
  reported pointcloud, rgbd_odometry, pointcloud_slam, visual_depth, VOP,
  navigation and loco_servo idle. Navigation had published zero commands;
  loco_servo had applied zero commands. The saved canvas contained navigation
  wiring; saved wiring alone does not imply a running task.
- The installed Perception still advertises the older internal mono/stereo
  pointcloud paths. Those paths were not called. PR371's current depth-only
  implementation must be evaluated separately from this installed version.

## Observed sources

The new boot differs from the previous 2026-11-13 multicast-only observation.
An eight-second passive capture on `eth10` found four separate H.264/RTP
unicast streams from `192.168.123.161` to `192.168.123.164`. Each had payload
type 96, its own SSRC, and no RTP extension. H.264 SPS dimensions/timing and
decoded ROS JPEG dimensions were checked independently.

| Destination | Existing Driver name | Encoded size | Published JPEG | SPS / observed rate |
| --- | --- | --- | --- | --- |
| 5001 | main | 720×1280 | 1280×720, clockwise rotation | 15 / 14.97 Hz |
| 5002 | left | 544×448 | 544×448 | 10 / 10.02 Hz |
| 5003 | right | 544×448 | 544×448 | 10 / 10.02 Hz |
| 5000 | depth | 544×448 | 544×448, sampled image black | 10 / 9.98 Hz |

The two named left/right samples depict different views with parallax. This
supports two image sources, not verified optical left/right identity,
rectification, simultaneous exposure or a common capture clock. Each observed
stream contains one image, not demonstrated side-by-side stereo. The depth
output is an 8-bit, three-channel JPEG visualization; no measured Z16 depth,
scale, valid range or native PointCloud2 was observed in the ROS domain-42
graph. This does not establish that the hardware has no depth capability.

A separate 15.03-second passive ROS subscription received 225 main, 150 left,
151 right and 144 depth JPEGs; 298 IMU JSON samples; and 148 odometry samples.
Those counts include discovery and existing publisher batching, so they are
not sensor exposure rates. The IMU messages contain quaternion, gyroscope,
accelerometer, RPY and temperature, but no capture time, clock identity or
camera extrinsic. Odometry provides body twist, `pose:null`, and explicitly
reports `stamp_source:received` because the vendor stamp is unusable.

## Time and latency evidence

The installed camera code discards RTP metadata, decodes/re-encodes JPEG, then
sets `header.stamp` from the local ROS clock immediately before publishing.
`header.frame_id` was empty. These are publication times, not capture times.
The host reported `NTP=no`, `NTPSynchronized=no`; even synchronized host NTP
would not prove an exposure-to-RTP or IMU-to-video clock mapping.

An additional read-only raw DDS subscriber observed 6322 lowstate packets
and 2992 sport packets in approximately six seconds. Lowstate `tick` changed
from 1082803 to 1088808; the sampled sport `stamp.sec/nanosec` values were
zero. Tick units, wrap/reset handling and its relationship to video exposure
remain unverified. Do not convert it to a capture clock by fitting receipt
times. The owned subscriber logged a reader-close callback race during exit;
its process exited without changing the existing Driver subscriptions.

| Stage | Evidence / limitation |
| --- | --- |
| Exposure → network reception | Unmeasurable without capture-clock mapping; RTP counters alone are insufficient. |
| Packet arrival span per marked RTP frame | main median/max 0.440/2.269 ms; left 0.232/2.617; right 0.390/4.316; depth 0.115/1.639. Receiver-side packet spans only, not transmission latency. |
| Decode, JPEG encode and existing pipe wait | Not separately instrumented on the running application. A real-pipe regression independently reproduces the 64 KiB read wait. |
| Publication header → observer callback | Median main 9.77 ms, left 3.80, right 3.76, depth 1.43; maxima 11.57/8.88/12.32/12.24 ms. Includes local delivery and observer scheduling. |
| Capture pairing | Not measured: existing JPEG headers lost exposure identity and time. Close receipt times cannot establish a stereo pair. |
| VO / fusion / mapping computation | Not run: required inputs are unavailable. No tracking or precision claim. |

The captured RTP sequences had no observed gaps and tcpdump reported zero
kernel drops in this bounded sample; this is not a lossless-link guarantee.
A separate eight-second passive RTCP-type filter captured no SR/RR evidence;
it does not prove that the sender never emits RTCP. A sender report would
still need evidence tying its RTP timestamp to actual exposure.

## Calibration and frames still required

- Obtain device-specific lens models, K/D for both actual stream resolutions,
  stereo R/T (with direction and length units), calibration ID/device serial,
  reprojection/epipolar validation, and any sender crop/rotation/rectification.
  Obtain depth encoding/scale, invalid-value convention and validated range if
  the vendor exposes measured depth instead of reconstructing stereo.
- The existing `camera_specs.py` main FOV is a historical plane fit on
  `r1_sz, 2026-09-23`; it is not full calibration for this unit. Main K/D are
  null and distortion is unknown; all three other ports report unknown optics.
- `/home/unitree/ws/src/stereo_param` has July capture/calibration scripts and
  15 named image pairs, but no calibration output was found in that directory
  or the bounded filename search. The scripts grab the latest frame from each
  receiver without capture synchronization. Board dimensions, device identity
  and capture pairing are unverified. Do not run them as trusted calibration,
  use their template constants, or infer calibration from matching filenames.
- Sender SSH on `.161:22` refused connections; `.100:22` timed out. Request a
  vendor export or supervised sender read access, including stream routing and
  capture timestamp documentation. Do not switch its services to investigate.
- Request the camera/IMU axes, quaternion order, units and calibrated rotation
  and sensor-origin translation with provenance. The visualization URDF has
  pelvis/head joints but no calibrated camera/IMU chain. Historical body-twist
  measurements and the `base_link` control declaration are not that chain.
- Optional body pose needs T_body_camera at each image capture time. Without
  it, keep the primary pose camera_optical and body_pose unavailable. Do not
  copy Tianyi extrinsics, joints or body-frame defaults.

## Minimum route and acceptance gates

First seek a vendor measured-depth plus reference-image export with real
capture timestamps. If only calibrated, synchronized stereo is available,
rectify and reconstruct depth in the R1 upstream source adapter, retaining
capture identity and calibration provenance. Neither path belongs inside
pointcloud. PR371 currently requires registered rectified 640×480 JPEG plus
measured millimetre depth-zlib; 544×448 stereo needs a calibrated transformation
of the image grid and K, or a separately tested generic size extension. Do not
merely change the declared dimensions.

Start with camera-only VO after image/depth clock and geometry evidence pass.
Add IMU assistance only after the raw source is adapted with validated
capture time and camera-relative calibration; add optional body pose only
with its own capture-time chain. pointcloud consumes the same measured depth;
pointcloud_slam consumes native metric PointCloud2 plus tracking odometry and
keeps segments separate. A monocular estimated-depth stream is not a measured
depth substitute.

The local patch only removes waiting for a full 64 KiB of GStreamer stdout
and preserves a JPEG SOI split across reads. It adds no calibration, clock
mapping or SLAM capability and has not been deployed. Next gates are verified
interfaces/time/calibration, stationary tracking, authorized straight/turn
runs, then sustained fusion/mapping. Record tracking/fusion mode, restarts,
segment changes, frame loss and stage timings at every gate. Offline tests,
passive snapshots and visual_only fallback cannot pass the later gates.

An offline call using the actual MCP camera declarations was rejected by
PR371's input validator: both main/depth declare JPEG, and measured
`image/depth-zlib` is missing. Existing platform contract/IMU/depth-only
regressions passed (75 tests); no platform code was changed.

## Patch verification

The four final real-pipe regressions fail against pristine base code and pass
with this patch. R1/camera related tests pass: 128 on Python 3.12; the four new
tests also pass on Python 3.10. Independent review found no blocking issues
and separately reverted each fix in memory to confirm its regression fails.

A broad repository run with `pytest -q --continue-on-collection-errors` did
not pass: 437 failed, 4819 passed, 56 skipped, 90 errors, 918 subtests passed.
It includes missing optional/hardware packages, cross-bundle module import
collisions and callback TLS/fixture failures. All 76 collection-error file
names also occur when collecting a pristine base extraction (which has one
additional error); this does not establish that every execution failure is
pre-existing. The broad run is not a successful full regression. Complete
failure names/logs are retained locally; unrelated code was not modified.

Local evidence capture SHA-256:
`5a1e8984e732f4933dc08018cfa85b36e438b82f1cc6b9408bb09404b53304f5`
(`video-passive.pcap`, 6,317,288 bytes). Raw images/packet data remain local;
no credentials or raw scene images are included in this change.

At the final 22:09 check, all four container IDs and GStreamer PIDs matched
the initial inventory. Mapping/VO/pointcloud remained idle, navigation and
loco_servo command counts remained zero, and no owned probe process remained.
The owned remote packet file was removed after its local copy was verified.
