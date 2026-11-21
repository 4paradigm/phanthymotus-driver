# Panther spatial/audio validation — 2026-11-21

Branch: `feat/add_phanther_spatial`, based on main `25c56b0f`.
Repository edits are confined to `unixai/panther`.

## Camera mount estimate and base_servo follow-up

Read-only live URDF/static-TF inspection located the body/head chain ending
in `link_rgbd`, and the separate Orbbec root `camera_link`. No measured transform
between those origins was found. The configurable `estimated_body_mount`
assumes coincident CAD/source origins by default and composes both capture-time
chains inside this driver. It does not broadcast a new host TF or edit the
vendor URDF. Native connected TF takes precedence. Packets explicitly retain
`source=configured_cad_estimate` and `accuracy_validated=false`.

The 20-second read-only estimate probe produced 79 body-mount packets,
39 color frames, 40 depth frames, 1972 camera IMU packets and 197 odometry
packets. The last estimated optical origin in `base_link` was approximately
[0.289892, -0.013848, 1.645219] metres. This numerical chain validation does not
verify head-axis modelling, mounting accuracy or moving-map quality.

Read-only parameter queries confirmed vendor
`/movebase3_drive_controller.cmd_vel_timeout=0.5` seconds,
Nav2 `local_costmap.robot_radius=0.37` metres, and Orbbec
`depth_registration=true`, `enable_depth_scale=true`, `time_domain=global`.
No parameter was changed. Real stream start verifies the controller timeout.

`tests/probe_base_servo.py` exercised the real spawned domain-42 receiver,
JSON/ControlSink/lease handling and robot-domain Twist encoding against only
`/panther_validation/base_servo/input` and `/panther_validation/base_servo/output`.
Its output boundary was a private recorder, not the actual base command topic.
It received 70 output frames (21 nonzero), observed zero output 51.7 ms after
the final nonzero frame, and confirmed pause/stop plus receiver/thread cleanup.
That interval is not a physical braking or generation-to-stop measurement.
Twelve base-servo regressions also passed inside the robot image. ActuCore's
actual `control_stream.negotiate` accepted the descriptor; its 10 Hz TTL is
200 ms, within this sink's bounds.

Focused regressions cover stale/future/old-session commands, TTL/observation
expiry, unsupported axes, sequence/priority checks, immediate zero, acceleration
bounds, receiver exit, independent watchdog, unconfirmed stop ownership and
manual/Nav2 arbitration. Estimated-mount composition/native precedence is
also covered. No real base movement, visual arrival, braking-distance test,
installation calibration or new production deployment was performed.

Final local validation: 51 focused tests plus 48 existing Panther regressions
passed (99 total); Compose validation and `git diff --check` passed. The final
extra regression confirms that zero input without a matched reader cannot
claim a confirmed stop. All submitted files are under `unixai/panther`.

## PR519 sensor contracts follow-up

The new `tests/probe_sensor_inputs.py` ran as a temporary read-only process
inside the existing driver image, with changed Python modules loaded from
`/tmp/panther-sensor-validation` via the existing read-only host-tmp mount.
It did not replace production files, start/restart camera services, publish
commands, drive the robot or publish to the production Agent Core topics.

The first 20-second run received/encoded 39 color frames, 39 depth frames,
1969 camera IMU packets and 197 odometry packets. JPEG decoded to 480x640x3;
zlib depth decoded to 480x640 LE uint16 with 261751 valid pixels, valid depth
1366–11576 mm in the sampled last frame. Color and depth shared identical
rectified K: fx 522.753742, fy 522.827600, cx 323.204007, cy 242.011975.
Source K and distortion were read live; these numbers are evidence for this
device only, not calibration defaults. Original source capture headers were
retained. Numeric reports contain no image or audio recordings.

Camera IMU TF supplied rotation and translation from
`camera_accel_gyro_optical_frame` to `camera_color_optical_frame`.
In these initial runs the independent TF trees were disconnected: body-mount
sidecars correctly reported unavailable and omitted their matrix. No installation
estimate was enabled in these runs. Clock mapping remains explicitly unverified;
these stationary format checks do not validate motion accuracy or fusion.

The repeat run after depth-unit normalization received 40 color frames,
39 depth frames, 1965 camera IMU packets and 197 odometry packets. JPEG/depth
decoding and shared K passed again; all 12 numerical/port tests also passed
inside the robot image. Optional-IMU watchdog isolation was subsequently
covered by an additional local lifecycle test.

Thirteen focused local tests cover common-ray geometry, calibration rejection,
crop/resize K, depth stride/endianness/units/invalid values, IMU vectors/TF/time,
odometry pose/time/null semantics, unavailable mount clearing and public ports.
The complete local Panther-focused run passed 37 tests, plus the existing
48-test Panther suite (85 total); Compose checks and `git diff --check` passed.
No new image
has been built or deployed for this follow-up.

## Speaker volume follow-up, local validation

Added per-stream `get_volume` / `set_volume` with integer range 0–100,
default 100 (unchanged PCM). Five focused tests cover signed PCM attenuation,
silence/unity, invalid input, idle configuration without starting hardware,
worker command/acknowledgement and retained settings, and mic schema isolation.
The focused suite now passes 24 tests; the existing Panther suite passes 48.
These volume changes have not been deployed or tested on physical hardware.
Microphone capture and Agent Core rendering remain unchanged.

## Audio, real hardware

An independent temporary container used the existing driver image with the new
`audio.py`, host PulseAudio socket/cookie mounts, and a private domain-42 topic.
It did not replace or restart the production driver container.

- Mic: AISpeech Mic Arry 8 USB source, S16LE mono 16000 Hz.
- 87 valid packets, 2048 bytes each, 178176 bytes / 5.568 seconds received.
- Ambient microphone signal RMS 182.04, peak 1252; zero malformed packets.
- Speaker: C-Media USB sink, 16 test packets, zero dropped packets, queue drained.
- Sink-monitor recording detected the quiet 440 Hz playback: peak 963.
- No room audio recording was saved; only signal statistics were retained.

Probe: `tests/probe_audio.py`. Physical audibility and ASR/TTS conversation
quality still need an operator's listening check.

## Mapping, real hardware, no driving

Only `/lidar_front/scan` produced actual scans. The vendor dual-lidar merger
reported `rear_missing` and published no `/scan` samples. The new card's explicit
single-lidar relay restored the vendor SLAM input without changing sensor frame,
timestamps or any existing card implementation.

The final code completed the stationary workflow:
`start_mapping -> tag_place -> stop_mapping -> load_map -> localized`.
The final map was 216 x 123 cells at 0.05 m/cell. The `stationary_start` tag,
native map artifacts, occupancy snapshot and SQLite records were saved and
reloaded. Pose came from the fresh `map -> base_link` TF chain; the vendor
`/pose` topic did not provide continuous stationary updates.

Two stationary test maps remain on the robot:
`validation_20261121_184417` and `validation_20261121_184846`.
Native files are under `/home/unix_ai/map_dir`; isolated test archives/SQLite
are under `/tmp/panther-spatial-validation`. These are interface-validation
maps of the stationary surroundings, not a commissioned navigation map.
Deployment defaults use `/opt/phanthy-motus/data/maps` and
`/opt/phanthy-motus/data/controlled_spatial.db`, matching Tianyi/G1's mount.

The tentative rear-port udev rule and device alias were removed after hardware
clarification, and the temporary rear-driver recovery process was stopped.
The driver does not install a rear-device rule or infer lidar hardware from
USB serial interface numbers.

Probe: `tests/probe_spatial.py`. It never sends a velocity or navigation goal.

## Navigation lifecycle, isolated ROS integration

`tests/probe_navigation.py` ran against a private fake Nav2 action server on
ROS domain 91, action `/panther_validation/navigate_to_pose`.
All five cases passed: succeeded, aborted, rejected, cancelled, timed out.
Each emitted exactly one completion callback with its matching action ID.
No goal was sent to the physical robot's Nav2 server.

Actual driving, obstacle avoidance, arrival accuracy and interruption during
physical movement remain to be verified on a complete map with an operator.

## Local checks

Review follow-up (2026-11-21): 54 focused tests and 48 existing Panther tests
passed (102 total), and `scripts/check_service_yml.py unixai/panther` passed.
Regression coverage checks that `controlled_spatial.start` returns `ready`
while preserving detailed mapping status under `data`, and that audio, map
publisher and base-servo spawned entry points install `common.logsafe` before
child setup. These checks require no ROS installation or physical movement.
The log wrapper protects Python stdout/stderr; this is not a guarantee for
arbitrary native-library writes or a repair for existing Docker logs.
This follow-up adds no Docker packages, mounts or card declarations.

- Existing Panther regression suite: 48 passed.
- New focused tests: 19 passed (store/schema compatibility, exact tag lookup,
  stale pose/input rejection, map isolation, cancellation/timeout ownership,
  single-lidar relay, occupancy geometry and sensor/mapping binary payload).
- All 16 pre-existing MCP card declarations are identical to main.
- Python sources parse, YAML parses, `git diff --check` passes.
- Production image has not been rebuilt or deployed; Dockerfile includes the
  added message services and Nav2/TF runtime dependencies for the next build.
