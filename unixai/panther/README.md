# UniX AI Panther Driver

ROS 2 Humble driver for the UniX AI Panther mobile manipulator.

The robot-facing process uses CycloneDDS on domain 0. A separate spawned
process publishes normalized sensor-card data to Agent Core on FastDDS domain
42 using `/opt/phanthy-motus/dds-local.xml`; raw robot command topics never
cross that bridge.

The robot-side CycloneDDS context loads the host Panther profile from
`/home/unix_ai/config/cyclonedds.xml`. Panther confines its domain-0 graph to
`127.0.0.1`; host networking and the shared profile are therefore both
required for discovery.

## Implemented cards

- Sensors: `joints`/`model`, compact `robot_state`, `battery`, `imu`, `odom`,
  merged `laser_scan`, lazy `camera_color`, and lazy `camera_depth`.
- Actuators: `base_drive`, `arm_control`, `gripper`, `lifting`, and `head`.
- Audio: `mic` publishes `audio/pcm-16k` from the host PulseAudio source;
  `speaker` plays a canvas-connected `audio/pcm-16k` stream through its sink.

The audio cards run in separate domain-42 processes and use the host PulseAudio
server; they never open ALSA devices directly. `PANTHER_PULSE_SERVER=auto`
(the default) probes the usual runtime socket and the vendor's randomly named
`/tmp/pulse-*/native` sockets on every start. Compose exposes host `/tmp` at
`/run/panther-host-tmp` read-only, so a reboot does not bake an obsolete socket
name into the deployment. The cookie is mounted read-only; override
`PANTHER_PULSE_COOKIE_PATH` if it differs from
`/home/unix_ai/.config/pulse/cookie`.

Verified physical routes on the development robot are the AISpeech USB
`Mic Arry 8` source and C-Media USB Audio Device sink. `platform-sound` is the
onboard sound card, not these devices. Override `PANTHER_PULSE_SOURCE` and
`PANTHER_PULSE_SINK` after checking `pactl list short sources/sinks` on another
robot. Pulse clients run as host UID 1001/GID 1002; override
`PANTHER_PULSE_UID/GID` if the audio user differs. PCM is signed 16-bit
little-endian, mono, 16000 Hz; microphone packets contain 2048 bytes (64 ms).
`speaker.start(input_topic=...)` subscribes to `audio_msgs/AudioChunk` on the
connected domain-42 topic. Playback validates format/sample alignment, handles
partial pipe writes, and reports buffer overflow rather than silently dropping
speech. Use `mic -> ASR` and `TTS -> speaker` on the canvas. For simultaneous
listening and speaking, insert the perception echo-cancellation card before ASR.

`speaker` also accepts `get_volume` and `set_volume(volume)`:

```json
{"action": "get_volume"}
{"action": "set_volume", "volume": 50}
```

Volume is an integer from 0 to 100 and controls only this card's playback
stream: 0 is silent, 50 halves PCM amplitude (about -6 dB), and the default
100 preserves the original PCM level. Responses include `volume` and
`volume_scope: "stream"`; this is not a reading of the host master volume.
The independent host/speaker volume still affects physical loudness.
Settings can be queried or changed while idle without opening an audio device,
and changes during playback apply to subsequent dequeued blocks. Audio already
buffered in PulseAudio may still play at the previous level. Volume survives
card stop/start and worker restarts, but resets when the driver restarts
(optional `audio.speaker_volume` config sets its initial value). It does not
change microphone samples, the input topic, or the host's shared sink volume.

## Computation inputs compatible with PR519

The existing camera, IMU and odometry cards also supply the numeric contracts
used by [Tianyi PR519](https://github.com/4paradigm/phanthymotus-driver/pull/519).
Agent Core is unchanged. Topic paths below use `/<namespace>/panther` as `P`.

| Card / output | Format and message | Geometry / units / timing |
|---|---|---|
| `camera_color`, `P/camera/rgb` | `image/jpeg`, `sensor_msgs/CompressedImage` | Rectified 640x480 JPEG; original source frame and capture header |
| `camera_depth`, `P/camera/depth` | `image/depth-zlib`, `sensor_msgs/CompressedImage` | 640x480 LE uint16 axial millimetres, zero invalid, zlib without a header prefix |
| `imu`, original `P/imu` | `data/json`, `std_msgs/String` | Existing body IMU display remains available |
| `imu.camera_imu`, `P/camera/rgbd/imu` | `sensor/imu`, `std_msgs/String`, `motus.imu/1` JSON | Raw camera gyro xyz in rad/s and acceleration xyz in m/s², source nanosecond times, optical-frame rotations and sensor origins from TF |
| `odom`, `P/state/odom` | `state/odom`, `std_msgs/String`, `motus.odom/1` JSON | Body `[vx,vy,null,null,null,wz]`, m/s and rad/s; source pose, quaternion, covariance and its own measurement time. Connect this port to ActuCore `navi` for motion feedback. |

The two Gemini cards each retain one image output. They use live
`/camera/{color,depth}/camera_info` to derive a common rectified pinhole ray
window. Color distortion is removed; depth is sampled by nearest neighbour.
The `camera_info` in tool declarations and action responses contains
`motus.camera/1`, actual output K/D, size, optical frame, camera identity,
registration and clock provenance. No generic Gemini calibration is hardcoded.
Missing or contradictory calibration pauses image forwarding and is reported.
The effective field can be slightly cropped to keep all rays within both images.

The Pixe XYZ stereo module has two separate cards, `camera_left` and
`camera_right`. Each publishes one 1280×960 JPEG stream at up to 10 Hz on
`P/camera/left` or `P/camera/right`. The two images from one source frame carry
the same timestamp. The corresponding `camera_info` declaration includes native
intrinsics and distortion derived from `M1/D1` or `M2/D2` in the host's vendor
`stereo.yaml`. On the vendor I420 path, the per-eye camera matrix is preserved and the
published distortion coefficients are zero. The YUYV path retains the original
lens distortion. If the calibration file is missing or invalid, the cards keep streaming and report unknown optics.

The cards share a domain-0 ROS image subscription to the 2560×960 I420 or YUYV
feed on `/ros_srt`. On the first card start, a background check reuses an
existing publisher or enables `cam_share_yuv` in the active compatible vendor
template. If needed, the vendor monitor starts or restarts only the head video
stream. This can briefly interrupt the vendor head video. Set
`plugins.stereo_camera.manage_vendor_source` to false when the source is
managed externally. Stopping the last card removes the subscription and
leaves the vendor stream available to its other consumers. The ROS callback
stores only the latest frame. A shared worker encodes the two eyes, and each
output uses its own publish bridge.

`plugins.camera.registered_depth` must describe a vendor-registered common
optical frame (true on the development robot). For uint16 input,
`source_depth_unit_mm` defaults to 1.0, matching this robot's vendor output;
adjust it only for a source that uses a different verified scale. Float32 ROS
depth is metres. Invalid, negative, nonfinite or unrepresentable depths become
zero. Output stays millimetres regardless of source encoding or scale.

Starting the existing `imu` card additionally arms the optional camera IMU
from `/camera/gyro_accel/sample`, sharing the existing Orbbec owner. It does
not substitute body IMU readings for camera motion. Source-frame raw vectors
are not pre-rotated: the consumer applies `camera_from_gyro` / `camera_from_accel`
once. Translations express the source's declared combined IMU origin in camera
coordinates. Absent source motion or TF is reported in `imu_status`; body IMU
display and valid RGB-D remain usable. The camera watchdog does not restart
RGB-D because this optional IMU is missing.

Camera declarations advertise the optional `state/extrinsic` sidecar
`P/camera/body_mount` under `vendor.body_mount`. It carries
`motus.camera_mount/1`, a capture-time `body_from_camera` 4x4 transform in metres,
or an explicit unavailable status without a retained matrix. Lookups use the
actual image timestamp, never latest TF. The development robot's camera and
`base_link` TF trees are disconnected. `estimated_body_mount` now explicitly
connects them numerically inside this driver: capture-time `base_link <-
link_rgbd`, a configured installation offset, then vendor `camera_link <-
optical`. The default offset assumes CAD `link_rgbd` and vendor `camera_link`
origins coincide. This is an **installation estimate**, not measured calibration:
packets report `source: configured_cad_estimate` and `accuracy_validated: false`.
Head joints still participate through the live capture-time TF chain. No
competing static transform is broadcast into the host graph. A real native
body-camera TF always takes precedence; disable `estimated_body_mount.enabled`
to require it. Set `translation_m` / `rpy_rad` only from known mounting data.
Missing either capture-time chain reports unavailable rather than reusing an
old transform. The original vendor URDF is unchanged.

`odom_interface.slam_pose` advertises `motus_slam/1` on the existing port and
`pose_drift: unbounded`. `/odom` supplies the original pose frame, `base_link`
child frame and measurement timestamp; `pose.stamp_ms` and
`vendor.motus_slam.stamp_ns` retain its own time independently of the 100 ms
window-averaged control twist. Invalid pose/time clears the computation pose;
source-time regression or frame changes create a new segment identity.
The control stream declares 10 Hz, robot time and a 200 ms driver-side
publication budget for the latest measurement. If the source clock cannot be
compared to wall time, `stamp_ms` uses callback receipt time and the sample's
`vendor.stamp_source` marks that fallback.

Image, IMU and odometry headers are preserved without receipt-time substitution
for computation poses or IMU. `clock_id: ros_domain_0` identifies the source
ROS timeline; mapping is explicitly unverified. This adapter does not claim
hardware synchronization, measured body installation calibration, or moving
SLAM accuracy. The source's numeric data remains available with that provenance.

## Continuous visual navigation: base_servo

Wire ActuCore `navi`'s `control/velocity` output to `base_servo`. Its
`control_interface` declares `motus.control/1`, six body-frame SI values in
`[vx, vy, vz, wx, wy, wz]` order. Default input is `/actucore/navi/cmd`;
`start(input_topic)` selects another canvas connection. Only vx/vy/wz are
actuated; nonzero vz/wx/wy are rejected. Output uses the existing matched
Panther `/cmd_vel` route (or its configured fallback) on robot domain 0.
Angular values here are **rad/s**, unlike the call-shaped `base_drive` form.

The card's bundle `start()` is inert. Explicit canvas/MCP `start` reserves the
chassis and attaches an isolated FastDDS domain-42 receiver. `pause` zeros the
output and releases ownership after three matched zero writes; `resume`
requires fresh readiness, ownership and newly generated commands. `stop`
also tears down the receiver. `config(dry_run, rotate_only)` requires a
confirmed pause/stop; dry-run runs command validation without robot output.

The default ceilings inherit `base_drive`: 0.50 m/s per linear axis and 45°/s
(0.785 rad/s) in yaw. The servo config can lower either ceiling. Acceleration
step bounds use configured acceleration / the
20 Hz maximum input rate; a 10 Hz source therefore ramps more conservatively.
The sink is rebased to zero for the first command and after each watchdog hold.
Explicit zero axes stop immediately. Navi uses the configured chassis half-width
of 0.33 m. The vendor CAD main `base_link` mesh spans about 0.454 m laterally;
0.33 m includes margin for the chassis. The separate turning radius of 0.37 m
comes from vendor Nav2. The footprint describes the chassis with its arms stowed.

Commands require finite generation and observation timestamps, source/sequence,
matching mode/width, and TTL in `(0, 500]` ms. Expired, old-session, regressed,
unsupported-axis and future-dated commands are rejected. A single latest-value
inbox bounds backlog and input application is capped at 20 Hz. The output
lease expires at the earliest command TTL, observation expiry or 300 ms of
silence. An independent thread emits zeros, including after receiver exit;
stop status remains unconfirmed if its output has no matched subscriber.
Confirmed here means zero commands reached a matched ROS reader, not measured
physical standstill. The vendor's own `cmd_vel_timeout` must be in `(0, 0.5]` s
before real stream start; it provides the controller-side process-death backstop.
No timeout parameter is changed by the card.

`base_drive`, `controlled_spatial` Nav2 goals and `base_servo` share an explicit
chassis gate. Stream start/resume refuses an existing call motion, pending
stop or Nav2 goal. A new explicit `base_drive` motion or Nav2 goal pauses the
stream first and refuses handover if zero output is unconfirmed. Once paused,
old queued commands cannot resume the stream. A Nav2 cancellation retains
ownership until its terminal result. This arbitration covers this driver's
cards; external vendor/remote-control publishers remain separate authorities.

## Controlled mapping and navigation

`controlled_spatial` is a new independent card; existing body, camera and lidar
cards retain their behavior. It uses the already-running vendor ROS services:
`/unix_slam/reset`, `set_localization_mode`, `save_map`, `load_map`,
`get_map_list`, `delete_map`, and Nav2 `/navigate_to_pose`. It does not launch
another navigation stack or change controller parameters.

- `start_mapping(map_name)`: require fresh `/scan` input, reject existing
  names, reset SLAM and enable mapping. Drive manually using the remote control.
- `tag_place(name, description)`, `list_tags`, `untag_place(name)`: store
  map-specific semantic tags with fresh map-frame position and yaw (radians).
- `stop_mapping`: save the native pose graph and map, archive native files and
  a display occupancy grid, then switch to localization. Failed operations
  retain an error rather than report a successful save.
- `list_maps`, `load_map(map_name)`, `unload_map()`, `delete_map(map_name)`: unload the active chassis map before deleting it, while preserving map-scoped
  tags, wait for a fresh pose after loading, and reject deletion of the active
  map or a map not managed by this card.
- `navigate_to_tag(tag_name)`, `navigate_to_pose(x, y, yaw)`: require a loaded
  map, fresh localization and live scans. Tag names match exactly. Nav2 terminal
  results determine completion; timeout requests cancellation and retains
  ownership until Nav2 confirms termination.
- `stop_nav`: cancel the owned navigation goal, including a goal still being
  accepted. Interrupt hooks share the existing `base` resource.
- `stop`: release the card's ROS node and scan relay once Nav2 has confirmed
  that any owned goal has ended. If cancellation is still pending, retry after
  it finishes. During mapping, use `stop_mapping` to save the active session
  before stopping the card. `start` recreates the node. Reload the map to
  verify localization before navigating again.
- `info` / `get_pose`: expose mapping, localization, scan freshness and
  navigation status. No stale pose is used for tagging or navigation.

Navigation completion verifies Agent Core over HTTPS. For the default loopback
endpoint, the driver trusts the self-signed certificate at
`/opt/phanthy-motus/data/certs/cert.pem` through the existing data mount. An
explicit `AGENT_CORE_CA_CERT` overrides this default and requires the certificate
hostname to match `AGENT_CORE_URL`, as does system CA verification for other hosts.

`spatial_map` provides a `sensor/mapping` stream with the same binary and
metadata contract as Tianyi/G1. Use `start`, `list_maps`, `select_map(map_name)`
and `stop` to display occupied cells, robot pose and semantic tags. Saved grids
are immutable display sources; a live grid cannot alter a saved-map view.

Persistence uses the same host mount as Tianyi/G1:
`/opt/phanthy-motus/data:/opt/phanthy-motus/data`. Native artifacts and occupancy
snapshots are archived under `/opt/phanthy-motus/data/maps/<map_name>/`, and
semantic tags are in `/opt/phanthy-motus/data/controlled_spatial.db`.
The vendor node independently writes `/home/unix_ai/map_dir`; Compose mounts
that path at the same location to archive/restore its files. Keep both trees
when backing up a robot. Runtime active-map state is deliberately not restored
as localized after a restart: explicitly load the intended map again.

### Single-lidar Panther

The development robot has one observed live lidar topic, `/lidar_front/scan`.
The vendor defaults start a dual-lidar merger that waits for an absent rear
input (`rear_missing`), so `/scan` and consequently SLAM have no data.
`controlled_spatial.single_lidar_topic` explicitly selects the live sensor and
relays its `LaserScan` to the vendor `/scan` input. `self_filter_box_m` uses the
TF at scan time to identify chassis returns in `base_link`. Those rays become
invalid (`NaN`). The sensor frame, timestamps and beam geometry are preserved
for the vendor SLAM node. `info` shows the number of filtered returns.
If the vendor merger starts publishing real merged scans, the relay yields.
Set this setting to `""` on a verified dual-lidar model with working native
fusion. A second USB serial interface is not evidence of a second physical
lidar; no rear-device alias or udev rule is installed by this driver.

Build a new map after enabling the filter because old saved maps can contain
chassis returns. Stay still when calling `start_mapping` until it reports
`mapping_start_pose` (also shown by `info`). The driver saves this starting
`base_link` pose with the map. Return to that physical position and heading
before calling `load_map(map_name)`, which seeds SLAM automatically.
An older map without a saved start pose can still be loaded for viewing.
Build and save a new map to use verified navigation.
The driver requires fresh TF within 0.25 m and 20 degrees of the seed for one
second before enabling navigation and tagging on a loaded map. `get_pose`
reports `localization_verified`. The driver also checks
`/bt_navigator/get_state` before sending a goal.

`model` falls back to the bundled, verbatim vendor `pmm_urdf` model before the robot
publishes anything. Once a live `/robot_description` is available, reading the
model resource atomically switches both the returned URDF and the `joints`
rendering indices to that same live snapshot. The response includes its source
and SHA-256 digest so a renderer can identify the active morphology. Live
`/robot_description` may narrow, but never widen, the bundled vendor motion
limits. Arm execution uses the intersection of both limit sets. `joints`,
`imu`, `odom`, `robot_state`, `battery`, and `laser_scan` are rate limited before crossing
the per-card domain-42 bridge.

Each Agent Core bridge lane creates one ROS publisher for an allowlisted
`topic_out` and no subscription, parameter service, or rosout endpoint. A DDS
participant does not receive application samples without a DataReader, so
robot command topics have no ingress path in these bridge processes. The
FastDDS profile additionally confines the domain-42 traffic to loopback.
Ordinary sensor `start` returns `waiting_for_data` until the first fresh sample
instead of failing the card lifecycle.

`robot_state` contains compact readiness, controller, diagnostic, and hardware
summaries. Battery measurements use the separate `battery` card; gripper and
lifting feedback are available from their actuator cards. `laser_scan`
projects valid returns from `/scan` to XYZ in the scan frame, with Z set to
zero. The card publishes `sensor/pointcloud` at up to 5 Hz on
`/<namespace>/panther/laser_scan/pointcloud`, keeping the binary stream separate
from the legacy JSON topic. ROS consumers on
the body domain can subscribe to `/<namespace>/panther/laser_scan/points`
as `sensor_msgs/msg/PointCloud2`, with the source frame and scan timestamp.

The first camera card started on the canvas reuses an already-active host
`panther_orbbec.service` without taking ownership. Otherwise it creates a
uniquely named host transient unit through `nsenter + systemd-run`, using the
vendor workspace and launch command documented for Panther. Only that unique
unit may be stopped or watchdog-restarted by this driver; a loaded but inactive
host unit is left untouched. RGB and depth share the source process. Malformed
frame diagnostics are bounded and sampled at the first failure and every 100th
failure rather than logged once per frame.

## Container dependencies

Runtime Python and ROS dependencies are installed only through apt; there is no
second pip requirements layer. This avoids duplicate NumPy/OpenCV binaries and
keeps their ABI aligned with ROS Humble. The non-obvious packages are retained
for these concrete runtime paths:

| Package | Runtime reason | Image impact policy |
|---|---|---|
| `ros-humble-nav-msgs` | `nav_msgs/msg/Odometry` on `/odom` | Required for measured body velocity and pose |
| `ros-humble-nav2-msgs` | `NavigateToPose` action for `controlled_spatial` | Panther navigation runtime dependency |
| `ros-humble-lifecycle-msgs` | Check that the host Nav2 navigator is active | Explicit dependency already present in the tested driver image |
| `ros-humble-std-srvs` | Vendor SLAM `Trigger` / `SetBool` services | Panther mapping/localization runtime dependency |
| `ros-humble-tf2-ros` | Capture-time camera/IMU transforms and map-frame localization | Panther TF runtime dependency |
| `ros-humble-rcl-interfaces` | Read `cmd_vel_timeout` before arming `base_servo` | Panther controller-timeout verification dependency |
| `ros-humble-rmw-cyclonedds-cpp` | Robot-facing domain 0 process | Required |
| `ros-humble-rmw-fastrtps-cpp` | Agent Core domain 42 bridge process | Required |
| `python3-numpy`, `python3-opencv` | RGB/depth conversion and laser rendering | Apt-only; no duplicate pip wheels |
| `util-linux` | `nsenter` into the host namespaces for the lazy camera source | Required |
| `systemd` | `systemctl`/`systemd-run` clients for the owned host transient unit | Required; the driver does not run systemd in the container |

For a base-image update, measure package-layer changes with `docker history`
instead of recording a size that becomes stale with the moving base image.
These ROS packages can increase image size when absent from the base; no
zero-growth claim is made. The follow-up review fix adds no packages or mounts.

Physical capabilities are registered only when their plugin has
`enabled: true`. Motion cards are declared as `type: actuator`, so Agent Core
applies its platform-level execution confirmation; Panther does not expose or
inject a second `confirm_motion` parameter. After confirmation, commands are
still rejected unless live robot/controller state and configured limits pass
their card-specific checks.
Arm and head commands
also require a live `/robot_description`; missing limits cause a hard rejection
rather than a guessed fallback. Base and arm movements are
asynchronous, return `action_id`, report completion, and can be stopped. Arm
controller state is restored to its pre-command snapshot. Base movement is
finite-duration. A stop remains pending until its publisher has a matched DDS
subscriber, then publishes zero velocity for the configured stop interval;
subscriber loss is never reported as a confirmed stop. Arm `home` retains the
same per-command delta limit as ordinary moves because its all-zero candidate
pose has not been independently commissioned. Head `home` uses its configured
low-speed profile.

The mounted robot-side CycloneDDS profile is followed by a bundled override
that sets tracing verbosity to `none` and its output to `/dev/null`, preventing
a host profile from enabling high-volume per-message container logs.

Panther explicitly advertises `localhost` by default through
`PANTHER_MCP_ADVERTISE_HOST`, matching its host-network Agent Core deployment.
Set that variable to a host address reachable by Agent Core when the two run on
different machines. This Panther-specific setting does not change the shared
registration default used by other drivers.

Motion-card angles are exposed in degrees. The ROS messages and live URDF still
use radians, so conversion happens only at the robot boundary. The head mapping
follows physical-robot testing rather than the vendor URDF axis labels:
`yaw_deg` controls the first head channel (`joint_head0`) and `pitch_deg`
controls the second (`joint_head1`).

`arm_control move` exposes one eight-axis pose. `side=left` and `side=right`
apply it directly to one arm; `side=both` mirrors it across the sagittal plane
using the vendor URDF joint axes. `move_separate` exposes the full independent
left-eight plus right-eight target. Legacy vector arguments remain accepted by
the runtime but are not shown in the ordinary canvas form. Each form field
states the corresponding vendor-URDF range; shared fields show separate
left/`both` and right ranges where they differ. Every completed target is
checked against the vendor/live limit intersection before an action is queued.
The canvas uses short English semantic fields (`shoulder_rot`, `shoulder_abd`,
`shoulder_raise`, `elbow_rot`, `elbow_flex`, and `wrist_1` through `wrist_3`).
Independent-arm fields add an `l_` or `r_` prefix. Their compact descriptions
put the degree range first so narrow cards show the safety limit. The old
`j0_deg`/`la0_deg`/`ra0_deg`-style keys remain accepted as hidden API aliases.

## Deliberately deferred

- Continuous 50 Hz dual-arm servo: watchdog/hold behavior needs a hardware test.
- Emergency-stop card: no trustworthy ROS emergency-stop state or command was
  identified. Driver `stop` actions are not represented as hardware e-stop.

## Checks

```bash
python3 scripts/check_service_yml.py unixai/panther
python3 tests/test_panther_driver.py -v
```

Deployment uses the image's `/deploy/service.yml` fragment. The repository
`deploy/run-pr-image.sh` helper extracts that fragment, substitutes the tested
image reference, and starts it as an isolated Compose project.


## One-shot tabletop pick

`tabletop_pick` uses the existing Panther color and depth cameras and the
existing Perception VOP tool. Start both camera cards and then start
`tabletop_pick`. Each observation uses a synchronized pair of rectified
frames and the intrinsics derived from the camera's `CameraInfo`. The card
uploads the chosen JPEG to local Perception and calls `recognize_by_photo`.
Perception keeps snapshots according to its existing upload retention policy.
When upload authentication is enabled, pass the same `ACCESS_TOKEN` to the
Panther container.

- `preview` takes the exact VOP class as `object_name` and optional `color`.
  It returns the estimated tool point and approach point in `base_link`.
- `pick` takes the same fields and `confirm_motion: true`. It opens, approaches,
  descends, closes, and lifts once. It reports completion through ACP and
  keeps the gripper closed. `grasp_checked: false` means positional feedback
  was used and object presence was not independently confirmed.
- `stop` cancels the active pick through the trajectory action. A gripper
  position command already received by its controller may continue.

Configure `plugins.tabletop_pick` with the measured color-optical-to-base
transform at the fixed head/lift pose, TCP orientation, workspace, arm side
and per-class gripper targets. Calibrate the selected gripper's feedback
endpoints separately from the normalized command values. Check the active VOP
vocabulary before commissioning an object class, such as `bottle` or `mouse`.

Run MoveIt on the robot ROS domain with the full Panther collision model,
current robot state, configured single-arm group and tool link, and the actual
tabletop planning scene. The card rejects incomplete Cartesian paths and
checks returned waypoints against the device's arm limits. Confirm the
emergency-stop diagnostic name and verify its normal and abnormal states on
the robot. After checking `preview` and validating the workcell, enable
`allow_motion` for supervised commissioning. It defaults to `false`.
