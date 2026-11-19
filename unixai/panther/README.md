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
  `laser_scan` point cloud, lazy `camera_color`, and lazy `camera_depth`.
- Actuators: `base_drive`, `arm_control`, `gripper`, `lifting`, and `head`.
- Audio: `mic` publishes `audio/pcm-16k` from the host PulseAudio source;
  `speaker` plays a canvas-connected `audio/pcm-16k` stream through its sink.
- Media: `vision_capture` takes JPEG photos, records silent MP4 clips, and lists or deletes saved files.

The audio cards run in separate domain-42 processes and use the host PulseAudio
server at `/run/user/1001/pulse/native`; they never open ALSA devices directly.
The Compose fragment mounts the Pulse socket directory and the authentication
cookie. Set `PANTHER_PULSE_COOKIE_PATH` to the host cookie path if it differs
from `/home/unix_ai/.config/pulse/cookie`. Set `PANTHER_PULSE_SOURCE` and
`PANTHER_PULSE_SINK` to override the configured Pulse device names after
checking `pactl list sources` and `pactl list sinks` on the robot.
Pulse clients run as host UID 1001/GID 1002 because the host server rejected
container root; override `PANTHER_PULSE_UID` and `PANTHER_PULSE_GID` if the host
audio user differs. The driver and ROS bridge retain their existing identities.
The reported `platform-sound` names have not yet been verified against the physical M8 mic
and speaker. Before enabling a project, verify container-side `pactl info`,
then test `mic -> ASR` and `TTS -> speaker` on Panther. For simultaneous
listening and speaking, insert the perception echo-cancellation card before
ASR and check interaction with the robot's native AIUI conversation service.

`model` and `joints` use the same bundled articulated display model throughout
the session. The live `/robot_description` can encode the lift and fingers as
fixed joints, so it is kept separate from Canvas rendering. The display model
uses the measured yaw/pitch order, the bundled elbow axes, and maps gripper
feedback as `1 - position` before expanding the finger linkage.
These are display-only corrections. `joints` combines `/joint_states` with fresh
lift and gripper feedback, and `joints info` reports `body_joint` in metres.
The display model has a cuboid base outline (650 × 510 mm in plan, with an
approximate 200 mm height) and a fixed guide column ending at the neutral
pose's head level. The torso, head and arms are on the separate `body_joint` prismatic branch. A
renderer with prismatic support moves that branch along the column; the current
rotation-only Canvas shows the column and reports lift height without animating it.
The response includes the display model's SHA-256 digest. Live
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
lifting feedback are available from their actuator cards. `laser_scan` prefers
fresh `/scan` from the vendor merger and falls back to `/lidar_front/scan`; it
does not fuse the feeds a second time. The first Canvas output is the native
`sensor/pointcloud` stream; the second keeps the scan JSON with nearest-return
pooling and quality metrics. Each point-cloud packet contains only the current
scan projected in its own XY plane (`z=0`), capped at 20,000 points and a 12 m
range by default. `info` reports the source, scan frame and point count. The
current Canvas renderer replaces its last displayed frame when a new non-empty
cloud arrives.

The first camera card started on the canvas reuses an already-active host
`panther_orbbec.service` without taking ownership. Otherwise it creates a
uniquely named host transient unit through `nsenter + systemd-run`, using the
vendor workspace and launch command documented for Panther. Only that unique
unit may be stopped or watchdog-restarted by this driver; a loaded but inactive
host unit is left untouched. RGB and depth share the source process. Malformed
frame diagnostics are bounded and sampled at the first failure and every 100th
failure rather than logged once per frame.

To take a photo, start `camera_color` (or use an already-running RGB publisher),
then execute `vision_capture → capture_photo`. The card waits up to 3 seconds
for a new frame and returns `file_path` and the ROS frame timestamp. The JPEG is
saved under `/opt/phanthy-motus/data/vision_capture/panther/photos/` on the robot's
existing persistent data mount. `record_video` takes `duration_s` (1–30 s,
default 5 s) and reports a silent H.264 MP4 in the sibling `videos/` directory
on completion. `stop` cancels an in-flight recording and discards its temporary
files. Both capture actions share the RGB source and leave its preview running.
`list` returns up to 100 of the newest saved media files. Pass an exact
`filename` from that result as `name` to `delete` to remove just that file.

## Container dependencies

Runtime Python and ROS dependencies are installed only through apt; there is no
second pip requirements layer. This avoids duplicate NumPy/OpenCV binaries and
keeps their ABI aligned with ROS Humble. The non-obvious packages are retained
for these concrete runtime paths:

| Package | Runtime reason | Image impact policy |
|---|---|---|
| `ros-humble-nav-msgs` | `nav_msgs/msg/Odometry` on `/odom` | Required even though the navigation card was removed |
| `ros-humble-rmw-cyclonedds-cpp` | Robot-facing domain 0 process | Required |
| `ros-humble-rmw-fastrtps-cpp` | Agent Core domain 42 bridge process | Required |
| `python3-numpy`, `python3-opencv` | RGB/depth conversion | Apt-only; no duplicate pip wheels |
| `ffmpeg` | H.264 MP4 encoding for `vision_capture` | Runtime only |
| `util-linux` | `nsenter` into the host namespaces for the lazy camera source | Required |
| `systemd` | `systemctl`/`systemd-run` clients for the owned host transient unit | Required; the driver does not run systemd in the container |

For a base-image update, measure package-layer changes with `docker history`
instead of recording a size that becomes stale with the moving base image.

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
python3 -m pytest unixai/panther/test_panther_vision_capture.py
```

Deployment uses the image's `/deploy/service.yml` fragment. The repository
`deploy/run-pr-image.sh` helper extracts that fragment, substitutes the tested
image reference, and starts it as an isolated Compose project.

