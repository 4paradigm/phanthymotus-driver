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

- Sensors: `joints`/`model`, consolidated `robot_state`, `imu`, `odom`,
  merged `laser_scan`, lazy `camera_color`, and lazy `camera_depth`.
- Actuators: `base_drive`, `arm_control`, `gripper`, `lifting`, and `head`.

`model` falls back to the bundled, stripped Panther URDF before the robot
publishes anything. Once a live `/robot_description` is available, reading the
model resource atomically switches both the returned URDF and the `joints`
rendering indices to that same live snapshot. The response includes its source
and SHA-256 digest so a renderer can identify the active morphology. Live
`/robot_description` also remains authoritative for motion limits. `joints`,
`imu`, `odom`, `robot_state`, and `laser_scan` are rate limited before crossing
the per-card domain-42 bridge.

`robot_state` combines the low-rate battery, compact diagnostics, hardware
events, controller states, gripper feedback, lifting feedback, and explicit
`not_ready_reasons`. High-rate or typed streams remain separate. `laser_scan`
consumes only `/scan`, whose publisher on Panther is the existing
`dual_lidar_merger`; this driver performs no second lidar fusion, but pools each
angular bin to its nearest valid return and reports scan quality.

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
| `ros-humble-nav-msgs` | `nav_msgs/msg/Odometry` on `/odom` | Required even though the navigation card was removed |
| `ros-humble-rmw-cyclonedds-cpp` | Robot-facing domain 0 process | Required |
| `ros-humble-rmw-fastrtps-cpp` | Agent Core domain 42 bridge process | Required |
| `python3-numpy`, `python3-opencv` | RGB/depth conversion and laser rendering | Apt-only; no duplicate pip wheels |
| `util-linux` | `nsenter` into the host namespaces for the lazy camera source | Required |
| `systemd` | `systemctl`/`systemd-run` clients for the owned host transient unit | Required; the driver does not run systemd in the container |

For a base-image update, measure package-layer changes with `docker history`
instead of recording a size that becomes stale with the moving base image.

Every physical-motion action requires `confirm_motion: true`. Arm and head
commands also require a live `/robot_description`; missing limits cause a hard
rejection rather than a guessed fallback. Base and arm movements are
asynchronous, return `action_id`, report completion, and can be stopped. Arm
controller state is restored to its pre-command snapshot. Base movement is
finite-duration and publishes repeated zero commands when it ends or is
cancelled. Arm/head `home` remains unavailable until a verified pose is placed
in `config.yaml`.

Motion-card angles are exposed in degrees. The ROS messages and live URDF still
use radians, so conversion happens only at the robot boundary. The head mapping
follows physical-robot testing rather than the vendor URDF axis labels:
`yaw_deg` controls the first head channel (`joint_head0`) and `pitch_deg`
controls the second (`joint_head1`).

Physical movement is additionally blocked by
`safety.commissioning_motion_enabled: false`. Do not enable it until the
commissioning velocity, duration and arm-delta caps have been approved on the
target robot.

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

