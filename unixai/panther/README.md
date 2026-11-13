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

`model` uses the bundled, stripped Panther URDF so the skeleton is available
before the robot publishes anything. Live `/robot_description` remains the
authoritative source for motion limits, and differences are reported. `joints`,
`imu`, `odom`, `robot_state`, and `laser_scan` are rate limited before crossing
the per-card domain-42 bridge.

`robot_state` combines the low-rate battery, compact diagnostics, hardware
events, controller states, gripper feedback, lifting feedback, and explicit
`not_ready_reasons`. High-rate or typed streams remain separate. `laser_scan`
consumes only `/scan`, whose publisher on Panther is the existing
`dual_lidar_merger`; this driver performs no second lidar fusion, but pools each
angular bin to its nearest valid return and reports scan quality.

The first camera card started on the canvas creates/starts the host transient
unit `panther_orbbec.service` through `nsenter + systemd-run`, using the vendor
workspace and launch command documented for Panther. RGB and depth share that
source process. Stopping a card disarms only its forwarding path, and a bounded
watchdog can restart the host unit after a measured frame stall.

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

