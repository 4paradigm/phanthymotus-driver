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
  merged `laser_scan`, lazy `camera_rgb`, and lazy `camera_depth`.
- Actuators: `base_drive`, `arm_control`, `gripper`, `lifting`, and `head`.

`robot_state` combines the low-rate battery, diagnostics, hardware events,
controller states, gripper feedback, and lifting feedback. High-rate or typed
streams remain separate. `laser_scan` consumes only `/scan`, whose publisher on
Panther is the existing `dual_lidar_merger`; this driver performs no lidar
fusion itself.

The first camera card started on the canvas creates/starts the host transient
unit `panther_orbbec.service` through `nsenter + systemd-run`, using the vendor
workspace and launch command documented for Panther. RGB and depth share that
source process. Stopping a card disarms only its forwarding path, and a bounded
watchdog can restart the host unit after a measured frame stall.

Every physical-motion action requires `confirm_motion: true`. Arm and head
commands also require a live `/robot_description`; missing limits cause a hard
rejection rather than a guessed fallback. Base movement is finite-duration and
publishes repeated zero commands when it ends or is cancelled.

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

