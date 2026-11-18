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

Physical motion is disabled by default. Standard deployment enables it only
when the mounted host file
`/opt/phanthy-motus/data/panther-motion.authorization` contains exactly
`ENABLE_PANTHER_MOTION`. Removing that file and restarting disables motion.
This explicit host action cannot be inherited accidentally from the Compose
environment and is not exposed as an MCP argument. Clicking Execute is the
per-command user confirmation, so `confirm_motion` is not displayed on the
cards; the plugin injects it only at the low-level device boundary. Commands
are still rejected unless the deployment gate and live
robot/controller state and configured limits pass their card-specific checks.
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
The canvas uses English semantic titles (`Shoulder Rotation`, `Shoulder
Abduction`, `Shoulder Raise`, `Elbow Rotation`, `Elbow Flexion/Extension`, and
`Wrist 1` through `Wrist 3`) while retaining the existing parameter keys for
API compatibility.

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

