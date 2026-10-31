# S10 Pro driver

This is the final S10 Pro bundle. It combines the upstream isolated ROS reader
and skeleton mapping with the validated native TCP control cards, ACP
completion callbacks, four camera lanes, split raw/map LiDAR streams, lights,
charging and navigation.

| Card | Input | Output |
| --- | --- | --- |
| Battery | `/BATTERY_DATA` (`drdds/msg/BatteryData`) | `soc`, `soh`, `current`, `voltage`, `cell_vol`, `temperature`, `cycle` as JSON |
| Joints | `/JOINTS_DATA_10HZ` (`drdds/msg/JointsData`) and `/IMU` | `joints_info` JSON with joint names/angles/speeds/torques/temperatures; `joints` skeleton with 16 joints and IMU orientation |
| LiDAR | 22 historical/live PointCloud2 candidates; `/ACCUMULATED_POINTS_MAP` remains the map source | Split `lidar_cloud` and `lidar_map` `sensor/pointcloud` streams with bounded latest-frame workers; only one fresh cloud source is active at a time |
| IMU | `/IMU` (`sensor_msgs/msg/Imu`) | source `header.stamp`, quaternion (w,x,y,z), angular velocity, linear acceleration, Euler angles (radians), temperature |
| `loco_servo` | `control/velocity` (`motus.control/1`, twist) | bounded continuous velocity commands to the native S10 TCP controller; dry-run by default |
| Odom | `/MOTION_INFO` (`drdds/msg/MotionInfo`) | `state/odom` with measured body `vx`, `vy`, and `wz`; no fabricated pose |

`model` returns a mesh-free kinematic URDF used by the joint renderer. It has
the 16 actuated joints and limits, but deliberately no STL dependency. A fixed
arrow marks mechanical +X/front. The source joint order and angle mapping live
in `s10_skeleton.py`.
The skeleton represents posture and does not track global translation.

Battery values retain the vendor message units: voltage is 10 mV and current
is signed 10 mA; `soc` is percent. Missing SOH/cell voltages and IMU temperature
are null/empty, not invented measurements. Missing or invalid IMU source time
is `header.stamp: null`. Point coordinates remain in the input LiDAR frame;
there is no mounting transform, temporal accumulation or object recognition.
Joint latency and LiDAR coordinate refinements remain separate local work.

Motion uses native S10 TCP commands and reports long-running completion through
`/api/acp/complete`; locomotion movement accepts `yaw_rate_deg_s` in degrees per
second and a bounded duration. Head and tail lights are independent cards.
`loco_servo` is the streaming counterpart for visual navigation policies. It
subscribes on Core domain 42 through a loopback-only bridge process, requires
`confirm=true` before switching out of dry-run, and stops on its watchdog.
The `navigation.route` action sends a sequence of up to 64 vendor `Item` targets
in one native task request; each point has map coordinates, yaw, and point type,
while the route shares gait, speed, direction, obstacle, and navigation mode.

## Runtime

The shared MCP server binds all host interfaces on port 15745. `ROS_NAMESPACE` defaults to
`s10_pro`; output topics follow `/<namespace>/<stream>` (for example
`/s10_pro/imu`).
Four sensor calls use empty arguments. Start/stop/status/config lifecycle calls
remain supported by the shared MCP bundle; stopping a card releases only its
own subscription lease. Data is published only while the relevant input is
fresh. Input freshness uses local monotonic receipt time, not ROS clock time.

Fast DDS profile selection is process-global. The robot-domain participant is
intentionally reachable on the validated S10-facing subnet (and loopback) so it
can consume vendor ROS topics. The Core-domain participant is domain 42 and is
loopback-profiled; media/state publishers and the `loco_servo` subscriber are
isolated child processes. Media conversion and publication
use independent bounded lanes so a camera or point-cloud frame cannot block
control/state callbacks.

## Image and deployment

Build with the repository's normal driver build workflow. `driver.yaml` declares
`../../common` as a build-context extra; the Dockerfile expects the staged
`common/` directory. It pins the ROS Humble base image by digest, builds the six
required vendor message definitions in a build stage, and copies only their
runtime interfaces into the final image. See `vendor/SOURCE.md` for provenance.
The final image is larger than the base image because this driver needs the
Fast DDS RMW package plus PyYAML and Pillow; apt recommends and pip caches are
disabled/removed to keep that growth limited to this component.

`deploy/service.yml` is a service fragment for the normal platform deployment.
It does not mount host files; the interface, robot subnet, and native server
address must be set in its environment/configuration for the target host.
`main.py` generates the robot and Core Fast DDS profiles at startup.

This bundle uses the repository's shared registration helper. That helper does
not verify the registration endpoint's TLS certificate, and its MCP listener is
not loopback-only. The service uses host networking, so deploy only on a
firewalled, isolated host/network where port 15745 is not reachable by
untrusted clients. Per-driver CA pinning and loopback binding require a shared
runtime change and are not provided by this component.

The deployment values for `NETWORK_INTERFACE`, `ROBOT_NETWORK_CIDR`, and the
basic-server host must describe the isolated S10 network on the target host.

The same image supports AOS and an external Jetson without a deployment-mode
switch. `NETWORK_INTERFACE=auto` asks the kernel for the source address used to
reach the configured S10 basic-server host, then requires that address to map
to exactly one local interface. Thus AOS commonly resolves to `eth0`, while a
Jetson connected to the robot-side gigabit LAN commonly resolves to `eno1`.
The Jetson interface must have an unused static address in `10.21.33.0/24` and
the server remains `10.21.33.103`; do not use the NOS `10.21.31.0/24`
service/PPS network for the robot-facing port. Ambiguous or missing routes fail
startup instead of selecting an arbitrary interface.

`controlled_spatial` auto-selects local `drmap` only when the basic-server IP
is assigned to this host and both `drmap` and its map directory are present.
Otherwise the fixed allowlisted operations run over SSH to that server IP as
`user` (overridable with `S10_DRMAP_SSH_USER`). Supply the password through the
runtime secret `S10_DRMAP_PASSWORD`, or provision an SSH key; no password is
stored in the image, source, or service fragment. SSH uses `accept-new` host
key handling, so the first connection pins the presented host key and later
changes fail closed. The tutorial demonstrates `/var/opt/robot/data/maps`,
`drmap mapping start`, `drmap mapping stop`, and `drmap map list`; responses
from `drmap --format json map list` are parsed when valid. The SSH management
host can differ from the TCP controller address and is set with
`S10_DRMAP_SSH_HOST` (the tutorial session shows `10.21.33.106`). Map
activation, deletion, and mapping status remain unavailable until their exact
CLI schema and error semantics are verified on target firmware.

The native navigation card uses the currently loaded map and supports pose
initialization, goal/route execution, status, cancellation, and perception
status. The S10 software development guide documents the TCP `Item[]` route
contract and navigation status/location fields. It does not define map CRUD,
route persistence, or a mapping API. The separate [mapping tutorial](https://support.deeprobotics.cn/docs/?lang=zh-CN#/259675993)
documents only the interactive `drmap mapping start` / `drmap mapping stop`
workflow, says it must be run from the robot's map-management directory, and
does not specify the directory, executable deployment path, status/output
schema, map selection, or error contract. `controlled_spatial` therefore
implements only the demonstrated map listing and mapping start/stop operations;
it does not claim support for map CRUD or persistent saved routes. The existing
multi-point native route remains the supported navigation route contract.

## Validation

Run from repository root:

```sh
python -m pytest -q tests/test_s10_four_monitors.py tests/test_s10_registration.py tests/test_s10_monitor_runtime.py
```

Tests use synthetic messages, fake subscriptions and local HTTP. They do not
send robot commands. Earlier combined-image robot checks are historical and do
not validate this independent monitoring image; a new Bot build and robot-level
monitor validation are required before claiming this version accepted.
