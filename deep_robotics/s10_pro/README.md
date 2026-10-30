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

`model` returns the S10 skeleton URDF used by the joint renderer. It is a
simplified line model, not a full visual mesh. A fixed arrow marks mechanical
+X/front. The source joint order and angle mapping live in `s10_skeleton.py`.
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

## Runtime

The MCP server listens on loopback port 15745. `ROS_NAMESPACE` defaults to
`s10_pro`; output topics follow `/<namespace>/<stream>` (for example
`/s10_pro/imu`).
Four sensor calls use empty arguments. Start/stop/status/config lifecycle calls
remain supported by the shared MCP bundle; stopping a card releases only its
own subscription lease. Data is published only while the relevant input is
fresh. Input freshness uses local monotonic receipt time, not ROS clock time.

Fast DDS profile selection is process-global. The parent and its domain-42
publishers are loopback-only, while the robot ROS participant uses the validated
S10-facing interface and vendor UDP profile. Media conversion and publication
use independent bounded lanes so a camera or point-cloud frame cannot block
control/state callbacks.

## Image and deployment

Build with the repository's normal driver build workflow. `driver.yaml` declares
`../../common` as a build-context extra; the Dockerfile expects the staged
`common/` directory. It pins the ROS Jazzy base image by digest, builds the six
required vendor message definitions in a build stage, and copies only their
runtime interfaces into the final image. See `vendor/SOURCE.md` for provenance.

`deploy/service.yml` is a service fragment for the normal platform deployment.
Before starting, provision these host files:

- `/opt/phanthy-motus/dds-local.xml`: platform loopback-only DDS profile.
- `/opt/phanthy-motus/s10-reader-udp.xml`: vendor UDP profile (template included).
- `/opt/phanthy-motus/s10-pro/config.yaml`: copy/edit the shipped config for the device.
- `/opt/phanthy-motus/s10-pro/core-cert.pem`: the trusted Core leaf certificate,
  obtained from the operator-managed Core installation through a trusted path.

Registration verifies HTTPS using the explicitly mounted Core leaf certificate.
For CA-signed Core deployments use `S10_REGISTRATION_CA_FILE` instead of
`S10_REGISTRATION_PINNED_LEAF_FILE`. Registration retries on failure; the MCP
endpoint remains available. No private certificate or token is committed.

The deployment values for `NETWORK_INTERFACE`, `ROBOT_NETWORK_CIDR`, and the
basic-server host must describe the isolated S10 network on the target host.

## Validation

Run from repository root:

```sh
python -m pytest -q tests/test_s10_four_monitors.py tests/test_s10_registration.py tests/test_s10_monitor_runtime.py
```

Tests use synthetic messages, fake subscriptions and local HTTP. They do not
send robot commands. Earlier combined-image robot checks are historical and do
not validate this independent monitoring image; a new Bot build and robot-level
monitor validation are required before claiming this version accepted.
