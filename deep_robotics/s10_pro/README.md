# S10 Pro monitoring driver

This bundle exposes four read-only sensor cards and the joint-model resource.
It is independent of PR #451 and contains no movement actions, authorization,
automatic preparation, task database or ACP action-completion callbacks.
Agent Core and shared `common/` code are not modified.

| Card | Input | Output |
| --- | --- | --- |
| Battery | `/BATTERY_DATA` (`drdds/msg/BatteryData`) | `soc`, `soh`, `current`, `voltage`, `cell_vol`, `temperature`, `cycle` as JSON |
| Joints | `/JOINTS_DATA_10HZ` (`drdds/msg/JointsData`) and `/IMU` | `joints_info` JSON with joint names/angles/speeds/torques/temperatures; `joints` skeleton with 16 joints and IMU orientation |
| LiDAR | `/LIDAR/POINTS` (`sensor_msgs/msg/PointCloud2`) | `sensor/pointcloud`: little-endian uint32 record size/count, then float32 XYZ; at most 40,000 valid points per frame |
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

## Runtime

The MCP server listens on loopback port 15745. `ROS_NAMESPACE` defaults to
`s10_pro`; output topics follow `/<namespace>/s10_pro/<stream>`.
Four sensor calls use empty arguments. Start/stop/status/config lifecycle calls
remain supported by the shared MCP bundle; stopping a card releases only its
own subscription lease. Data is published only while the relevant input is
fresh. Input freshness uses local monotonic receipt time, not ROS clock time.

Fast DDS profile selection is process-global. The parent publishes to Core's
loopback-only domain 42, while a child process subscribes to the robot's domain
0 using the vendor UDP profile. No robot-domain publishers, services, or command
interfaces are created. The child IPC retains the newest pending record per
source with bounded buffers; this does not guarantee a particular display delay.

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

This monitor-only image does not provide motion cards. Replacing an existing
combined motion driver with it removes that driver's motion catalogue; this PR
cleanup does not deploy or replace any running robot container.

## Validation

Run from repository root:

```sh
python -m pytest -q tests/test_s10_four_monitors.py tests/test_s10_registration.py tests/test_s10_monitor_runtime.py
```

Tests use synthetic messages, fake subscriptions and local HTTP. They do not
send robot commands. Earlier combined-image robot checks are historical and do
not validate this independent monitoring image; a new Bot build and robot-level
monitor validation are required before claiming this version accepted.
