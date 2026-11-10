# G1 SLAM input verification and read-only capture

This driver branch implements two **candidate** inputs for the generic
`pointcloud_slam` Perception card:

- `/<namespace>/slam/mapping/odom` — ROS `std_msgs/msg/String`, `motus.odom/1`
- `/<namespace>/lidar/cloud_native` — ROS `sensor_msgs/msg/PointCloud2`
- `/<namespace>/lidar/cloud_native/metadata` — ROS `std_msgs/msg/String`

The existing `/<namespace>/loco/state` and `/<namespace>/lidar/cloud` outputs
are unchanged. Do not connect `sensor/pointcloud` to the native SLAM input.

**Not ready for mapping:** The G1 odometry adapter deliberately reports
`vendor.motus_slam.status = unavailable` and declares
`odom_interface.slam_pose.available = false` until measurement-time clock
semantics, tracking-quality/status, pose reset/relocalization boundaries and
measured lidar extrinsics are proven on the installed robot. This guard must
not be disabled merely to start RTAB-Map.

## Offline contract tests

Run from the driver repo root on the branch corresponding to PR #499:

```bash
python3 -m pytest -q \
  tests/test_spatial_slam.py \
  tests/test_g1_mapping_odom.py \
  tests/test_g1_slam_input.py \
  tests/test_g1_pointcloud_layout.py \
  tests/test_slam_input_metadata.py \
  tests/test_g1_slam_capture_probe.py
```

The canonical contract is `common/spatial_slam.py`. Its source must stay
**byte-identical** to main repository
`perception/plugins/pointcloud_slam_contract.py`. For an additional
cross-repository test, set `PERCEPTION_REFERENCE_ROOT` to the local
`phanthymotus` checkout at the tested PR #371 SHA before running pytest.

## On-robot collection (no motion commands)

Before collecting, check that a G1 image **built from PR #499** is actually
running. A pushed PR commit does not update the robot.

```bash
docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
docker inspect embodied-unitree-g1 --format '{{.Config.Image}}'
docker exec -it embodied-unitree-g1 bash
```

Inside the container, where the Unitree SDK and CycloneDDS libraries exist:

```bash
python3 /work/slam_capture_probe.py --iface eth0 --duration 30 \
  --output-dir /opt/phanthy-motus/data/slam_capture
```

Use the actual robot-facing interface instead of `eth0` if different.
The script subscribes to the following native **Unitree domain 0** DDS topics,
never starts mapping or issues robot motion commands:

- `rt/unitree/slam_mapping/odom`: source header, position, xyzw orientation,
  covariance
- `rt/utlidar/cloud_livox_mid360`: header and field/layout information;
  **no raw cloud payload** in this small JSONL
- `rt/slam_info` and `rt/slam_key_info`: capped vendor status strings

All records include `received_ns` and `received_monotonic_ns`, which are
*receipt* timestamps, not assertions about sensor measurement time. A
`sampler_generation` is not a robot clock epoch. The queue (256 entries),
duration (up to 120 seconds), row count (up to 20,000) and status payload length
(4096 characters) are bounded. If some vendor topics do not publish, their
counts remain zero; this does not prove those services are unsupported.

To capture full native PointCloud2 for offline decoding, use ROS 2 domain 42
and record the three ROS outputs with `ros2 bag record`. For example:

```bash
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
ros2 topic list -t | grep -E 'slam/mapping/odom|lidar/cloud_native'
# Replace <namespace> with the namespace actually reported by ROS:
ros2 bag record -o /opt/phanthy-motus/data/slam_capture/static \
  /<namespace>/slam/mapping/odom \
  /<namespace>/lidar/cloud_native \
  /<namespace>/lidar/cloud_native/metadata
```

Use Ctrl+C to finalize the bag. If `ros2 bag` is unavailable, do not
reconfigure the running production container just to install packages.
Record the subscriber and publisher QoS, exact ROS topic names, capture
conditions and the robot's installed firmware/service configuration.

## G1 URDF operator-confirmed lidar calibration

The operator has previously tested the installed G1 mount represented by
`resource/g1_model.urdf`. The driver reads the actual URDF fixed-joint chain
and marks that **URDF transform** `calibrated: true` with source
`g1_model.urdf:operator_confirmed`, a SHA-256-based version and
`calibration_evidence: operator_confirmed_prior_g1_test`.

The calibrated model mount, `torso_link -> mid360_link`, is exposed in
`lidar_cloud.info().urdf_mount` even before the ROS sources publish.
The native SLAM input declaration's `extrinsic` is populated only when both
`rt/unitree/slam_mapping/odom`'s actual `child_frame_id` and raw
`rt/utlidar/cloud_livox_mid360`'s `header.frame_id` can be mapped through
**fixed joints in that same URDF**, with the parent and child names preserved.
Do not assume that a vendor frame named `source_imu` is `imu_in_torso`,
or that `livox_native` means `mid360_link`.

If the operator has confirmed such aliases for this installed G1, configure:

```yaml
plugins:
  lidar:
    enabled: true
    slam_urdf_frame_aliases:
      source_imu: imu_in_torso
      livox_native: mid360_link
```

These are example **link mappings**, not verified defaults. Only enable them
after comparing the real DDS source frames with the model. An unmapped frame,
a dynamic-joint chain (such as pelvis to torso), a missing URDF, or a
conflicting manual transform fails closed: `extrinsic: null` plus
`urdf_calibration_error` in `lidar_cloud.info()`.

The G1 `motus.odom/1` producer remains `unavailable` until tracking,
clock and segment identity are verified. `calibrated: true` on the
physical URDF mount does **not** make the whole SLAM input stream valid.

## What remains dependent on the robot

- Whether Odometry and Livox scan headers are **measurement timestamps** and
  share a verified clock/epoch; scan timing and deskew semantics
- Grounded per-pose tracking/invalid status from vendor state messages
- Restart, disconnect, localization loss, relocation and coordinate-reset
  segment boundaries
- Confirm the real vendor odometry/cloud frame labels or explicit mappings
  to the user-verified G1 URDF; do not infer unknown frame aliases
- Live ROS/DDS transport, actual Docker image build and RTAB-Map acceptance

`plugins.lidar.slam_extrinsic_candidate` optionally accepts a static 4×4
transform with `parent_frame`, `child_frame`, `source`, `version`, and
`matrix`. It is checked against the **shared** rigid mount validator,
visible in `lidar_cloud.info()` diagnostics and always unverified; it
cannot self-assert calibration or satisfy SLAM admission.
