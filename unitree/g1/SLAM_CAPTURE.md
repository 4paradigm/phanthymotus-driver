# G1 SLAM input verification and read-only capture

This driver branch implements two **candidate** inputs for the generic
`pointcloud_slam` Perception card:

- `/<namespace>/slam/odom` — **regular** G1 `rt/odommodestate` converted to ROS `std_msgs/msg/String`, `motus.odom/1`.
- `/<namespace>/slam/mapping/odom` — legacy vendor SLAM diagnostics, **not the selected RTAB-Map input**.
- `/<namespace>/lidar/cloud_native` — ROS `sensor_msgs/msg/PointCloud2`
- `/<namespace>/lidar/cloud_native/metadata` — ROS `std_msgs/msg/String`

The existing `/<namespace>/loco/state` and `/<namespace>/lidar/cloud` outputs
are unchanged. Do not connect `sensor/pointcloud` to the native SLAM input.

**No dependency on vendor mapping SLAM:** The new `state/odom` output is
converted directly from ordinary `rt/odommodestate` (position[3], velocity[3],
IMU quaternion WXYZ and source TimeSpec). `/loco/state` remains unchanged.

**G1 live finding (2026-11-10):** Both native `rt/odommodestate` (~500 Hz)
and `rt/lf/odommodestate` (~20 Hz) provided `stamp.sec=0`,
`stamp.nanosec=0`. Native position/velocity/IMU fields were populated and
the error code was zero. The driver continues to subscribe to the high-rate
source, but independently samples its `/<namespace>/slam/odom` diagnostic
candidate at approximately 20 Hz; existing `/<namespace>/loco/state` remains
at approximately 10 Hz. Neither rate is an estimator quality claim.

Missing or invalid source timestamps do **not** hide finite raw motion values:
`vendor.raw_position`, `raw_imu_quaternion_wxyz`,
`raw_imu_quaternion_xyzw`, `raw_velocity`, `raw_yaw_speed`,
`raw_source_stamp`, `received_ns`, and `received_monotonic_ns` stay
available for diagnosis. Wall and monotonic times are explicitly labeled
`driver_callback_receipt_not_measurement`; the output `stamp_ms` remains
null for zero source stamps. No arrival time is substituted for exposure
time, no fake clock relation is created and unknown frames are not guessed.

**Not yet an admitted tracking pose:** This G1 SportModeState message lacks
explicit pose frame IDs, verified measurement-clock semantics, an unambiguous
position/IMU orientation reference, tracking status and reset identity. The
adapter preserves raw observations in `vendor` but deliberately reports
`vendor.motus_slam.status = unavailable`,
`odom_interface.slam_pose.available = false` and `segment_id: null`.
A correctly shaped sample is not proof of tracking, so the driver must not
force `tracking` to bypass PR #371.

## Offline contract tests

Run from the driver repo root on the branch corresponding to PR #499:

```bash
python3 -m pytest -q \
  tests/test_spatial_slam.py \
  tests/test_g1_mapping_odom.py \
  tests/test_g1_sport_odometry.py \
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

- `rt/odommodestate`: **ordinary odometry** source stamp, position, velocity,
  IMU WXYZ orientation, gyroscope and error code
- `rt/unitree/slam_mapping/odom`: old vendor mapping/SLAM diagnostic (optional;
  this topic need not publish for regular odometry to work)
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
ros2 topic list -t | grep -E 'slam/odom|slam/mapping/odom|camera/depth|lidar/cloud_native'
# Replace <namespace> with the namespace actually reported by ROS:
ros2 bag record -o /opt/phanthy-motus/data/slam_capture/static \
  /<namespace>/slam/odom \
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
the explicitly configured ordinary odometry reference `child_frame_id` and
raw `rt/utlidar/cloud_livox_mid360`'s `header.frame_id` can be mapped through
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

The new G1 ordinary `motus.odom/1` producer remains `unavailable` until
pose frame, tracking, clock and segment identity are verified. `calibrated: true` on the
physical URDF mount does **not** make the whole SLAM input stream valid.

## Optional ordinary odometry frame labels

The native `SportModeState_` has `position[3]`, IMU quaternion WXYZ and
`stamp`, but **does not identify the pose's world or child frame**.
When the real frame relationship has been verified, configure BOTH labels:

```yaml
plugins:
  loco:
    enabled: true
    slam_regular_odometry:
      frame_id: odom
      child_frame_id: imu_in_torso
```

This is an **illustrative example, not a verified G1 default**.
Do not guess that a position referenced to the robot base center is identical
to the body IMU frame; they may require an additional transform.
With no labels, the candidate publishes `pose: null` with full raw
position/orientation evidence under `vendor`. Even with explicit labels, a
zero native timestamp still produces `pose: null`, `stamp_ms: null` and
`invalid_source_stamp`, alongside preserved raw measurements and receipt
times. If a subsequently verified source timestamp is present, labels can
enable a candidate pose, but it still reports `status: unavailable` until
tracking, timestamp provenance and reset guarantees are verified.

To check the non-lidar data while Mid360 is unavailable:

```bash
ros2 topic echo --once /<namespace>/loco/state
ros2 topic echo --once /<namespace>/slam/odom
ros2 topic info -v /<namespace>/camera/depth
```

For RealSense-based RTAB-Map mapping, an upstream depth-to-metric-PointCloud2
converter must also supply real camera intrinsics, capture-time provenance and
camera-to-odom-reference extrinsics. A raw compressed depth image is not
`sensor/pointcloud2`.

## What remains dependent on the robot

- Whether SportModeState and RealSense/depth-derived pointcloud stamps are
  **measurement timestamps**, and a verified clock relationship/epoch exists
- Grounded per-pose tracking/invalid status and source frame semantics for
  ordinary `rt/odommodestate` (without assuming the vendor SLAM service)
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
