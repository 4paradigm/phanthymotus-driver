# Spatial input audit — 2026-11-23

Compared with driver [PR519](https://github.com/4paradigm/phanthymotus-driver/pull/519),
head `e7c904f9c04d9434792702f017725f76ca54e645`, and the current consumer contract
described in [PR371](https://github.com/4paradigm/phanthymotus/pull/371).
This records source inspection and stationary reads, not dynamic navigation or mapping acceptance.

| Input | Kuavo implementation | Remaining physical/runtime limitation |
| --- | --- | --- |
| RGB geometry | `motus.camera/1`, actual JPEG dimensions, runtime rectified K/D and optical frame | Lens intrinsics do not define the body mount |
| Registered depth | 640×480 little-endian uint16 mm, zero invalid, lossless `image/depth-zlib`; rectified RGB and depth share geometry/identity after validation | Source depth has several hundred milliseconds of observed frame age; source-clock accuracy remains unverified |
| Camera/body mount | New host helper computes runtime URDF FK from measured lower-body/head joints, bridges the SDK depth root, and emits capture-associated `motus.camera_mount/1` matrices | Model estimate, not installation calibration; candidate observed 160/160 available packets and connected TF while stationary; candidate was reverted after validation, full container not upgraded |
| Camera IMU | Named `camera_imu` `sensor/imu` output; `motus.imu/1` raw vectors, ns timestamps, SDK rotations and sensor-origin translations | Initially no samples in ROS or independent SDK (including root). A camera-device soft reboot restored both native streams and existing relay packets; service/relay restarts alone did not. Root cause of the latched acquisition state is not established |
| Body IMU | Separate `/imu` display | Display data is not silently substituted for camera-relative gyro/accel fusion data |
| Odometry | `state/odom` / `motus.odom/1`, original `/odom` pose timestamp, position/quaternion/covariance, `vendor.motus_slam` measurement metadata, separate receipt-time navigation feedback | Source-clock relationship to the camera remains unverified; do not use the receipt timestamp in place of the source pose time for interpolation |
| Native point cloud | No camera PointCloud2 card; the laser display is a packed planar cloud | It is not `sensor/pointcloud2`. PR371 depth-derived point clouds are an alternative using the registered depth and intrinsics |
| Chassis limits | `motus.control/1`, velocity limits, watchdog, odometry timing and model-derived footprint | The chassis footprint is not full-body clearance when arms are extended |

## Body mount evidence

Before the candidate test, the live ROS1 tree contained `odom → base_footprint → base_link` and, separately,
`camera_head_link → … → camera_head_color_optical_frame`. Both capture-time and
latest TF lookups from `base_link` to `camera_head_color_optical_frame` failed
with disconnected-tree errors. SDK camera/IMU internal transforms do not bridge
that missing installation transform.

The required direction is a metric column-vector transform:
`p_body = body_from_camera @ p_camera`, represented by a finite rigid 4×4 matrix.
The camera optical convention differs from body forward/left/up. A complete mount
needs the URDF joint chain and measured installation relation between the model's
camera mount and the SDK camera root. Because both the head and torso move,
current joint angles are not automatically the angles at image capture time.
Any URDF/configuration-derived estimate must remain labelled as an estimate,
with its timestamp basis and accuracy limitation stated. An identity matrix or
zero lever arm is not an acceptable substitute for the missing installation data.

## Dynamic model mount implementation and validation

`host/body_mount.py` uses the live `/humanoid_description`, verified v63 lower4 +
arms14 + head2 sensor ordering and actual `joint_q` radians. The chain includes
knee, leg, waist pitch/yaw and head yaw/pitch. The fixed model link
`head_camera_depth` has origin `[0.011715, 0.0475, 0.014311]` m relative to
`head_camera_base`, matching the Orbbec Gemini335L CAD depth frame. The installed
SDK gives `camera_head_link` the depth-stream origin, so it is joined to that
model depth origin, not directly to the CAD mounting screw frame. SDK internal
TF supplies the remaining color optical transform.

The helper publishes a dynamic `base_link → camera_head_link` edge at at most
50 Hz using controller ROS read-completion timestamps, and refuses a competing
camera-root TF authority. `sensor_time` is not used as a wall clock on v63.
For each rectified image capture, it interpolates bracketing measured joints
with at most a 50 ms gap, composes SDK internal TF, and emits a finite metric
4×4 `body_from_camera` matrix. It never extrapolates the newest joint pose to
an old image. Camera cards expose the shared `state/extrinsic` output and
`camera_info.vendor.body_mount` declaration. Missing evidence remains unavailable.

A stationary candidate test received 160/160 available mount packets. The last
optical origin was approximately `[0.226667, 0.030731, 1.449141]` m in `base_link`
for that measured posture. TF lookup across the previously disconnected trees
succeeded; registered depth continued publishing with observed age about 423 ms.
The candidate relay was then removed and the original relay restored. These
results validate the connection and numeric transport, not moving-map accuracy.

All packets retain `accuracy_validated=false`, runtime model hash, joint bracket,
and unverified clock/installation provenance. To improve absolute accuracy,
measure or calibrate camera-to-head installation (e.g. a surveyed AprilTag/
checkerboard with several measured head/torso postures), fit the fixed mounting
correction, then validate across withheld postures. This cannot be inferred from
camera intrinsics alone. Cross-board clock offset also needs measurement.

## IMU recovery evidence

Gemini335L firmware 1.4.60 / SDK 1.10.22 enumerated gyro/accel profiles, but raw
subscriptions, synchronous combined output, and an independent SDK-only reader
all received zero frames. The SDK-only test also failed as root. This excludes
JSON forwarding and simple USB access permissions as sufficient explanations.
A camera-device-only SDK `reboot()` restored native gyro/accel, existing
`motus.imu/1` packets and RGB. A subsequent 12-second observation measured about 202.7 Hz for each
native IMU stream and relay packets, with zero duplicate stamps; packet ages were
about 1.1–18.9 ms. RGB was about 15 Hz and rectified depth about 10 Hz (source
age 453–499 ms remained). Firmware was not upgraded; controller services
and motion modes were not changed. The relay now expires its own availability
within 500 ms of losing successful real samples. No automatic repeated reboot
policy was added. Device recovery does not establish reboot endurance.

## Consumer implications

The current PR371 accepts supported numeric formats. Names, calibration labels,
verification flags and freshness labels are diagnostics, not additional admission
gates. Unverified time therefore does not itself mean that a packet is rejected.
It also does not establish physically correct camera/IMU/odom association.

On older deployed versions without a supplied numeric mount, ordinary static cloud/pose mapping may assume
already-aligned coordinates. That assumption is physically inappropriate for an
untransformed head optical cloud combined with chassis odometry. Explicit dynamic
mounting needs real numeric matrices. The camera-reference RGB-D route remains
an option without claiming that its pose is the chassis pose. Full IMU fusion
also needs actual packets and both sensor-origin translations.

The inspected ActuCore navi uses intrinsics, depth, detections, measured planar
odometry and the downstream control descriptor; it does not compensate articulated
head motion using this missing transform. Keep the head forward and stationary.
Body/world point-cloud SLAM has different mount requirements; a successful navi
start is not evidence that its inputs are ready for that computation.

## Follow-up order

1. Observe IMU stability across future camera/robot restarts; retain explicit unavailability on silence.
2. Deploy the tested model mount with the full driver bundle; measure installation
   corrections before claiming calibrated accuracy.
3. Measure camera/IMU/odom time relationships, including SDK depth age, without
   rewriting source timestamps to the current receipt time.
4. Validate moving/turning map alignment with the intended consumer version.

This follow-up restarted only camera-related services and soft-rebooted the camera device.
Temporary systemd overrides were removed. No credentials, controller settings or motion targets were changed.
