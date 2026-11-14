# Camera IMU input for RGB-D assistance

The existing IMU card appends an optional, explicitly named `camera_imu`
output (`sensor/imu` on `std_msgs/msg/String`). Its original display output
retains its source: G1/Bumi body IMU is not replaced by camera IMU. Native
`sensor_msgs/msg/Imu` is used only inside Perception's estimator session.
The original RGB card supplies registered color; the original depth card's
single millimetre-depth output is shared by display, RGB-D odometry and
Point Cloud. The standalone `camera_rgbd` card is retired. Connect the camera
motion output and set `imu_assisted=true` on the Perception card to request
assistance. Default RGB-D operation does not require motion hardware.

Each JSON object has `schema: "motus.imu/1"` and the following fields:

| Fields | Meaning |
| --- | --- |
| `source_id`, `epoch` | Physical motion-source identity and capture generation |
| `stamp_ns`, `accel_stamp_ns` | Gyro and acceleration capture times in the RGB-D clock; never publication/receipt stamps |
| `stamp_source` | `measurement` |
| `clock_id`, `camera_frame_id` | Exact values from the two RGB-D camera declarations |
| `gyro_frame_id`, `accel_frame_id` | Actual raw motion axes |
| `camera_from_gyro`, `camera_from_accel` | Nested row-major proper 3×3 rotations, mapping each raw vector to camera axes |
| `rotation_source` | `sdk_factory_extrinsics` or `tf` |
| `camera_from_gyro_translation`, `camera_from_accel_translation` (optional) | xyz metres: each raw sensor origin expressed in the target camera optical coordinates |
| `translation_source` (optional) | `sdk_factory_extrinsics` or `tf`; present with the calibrated translation pair |
| `angular_velocity`, `linear_acceleration` | Raw xyz arrays in their own source frames |
| `units` | `{"angular_velocity":"rad/s","linear_acceleration":"m/s^2"}` |
| `clock_mapping` (optional) | Source mapping provenance, without claiming unverified relationships as verified |

For either sensor, the full extrinsics map a source-frame point as
`p_camera = camera_from_sensor @ p_sensor + camera_from_sensor_translation`.
The translation is the sensor origin in the camera frame, with no inversion or
lever-arm sign change. RealSense reads `get_extrinsics_to(depth).translation`;
Tianyi reads the translation and rotation from the same source-time TF lookup.
Only finite xyz translation pairs are published as calibrated. Missing or
invalid translation omits both translation fields and `translation_source`,
keeps attitude-only packets and video available, and reports
`imu_status.fusion.state="unavailable"` with a reason. This nested status reports
calibration capability; the parent IMU status still reports live capture health.
The short-term fusion consumer requires calibrated translations and must reject
an absent pair; a missing lever arm is never replaced by zeros.

Raw vectors are not pre-rotated; Perception applies the stated rotations once.
It requires gyro/acceleration capture skew ≤20 ms, gyro update gaps ≤100 ms,
source age ≤500 ms, and future skew ≤50 ms. Epoch, clock, axes or calibration
changes must not silently continue an existing estimator segment.

Tianyi uses vendor Orbbec motion topics and TF at each individual capture time.
G1/Bumi discover supported RealSense motion profiles and share the existing
camera owner; SDK column-major extrinsics are converted to this row-major
contract. G1 requires global capture time; Bumi reuses the RGB-D branch's exact
mapping offset and explicitly keeps approximate mapping unverified. Missing
motion streams, unavailable TF or invalid clocks withdraw only IMU availability;
video and RGB-D remain usable. `imu_status` is reported separately.

Tianyi buffers up to 20 raw gyro and acceleration captures per subscription
using best-effort delivery, retaining 200 ms of 100 Hz history. The previous
100-capture queue accumulated roughly 500 ms of latency under mixed sensor
load. Capture timestamps and the 500 ms source-age limit remain unchanged.
Stalls longer than the retained history can drop captures; the real gap remains
visible, and the consumer's 100 ms gap check can require a new segment. Motion
subscriptions and their TF listener use a dedicated IMU-only node/executor
and lock. Camera and IMU inputs each have an independent robot-domain-0 DDS
participant, separate from body-state traffic and from each other; camera
images, calibration and parameter callbacks have their own executor. The
vendor profile is retained.
The driver body-domain executor runs callbacks serially: in the same mixed-load
diagnostic, the default multithreaded executor (including a two-worker limit)
backlogged motion captures to about 200 ms, while serial body callbacks reduced
their median age to 11–13 ms. Camera and motion executors remain independent;
the domain-42 executor is unchanged. Callback exceptions are logged and do not
terminate the body receive loop. Heavy body callbacks can still delay body
feedback and require workload-specific validation.
Image publication cannot hold the IMU lock. Stop/restart still
serialize IMU publication, and capture age is checked again after TF processing.

A private-namespace diagnostic source plus the production consumer completed
35 s of attitude-assisted F2F with 217 distinct tracking captures and no restart.
This is a bounded diagnostic result, not acceptance of the deployed full driver
or robot motion. The same trial with short-term compensation and depth geometry
enabled repeatedly exceeded the modeled 0.10 m position uncertainty limit.
Use `imu_assisted=true`, `short_term_compensation=false`, `depth_geometry=false`
for the currently verified attitude-only path; the optional fusion mode remains
separate acceptance work. No timestamp, covariance or uncertainty gate was relaxed.

The default IMU-assisted mode estimates relative attitude from gravity and gyro;
it provides no absolute heading or inertial translation. Optional short-term
compensation uses the full calibrated extrinsics and has its own bounded fusion
policy; source calibration availability alone does not establish valid motion
tracking or certified covariance.
Software/offline acceptance does not establish real-device timing, calibration
or estimator quality; hardware testing remains a separate acceptance step.
