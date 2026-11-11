# Camera IMU input for RGB-D attitude assistance

The optional third `camera_rgbd` output is `sensor/imu` on
`std_msgs/msg/String`. This preserves the existing JSON format family; native
`sensor_msgs/msg/Imu` is used only inside Perception's estimator session.
Connect color and depth normally; connect this third output and set the
Perception card's instance `imu_assisted=true` to request assistance. Default
RGB-D operation does not require motion hardware.

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
| `angular_velocity`, `linear_acceleration` | Raw xyz arrays in their own source frames |
| `units` | `{"angular_velocity":"rad/s","linear_acceleration":"m/s^2"}` |
| `clock_mapping` (optional) | Source mapping provenance, without claiming unverified relationships as verified |

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

The Perception filter estimates relative attitude from gravity and gyro. It
provides no absolute heading, certified covariance or inertial translation.
Software/offline acceptance does not establish real-device timing, calibration
or estimator quality; hardware testing remains a separate acceptance step.
