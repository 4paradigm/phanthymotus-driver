# A3 Ultra sensor notes

The two wrist D405 camera topics are advertised by the robot's
`hal_depth_camera_node`, but on the inspected A3 Ultra they have no image
samples (four-second best-effort probes returned none). The driver therefore
does not expose wrist camera cards or subscribe to those topics. This is a
robot HAL/firmware issue, not a frontend topic-name issue; restore the entries
in `config.yaml` and `driver.yaml` only after the vendor node produces samples.

The joints card combines arm, hand, neck, waist, and leg feedback. Pelvis IMU
orientation is published as `imu_quat` for root tilt; a lying pose cannot be
represented by joint angles alone.

The remaining camera preview streams are rate-limited before JPEG encoding.
This bounds queue growth and keeps the latest frame available to the dashboard.
