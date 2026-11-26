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

## O10Hand tactile sensor (`hand_tactile`)

Place the sensor on the A3 Ultra canvas and start it, then open **View Data
Stream** on `/<namespace>/hand/tactile_preview`. Both hands show a schematic
of the five fingertips, palm and back of hand, with a raw peak value inside
each named region. The compact 800-by-600 preview uses larger labels and
values. The colors represent raw sensor intensity. The diagram
does not map individual points to positions on the physical hand.

The JSON output at `/<namespace>/hand/tactile` groups all 536 raw readings
under `hands.left` and `hands.right`. Each region reports its `raw` samples,
`peak_raw` (largest magnitude with the original sign) and `nonzero_points`.
The supported AimDK v3.3 layout per hand is 16 thumb, 18 per other finger,
78 palm and 102 back-of-hand points. Fresh all-zero samples stay valid.
When feedback is missing, stale or unsupported, the preview shows the data
status and the JSON has `hands: null`.

This is a driver sensor card. A dedicated best-effort subscription reads
`/motion/control/hand_joint_state` on the existing robot ROS context. Its own
single-threaded executor stores only the latest sample and its monotonic receipt
time, independently of the aggregate joint mirror. Receiving stops when the card
stops. Data becomes stale after one second without an actual arrival, including
when consecutive packets reuse a source timestamp. The card publishes at up to
5 Hz while its canvas instance is running. The
card has no movement or grasp commands and does not convert raw values to
newtons. The two outputs use a dedicated bounded bridge lane when the A3 core
bridge is enabled.

On the robot, verify that pressing and releasing a fingertip and palm changes
and clears the corresponding region. Compare the existing joint and camera
topics before, during and after starting and stopping the tactile card. Capture
the preview and the JSON readback for acceptance.
