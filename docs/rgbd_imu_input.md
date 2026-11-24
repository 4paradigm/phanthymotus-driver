# IMU input for RGB-D assistance

Connect the optional `sensor/imu` output (`std_msgs/msg/String`, `motus.imu/1`)
to Perception RGB-D odometry and enable the intended assistance mode. The
existing IMU display output keeps its own topic and format. RGB-D does not
require motion hardware.

PR371 uses [numeric-format admission](https://github.com/Nidhogg2002/phanthymotus/blob/feat/pointcloud-slam-offline-eval/perception/docs/rgbd-imu.md).
Source names, capture/read/receipt labels, calibration descriptions, clock/frame
names and verification flags describe the data; they are not consumer gates.
A timestamp labelled `read_completion` participates in the full calculation.
Providers own actual time alignment, units, axes and installation geometry.

## Numeric packet and physical conventions

| Fields | Meaning |
| --- | --- |
| `schema` | `motus.imu/1` wire layout |
| `stamp_ns`, `accel_stamp_ns` | Integer nanosecond times used in numeric association |
| `angular_velocity`, `linear_acceleration` | Finite raw xyz arrays; rad/s and m/s² conventions |
| `camera_from_gyro`, `camera_from_accel` | Finite numeric 3×3 rotations expressing the raw axes in camera optical axes |
| `camera_from_gyro_translation`, `camera_from_accel_translation` | Finite xyz metres required by full fusion; each sensor origin expressed in camera coordinates |
| `source_id`, `epoch`, frame/clock IDs | Source and coordinate diagnostics |
| `stamp_source`, `rotation_source`, `translation_source`, `calibration_id`, `clock_mapping` | Provenance, including whether times represent capture or reading |

Use column vectors: `p_camera = camera_from_sensor @ p_sensor + translation`.
Provide proper physical rotations and actual sensor origins. Raw vectors are
not pre-rotated; the consumer applies the stated rotations once. Missing
translation vectors permit attitude assistance but do not supply the numeric
packet needed by full fusion. Do not fabricate zero lever arms.

## Bumi — body IMU

Bumi uses HighController body IMU readings from `/NS/state/imu`, not RealSense
motion streams. Its display output remains `/NS/state/imu` (`data/json`); the
named `body_imu` computation port is `/NS/state/imu_calibrated` (`sensor/imu`).
The shared camera owner requests only color and depth. No camera-IMU helper,
second SDK owner or RealSense motion profile is used.

`BodyImuSource` preserves body angular velocity and acceleration, inverts the
configured fixed camera-to-body rotation, and publishes that rotation in both
sensor-to-camera fields. The shipped rotation defines a level-forward axis
convention. The Bumi preset includes the operator-confirmed gyro and accel
lever arms in RGB optical metres. The adapter publishes the lever arms only
when both are supplied, and `imu_status.fusion` reflects their availability
and input freshness.

The SDK supplies no acquisition timestamp. Packets carry the host reading time
unchanged with `stamp_source=read_completion` and an unverified
`host_wall_read_completion` mapping. This is usable numeric time, not evidence
of hardware synchronization. Identical cached SDK values do not refresh a
measurement. Connect `imu.body_imu` for attitude assistance; full fusion needs
the two real translation vectors as well.

## Tianyi and G1 — separate adapters

Tianyi PR519 uses the existing Orbbec gyro/acceleration topics. Its named
`camera_imu` port is `/NS/camera/rgbd/imu`; source-time TF supplies sensor
rotations and, when available, translations relative to the camera. Missing
translation preserves attitude packets while reporting full fusion unavailable.
The camera service's global-clock patch and producer-side timing checks are
documented with that driver. They are not additional PR371 admission rules.

G1's RealSense adapter is maintained separately in PR499. Its factory motion
extrinsics and capture-clock policy are specific to that source; they do not
describe Bumi. Bumi does not use a RealSense motion helper.

Format acceptance and factory/TF availability do not establish motion accuracy.
Software tests and historical static runs do not validate current deployments,
walking/turning performance or map accuracy.
