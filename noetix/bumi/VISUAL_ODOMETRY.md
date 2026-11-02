# Bundle-local RGB-D visual odometry (`rgbd_visual_odometry`)

Read-only visual odometry runs **inside the Bumi bundle**: `main.py` starts
`VisualOdometryPlugin`, which supervises one `rtabmap_odom/rgbd_odometry` child
and publishes `motus.odom/1` on `/<namespace>/bumi_visual_odometry/state/odom`. It is not a
second driver, image or container. The robot body IMU optionally supplies a
rotation prior; translation still comes from RGB-D. There is no
`robot_localization`/EKF or full visual-inertial state estimator.

The card is controlled by `plugins.visual_odometry.enabled` in `config.yaml`.
It is **enabled by default**. Set the key to `false` in `config.yaml` or a mounted
config to disable it. When enabled, `main.py` exports
`BUMI_VISUAL_ODOMETRY=1` before the camera subprocess starts, so the same
pyrealsense2 capture process publishes the raw RGB-D branch `visual_inputs.py`
uses. Disabled means no raw branch, no estimator child, no extra DDS traffic.

## Card

| Field | Value |
| --- | --- |
| Tool / card | `rgbd_visual_odometry` (sensor, read-only) |
| MCP actions | `start`, `stop`, `info` (system actions only; no execution form/button) |
| Canvas output | `/<namespace>/bumi_visual_odometry/status`, `std_msgs/msg/String`, port format `data/json`, about 5 Hz |
| Full ROS output | `/<namespace>/bumi_visual_odometry/state/odom`, `std_msgs/msg/String`, `motus.odom/1`, 20 Hz |
| Consumer | none; it is not wired into `loco_target`/`loco_servo` |

`stop` reaps the estimator/filter children and the card's private topics, and sends a
transient-local `Bool(false)` on the private `/<NS>/odometry_input/raw_enabled`
control topic so the camera subprocess stops converting/serializing the raw
RGB-D and raw IMU branches too. The latched sample is delivered even if the camera process
starts late or is restarted while the card is stopped; the camera branch defaults
to disabled. `start` sends `Bool(true)` and resumes it. The shared camera pipeline, the
`camera`/`depth` cards, photo/video capture, audio and every motion card keep
running; only the visual-odometry branch and estimator are affected.

The canvas displays only `status`, `displacement_m`, `position_m`,
`pose_reference`, `age_ms` and `reason`. Positions and displacement are rounded
to millimetres for display; the full odometry retains its original precision.
Displacement is the 3-D straight-line distance from the **first tracking pose of
the current segment**, not accumulated walking distance or validated accuracy.
The shipping `config.yaml` sets a permanent **body-axis convention** for the
standard level, forward-facing camera: `x` forward, `y` left, `z` up
(`pose_reference=body_axes`). It is an axis assumption, not a measured extrinsic;
the pose origin is the camera because the default translation is zero. Reset
starts a new segment and distance origin.

The canvas declares two outputs: the 5 Hz `status` summary (`data/json`) and the
full `state/odom` sample (`motus.odom/1`) so other cards can consume the pose
without a second converter. Existing ROS subscribers can also use
`/<namespace>/bumi_visual_odometry/state/odom` directly. The card's details button fetches `info`, whose
description contains the complete current sample, including covariance, timing,
segment/calibration IDs and visual diagnostics; the same sample is returned as
`sample`, with `odom_topic` pointing to the original ROS output. Details are a
snapshot: reopen them to refresh.

The 20 Hz supervision keeps the camera feed on a 1 s stale clock (measured raw
cadence is ~8.3 Hz, worst observed motion gap ~0.4 s; the old 1.8 Hz
documentation predates the bundle raw branch), while RTAB-Map results and
diagnostics get a 1.5 s grace: walking increased observed processing intervals
from 0.13 s to 0.37 s. A single bad pose is dropped. An `OdomInfo` that explicitly
reports `lost` or falls below the inlier floor ends the segment immediately
(the current pose is cleared, not published as tracking); the delayed restart
then starts a **new segment** (fresh estimator, fresh odom frame). Sustained
gaps and repeated pose discontinuities do the same. Status transitions publish
immediately even between 5 Hz summary updates. Lost or stopped output clears
both displayed position and displacement rather than repeating the last valid
pose.

## Inputs

Published by the bundle's camera subprocess when the card is enabled
(`NS` = bundle namespace):

| Topic | Type / meaning |
| --- | --- |
| `/NS/odometry_input/rgb` | `sensor_msgs/Image`, bgr8 source colour frame |
| `/NS/odometry_input/depth` | `sensor_msgs/Image`, 32FC1 metres in the colour grid (SDK `depth_scale`) |
| `/NS/odometry_input/info` | `sensor_msgs/CameraInfo`, live K/D/R/P; zero coefficients as `plumb_bob` |
| `/NS/odometry_input/camera_meta` | `std_msgs/String` JSON: epoch, per-stream source time/frame/clock domain, depth scale, receive time, mapped stamps |

RGB/depth source stamps must be within 20 ms. The sidecar validates the time
domain and transport age, undistorts when the camera actually reports non-zero
coefficients (OpenCV is available in the bundle image), rejects unusable depth
and requires the RGB-D frame pair and CameraInfo to describe the same aligned
frame. An unknown distortion model or a changed frame/intrinsics/epoch ends the
segment; the supervisor starts a fresh segment automatically, so no manual
reset is required.

## Estimation

- One child: `/opt/ros/humble/lib/rtabmap_odom/rgbd_odometry`, private topics
  `/<segment>/rgbd_image`, `/<segment>/visual`, `/<segment>/diagnostic`.
  The validated pair is forwarded as one `rtabmap_msgs/RGBDImage` with both
  source image stamps preserved (`subscribe_rgbd: true`), avoiding a second
  approximate synchronizer waiting for another frame. The 20 ms source-pair
  check and the camera 250 ms/1 s freshness limits are unchanged; RTAB-Map
  results get the 1.5 s grace described above.
- One segment-scoped static TF `base -> optical` is published before the child
  starts. The default config uses the standard level-forward rotation
  (`[-0.5, 0.5, -0.5, 0.5]`, xyzw) with zero translation, so `base` carries robot
  body axes (x forward, y left, z up) at the camera origin; `publish_tf` of the
  estimator stays false. Removing calibration falls back to `rgb_optical`.
- With `imu_assisted: false`, the unused `imu` remap prevents an accidental live
  subscription. With body assistance enabled, the segment's private republished
  body orientation topic is remapped to RTAB-Map.
- The pose is accepted only when the RTAB-Map `OdomInfo` for the same stamp says
  tracking is valid (`inliers >= 10`, `matches >= inliers`, not `lost`).

## Output contract

Payload reuses `common/odom.py` (`motus.odom/1`). `twist` is always six `null`s:
RTAB-Map's odometry message is a trajectory delta, not a measured body twist.
`pose` is `null` unless the segment is tracking; no last pose is repeated.

- `pose`: `{position, quaternion xyzw, frame_id, child_frame_id, covariance[36]}`;
  `child_frame_id` is `<segment>_base`, which carries robot body axes by default
  (x forward, y left, z up) at the camera origin.
- `vendor`: `status` (`initializing`/`tracking`/`lost`), `reason`, `segment_id`,
  `calibration_id`, `pose_reference` (`body_axes` by the shipped level-forward
  assumption; `body` for a measured mount; `rgb_optical` if calibration is
  removed),
  `visual_diagnostic`, `input_age_ms`, `visual_stamp_ns`,
  `camera_time_provenance`, `source: rtabmap_rgbd_odometry`, `accuracy_validated:
  false`, `twist_source: null`.
- `info()` declares an empty `odom_interface.provides` (no velocity axes),
  `pose_drift: unbounded`, `rate_hz: 20`, plus the current quality.

## Body IMU rotation assistance

`plugins.visual_odometry.imu_assisted: true` (shipped default) polls the
HighController body IMU directly at 20–50 Hz and republishes changed samples as
`sensor_msgs/Imu` on the segment-private `/<segment>/imu` topic. RTAB-Map waits
for this orientation prior and uses it for rotation only; acceleration is not
integrated into position. `imu_source: state` is the fallback that consumes
StatePlugin's `/state/imu` JSON; `controller` is the shipped default. The camera
stays on its normal video-only `pyrealsense2` path, so no custom RSUSB SDK,
camera-mount TF or Madgwick child is started.

The SDK exposes no hardware capture timestamp; the payload marks `stamp_ns` as
`timestamp_source: read_completion`, and the card applies a conservative 500 ms
freshness window. Unchanged controller samples are not republished, so a cached
sample cannot keep the freshness clock alive. Missing orientation prevents tracking; orientation older than
500 ms invalidates the segment. Camera-IMU mode is not packaged and is rejected
at construction rather than starting a camera that cannot open its SDK. These checks reject bad input; they do not establish metric
accuracy. Static IMU bias, walking blur and long-term drift still require
on-robot validation. Set `imu_assisted: false` to return to visual-only mode.

## Validation status

ROS-free regression for this branch: **466 passed**, run with
`python3 -m pytest -q tests/bumi_visual_odometry tests/test_bumi*.py noetix/bumi/tests`.
The visual-odometry tests cover the pose/quality contract, launch parameters,
plugin lifecycle/actions, static TF/mount and the gated camera raw branch.

On-robot verification: the body-IMU path reaches `tracking`; controller-source
direct polling and stop/start behavior still need hardware stress validation.
The camera-IMU RSUSB build path and its modules have been removed.
