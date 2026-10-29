# Bundle-local RGB-D visual odometry (`rgbd_visual_odometry`)

Read-only visual odometry runs **inside the Bumi bundle**: `main.py` starts
`VisualOdometryPlugin`, which supervises one `rtabmap_odom/rgbd_odometry` child
and publishes `motus.odom/1` on `/bumi_visual_odometry/state/odom`. It is not a
second driver, image or container, and it is not a fusion estimator: no IMU, no
`robot_localization`/EKF, no visual-inertial fusion, no distance feedback.

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
| Canvas output | `/bumi_visual_odometry/status`, `std_msgs/msg/String`, port format `data/json`, about 5 Hz |
| Full ROS output | `/bumi_visual_odometry/state/odom`, `std_msgs/msg/String`, `motus.odom/1`, 20 Hz |
| Consumer | none; it is not wired into `loco_target`/`loco_servo` |

`stop` reaps the estimator child and the card's private topics, and sends a
`Bool(false)` on the private `/<NS>/odometry_input/raw_enabled` control topic so
the camera subprocess stops converting/serializing the raw RGB-D branch too.
`start` sends `Bool(true)` and resumes it. The shared camera pipeline, the
`camera`/`depth` cards, photo/video capture, audio and every motion card keep
running; only the visual-odometry branch and estimator are affected.

The canvas displays only `status`, `displacement_m`, `position_m`,
`pose_reference`, `age_ms` and `reason`. Positions and displacement are rounded
to millimetres for display; the full odometry retains its original precision.
Displacement is the 3-D straight-line distance from the **first tracking pose of
the current segment**, not accumulated walking distance or validated accuracy.
Without a measured mount the reference is `rgb_optical` (x right, y down, z
forward), not robot body axes. Reset starts a new segment and distance origin.

Only the summary is declared as a canvas output, because the monitor creates a
panel for every declared output. Existing ROS subscribers can continue using
`/bumi_visual_odometry/state/odom`. The card's details button fetches `info`, whose
description contains the complete current sample, including covariance, timing,
segment/calibration IDs and visual diagnostics; the same sample is returned as
`sample`, with `odom_topic` pointing to the original ROS output. Details are a
snapshot: reopen them to refresh.

The 20 Hz supervision keeps the camera feed on a 500 ms stale clock, while
RTAB-Map results and diagnostics get a 1.5 s grace: walking increased observed
processing intervals from 0.13 s to 0.37 s, and a 500 ms visual edge was below
the real latency. A single bad pose is dropped. An `OdomInfo` that explicitly
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
  check and the camera 250/500 ms freshness limits are unchanged; RTAB-Map
  results get the 1.5 s grace described above.
- One segment-scoped static TF `base -> optical` is published before the child
  starts. Without a measured mount it is identity, so the base frame coincides
  with the optical frame and the pose stays a camera pose; `publish_tf` of the
  estimator stays false.
- No IMU topic is fused; the unused `imu` remap prevents an accidental live
  subscription.
- The pose is accepted only when the RTAB-Map `OdomInfo` for the same stamp says
  tracking is valid (`inliers >= 10`, `matches >= inliers`, not `lost`).

## Output contract

Payload reuses `common/odom.py` (`motus.odom/1`). `twist` is always six `null`s:
RTAB-Map's odometry message is a trajectory delta, not a measured body twist.
`pose` is `null` unless the segment is tracking; no last pose is repeated.

- `pose`: `{position, quaternion xyzw, frame_id, child_frame_id, covariance[36]}`;
  `child_frame_id` is `<segment>_base` (identity-coincident with the optical frame
  unless a mount is measured).
- `vendor`: `status` (`initializing`/`tracking`/`lost`), `reason`, `segment_id`,
  `calibration_id`, `pose_reference` (`rgb_optical` by default, `body` only with
  a measured mount), `visual_diagnostic`, `input_age_ms`, `visual_stamp_ns`,
  `camera_time_provenance`, `source: rtabmap_rgbd_odometry`, `accuracy_validated:
  false`, `twist_source: null`.
- `info()` declares an empty `odom_interface.provides` (no velocity axes),
  `pose_drift: unbounded`, `rate_hz: 20`, plus the current quality.

## Optional measured camera mount

Without `calibration` the pose reference is the camera optical origin; this is not
a claim that the camera-to-body extrinsic is zero. To express the same trajectory
at a body reference point, set a measured mount in `config.yaml` (or through the
persistent `configSchema` calibration field, which the platform replays through
the internal `config` action on start):

```yaml
plugins:
  visual_odometry:
    enabled: true          # default in config.yaml
    max_rate_hz: 10        # raw branch sample rate, bounds DDS/CPU load
    calibration:
      calibration_id: measured-YYYYMMDD
      fixed_mount: true
      camera_to_body:
        translation: [x, y, z]          # camera origin in body coordinates, metres
        quaternion: [qx, qy, qz, qw]    # normalized xyzw, optical C to body B
```

The values become the base->optical static TF; the estimator path is unchanged.
`bumi_model.urdf` has a `waist_yaw_joint` but no camera mount chain, so the body
reference point and whether the camera crosses the waist joint must be confirmed
on hardware; do not use the rendered URDF as a measured extrinsic.

## Build and probe

The bundle image installs `ros-humble-rtabmap-odom` and verifies the estimator
executable at build time; `tf2_ros` and the message packages come transitively
from that package, so they are not installed twice. The shared ROS base image only
contains `/opt/ros/humble`; it does **not** configure a ROS 2 apt source, so the
Dockerfile adds the repository and its signing key first. It defaults to the
Tsinghua `ros2/ubuntu` mirror (the build machine is in Tencent Cloud) and falls
back to `packages.ros.org`; override with
`--build-arg ROS_APT_MIRROR=<url>` when building directly. Nothing else is
added: the bundle already ships numpy/OpenCV/PyYAML for its camera and config.

A synthetic integration probe is copied into the image. It never opens the
RealSense device and never commands the chassis; run it in an isolated domain:

```sh
ROS_DOMAIN_ID=239 python3 /work/visual_odometry_probe.py --output /data/probe.json
```

Expected: `integration_passed: true` (real `rgbd_odometry`, synthetic one-metre
forward motion, input loss -> `lost`, reset -> new segment). It is not hardware
accuracy validation.

## Validation status

ROS-free regression for this branch: `479 passed` in the Bumi suite; the
visual-odometry tests cover the pose/quality contract, launch parameters (single
estimator, no EKF/IMU), plugin lifecycle/actions, static TF/mount and the gated
camera-subprocess raw branch.

**Not executed:** ARM64 image build, real ROS probe, deployment, DDS timing /
media-load measurement, real-robot accuracy. The card is read-only and defaults
to `initializing`, so it cannot inject invented odometry before those checks.

## Limitations

- Historical shared capture rate is about 1.8 Hz; fast rotation, low texture and
  occlusion make loss expected, and loss is latched rather than degraded.
- No loop closure/global relocalization; drift is unbounded by design.
- The camera source has no confirmed exposure timestamp; the device-to-host
  mapping is an arrival-time approximation.
- 20 ms pairing, 10 inliers, 0.2 m position sigma and the 250/500 ms ages are
  rejection thresholds, not accuracy guarantees.
