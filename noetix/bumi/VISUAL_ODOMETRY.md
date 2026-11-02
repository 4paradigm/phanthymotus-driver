# Bundle-local RGB-D visual odometry (`rgbd_visual_odometry`)

Read-only visual odometry runs **inside the Bumi bundle**: `main.py` starts
`VisualOdometryPlugin`, which supervises one `rtabmap_odom/rgbd_odometry` child
and publishes `motus.odom/1` on `/bumi_visual_odometry/state/odom`. It is not a
second driver, image or container. Camera IMU orientation optionally supplies a
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
| Canvas output | `/bumi_visual_odometry/status`, `std_msgs/msg/String`, port format `data/json`, about 5 Hz |
| Full ROS output | `/bumi_visual_odometry/state/odom`, `std_msgs/msg/String`, `motus.odom/1`, 20 Hz |
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
`/bumi_visual_odometry/state/odom` directly. The card's details button fetches `info`, whose
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
  subscription. With assistance enabled, only the segment's private filtered
  camera IMU topic is remapped to RTAB-Map.
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

## Camera IMU rotation assistance

`plugins.visual_odometry.imu_assisted: true` (shipped default) selects the
isolated RSUSB SDK **only for the camera subprocess**. Set it to `false` and
restart the bundle to restore the original native-SDK visual-only path.
`state/odom`, the compact canvas status, RTAB-Map pose/diagnostic checks and
existing camera/visual freshness limits are preserved. This does not integrate
acceleration into position or continue estimating distance after visual loss.

One asynchronous pipeline captures RGB, depth, gyro and accel. The active device
uses the SDK's shared global-time fit to compensate clock drift; three seconds
of startup reports are drained before publishing. No extra arrival-time offset
is applied to global timestamps. Domain, ordering and age checks remain active;
a late clock-fit startup fails closed rather than claiming a valid pose. Accel is
linearly interpolated between measured brackets at each actual gyro timestamp;
no extrapolation or repeated latest samples are used. SDK extrinsics rotate
accel into gyro axes, and a static `optical -> camera_imu` TF preserves the
factory rotation and translation to RGB. The assumed body-axis mount above is
still an assumption, separate from these factory sensor extrinsics.

A supervised `imu_filter_madgwick_node` estimates orientation (`use_mag=false`,
`publish_tf=false`, ENU, gain 0.01). RTAB-Map waits for this orientation and uses
its private `/<segment>/imu` topic. `Odom/GuessSmoothingDelay=0` explicitly keeps
acceleration out of motion prediction. The filter is reaped with the estimator on
stop/reset. Missing orientation prevents tracking; orientation older than
500 ms invalidates the segment. Raw motion gaps over 25 ms reset the motion
epoch and require a fresh estimator segment. These thresholds reject bad input;
they do not establish metric accuracy. Static gyro bias, walking blur and
long-term drift still require on-robot validation.

Full diagnostics are in `vendor.imu_assistance`: factory mount, raw sample
count/gap, latest raw stamp, reset reason and filtered orientation stamp. Raw measurements are
on `/<NS>/odometry_input/imu_raw`; these preserve physical gyro-frame axes and
contain no orientation until filtered. No IMU fields are added to the compact
canvas status. The SDK is built in a separate Docker stage; only runtime shared
libraries and Python bindings are copied into `/opt/bumi-rsusb`, alongside the
existing native SDK. The only extra ROS runtime package is
`ros-humble-imu-filter-madgwick`.

## Standalone camera IMU access probe

The D435i has an internal gyro and accelerometer. Some Jetson kernels omit
`CONFIG_HID_SENSOR_HUB` and the HID accelerometer/gyro modules: the native
RealSense backend then exposes RGB/depth but no Motion Module, even in a
privileged container with `/dev` mounted. The bundle currently enables only the
RGB/depth streams in visual-only mode; assistance uses the shared RSUSB pipeline
described above.

For that kernel configuration, build an isolated RSUSB SDK, which implements
UVC/HID in userspace. The helper pins official librealsense v2.59.1 and verifies
the source archive SHA-256. It requires CMake, a C++ compiler, Git, curl, Python
development headers and libusb development files. Run inside the bundle:

```sh
bash /work/build_realsense_rsusb.sh /tmp/bumi-rsusb
```

**Stop the existing camera capture before running the probe**, and use a
maintenance context. A separate RSUSB context can interrupt the native-backend
camera worker even though the probe does not start video streams. Do not run
this alongside its camera. On the tested target, restarting the bundle alone
did not restore video: a camera `hardware_reset()` through the original native
SDK, followed by a bundle restart, restored capture. This resets the camera
without updating its firmware.

```sh
PYTHONPATH=/tmp/bumi-rsusb/install/python \
  python3 /work/realsense_imu_probe.py --seconds 10 --output /data/camera-imu-probe.json
```

Keep the robot stationary for the probe. It opens only the motion sensor, uses
hardware timestamps with global-time conversion disabled, and drains startup
reports for two seconds before recording the requested duration. It checks
sensor timestamp order, finite values, rate and gaps, and reports gyro (rad/s)
and accelerometer (m/s²) statistics. It exports each motion profile's transform
to the 640×480 BGR8 colour profile as a **row-major** matrix with metre
translation and an explicit `p_color = R_color_motion * p_motion + t_color_motion`
direction; the SDK's original rotation array is column-major. A missing IMU,
invalid transform or unusable sample stream fails the probe instead of supplying
identity extrinsics or repeated samples.

The SDK is installed under the chosen build directory and selected only via
`PYTHONPATH`; the helper does not replace the existing Python package, flash
firmware or load kernel modules. RSUSB performance and coexistence must be
checked on the target before using it as
the bundle's default backend. The probe establishes IMU access and exports SDK
extrinsics; the probe's hardware timestamps must be mapped to the image clock
before fusion. Full VIO still needs synchronized image/IMU delivery, noise/bias and
calibration checks, an estimator, and walking/metric validation.

On the tested Bumi (Jetson kernel `5.15.185-tegra`, D435i firmware `5.15.1.55`),
this SDK exposed gyro/accel profiles at 200 Hz and readable motion-to-colour
extrinsics. Ten-second recordings after warmup measured about 193–196 Hz with
54–55 ms gaps, failing the probe's five-period continuity limit (25 ms at
200 Hz). Native camera capture also timed out during separate RSUSB access.
This proves IMU/extrinsics access, **not VIO-ready timing or coexistence**.
A later 60-second single-pipeline RGB/depth/gyro/accel experiment had zero
missing frame numbers: gyro 199.49 Hz (maximum gap 5.014 ms), accel 202.82 Hz
(4.934 ms), RGB/depth 29.98 Hz, all on hardware timestamps. The assistance path
uses that single pipeline; it does not run this standalone motion-only probe
alongside video. These capture measurements do not prove walking accuracy.

See the [official Jetson backend guide](https://github.com/realsenseai/librealsense/blob/v2.59.1/doc/installation_jetson.md)
and [D435i IMU documentation](https://github.com/realsenseai/librealsense/blob/v2.59.1/doc/d435i.md).

## Camera mount / body-axis convention

`config.yaml` ships a permanent level-forward convention so the position axes
match a robot body frame without a manual step:

```yaml
    calibration:
      calibration_id: assumed-level-forward-optical-to-body
      fixed_mount: true
      assumed: true
      camera_to_body:
        translation: [0.0, 0.0, 0.0]
        quaternion: [-0.5, 0.5, -0.5, 0.5]
```

The quaternion maps optical axes to body axes:
`optical x (right) -> body -y`, `optical y (down) -> body -z`,
`optical z (forward) -> body +x`. Translation is the camera origin in robot
coordinates; zeros means the position origin stays at the camera (distance is
unaffected by this constant offset). Replace both values with a measured mount
if the camera is not level/forward or the body origin is elsewhere; the platform
persistent `configSchema` calibration field replays through the internal
`config` action on start, and a measured mount can also be set in `config.yaml`:

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
If `calibration` is removed entirely, the reference falls back to `rgb_optical`.
`bumi_model.urdf` has a `waist_yaw_joint` but no camera mount chain, so the body
reference point and whether the camera crosses the waist joint must be confirmed
on hardware; do not use the rendered URDF as a measured extrinsic.

## Build and probe

The bundle image installs `ros-humble-rtabmap-odom` and
`ros-humble-imu-filter-madgwick` and verifies the estimator
executable at build time; `tf2_ros` and the message packages come transitively
from that package, so they are not installed twice. The shared ROS base image only
contains `/opt/ros/humble`; it does **not** configure a ROS 2 apt source, so the
Dockerfile adds the repository and its signing key first. It defaults to the
Tsinghua `ros2/ubuntu` mirror (the build machine is in Tencent Cloud) and falls
back to `packages.ros.org`; override with
`--build-arg ROS_APT_MIRROR=<url>` when building directly. The bundle already
ships numpy/OpenCV/PyYAML. The separate SDK build stage adds about 18 MiB of
runtime shared libraries/bindings on the tested ARM64 target; compiler, source
and headers stay out of the final image. The extra Madgwick package has about
1.2 MiB installed size. This feature therefore has a small runtime image cost.

A synthetic integration probe is copied into the image. It never opens the
RealSense device and never commands the chassis; run it in an isolated domain:

```sh
ROS_DOMAIN_ID=239 python3 /work/visual_odometry_probe.py --output /data/probe.json
```

Expected: `integration_passed: true` (real `rgbd_odometry`, synthetic one-metre
forward motion, input loss -> `lost`, reset -> new segment). It is not hardware
accuracy validation.

## Validation status

ROS-free regression for this branch: **397 passed**, run with
`python3 -m pytest -q tests/bumi_visual_odometry tests/test_bumi*.py noetix/bumi/tests`; the
visual-odometry tests cover the pose/quality contract, launch parameters (single
estimator, optional IMU attitude filter and no translation EKF), plugin lifecycle/actions, static TF/mount and the gated
camera-subprocess raw branch.

SSH deployment verified actual camera IMU input, Madgwick orientation and
RTAB-Map's private IMU subscription. Stop/start testing disabled raw IMU output
while JPEG continued at 30 Hz, without replacing the camera process. Pausing
Madgwick caused `lost` with no pose in 0.528 s, followed by recovery into a new
segment. A 75-second stationary test had 1438 tracking samples, no lost status
and one segment; raw/filtered IMU were approximately 199 Hz and JPEG 30 Hz.
That early fixed-offset build later reset its IMU epoch in a 0.66 m walking
trial due to clock mapping, so it is not the final timing validation. The final
implementation uses the SDK shared clock fit described above.

With the final SDK-clock path, a 300-second stationary run had 5976 tracking
samples, one segment and no loss. Raw/filtered IMU averaged about 199 Hz and
JPEG 30 Hz; image metadata reported `global_time`. A subsequent 180-second
walking run had 3461 tracking samples, one segment and no loss/reset. Against
the operator's approximate 0.87 m tape distance, the displacement between the
first/last five-second mean positions was 0.9216 m: +0.0516 m (about +5.9%).
This single approximate measurement does not establish accuracy; the output
continues to report `accuracy_validated=false`.

The final timing/accuracy measurements are recorded with the robot acceptance
artifacts. An ARM64 Docker image rebuild and statistically meaningful
walking accuracy validation are not established by source tests or a single
walking trial. The card is read-only and suppresses pose unless all required
inputs and RTAB-Map diagnostics pass.

## Limitations

- Measured raw-branch cadence is about 8.3 Hz at the 10 Hz cap (the old sidecar
  figure of ~1.8 Hz predates this branch); fast rotation, low texture and
  occlusion make loss expected, and loss is latched rather than degraded.
- No loop closure/global relocalization; drift is unbounded by design.
- Pose position is expressed in the segment's odometry frame, while the child
  base frame carries the configured body axes. IMU initialization can change
  the odometry frame's initial heading; without a magnetometer it is not an
  absolute heading. Do not assume a straight walk must change only odom x.
- The camera source has no confirmed exposure timestamp; the device-to-host
  mapping is an arrival-time approximation.
- 20 ms pairing, 10 inliers, 0.2 m position sigma and the 250 ms/1 s ages are
  rejection thresholds, not accuracy guarantees.
