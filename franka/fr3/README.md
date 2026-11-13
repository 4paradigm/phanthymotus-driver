# Franka FR3 Driver

The ARM64 driver connects to FR3 controllers over a wired network and reads
external RealSense cameras over USB. It registers with Agent Core as
`franka-fr3-driver` on MCP port `15723`.

| Card or resource | Type | Function |
| --- | --- | --- |
| `joint_states` | Sensor | Actual seven-axis positions for monitoring and 3D visualization |
| `model` | Resource | FR3 URDF used by the skeleton renderer |
| `joint_control` | Actuator | Absolute joint targets, single-axis jog, and controlled stop |
| `gripper` | Actuator | Franka Hand opening, grasp, calibration, and stop |
| `ext_camera` | Sensor | RGB, depth, or left near-infrared from a selected RealSense |

## Requirements

- ARM64 Jetson with an ARM64 ROS 2 Humble driver image and a running Agent Core.
- FR3 on a reachable wired network with a unique IP per controller. Activate FCI
  in Desk and unlock joints before commanding arm motion.
- The image pins libfranka 0.18.1 for FCI protocol 10. System image 5.10.0 is the
  validated deployment target. See the [Franka compatibility matrix](https://frankarobotics.github.io/docs/doc/libfranka/docs/compatibility_matrix.html).
- The motion backend requires FIFO scheduling. Standard deployment selects
  `FRANKA_REALTIME_POLICY=standard` for the stock Jetson kernel. Select
  `enforce` to additionally require PREEMPT_RT.
- External USB RealSense for `ext_camera`. D435i is the validated model. Other
  RealSense devices must support the requested stream profiles.

## Build and deploy

Deploy the published driver through Agent Core, or build from the repository
root with `./build.sh franka/fr3`. For a local build without publishing:

```bash
context=$(mktemp -d)
cp -a franka/fr3/. "$context/"
cp -a common "$context/common"
docker build -t franka-fr3:local "$context"
```

For standalone Compose, put [deploy/service.yml](deploy/service.yml) under
`services:` and replace `__IMAGE__` with the selected image. Keep host networking,
DDS domain 42, the read-only DDS profile and Core CA mounts, USB device access,
and the scheduling permissions in the template. Set `FRANKA_AGENT_CORE_URL`
if the default Core URL does not match the deployment. SDK source and checksum
details are in [vendor/SOURCE.md](vendor/SOURCE.md).

## Canvas configuration

Choose a controller by model, serial, and IP in the card settings. Use
**手动输入 IP** under advanced settings if discovery does not list it. Stop the
card before changing its controller. Each card keeps its own selection.
The display frequency controls status publication and defaults to 10 Hz.

Start `joint_states` and open its skeleton stream in the monitoring dashboard.
The renderer loads `model` automatically. The instance topic is
`/franka/<card_id>/state/joints` with format `sensor/skeleton` and angles in
radians. A retained pose can be stale. Use `info` to check connection and
sample freshness.

## Joint control

| Action | Parameters |
| --- | --- |
| `set` | Absolute target angles in degrees. Omitted joints hold their starting position. |
| `jog` | Joint 1–7 and a signed, nonzero increment in degrees. |
| `stopmotion` | Stops the current motion and keeps the card available. |

Speed is 0.1–10 degrees/s. The driver checks target limits with a 0.01 rad
margin and rejects trajectories exceeding the 120 s motion budget. Joint
limits do not model obstacles in the workcell.

`joint_states` publishes joint feedback for monitoring.

## Franka Hand

`joint_control` and `gripper` are single-instance cards with no configuration panel.
Select `arm` and enter speed, force and other parameters directly in each action.
Without a workstation arm mapping, enter `robot_ip` in the action instead.
The first execution connects and waits up to five seconds for fresh, control-ready
feedback before submitting motion. Offline or unsafe states still reject motion.

| Action | Parameters |
| --- | --- |
| `set_position` | Total finger spacing in mm and speed in mm/s. |
| `grasp` | Expected object width, speed, force in N, and optional opening bounds. |
| `homing` | Calibrates the opening range by moving the fingers. |
| `stopmotion` | Requests a confirmed stop and may release a held object. |

The width limit is the smaller of 80 mm and the measured maximum opening.
Speed must be supplied for each opening/grasp action (range 0.1–100 mm/s). Grasp requires force (30–70 N). The `info` action reports the current and maximum opening.

## Action feedback

Motion calls return `state=running` and a unique `action_id`. On termination,
the driver posts `completed`, `cancelled`, or `error` to `/api/acp/complete`
using the same ID. Loopback callbacks (`localhost` and numeric loopback IPs)
follow the Core example without TLS certificate verification. Remote HTTPS
callbacks require `AGENT_CORE_CA_CERT` and retain certificate/hostname verification.

Motion cards have no monitor output and do not send extra `/api/event`
notifications. Core handles the initial MCP response and the standard ACP
completion event. With intelligent control running, completion can appear in
Activity as a collector trigger or a current-turn notification; Core may
summarize successful results. The `info` action retains the full last result
and ACP delivery status. No Activity token is required by this driver.

## External RealSense camera

Select a USB serial and one channel per card. Cards on the same serial share
a capture session. A disconnected camera is retried by its original serial.
After saving a channel change, reopen the preview to subscribe to the new topic.

| Channel | Format | Data |
| --- | --- | --- |
| `rgb` | `image/jpeg` | Color image |
| `depth` | `image/depth-zlib` | Compressed uint16 distances in mm, with 0 for invalid pixels |
| `infrared` | `image/jpeg` | Left near-infrared grayscale |

Resolve the current topic and format through instance `info`. Depth is in the
native depth frame without RGB alignment. The optical frame ID identifies the
camera and channel. Timestamps use the publication clock. Check `fresh` and
`frames_published` to verify that the preview is receiving new images.

## Verification

```bash
python3 -m pytest tests/test_franka_*.py
python3 -m unittest discover -s franka/fr3/native -p 'test_*.py'
python3 scripts/check_service_yml.py franka/fr3
```

For hardware acceptance, record the image version and controller tested.
Confirm fresh joint state, the three camera channels and USB reconnect,
then verify arm and gripper motion, stop behavior, and ACP acknowledgement
with the operator monitoring the workcell.

With `motion_arm_ips` configured, each action selects the left or right arm.
Each arm accepts one in-flight action per control type. The workstation mapping
is stored under `motion_arm_ips.left` and `motion_arm_ips.right` in the driver
configuration; it is not a canvas card configuration.

## Grasp limits

Grasp requires `width_mm`, `speed_mm_s` and `force_n`; missing fields are listed
in the error before connecting to hardware. Optional `min_width_mm` and
`max_width_mm` are absolute opening bounds. A 20 mm target with bounds 18 and
22 mm is converted to SDK inner/outer tolerances of 2 mm each (total span 4 mm).
An omitted lower/upper bound uses target minus/plus 5 mm, bounded to 0–80 mm.
The limits describe acceptable grasp width, not extra move commands.

### Wrist vision pick and place

`vision_pick_and_drop` supports RealMan-compatible `observe`, `grab_to`,
`grab_by` and `cancel`, with `start`, `stop`, `info` and `config` lifecycle calls.
Connect depth (`image/depth-zlib`), RGB (`image/jpeg`), and VOP objects
(`data/json`) from the same wrist camera. VOP must consume that RGB topic.

Configure `vision_pick_and_drop.camera_serial` and `motion_arm_ips.right` for
the right arm. Store the left camera, observation pose and planning calibration
under `vision_pick_and_drop.arm_profiles.left`, with `motion_arm_ips.left`.
The card configuration selects `IP（左手）` or `IP（右手）`. Switching selects
that arm's saved profile and invalidates the previous photograph; run `observe`
again. Connected RGB/depth topics stay unchanged while their camera device is
switched to the selected wrist. Switching during an active action is rejected.
The left planning profile stores the commissioned 25 mm grasp-depth offset
separately from the measured hand-eye transform. Optional per-arm X/Y/Z
adjustments default to zero. Positive X/Y follow image right/down along the
table, and positive Z moves deeper along the downward tool axis.
Set `observation_joints_deg` to seven
measured FR3 angles in degrees; RealMan's default angles are invalid for FR3.
Supply `speed_deg_s` explicitly on each transfer action (0.1–5 degrees/s),
without a default. Transfer and return use that same requested joint speed limit.
The 5 degrees/s limit is the current driver commissioning cap, not a percentage
reference or the FR3 hardware maximum. No speed-percentage setting is exposed.

Call `observe` with `confirm_motion=true`; it uses a fixed 2 degrees/s observation speed without a speed input. If already within 0.2 degrees of
all observation joints, no motion command is sent. Otherwise the card sends
one joint target; review the return path before using it from other poses.
This observation-only move does not run the grasp planner. It opens the unloaded
hand to its calibrated maximum before moving, without applying grasp force.

The card retains one private aligned RGB-D snapshot and the SDK EE pose.
Public results contain `observation_id`, `captured_at`, dimensions, pose
(position in metres, RPY in radians), VOP `objects`, and synchronization data.
`position` remains image-centred normalized XY in [-1,1], not a 3D position.
The immediate response is `running` plus `action_id`; completion, error or
cancellation is posted to Core's `/api/acp/complete`, using the existing FR3
loopback TLS policy. `info.last_result.callback` records delivery status.

Input matching uses publication timestamps, not measured hardware exposure
synchronization. RGB/depth and detection/RGB skew are bounded to 200 ms;
only fresh data acquired after a stationary observation window is accepted.
Do not use this commissioning stage for moving-target grasping.

For planned FR3 grasp actions, compute the grasp target from the latest
stationary `observe` snapshot and plan the approach and lift before moving.
Use that saved target throughout the approach, without a close-range camera
or depth confirmation. This assumes the object stays stationary. If the
scene changes, acquire a new observation and replan. On hardware, use the
gripper's `is_grasped` feedback before lifting.

Grasp planning runs automatically inside `grab_to` and `grab_by`. It estimates
the table and target from the saved RGB-D snapshot, aligns the jaws, then checks
the approach, vertical descent, grasp/lift, transfer and return before motion.
Configure `vision_pick_and_drop.planning.handeye` with the matching calibration.
The 20 mm empirical grasp offset and per-arm depth correction only set the target;
they do not define the physical finger geometry. The Hand V2 collision model uses
the measured SDK `F_T_EE`, housing bounds and all four collision boxes per finger.
It checks the 0–80 mm opening envelope against the table with 5 mm clearance
along sampled pick, lift, transfer and return splines. Left-arm opening and
standalone observation returns also check the fixed calibrated table plane.
Recalibrate that plane if the bench or robot mounting moves. An unsafe target is
rejected without changing the saved 25 mm left correction. This model check is
not a contact-based six-axis TCP calibration. Insufficient depth or an
unreachable path returns an error without sending motion commands.

Action inputs follow RealMan coordinates/displacements, optional `rotation_deg`,
and `confirm_motion=true`, with FR3-specific required speed inputs: `speed_deg_s` and `speed_mm_s`.
Supply `speed_deg_s` (0.1–5 degrees/s) for arm paths and `speed_mm_s` (0.1–100 mm/s)
on each transfer for Hand opening/closing; there is no default or native Hand
percentage-speed interface. `force_n` (30–70 N) is persistent configuration and
must be set explicitly without an implicit default. RealMan raw force registers and fixed descent distances are not used;
FR3 retains depth-based grasp-height planning. Fresh `is_grasped` confirmation is required
before lifting. `info` reports progress; `cancel` stops pending commands without
automatically opening an already held object. Failed or uncertain grasps report
`holding_object_possible` and `recovery_required`.

For commissioning, use the matching right-arm/camera hand-eye profile in
`vision_pick_and_drop.planning.handeye`. Start the RGB, depth and VOP inputs,
run `observe`, select its object's normalized position, then run `grab_to` or `grab_by`.
Use `grab_by` to grab, move by actual table millimetres and place:

```json
{"action":"grab_by","start_point_x":-0.33,"start_point_y":0.32,
 "delta_x":30,"delta_y":0,"rotation_deg":0,
 "speed_deg_s":5,"speed_mm_s":20,"confirm_motion":true}
```

`delta_x` is positive toward image right; `delta_y` is positive toward image
down. The calibrated camera axes are projected onto the table plane; these
are not pixel offsets or fixed base-frame axes. Both values are required.
For `grab_to`, replace both deltas with `target_point_x` and `target_point_y`
in [-1,1] from the same saved observation. The selected placement ray must
have valid depth and intersect the fitted table. Placement assumes the same
supporting table plane and a stationary scene.

`rotation_deg` defaults to zero and accepts [-180,180]. It adds a rotation
about the downward gripper axis after lifting, clockwise viewed from above;
it does not replace automatic jaw alignment at the pick point. The complete
pick, transfer, release retreat and observation return are planned before
any command. The transfer is one continuous PATH through lift, optional
rotation, translation and descent. After opening is confirmed, the arm
retreats 150 mm then returns to the saved observation joints. Each phase
uses a continuous spline, rather than separate stop/start joint targets.

The immediate result is `state: running` with `action_id`; a single terminal
ACP callback reports `completed`, `error` or `cancelled`. `info.stage` gives
progress. Successful transfer results include `observation_id`, `pick_pixel`,
`pick_base_xy_mm`, `place_base_xy_mm`, `final_pose`, `rotation_deg`,
`rotation_completed`, `release_completed`, `transfer_completed`, `return_completed`,
`observe_after_transfer`, `observation`, `recovery_required` and
`grasp_checked`. `grab_by` additionally returns `delta_x`, `delta_y` and
`direction_reference: observation_image`; `grab_to` returns `place_pixel`.
Base XY fields retain RealMan's format; because this FR3 is side-mounted,
they must not be interpreted as the table's right/down axes. Feedback checks
Hand `is_grasped` before and during transport and confirms opening before
retreat; it does not measure actual contact force or prove object placement.
On error/cancel the card stops pending commands and never automatically
opens a possibly held object. `release_completed` distinguishes failures
before and after release. Completed transfer/return flags are preserved if a
later return or camera capture fails; do not repeat an already completed transfer.
`last_completed_stage` identifies the last confirmed motion phase.
`feedback_recoveries` is 0: FR3 currently aborts invalid feedback instead of
implementing RealMan's feedback-recovery wait loop.

Configure `observe_after_transfer` (boolean, default false) to capture a fresh
RGB-D/VOP observation after the planned return, under the same action ID and ACP
completion. When disabled, `observation` is `{"skipped":true,"reason":"disabled"}`
and `observation_required` is true. When enabled, `observation` contains the same
public fields as `observe`, including the new `observation_id` and `objects`;
`observation_required` is false and the snapshot is available for the next transfer.
The outer `observation_id` still identifies the photo used for this transfer.
Capture failure reports an error without clearing confirmed transfer/return flags.
A planning rejection before session acquisition retains the original snapshot.
FR3-specific hand speed/force configuration, finite timeouts, stage names and SDK
`is_grasped` checks remain; RealMan's fixed descent and raw force settings are not
copied to this hardware. No `/api/event` calls are used.

Use the existing VOP data viewer for normalized target coordinates. A label
may be ignored, but its position must point to the intended object, not another
part of the image. The driver does not advertise a result stream; terminal
results remain available through `info.last_result` and the ACP callback.
Depth is read automatically from the saved RGB-D frame. A single missing
centre pixel can use a consistent 5x5 neighbourhood; a sparse or mixed-depth
patch is rejected with its normalized coordinate and RGB pixel in the error.
Observation input synchronization times out after 30 seconds.
