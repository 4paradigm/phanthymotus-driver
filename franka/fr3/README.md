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
| `controller_configuration` | Configuration | SSH credentials for automatic PNP controller discovery |

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
Both cards use the Driver's verified Controller Configuration to resolve the selected
side. No canvas connection is required. Without a verified mapping, new motion is
rejected and stop remains available. Address changes apply while idle and release
the card's previous sessions. Active motion or a known held object blocks a change.

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

With `motion_arm_ips` configured, actions select the physical left or right arm.
The Driver resolves the selected side using the mapping verified at startup.
Use each controller's C2 (Shop Floor network) IP, match the physical mounting
side, and give the arms distinct addresses.

## Grasp limits

Grasp requires `width_mm`, `speed_mm_s` and `force_n`; missing fields are listed
in the error before connecting to hardware. Optional `min_width_mm` and
`max_width_mm` are absolute opening bounds. A 20 mm target with bounds 18 and
22 mm is converted to SDK inner/outer tolerances of 2 mm each (total span 4 mm).
An omitted lower/upper bound uses target minus/plus 5 mm, bounded to 0–80 mm.
The limits describe acceptable grasp width, not extra move commands.

## Visual pick and place

`vision_pick_and_drop` provides `observe`, `grab_to`, `grab_by`, and `cancel`.
Select only `left` or `right`; camera assignments, observation joints, tool
geometry and accepted hand-eye calibration are stored per physical side in
`config.yaml`. The deployed .3/262922070633 pair is left and the
.2/263022071451 pair is right. Endpoint changes do not recalibrate a robot.

Connect these inputs, in any order:

| Input | Format | Contract |
| --- | --- | --- |
| RGB | `image/jpeg` | Selected wrist camera's `/rgb` topic |
| Depth | `image/depth-zlib` | Same camera's `/depth` topic |
| Objects | `data/json` | VOP must consume that RGB; output is `/rgb/objects` |

Configure `force_n` (30–70 N), `speed_deg_s` and `speed_mm_s` before transfer.
Speeds are joint degrees/s and total finger-spacing mm/s, not percentages.
`observe_after_transfer` optionally captures another observation after returning.
Legacy saved IP-bearing arm labels migrate by physical side. Dual-arm execution requires the IP input; configured YAML defaults alone do not
authorize motion.

`observe` uses the configured joint speed limit and returns object names,
normalized positions and confidence. Position is image-centered XY in [-1,1],
with positive X right and positive Y down; it is not a robot-base coordinate.
The driver retains aligned RGB-D, camera calibration and the captured SDK EE
pose privately. Publication timestamps are bounded to 200 ms skew; they are not
hardware exposure synchronization. Objects must remain stationary after capture.

Both transfer actions automatically observe when no saved photograph is available:

- `grab_to`: `start_point_x/y` and `target_point_x/y`, all from the same image.
- `grab_by`: `start_point_x/y` plus `delta_x/y` in physical table millimetres,
  aligned with image right/down. A displacement or rotation must be nonzero.
- Both require `confirm_motion=true`; optional `rotation_deg` is [-180,180],
  clockwise viewed from above, applied after lifting.

The driver requests plans from MoveIt on the PNP host over SSH and executes
the validated trajectories on Orin with libfranka. It checks joint limits, sampled paths, modeled arm clearance
and Hand/table clearance. Environment geometry beyond those models is not
certified. Unaccepted calibration, insufficient depth or infeasible paths reject
transfer. The original calibration matrices are retained when an IP changes.
The Hand's `is_grasped` feedback is checked before and during transport; it does
not measure contact force or prove placement.

Transfers return `running` with an `action_id`; one ACP callback reports completion,
error or cancellation. `info` retains the result, stage, mapping and input readiness.
Check `transfer_completed`, `release_completed`, `return_completed`,
`last_completed_stage`, `observation_required` and `recovery_required` before retrying.
Cancellation never automatically opens a possibly held object. A failed return or
post-transfer capture does not erase an already completed placement.

## Controller Configuration

The Driver starts discovery before initializing the camera and motion cards.
No canvas instance is required. The configuration panel contains only the SSH
username and password, which default to `pnp` and `ros`. Saving changed
credentials triggers discovery again. The panel has no input or output ports
and does not expose the discovered addresses.

Discovery probes the robot Ethernet network and then tries the last verified
host if necessary. Set `FRANKA_ROBOT_INTERFACE` to select the interface. The
Driver verifies the host key under `/opt/phanthy-motus/franka-controller-access`
with alias `franka-pnp-controller` and reads the existing Desk and hardware
configuration. Both sides must be present and agree where the files overlap.

The Driver stores verified addresses and credentials separately under
`/opt/phanthy-motus/franka-ip-sync`. Credentials are private to the Driver and
never included in action results. Each Driver boot verifies the mapping
before accepting new motion. Joint control, gripper and visual pick use the
same mapping, and visual pick also uses the shared SSH credentials.
