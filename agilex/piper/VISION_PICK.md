# vision_pick_and_drop

This PiPER actuator uses aligned RGB-D to pick from an operator-selected point,
carry the object to a configured drop point, then open the gripper and retract.
It uses the same arm connection, motion permit, and interrupt handling as Arm
control.

## Enable the card

Configure the camera calibration in the PiPER deployment YAML and set
`vision_pick.enabled: true` to load the card for pick-point preview. Configure
the verified TCP and transfer geometry before using `grab_to`. Set
`camera_topic_base` to the existing camera's topic prefix. The robot DDS domain
must contain these topics:

- `<prefix>/color/image_raw/compressed`
- `<prefix>/aligned_depth_to_color/image_raw`
- `<prefix>/aligned_depth_to_color/camera_info`

The card is discovered through MCP when enabled in a configured driver.
Set the driver's `sdk_protocol: v3` for V3 firmware such as `S-V1.9-0`.
For a former follower arm, complete the documented AgileX control-mode
handoff on site before granting computer motion permission.

All three must share the color optical frame and image dimensions. Depth accepts
`16UC1` in millimetres or `32FC1` in metres. Camera and robot messages must use
the host clock or a synchronized clock. VOP may consume the card's RGB topic
and publish detections to its `/objects` suffix in the core DDS domain. Detection
timestamps only allow approximate association in a stationary scene. Confirm the
pixel on the RGB image. VOP is optional for manual localization.

## Calibration

For a fixed camera, configure `vision_pick.calibration` with
`id`, `arm_can_interface`, `optical_frame_id`, `reference_frame: base_link`,
a 3×3 `rotation` matrix, and a three-element `translation_mm`. The transform
maps a point from the camera optical frame into the reference frame.

For a wrist camera, use `reference_frame: link6` and
include `arm_urdf_sha256`, the SHA-256 of the URDF used during calibration.
The default model is `resource/upstream/piper_description.urdf`. Set
`vision_pick.arm_urdf` if calibration uses a different PiPER URDF.
By default the card reads cached six-axis CAN feedback from the shared PiPER
driver. Start Arm control to establish its read connection if needed. For
installations with a separate joint publisher, `joint_state_topic` can select
time-stamped ROS `sensor_msgs/msg/JointState` in the robot DDS domain, with
`joint1` through `joint6` in radians and the same convention as the URDF.
Wrist observations require joint samples on either side of RGB and depth
within a 100 ms window. The card rejects samples with more than 0.15° of
variation in any axis during that window.

Set `intent_control.tcp_offset_mm_deg` to the measured SDK end-frame to
gripper-tip transform. Set `intent_control.tcp_verified` after verifying it on
the installed gripper. The arm must provide fresh SDK end pose and joint
feedback before a transfer can start.

## Configure the workcell

Set these fields in `vision_pick.transfer` using the installed workcell dimensions:

| Field | Value |
| --- | --- |
| `workspace_mm` | Three pairs of inclusive TCP limits for base X, Y and Z |
| `pick_tcp_offset_mm` | Base-frame offset from the observed surface point to the gripping TCP |
| `drop_tcp_xyz_mm` | Base-frame TCP coordinates for release |
| `transit_z_mm` | Verified travel height at least 10 mm above both pick and drop TCPs |
| `tool_rpy_deg` | Installed gripping orientation, used throughout the transfer |
| `gripper_open_close_mm` | Release opening and empty-close target inside the effective gripper limits |
| `contact_width_mm` | Measured gripping-width interval for the supported object |
| `min_contact_effort_nm` | Validated minimum magnitude of measured grip effort |
| `speed_percent` | SDK speed percentage as an integer from 1 to 5 |

The workspace limits constrain TCP targets. Check the whole arm's swept path
and the held object against the fixture, table and other arm before enabling
motion. The card uses linear SDK moves and requires a stationary start at the
configured transit height and tool orientation, with both arm and gripper
enabled in CAN control.

## Use on the canvas

Add the PiPER driver's `vision_pick_and_drop` actuator. For manual localization
it is the only card needed. The PiPER deployment supplies color, aligned depth,
camera intrinsics and wrist joint feedback over ROS and CAN. To use object
detections, add `vop` and connect the actuator's `image/jpeg` output to the VOP
image input. Use the vocabulary for the objects in the scene. The driver reads
the VOP output on `<RGB topic>/objects` automatically. `info.topic_out` shows
the RGB topic and `info.source.objects_topic` shows the return topic. VOP needs
no return connection on the canvas.

1. Use `info` to check camera and motion status.
2. Run `observe` with the arm and object stationary. Use its observation ID
   and normalized image coordinates with `plan_pick`. Use `start_point_x` and
   `start_point_y` with the same convention as RealMan and FR3. With VOP, copy
   `objects[].position[0]` and `[1]` from the observation. The image centre is
   `(0, 0)`, and X and Y increase to the right and down.
   Preview returns the selected base-frame point while transfer setup is
   pending. `info.transfer_setup_error` identifies the remaining setup.
3. Review the returned waypoints. With the shared PiPER motion permit open,
   pass the returned plan ID to `grab_to` and confirm the path and workspace.
4. Read `info` or wait for ACP completion. `cancel` stops an active
   operation. After a stopped or failed transfer that may have moved the arm,
   recover the arm and check the gripper on site before restarting the driver.

To check camera and calibration without sending motion:

```bash
python3 agilex/piper/deploy/vision-check.py --point 0 0
```
