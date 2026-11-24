# PiPER RGB-D pick preview

`piper_vision_pick_and_drop` currently provides read-only pick localization. It
publishes RGB for VOP, saves a short-lived aligned RGB-D observation, and
projects an operator-selected pixel into the camera frame and, when calibrated,
the arm base frame. The card sends no CAN commands and exposes no motion action.

## Enable the card

In the PiPER deployment YAML, enable `vision_pick.enabled` and set
`camera_topic_base` to the existing camera's topic prefix. Keep the camera
publisher running with these topics in the robot DDS domain:

- `<prefix>/color/image_raw/compressed`
- `<prefix>/aligned_depth_to_color/image_raw`
- `<prefix>/aligned_depth_to_color/camera_info`

All three must share the color optical frame and image dimensions. Depth accepts
`16UC1` in millimetres or `32FC1` in metres. Camera and robot messages must use
the host clock or a synchronized clock. VOP may consume the card's RGB topic
and publish detections to its `/objects` suffix in the core DDS domain. Detection
timestamps only allow approximate association in a stationary scene. Confirm the
pixel on the RGB image. VOP is optional for manual localization.

## Calibration

Fixed cameras may run without calibration and return camera coordinates only.
To also obtain base coordinates, configure `vision_pick.calibration` with
`id`, `arm_can_interface`, `optical_frame_id`, `reference_frame: base_link`,
a 3×3 `rotation` matrix, and a three-element `translation_mm`. The transform
maps a point from the camera optical frame into the reference frame.

For a wrist camera, set `joint_state_topic` to the actual joint feedback topic
for this arm in the robot DDS domain. It must publish time-stamped ROS
`sensor_msgs/msg/JointState` with `joint1` through `joint6`, in radians and
in the same joint convention as the URDF. Use `reference_frame: link6` and
include `arm_urdf_sha256`, the SHA-256 of the URDF used during calibration.
The default model is `resource/upstream/piper_description.urdf`. Set
`vision_pick.arm_urdf` if calibration uses a different PiPER URDF.
Wrist observations require joint samples on either side of RGB and depth
within a 100 ms window. The card rejects samples with more than 0.15° of
variation in any axis during that window.

## Check the preview

```bash
python3 agilex/piper/deploy/vision-check.py --endpoint http://localhost:15741/mcp --point 0 0
```

`--point 0 0` selects the image centre. Use `--pixel U V` for a known pixel.
The response includes `camera_xyz_mm`, optional `base_xyz_mm`, and
`executed: false`.
