# Source of `panther.urdf`

| | |
|---|---|
| Robot | UniX AI Panther, URDF name `mh3` (xacro source on the robot: `/opt/ros/humble/share/mh3_description/urdf/mh3.urdf.xacro`) |
| Captured | 2026-11-10, `ros2 topic echo /robot_description --once --field data`, on a Panther running ROS 2 Humble |
| Capture sha256 | `45cff333b0f7997d9b5ff9564b101736e51a9de80d9b16a5f220b96f7dae461f` (143,384 bytes, UTF-16 with BOM as written by PowerShell's `>`) |
| Reduced by | `tools/strip_urdf.py` (`python tools/strip_urdf.py CAPTURE resource/panther.urdf`) |

## What was removed

Only things the skeleton renderer does not use and that do not exist in the image:

- every `<visual>`, `<collision>` and `<inertial>` block (104 `file:///opt/ros/humble/...` mesh references);
- the four `<gazebo>` blocks and five `<ros2_control>` blocks. The latter contain nested
  `<joint name="joint_la0">`-style elements that a tag-based parser would take for real joints.

57 links and 56 joints remain. Joint origins, axes, types and limits are the vendor's, unmodified.

## Properties worth knowing

- 18 joints are `revolute`: `joint_la0..7`, `joint_ra0..7`, `joint_head0`, `joint_head1`. They are
  exactly the 18 names `/joint_states` carries, which is what the renderer matches on.
- `body_joint` (the lift) is `fixed` with limit 0..0.6, but the robot reported 0.61. The limit is
  stale and the joint is not drawn moving. The gripper joints are all `fixed`.
- `joint_head0` is a pitch axis (y) and `joint_head1` a yaw axis (z); the vendor SDK document
  describes them the other way round.
- `joint_ra7` allows +-1.57 where `joint_la7` allows +-0.69; this is as received.
