# FR3 visualization model

`fr3.urdf` is a simplified, visualization-only seven-joint FR3 chain, derived
from Franka Robotics' Apache-2.0 licensed `franka_description` at commit
`7aeeddc449edf8d62b594f9e36a81da53e7796f9`:

- https://github.com/frankarobotics/franka_description/blob/7aeeddc449edf8d62b594f9e36a81da53e7796f9/robots/fr3/kinematics.yaml
- https://github.com/frankarobotics/franka_description/blob/7aeeddc449edf8d62b594f9e36a81da53e7796f9/robots/fr3/joint_limits.yaml

The upstream license and attribution notice are preserved in
`LICENSE.franka_description` and `NOTICE.franka_description`. The original
kinematics values are included as `fr3_kinematics.yaml`. Changes: expanded eight
joint origins into a standalone URDF, named links/joints with the `fr3_` prefix,
used the local Z axis for revolute joints, copied the FR3 position/velocity/effort
limits, and omitted meshes, collision shapes, inertial data, transmission and
gripper geometry. Joint 8 is the fixed flange offset. No external mesh download
is needed by the Motus skeleton renderer.

This is not a dynamics, collision-planning or motion-control model. Displayed
coordinates are relative to the robot base; installation tilt and world-frame
placement are not inferred. Tool/gripper geometry and finger states are outside
this first driver release.
