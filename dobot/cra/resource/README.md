# Official skeleton model

`cr3_robot.urdf` is copied with trailing whitespace normalized from:

- Repository: https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4
- Commit: `ec201c40d5db63185a3593ec9c39f80fe25aa527`
- Path: `dobot_rviz/urdf/cr3_robot.urdf`
- SHA-256: `e79a0f39c0c06f2a645031f3a3d92bc111e2968844bb30a59846ddc838376bc7`
- License: MIT, retained as `LICENSE.dobot`.

The V4 CRA ROS model uses joint1..joint6. The model resource derives a display-only URDF from the original file: the
joint4 offset is split into perpendicular rigid segments using fixed +90/-90
frames. Compound RPY origins are factored into single-axis fixed rotations so
the existing platform renderer and standard URDF FK agree. All six original
actuated joints and all physical link transforms are preserved. No STL meshes,
RViz, MoveIt or platform modifications are required.

The supplied TCP/IP V4.6.6 manual maps feedback `CRRobotType=113` to CR3A.
That type was also observed directly from the target controller. This driver
selects the official V4 `cr3` chain for that type only. It does not select it for
other payload/geometry variants based on the CC262 controller model or a name
prefix. The model is for state visualization, not motion planning or calibration.

The upstream publisher sends SDK joint positions under joint1..joint6; our
feedback adapter supplies the same order with degree-to-radian conversion.
No arbitrary joint-angle offsets, direction changes or IMU orientation are added.
