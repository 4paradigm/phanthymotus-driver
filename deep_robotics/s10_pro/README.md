# Deep Robotics S10Pro driver adapter

This driver works immediately with standard ROS2 `Twist`, `PoseStamped`,
`JointState`, `Imu`, `Odometry`, `BatteryState`, `Image`, and `PointCloud2`
contracts. Vendor-specific gait, posture, recovery, stunt/dance, mapping, patrol,
follow and docking commands use the correlated JSON bridge described in
`common/ros2_json_bridge.py`.

The adapter reports publication separately from physical acknowledgement. Once
the supplier delivers the S10Pro message/action package, bind those commands in
`device.py` while keeping the MCP contract unchanged.

The robot has not arrived yet. Standard message wiring and acknowledgement
semantics are contract-tested, but the actual S10Pro topic names, supplier IDL,
gait/action catalog, safety limits and every physical workflow still require
supplier confirmation and real-hardware validation.
