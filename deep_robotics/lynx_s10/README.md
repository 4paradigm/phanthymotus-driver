# 云深处山猫 S10 Driver

This driver works immediately with standard ROS2 `Twist`, `PoseStamped`,
`JointState`, `Imu`, `Odometry`, `BatteryState`, `Image`, and `PointCloud2`
contracts. Vendor-specific gait, posture, recovery, stunt/dance, mapping, patrol,
follow and docking commands use the correlated JSON bridge described in
`common/ros2_json_bridge.py`.

The adapter reports publication separately from physical acknowledgement. Once
the supplier delivers the Lynx S10 message/action mapping, bind those commands in
`device.py` while keeping the MCP contract unchanged.

The robot has not arrived yet. Standard message wiring and acknowledgement
semantics are contract-tested, but the actual Lynx S10 topic names, supplier IDL,
gait/action catalog, safety limits and every physical workflow still require
supplier confirmation and real-hardware validation.

The Docker image pins the official `DeepRoboticsLab/deep-robotics-msg` ROS2
interface package. Its public messages are available for the typed supplier
binding, but the official repository does not currently document which of those
topics and state values are enabled on Lynx S10 firmware.
