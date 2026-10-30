# Vendor message provenance

The six `.msg` files are a monitoring-only subset with unchanged field definitions of
https://github.com/DeepRoboticsLab/deep-robotics-msg at commit
`a0d1a29eec5c4db5a9107595bb51e3be8122b86c`.
Trailing blank lines are normalized. They retain the `drdds` package name and the source license. The build manifest
keeps ROS interface generation but omits optional Debian packaging.
BatteryData and JointsData use MetaType plus their nested value definitions.
IMU and LiDAR use standard sensor_msgs interfaces provided by ROS Jazzy.
No vendor motion command definitions or controller SDK are included.
