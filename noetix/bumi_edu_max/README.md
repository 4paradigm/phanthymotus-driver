# BumiEDU Max driver

This bundle directly wraps the public `Noetix-Robotics/noetix_sdk_bumi` native
DDS SDK at pinned revision `052ea95b`. It exposes high-level walking/running,
all public `ControlCmd` actions, three dance programs, gestures, teaching
record/save/playback, fall recovery, state telemetry, and EDU low-level motor
control. The low-level API defaults to the SDK's public 21-motor example and is
configurable with `control.joint_count`.

The driver uses ROS2 only to bridge normalized state to Agent Core; commands to
the robot use the vendor SDK directly. Set `NETWORK_INTERFACE`, `ROS_NAMESPACE`,
or `BUMI_SDK_PATH` as required by the deployment.

Before hardware validation, obtain the EDU Max joint names/limits and confirm
the supplied `dds.xml`, firmware command table, action durations and safety
state machine against the robot firmware version.
