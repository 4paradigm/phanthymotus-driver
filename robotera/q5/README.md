# RobotEra Q5 driver

Uses RobotEra's `xbot_common_interfaces` and `era_nav_msgs` contracts. The driver exposes joint lifecycle and trajectories, wheel-base control, XHand control, dual-arm/head MPC, LiDAR mapping and topological navigation.

Pinned upstream revisions are recorded in the Dockerfile. Default robot DDS domain is `211`; override the network interface with `NETWORK_INTERFACE`.
