# RobotEra Q5 driver

Uses RobotEra's `xbot_common_interfaces` and `era_nav_msgs` contracts. The driver exposes joint lifecycle and both vendor trajectory APIs, wheel-base and body position control, XHand/XHand Lite, dual-arm/head MPC, recorded gestures, audio playback/volume, battery state, RealSense RGB/depth snapshots, LiDAR mapping, topological navigation, and custom choreography.

Pinned upstream revisions are recorded in the Dockerfile. Default robot DDS domain is `211`; override the network interface with `NETWORK_INTERFACE`. Q5 software `B4.1_V1.3.0` currently documents position control only: velocity, feed-forward, `kp`, and `kd` fields are transmitted for message compatibility but are ignored by that firmware.

Hardware varies by SKU: Air/Air+ use XHand 1 Lite, Pro/Pro+ use XHand 1, and Jetson/navigation availability is model-dependent. Recorded gestures and audio assets must first be uploaded through XOS; navigation also requires the vendor navigation option.

This implementation is contract-tested against the Q5 V2.0 manual and pinned public ROS2 definitions. The robot has not arrived yet, so real-hardware validation, safe speed/acceleration limits, action-name inventory, and installed optional hardware remain pending.
