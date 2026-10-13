# TRON2 skeleton model provenance

Source: [LimX Dynamics TRON2 robot description](https://github.com/limxdynamics/tron2-robot-description/blob/f547f5bc949f2a4c98e076e61cf6d3ca73d179a0/tron2a/DACH_TRON2A/urdf/robot.urdf).
Pinned commit: `f547f5bc949f2a4c98e076e61cf6d3ca73d179a0`.

`tron2_dach.urdf` is a modified, kinematics-only version for the Motus stick-figure renderer: robot/link/joint names, parent/child links, origins, axes and limits are preserved exactly. Visuals, collision geometry, inertial elements and MuJoCo metadata are removed. No mesh/USD assets, new runtime dependencies or external asset downloads are included.

License: Apache-2.0; upstream LICENSE, NOTICE and THIRD_PARTY_NOTICES.md are retained here. The upstream pending mesh provenance entries do not concern any copied mesh: none is included.

The state index mapping is the [official TRON2 manual](https://www.limxdynamics.com/zh/documents/844648486841487360) joint order: left arm 0–6, right arm 7–13, head pitch 14, head yaw 15. URDF document order puts head yaw before pitch, so document iteration order must not be used as motor order.

This model represents DACH_TRON2A dual arms with head. It is not a model of the sole/wheeled variants. Missing IMU orientation is omitted from the stream; no measured body orientation is fabricated. An empty array must not be sent: Motus treats it as a truthy quaternion and assigns four undefined components to the model root.
