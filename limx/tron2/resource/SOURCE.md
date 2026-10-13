# TRON2 skeleton model provenance

Source: [LimX Dynamics TRON2 robot description](https://github.com/limxdynamics/tron2-robot-description/blob/f547f5bc949f2a4c98e076e61cf6d3ca73d179a0/tron2a/DACH_TRON2A/urdf/robot.urdf).
Pinned commit: `f547f5bc949f2a4c98e076e61cf6d3ca73d179a0`.

`tron2_dach.urdf` is a modified, kinematics-only version for the Motus stick-figure renderer: All original robot/link/joint names, parent/child links, origins, axes and limits are preserved exactly. Additional `display_hand_gripper_*` fixed links describe two-finger schematic outlines attached to the existing `grasper_L_Link` and `grasper_R_Link`; these are display geometry, not extra actuated joints or reported feedback. Visuals, collision geometry, inertial elements and MuJoCo metadata are removed. No mesh/USD assets, new runtime dependencies or external asset downloads are included.

License: Apache-2.0; upstream LICENSE, NOTICE and THIRD_PARTY_NOTICES.md are retained here. The upstream pending mesh provenance entries do not concern any copied mesh: none is included.

The state index mapping is the [official TRON2 manual](https://www.limxdynamics.com/zh/documents/844648486841487360) joint order: left arm 0–6, right arm 7–13, head pitch 14, head yaw 15. URDF document order puts head yaw before pitch, so document iteration order must not be used as motor order.

This model represents DACH_TRON2A dual arms with head. It is not a model of the sole/wheeled variants. Missing IMU orientation is omitted from the stream; no measured body orientation is fabricated. An empty array must not be sent: Motus treats it as a truthy quaternion and assigns four undefined components to the model root.

## Gripper display outlines

The Motus skeleton renderer draws joint-origin connectors and fixed endpoints rather than URDF visual meshes. The driver adds eight fixed helper links/joints (two bases and two tips per gripper) to make the two installed parallel grippers recognizable using that existing renderer, without a Core change or any copied mesh asset. The original 16 motors, their transforms and state mapping remain unchanged.

These original schematic connectors use a closed default display: the two finger centerlines are 8 mm apart, so the existing renderer's 4 mm radius finger cylinders touch; display finger length remains 65 mm. This is not an 8 mm jaw opening. Their dimensions and fixed closed state are visualization choices, not calibrated stock-gripper CAD, tool-center-point coordinates, collision geometry or measured jaw position. The outlines follow their respective real wrist transforms. The current 16-joint feedback does not contain jaw opening; the driver does not publish synthetic gripper angles or animate unmeasured opening.
