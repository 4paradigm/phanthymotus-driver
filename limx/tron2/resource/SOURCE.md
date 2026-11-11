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

## WF/SF configuration-specific skeletons

`tron2_wf.urdf` and `tron2_sf.urdf` derive from the same pinned official commit above:

- [WF_TRON2A/urdf/robot.urdf](https://github.com/limxdynamics/tron2-robot-description/blob/f547f5bc949f2a4c98e076e61cf6d3ca73d179a0/tron2a/WF_TRON2A/urdf/robot.urdf), source SHA-256 `825d4ddbe9715db035ed689570d8af60629c35e22bde0de7e5ab12dfa069cec6`.
- [SF_TRON2A/urdf/robot.urdf](https://github.com/limxdynamics/tron2-robot-description/blob/f547f5bc949f2a4c98e076e61cf6d3ca73d179a0/tron2a/SF_TRON2A/urdf/robot.urdf), source SHA-256 `dc776dc6230938bfa38e2b71ea92bd8ecab8a2e2199f9fbd920deba6aa2ee282`.

All original link/joint names, parent/child links, joint types, origins, axes and limits are preserved. Visuals, collision/inertial elements and MuJoCo metadata are removed; no mesh assets are redistributed or downloaded at runtime. The existing Apache-2.0 license and notices apply. Integrity tests pin hashes of the original kinematic elements separately from display helpers.

The [official manual](https://www.limxdynamics.com/zh/documents/844648486841487360) section 1.5 defines indices0–4 as left hip pitch/roll/yaw, knee and wheel (WF) or ankle pitch (SF), followed by the corresponding right-side indices5–9. The two 10-joint variants cannot be distinguished by array length. Selection uses the reported startup ACCID and model and joint data share this selection. Factory joint angles are passed through unchanged: no home pose, hip-yaw offset, normalization, root IMU quaternion or world position is fabricated.

The renderer draws kinematic connectors, not meshes, so each wheel has four `display_wheel_*` fixed spoke endpoints (nominal display radius75mm); each sole has two `display_sole_*` fixed endpoints (nominal toe130mm, heel70mm, offset40mm below ankle). These are original schematic drawing dimensions, not calibrated wheel/foot CAD, collision dimensions or measured contact points. They follow the corresponding real wheel/ankle transforms but do not add actuated joints or synthetic feedback. There are still exactly10 real joints for each lower-limb model. Footed SF is covered by model/contract tests and remains unverified on this physical robot.


下肢显示变换：display_world固定根节点的旋转由本连接首次有效准备姿态reference_q与厂家URDF正运动学计算，使两侧膝到轮轴/踝的下腿段平行显示地面；不使用头部朝上规则。该旋转随后保持固定，原base_Link、电机关节轴、限位与实时q不变。不是厂家新增电机、IMU数据或实测重力方向。
