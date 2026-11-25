# LinkerHand L20 skeleton model and calibration provenance

Source: LinkerHand official URDF viewer and download site, <https://urdf-view.linkerbot.cn>, downloaded by the project team on 2026-11-12.

License: Apache License 2.0. The download contains no license file; on 2026-11-16 the vendor confirmed to the project team that these files are under the same license as their GitHub repository [linker-bot/linkerhand-urdf](https://github.com/linker-bot/linkerhand-urdf). `LICENSE` in this directory is a verbatim copy of that repository's `LICENSE` (main branch).

Model applicability: the files are labelled G20. The vendor SDK ([linkerbot-hand-legacy-cpp-sdk](https://github.com/linker-bot/linkerbot-hand-legacy-cpp-sdk) README) states that `L20_10` and `L20_11` alias `G20` and that the ordinary `L20` is a different hand; on 2026-11-16 the vendor also confirmed that these files are the correct model for the L20-10. The ordinary `L20` model in linkerhand-urdf has the same joint names but different axes, origins and limits, and must not be used for this hand.

| File | Downloaded as |
| --- | --- |
| `linkerhand_g20_left.urdf` | `LEFT/linkerhand_g20_left.urdf` |
| `linkerhand_g20_right.urdf` | `RIGHT/linkerhand_g20_right.urdf` |
| `g20_left_G20.json` | `LEFT/g20_left_G20.json` |
| `g20_right_G20.json` | `RIGHT/g20_right_G20.json` |

All four files are copied verbatim (no edits). The download also contains STL meshes under `LEFT/meshes` and `RIGHT/meshes`; they are not included, the URDFs still reference them, but the driver never serves those references.

The URDFs are SolidWorks exports (internal robot name `linkerhand_L25NS_{left,right}.urdf`). The JSON files are vendor calibration records (`schema_version` 4, `model` G20, serial `G20_LEFT_001` / `G20_RIGHT_001`, `quality.passed` true, validation MAE 0.0032 / 0.0029 rad). For every URDF revolute joint they give the driving motor (`motor_index`, the driver's 20-item vector index) and a 256-entry `angle_rad` table indexed by the raw 0..255 position. The five distal joints are marked `passive` and have their own table on their parent's motor. The index, middle and ring tables are copies of the pinky tables (`source_joint`).

## Runtime use

`joints_model.py` checks on load that each JSON covers exactly the URDF's revolute joints and that its motor assignment matches `URDF_JOINT`; it raises otherwise. Angles are table lookups without interpolation or offsets (`urdf_zero_offset_rad` is informational: every table is already 0 at its zero command).

The Core skeleton panel passes its topic to the `model` tool. `joints_model.py` selects the matching single-hand kinematics-only URDF from `/hand/left/` or `/hand/right/`; only a request without a topic falls back to the dual-hand URDF built from both files:

- Every link and joint name is prefixed with `left_` / `right_`; joint types, axes, limits and mimic relations are kept. Visual, collision and inertial elements are dropped.
- Joint origin translations are scaled by 4 because Core's fixed camera is framed for full-size robots. Each hand root is attached to `dual_hand_base` at x = -0.45 m (left) / +0.45 m (right), turned -90° about z (`ANCHOR_YAW`) so both palms face Core's default camera with the thumbs outward.
- URDF `rpy` is a fixed-axis rotation `Rz(yaw)·Ry(pitch)·Rx(roll)`, while the current Core renderer applies it as intrinsic XYZ. Origins with more than one non-zero rpy component are split into single-axis fixed helper joints (`*_rpyN_fixed`) in the order yaw, pitch, roll, giving the same transform under either interpretation.
- Core draws bones only between joint origins, so each finger gets a fixed leaf link `{side}_{finger}_tip` (joint `*_tip_fixed`) on its distal link. The offsets (`FINGERTIPS` in `joints_model.py`) continue the direction of the previous bone (the last joint's origin offset) up to the farthest extent of the downloaded distal STL mesh along that direction: thumb (-0.0044, 0, 0.0315) m, other fingers (0.0019, 0, 0.0214) m, identical for both sides. (The mesh's farthest point lies towards the back of the hand and made straight fingers look bent outward.)

Each side has 21 revolute joints: 16 driven by measured feedback and 5 passive distal joints from their calibration table (`source: derived_calibration`, not measurements). Several tables run slightly past the URDF limits (up to 0.22 rad for the right `thumb_ip`); skeleton values are clamped to the limits and flagged `clamped_to_model`, while the diagnostic sample keeps the unclamped table angle.

## Verification

- The calibration records belong to reference units (`*_001`); the vendor stated that per-unit recalibration is not needed. Offline, both models reproduce the OK and V presets. On hardware the right thumb pitch, yaw and tip directions match the model (2026-11-12), and `thumb_roll` at raw 0 and 255 matches on both hands (2026-11-16). On hardware (2026-11-13) the V preset displayed correctly on both hands after negating the left four-finger `mcp_roll` tables (`NEGATED_TABLES` in `joints_model.py`).
