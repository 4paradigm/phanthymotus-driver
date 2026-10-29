# Exit teaching and hold

`arm_control` now exposes `exit_teaching_and_hold` with the English label
**Exit teaching and hold**. It takes no parameters and participates in the same
asynchronous task, completion, Stop and feedback lifecycle as Move. Move itself
continues to reject teaching mode.

The action requires fresh, finite, in-limit joint feedback, no fault, all six
arm motors already enabled, and teaching recording/playback stopped. It queries
the firmware on the current connection and accepts recognized PiPER 1.x
versions from `S-V1.8-5` onward; older or unknown versions fail without a mode
command. Already being in CAN control returns an explicit no-op.

Before switching, joints must remain within 0.2 degrees of the initial sample
for 0.3 seconds. The latest measured six-axis pose is captured once, without
clamping, and sent with CAN / MOVE_J / position-velocity mode at 1% speed through
the existing pinned SDK. No ResetPiper, MotionCtrl_1, homing, enable, disable,
gripper or joint-zero command is part of this action.

Completion requires fresh feedback newer than the command, CAN mode, all arm
motors enabled, the controller's reached flag, and each joint within 0.5 degrees
of the captured pose continuously for 0.5 seconds. The switch/hold deadline is
5 seconds. Mode ownership loss, fault, stale feedback, changed enables or drift
above 1 degree fails the task. Stop prevents further old targets and allows
one best-effort hold at the latest measured pose only if CAN control remains
healthy and enabled. It cannot undo an already transmitted mode change or
guarantee a hold after feedback/control is lost. No failure falls back to reset.

The completion result includes the firmware, measured/target angles and maximum
observed drift. `info` and the background parameter stream expose actual task
phase/results; accepting a task is not completion.

## Evidence and limits

Official ROS driver reference, pinned for review:

- [Firmware requirements](https://github.com/agilexrobotics/agx_arm_ros/blob/bc485ab8e3d25abbcd3a1b58c105be12bad5dadc/README_EN.md).
- [Seamless-mode detection and measured-joint hold](https://github.com/agilexrobotics/agx_arm_ros/blob/bc485ab8e3d25abbcd3a1b58c105be12bad5dadc/src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py).
- Existing SDK remains `8d4dc29bd6e753066dedee235c008e90ecb2ab4a`; pyAgxArm
  source was consulted, not installed.

On 2026-10-14, a firmware information query on the isolated new slave's CAN1
returned `S-V1.9-0`. This establishes the documented version condition only.
Physical mode transition, torque continuity and position drift are **not yet
accepted on this device**. Initial physical acceptance must record before/after
joint and mode feedback and observe the actual arm; simulation is not proof of
zero movement. Keep this as a user-test candidate until that acceptance passes.

Tests: `tests/test_piper_exit_teaching.py` uses a recording SDK to cover the
contract, firmware gate, command ordering, fixed measured target, actual
completion, timeout, mode/fault/feedback/enable changes, unexpected drift,
concurrency and Stop before/after the switch.

Common agreements remain: Agent Core unchanged; only arm 1 deployed; preserve
arm 2, live-camera changes, local smoke-test.py, configuration and backups.
No automatic physical acceptance, PR push or Bot request as part of adding
this test candidate.
