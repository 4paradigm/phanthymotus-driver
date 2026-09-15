# Unitree AS2 driver

This bundle targets the Unitree **AS2** SDK service documented in the Unitree
SDK guide. The SDK package currently exposes this service as
`unitree_sdk2py.a2.sport.SportClient` (the same API names and service version
used by the AS2 firmware): `Move`, `StopMove`, `StandUp`, `StandDown`,
`BalanceStand`, `RecoveryStand`, `Damp`, `Euler`, `BodyHeight`, `BodyPosition`,
`SwitchGait`, `SpeedLevel`, `SetAutoRecovery`, `FrontFlip`, and `BackFlip`.

Run `python3 main.py <robot-interface>` on the robot network (normally the
interface with a `192.168.123.x` address). The bundle exposes MCP on port
`15704`, publishes JSON state streams under the resolved ROS namespace, and
uses a dedicated process for RPC calls so ROS callbacks cannot starve SDK
responses.

`duration=-1` starts a 10 Hz velocity command loop; `stop_move`, shutdown, and
any plugin stop path terminate that loop and issue `StopMove`. Velocity and
attitude inputs are clamped before reaching the robot. Acrobatics should only
be invoked with a clear area and appropriate operator approval.

The checked-in `resource/as2_model.urdf` is a kinematic 12-joint quadruped
descriptor for visualization. Replace it with the calibrated model supplied by
the robot deployment when available.
