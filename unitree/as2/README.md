# Unitree AS2 driver

This bundle targets the Unitree **AS2** SDK service documented in the Unitree
SDK guide. It vendors Unitree's official `unitree_sdk2_python` master at
`65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5` and uses its dedicated
`unitree_sdk2py.as2.sport.SportClient`: `Move`, `StopMove`, `StandUp`, `StandDown`,
`BalanceStand`, `RecoveryStand`, `Damp`, `Euler`, `BodyHeight`, `BodyPosition`,
`SwitchGait`, `SpeedLevel`, `SwitchJoystick`, `SetAutoRecovery`, `GetState`,
`FrontFlip`, and `BackFlip`.

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

The official AS2 SDK currently does not include an AS2 SLAM or navigation
client. Therefore this bundle intentionally does not advertise `controlled_spatial`:
the Go2 implementation depends on a different `g1.slam.SlamClient` plus host
`unitree_slam` binaries, neither of which is an AS2 SDK contract. `lidar_cloud`
is available for consuming the AS2 LiDAR stream; add controlled mapping only
after an AS2 firmware deployment supplies and validates a compatible SLAM service.
