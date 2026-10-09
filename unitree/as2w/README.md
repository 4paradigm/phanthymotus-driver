# Unitree As2W driver

This bundle targets the Unitree **As2W** wheel-legged robot and its SDK service documented in the Unitree
SDK guide. It vendors Unitree's official `unitree_sdk2_python` master at
`65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5` and uses its dedicated
`unitree_sdk2py.as2.sport.SportClient`: `Move`, `StopMove`, `StandUp`, `StandDown`,
`BalanceStand`, `RecoveryStand`, `Damp`, `Euler`, `BodyHeight`, `BodyPosition`,
`SwitchGait`, `SpeedLevel`, `SwitchJoystick`, `SetAutoRecovery`, `GetState`,
`FrontFlip`, and `BackFlip`.

The driver accepts an explicit robot interface from a command-line argument or
`NETWORK_INTERFACE`, for example `NETWORK_INTERFACE=eno1`. Command-line values
take priority over the environment, followed by `robot_interface` in
`config.yaml`. An empty value or `auto` enables safe discovery: the driver
selects a unique, UP, wired, non-virtual interface on Unitree's
`192.168.123.0/24` robot network. It never falls back to an arbitrary Wi-Fi or
office-network interface. If no unique robot interface can be identified, or
CycloneDDS cannot bind to the selected interface, MCP starts in degraded mode
without DDS publishers. The bundle exposes MCP on port
`15709`, publishes JSON state streams under the resolved ROS namespace, and
uses a dedicated process for RPC calls so ROS callbacks cannot starve SDK
responses.

Deployment follows agent-core's DDS isolation contract: ROS2 uses FastDDS
Domain 42 and the mounted `/opt/phanthy-motus/dds-local.xml` loopback profile;
the Unitree SDK uses CycloneDDS Domain 0 and binds to the robot interface passed
to `main.py`. These are deliberately separate DDS implementations and domains.
The image installs the small CMake toolchain because the SDK's pinned
`cyclonedds==0.10.5` Python binding must link against a matching CycloneDDS
build; the vendored CRC `.so` files are the official SDK's architecture-specific
runtime dependencies and are required on both amd64 and aarch64.

`duration=-1` starts a 10 Hz velocity command loop; `stop_move`, shutdown, and
any plugin stop path terminate that loop and issue `StopMove`. Velocity and
attitude inputs are clamped before reaching the robot. Special actions should only
be invoked with a clear area and appropriate operator approval.

The checked-in `resource/as2w.urdf` kinematic model is based on Unitree's
official `unitree_ros/robots/as2w_description`; it retains inertial and joint
limits but omits the vendor STL visual/collision meshes. The driver only needs
the kinematic chain for the `joints` skeleton card, avoiding large binary
assets in the repository. As2W has 16 movable joints (12 leg joints plus 4
continuous wheel-foot joints) and the fixed JT128 sensor mount.

`controlled_spatial` is a thin adapter for Unitree's documented `slam_operate`
service: mapping, relocalization, and point-goal navigation. The latest AS2
SDK does not package a model-specific SLAM client, so the driver implements the
documented common RPC contract directly in an isolated CycloneDDS process. It
requires the vendor `unitree_slam` service to be installed and already running
on the robot or extension host; the driver does not start that service.

`special_action` exposes the AS2 SportClient's `FrontFlip`, `BackFlip`,
`HandStand`, and `BipedStand` actions. It is intentionally separate from the
continuous `loco` control card.

No-hardware checks are available with `python3 test_driver.py`; they cover
action lifecycle, schemas, model resources, and full-size low-state arrays.
