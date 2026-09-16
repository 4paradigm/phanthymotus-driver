# Unitree AS2W driver

This bundle targets the Unitree **AS2W** wheel-legged robot and its SDK service documented in the Unitree
SDK guide. It vendors Unitree's official `unitree_sdk2_python` master at
`65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5` and uses its dedicated
`unitree_sdk2py.as2.sport.SportClient`: `Move`, `StopMove`, `StandUp`, `StandDown`,
`BalanceStand`, `RecoveryStand`, `Damp`, `Euler`, `BodyHeight`, `BodyPosition`,
`SwitchGait`, `SpeedLevel`, `SwitchJoystick`, `SetAutoRecovery`, `GetState`,
`FrontFlip`, and `BackFlip`.

Run `python3 main.py <robot-interface>` on the robot network (normally the
interface with a `192.168.123.x` address). If the supplied interface is absent,
the entry point tries available container interfaces and then starts MCP in a
degraded mode without DDS publishers. The bundle exposes MCP on port
`15709`, publishes JSON state streams under the resolved ROS namespace, and
uses a dedicated process for RPC calls so ROS callbacks cannot starve SDK
responses.

Deployment follows agent-core's DDS isolation contract: ROS2 uses FastDDS
Domain 42 and the mounted `/opt/phanthy-motus/dds-local.xml` loopback profile;
the Unitree SDK uses CycloneDDS Domain 0 and binds to the robot interface passed
to `main.py`. These are deliberately separate DDS implementations and domains.
The image installs the small CMake toolchain because the SDK's pinned
`cyclonedds==0.10.5` Python binding must link against a matching CycloneDDS
build. During the image build, Docker selects the target architecture, downloads
only its CRC `.so` from the pinned official Unitree SDK commit through jsDelivr's
GitHub CDN, and verifies the artifact against a fixed SHA-256 digest.
Architecture-specific binaries are not stored in this repository.

`duration=-1` starts a 10 Hz velocity command loop; `stop_move`, shutdown, and
any plugin stop path terminate that loop and issue `StopMove`. Velocity and
attitude inputs outside the documented bounds are rejected before reaching the
robot. `timed_move` requires a positive `duration`, runs asynchronously, returns
an ACP `action_id`, and reports completion after `StopMove`; it never blocks the
MCP request thread. Positive durations passed to `move` are rejected so its
immediate and continuous modes are never mistaken for ACP actions.
Special actions should only be invoked with a clear area and appropriate
operator approval. Conservative AS2 limits are enforced for Euler attitude
(roll ±0.2 rad, pitch/yaw ±0.3 rad), body height (±0.3 m), body position
(x/y/z ±0.2 m), speed level (-1/0/1), and the SDK-documented gait type 0.

The checked-in `resource/as2w.urdf` kinematic model is based on Unitree's
official `unitree_ros/robots/as2w_description`; it retains inertial and joint
limits but omits the vendor STL visual/collision meshes. The driver only needs
the kinematic chain for the `joints` skeleton card, avoiding large binary
assets in the repository. AS2W has 16 movable joints (12 leg joints plus 4
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

`remote_controller` decodes the AS2W `LowState_.wireless_remote[40]` payload
with the button and axis layout defined by the vendored official AS2 SDK. It
publishes 14 buttons, 4 axes, activity and freshness state at up to 10 Hz on
`/{namespace}/state/remote_controller`; MCP `info` and `read` are also
supported.

`trajectory_motion` generates bounded 10 Hz velocity trajectories through the
official `SportClient.Move` API. Its atomic actions are `circle`,
`figure_eight`, and `slalom`; each has a finite duration, publishes an action
identifier, and can be interrupted immediately with `stop`.
Starting any generated trajectory requires an explicit `confirm=true`. Terminal
success, cancellation, and error states are reported to Agent Core through ACP.

`motion_recorder` records timestamped velocity samples submitted through its
own `drive` action and stores them under
`/opt/phanthy-motus/data/as2w-motion-recordings`. It supports
`record_start`, `record_stop`, `play`, `stop_playback`, `list`, `delete`, and
`status`. Playback is mutually exclusive with generated trajectories and always
ends with `StopMove`. Calls made directly to the separate `loco` card are not
implicitly recorded; a workflow that needs recording must route velocity
commands through `motion_recorder.drive`.
Both `drive` and `play` require `confirm=true`; playback also reports its
terminal state through ACP so the physical-action barrier is released promptly.
