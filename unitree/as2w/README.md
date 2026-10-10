# Unitree As2W driver

This bundle targets the Unitree **As2W** wheel-legged robot and its SDK service documented in the Unitree
SDK guide. It vendors Unitree's official `unitree_sdk2_python` master at
`65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5` and uses its dedicated
`unitree_sdk2py.as2.sport.SportClient`: `Move`, `StopMove`, `StandUp`, `StandDown`,
`BalanceStand`, `RecoveryStand`, `Damp`, `Euler`, `BodyHeight`, `BodyPosition`,
`SwitchGait`, `SpeedLevel`, `SwitchJoystick`, `SetAutoRecovery`, `GetState`,
`FrontFlip`, `BackFlip`, `HandStand`, and `BipedStand`.

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
attitude inputs are clamped before reaching the robot. Special motions should only
be invoked with a clear area and appropriate operator approval.

The checked-in `resource/as2w.urdf` kinematic model is based on Unitree's
official `unitree_ros/robots/as2w_description`; it retains inertial and joint
limits but omits the vendor STL visual/collision meshes. The driver only needs
the kinematic chain for the `joints` skeleton card, avoiding large binary
assets in the repository. As2W has 16 movable joints (12 leg joints plus 4
continuous wheel-foot joints) and the fixed JT128 sensor mount. The live
`joints`/`joint_state` cards intentionally publish the 12 populated leg-motor
slots from LowState; the remaining vendor array slots are reserved zeros.

`controlled_spatial` is a thin adapter for Unitree's documented `slam_operate`
service: mapping, relocalization, and point-goal navigation. The latest AS2
SDK does not package a model-specific SLAM client, so the driver implements the
documented common RPC contract directly in an isolated CycloneDDS process. It
requires the vendor `unitree_slam` service to be installed and already running
on the robot or extension host; the driver does not start that service.

`slam_mapping` is the matching read-only visualization card. It listens to the
vendor mapping and relocation point-cloud topics plus `rt/slam_info`, keeps a
bounded voxel map, and publishes the shared `sensor/mapping` packet on
`/<namespace>/spatial/mapping`. The control and visualization cards are kept
separate, following the G1/Go2 card contract, so stopping the dashboard view
does not interrupt mapping or navigation.

`special_motion` exposes the AS2 SportClient's `FrontFlip`, `BackFlip`,
`HandStand`, and `BipedStand` actions. It is intentionally separate from the
continuous `loco` control card. Dangerous motions validate the current FSM
state before dispatch, while the existing asynchronous completion and
cancellation behavior is retained. Timed and continuous locomotion actions
return action IDs and report completion, cancellation, or RPC failure through
ACP; `stop_move` also returns immediately and completes through ACP so a slow
firmware RPC cannot hold the MCP request open. If AS2 reports a standing state
but rejects the first velocity command, the driver performs the required hidden
balance transition and retries. AS2 `AI_*` standing, walking, and down states
are recognized explicitly.

High-rate LowState, BMS, and sport-state callbacks retain only their newest
sample and publish from a 60 Hz worker, preventing stale JSON work from
blocking DDS callbacks. Lidar runs in a separate OS process and uses a one-frame
queue. It probes for an already-installed CuPy CUDA runtime without downloading
one: CPU fallback is capped at 2,000 latest points, while CUDA may use the
configured 12,000-point budget. `led` controls the AS2 RGB LED through a
separate audio-service worker, so its keepalive cannot delay locomotion or
speaker PCM.

The multimedia cards use transports verified on As2W hardware. `speaker`
streams PCM-16k through the A2 `voice` service, while `camera` publishes JPEG
frames returned by the Go2-compatible `videohub` service. `mic` listens for the
official A2 multicast stream at `239.168.123.161:5555`; when the robot's voice
assistant / wake-up conversation mode is disabled, the card remains in a
diagnostic `waiting` state and automatically recovers when packets appear.
Speaker ROS subscription, jitter buffering, and the A2 audio client now live in
one spawned process. PCM therefore stays in a thread-local queue instead of
being copied through a multiprocessing queue, and a busy main ROS executor
cannot delay playback. Cold playback uses the G1/R1-proven 300 ms prefill and
300 ms `PlayStream` blocks with a 240 ms maximum queued lead. A genuine
underflow uses a separate 500 ms recovery prefill, so lowering startup latency
does not make a delayed TTS segment resume one block at a time. Internal EOF
markers produced by split TTS text preserve the current playback timeline. Its
`info` response reports first-input-to-play latency, recovery wait, input gaps,
queue drops, partial flushes, underflows, `PlayStream` failures, and RPC latency.

Camera capture and ROS publication run in two spawned processes, following the
same isolation principle as Q5's capture/bridge split. A fixed double-buffer in
shared memory connects them, so only the newest JPEG is retained and the
roughly 300 KB frames are not pickled through a multiprocessing queue. The ROS
publisher assigns `CompressedImage.data` through a buffer-compatible byte array,
avoiding slow per-byte conversion. A busy FastDDS publisher therefore cannot
hold the videohub client's Python GIL, while a slow publisher drops stale frame
sequences instead of increasing end-to-end latency. Camera
`info` reports separate capture/publish rates, videohub RPC time, message-build
and publish-call time, average frame size, dropped stale frames, and both child
process states/PIDs. Like G1
`camera_rgb`, it is an always-on state source: canvas lifecycle `stop` detaches
intelligent control but does not stop publication. The child processes are
physically closed only when the driver bundle shuts down; repeated canvas starts
reuse the live stream without resetting frame counters.
The component vendors the three-field `audio_msgs/AudioChunk` interface used by
the perception audio bus and builds it in a dedicated Docker stage. The runtime
image receives only the generated `/as2w_ws/install` overlay and validates an
`AudioChunk` import during the build; it does not depend on an undocumented
`/ros_ws` artifact in the shared ROS base image.

No-hardware checks are available with `python3 test_driver.py`; they cover
action lifecycle, schemas, model resources, RPC correlation, and full-size
low-state arrays.
