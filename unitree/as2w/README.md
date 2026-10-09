# Unitree As2W driver

This bundle targets the Unitree **As2W** wheel-legged robot and its SDK service documented in the Unitree
SDK guide. It vendors Unitree's official `unitree_sdk2_python` master at
`65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5` and uses its dedicated
`unitree_sdk2py.as2.sport.SportClient`: `Move`, `StopMove`, `StandUp`, `StandDown`,
`BalanceStand`, `RecoveryStand`, `Damp`, `Euler`, `BodyHeight`, `BodyPosition`,
`SwitchGait`, `SpeedLevel`, `SwitchJoystick`, `SetAutoRecovery`, `GetState`,
`FrontFlip`, and `BackFlip`.

`Damp` and `Euler` are internal SDK operations and are not exposed as MCP
actions. `stand_down` only invokes the AS2 `StandDown` posture operation; it
does not automatically call `Damp`. A successful action is reported through
ACP with the observed final state.

Run `python3 main.py <robot-interface>` on the robot network. Set
`NETWORK_INTERFACE` to the actual host interface name when it is known; when it
is empty, the deployment tries available wired adapters (`eth*`, `en*`, and
`usb*`) instead of assuming the host calls the robot port `eth0`. If an
explicit interface is absent or CycloneDDS cannot bind to any candidate, the
entry point starts MCP in degraded mode without DDS publishers. It never falls
back to Wi-Fi, because that would register a driver that cannot talk to the robot. The
bundle exposes MCP on port `15709`, publishes JSON state streams under the
resolved ROS namespace, and
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

`duration=-1` starts a 10 Hz velocity command loop; negative `vx` means
backward motion and is passed through unchanged. `stop_move`, shutdown, and
any plugin stop path terminate that loop and issue `StopMove`. Velocity and
attitude inputs are clamped before reaching the robot. Special actions should only
be invoked with a clear area and appropriate operator approval.

The checked-in `resource/as2w.urdf` kinematic model is based on Unitree's
official `unitree_ros/robots/as2w_description`; it retains inertial and joint
limits but omits the vendor STL visual/collision meshes. The driver only needs
the kinematic chain for the `joints` skeleton card, avoiding large binary
assets in the repository. As2W publishes 12 active leg joints in `rt/lowstate`.
The skeleton names use the canonical `*_joint` suffix and match the URDF joint
names exactly. Its fixed-size motor array also contains four reserved zero
slots. The driver publishes only the 12 active motors as raw joint state. The
skeleton stream additionally publishes the four wheel-foot joints as
`virtual: true` with zero position, because they are kinematic URDF joints
rather than low-state motors; this lets the renderer traverse and display the
calf-to-wheel links. The model retains the fixed JT128 sensor mount.

`controlled_spatial` is a thin adapter for Unitree's documented `slam_operate`
service: mapping, relocalization, and point-goal navigation. The latest AS2
SDK does not package a model-specific SLAM client, so the driver implements the
documented common RPC contract directly in an isolated CycloneDDS process. It
requires the vendor `unitree_slam` service to be installed and already running
on the robot or extension host; the driver does not start that service.

The `mic` card receives the robot-body microphone's Unitree audio multicast
stream (`239.168.123.161:5555`) and republishes it to
`/<namespace>/mic/audio` as `audio_msgs/AudioChunk` (`audio/pcm-16k`). It does
not read an extension-board ALSA device. This is the same raw PCM path used by
the upstream Unitree SDK example `example/a2/audio/a2_audio_client_example.cpp`;
that example also subscribes to `rt/audio_msg` for ASR text. The AS2
`AudioClient` exposes playback, TTS, volume, and LED APIs, but not a raw capture
RPC, so the multicast receiver is the appropriate robot-body input path. The `speaker`
card has no default input topic: `speaker start` must provide the connected
`input_topic`; callers may use any `AudioChunk` stream. It streams bounded
PCM blocks through the AS2 `voice` service and exposes volume get/set actions.
Audio service availability depends on the AS2 firmware configuration.
The multicast membership is bound to the same selected robot interface as
Unitree DDS; it does not use the host default route. The speaker only
subscribes after the canvas supplies an explicit `input_topic`; it never
subscribes to a hard-coded TTS topic at bundle startup.

The `camera_rgb` card polls the verified AS2 `videohub.GetImageSample()` service
and publishes JPEG `sensor_msgs/CompressedImage` frames to
`/<namespace>/camera/rgb`. The current AS2 SDK and machine expose no depth-camera
service or ROS2 depth topic, so `camera_depth` is intentionally not registered
until a real depth source is identified.

`special_motion` exposes the AS2 SportClient's `FrontFlip`, `BackFlip`,
`HandStand`, and `BipedStand` actions. It is intentionally separate from the
continuous `loco` control card.

The RPC proxy runs sport, speaker-audio, LED-audio, and video clients in separate
workers. State DDS callbacks only replace a latest-value cache; a 60 Hz publisher
worker serializes and publishes the newest joint and locomotion samples instead
of draining stale samples. The microphone's multicast receiver publishes
compliant 16 kHz mono PCM frames.

The lidar conversion worker is isolated from the DDS callback and uses a bounded
latest-frame queue. It intentionally renders at most 2,000 points per frame so
large Livox packets do not consume the CPU budget needed by joints and
locomotion state. RGB camera RPC polling and JPEG publication also use separate
workers with a one-frame latest-value queue; a slow or unavailable snapshot does
not build a stale frame backlog. The camera RPC remains firmware-dependent: a
videohub timeout means the firmware did not return a frame, not that the ROS
publisher is buffering old frames.

In the checked-in AS2W configuration, lidar and RGB camera each run in a
separate OS process. This keeps point-cloud conversion and a blocked videohub
RPC out of the main MCP/state executor. Lidar probes for an already-installed
CuPy CUDA runtime and logs either `backend=cuda device=0` or
`backend=cpu fallback=...`; no CUDA package is downloaded implicitly. CPU
rendering remains bounded to 2,000 latest points, while `max_render_points`
(12,000 in the checked-in config) is used only by the CUDA path. The host must
expose Jetson GPU devices and a compatible CUDA Python runtime inside the
container. The inspected target container exposed `/dev/nvhost*`, but no
verified CuPy runtime, so CPU fallback is currently expected until the image
runtime is rebuilt with the required CUDA access.

No-hardware checks are available with `python3 test_driver.py`; they cover
action lifecycle, schemas, model resources, and full-size low-state arrays.

Change scope and validation notes:

- The audio, video, LED, locomotion, and state changes are intentional parts of
  the AS2W hardware card bundle. The Dockerfile does not add an APT or pip
  package for these cards. It sources `/ros_ws/install/setup.bash` because the
  runtime imports the shared `audio_msgs` message package; the image and
  runtime dependency are otherwise unchanged.
- The `driver.yaml` changes only advertise the cards that are registered by
  this bundle; metadata does not add image contents.
- Run `python3 -m unittest unitree/as2w/test_driver.py` for the no-hardware
  contract suite, `python3 -m compileall -q unitree/as2w` for syntax checks,
  and `git diff --check` before submitting. Hardware-dependent audio multicast,
  AS2 voice, videohub, and sport behavior still require validation against the
  target robot firmware.
- The RGB camera process has an explicit graceful-stop protocol: the parent
  signals the worker, the worker shuts down its ROS executor and closes the
  camera node and nested RPC workers, and only then does the parent use a
  bounded terminate fallback. The LED `start` action also restarts its
  keepalive thread after a prior `stop` when a non-black color is selected.
- On the target Orin host, the robot adapter was observed as `eno1` with
  `192.168.123.100/24` while Wi-Fi carried the default route. A read-only
  listener bound explicitly to `eno1` received 31 microphone multicast packets
  of 5120 bytes during a five-second sample, confirming that the robot is
  transmitting audio and that interface selection is material. Speaker output
  still requires an `AudioChunk` producer on its input topic and AS2 voice
  service availability.

Loco examples:

`move.vx` is signed: positive is forward and negative is backward.
`move.vyaw` is expressed in degrees per second by MCP (the driver converts it
to radians per second at the Unitree SDK boundary).
After `StopMove`, some AS2 firmware keeps `GetState().fsm_name` at
`AI_FREE_WALK` even though the velocity command has stopped. The driver
reports a completed ACP result with `state_stale=true` after the accepted
`BalanceStand` normalization instead of falsely leaving the action running;
the observed state label is still included in the result. It also keeps that
accepted standing normalization as an internal state anchor, so a later
`body_height`, `stand_up`, or `balance_stand` is not blocked by a stale
firmware label. A new `move` or posture transition replaces the anchor.
`body_height` takes an explicit absolute target in meters, for example
`{"action":"body_height","height":0.35}`. The public range is `0.17` to
`0.50` m; around `0.48` m includes the high-stand leg geometry and wheel
radius in the bundled AS2W URDF. AS2's `BodyHeight` SDK parameter is the target height itself,
unlike the relative offset convention used by some Go2 SDKs, so the driver
sends the requested height directly. The response includes both `height_m`
and `sdk_height_m`.

The `loco_state.mode`/`mode_name` pair comes from the numeric
`SportModeState.mode` field (`0=IDLE_DEFAULT_STAND`, `1=BALANCE_STAND`,
`2=POSE`, `3=LOCOMOTION`, `5=LIE_DOWN`, `6=JOINT_LOCK`, `7=DAMPING`,
`8=RECOVERY_STAND`). It is deliberately not substituted with the separate
SportClient `GetState().fsm_name`. When the firmware publishes
`SportModeState.body_height == 0`, the driver reports
`body_height_valid=false` and `body_height_status=unavailable`; zero is not
presented as a measured zero-meter body height.

```json
{"action":"move","vx":0.3,"vy":0,"vyaw":0,"duration":2}
{"action":"move","vx":0.2,"vy":0,"vyaw":0,"duration":-1}
{"action":"stop_move"}
{"action":"speed_level","speed_preset":"slow"}
{"action":"auto_recovery","flag":true}
{"action":"switch_joystick","flag":false}
{"action":"left_side_gait","flag":true}
```

`flag` is not a generic parameter: it enables or disables automatic fall
recovery, gives or removes joystick control, or enters/exits a side gait.
`special_motion` requires `confirm: true`; flips are one-shot actions, while
`handstand` and `biped_stand` use `enter: true` to enter and `enter: false` to
exit. These motions are posture- and firmware-dependent and require a clear
safety area.

Speaker input is raw signed 16-bit little-endian mono PCM at 16 kHz. It accepts
both `audio/pcm-16k` and the existing Agent Core alias
`pcm_16k_16bit_mono`. The
speaker strips the TTS end-of-utterance marker (`01 00 ff ff 01 00 ff ff`)
instead of sending it to the robot, flushes each utterance at that boundary,
and forwards exactly 300 ms blocks required by AS2 voice startup, splitting
queued data at the SDK boundary. The
blocks are paced at real time with no intentional lead, preventing the AS2
voice buffer from being overrun and producing clipped/distorted initial audio.

## Review and validation notes

The change scope includes the AS2W audio cards, RGB camera and LED cards,
high-rate state and lidar publication, isolated sensor/RPC workers, robot
interface selection, locomotion ACP completion reporting, and `body_height`
validity metadata. It is not limited to catalog metadata.

The no-hardware contract suite is run with
`python3 -m unittest unitree/as2w/test_driver.py`; it covers card schemas,
worker lifecycle behavior, audio/camera payload helpers, ACP contracts, and
safe interface selection. On the target Orin host, a read-only listener bound
to the robot adapter received the robot microphone multicast, while the RGB
camera RPC timed out on the tested firmware. Speaker playback remains
dependent on an `AudioChunk` producer and the firmware voice service; these
are runtime limitations, not successful-stream assumptions.

Lidar rendering uses CuPy/CUDA only when an existing runtime provides both a
visible CUDA device and a usable CuPy installation. Otherwise it uses the
dependency-free CPU path; this component does not add a CUDA or CuPy image
dependency.

Automatic interface discovery only considers active wired adapters and prefers
the configured AS2 subnet (`192.168.123.0/24`). If none is available, DDS
initialization is refused and MCP starts in degraded mode. An empty interface
is never passed to CycloneDDS, preventing accidental Wi-Fi selection.
