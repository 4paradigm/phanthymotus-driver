# S10 Pro driver

This is the final S10 Pro bundle. It combines the upstream isolated ROS reader
and skeleton mapping with the validated native TCP control cards, ACP
completion callbacks, four camera lanes, split raw/map LiDAR streams, lights,
charging and navigation.

| Card | Input | Output |
| --- | --- | --- |
| Battery | `/BATTERY_DATA` (`drdds/msg/BatteryData`) | `soc`, `soh`, `current`, `voltage`, `cell_vol`, `temperature`, `cycle` as JSON |
| Joints | `/JOINTS_DATA_10HZ` (`drdds/msg/JointsData`) and `/IMU` | `joints_info` JSON with joint names/angles/speeds/torques/temperatures; `joints` skeleton with 16 joints and IMU orientation |
| LiDAR | 22 historical/live PointCloud2 candidates; `/ACCUMULATED_POINTS_MAP` remains the map source | Split `lidar_cloud` and `lidar_map` `sensor/pointcloud` streams with bounded latest-frame workers; only one fresh cloud source is active at a time |
| IMU | `/IMU` (`sensor_msgs/msg/Imu`) | source `header.stamp`, quaternion (w,x,y,z), angular velocity, linear acceleration, Euler angles (radians), temperature |
| `loco_servo` | `control/velocity` (`motus.control/1`, twist) | bounded continuous velocity commands to the native S10 TCP controller; dry-run by default |
| Odom | `/MOTION_INFO` (`drdds/msg/MotionInfo`) | `state/odom` with measured body `vx`, `vy`, and `wz`; no fabricated pose |
| `mic` | AOS ALSA capture device, local or SSH-forwarded | `/{namespace}/mic/audio`, `audio/pcm-16k` (`audio_msgs/msg/AudioChunk`), for the Perception ASR card |
| `speaker` | connected `audio/pcm-16k` input topic | plays TTS PCM on the AOS ALSA playback device, locally or over SSH |

The microphone captures mono S16LE at 16 kHz in 32 ms frames and publishes
independently of ASR. It starts with the driver bundle and can also be
started/stopped from its card. Connect its `/{namespace}/mic/audio` output to
any compatible audio consumer. The speaker subscribes to the topic supplied by
the canvas and does not require TTS; it accepts both `audio/pcm-16k` and
`pcm_16k_16bit_mono` AudioChunk markers, including the Agent Core
`/remote_control/mic` stream. It ignores the audio end-of-stream marker.
`S10_AUDIO_MODE=auto` uses local ALSA when the configured AOS address is
bound on this host (onboard deployment), otherwise it runs `arecord`/`aplay` on
AOS over SSH (external Jetson deployment). The default AOS endpoint is
`10.21.33.103`, with `user` and the configured S10 SSH password; override with
`S10_AUDIO_SSH_HOST`, `S10_AUDIO_SSH_USER`, and `S10_AUDIO_SSH_PASSWORD`.
Capture and playback both default to the verified AOS onboard codec
`plughw:CARD=rockchipes8388,DEV=0` (ALSA card 0); the USB `Speaker` device is
playback-only on this robot. These device settings are used in both local and
SSH modes; override them independently when the AOS card names differ.
The mic bridge keeps at most four 32 ms frames (128 ms) and replaces the oldest
queued frame if its publisher falls behind. Raw `arecord`/SSH output has no
hardware capture timestamp, so mic messages use a zero header stamp and
consumers must treat capture time as untrusted. `mic info` reports bridge
frames dropped during the current capture session.
`S10_AUDIO_CAPTURE_DEVICE` and `S10_AUDIO_PLAYBACK_DEVICE` override the
respective ALSA devices. Set `S10_AUDIO_MODE=local` or `ssh` to force a mode.
Deployment maps `/dev/snd` for local operation; external operation transports
PCM over SSH and does not use the Jetson's unrelated local sound card.

`model` returns a mesh-free kinematic URDF used by the joint renderer. It has
the 16 actuated joints and limits, but deliberately no STL dependency. A fixed
arrow marks mechanical +X/front. The source joint order and angle mapping live
in `s10_skeleton.py`.
The skeleton represents posture and does not track global translation.

Battery values retain the vendor message units: voltage is 10 mV and current
is signed 10 mA; `soc` is percent. Missing SOH/cell voltages and IMU temperature
are null/empty, not invented measurements. Missing or invalid IMU source time
is `header.stamp: null`. Point coordinates remain in the input LiDAR frame;
there is no mounting transform, temporal accumulation or object recognition.
Joint latency and LiDAR coordinate refinements remain separate local work.

Motion and lighting use the vendor `basic_server` JSON framing over TLS/TCP on
port `30003`. The client verifies the `server.local` endpoint using the
S10 certificate SHA-256 pin in `config.yaml`; deployments can override the
name/pin or provide a CA with `S10_BASIC_SERVER_TLS_CA`. There is no plaintext
fallback. Motion uses native S10 commands and reports long-running completion through
`/api/acp/complete`; locomotion movement accepts `yaw_rate_deg_s` in degrees per
second and a bounded duration. Head and tail lights are independent cards.
`loco_servo` is the streaming counterpart for visual navigation policies. It
subscribes on Core domain 42 through a loopback-only bridge process, requires
`S10_MOTION_ENABLED=1` at deployment plus `confirm=true` before switching out
of dry-run (also accepted atomically on `start`), and stops on its watchdog.
The supplied service fragment defaults to read-only; explicitly set
`S10_MOTION_ENABLED=1` for supervised physical operation. The speed limits
match the real-robot-tested `loco` card (`vx=1.67 m/s`, `vy=0.4 m/s`,
`wz=1.0 rad/s`). Per-step delta limits are additional software shaping, not
claimed as measured hardware limits.
The `navigation.route` action sends a sequence of up to 64 vendor `Item` targets
in one native task request; each point has map coordinates, yaw, and point type,
while the route shares gait, speed, direction, obstacle, and navigation mode.

## Runtime

The shared MCP server binds all host interfaces on port 15745. `ROS_NAMESPACE` defaults to
`s10_pro`; output topics follow `/<namespace>/<stream>` (for example
`/s10_pro/imu`).
Four sensor calls use empty arguments. Start/stop/status/config lifecycle calls
remain supported by the shared MCP bundle; stopping a card releases only its
own subscription lease. Data is published only while the relevant input is
fresh. Input freshness uses local monotonic receipt time, not ROS clock time.

Fast DDS profile selection is process-global. The robot-domain participant is
intentionally reachable on the validated S10-facing subnet (and loopback) so it
can consume vendor ROS topics. The Core-domain participant is domain 42 and is
loopback-profiled; media/state publishers and the `loco_servo` subscriber are
isolated child processes. Media conversion and publication
use independent bounded lanes so a camera or point-cloud frame cannot block
control/state callbacks.

## Image and deployment

Build with the repository's normal driver build workflow. `driver.yaml` declares
`../../common` as a build-context extra; the Dockerfile expects the staged
`common/` directory. It pins the ROS Humble base image by digest, builds the six
required vendor message definitions plus shared `audio_msgs/AudioChunk` in a
build stage, and copies only their
runtime interfaces into the final image. See `vendor/SOURCE.md` for provenance.
The final image is larger than the base image because this driver needs the
Fast DDS RMW package, ALSA command-line tools (`alsa-utils`), PyYAML, Pillow,
`openssh-client`, and `sshpass`.
The SSH packages support the current one-click NOS `drmap` password path; they
can be removed when deployment supports a managed SSH key/secret. Apt
recommends and pip caches are disabled/removed to keep this growth limited to
the component.

`deploy/service.yml` is a service fragment for the normal platform deployment.
It maps `/dev/snd` so the cards can access ALSA devices on the deployment host
and preconfigures the interface auto-detection,
robot subnet, AOS control endpoint, and NOS map-management endpoint for a
standard S10 deployment. Override these values only for a different robot
network or changed robot credentials.
`main.py` generates the robot and Core Fast DDS profiles at startup.

This bundle uses the repository's shared registration helper. That helper does
not verify the registration endpoint's TLS certificate, and its MCP listener is
not loopback-only. The service uses host networking, so deploy only on a
firewalled, isolated host/network where port 15745 is not reachable by
untrusted clients. Per-driver CA pinning and loopback binding require a shared
runtime change and are not provided by this component.

The deployment values for `NETWORK_INTERFACE`, `ROBOT_NETWORK_CIDR`, and the
basic-server host must describe the isolated S10 network on the target host.

The same image supports AOS and an external Jetson without a deployment-mode
switch. `NETWORK_INTERFACE=auto` asks the kernel for the source address used to
reach the configured S10 basic-server host, then requires that address to map
to exactly one local interface. Thus AOS commonly resolves to `eth0`, while a
Jetson connected to the robot-side gigabit LAN commonly resolves to `eno1`.
The Jetson interface must have an unused static address in `10.21.33.0/24` and
the server remains `10.21.33.103`; do not use the NOS `10.21.31.0/24`
service/PPS network for the robot-facing port. Ambiguous or missing routes fail
startup instead of selecting an arbitrary interface.

`10.21.33.103` is the AOS TCP control/sensing endpoint, while `10.21.33.106`
is the NOS map-management endpoint. `controlled_spatial` auto-selects local
`drmap` only when the NOS management IP is assigned to this host and both
`drmap` and its map directory are present. Otherwise the fixed allowlisted
operations run over SSH to the NOS host as
`user` (overridable with `S10_DRMAP_SSH_USER`). The service fragment defaults
`S10_DRMAP_PASSWORD` to the robot's reported default password; override it via
the deployment environment after changing the robot credential. This default
is weak and is stored in the deployment configuration, not the image. SSH uses
`accept-new` host-key handling, so the first connection pins the presented host
key and later changes fail closed. The tutorial demonstrates `/var/opt/robot/data/maps`,
`drmap mapping start`, `drmap mapping stop`, and `drmap map list`; responses
from `drmap --format json map list` are parsed when valid. The SSH management
host is set with `S10_DRMAP_SSH_HOST` and defaults to `10.21.33.106`. Map
activation, deletion, and mapping status remain unavailable until their exact
CLI schema and error semantics are verified on target firmware.

The native navigation card uses the currently loaded map and supports pose
initialization, goal/route execution, status, cancellation, and perception
status. The S10 software development guide documents the TCP `Item[]` route
contract and navigation status/location fields. It does not define map CRUD,
route persistence, or a mapping API. The separate [mapping tutorial](https://support.deeprobotics.cn/docs/?lang=zh-CN#/259675993)
documents only the interactive `drmap mapping start` / `drmap mapping stop`
workflow, says it must be run from the robot's map-management directory, and
does not specify the directory, executable deployment path, status/output
schema, map selection, or error contract. `controlled_spatial` therefore
implements only the demonstrated map listing and mapping start/stop operations;
it does not claim support for map CRUD or persistent saved routes. The existing
multi-point native route remains the supported navigation route contract.

## Validation

Run from repository root:

```sh
python3 -m pytest -q tests/test_s10_pro_control.py tests/test_s10_pro_audio.py
```

The latest local run passed 81 tests. Tests use synthetic messages, fake native
TCP clients, and fake Core-domain subscribers; they do not send robot commands.
They cover interface auto-detection, AOS/NOS separation, control confirmation,
valid/invalid continuous-control messages, tested `loco` speed-limit parity,
start-time confirmation, sanitized remote output, watchdog stopping, and
subscriber failure stopping. A new Bot build and robot-level validation are still required
before claiming this version accepted. Audio tests validate contracts and frame
filtering only; ALSA capture/playback still needs validation on the target host.
