# S10 Pro driver

This is the final S10 Pro bundle. It combines the upstream isolated ROS reader
and skeleton mapping with the validated native TCP control cards, ACP
completion callbacks, four camera lanes, split raw/map LiDAR streams, lights,
charging and navigation.

| Card | Input | Output |
| --- | --- | --- |
| Battery | `/BATTERY_DATA` (`drdds/msg/BatteryData`) | `soc`, `soh`, `current`, `voltage`, `cell_vol`, `temperature`, `cycle` as JSON |
| Joints | `/JOINTS_DATA_10HZ` (`drdds/msg/JointsData`) and `/IMU` | `joints_info` JSON with joint names/angles/speeds/torques/temperatures; `joints` skeleton with 16 joints and IMU orientation |
| Cameras | AOS RTSP streams on `10.21.33.103:8554` | Four JPEG streams on the existing `camera_front`, `camera_rear`, `camera_left`, and `camera_right` topics |
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
On each mic start, the driver selects the ES8323 differential mic PGA inputs
and enables `Main Mic`. It reads the codec's low-level noise-gate state and turns
the gate off only when it is enabled; an already-disabled gate is left alone.
The AOS factory ALSA state selected Line 1 and enabled a gate that reduced the
published signal to near-zero; `mic info` includes latest-frame PCM peak, RMS,
and nonzero-sample ratio to distinguish live audio from a silent input.
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

Motion and lighting use the vendor `basic_server` JSON framing on port `30003`.
The deployed S10 service currently uses plain TCP (`S10_BASIC_SERVER_TRANSPORT=tcp`)
because the execution startup gate below ensures TLS is disabled before use.
The TLS client options remain available for standalone diagnostics; they do not
override the execution gate. In TLS mode the client verifies the `server.local`
endpoint using the S10 certificate SHA-256 pin in `config.yaml`; deployments can
override the name/pin or provide a CA with `S10_BASIC_SERVER_TLS_CA`. The robot
server requests a client certificate, so set `S10_BASIC_SERVER_TLS_CLIENT_CERT`
and `S10_BASIC_SERVER_TLS_CLIENT_KEY` to the vendor-issued identity files.
Motion uses native S10 commands and reports long-running completion through
`/api/acp/complete`; locomotion movement accepts `yaw_rate_deg_s` in degrees per
second and a bounded duration. Head and tail lights are independent cards.
When Agent Core starts an execution card (`loco`, `loco_servo`, navigation,
charging, or a light), the driver checks AOS `robot_server` TLS state first by
SSH and runs the vendor's guarded `Network.toml` procedure. If SCE confirms TLS
is already disabled, startup proceeds without restarting the service. If the
procedure cannot confirm plaintext or reports that the robot must be rebooted,
the card start fails with the reason shown in the frontend; no execution command
is sent. After a successful check, this driver uses TCP for basic_server.
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
ACP assigns `base` to `loco`, `loco_servo`, navigation, and charging, and `map`
to navigation and map management; lights and speaker use independent `head_light`,
`tail_light`, and `mouth` resources. Long-running locomotion and navigation calls
return unique `action_id` values and post one terminal completion. A zero-duration
loco command is a synchronous stop and creates no ACP pending action. The continuous
servo has no ACP completion because it is a persistent stream: navigation pauses it
before taking the base, and the servo refuses to start while a navigation task owns it.

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
isolated child processes. RTSP pulls use independent threads; lidar conversion
and Core-domain publication use bounded queues so media work does not block
robot control/state callbacks.
The camera cards use the AOS RTSP endpoints supplied by the vendor: front wide
`/video4`, rear wide `/video0`, left wide `/video2`, and right wide `/video6`.
The validated vendor streams are H.264 at 1024x576 and 30 fps. The driver
pulls each stream over RTSP/TCP with FFmpeg, caps output at 15 fps,
scales frames to at most 640 pixels wide, and extracts MJPEG frames,
and publishes them to the existing Core-domain JPEG topics. Camera input is
RTSP-only: the robot ROS graph is scanned for point clouds, never image topics.
The driver also reports each camera's body-frame mount position in
`camera_info[].vendor.mounting.position_m`, based on the vendor Hardware
Development Manual v1.0.2 sections 1.6 and 1.9. These values are converted
from millimetres to metres. The manual provides no camera orientation, and the
RTSP stream provides no intrinsics, so orientation and optical calibration
remain unknown.
A one-frame low-latency pipeline reconnects after stream interruption. The
previous ROS image-topic receive path has been removed because the vendor camera service
does not publish camera images over DDS. The final image includes the `ffmpeg`
runtime for RTSP demuxing.

## Image and deployment

Build with the repository's normal driver build workflow. `driver.yaml` declares
`../../common` as a build-context extra; the Dockerfile expects the staged
`common/` directory. It pins the ROS Humble base image by digest, builds the six
required vendor message definitions plus shared `audio_msgs/AudioChunk` in a
build stage, and copies only their
runtime interfaces into the final image. See `vendor/SOURCE.md` for provenance.
The final image is larger than the base image because this driver needs the
Fast DDS RMW package, ALSA command-line tools (`alsa-utils`), PyYAML,
`openssh-client`, and `sshpass`. FFmpeg is also required for RTSP camera input.
The SSH packages support the current one-click NOS `drmap` password path; they
can be removed when deployment supports a managed SSH key/secret. The image
includes the current AOS/NOS ED25519 host key pin in `resource/ssh_known_hosts`;
SSH uses strict checking and accepts an alternate pinned file via
`S10_SSH_KNOWN_HOSTS`. Apt
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
the deployment environment after changing the robot credential. SSH verifies
the pinned AOS/NOS host key before sending this credential; host-key changes
fail closed. The vendor's body-mapping guide documents a recommended maximum
area of `50m x 50m`, a static environment, a loop-closure route, unobstructed
LiDAR, online LiDAR/IMU, and a stationary standing robot in normal mode, agile
mode, basic gait before mapping starts. `controlled_spatial` exposes `map_disk`,
`map_list`, and read-only `mapping_status`, plus confirmed `map_activate`,
`map_apply`, `mapping_start`, and `mapping_stop`. The guide documents
`drmap map activate MAP_NAME` after mapping and `drmap map apply MAP_NAME`
followed by a localization restart when switching an existing map; the card
preserves those two workflows separately. Start accepts an optional map name,
`no_activate`, and
`no_rviz`, matching the documented `drmap mapping start` options. A map started
with `no_activate` can later be activated through the card's confirmed
`map_activate` action.
Stopping mapping is documented to restart localization automatically; status
uses `drmap mapping status` plus both systemd services and
`/tmp/mapping_path.env` for operator verification. The vendor CLI has no
semantic tag/POI command in the target firmware. The driver therefore stores
G1/Tianyi-compatible named tags in the persistent
`/opt/phanthy-motus/data/s10_pro/tags.json` volume, using the current localized
pose and scoping tags by selected map. Named navigation points use the
documented map-coordinate route contract. The SSH
management host is set with `S10_DRMAP_SSH_HOST` and
defaults to `10.21.33.106`.

The single `controlled_spatial` card uses the currently loaded map and is the
full-stack mapping/localization/navigation card: mapping lifecycle, map
listing/activation, tags, native pose initialization, goal/route execution,
status, cancellation, and perception status. The separate `navigation` card is
not registered, matching the Tianyi/G1 contract. The S10 software development
guide documents the TCP `Item[]` route contract and navigation status/location
fields; the vendor [body-mapping guide](https://support.deeprobotics.cn/docs/?lang=zh-CN#/259676041)
documents `drmap mapping start/stop`, map listing and disk checks, service
status, start options, and result files. `controlled_spatial` combines those
documented operations with driver-managed tags because the vendor CLI does not
expose a tag API.

## Validation

Run from repository root:

```sh
python3 -m pytest -q tests/test_s10_pro_control.py tests/test_s10_pro_audio.py
```

The latest local run passed 99 tests. Tests use synthetic messages, fake native
TCP clients, and fake Core-domain subscribers; they do not send robot commands.
They cover interface auto-detection, AOS/NOS separation, control confirmation,
valid/invalid continuous-control messages, tested `loco` speed-limit parity,
start-time confirmation, sanitized remote output, watchdog stopping, and
subscriber failure stopping, PCM chunk contracts, AOS TLS-gate success/failure,
and frontend-visible reboot errors. The service-fragment isolation check also
passes. These checks do not send robot commands or execute the TLS script on the
robot. A new Bot build and robot-level validation are still required before
claiming deployment acceptance. ALSA capture/playback still needs validation on
the target host.
