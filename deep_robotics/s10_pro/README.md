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
| `camera_front__depth` | `/LIDAR/POINTS` and automatically loaded robot-specific optical extrinsics | One locally hole-filled front-view depth stream on `/{namespace}/camera_front/depth`, `image/depth-zlib`, 640x480 uint16 millimetres |
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
On each mic start, the driver selects the ES8323 differential mic PGA inputs,
enables `Main Mic`, and sets both capture-channel gains to +12 dB (ALSA step 4).
It captures both hardware channels and publishes the stronger right channel as
mono PCM; an AOS speech check measured roughly 3x the RMS on the right channel.
The driver reads the codec's low-level noise-gate state and turns
the gate off only when it is enabled; an already-disabled gate is left alone.
The AOS factory ALSA state selected Line 1 and enabled a gate that reduced the
published signal to near-zero; `mic info` includes latest-frame PCM peak, RMS,
and nonzero-sample ratio plus a rolling 3.2-second peak/RMS/nonzero ratio to
distinguish live audio from a silent input after a short speech test.
The mic bridge keeps at most four 32 ms frames (128 ms) and replaces the oldest
queued frame if its publisher falls behind. Raw `arecord`/SSH output has no
hardware capture timestamp, so mic messages use a pipe-arrival estimate in
the header and consumers must treat capture time as untrusted. `mic info` reports bridge
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
subscribes on Core domain 42 through a loopback-only bridge process. A frontend
`start` action arms real velocity input by default; this action is the operator
confirmation, so no separate `dry_run=false` or `confirm=true` arguments are
needed. Set `dry_run=true` explicitly for validation without native movement.
Live startup sends navigation mode (`Mode=1`) and navigation-flat gait
(`GaitParam=0x3002`) before subscribing to velocity input, matching `loco.move`.
If either send fails, startup returns an error without arming the stream.
Dry-run startup skips both commands. A successful send is not confirmation
that the physical mode transition has completed; the standing-state gate still applies.
Real motion still requires `S10_MOTION_ENABLED=1` in the deployment environment;
the supplied service fragment enables that deployment permission by default.
Set `S10_MOTION_ENABLED=0` to revoke it. The servo stops on its watchdog. The speed limits
match the real-robot-tested `loco` card (`vx=1.67 m/s`, `vy=0.4 m/s`,
`wz=1.0 rad/s`). Per-step delta limits are additional software shaping, not
claimed as measured hardware limits. Before every velocity command, it also
requires a fresh (at most 1 second old) standing `MotionState` (`1` or `17`)
from `/MOTION_INFO` or the BasicServer status report. Missing, stale, or
non-standing state causes a zero-velocity hold and the command is refused.
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
from millimetres to metres. The manual provides no camera orientation, so
orientation remains unknown.
Each camera card has one setting, `undistort` (default `true`), saved per lane
in `/opt/phanthy-motus/data/s10_pro/camera_undistort.json`. When it is on, the
lane's FFmpeg command gains a `remap` stage at the 1024x576 source resolution
and the published JPEG is already rectified; `camera_info` then declares
`pinhole`, the calibrated K and zero D, with the half-FOV derived from that K
(about 0.79 rad horizontal). When it is off the raw, barrel-distorted stream is
published and its geometry is declared `unknown`: no pinhole angle fits it (the
calibrated K's is about 10 degrees short at the edges). Changing the switch reconnects only that lane;
restart downstream vision cards so they re-read the declaration, and do not
chain a separate undistortion card behind a rectified camera.
The per-camera K/D are the chessboard calibrations in `undistortion.py`
(`CALIBRATIONS`, measured 2026-11-07/08, RMS 0.21-0.22 px). They are per model,
not per unit, and are not editable from the card; re-measure with
`tools/calibrate_chessboard.py` (a workstation tool needing numpy and OpenCV; it
is not shipped in the image) and replace the row. These replace the earlier rough front-camera wall estimate of
`0.423 rad`.
A one-frame low-latency pipeline reconnects after stream interruption. The
previous ROS image-topic receive path has been removed because the vendor camera service
does not publish camera images over DDS. The final image includes the `ffmpeg`
runtime for RTSP demuxing.

## Front-camera sparse depth

The Pro bundle includes one additional sensor card, `camera_front__depth`.
It projects the original LiDAR XYZ into the rectified front camera's optical
view. It uses the existing front-camera calibration without pulling or decoding
another RTSP stream. It fills supported short gaps and publishes only one
output on the existing depth topic; there is no separate raw or `depth_filled`
output. It publishes only after the card's `start` action; driver
bundle startup only advertises the card.

### First-use configuration

Keep the `camera_front` card's `undistort` setting enabled.
Start the card normally; calibration and transport fields
are not exposed in canvas settings. The inspected robot's identity-bound
calibration is included in the image and selected automatically. Other robots
need their own calibration, independently of canvas projects.

At each fresh start, the driver reads
`/opt/phanthy-motus/data/s10_pro/camera_front_extrinsics.json` (override with
`S10_FRONT_CAMERA_EXTRINSICS`). This is a new deployment interface, not an
existing factory file discovered on the robot. The driver only reads it.
When the default path is absent and no maintenance matrix is configured, it
reads the NOS `/etc/machine-id` SHA-256 over existing strict SSH and selects
exactly one matching record under `resource/calibration/` in the image. No
data-volume file is created. An explicit `S10_FRONT_CAMERA_EXTRINSICS` override
never falls back to a bundled record, even when its path is missing. Present
invalid deployment records also never fall back.
The version-1 JSON object requires:

| Field | Required meaning |
| --- | --- |
| `version` | Integer `1` |
| `camera_id` | `deeprobotics/s10_pro/camera_front` |
| `front_camera_stream` | Exact configured front RTSP URL; normally `rtsp://10.21.33.103:8554/video4` |
| `cloud_topic` | `/LIDAR/POINTS`; no automatic map/display fallback |
| `cloud_frame_id` | Calibrated source frame, checked against each message's `header.frame_id` |
| `camera_optical_frame_id` | Target optical frame named by the calibration; this does not assert a manufacturer TF exists |
| `translation_unit` | `m` |
| `camera_axes` | `x_right_y_down_z_forward` |
| `camera_optical_from_cloud` | Measured directed numeric 4x4 rigid transform |

No guessed default matrix is supplied: an identity or visual estimate is not a
calibration. Missing or unmatched records return `CALIBRATION_MISSING`;
an unavailable NOS identity returns `CALIBRATION_IDENTITY_UNAVAILABLE`. Malformed records,
wrong camera/stream/topic/units/axes, or invalid matrices return
`CALIBRATION_INVALID`. Start failures remain visible in `info`. A present invalid
record cannot silently fall back to a previous or maintenance configuration.
Stop/start reloads the file; repeated start of a healthy session is idempotent.

For the robot inspected on 2026-11-14, the operator has confirmed the missing
body/cloud and camera-axis relationships. Its derived mounting record is
[`resource/calibration/s10_pro_cd1_inspected_20261114.json`](resource/calibration/s10_pro_cd1_inspected_20261114.json).
It is scoped to that robot and preserves its manual/confirmation provenance.
The image packages this record with `robot_identity.kind=nos_machine_id_sha256`
and the inspected NOS identity digest. IP address and hardware model do not
select it. The digest identifies the NOS operating-system installation, not
an immutable robot serial number; replacing/reimaging NOS requires rechecking
the binding and calibration. It is not a model-wide fallback. Each fresh start
rechecks identity. This loader has been verified in a temporary in-memory
probe on the actual NOS, without writing the calibration onto the robot;
image/cloud reprojection remains unverified.

### Automatic point-cloud transport

Each start first uses the existing robot-domain ROS subscription. After three
seconds without valid clouds, the card automatically starts a read-only SSH
reader on NOS and delivers its samples directly to this card. It does not
republish `/LIDAR/POINTS` into either ROS domain. A healthy local ROS source
never starts SSH; after fallback the session keeps SSH as its sole input until
stop/start, avoiding duplicate input from a recovering DDS connection.

The reader reuses `controlled_spatial.ssh_host` / `ssh_user` and existing
`S10_DRMAP_SSH_HOST`, `S10_DRMAP_SSH_USER`, `S10_DRMAP_PASSWORD` deployment
settings. It verifies the pinned host key (`S10_SSH_KNOWN_HOSTS`) and uses
existing key authentication if no password is configured. No canvas settings
or new credentials are required. It runs the module from memory using the
already-installed NOS `/opt/ros/jazzy` runtime, in the configured robot domain.
Localhost-only discovery applies only to this temporary process; factory
services and DDS configuration are not changed. ROS file logging, rosout,
parameter-event publication and parameter/type-description services are
disabled. Stop closes the reader and its owned local SSH process group.

The protocol preserves original frame, source stamp, fields, byte order and
point bytes across ROS distributions. Each request transfers at most one
latest sample; subscriber depth is one. Source queue age plus request/response
time must be at most one second. Startup has a 15-second budget, subsequent
responses three seconds; disconnected/stalled readers retry after one second.
Delayed startup samples are discarded. `info.input.transport` reports `ros`
or `ssh`, with relay state/count under `info.input.relay`. Failures are explicit:
`CLOUD_RELAY_FAILED`, `CLOUD_RELAY_STALE`, or `CLOUD_RELAY_DATA_TIMEOUT`.
Missing/invalid extrinsics still fail before any relay starts.

Canvas settings contain only `min_depth_m`, `max_depth_m`, and `rate_hz`.
The existing maintenance `config` API and saved settings remain compatible:

| Setting | Meaning | Default |
| --- | --- | --- |
| `cloud_topic` | One raw `sensor_msgs/PointCloud2` topic whose XYZ values are in metres | `/LIDAR/POINTS` |
| `cloud_frame_id` | The exact calibrated `header.frame_id` of that source | Obtained from deployment calibration |
| `camera_optical_from_cloud` | Measured rigid 4x4 transform | Obtained from deployment calibration; no guessed default |
| `min_depth_m` / `max_depth_m` | Accepted positive optical-Z range, in metres | `0.1` / `50.0` |
| `rate_hz` | Maximum number of clouds submitted per second | `5.0`, configurable up to `15.0` |

The transform uses column vectors:
`[X_camera, Y_camera, Z_camera, 1] = camera_optical_from_cloud * [X_cloud, Y_cloud, Z_cloud, 1]`.
Camera optical axes are X right, Y down, Z forward; translation is in metres.
The rotation must be orthonormal with determinant +1, and the last row must be
`[0, 0, 0, 1]`. The source frame and transform must come from calibration of the
selected stream on this robot. The driver does not infer them from the display
cloud, map cloud, vendor mounting positions, or an assumed identity transform.
On the inspected CD1, `/LIDAR/POINTS` is already transformed and merged from
both RoboSense Airy lidars in `lidar_link`. Do not apply the individual lidar
mounting transforms a second time. The topic default alone does not establish
the source-to-camera transform.

The card's `config` action accepts the same fields either directly or inside a
`config` object; API callers can supply the transform as a nested numeric array
or a JSON string. Valid settings are saved atomically to
`/opt/phanthy-motus/data/s10_pro/camera_front_depth.json` (override with
`S10_FRONT_DEPTH_CONFIG`). Saving a configuration stops this depth card and
returns `restart_required: true`; start it again to use the new settings.
Invalid maintenance configuration leaves the previous configuration and running
session intact. A complete maintenance calibration is accepted only when the
separate deployment record is absent; a present valid deployment record owns
the source topic, frame and transform. Automatic values are not written into
maintenance settings. `info` reports the effective settings, counters, last
error/code and `camera_info`. Metadata records `extrinsics_source` and
`reprojection_verified: false`: loading a valid matrix is not a reprojection test.

### Output and interpretation

The output is `sensor_msgs/CompressedImage` with
`format="16UC1; compressedDepth zlib"`: zlib level-1 compression of a fixed
640x480, row-major, little-endian uint16 millimetre buffer. Zero means unknown.
At each observed pixel the closest positive camera Z wins. The single output
then fills horizontal/vertical gaps of at most eight pixels, bounded by original
measurements whose depths differ by no more than both 150mm and 6% of the nearer
depth. It preserves measured pixels, rejects conflicting estimates and never
uses filled pixels as new anchors. Large gaps and unsupported regions stay zero.
No learned filling, temporal accumulation or motion compensation is applied.
The result remains sparse and must not be treated as evidence that an
unobserved area is clear. Metadata declares `hole_filling: true`,
`depth_source: estimated_interpolation` and `interpolated_pixels_are_measured: false`.
`info.stats` keeps measured coverage, filled coverage, filled-pixel count and
conflict count; the original projection exists only inside the worker.

The existing depth renderer requires 640x480, while front RGB is 640x360.
The depth declaration supplies its own K using pixel-centre scaling:
`fx_depth = fx_rgb`, `fy_depth = fy_rgb * 4/3`, `cx_depth = cx_rgb`, and
`cy_depth = (cy_rgb + 0.5) * 4/3 - 0.5`. The corresponding pixel mapping is
`u_depth = u_rgb`, `v_depth = (v_rgb + 0.5) * 4/3 - 0.5`.
`camera_info.vendor` declares this mapping, optical-Z units, sparsity, invalid
values, the configured extrinsics, and `depth_aligned_to_rgb: false`.

The depth header copies the selected LiDAR message's stamp exactly and uses
the calibration's `camera_optical_frame_id` (the historical driver label
`s10_pro_camera_front_optical` for maintenance-only configurations). This preserves source time without
asserting that the source clock is a verified acquisition clock. The existing
RTSP JPEG timestamps are publication times; no exposure pairing is available.
Metadata therefore declares `timestamp_source="lidar_header_unverified"`,
`temporal_synchronization: false`, unknown RGB/depth capture skew and unknown
source scan deskew. This output alone is not synchronized RGB-D input for VO.

Only the explicitly configured raw topic is subscribed. There is no automatic
source switching, and the display LiDAR's coordinate changes are not reused.
Mismatched frames, zero/invalid/repeated/backward source stamps, malformed
clouds, and results older than one second since local receipt are rejected.
Restart the card after a source clock reset. Projection and compression run in
one bounded worker process, with one pending input and one pending result;
each cloud samples at most 200,000 points uniformly. Source data is limited to
64 MiB and two million points. Effective output rate can be below `rate_hz`.
Stopping or reconfiguring the card discards in-flight results and releases its
own subscription and worker. Changing front-camera undistortion invalidates
pending depth, and disabling it blocks further depth publication. The depth
card does not clear errors from the RGB capture pipeline.

An owned one-second watchdog reports input health in `info.input`: `waiting`,
`live`, `stale`, `no_publisher`, or `idle`, plus the observed frame, publisher
count and age of the last valid cloud. After three seconds without valid input,
`CLOUD_PUBLISHER_UNAVAILABLE` means the driver sees no publisher;
`CLOUD_DATA_TIMEOUT` means input is silent or publisher discovery is unavailable.
The watchdog is released on stop or reconfiguration. A valid returning cloud
recovers input health. A frame mismatch is `CLOUD_FRAME_MISMATCH` and cannot
replace the calibration's source frame.

On the inspected robot, the factory raw writers advertise only SHM/loopback
locators. Jetson therefore cannot discover them through the physical subnet,
even in domain 0 with compatible sensor QoS. Changing Jetson subscriber QoS or
silently choosing `/ALIGNED_POINTS` cannot fix that publishing boundary.
See [the read-only diagnosis](docs/front-depth-diagnosis-20261114.md).

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
