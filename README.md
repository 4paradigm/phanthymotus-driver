# Phanthy Motus Drivers

[中文文档](README_zh.md) | [Official Website](https://motus.phanthy.com)

Hardware drivers for the **[Phanthy Motus](https://github.com/4paradigm/phanthymotus)** embodied AI platform.

Each driver is a standalone [MCP](https://modelcontextprotocol.io) HTTP server that exposes hardware capabilities as tools. Drivers automatically register with the [Phanthy Motus Agent Core](https://github.com/4paradigm/phanthymotus) on startup.

## Available Drivers

| Driver | Hardware | Port | Description |
|--------|----------|------|-------------|
| `unitree/g1` | Unitree G1 Humanoid | 15701 | Locomotion, arm control, mic, speaker, LED, state monitoring |
| `unitree/go1` | Unitree Go1 (EDU) Quadruped | 15715 | State, locomotion, camera (RGB/depth/pointcloud), ext peripherals, URDF |
| `unitree/go2` | Unitree Go2 Quadruped | 15703 | Locomotion, obstacle avoidance, voice, video, navigation |
| `unitree/r1` | Unitree R1 (EDU) Humanoid | 15702 | Mic, TTS, LED, locomotion, stereo camera, state monitoring |
| `dji/M300` | DJI Matrice 300 RTK | 15702 | Flight control, telemetry, perception, HMS, aircraft info |
| `dji/mavic3e` | DJI Mavic 3E/3T | 15702 | Flight control, camera (wide/zoom/IR), gimbal, waypoint missions, perception, IR thermometry |
| `dji/mavic4e` | DJI Mavic 4E/4T | 15703 | Flight control, camera, gimbal, waypoint missions, telemetry, perception |
| `engineai/t800` | EngineAI T800 Development Edition | 15708 | ROS2/Native SDK, full state, dance/gesture sequences, virtual gamepad, locomotion and low-level joint control |
| `noetix/bumi` | Noetix Bumi-EDU Humanoid | 15704 | Mic, speaker, locomotion, RealSense camera, state monitoring |
| `x-humanoid/tianyi2.0` | Tianyi 2.0 Pro Humanoid | 15707 | 35DOF (wheeled chassis + dual arms + dexterous hands + head + navigation) |
| `deep_robotics/lynx_m20` | DEEPRobotics Lynx M20 | 15716 | Official ROS 2/Fast DDS interfaces and basic_server TCP/UDP native control, with Standard/Pro capability isolation |
| `pnpbotics/adam` | PNPbotics Adam Humanoid | 15702 | State, locomotion (gRPC), upper body control, dexterous hands, 3D model |

## Quick Start

### Prerequisites

- Docker (ARM64)
- A running [Phanthy Motus Agent Core](https://github.com/4paradigm/phanthymotus) instance

### Deploy via Web Dashboard (Recommended)

The easiest way to deploy a driver is through the Agent Core Web Dashboard. Navigate to **Deploy** in the top menu — you can browse reviewed and published driver versions, select a version, and deploy with one click. No manual build required.

### Build & Deploy Your Own Driver

If you need to build from source or develop a custom driver:

```bash
cp .env.example .env  # Fill in registry credentials

# Build a specific driver
./build.sh unitree/g1
./build.sh engineai/t800
```

When run without arguments, `build.sh` shows an interactive multi-select menu to choose which drivers to build. You can also pass driver paths directly for CI usage:

```bash
# Build multiple drivers
./build.sh unitree/g1 phanthy/remote_control
```

Once the driver container starts, it registers itself with Agent Core at `http://<agent-core>:15678/api/mcp`. You can then see the device and its tools in the Web Dashboard.

For a G1 development deployment, run the versioned Git deployment entry from
a clean, pushed branch:

```bash
./unitree/g1/deploy/deploy-from-git.sh g1-bj-wifi
```

The script does not copy the local working tree to the robot. It records the
local branch and commit, fetches that branch in
`~/hanzebei/phanthymotus-driver` on the target, requires the fetched tip to
match the exact commit, builds with the repository's G1 Dockerfile, and applies
the complete `unitree/g1/deploy/service.yml` runtime contract on top of the
target's Agent Core Compose file. If GitHub's Git endpoint is unavailable, a
GitHub repository automatically falls back to the official `codeload.github.com`
archive for the same pinned commit; third-party source mirrors are not used.
Set `REPO_URL`, `SOURCE_ARCHIVE_URL`, `SOURCE_REF`, `EXPECTED_COMMIT`,
`REMOTE_REPO`, or `IMAGE` only when overriding those explicit deployment
inputs. `DRY_RUN=1` validates and prints the resolved provenance without
connecting to the robot.

The G1 Dockerfile uses the Aliyun mirrors for both Ubuntu Ports and PyPI by
default. It replaces the obsolete Tencent Cloud Ubuntu source inherited from
the base image before installing build dependencies. Direct Docker builds can
override these defaults with the `UBUNTU_PORTS_MIRROR` and `PYPI_MIRROR` build
arguments; the runtime base image remains unchanged.

### Run Locally (without Docker)

```bash
cd unitree/g1
pip install -r requirements.txt
python main.py
```

## How It Works

1. Driver starts as an MCP HTTP server on its designated port
2. Driver sends a registration request to Agent Core
3. Agent Core discovers the driver's tools via MCP `initialize` and `tools/list`
4. Tools become available to the LLM agent and appear in the Web Dashboard
5. The LLM agent can invoke tools via MCP `tools/call`

### G1 Controlled Navigation Velocity

The G1 `loco` actuator accepts a lease-bound
`phanthy.navigation.velocity_proposal.v1` input. Valid proposals are executed
through a reliable `KEEP_LAST(depth=1)` subscription and a capacity-one
latest-only execution queue, so older unread or pending velocities are replaced
instead of backlogged. A proposal TTL lapse immediately triggers `StopMove`;
only a successful post-call zero-odometry confirmation keeps the same navigation
lease recoverable for the next fresh proposal. Hard safety, identity, sequence,
RPC, and stop-confirmation faults still disarm the lease.

`loco.start` connects the sole proposal topic and remains physically stopped.
While idle, the Driver validates the first fresh, legal, nonzero proposal and
atomically binds its `nav_id` before executing that same proposal. Another ID
cannot replace or interrupt the active task. A terminal zero proposal first
enters `terminal_pending_stop`; only a successful zero-odometry stop
confirmation retires the active ID and keeps the subscription ready for the
next task. A failed confirmation stays fail-closed, and a later successful
retry restores `awaiting_first_valid_proposal`. Invalid, stale, zero bootstrap,
terminal bootstrap, retired-ID, and mid-task mismatched-ID proposals never
establish or replace a lease, so Agent Core needs no per-task authorization
action for consecutive navigation tasks.

`nav_id` provides task isolation and replay protection; it is not publisher
authentication. First-proposal binding therefore assumes the robot ROS 2/DDS
network is trusted and isolated. An untrusted participant that can publish to
the proposal topic could claim an idle lease, so that topic must not be exposed
to untrusted publishers.

`loco info` exposes proposal counters, the coalesced count, measured RPC and
queue latency, rolling RPC p50/p95/p99/max values, rejection reasons, and the
last confirmed proposal stop. The `last_set_velocity_duration_ms` value is
measured RPC time, not the proposal TTL budget.

The velocity proposal contract matches the `loco.move` input bounds: forward
and lateral velocity are each limited to `[-1.0, 1.0] m/s`, and yaw velocity is
limited to `[-2.0, 2.0] rad/s`. The Driver still rejects non-finite or
out-of-range values before any motion RPC.

### G1 Navigation Sensors

The read-only `navigation_sensors` Driver plugin launches an isolated worker
process which subscribes directly to the MID360 raw DDS streams instead of
converting the body `/ubuntu/state/imu` JSON. Keeping raw LiDAR conversion and
IMU forwarding out of the full Driver process prevents the legacy LiDAR,
camera, and MCP threads from starving navigation input callbacks. The worker
publishes two algorithm-independent navigation sensor topics:

- `/ubuntu/navigation/lidar` — `sensor_msgs/msg/PointCloud2`,
  RELIABLE + KEEP_LAST(2), with `x/y/z/intensity/tag/line/timestamp` fields;
- `/ubuntu/navigation/imu` — `sensor_msgs/msg/Imu`, RELIABLE + KEEP_LAST(200);

Both cards share this single worker. Stopping either `navigation_lidar` or
`navigation_imu` stops both streams and releases the MID360 worker; starting
either card starts a fresh shared worker again.

LiDAR and IMU retain their shared MID360 source clock and are normalized into
one ROS system-time domain. Samples are dropped while clock offset estimation
is not ready or after an invalid/reset observation; the Driver never invents a
source timestamp. The fixed upside-down mounting rotation is applied equally
to cloud and IMU. The existing `/ubuntu/lidar/cloud` legacy card remains
enabled for Canvas and safety consumers.

Navigation consumers such as LiDAR-inertial mapping and path planning can bind
to these topics without the Driver naming or selecting a specific algorithm.
The body IMU JSON is approximately 20 Hz, has no source timestamp, and is not a
valid substitute for the MID360 built-in IMU. Before accepting a navigation
run, verify the isolated worker delivers approximately 10 Hz LiDAR and 200 Hz
IMU; materially lower rates or repeated source-stamp gaps invalidate the run.

### G1 Self-Describing Camera Frames

The RealSense plugin keeps the legacy `/ubuntu/camera/rgb` compressed image,
`/ubuntu/camera/depth` image, and distance topics unchanged. It additionally
exposes two latest-only, BEST_EFFORT self-describing frame streams. They are
general sensor/data-collection outputs and are not coupled to navigation:

- `camera_rgb_frame` → `/ubuntu/camera/rgb_frame` —
  `phanthy.sensor.camera_rgb_frame.v1`;
- `camera_depth_frame` → `/ubuntu/camera/depth_frame` —
  `phanthy.sensor.camera_depth_frame.v1`.

Both use `std_msgs/msg/UInt8MultiArray` as a transport for the `PSE1` binary
envelope: a fixed little-endian header (`magic`, JSON metadata length, binary
payload length), canonical JSON metadata, then JPEG or zlib level-1 losslessly
compressed little-endian Z16 bytes. Depth metadata declares the codec in
`image.compression` and includes compressed and uncompressed byte counts; zlib
decompression restores the original Z16 payload.
Every frame repeats its active-profile intrinsics, stable `calibration_id`,
RealSense Depth-to-RGB transform, source/Driver-receive timing, and the
configured LiDAR-to-RGB calibration. Invalid, warming-up, reset, or
out-of-order source timestamps are published as explicitly unavailable; the
Driver does not replace them with publish time.

The bundled LiDAR-to-camera transform is derived from the pinned official G1
URDF and is intentionally marked `factory_nominal`. It is not a measured
per-robot calibration and must not be relabeled `validated_on_device` until a
projection overlay and pixel-residual acceptance run has been recorded on that
G1. Missing or invalid calibration is represented as `unavailable`, never as
an identity transform. Camera reconnects rebuild the profile calibration and
therefore update `calibration_id` when serial, resolution, intrinsics, depth
scale, or extrinsics change.

## Writing a New Driver

Want to add support for new hardware? See the **[Driver Development Guide](README_dev.md)** for the full specification, including:

- MCP protocol implementation (JSON-RPC 2.0 methods)
- Tool definition spec (`inputSchema`, `configSchema`, `multiInstance`, `x-action-params`)
- Instance management (`multiInstance` flag, `scope` for config fields)
- Plugin lifecycle (`__init__`, `get_tool`, `start`, `stop`, `dispatch`)
- `driver.yaml` and `config.yaml` metadata format
- Registration and heartbeat with Agent Core
- Port allocation (15700–15799 range)

Quick overview:
- Each driver implements MCP JSON-RPC 2.0 over HTTP (`initialize`, `tools/list`, `tools/call`)
- Tool naming convention: `{device}_{action}` (e.g., `loco_move`, `mic_start`)
- Driver port range: **15700–15799**

### Topic Inference via `info` Action

**All** tools that produce or consume ROS2 topics must implement an `info` action. The Agent Core canvas calls `info(instance_id, input_topic)` immediately after a card is placed or wired, and uses the returned `topic_out`/`topic_in` as the **authoritative** topic path. Static definitions in `tool.topic_out` are used only as a fallback when `info()` is unavailable.

**Rule: driver owns topic path logic; canvas only reads the result.**

| Tool type | `info()` input | `topic_out` computation |
|-----------|---------------|------------------------|
| Static sensor (mic, imu, camera…) | — | Return fixed `self._topic` |
| multiInstance sensor (ext_mic, ext_camera) | `instance_id` | `/{namespace}/{tool}/{instance_id}/…` (replace `-` → `_`) |
| Processor (asr, tts) | `input_topic` | `{input_topic}/{tool_name}` |

Example for a static sensor:
```python
def dispatch(self, action: str, args: dict) -> dict | None:
    if action == "info":
        return {"state": self.state, "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}]}
    return None
```

> **Note:** ROS2 topic names only allow alphanumerics, `_`, `~`, `{`, `}`. Canvas card IDs
> contain hyphens (e.g. `card-abc123`), so drivers must sanitize `instance_id` before
> embedding it in a topic path: `instance_id.replace('-', '_')`.

The Agent Core canvas calls this endpoint immediately after a card is placed or wired,
so output port labels are populated without waiting for `start`.

---

## Data Rendering in Agent Core Dashboard

The Agent Core Web Dashboard renders live data streams based on the `format` field declared in a tool's `topic_out`. Each format is matched to a specialized renderer:

| Format | Renderer | Description |
|--------|----------|-------------|
| `audio/pcm-16k` | Audio waveform | PCM audio visualizer with playback |
| `video/mjpeg` | Video stream | Motion JPEG video display |
| `image/jpeg` | Image | Static JPEG image |
| `image/depth-z16` | Depth colormap | 16-bit depth image with color mapping |
| `data/json` | Text / KV panel | JSON key-value display |
| `text/*` | Text | Plain text display |
| `sensor/skeleton` | 3D Skeleton | URDF-based 3D skeleton with joint rotation |
| `sensor/lidar*` | Lidar scan | 2D/3D lidar point visualization |
| `sensor/pointcloud` | Point cloud | 3D point cloud renderer |
| `sensor/mapping` | 2D Map | Occupancy grid / SLAM map |

### Skeleton Rendering (`sensor/skeleton`)

For robot state monitoring, declare `"format": "sensor/skeleton"` in `topic_out`. The dashboard will:

1. Call your driver's `model` tool (type: `resource`) to fetch the URDF
2. Parse the URDF kinematic chain in the browser
3. Render a 3D skeleton with joint positions from the URDF
4. Apply real-time joint angles from `sensor/skeleton` topic data

**Requirements for skeleton support:**

- A `model` tool (type `resource`) that returns `{"urdf": "<URDF XML>"}` via MCP
- A `joints` tool (type `sensor`) with `topic_out` format `sensor/skeleton`
- Joint data published as `{"joints": [{"idx": 0, "name": "joint_name", "q": angle}, ...]}`
- **Joint names in data must match URDF joint names exactly** (e.g., `FL_hip_joint` not `FL_hip`)
- **`dispatch()` must return a plain dict** (e.g. `{"urdf": "..."}`) — do NOT return pre-wrapped MCP content arrays (see README_dev.md § "dispatch() Return Value Format")

---

## Audio Requirements for ASR Compatibility

Any driver that publishes audio for use with the Perception ASR plugin must meet the following requirements. Failure to comply will result in the ASR receiving audio but producing no output (the VAD silently discards non-conforming frames).

### ROS2 Message Type

```
audio_msgs/AudioChunk
  std_msgs/Header header
  string format          # must be exactly "audio/pcm-16k"
  uint8[] data           # raw PCM bytes
```

### PCM Format

| Parameter | Required value |
|-----------|---------------|
| Encoding | 16-bit signed integer, little-endian (PCM_S16_LE) |
| Sample rate | **16 000 Hz** |
| Channels | **Mono (1 channel)** |
| `format` field | `"audio/pcm-16k"` |

### Chunk Size

| Parameter | Constraint |
|-----------|-----------|
| Minimum | **1 024 bytes** (512 samples ≈ 32 ms) |
| Recommended | 1 024 – 4 096 bytes (32 – 128 ms) |

Chunks smaller than 1 024 bytes are **silently discarded** by the VAD. This is the most common cause of "ASR receives audio but never outputs text."

### The 48 kHz USB Mic Pitfall

Most USB audio interfaces capture at 48 000 Hz natively. After downsampling to 16 000 Hz, a 512-frame ALSA period yields only **170 samples (340 bytes)** — below the minimum. You must accumulate resampled output into a buffer and only publish when 512 samples are ready:

```python
TARGET = 1024  # bytes — 512 int16 samples @ 16 kHz
_buf = bytearray()

# Inside the capture loop, after resampling to 16 kHz:
_buf += resampled_bytes
while len(_buf) >= TARGET:
    chunk, _buf = bytes(_buf[:TARGET]), _buf[TARGET:]
    msg = AudioChunk()
    msg.format = "audio/pcm-16k"
    msg.data = list(chunk)
    publisher.publish(msg)
```

This pattern is already applied to the `ext_mic` plugin in `unitree/g1/ext_devices.py`.

See [perception/README.md](https://github.com/4paradigm/phanthymotus/blob/main/perception/README.md) in the main repository for full VAD tuning options.

See [CONTRIBUTING.md](CONTRIBUTING.md) for development setup and PR guidelines.

## License

[Apache License 2.0](LICENSE)
