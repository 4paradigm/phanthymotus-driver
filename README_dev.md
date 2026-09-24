# Driver Development Guide

The hardware driver layer (Layer 1) exposes device capabilities to Agent Core as MCP HTTP Servers.

---

## Directory Structure

Each driver is an independent Python package:

```
drivers/
├── <provider>/
│   └── <model>/
│       ├── main.py            # MCP HTTP Server entry point
│       ├── device.py          # Device plugin implementation
│       ├── config.yaml        # Plugin enable/disable configuration
│       ├── driver.yaml        # Metadata (ID, port, description)
│       ├── Dockerfile         # ARM64 container build
│       └── requirements.txt   # Python dependencies
```

Examples: `drivers/unitree/g1/`, `drivers/phanthy/remote_control/`

---

## MCP Protocol

Each driver implements [MCP](https://modelcontextprotocol.io) JSON-RPC 2.0 over HTTP, exposing three methods:

| Method | Description |
|--------|-------------|
| `initialize` | Handshake, returns `serverInfo.name` |
| `tools/list` | List all tools (with schema) |
| `tools/call` | Call a tool `{name, arguments}` |

The HTTP endpoint is uniformly `/mcp` (POST).

---

## Tool Definition Specification

Each tool returns a dict containing the following fields:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `name` | string | Yes | Tool name (e.g. `loco`, `mic`), unique within the same driver |
| `type` | string | Yes | `sensor` (data stream) \| `actuator` (executable) \| `processor` (data processing) \| `resource` (static resource) |
| `multiInstance` | boolean | No | Whether the tool can be added to the canvas multiple times. `true` = multiple instances allowed (e.g. ASR/TTS with different input topics), `false` (default) = single instance only |
| `description` | string | Yes | Tool description, used by both LLM and frontend |
| `inputSchema` | object | Yes | JSON Schema defining call parameters |
| `configSchema` | object | No | Persistent configuration schema (e.g. API Key), rendered as a config form in the frontend |
| `topic_out` | array | No | List of output ROS2 DDS topics `[{topic, format}]` |
| `topic_in` | array | No | List of input ROS2 DDS topics `[{format}]` |

### Tool Types

- **sensor**: Data stream tool, cannot be called directly. Controlled via `start`/`stop` system actions, data is pushed through ROS2 topics
- **actuator**: Tool that performs executable actions. Different operations are dispatched via the `action` field
- **processor**: Data processing tool. Receives input topic data, processes it, and outputs to a topic

### inputSchema

Standard JSON Schema format. For actuator tools, it typically includes an `action` field (enum) to distinguish between different operations:

```python
"inputSchema": {
    "type": "object",
    "properties": {
        "action": {
            "type": "string",
            "enum": ["move", "stop"],
            "description": "Action to perform",
        },
        "vx": {"type": "number", "description": "Forward velocity"},
    },
    "required": ["action"],
}
```

### configSchema

Optional. Defines persistent parameters that users configure in the frontend (e.g. API Key, model name). The frontend automatically renders a configuration form.

Each property can declare a `"scope"` field:

| Scope | Description |
|-------|-------------|
| `"shared"` (default) | Global config shared across all instances. Configured via the sidebar config button. |
| `"instance"` | Per-instance config. Each canvas card instance can have its own value. Configured via the card's gear button. |

For `multiInstance: true` tools, `scope` determines whether a config field is set once globally or independently per instance. For single-instance tools, all fields are effectively shared.

```python
"configSchema": {
    "type": "object",
    "properties": {
        "api_key":  {"type": "string", "description": "API Key", "format": "password", "scope": "shared"},
        "model":    {"type": "string", "description": "Model name", "scope": "instance"},
    },
    "required": ["api_key"],
}
```

#### Marking sensitive fields (`x-sensitive`)

Canvas configuration is packaged into shareable **Solutions** (Agent Core's
`/api/solutions/pack`), which are uploaded to the Resource Center marketplace.
Packaging blanks out sensitive values, but it can only do that for fields the
tool **declares** as sensitive — there is no field-name blocklist, because a
guessing heuristic would both miss real secrets and wrongly clear innocent
fields. A field counts as sensitive when either holds:

| Declaration | Also does |
|-------------|-----------|
| `"format": "password"` | Frontend renders a masked password input |
| `"x-sensitive": true`  | Nothing visually — use when the field must stay visible while typing |
| `"x-sensitive": false` | Opts **out** of packaging redaction even though `format: password` masks it |

```python
"configSchema": {
    "type": "object",
    "properties": {
        "api_key":     {"type": "string", "format": "password"},        # masked + never packaged
        "device_token":{"type": "string", "x-sensitive": True},         # visible + never packaged
        # Fixed factory password — masked in the UI, but not a user secret. Blanking it
        # would only make the recipient retype the same default.
        "ssh_pass":    {"type": "string", "format": "password", "x-sensitive": False},
        "endpoint":    {"type": "string"},                             # packaged as-is
    },
}
```

`format: password` defaults to sensitive on purpose — an unmarked password box is
assumed to be a real secret, so drivers written before this convention stay safe.
Use `"x-sensitive": false` only when the value is a fixed, publicly documented
default; if an operator can put a real credential in that field, leave it sensitive.

Anything you don't mark is packaged verbatim and becomes readable by everyone
who downloads the solution. Mark every credential, token, license key, private
endpoint and personal identifier. Fields that were blanked are reported to the
loading user as "needs configuration", so marking a field does not break the
solution — it just makes the recipient fill in their own value.

Two formats are cleared automatically and need no marking, because their values
only mean something on the machine they were set on: `channel-select` (a local
channel id) and `audio-input-device` (a local sound-card device).

---

## x-action-params Specification

### Problem

When a tool has multiple actions and different actions require different parameters (e.g. `loco`'s `move` requires velocity parameters while `stop` does not), all parameters are unioned into a flat schema, causing:

1. The LLM sees all parameters mixed together and cannot distinguish which belong to which action
2. The frontend displays all fields simultaneously, resulting in poor user experience

### Solution

Declare the `x-action-params` field in `inputSchema` to specify the corresponding parameter list and independent description for each action.

### Format

```python
"inputSchema": {
    "type": "object",
    "properties": {
        "action": {"type": "string", "enum": ["move", "stop", "set_stand_height"], ...},
        "vx":     {"type": "number", "description": "Forward velocity"},
        "height": {"type": "number", "description": "Standing height 0.0-1.0"},
    },
    "required": ["action"],
    "x-action-params": {
        "move":             {"params": ["vx", "vy", "vyaw"], "description": "Move the robot with velocities"},
        "stop":             {"params": [],                    "description": "Stop all movement"},
        "set_stand_height": {"params": ["height"],            "description": "Set standing height"},
    },
}
```

Each action entry:

| Field | Type | Description |
|-------|------|-------------|
| `params` | string[] | List of parameter keys used by this action (the `action` field itself does not need to be included) |
| `description` | string | Independent description for this action, used as the LLM function description |

### Effect

Agent Core automatically processes `x-action-params`:

- **LLM side**: Automatically splits into multiple independent functions (e.g. `mcp__unitree__loco__move`, `mcp__unitree__loco__stop`), each containing only the corresponding parameters
- **Frontend side**: When switching the action dropdown in canvas cards, only the corresponding parameter fields are displayed
- **Driver side**: No changes to dispatch logic needed; Agent Core automatically injects `action` into args when calling

### When to Use

- Must be used when a tool has multiple actions and **different actions require different parameters**
- Not needed when all actions share the same parameters (e.g. `switch_mode` where all modes only need the `mode` field)
- Not needed for single-action tools

### Complete Example

```python
def get_tool(self) -> dict:
    return {
        "name": "loco",
        "type": "actuator",
        "description": "G1 locomotion control — move, stop, set height, wave/shake hand",
        "inputSchema": {
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "enum": ["move", "stop", "set_stand_height", "wave_hand", "shake_hand"],
                    "description": "Action to perform",
                },
                "vx":         {"type": "number",  "description": "Forward velocity m/s [-1, 1]"},
                "vy":         {"type": "number",  "description": "Lateral velocity m/s [-1, 1]"},
                "vyaw":       {"type": "number",  "description": "Yaw rotation rad/s [-2, 2]"},
                "continuous": {"type": "boolean", "description": "Keep moving until stop (default false)"},
                "height":     {"type": "number",  "description": "Normalized height 0.0-1.0"},
                "turn":       {"type": "boolean", "description": "Turn while waving (default false)"},
            },
            "required": ["action"],
            "x-action-params": {
                "move":             {"params": ["vx", "vy", "vyaw", "continuous"], "description": "Move the robot with specified velocities"},
                "stop":             {"params": [],                                 "description": "Stop all movement immediately"},
                "set_stand_height": {"params": ["height"],                         "description": "Set the robot's standing height (0.0-1.0)"},
                "wave_hand":        {"params": ["turn"],                           "description": "Perform a waving hand gesture"},
                "shake_hand":       {"params": [],                                 "description": "Perform a handshake gesture"},
            },
        },
    }
```

---

## Plugin Lifecycle

Each device capability is encapsulated as a Plugin class that must implement:

```python
class MyPlugin:
    PREFIX = "my_tool"  # Tool name prefix (for multi-tool plugins)

    def __init__(self, plugin_config: dict, namespace: str, executor, ...):
        """Initialize. plugin_config comes from config.yaml, namespace is the ROS2 namespace."""
        pass

    def get_tool(self) -> dict:
        """Return a single tool definition."""
        # Or get_tools(self) -> list to return multiple

    def start(self) -> None:
        """Start the plugin (e.g. begin data acquisition)."""
        pass

    def stop(self) -> None:
        """Stop the plugin."""
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        """Dispatch a tool call. action is popped from args, args contains the remaining parameters."""
        if action == "start":
            return {"state": "running"}  # or "ready" for actuators
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running", "topic_out": [...]}
        if action == "do_something":
            return {"result": "ok"}
        return None
```

### dispatch() Return Value Format (CRITICAL)

**`dispatch()` must return a plain Python dict (or `None`).** The MCP HTTP handler automatically wraps it:

```python
# Handler does this automatically:
ok({"content": [{"type": "text", "text": json.dumps(result)}]})
```

**DO NOT** return pre-wrapped MCP content arrays from dispatch:

```python
# ❌ WRONG — causes double-wrapping, breaks frontend parsing
def dispatch(self, action, args):
    return [{"type": "text", "text": json.dumps({"urdf": data})}]

# ✅ CORRECT — return plain dict, handler wraps it
def dispatch(self, action, args):
    return {"urdf": data}
```

If you return `[{"type": "text", ...}]`, the handler wraps it again into `{"content": [{"type": "text", "text": "[{\"type\":\"text\",...}]"}]}` — the frontend receives double-encoded JSON and fails to parse, falling back to defaults.

- Provide `get_tool()` to return a single tool, or `get_tools()` to return multiple
- In `dispatch()`, `action` has already been extracted from args; if there is no action field, it equals the tool name

### start/stop in dispatch (Required)

**Every plugin must handle `start` and `stop` actions in its `dispatch()` method.** The MCP framework does NOT provide a default implementation — unhandled start/stop will return `None` to the caller, breaking the canvas lifecycle.

| Tool type | `start` return | `stop` return |
|-----------|---------------|---------------|
| sensor | `{"state": "running"}` | `{"state": "idle"}` |
| actuator | `{"state": "ready"}` | `{"state": "idle"}` |
| multiInstance sensor | Actual start logic (create node, open device) | Actual stop logic (destroy node, release device) |

**Rules:**

1. Always-on sensors (mic, imu, camera…): `start`/`stop` are no-ops that simply return the expected state dict
2. multiInstance sensors (ext_camera, ext_mic): `start` must create and activate the capture node; `stop` must destroy it and release resources
3. Actuators: `start`/`stop` are lifecycle markers; return "ready"/"idle" immediately

**Anti-pattern — do NOT do this:**

```python
# BAD: no start/stop handling, relies on framework magic
def dispatch(self, action: str, args: dict) -> dict | None:
    if action == "info":
        return {"state": "running", ...}
    return None  # start/stop will return None → broken!
```

**Correct pattern:**

```python
# GOOD: every plugin explicitly handles start/stop
def dispatch(self, action: str, args: dict) -> dict | None:
    if action == "start":
        return {"state": "running"}
    if action == "stop":
        return {"state": "idle"}
    if action == "info":
        return {"state": "running", "topic_out": [...]}
    return None
```

### Logging: keep stdout usable (Required)

A driver's stdout **is** its Docker log. The daemon frames every write into a
record (`local` = length-prefixed protobuf, `json-file` = JSON), and two things
break that framing so badly that `docker logs` returns nothing at all:

```
Error grabbing logs: invalid character '\x00' looking for beginning of value
Error grabbing logs: error unmarshalling log entry: proto: illegal tag 0 (wire type 6)
```

**1. Never redirect fd 1.** `fd 1 == the container log` is an invariant. A
`dup2(devnull, 1)` + `sys.stdout = os.fdopen(os.dup(1))` shuffle looks like a way
to silence a noisy native library, but it:

- leaves two buffered writers (`sys.stdout` and the still-live `sys.__stdout__`)
  on one pipe — writes above `PIPE_BUF` (4096 B on Linux) are **not atomic**, so
  concurrent lines interleave and tear a record in half;
- costs every `multiprocessing`/`subprocess` child its stdout, because
  `os.dup()` returns a non-inheritable fd and the child inherits fd 1 =
  `/dev/null`.

Silence the source instead: gate the prints, or set the library's own env var
(`CYCLONEDDS_URI` tracing to `/dev/null`, `RCUTILS_COLORIZED_OUTPUT=0`).

**2. Never truncate a live container's log file.** `truncate -s 0` resets the
file size but not the daemon's write offset, so the next write lands past EOF and
the kernel NUL-fills the gap — producing exactly the errors above. To reclaim
space use `docker restart <container>`; the daemon then reopens its writer
cleanly. Rotation is already declared in every `deploy/service.yml`.

**3. Install the atomic writer.** `common/logsafe.py` replaces `sys.stdout` with
a writer that emits each complete line in one `os.write`, capped below
`PIPE_BUF`, with C0 control characters and ANSI escape sequences stripped. Import
it first, and again at the top of every spawned child entry point — a spawned
child does not inherit the parent's `sys.stdout` object:

```python
from common import logsafe
logsafe.install()
```

Add `common` to the build context via `driver.yaml`, and copy it in:

```yaml
build_context_extras:
  - ../../common
```
```dockerfile
COPY common/ /work/common/
```

**4. Throttle per-frame logs.** Anything inside a sensor callback runs at 10–30 Hz.
Log the *state transition* unthrottled and sample the steady state:

```python
self._n = getattr(self, '_n', 0) + 1
if self._n == 1 or self._n % 100 == 0:
    print(f"[lidar] closest={d:.2f}m (n={self._n})", flush=True)
```

**5. Escape anything remote-controlled.** With `network_mode: host` the MCP port
is reachable, so an HTTP request line is attacker-controlled bytes. Escape and
cap before printing:

```python
safe = msg.encode("unicode_escape").decode("ascii")[:200]
print(f"[mcp] {self.address_string()} {safe}")
```

**Debugging escape hatch:** the vendored Unitree SDK's per-RPC and per-PCM-chunk
prints are gated behind `UNITREE_RPC_DEBUG=1`. Set it when chasing RPC timeouts
(error 3104); leave it unset in production, where those prints cost 3–5 lines per
RPC call.

### Logging checklist for a new driver

All 13 existing drivers satisfy this; a new one is expected to as well.

- [ ] `driver.yaml` has `build_context_extras: [../../common]`
- [ ] `Dockerfile` has `COPY common/ /work/common/`
- [ ] `main.py` calls `logsafe.install()` before anything prints (or routes
      through `common.vendor_runtime.run_driver()`, which installs it for you)
- [ ] every `multiprocessing` child entry point calls `logsafe.install(check_fd=False)`
- [ ] no `os.dup(1)` / `dup2(..., 1)` anywhere
- [ ] `log_message` escapes and caps the request line
- [ ] no unthrottled `print` inside a per-frame / per-message callback
- [ ] `Dockerfile` sets `PYTHONUNBUFFERED=1` and `RCUTILS_COLORIZED_OUTPUT=0`,
      plus `CYCLONEDDS_URI` tracing to `/dev/null` if the driver uses CycloneDDS
- [ ] `deploy/service.yml` declares `logging: {driver: local, max-size: 10m, max-file: 3}`

A quick self-check before opening a PR:

```bash
grep -rn "os\.dup(1)" --include="*.py" .            # must be empty
grep -rlF 'print(f"[mcp] {self.address_string()} {msg}")' --include=main.py .   # must be empty
```

Reviewers apply these as `agents/pr_review/rules/driver.md` in the phanthymotus
repo.

---

## driver.yaml Metadata

```yaml
id: g1-driver                   # Unique ID
name: Unitree G1 Bundle          # Display name
category: driver                 # Fixed as "driver"
hardware_provider: unitree       # Hardware vendor
hardware_model: "g1"             # Hardware model
image_name: g1                   # Docker image name (without registry prefix)
port: 15701                      # MCP HTTP port
mcp_url: "http://localhost:15701/mcp"  # MCP endpoint
description: "..."               # Device description
```

---

## config.yaml

Controls plugin enablement:

```yaml
mcp_port: 15701
ros_namespace: ""   # Leave empty to auto-use hostname

plugins:
  mic:
    enabled: true
  tts:
    enabled: true
  speaker:
    enabled: true
  led:
    enabled: true
  loco:
    enabled: true
  arm:
    enabled: true
  state:
    enabled: true
```

The path is specified via the `CONFIG_PATH` environment variable (defaults to the same directory).

---

## Registration & Heartbeat

After startup, the driver automatically registers with Agent Core (port 15678):

```
POST http://<agent-core>:15678/api/mcp
{
  "id": "g1-driver",
  "name": "Unitree G1 Bundle",
  "url": "http://<driver-ip>:15701/mcp",
  "transport": "http"
}
```

Upon receiving this, Agent Core executes `initialize` → `tools/list` and registers the tools into the registry.

---

## Port Allocation

Driver ports are allocated in the **15700–15799** range. This table listed two
entries for a long time while fourteen drivers were in use; the authoritative
source is `*/*/driver.yaml`, and this is a snapshot of it:

| Driver | Port | | Driver | Port |
|--------|------|-|--------|------|
| Unitree G1 | 15701 | | Unitree Go1 | 15715 |
| Unitree R1 | 15702 | | DeepRobotics Lynx M20 | 15716 |
| Unitree Go2 | 15703 | | AgiBot X2 | 15717 |
| Noetix Bumi | 15704 | | RealMan RM75-6F-V | 15718 |
| Booster K1 | 15705 | | PNDbotics Adam | 15722 |
| BrainCo Revo2 | 15706 | | Chasing Qianjiao P200 Pro | 15739 |
| X-humanoid Tianyi 2.0 | 15707 | | RoboEra Q5 | 15793 |
| EngineAI T800 | 15708 | | | |
| Unitree AS2W | 15709 | | **Simulator (generic)** | **15711** |
| Phanthy Remote Control | 15710 | | *reserved for simulators* | *15712–15714* |

Ports outside this repo: Agent Core 15678, Perception MCP 15720, Perception WS
15721, actucore 15730.

New drivers should choose an unoccupied port. The WebSocket port is typically the MCP port + 1.

Check before choosing, rather than trusting the table:

```bash
grep -h '^port:' */*/driver.yaml | sort -u
```

---

## Data Format & Dashboard Rendering

The Agent Core Web Dashboard automatically selects a renderer based on the `format` field declared in `topic_out`. Understanding this mapping is essential when implementing sensor plugins.

### Format → Renderer Mapping

| Format | Renderer | canRender logic |
|--------|----------|----------------|
| `audio/*` (e.g. `audio/pcm-16k`) | Audio waveform | `hint.startsWith('audio/')` |
| `video/*` (e.g. `video/mjpeg`) | Video stream | `hint.startsWith('video/')` |
| `image/jpeg` | Camera image | `hint === 'image/jpeg'` |
| `image/depth-z16` | Depth colormap (raw) | `hint === 'image/depth-z16'` |
| `image/depth-zlib` | Depth colormap (zlib compressed) | `hint === 'image/depth-zlib'` |
| `image` | Generic image | `hint === 'image'` |
| `data/json` | Text / KV panel | `hint === 'data/json'` |
| `text/*` | Text display | `hint.startsWith('text/')` |
| `sensor/skeleton` | 3D Skeleton (URDF) | `hint === 'sensor/skeleton'` |
| `sensor/lidar*` | Lidar scan | `hint.startsWith('sensor/lidar')` |
| `sensor/pointcloud` | 3D Point cloud | `hint === 'sensor/pointcloud'` |
| `sensor/mapping` | 2D Occupancy map | `hint === 'sensor/mapping'` |
| `state/odom` | Text / KV panel | `motus.odom/1` — see "Robot Odometry" |
| `control/velocity` | Text / KV panel | `motus.control/1` `twist` — see "Continuous Control" |
| `control/joint` | Text / KV panel | `motus.control/1` joint modes |
| (no hint) | Activity stream | Fallback when no format specified |

The last three carry a declared schema rather than free-form JSON, and the port
format is what a *consumer* matches on when wiring a canvas — the renderer is
incidental. `control/*` is an **input** port on a driver command card;
`state/odom` is an output.

### Depth Rendering — `image/depth-z16` vs `image/depth-zlib`

Two depth formats are supported:

- **`image/depth-z16`**: Raw uint16 buffer (640×480 = 614KB/frame). Uses `sensor_msgs/Image`. Simple but high bandwidth — causes CPU saturation on ARM64 due to DDS serialization of large messages.

- **`image/depth-zlib`** (recommended): Zlib-compressed uint16 buffer (~10-15KB/frame). Uses `sensor_msgs/CompressedImage` with `format="16UC1; compressedDepth zlib"`. 47× smaller, negligible publish overhead. Dashboard decompresses in browser using native `DecompressionStream`.

**Driver-side usage (Python):**
```python
import zlib
import numpy as np
from sensor_msgs.msg import CompressedImage

depth_image = np.asanyarray(depth_frame.get_data())  # uint16, 640×480
compressed = zlib.compress(depth_image.tobytes(), 1)  # level=1 fastest

msg = CompressedImage()
msg.format = "16UC1; compressedDepth zlib"
msg.data = compressed
publisher.publish(msg)
```

**Tool definition:**
```yaml
topic_out:
  - topic: /{namespace}/camera/depth
    format: image/depth-zlib
```

### Skeleton Rendering (`sensor/skeleton`) — Full Spec

The skeleton renderer provides real-time 3D visualization of robot joint states. It supports **any** robot morphology (humanoid, quadruped, etc.) as long as URDF is provided.

#### Required Components

**1. `model` tool (type: `resource`)**

Provides the robot's URDF model to the dashboard. Must return the full URDF XML.

```python
def _model_tool(self) -> dict:
    return {
        "name": "model",
        "type": "resource",
        "description": "Robot URDF model for skeleton renderer",
        "inputSchema": {"type": "object", "properties": {}},
    }

# In dispatch:
if tool_name == "model":
    urdf_path = Path(__file__).parent / "resource" / "my_robot.urdf"
    return [{"type": "text", "text": json.dumps({"urdf": urdf_path.read_text()})}]
```

**2. `joints` tool (type: `sensor`)**

Publishes real-time joint state data. Must declare format `sensor/skeleton`.

```python
def _joints_tool(self) -> dict:
    return {
        "name": "joints",
        "type": "sensor",
        "multiInstance": False,
        "description": "Joint states at 10Hz",
        "inputSchema": {"type": "object", "properties": {}},
        "topic_out": [{"topic": self._joints_topic, "format": "sensor/skeleton"}],
    }
```

**3. Joint data format (published on the topic)**

```json
{
  "joints": [
    {"idx": 0, "name": "FL_hip_joint", "q": 0.123, "dq": 0.45, "tau": 1.2},
    {"idx": 1, "name": "FL_thigh_joint", "q": -0.5, "dq": 0.0, "tau": 0.8}
  ],
  "imu_quat": [1.0, 0.0, 0.0, 0.0]
}
```

#### Critical: Joint Name Matching

The renderer matches joint data to URDF joints by name. The matching logic is:

```javascript
const jointName = j.name || MOTOR_INDEX_MAP[j.idx];
const obj = this._joints[jointName];
```

**The `name` field in joint data MUST exactly match the URDF `<joint name="...">` attribute.**

| URDF joint name | Data `name` field | Result |
|-----------------|-------------------|--------|
| `FL_hip_joint` | `FL_hip_joint` | Matched |
| `FL_hip_joint` | `FL_hip` | **NOT matched** |
| `left_knee_joint` | `left_knee_joint` | Matched |

#### Rendering Fallback Chain

The skeleton renderer has a three-level fallback:

1. **URDF provided** (`data.urdf` exists) → Parse kinematic chain, build accurate 3D model
2. **Quadruped marker** (`data.type === 'quadruped'`) → Render generic quadruped stick figure
3. **Neither** → Render humanoid fallback skeleton (G1 proportions)

Always prefer returning full URDF (option 1) for accurate rendering. The humanoid fallback is a last resort and **will show a human figure regardless of your actual robot morphology**.

#### URDF File Placement

Store the URDF file in your driver's `resource/` directory:

```
drivers/unitree/go2/
├── resource/
│   └── go2_model.urdf    ← URDF file here
├── main.py
├── device.py
└── ...
```

The URDF does not need mesh files (`.dae`/`.stl`) — the renderer only uses the kinematic chain (joint origins, axes, parent-child relationships) to build a stick-figure skeleton.

#### IMU Orientation

If `imu_quat` (quaternion `[w, x, y, z]`) is included in the joint data, the renderer applies it to the root body orientation for real-time tilt visualization.

---

## Build & Deploy

```bash
# Build from the drivers/ root directory
./build.sh <provider>/<model>   # e.g. ./build.sh unitree/g1

# Or manual Docker build
cd drivers/unitree/g1
docker build -t g1-driver .
```

- All Dockerfiles are based on ARM64 architecture
- Tencent Cloud mirror sources are used for acceleration
- Image naming format: `${REGISTRY}/${IMAGE_NAMESPACE}/${image_name}:${TAG}`
- See `.env.example` for environment variable configuration

### Deployment via service.yml

Each driver must include a `deploy/service.yml` file that defines its Docker Compose service fragment. When deploying via the Agent Core Web Dashboard, Agent Core extracts this file from the driver image and merges it into the host's unified `docker-compose.yml` at `/opt/phanthy-motus/`.

**Required fields:**

```yaml
unitree-g1:                      # Service name (must be unique)
  container_name: embodied-unitree-g1  # Recommended: embodied-{provider}-{model}
  image: __IMAGE__               # Placeholder, replaced by Agent Core at deploy time
  privileged: true               # Required: access to /dev and hardware
  volumes:
    - /dev:/dev                  # Required: device access for cameras, sensors, etc.
    # Required: the loopback-only DDS profile. See "DDS isolation" below —
    # a driver that skips this cannot talk to Agent Core at all.
    - /opt/phanthy-motus/dds-local.xml:/opt/phanthy-motus/dds-local.xml:ro
  environment:
    - ROS_DOMAIN_ID=42           # Same on every robot; do not allocate per-robot
    - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    - FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml
    - PYTHONUNBUFFERED=1
  logging:
    driver: local
    options:
      max-size: "10m"
      max-file: "3"
  restart: unless-stopped
```

**Notes:**

- **`container_name` 命名建议**: 推荐使用 `embodied-{provider}-{model}` 格式（如 `embodied-dji-matrice300`）。Agent Core 会自动从 service.yml 中读取 `container_name` 并用于容器状态查询/停止/删除。如果不指定 `container_name`，Agent Core 会回退到 `embodied-{driver_id}` 作为默认值。自定义名称（如 `dji-m300`）也可以正常工作。
- `privileged: true` and `/dev:/dev` are mandatory for any driver that accesses hardware (cameras, USB devices, GPIO)
- `network_mode`, `ipc`, `pid` are injected by Agent Core during deployment — do not specify them in service.yml
- The `__IMAGE__` placeholder is automatically replaced with the actual image reference
- Service name should follow the pattern `{provider}-{model}` (e.g. `unitree-g1`, `phanthy-remote-control`)
- Do **not** set `FASTDDS_BUILTIN_TRANSPORTS`. It conflicts with the profile's
  `useBuiltinTransports=false`, and the value cannot be unset from compose once an image bakes it
  into its `ENV` — the XML wins anyway, so the variable is only a source of confusion.

---

### DDS isolation — load the profile unless the driver manages DDS itself

**Both lines above are mandatory for any driver that reaches Agent Core over FastDDS**, which is all
of them except the two dual-domain cases listed at the end of this section. A driver container without them is not merely
unisolated: with `useBuiltinTransports=false` everywhere else, it ends up on a different transport
from the rest of the machine and **cannot reach Agent Core at all**. The symptom is a device that
registers over HTTP and shows up in the dashboard, while none of its topics ever carry data.

Why the profile exists: `/remote_control/message` — a *command* topic — was reaching every robot on
the office LAN. An instruction typed on one robot was executed by a second one too, with the
identical timestamp in both logs. DDS has no addressing and no authentication; every subscriber on
the domain receives everything. The fix pins FastDDS to `127.0.0.1`
(`interfaceWhiteList`), and because containers run with `network_mode: host` they share one
loopback — the local bus works normally, nothing crosses the machine.

`ROS_DOMAIN_ID` stays **42 everywhere**. Per-robot domain numbers were tried and rejected: the
usable range is narrow, and cloned images have no way to coordinate a unique number.

**Your robot-body link is unaffected.** Drivers that speak to the hardware over the vendor SDK use
**CycloneDDS** with an explicitly bound interface (`ChannelFactoryInitialize(0, "eth0")`), and
`FASTRTPS_DEFAULT_PROFILES_FILE` only affects FastDDS. The two stacks coexist in one process.
Verified on a real R1: with and without the profile, a read-only `rt/lowstate` probe reported the
identical packet count and IMU yaw. Raw UDP multicast (R1's microphone uses `239.168.123.161:5555`
via `IP_ADD_MEMBERSHIP`) is likewise untouched — it is not DDS.

Two things that bite when deploying by hand:

- **A missing file fails silently, and worse.** If the host has no
  `/opt/phanthy-motus/dds-local.xml`, Docker's bind mount creates a *directory* with that name;
  FastDDS ignores it and falls back to every interface. Agent Core writes the file from its own
  image when it is absent — but a container that already mounted the phantom directory must be
  **recreated**, not restarted (`docker start` cannot change a mount type fixed at creation; it
  fails with `not a directory: Are you trying to mount a directory onto a file`).
- **Judge by socket bindings, not by config.** Check that the driver's UDP sockets bind loopback:
  `sudo ss -lunp | grep 179` should show `127.0.0.1:179xx` (plus a `239.255.0.1` multicast join,
  which is expected — the whitelist decides which interface it joins on). Agent Core also exposes
  `GET /api/peer/dds_isolation`.

**Two drivers do not set these lines in `environment`, for two different reasons — and neither is
"isolated" in the sense the fleet profile means.** If you write a driver in either shape, read the
row that matches:

| Driver | Why the compose variable does not work | Status |
|---|---|---|
| `engineai/t800` | Its `CMD` forces `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, so **both** its domains run on CycloneDDS. `FASTRTPS_DEFAULT_PROFILES_FILE` has no effect at all; CycloneDDS is configured through `CYCLONEDDS_URI`, which this driver pins to the robot interface (`eth1`) — for both contexts. | **Open gap.** Its domain-42 traffic is still on the LAN. Untried: no T800 hardware available. `check_service_yml.py` reports it as `GAP`. |
| `x-humanoid/tianyi2.0` | It holds **two FastDDS contexts in one process** (`DualDomainROS2` in `main.py`, `BridgeROS2` in `joints_bridge.py`), and the fleet profile would put the body link on loopback and cut it. It therefore selects the **vendor** profile (`/work/dds_profile.xml`) for the whole process, before any participant exists. | **Partly isolated, by whitelist rather than by loopback** — see below. |

### One profile per process — per-participant selection does not work

An earlier version of this section said tianyi "selects a profile per DDS context by setting the
variable around each `rclpy.init()`", and cited it as proof that per-participant profiles are
possible. **That was wrong, and shipping it silently cut part of the body link.** Two separate
reasons, either one fatal:

1. **FastDDS reads `FASTRTPS_DEFAULT_PROFILES_FILE` at participant creation, not at
   `rclpy.init()`** — and rmw_fastrtps creates the participant lazily, with the first `Node` on the
   context. Setting the variable around each `rclpy.init()` sets it around the wrong call: by the
   time the first real Node appears, the variable holds whatever was written last.
2. **The parsed profiles are cached process-wide**, so switching the variable between contexts
   cannot give them different profiles at all — it only decides which single profile both use.
   Measured on the robot: a process that set the vendor profile, created a domain-0 node, then set
   the loopback profile and created a domain-42 node, ended with *both* domains bound to
   `127.0.0.1` **and** `192.168.41.2` — the vendor whitelist, for both.

What the wrong profile cost, for calibration on how quiet this failure is: with the loopback-only
profile in force, the domain-0 participant bound `127.0.0.1` but not `192.168.41.2`, where the
vendor stack lives. Visible domain-0 topics fell from 77 to 33, all 26 of the driver's own
`tianyi2_*` nodes vanished from domain 0, and lyre's `/audio_play/play_text` became undiscoverable
(2.76 s to discover under the vendor profile; nothing after 15 s under the loopback one). Nothing
was logged. `/arm/status` and `/head/status` survived, so the robot looked healthy — while TTS
reported success and produced no sound, with lyre's journal confirming it had received nothing.

The earlier "verified" claim was a real measurement, but a one-sided one: it checked that domain 42
had moved to `127.0.0.1` and did not check what had happened to domain 0. When you verify an
isolation change, measure **both** sides of the link it runs through.

So tianyi runs the vendor profile process-wide. Its whitelist is
`{192.168.41.2, 127.0.0.1}`: the body link works, and — the point of the fleet-wide profile — the
**office LAN is excluded**, so domain 42 cannot carry `/remote_control/message` to another robot.
Be precise about what that is not: domain 42 is still reachable from the body board on
`192.168.41.x`. That board runs no Agent Core and is internal to this robot, so nothing there can
act on a command. Narrowing it to true loopback needs the agent-core-facing publishers moved into
their own process, one profile each.

Because of this, tianyi does **not** use the `dds-local.xml` mount, and `check_service_yml.py` does
not require it for that driver. Verified after the fix: both domains bind `127.0.0.1` and
`192.168.41.2`, **no `10.100.x`**; 26 `tianyi2_*` nodes visible on domain 0; TTS returns `ready`
and plays with a real sid, `PlayProgress` and a `COMPLETED` `PlayEvent`.

### service.yml checklist for a new driver

Run `./scripts/check_service_yml.py` to verify all of this — it is what a reviewer should run on any
PR that adds or edits a `deploy/service.yml`. It exits non-zero on a violation, so it also works as
a CI step.

- [ ] mounts `/opt/phanthy-motus/dds-local.xml:/opt/phanthy-motus/dds-local.xml:ro`
- [ ] sets `FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml`
- [ ] does not set `FASTDDS_BUILTIN_TRANSPORTS` — it conflicts with the profile's
      `useBuiltinTransports=false`, and an image that bakes it into `ENV` cannot be corrected from
      compose anyway (the XML wins, so the variable only misleads whoever reads the file next)
- [ ] `ROS_DOMAIN_ID=42` — the same on every robot; there is nothing to allocate. A driver holding
      a second context may spell it `<PREFIX>_ROS_DOMAIN_ID` for the body and
      `AGENT_CORE_ROS_DOMAIN_ID` for this one; only the Agent Core side must be 42
- [ ] `network_mode: host` — isolation works by confining DDS to loopback, and containers share a
      loopback only under host networking

The first two are the ones that break a robot rather than merely leaving it unisolated: with
`useBuiltinTransports=false` everywhere else, a container that misses them ends up on a different
transport from the rest of the machine and **cannot reach Agent Core at all** — the device registers
over HTTP and appears in the dashboard while none of its topics ever carry data, which sends you
looking at the driver instead of at compose.

The checker keeps two tables instead of one, so that neither exception quietly becomes a loophole:

- `OWN_PROFILE` — drivers that must **not** set the fleet profile because they ship their own for
  the whole process (`x-humanoid/tianyi2.0`; see § "One profile per process" above for why per-context
  selection is not an option). The mount is not required either — requiring it would imply the file
  is in use. Setting the fleet profile here is a failure, not a pass: it cuts the body link.
- `KNOWN_GAPS` — drivers a FastDDS profile cannot isolate at all, currently `engineai/t800`, whose
  RMW is CycloneDDS. It is reported as `GAP` and does not fail the run. Adding a third such driver
  means editing this table, which is the point: the gap stays visible rather than passing a check
  named "isolation".

`ipc` and `pid` are deliberately **not** checked — they vary legitimately across drivers (a drone
does not need `pid: host`), and the profile disables shared memory anyway.

---

## Action Completion Protocol (ACP)

When a tool performs a long-running physical action (TTS playback, navigation, arm gesture), the LLM agent needs to know when it finishes. ACP solves this at the **harness level** — Agent Core transparently tracks async actions via an automatic barrier. The LLM is unaware of the async machinery.

### How It Works

```
LLM calls speak("hello world")
  → Driver dispatch() returns {"state":"speaking", "action_id":"tts_speak_a7f3c"}
  → Agent Core registers pending action (from x-completion + action_id in result)
  → LLM sees immediate result, continues reasoning

LLM calls navigate_to_tag("P3")
  → Agent Core BARRIER: waits until speak-a7f3c completes before dispatching
  → ...TTS finishes, driver POSTs /api/acp/complete → pending cleared...
  → BARRIER releases, navigate dispatched
  → Driver returns {"state":"navigating", "action_id":"nav_b2e8d"}
  → LLM continues reasoning (can pre-plan next speech)

LLM calls speak("welcome to P3")
  → BARRIER: waits until nav_b2e8d completes
  → ...navigation arrives, driver POSTs completion...
  → speak dispatched
```

**Key**: The barrier is automatic and transparent. No `sync()` tool needed. The LLM just calls tools normally — the harness ensures physical actions execute sequentially.

### Barrier Scope

The barrier blocks **actuator** and **processor** type tools while pending actions exist. It does NOT block:
- **sensor** tools (camera, lidar, battery queries) — always instant
- **resource** tools (list_tags, list_maps, get_status) — read-only

### Driver Implementation Guide

#### 1. Declare `x-completion` in tool schema

Add to your tool's `inputSchema`:

```python
"inputSchema": {
    "type": "object",
    "properties": { ... },
    "required": ["action"],
    "x-completion": {
        "actions": ["speak", "navigate_to_tag"],  # which actions are async
        "timeout": 120                             # max wait seconds (fallback)
    }
}
```

Only declare actions that are genuinely long-running (>3s). Short blocking calls (1-2s service calls) should remain synchronous.

#### 2. Return `action_id` in async action responses

When an async action is dispatched, include `action_id` in the return dict:

```python
from uuid import uuid4

def dispatch(self, action, args):
    if action == "speak":
        action_id = f"tts_speak_{uuid4().hex[:8]}"
        threading.Thread(target=self._do_speak, args=(args["text"], action_id), daemon=True).start()
        return {"state": "speaking", "action_id": action_id}
```

The `action_id` must be unique — it's the correlation key for completion.

#### 3. POST completion to Agent Core

When the action finishes, POST to Agent Core's ACP endpoint:

```python
def _acp_callback(self, action_id: str, status: str, result: dict):
    """POST action completion to Agent Core."""
    import urllib.request as _urllib
    import ssl as _ssl
    import json
    import os as _os

    agent_core_url = _os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    ctx = _ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = _ssl.CERT_NONE
    payload = json.dumps({
        "action_id": action_id,
        "status": status,       # "completed" | "error" | "cancelled"
        "result": result,       # task-specific data
        "tool": self.PREFIX,    # tool name for logging
        "ts": __import__('time').time(),
    }).encode()
    try:
        req = _urllib.Request(
            f"{agent_core_url}/api/acp/complete",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        _urllib.urlopen(req, timeout=5, context=ctx)
    except Exception as e:
        import sys
        print(f"[ACP] callback failed for {action_id}: {e}", file=sys.stderr)
```

**Important**: Use `import os as _os` inside the callback function — nested functions and threads may not see module-level `os` in some contexts.

Call this from your worker thread when the action finishes. `AGENT_CORE_URL` env var is set in all driver containers.

#### 4. No SSE endpoint needed

Drivers do NOT need to implement an `/sse` endpoint. The completion notification is a simple HTTP POST.

### Advanced Patterns

#### Dynamic Timeout (TTS)

For text-to-speech, timeout should scale with text length:

```python
timeout = len(text) / 3.0 + 10  # ~3 chars/sec + buffer
```

If actual playback duration is known (e.g. from a progress callback):

```python
timeout = reported_duration + 5.0  # actual duration + small buffer
```

#### Race Condition Buffer (Event-Based Completion)

When completion depends on an external event (e.g. ROS2 PlayEvent topic), the event may arrive BEFORE your pending wait is registered. Use a buffer:

```python
class TtsPlugin:
    def __init__(self):
        self._play_event_buffer: dict[str, int] = {}   # sid → event_code
        self._pending_play: dict[str, threading.Event] = {}

    def _on_play_event(self, msg):
        """ROS2 callback — may fire before _pending_play[sid] exists."""
        sid = msg.sid
        event_code = msg.event_code
        # Always buffer
        self._play_event_buffer[sid] = event_code
        # Also signal if pending exists
        if sid in self._pending_play:
            self._pending_play[sid].set()

    def _wait_for_completion(self, sid, action_id, timeout):
        # Check buffer first (event already arrived)
        buffered = self._play_event_buffer.pop(sid, None)
        if buffered is not None:
            status = "completed" if buffered == 1 else "error"
            self._acp_callback(action_id, status, {})
            return
        # Not buffered yet — register and wait
        ev = threading.Event()
        self._pending_play[sid] = ev
        ev.wait(timeout=timeout)
        self._pending_play.pop(sid, None)
        buffered = self._play_event_buffer.pop(sid, None)
        status = "completed" if buffered == 1 else "error"
        self._acp_callback(action_id, status, {})
```

#### Stall Detection (Navigation)

For navigation actions, detect when the robot stops moving but hasn't arrived:

```python
def _nav_poll_thread(self, action_id, target, stall_timeout=60):
    last_pose = self._get_current_pose()
    last_move_time = time.time()

    while True:
        time.sleep(1.0)
        pose = self._get_current_pose()

        if self._has_arrived(pose, target):
            self._acp_callback(action_id, "completed", {"pose": pose})
            return

        if self._distance(pose, last_pose) > 0.05:  # moved
            last_pose = pose
            last_move_time = time.time()
        elif time.time() - last_move_time > stall_timeout:
            self._acp_callback(action_id, "error", {"error": "stall timeout"})
            return
```

### Backward Compatibility

- Tools without `x-completion` → unchanged behavior (sync return)
- Tools that don't return `action_id` → no pending registered, barrier passes through
- Drivers that don't POST completion → barrier will timeout gracefully (uses `x-completion.timeout`)
- Tools without `x-resource` → treated as exclusive against **everything** (old global
  barrier behaviour). Safe, but see "Declare it on *every* acting tool" below: a
  partially-declared driver is the case that behaves worst.

---

## Physical Resources (`x-resource`)

The ACP barrier is scoped by **physical channel**, not by tool type. Declare which
channel(s) an action occupies, next to `x-completion`:

```python
"inputSchema": {
    "type": "object",
    "properties": { ... },
    "required": ["action"],
    "x-completion": {"actions": ["speak"], "timeout": 60},
    "x-resource": "mouth",                    # or ["base", "arm_l"] for multi-channel
}
```

**Why this exists.** The barrier used to block *any* acting tool on *any* pending
action. That conflates two unrelated things — "I need X's result before Y"
(causality) and "X and Y both need the mouth" (exclusion) — and implements neither,
landing on "everyone waits for everyone". Speaking blocked navigating. One
background agent speaking blocked every other agent's every actuator call, on
unrelated hardware. `robotera/q5_bundle/` already splits `base_drive`,
`arm_gesture`, `leg_control` and `waist_control` into separate tools; the global
barrier serialised all four for no reason.

**Naming.** A resource is a thing there is physically one of. Use the same string
across every tool that drives the same hardware, and different strings for channels
that genuinely move independently:

| Channel | Typical tools |
|---------|---------------|
| `mouth` | `tts`, `speaker` |
| `base`  | `loco`, `navigate`, `base_drive` |
| `arm_l` / `arm_r` | `arm_gesture`, arm IK, gripper |
| `leg`   | `leg_control`, `switch_mode` |
| `waist` | `waist_control` |
| `head`  | gimbal / head pan-tilt |

**Rules.**

- **Undeclared means exclusive against everything.** Omitting `x-resource` is safe;
  a *wrong* one is not, since it can let two conflicting actions run at once.
- Malformed values (`{}`, `42`, `""`, `[]`) fall back to undeclared rather than to
  "conflicts with nothing" — a typo must not silently unlock parallel actuation.
- One tool may hold several channels: `"x-resource": ["base", "arm_l"]` for an action
  that drives while pointing. It then conflicts with anything touching either.
- Two *different* drivers using the same channel name is meaningful and correct —
  two `tts` tools on one robot are still one speaker.
- This is orthogonal to `type`. `type` decides whether a tool is barriered at all
  (`sensor`/`resource` never are); `x-resource` decides *what it waits for*.

### Declare it on *every* acting tool, not only the async ones

The barrier has two sides, and only one of them needs `x-completion`:

| role | what `x-resource` does | needs `x-completion`? |
|------|------------------------|------------------------|
| **holder** | tells others what this action blocks while it runs | yes — only an async action has a pending |
| **requester** | tells the barrier what this call must *wait for* | **no** — every `actuator`/`processor` tool asks |

Miss the requester side and the tool asks with "undeclared", which means *conflicts
with everything*, so it waits on any pending action anywhere on the robot. Measured
on Tianyi, where `arm_gesture`/`tts` were declared but the direct-control `arm`/`head`
were not:

| observed | cause |
|---|---|
| head sat idle 5 s before moving | `head` (undeclared) waited on an `arm_gesture` pending |
| arm sat idle 8 s before moving | `arm` (undeclared) waited on a `tts` pending |

So **a partially-declared driver is worse than an undeclared one**: undeclared is
uniformly serial and honest about it, partial looks like it should overlap and
doesn't. It also compounds — with motions serialised behind unrelated channels,
delegated subagents ran long enough to hit their delegation timeout, got cancelled
mid-run, and the caller redid work that had already happened.

Three-way summary of getting it wrong:

- **undeclared** → safe, slow. No correctness risk.
- **partially declared** → safe, slow, and *surprising*. This is the trap.
- **wrongly declared** → the only case that is unsafe, because it permits
  concurrency that the hardware does not.

A tool that genuinely occupies no channel cannot say so — an empty `x-resource`
normalises to "undeclared" on purpose, so a typo fails safe. Such a tool is almost
always mis-typed: if it only reads state, give it `type: sensor` or `resource`, which
exempts it from the barrier entirely. (Note that changing `type` also changes who may
call it: a `viewer`-role peer may call sensor tools. Do not retype a tool casually.)

### `x-resource` is a label, not a lock — two cards on one resource must arbitrate

ACP uses `x-resource` to decide what waits behind what *within its own
scheduling*. It does not stop two cards from reaching the same hardware by
different routes, and the moment a physical resource has both a **call-shaped**
card and a **stream-shaped** card, it has exactly that.

R1's chassis is the case to learn from. `loco` is call-shaped (`move`,
`stop_move`, LLM-callable); `loco_servo` is stream-shaped (a `control/velocity`
topic at 10 Hz). Both hold the same `RpcProxy` and both call `Move`. Matching
`x-resource: ["base"]` on the two makes them *look* coordinated while
`loco.move(duration=-1)` and a streaming policy fight each other at 10 Hz, with
nothing logging the conflict — each card is behaving exactly as designed.

If you add a stream-shaped card beside an existing call-shaped one:

- give the stream card a reference to the call-shaped plugin, and **refuse to
  start** while a continuous motion is in flight there;
- make the call-shaped actions **pause the stream first**. An explicit
  instruction from a person or the LLM outranks a running policy, and that
  direction must not be reversed;
- wire both into the interrupt card (`SmartMotionPlugin` and friends), or
  `interrupt` stops one and the other keeps driving;
- watch registration order in `main.py` — a plugin that looks its neighbour up
  by `PREFIX` finds nothing if it is constructed first.

The same applies to `arm` versus `servo` on G1; there the handover is mediated
by arm_sdk's weight ramp rather than by a reference, but the requirement — one
owner of the hardware at a time, explicitly — is the same.

### It is a vocabulary, not a fixed list — including for non-humanoids

Agent Core contains **no channel names at all**; it only intersects the strings
drivers declare. The names above are a humanoid convention, nothing more. A drone
would declare `rotor`, `gimbal`, `camera`; an underwater vehicle `thruster`,
`ballast`, `rudder`, `manipulator`. Nothing needs to change in the core for either.

Two limits are worth knowing before relying on it:

**It expresses mutual exclusion only.** Not ordering ("announce *before* moving"),
not simultaneity ("both arms must start together"), not reader/writer sharing, not
hierarchy (`arm_l` and a wrist-only tool are unrelated strings unless the wrist tool
also declares `arm_l`), and not capacity (two motors each fine alone but not
together). If your platform needs those, this is not the mechanism.

**It assumes actions are discrete and bounded** — the same assumption `x-completion`
makes. A multirotor's rotors are held *continuously* while airborne: hovering is a
state, not an action that completes. Declaring `rotor` on a takeoff tool that never
reports completion would hold that channel forever and block everything behind it.
For continuous-state platforms, either keep the state-entering tool out of ACP
(no `x-completion`, so no pending is held) or model the *transitions* as the actions.
`dji/mavic3e` currently declares no `x-completion` at all, so it is in the first
camp by default.

---

## System Hooks (`x-hooks`)

System hooks enable **instant, bypass-LLM actions** triggered by framework events. Unlike normal tool calls (which require LLM decision + ACP barrier), hooks fire directly and immediately (<50ms).

### Use Cases

- **LED feedback**: blink on hearing, breathe while thinking, flash red on error
- **Interrupt**: stop TTS/motion instantly on user barge-in (no barrier wait)
- **Status indicators**: hardware signals for robot state

### How It Works

```
Driver declares x-hooks in tool schema
  → Agent Core registers bindings at device init/heartbeat
  → System event occurs (ASR arrives, LLM starts, error...)
  → Agent Core fires hook: call_tool_direct() → bypasses barrier + ACP
    (on_notify is the one exception — see the table below)
  → Driver executes action immediately
```

### Driver Implementation

Add `x-hooks` to your tool's `inputSchema`:

```python
"inputSchema": {
    "type": "object",
    "properties": { ... },
    "required": ["action"],
    "x-hooks": {
        "on_hearing":    {"action": "effect", "params": {"effect": "blink_blue"}},
        "on_thinking":   {"action": "effect", "params": {"effect": "breathe_rainbow"}},
        "on_error":      {"action": "effect", "params": {"effect": "blink_red_5s"}},
        "on_kws_wakeup": {"action": "effect", "params": {"effect": "solid_blue_2s"}},
        "on_interrupt_all": {"action": "interrupt_all"},
    }
}
```

Each hook entry maps a `hook_id` to an action + params that will be called directly.

### Available Hook IDs

| Hook ID | Fired When | Typical Binding |
|---------|-----------|-----------------|
| `on_hearing` | ASR detects voice activity | LED blink / pause TTS |
| `on_kws_wakeup` | Wake word detected | LED solid / chime |
| `on_thinking` | LLM turn starts | LED breathe animation |
| `on_error` | LLM call fails after retries | LED red flash |
| `on_interrupt_speak` | User barge-in (speech) | Stop TTS |
| `on_interrupt_motion` | Emergency stop | Stop locomotion |
| `on_interrupt_all` | Full interrupt | Stop all outputs |
| `on_notify` | LLM produced a notify-worthy content string | Speak it (`{"action": "speak"}`), or flash/blink if no speaker |

`on_notify` fires with `extra_params={"text": "..."}` — the text is merged into whatever static
`action`/`params` your binding declares. A speech-capable tool typically just needs `{"action":
"speak"}` (the merged `text` key lines up with its own `speak` action). A device with no speaker
(e.g. a drone) can bind its LED tool instead, e.g. `{"action": "set_effect", "params": {"pattern":
"notify_blink"}}` — the unused `text` key in the merged args is harmless (hook calls skip schema
validation) and the LED just blinks per its own fixed pattern.

`on_notify` is **not** a true interrupt, and unlike every other hook here it does not fully bypass
the barrier: it narrates so the user isn't left in silence during a long tool-calling turn, so it
must not talk over whatever the robot is already saying, and it must not itself get talked over by
the very next LLM-issued tool call. Agent Core fires it with `barrier_aware=True`, which (1) skips
the call outright if the bound tool's `x-resource` is already held by a pending action, and (2)
registers any `action_id` the call returns as pending, same as a normal ACP dispatch — so a
subsequent `speak`/`navigate`/etc. from the LLM waits for it like it would for any other pending
action. If your `on_notify` binding declares `x-completion` and `x-resource` like a normal action
(Tianyi's `tts` does), this happens automatically; no driver-side change is needed to opt in.

### Key Differences from Normal Tools

| Aspect | Normal Tool Call | Interrupt Hook (`on_interrupt_*`, LED hooks) | `on_notify` |
|--------|----------------|-----------------------------------------------|-------------|
| Triggered by | LLM decision | System event | LLM wrote non-empty content |
| Barrier | Waits for pending | Bypasses | Skips the call if resource busy, else registers pending |
| ACP | Registers pending | Does not | Registers pending (if the tool declares `x-completion`) |
| Latency | 1-5s (LLM round) | <50ms | <50ms, or skipped entirely if busy |
| Schema validation | Yes | No | No |

### Manual Triggering (API)

```bash
# Fire a hook manually
curl -X POST https://localhost:15678/api/hooks/fire \
  -H 'Content-Type: application/json' \
  -d '{"hook": "on_interrupt_all"}'

# List registered hooks
curl https://localhost:15678/api/hooks
```

---

## Continuous Control (`motus.control/1`)

MCP `tools/call` is the control plane: low frequency, request/response, authorised.
It is the wrong shape for an execution model — a VLA policy or a navigation stack
produces tens of commands per second, and each one is not a question.

Those go on the **data plane**: a `control/*` DDS topic, exactly as a speaker
already takes its audio on `topic_in: audio/pcm-16k` while its start/stop go
through tools. `motus.control/1` is the agreement about what flows there.

Until this existed, `control/joint` and `control/velocity` were format strings in
agent-core's topic-inference table and nothing else: no fields, no units, no joint
order, and no driver subscribing to either.

Design rationale: `phanthymotus/docs/vla-integration.md` § "通用控制接口".

### Two halves

| Half | Where | What |
|------|-------|------|
| **Descriptor** | your command card's `info()` | what this driver accepts — the authoritative definition |
| **Message** | the `control/*` topic | one command, checked against the descriptor before it moves anything |

A **URDF is not the descriptor** and cannot substitute for it. It carries no units,
no control rate, no statement of whether you take absolute or incremental
positions, and no normalisation range. Reference it from `urdf_ref` as a
supplement for FK and collision geometry; do not derive the action interface
from it.

### Descriptor

```python
{
    "control_interface": "motus.control/1",
    "mode": "joint_position",       # joint_position | joint_velocity | joint_torque
                                    # | eef_pose | twist
    "dof": 14,
    "joint_names": [...],           # order IS the meaning of `values`
    "units": {"angle": "rad", "linear": "m", "time": "s"},
    "limits": {
        "lower": [...], "upper": [...],         # required
        "max_velocity": [...],                  # optional
        "max_delta_per_step": [...],            # optional
    },
    "frame": "base_link",           # eef_pose / twist only
    "end_effector": {"type": "gripper_2f", "range": [0.0, 0.09], "units": "m"},
    "rate": {"max_hz": 100, "expected_hz": 30, "watchdog_ms": 200,
             "max_obs_age_ms": 300},
    "force_torque": [30.0, 30.0, 30.0, 5.0, 5.0, 5.0],   # or null — REQUIRED either way
    "urdf_ref": "mcp__<id>__model",
}
```

`force_torque` must be present **even as `null`**. `parse_descriptor` rejects a
descriptor that omits it, because omitting it is how a robot ends up assumed to
have a protection it does not have.

#### `groups` — 混合向量，以及 `advisory`

一条 19 维的标准 G1 动作是两个末端位姿、两个夹爪、三个腰关节角。顶层一个 `mode`
说不清，所以每段可以自己声明：

```python
"groups": [
    {"name": "eef_l",  "offset": 0,  "count": 7, "unit": "m+quat",
     "resource": "arm_l", "mode": "eef_pose"},
    ...
    {"name": "waist",  "offset": 16, "count": 3, "unit": "rad",
     "resource": "waist", "mode": "joint_position", "advisory": True},
]
```

段必须按顺序无缝铺满 `[0, dof)`。不写 `mode` 就继承顶层。

**`advisory: true` 的意思是「这一段我收下，但不执行」。** `ControlSink` 因此跳过
这一段的限位、步长与速度检查 —— 检查一个不会被执行的数没有意义，而更糟的是，
一个为「这台机器人动不了这个轴」而卡死的限位会把**整条**指令拒掉，连同那些本可以
执行的维。

这不是「静默丢几维」这个仓库一直拒绝的那件事，区别在于**谁给的许可**：

| 侧 | 字段 | 含义 |
|---|---|---|
| 生产者（`motus.vla/1` capabilities） | `optional` | 我会发这一段，但任务不要求它被执行 |
| 驱动（这里） | `advisory` | 我收下这一段，但不执行 |

两者在 `actucore/plugins/vla/negotiate.py` 相遇，规则只有一句：**驱动标了
`advisory` 而生产者没标 `optional` → 拒绝协商**。两个名字故意不同，语义不对称，
同名会让一次复制粘贴把「可以不执行」变成「已经没执行」。两个字段都缺省 `false`。

只有**手段**能标 advisory，**目的**不能。判据是：丢掉它之后，还有没有东西能发现
执行错了。G1 的腰能标（末端位姿是绝对的，IK 挂实测腰角照样把手送到被指令的位姿；
真要因此够不着，残差会响亮地拒），末端位姿本身不能（丢了没有任何东西会发现）。

**不执行的那一段，IK 要挂实测值而不是指令值。** 这是 advisory 和「干脆放宽限位」
的全部区别：后者把一个机器人到不了的躯干姿态挂进链里，每一拍的手臂解都差同样
一点，没有一处报错，看起来像标定问题。

### Message

```python
{
    "schema": "motus.control/1",
    "seq": 1024,                    # per source, strictly increasing
    "stamp_ms": ...,                # when the command was generated
    "obs_stamp_ms": ...,            # which observation it was computed from
    "ttl_ms": 100,                  # expired commands are discarded
    "source": "mcp__actucore__vla", # who sent it — arbitration and audit
    "priority": 50,
    "mode": "joint_position",       # reconciled against the descriptor
    "dof": 14,
    "values": [...],                # engineering units, descriptor.joint_names order
    "gripper": 0.04,
    "chunk": {"index": 3, "size": 50},   # optional, for debugging
}
```

`obs_stamp_ms` is separate from `stamp_ms` on purpose. Remote inference routinely
produces a command that was generated just now from an 800 ms old picture; only
the observation timestamp catches that.

Carry `mode` and `dof` redundantly. They are the last line against an upstream
whose action space changed without the driver being told, and the correct answer
to that is to refuse — not to apply the first `dof` values and hope.

Payload is JSON in a `std_msgs/String`, like the perception cards: 30 Hz x 14 DOF
is about 30 KB/s, and the dashboard can render it directly.

### `twist` — the limit fields mean something different here

Every other mode carries **positions**; `twist` carries **velocities**. The
check chain was designed around the first, so two of the limit fields change
meaning under the second. Neither is flagged anywhere at runtime, and both look
correct until a robot accelerates like it was kicked.

| field | position modes | `twist` |
|---|---|---|
| `limits.lower` / `upper` | joint range | the **velocity envelope** — still right |
| `limits.max_delta_per_step` | velocity cap | the **acceleration** cap — also right, and the one you want |
| `limits.max_velocity` | velocity cap | jerk. Meaningless. **Do not declare it.** |

`max_velocity` is optional in `parse_descriptor`, so omitting it is the correct
action, not a workaround.

**Give an unactuatable axis `lower == upper == 0`, do not omit it.** A ground
base has no `vz`, `wx` or `wy`, and the vector is six wide regardless. Pinning
them to zero makes the sink **reject** a policy that puts anything there;
leaving them wide makes it silently ignore a model that believes it is
commanding vertical motion.

```python
"limits": {
    "lower": [-1.0, -1.0, 0.0,  0.0, 0.0, -2.0],
    "upper": [ 1.0,  1.0, 0.0,  0.0, 0.0,  2.0],
    "max_delta_per_step": [0.15, 0.15, 1e-6, 1e-6, 1e-6, 0.3],   # acceleration
    # no max_velocity — see above
},
```

First implementation: `unitree/r1/loco_servo.py`. Measured motion comes back on
`motus.odom/1`, whose axes are these axes — see that section.

### `limits.min_magnitude` — declare the speed below which your robot does nothing

A legged base has to assemble a whole gait cycle, so unlike a wheeled one it has
no creep regime: below some speed it does not move **at all**. R1 needs 0.4 m/s
and 1.0 rad/s. Under that the SDK accepts the command, returns 0, and the robot
stands still — 159 commands applied, no errors at any layer, no motion.

That is a property of the robot, so the robot declares it and the thing driving
it reads it. Per axis, in that axis' own units; `0` means no threshold.

```python
"min_magnitude": [0.4, 0.4, 0.0, 0.0, 0.0, 1.0],
```

Two consequences, and each one was a real robot standing still while every log
said it was moving:

**The step clamp must never be finer than the deadband on the same axis.** The
clamp is applied to the command, so a 0.30 rad/s acceleration cap against a
1.0 rad/s floor ramps 0.30 → 0.60 → 0.90 → 1.0 and the robot executes none of
the first three: three ticks of silence, then the turn arrives at full speed.
The ramp *down* crosses the floor on its first step, so it stops instantly —
slow to start, instant to stop, which is what a lurch is made of. Take
`max(accel, min_magnitude)` per axis (`loco_servo._step_limit`). Above the floor
the cap is then coarser than you wanted; below it, shaping acceleration is the
gait controller's job and never was yours.

**A consumer's own ceiling must sit above the floor.** Not your problem to
enforce, but say it in your card's docs: a policy whose `wz_max` is 0.8 against
a 1.0 floor has an entire output range the robot cannot execute, so every
command snaps to 0 or ±1.0 and the robot turns in a square wave. The navi card
reads `min_magnitude` at negotiation and raises its own ceilings off it
(`plugins/navi/policy.py::adopt_limits`) — that is the pattern to copy, not a
number to hard-code.

### `footprint` — declare the space your robot occupies

Optional, and additive to `motus.control/1`: an existing consumer that has never
heard of it keeps working. Declare it on any chassis a navigation policy will
drive.

```python
"footprint": {
    "shape": "box",
    "half_width": 0.179,        # m, half the widest lateral span
    "front": 0.095,             # m, ahead of the rotation centre
    "rear": 0.095,
    "height": 1.23,
    "source": "vendor-spec",    # "measured" | "estimate" | "vendor-spec"
    "arms": "at-rest",
},
```

Same reasoning as `min_magnitude`: it is a fact about the robot, and a policy
that hard-codes it is wrong on the next chassis. The consumer is a navigation
card, and what it is otherwise reduced to is testing a fixed angular slice of
its camera — which covers **a different width of the world at every distance**.
With a 63° lens the centre third spans `0.204 × distance` either side of the
axis, so at 0.8 m it is ±0.16 m: narrower than R1's shoulders, at precisely the
distance where stopping is decided. A doorframe 0.25 m off the axis is filed
under "left", the left third has never stopped forward motion, and the shoulder
goes into it while the depth map reports the way ahead as clear.

Three things about the fields:

* **`source` is part of the contract.** A datasheet box and one taken off this
  robot with a tape measure deserve different margins, and a consumer that
  cannot tell them apart will pick one number for both.
* **Declare the static envelope, not a clearance.** A swinging arm and a leg
  mid-stride both leave the torso box. The consumer adds its own margin —
  `navi` does, via `clearance_margin_m` — and a driver that pre-inflates its
  declaration makes that margin unknowable.
* **A missing declaration must be safe by default.** `navi` falls back to a
  half-width wider than any humanoid here and says so in `info().degraded`,
  because every failure mode of this number is one-sided: too wide costs some
  unnecessary slowing, too narrow puts a shoulder into a doorframe.

Not yet declared: where the **camera** is. A depth consumer also needs
`{fx, fy, cx, cy}` and the sensor's pose in the body frame to know which pixels
are floor and which are shoulder-height, and nothing in this project publishes
either — consumers currently work from a configured half-FOV, which is a guess
with no provenance at all. That is the same declaration in the perception layer,
and it is the next one to add.

### Bring the robot's DDS up with `common/dds_link`, not with one attempt

A bundle that calls `ChannelFactoryInitialize` once at start and carries on is
betting that its network interface already exists. On r1_sz that bet lost by
three seconds:

```
09:12:40  [bundle] namespace=ubuntu mcp_port=15702
09:12:43  [bundle] DDS init failed on 'eth10': channel factory init error.
          python3: eth10: does not match an available interface.
```

eth10 came up a moment later and stayed up all day. Nothing retried, so every
state topic on that robot — odometry, IMU, joints, battery, mainboard — was
empty from boot, while the cards publishing them sat on the canvas looking
healthy and declaring 10 Hz in `topic_out`. The robot itself was publishing
`rt/odommodestate` at 495 Hz the whole time.

```python
from common import dds_link

link = dds_link.install(network_iface)   # starts retrying in the background
link.wait(5.0)                           # optional: the common case looks synchronous
...
link.on_ready(self._subscribe)           # runs now if up, later if not
```

Three rules come out of that failure, and each is a separate way to stay broken:

* **Recompute the interface list on every attempt.** The fallback scan for an
  address on `192.168.123.x` ran at the same instant as the failure and found
  nothing either. The thing being waited for is an interface that does not
  exist yet, so a list captured at start cannot contain it.
* **Subscribe from `on_ready`, not from a constructor.** A constructor runs
  once, at the worst possible moment, and its `except` clause is where the
  failure goes to be forgotten.
* **Put the link state in `info()`.** `topic_out` promises a rate
  unconditionally; `info()` is the only place a reader can find out whether
  anything is coming out. And report *received* as well as *subscribed* —
  those are different facts and only the second one means the topic has data.

### Use `common/control.ControlSink` — do not write the checks yourself

There are fourteen bundles here. A safety chain copied fourteen times diverges
fourteen ways, and the copy that drifts is the one on the robot nobody is
watching.

```python
from common.control import ControlSink

self._sink = ControlSink(
    DESCRIPTOR,
    self._apply,                       # (values, gripper) -> None
    on_watchdog=self._decelerate_to_stop,
    on_abort=self._return_to_safe_pose,
)

# in your topic callback, after json.loads:
outcome = self._sink.submit(msg)

# from a timer, at least once per watchdog_ms:
self._sink.tick()

# if you have force-torque sensing:
self._sink.force_torque(readings)
```

The chain, in order:

| # | Check | Failure |
|---|-------|---------|
| 1 | schema / mode / dof against the descriptor | **REJECTED** |
| 2 | ttl expiry, stale observation, seq regression | **DROPPED** |
| 3 | priority arbitration between sources | **DROPPED** |
| 4 | `max_delta_per_step` | **CLAMPED** (applied) |
| 5 | position bounds, `max_velocity` | **REJECTED** |
| 6 | collision re-validation | *not implemented — see below* |
| 7 | per-axis force-torque threshold | **ABORTED** |
| 8 | continuity / committed window | *your controller's job — see below* |
| 9 | `watchdog_ms` with no valid command | hold |
| 10 | N consecutive watchdog periods | **stand down** (recoverable) |

**DROPPED vs REJECTED matters.** Dropped is the network being a network — stale,
out of order, outranked — and is counted, not reported. Rejected means somebody
wired something up wrong, and has to be visible. Do not collapse the two.

**Standing down is not aborting, and they need different things from you.**
Step 7 latches: the arm hit something, so the next command is precisely the one
that must not run, and only `reset()` clears it (`stats()["aborted"]`). Step 10
does not latch: nobody has spoken for N periods, so the *response* escalates —
`on_abort` fires, the arm stops hanging in the air — while the sink stays willing
to resume on the next command that passes every check above
(`stats()["stood_down"]`).

Collapsing the two produced a rule that contradicted itself: silence for four
watchdog periods resumed on its own, silence for five needed an operator to tear
the card down and rebuild it, though nothing about the robot differs across that
200 ms. It also broke a routine operation — restarting a policy card upstream
takes longer than five periods, so reconfiguring a policy left the driver
refusing every command until somebody noticed. Measured on Tianyi.

Resuming is safe for the same reason an ordinary hold is: step 2 means a
resuming command was computed from a recent observation, and step 4 still clamps
against the last applied values, which survive a stand-down. **Announce the
resumption on the activity stream** — for a stand-down exactly as for a hold,
since in both cases the robot starts moving again without anyone asking it to.

**Clamping vs rejecting matters too, and they go opposite ways.** An oversized
*step* is clamped: the point still goes where the policy meant, just more slowly,
and rejecting would break the motion into stutters over one noisy sample. A
*hard limit* violation rejects the entire command and never clamps to the bound —
clamping there invents a trajectory that is neither what the policy asked for nor
anything anyone validated, and every command after it is built on the false
premise that the robot reached the commanded point.

### Two gaps, stated rather than papered over

**Collision re-validation is not implemented.** MoveIt Pro checks every point of a
chunk against a planning scene with padding, and keeps watching the scene so an
object appearing mid-run stops the robot. We have no planning scene — only a URDF,
with no scene representation and no FK/collision runtime. Until one exists the
mitigations are procedural: run in simulation until you trust the policy, then
first real runs at reduced speed, with a person and a physical e-stop.

**Continuity is your controller's job.** Smoothing, densifying and blending chunks,
and sizing the committed window, happen where a trajectory is executed. One
consequence belongs here anyway: the committed window must be at least the p99
inference latency or the robot pauses between chunks, and the larger it is the
longer e-stop takes to actually stop. Choose it deliberately and report it in
`info()`.

### A pause is not a safe state

When commands stop arriving the robot holds — and it resumes the moment a valid
command lands, **without warning**. The `ttl_ms` check is the only thing keeping
it from resuming on a stale command, so set `ttl_ms` from measured latency: set it
generously and the protection is gone while still appearing to be there. Announce
the resume on the activity stream, and never treat a held robot as safe to
approach.

### Testing

`ControlSink` is ROS-free and takes an injected clock, so the whole chain tests on
a laptop with a fake `apply` and no robot:

```bash
python3 -m pytest tests/test_control_sink.py -q
```

Adding a driver-specific check? Add it there, not in your bundle.

---

## Robot Odometry (`motus.odom/1`)

The counterpart to `motus.control/1`: that one carries commands **to** a robot,
this one carries what the robot reports about **its own motion** back.

**It is the measurement dual of `twist`.** Same six axes, same order, same body
frame, same SI units. A consumer that commands
`values = [vx, vy, vz, wx, wy, wz]` and reads back
`twist = [vx, vy, vz, wx, wy, wz]` compares them with a subtraction and no
lookup table. That is the whole design, and the reason this is not a sixth
dialect of something that already exists five times.

Implementation: `common/odom.py`. Tests: `tests/test_odom_format.py`.

### Why not adopt a shape that already exists

The same physical quantity has five shapes in this repository, disagreeing on
the container type, on where the units live, and on whether an unmeasured axis
is reported at all:

| driver | linear | angular |
|---|---|---|
| Unitree R1/G1/Go2 | `velocity: [x,y,z]` | `yaw_speed`, a scalar |
| Unitree Go1 | `velocity_body_mps: {forward, lateral}` + `velocity_index_2_raw` | `yaw_speed_rad_s` |
| Booster K1 | `linear_velocity: [...]` | `angular_velocity: [...]` |
| EngineAI T800 | `linear_velocity: {x,y,z}` + `speed_m_s` + `valid` | `yaw_rate_rad_s` |
| EngineAI T800, same file | `linear_velocity: [...]` | — |

Two are worth pointing at. Go1's third component is called
`velocity_index_2_raw` — the name is an admission that nobody knows what it
means. And T800 grew a `valid` flag by itself, which means somebody already hit
the failure in rule 1 below and fixed it for one driver.

### The four rules

**1. An axis that was not measured is `null`, never `0.0`.** The one that
matters. A robot that does not report speed, reporting zero, is
indistinguishable from a robot standing still — which is exactly the failure
mode of everything that consumes this. A stuck-detector ("commanded 0.3 m/s,
measured nothing, so we have hit something") then fires on every robot that
simply has no odometry. Same reasoning as `descriptor.force_torque` being
required even as `null`: a missing protection must be visible, not assumed.

Per **axis**, not one `valid` flag for the sample: Go1-shaped partial knowledge
— two axes trustworthy, the third meaningless — is the normal case.

**2. Units live in `units`, not in field names.** `yaw_speed_rad_s` /
`speed_m_s` / `position_m` is not self-consistent within one vendor and cannot
survive a driver reporting degrees.

**3. `frame` is required.** Body versus world is the most dangerous ambiguity
here and not one of the five existing shapes states which it is. Guessing wrong
gives plausible numbers with the lateral sign flipped whenever the robot is not
facing along world x.

**4. Vendor fields go in `vendor`, untouched.** R1's `mode` / `gait_type` /
`body_height`, a wheeled base's battery and wheel RPM, a drone's barometric
altitude. Adopting this format costs a driver nothing it already reports, and a
consumer written against the core cannot be broken by a driver adding to
`vendor`. **Extensions grow into `vendor`; the core stays frozen.**

### Two halves, as in `motus.control/1`

| Half | Where | What |
|------|-------|------|
| **Interface** | your state card's `info()` | what this robot reports — read once, negotiated against |
| **Sample** | the `state/odom` topic | one reading, with `null` for anything not measured |

The `null`s in a sample say "not in this frame". A consumer also has to know *at
start* that a robot never reports speed at all, so it can degrade or refuse then
rather than discovering it at 10 Hz. Hence the declaration. agent-core's
existing `info()` path carries it with no changes.

```python
# info()
{"odom_interface": {"schema": "motus.odom/1", "frame": "body",
                    "provides": ["vx", "vy", "wz"],
                    "rate_hz": 10, "pose_drift": "unbounded"}}
```

```python
# one message on the topic
{
    "schema": "motus.odom/1",
    "stamp_ms": 1758537600123,
    "frame": "body",
    "units": {"linear": "m/s", "angular": "rad/s", "length": "m"},
    "twist": [0.31, 0.02, null, null, null, -0.42],   # AXES order; null = unmeasured
    "pose": null,
    "contact": {"grounded": true},
    "vendor": {"mode": 811, "gait_type": 1, "body_height": 0.78}
}
```

`pose` is separate and defaults to `null` because legged dead reckoning drifts
without bound. If you report one, state which: `pose_drift` is `none` |
`unbounded` (legged) | `bounded` (wheel odometry with a correction source, or
SLAM). That field is what decides whether a consumer may accumulate it.

Payload is JSON in a `std_msgs/String` on a `state/odom` port, like every other
state card.

### Implementing it in a new driver

1. Build the declaration once, at construction, and return it from `info()`:

   ```python
   from common.odom import build_interface, build_sample

   self._odom_interface = build_interface(
       provides=["vx", "vy", "wz"],      # ONLY the axes you actually measure
       rate_hz=10, pose_drift="unbounded",
   )
   ```

   `build_interface` validates what it builds, so a malformed declaration fails
   in your unit test rather than on a robot.

2. In your vendor callback, map into `AXES` order and **use `None` for every
   axis you do not measure**:

   ```python
   out = build_sample(
       stamp_ms=int(time.time() * 1000),
       twist=[vel[0], vel[1], None, None, None, msg.yaw_speed],
       vendor={"mode": msg.mode, "gait_type": msg.gait_type},
   )
   ```

3. Declare the port: `"topic_out": [{"topic": …, "format": "state/odom"}]`.

4. Read axes with `common.odom.axis(sample, "vx")`, never by indexing
   `sample["twist"]`. That helper is the one place keeping "not moving" and
   "does not know" apart, and indexing the list directly is one `or 0.0` away
   from losing it.

**Do not convert an existing state topic in place.** Publish `state/odom`
alongside whatever the driver already sends. The old shape has consumers you
cannot see from inside the bundle, and a 10 Hz duplicate is far cheaper than a
migration across fourteen of them.

### Downsampling: average, do not pick

Every vendor publishes state far faster than 10 Hz — R1 sends `rt/odommodestate`
at about 495 Hz — so a card publishing at 10 Hz discards roughly 49 readings out
of every 50. **Where the throttle sits decides whether they are averaged or
aliased.** Returning early from the callback and publishing the one reading that
survived is an unfiltered decimation: the noise averaging would have removed is
folded into the output instead, and the consumer this format was shaped around is
a stuck-detector reading a threshold crossing, which a noisy sample crosses when
it should not.

So read every message and publish the mean:

```python
from common.odom import mean_twist

def _on_state(self, msg):
    self._burst.append([vel[0], vel[1], None, None, None, msg.yaw_speed])
    if time.monotonic() - self._last < 0.1:
        return                                  # throttle the publish, not the read
    self._last = time.monotonic()
    burst, self._burst = self._burst, []
    sample = build_sample(stamp_ms=…, twist=mean_twist(burst), …)
```

`mean_twist` keeps the null rule through the average: an axis is averaged over the
readings that carry a number and stays `None` when none of them do. `sum() / len()`
is the obvious thing to write here and it turns an unmeasured axis into a measured
one. An empty window gives all-`None` — nothing arrived, which is not the same
fact as a robot standing still, so report how many readings the average came from
(R1 puts it in `vendor.samples`).

Do **not** average the vendor block. A gait enum has no mean; take it from
whichever message is current at publish time. The twist and the vendor fields then
describe slightly different instants, which is correct for what each one is: a
measurement to be filtered, and a label to be reported.

### `stamp_ms` has two jobs, and a robot's own clock may only do one

The checklist below asks for when the reading was *taken*, and a vendor message
normally carries that. But `stamp_ms` is also what `is_fresh` subtracts from the
**consumer's** clock. A robot whose clock is not synchronised with the host
satisfies the first job and destroys the second: every sample reads as minutes old
or as arriving from the future, and a consumer that trusts it stops the robot for
blindness it does not have. Neither symptom mentions a clock.

`common.odom.resolve_stamp_ms` picks between the two and records which it used:

```python
stamp_ms, provenance = resolve_stamp_ms(
    vendor_ms=_timespec_ms(msg.stamp),        # None if the SDK has no stamp
    received_ms=int(time.time() * 1000),
)
sample = build_sample(stamp_ms=stamp_ms, twist=…, vendor={**provenance, …})
```

A vendor stamp is quoted only when it is plausible as a wall clock *and* within
`MAX_CLOCK_SKEW_MS` of ours; otherwise the arrival time is published, which is at
least comparable. Either way `vendor.stamp_source` says which one arrived, so a
consumer never has to guess and the fallback is not a silent approximation. It is
deliberately **not** offset-corrected: subtracting a measured offset would let a
skewed clock be quoted, and on a clock that drifts rather than merely sits offset
that decays into a wrong answer which still looks principled. `stamp_skew_ms` is
reported so the skew can be measured first.

### Verifying `frame`, and why the robot cannot be its own witness

`frame` is the one field a consumer cannot sanity-check — `common/odom.py` takes it
as given and so does actucore's reader, which only checks that a sample *says*
body. Get it wrong and `vx`/`vy` swap at any non-zero heading: plausible numbers,
no error, nothing anywhere to notice. So it has to be measured, and measuring it on
R1 took four walks and produced two lessons worth more than the answer.

**Do not verify it against the robot's own position.** The obvious test differences
`position` against the integral of `velocity`, with and without the heading
rotation, and takes whichever fits. That presumes `position` is trustworthy. On
r1_sz it is not: over a **tape-measured 3 m straight walk** `position` reported
**0.81 m** — 73% short — with a path 2.3x its own net displacement, so it loses the
*shape* of the trajectory and not merely the origin. Three runs came back "neither
hypothesis explains the path", which was true and said nothing about frames.

**Separate scale from rotation and the frame question survives a broken reference.**
Fit one *complex* gain per hypothesis: the magnitude is how far the two sources
disagree about distance, the phase is the rotation the hypothesis still needs. Only
the phase answers the frame question, and it does not care about scale. On r1_sz the
body hypothesis needed **+0.2°** and the world one **−144°**, decided from the same
run whose magnitudes disagreed fourfold. Report the magnitude separately, as its own
finding rather than as a failure.

**Fix the ground truth before the robot moves.** Nothing inside the robot can say
which of two disagreeing sources is right. Three segments, each 30 seconds, each
isolating one quantity, all read out per-source by
`scripts/probe_r1_odom_frame.py` (read-only — it publishes nothing):

| segment | what you fix beforehand | what it settles |
|---|---|---|
| straight line, tape-measured | distance, and zero turn | the **scale** of `velocity` and of `position`, independently |
| one full turn in place | 360°, and zero displacement | the scale of `wz` — needs no measuring tool |
| closed loop back to a taped mark | net displacement is zero | the frame, and whether the integration closes |

Run the straight line first; it alone tells you which source is wrong and by how
much. On r1_sz: `velocity` 3.62 m against a measured 3 m (**20% high**, straightness
1.03), `position` 0.81 m (**73% short**, straightness 2.30).

**Record the measurement where the next person will find it**, not only in a commit
message — R1 keeps it in `health()` under `odom_measured` and `position_unusable`.
And do not correct a 20% overread with a scalar: one measurement against an
approximate distance is not a calibration, and a magic number makes a wrong figure
look authoritative. State it, and check it is inside the margin that matters — 20%
cannot push a stalled robot over navi's "below 20% of commanded" stuck threshold.

### Checklist for a new driver

- [ ] `provides` lists only axes genuinely measured — not the ones the SDK has a
      field for
- [ ] every unmeasured axis is `None` in `twist`, and no `or 0.0` anywhere near it
- [ ] `frame` is right; if it is `world`, say so rather than relabelling it body.
      **Check it against a trajectory you know in advance** — see below
- [ ] `stamp_ms` comes from `resolve_stamp_ms`, and its provenance is in `vendor`
- [ ] the 10 Hz publish averages the window with `mean_twist` rather than
      publishing one reading out of every N
- [ ] vendor-specific fields are under `vendor`, not at the top level
- [ ] `pose_drift` is honest; `unbounded` unless there is a correction source
- [ ] a unit test calls `parse_interface()` on your declaration

## Camera Parameters (`camera_info`)

The third of the three declarations a driver makes about itself.
`motus.control/1` says what a robot can be *told*; `motus.odom/1` says what it
*reports*; this one says what a camera *sees* — and unlike the other two it does
not stop at the driver. It travels along the canvas connections, and each
processor it passes through rewrites the parts its own processing changed.

Implementation: `common/camera_info.py`. Tests: `tests/test_camera_info.py`.
Reference declaration: `unitree/r1/camera_specs.py`.

### Why a camera has to say this itself

Nothing in an image carries geometry. A depth map is 640x480 numbers, each a
distance, and nothing in it says how wide the lens was. But the decisions made
from it are metric — "is there room for my 0.36 m shoulders" — so somewhere a
pixel column has to become a lateral offset in metres, and that conversion needs
the field of view.

Until this existed, that number lived in the *navigation policy's* config file,
typed in by hand. On r1_sz it read 0.55 rad (~63 deg full, an ordinary lens)
against a lens that measures 0.888 (~102 deg, ultra-wide). The avoidance corridor
is metric — half-width 0.329 m — so every frame it converted that width back into
a column range, and with the field of view understated the slice came out too
wide: at 1 m it sampled 84% of the picture's half-width, which really spans
±0.93 m. **The corridor was 1.86 m wide, wider than any door.** Every doorframe
counted as dead ahead, the clearance reading was the distance to the door plane
rather than through the opening, and the robot turned away 0.6 m short of a gap
it fitted through.

The value had been measured on that robot the day before. It went into a report
and not into a config file, and nothing anywhere noticed.

**Note how it failed.** A wrong deadband makes a robot stutter and you see it in
the first second. A wrong field of view makes a robot refuse doorways *while the
depth map reports clear ahead* — the only evidence is the behaviour. That is why
this is a declaration and not a setting: the camera knows, and had no way to say
so.

Same rule as `limits.min_magnitude` and `footprint`, which the chassis declares
and `navi` adopts at start. Camera parameters were the last exception.

### The four rules

**1. A quantity that is not known is `null`, never a guess.** A plausible number
makes the consumer believe it knows. The consumer must be able to tell "nobody
told me" from "I was told", so it can fall back conservatively *and report that
it did*. R1 declares `half_fov_rad` for `camera_main` and `null` for the other
three, because only one has been measured.

**2. Meaning and units are fixed by this document.** `half_fov_rad` is the
**horizontal half** field of view, in radians. The vertical angle is a separate
field and **cannot be derived from the aspect ratio** — a processor that resizes
1280x720 into 640x480 stretches the picture, so the pixels are no longer square
and the two angles are no longer related by the frame's shape.

**3. `source` is required.** `measured` / `vendor-spec` / `derived-from-K` /
`inherited` / `manual` / `unknown`. Whether a number can be trusted is mostly a
question of where it came from, and `unknown` is a legitimate answer that has to
be *stated* rather than left out.

**4. `id` survives the chain, `width`/`height` are rewritten at each stop,
`pipeline` records who touched it.** Downstream cards look up their own tables by
`id` (a depth calibration is a property of camera × model, so it lives with the
model and is *keyed* by the camera). `width`/`height` describe the image **this
port publishes**, not the original. `pipeline` is what makes a wrong number
traceable to the stage that changed it.

### Shape: ROS first, convenience second

The core is `sensor_msgs/CameraInfo`'s — `width`, `height`, `distortion_model`,
`D`, `K` — so anything with a real calibration fills it mechanically. A fisheye's
distortion is a thing only `D` can express: `tan(theta)` overstates the lateral
offset towards the edges of a wide lens, so the pinhole model consumers use today
is an approximation that holds near the centre.

`half_fov_rad` sits alongside as a derived convenience, because an angle is what
consumers actually need and because a tape measure produces one directly while
producing no `K` at all. **When both are present `K` wins** (`resolve_half_fov()`):
a calibration matrix is solved from many observations, the angle beside it is
usually a tape measure and some trigonometry.

```json
"camera_info": [
  {"schema": "motus.camera/1",
   "topic": "/ubuntu/camera/main",
   "format": "image/jpeg",
   "id": "unitree/r1/camera_main",
   "width": 1280, "height": 720,
   "distortion_model": "unknown",
   "D": null, "K": null,
   "half_fov_rad": 0.888,
   "half_fov_v_rad": null,
   "source": "measured",
   "measured_on": "r1_sz, 2026-09-23",
   "pipeline": ["unitree/r1/camera_main"],
   "vendor": {"note": "约 102 度全视场（超广角）"}}
]
```

It is a **list**, one entry per output port, each naming its own `topic`. A card
may publish several ports and a consumer may have several inputs, so neither side
can join on list position — and joining on the upstream *card's name* would undo
the reason cards dispatch inputs by what they carry.

### How it reaches a consumer

Returned from the camera tool's `info()`. agent-core already calls `info()` on
every card right after it starts (`api/config.py` `_resolve_and_register`), and at
project start a card's sources are always started first — so the declaration is
already in hand with no extra round trip. agent-core collects the declarations on
a card's **inbound connections**, keys them by topic, and passes them as
`camera_info` on that card's `start`.

This mirrors `control_interface`, which travels the other way (a command producer
is handed its *consumer's* action space). Two consequences worth knowing:

- **An upstream that declares nothing is not an error.** Most cards have never
  heard of this format. The consumer degrades and says so.
- **Starting a single card from the canvas does not carry it**, same as
  `control_interface` today. The consumer must tolerate its absence.

### Implementing this for another driver

1. For each camera output port, call `common.camera_info.build()` once and return
   the list under `camera_info` from that tool's `info()`. It is a declaration,
   not runtime state, so answer it whether or not the camera is streaming.
2. Put the numbers in a module that does **not** import `rclpy`
   (`unitree/r1/camera_specs.py` is the model), so a unit test can assert them on
   a laptop. These are exactly the numbers that must not reach a robot unchecked.
3. `id` is `"<vendor>/<model>/<port>"`, stable across reboots and across every
   unit of that model.
4. To measure a lens: `phanthymotus/actucore/tools/measure_fov.py` — a plane of
   known width at a tape-measured distance. **Re-measure after any lens change.**

Checklist before you call it done:

- [ ] every camera tool's `info()` carries a `camera_info` entry for its topic
- [ ] unmeasured lenses report `half_fov_rad: null` with `source: "unknown"`
- [ ] `half_fov_rad` is the **half** angle (`build()` refuses a full one, but
      only when it exceeds 90 deg — below that nothing can catch the mistake)
- [ ] `width`/`height` describe what that port publishes
- [ ] the spec module is listed in the bundle's Dockerfile `COPY` lines
      (`tests/test_dockerfile_copies.py` checks this)
- [ ] a unit test calls `parse()` on every declaration the driver can emit
