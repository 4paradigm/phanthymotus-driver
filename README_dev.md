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

Driver ports are allocated in the **15700–15799** range:

| Driver | Port |
|--------|------|
| Unitree G1 | 15701 |
| Phanthy Remote Control | 15710 |

New drivers should choose an unoccupied port. The WebSocket port is typically the MCP port + 1.

---

## Data Format & Dashboard Rendering

The Agent Core Web Dashboard automatically selects a renderer based on the `format` field declared in `topic_out`. Understanding this mapping is essential when implementing sensor plugins.

### Format → Renderer Mapping

| Format | Renderer | canRender logic |
|--------|----------|----------------|
| `audio/*` (e.g. `audio/pcm-16k`) | Audio waveform | `hint.startsWith('audio/')` |
| `video/*` (e.g. `video/mjpeg`) | Video stream | `hint.startsWith('video/')` |
| `image/jpeg` | Camera image | `hint === 'image/jpeg'` |
| `image/depth-z16` | Depth colormap | `hint === 'image/depth-z16'` |
| `image` | Generic image | `hint === 'image'` |
| `data/json` | Text / KV panel | `hint === 'data/json'` |
| `text/*` | Text display | `hint.startsWith('text/')` |
| `sensor/skeleton` | 3D Skeleton (URDF) | `hint === 'sensor/skeleton'` |
| `sensor/lidar*` | Lidar scan | `hint.startsWith('sensor/lidar')` |
| `sensor/pointcloud` | 3D Point cloud | `hint === 'sensor/pointcloud'` |
| `sensor/mapping` | 2D Occupancy map | `hint === 'sensor/mapping'` |
| `sensor/htmsg` | HT structured message | `hint === 'sensor/htmsg'` |
| (no hint) | Activity stream | Fallback when no format specified |

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
  image: __IMAGE__               # Placeholder, replaced by Agent Core at deploy time
  privileged: true               # Required: access to /dev and hardware
  volumes:
    - /dev:/dev                  # Required: device access for cameras, sensors, etc.
  environment:
    - ROS_DOMAIN_ID=42
    - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    - FASTDDS_BUILTIN_TRANSPORTS=DEFAULT
    - PYTHONUNBUFFERED=1
  logging:
    driver: local
    options:
      max-size: "10m"
      max-file: "3"
  restart: unless-stopped
```

**Notes:**

- `privileged: true` and `/dev:/dev` are mandatory for any driver that accesses hardware (cameras, USB devices, GPIO)
- `network_mode`, `ipc`, `pid` are injected by Agent Core during deployment — do not specify them in service.yml
- The `__IMAGE__` placeholder is automatically replaced with the actual image reference
- Service name should follow the pattern `{provider}-{model}` (e.g. `unitree-g1`, `phanthy-remote-control`)
